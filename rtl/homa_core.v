// Homa protocol core (draft-ousterhout-tsvwg-homa-00). Owns the RPC slot table and runs one job per
// cycle, in priority order: DATA admission/commit, received control packets, transmit selection,
// delivery completion, application load/finalize, timer scan (one slot per job).
// Each RPC uses one slot; a slot has one outgoing and one incoming message buffer.
module homa_core #(
    parameter SEG_BYTES     = 1024,   // DATA payload per packet; power of 2, 8..32768
    parameter MAX_MSG       = 16384,  // largest message; power of 2, multiple of SEG_BYTES
    parameter UNSCHED_MAX   = 4096,   // longer outgoing messages are scheduled (START_MSG + GRANTs)
    parameter RTT_BYTES     = 4096,   // granted-but-not-received bytes kept per active message
    parameter NRPC          = 8,      // RPC slots; power of 2, >= 2
    parameter SRV_RESERVE   = 2,      // slots our own requests may not use, kept for incoming ones
    parameter OVERCOMMIT    = 4,      // incoming messages granted at once
    parameter NPEER         = 4,      // peers whose CUTOFFS are remembered; power of 2
    parameter TICK          = 15625,  // cycles per timer tick
    parameter RESEND_TICKS  = 50,     // ticks without progress before RESEND / START_MSG retry
    parameter ABORT_RESENDS = 5,      // unanswered retries before a client RPC is aborted
    parameter NEEDACK_TICKS = 50,     // ticks after a response is sent before NEED_ACK
    parameter ACK_GIVEUP    = 20,     // unanswered NEED_ACKs before the server drops the RPC
    parameter EVQ           = 4,      // received control packet queue; power of 2
    parameter SLQ           = 8,      // queue for replies that have no RPC slot; power of 2
    parameter CUT_COOL      = 1024    // min cycles between CUTOFFS packets
) (
    input  wire                 clk,
    input  wire                 rst,

    input  wire [255:0]         cfg_cutoffs,    // our unscheduled cutoffs, advertised in CUTOFFS
    input  wire [15:0]          cfg_cver,
    input  wire [3:0]           cfg_sched_lvls, // priority levels 0..n-1 used for grants (1..8)
    input  wire [63:0]          cfg_id_base,    // first client RPC id after reset; vary it per reset

    // ingress
    input  wire                 adm_req,
    output wire                 adm_ok,
    output wire [$clog2(NRPC)-1:0] adm_slot,
    input  wire [31:0]          d_saddr,
    input  wire [15:0]          d_sport,
    input  wire [15:0]          d_dport,
    input  wire [63:0]          d_id,
    input  wire [31:0]          d_len,
    input  wire [31:0]          d_off,
    input  wire [63:0]          d_ackid,
    input  wire [15:0]          d_cver,
    input  wire [15:0]          d_plen,
    input  wire                 cmt_req,
    input  wire                 cmt_ok,
    input  wire [$clog2(NRPC)-1:0] cmt_slot,
    input  wire                 ev_push,
    input  wire [639:0]         ev_hdr,
    input  wire [31:0]          ev_saddr,
    input  wire [15:0]          ev_len,
    output wire                 ev_full,

    // egress descriptors
    output reg                  tx_valid,
    input  wire                 tx_ready,
    output reg  [7:0]           tx_type,
    output reg  [31:0]          tx_daddr,
    output reg  [15:0]          tx_sport,
    output reg  [15:0]          tx_dport,
    output reg  [63:0]          tx_id,
    output reg  [2:0]           tx_prio,        // packet priority (switch queue)
    output reg  [31:0]          tx_off,
    output reg  [31:0]          tx_len,         // DATA/START_MSG: message length; RESEND: length
    output reg  [15:0]          tx_plen,
    output reg  [7:0]           tx_pf,          // GRANT/RESEND: priority field; DATA: retransmit flag
    output reg  [15:0]          tx_cver,
    output reg  [$clog2(NRPC)-1:0] tx_slot,
    input  wire [NRPC-1:0]      eg_lock,

    // application send: setup then finalize
    input  wire                 ld_req,
    input  wire                 ld_resp,        // 1: response to a delivered request
    input  wire [31:0]          ld_addr,
    input  wire [15:0]          ld_rport,
    input  wire [15:0]          ld_lport,
    input  wire [$clog2(NRPC)+7:0] ld_handle,  // {gen, slot} of the request being answered
    input  wire [31:0]          ld_mlen,
    output reg                  ld_ack,
    output reg                  ld_ok,
    output reg  [$clog2(NRPC)+7:0] ld_hout,
    output reg  [63:0]          ld_id,
    input  wire                 fin_req,
    input  wire [$clog2(NRPC)-1:0] fin_slot,
    output reg                  fin_ack,

    // delivery
    output reg                  dq_push,
    output reg  [$clog2(NRPC)+7:0] dq_handle,
    output reg                  dq_resp,        // 1: response to our request; 0: request to us
    output reg  [31:0]          dq_addr,
    output reg  [15:0]          dq_rport,
    output reg  [15:0]          dq_lport,
    output reg  [63:0]          dq_id,
    output reg  [31:0]          dq_len,
    input  wire                 dd_req,
    input  wire [$clog2(NRPC)-1:0] dd_slot,
    output reg                  dd_ack,

    // client RPCs that timed out
    output reg                  ab_valid,
    output reg  [63:0]          ab_id,
    input  wire                 ab_ready
);

`include "homa_defs.vh"

localparam SL     = $clog2(NRPC);
localparam SEG_LG = $clog2(SEG_BYTES);
localparam MSG_LG = $clog2(MAX_MSG);
localparam NSEG   = MAX_MSG / SEG_BYTES;
localparam LW     = MSG_LG + 1;
localparam PLG    = (NPEER > 1) ? $clog2(NPEER) : 1;
localparam EQL    = $clog2(EVQ);
localparam SQL    = $clog2(SLQ);
localparam TW     = $clog2(TICK + 1);
localparam CW     = $clog2(CUT_COOL + 1);
localparam [LW-1:0] SEGL = SEG_BYTES;

localparam [2:0] PH_CLOAD = 3'd0, PH_CACT = 3'd1, PH_SRECV = 3'd2, PH_SSERVE = 3'd3,
                 PH_SLOAD = 3'd4, PH_SSEND = 3'd5;

generate
    if (SEG_BYTES < 8 || SEG_BYTES > 32768 || (SEG_BYTES & (SEG_BYTES - 1)) != 0 || MAX_MSG < SEG_BYTES ||
        (MAX_MSG & (MAX_MSG - 1)) != 0 || NRPC < 2 || (NRPC & (NRPC - 1)) != 0 ||
        UNSCHED_MAX < 1 || UNSCHED_MAX > MAX_MSG || RTT_BYTES < SEG_BYTES || SRV_RESERVE >= NRPC ||
        RTT_BYTES % SEG_BYTES != 0 || RTT_BYTES > MAX_MSG || OVERCOMMIT < 1 || OVERCOMMIT > NRPC ||
        (NPEER & (NPEER - 1)) != 0 || TICK < NRPC + 2 || RESEND_TICKS < 1 || RESEND_TICKS > 250 ||
        NEEDACK_TICKS < 1 || NEEDACK_TICKS > 250 || ABORT_RESENDS < 1 || ABORT_RESENDS > 250 ||
        ACK_GIVEUP > 250 || (EVQ & (EVQ - 1)) != 0 || EVQ < 2 || (SLQ & (SLQ - 1)) != 0 || SLQ < 2 ||
        // draft §7: the server must wait longer than a client takes to give up
        ACK_GIVEUP * NEEDACK_TICKS <= (ABORT_RESENDS + 1) * RESEND_TICKS) begin : bad_parameter
        homa_core_invalid_parameter u_fail ();
    end
endgenerate

// slot table
reg  [NRPC-1:0] v, dead, role, dlv, load;
reg  [NRPC-1:0] tx_on, tx_sch, rx_on, rx_sch, rx_done;
reg  [NRPC-1:0] p_regrant, p_resend, p_busy, p_start, p_needack;
reg  [2:0]      ph      [0:NRPC-1];
reg  [31:0]     peer    [0:NRPC-1];
reg  [15:0]     pport   [0:NRPC-1];
reg  [15:0]     lport   [0:NRPC-1];
reg  [63:0]     rid     [0:NRPC-1];   // even client-assigned id
reg  [7:0]      gen     [0:NRPC-1];
reg  [LW-1:0]   t_len   [0:NRPC-1];
reg  [LW-1:0]   t_gnt   [0:NRPC-1];
reg  [LW-1:0]   t_sent  [0:NRPC-1];
reg  [LW-1:0]   rs_s    [0:NRPC-1];   // pending retransmit range [rs_s, rs_e)
reg  [LW-1:0]   rs_e    [0:NRPC-1];
reg  [2:0]      t_prio  [0:NRPC-1];   // from GRANT
reg  [2:0]      t_uprio [0:NRPC-1];   // from peer cutoffs
reg  [2:0]      rs_prio [0:NRPC-1];
reg  [15:0]     t_cver  [0:NRPC-1];
reg  [LW-1:0]   r_len   [0:NRPC-1];
reg  [LW-1:0]   r_rcvd  [0:NRPC-1];
reg  [LW-1:0]   r_gnt   [0:NRPC-1];
reg  [NSEG-1:0] bm      [0:NRPC-1];
reg  [7:0]      silent  [0:NRPC-1];
reg  [7:0]      nres    [0:NRPC-1];

reg  [63:0]     next_id;
reg             ak_v, ak_new;         // DATA packet admitted and still arriving
reg  [SL-1:0]   ak_slot;

// peer cutoffs
reg  [NPEER-1:0] pv;
reg  [31:0]      paddr [0:NPEER-1];
reg  [255:0]     pcut  [0:NPEER-1];
reg  [15:0]      pver  [0:NPEER-1];
reg  [PLG-1:0]   prr;

integer i, k, ci, gi, gj, ri, ti, xi, pi, fi;

// received control packet queue
reg  [639:0]  eq_hdr  [0:EVQ-1];
reg  [31:0]   eq_addr [0:EVQ-1];
reg  [15:0]   eq_len  [0:EVQ-1];
reg  [EQL:0]  eq_wp, eq_rp;
wire [EQL:0]  eq_cnt = eq_wp - eq_rp;
wire          eq_ne  = (eq_cnt != 0);
assign ev_full = (eq_cnt >= EVQ);

wire [639:0]  eh      = eq_hdr[eq_rp[EQL-1:0]];
wire [31:0]   eh_addr = eq_addr[eq_rp[EQL-1:0]];
wire [15:0]   eh_len  = eq_len[eq_rp[EQL-1:0]];
wire [7:0]    eh_type = hb8(eh, 11);
wire [15:0]   eh_sport = hb16(eh, 0);
wire [15:0]   eh_dport = hb16(eh, 2);
wire [63:0]   eh_id   = hb64(eh, 20);
wire [31:0]   eh_f28  = hb32(eh, 28);
wire [31:0]   eh_f32  = hb32(eh, 32);
wire [2:0]    eh_p32  = hb8(eh, 32);
wire [2:0]    eh_p36  = hb8(eh, 36);
reg  [2:0]    ai;                     // ACK: 0 = common header, 1..5 = extra acks

wire [15:0] nacks = hb16(eh, 28);
wire [2:0]  ack_room = (eh_len >= 80) ? 3'd5 : (eh_len >= 70) ? 3'd4 : (eh_len >= 60) ? 3'd3 :
                       (eh_len >= 50) ? 3'd2 : (eh_len >= 40) ? 3'd1 : 3'd0;
wire [2:0]  ack_n    = (nacks < ack_room) ? nacks[2:0] : ack_room;
reg  [63:0] ack_x;
always @* begin
    case (ai)
    3'd1:    ack_x = hb64(eh, 30);
    3'd2:    ack_x = hb64(eh, 40);
    3'd3:    ack_x = hb64(eh, 50);
    3'd4:    ack_x = hb64(eh, 60);
    default: ack_x = hb64(eh, 70);
    endcase
end

// replies with no slot (RPC_UNKNOWN, ACK, CUTOFFS)
reg  [7:0]    sq_type [0:SLQ-1];
reg  [31:0]   sq_addr [0:SLQ-1];
reg  [15:0]   sq_sp   [0:SLQ-1];
reg  [15:0]   sq_dp   [0:SLQ-1];
reg  [63:0]   sq_id   [0:SLQ-1];
reg  [SQL:0]  sq_wp, sq_rp;
wire [SQL:0]  sq_cnt  = sq_wp - sq_rp;
wire          sq_ne   = (sq_cnt != 0);
wire          sq_full = (sq_cnt >= SLQ);
wire [SQL-1:0] sq_h   = sq_rp[SQL-1:0];

// RPC lookup (one port)
// a packet with S=0 comes from a client, so we are the server
reg  [31:0] k_addr;
reg  [63:0] k_id;
reg         k_srv;
always @* begin
    if (adm_req) begin
        k_addr = d_saddr; k_id = {d_id[63:1], 1'b0}; k_srv = !d_id[0];
    end else begin
        k_addr = eh_addr;
        k_id   = (eh_type == T_ACK && ai != 0) ? {ack_x[63:1], 1'b0} : {eh_id[63:1], 1'b0};
        k_srv  = (eh_type == T_ACK) ? 1'b1 : !eh_id[0];
    end
end

reg          hit;
reg [SL-1:0] hs;
reg          free_any;
reg [SL-1:0] fs;
reg [SL:0]   nfree;
always @* begin
    hit = 1'b0; hs = 0; free_any = 1'b0; fs = 0; nfree = 0;
    for (ci = NRPC - 1; ci >= 0; ci = ci - 1) begin
        if (v[ci] && !dead[ci] && role[ci] == k_srv && peer[ci] == k_addr && rid[ci] == k_id) begin
            hit = 1'b1; hs = ci[SL-1:0];
        end
        if (!v[ci] && !(ak_v && ak_new && ak_slot == ci)) begin
            free_any = 1'b1; fs = ci[SL-1:0]; nfree = nfree + 1'b1;
        end
    end
end

// DATA admission (beat 6)
wire [31:0] a_seg = d_off >> SEG_LG;
wire        a_ingnt = ({1'b0, d_off} + d_plen <= r_gnt[hs]);   // segment inside what we granted
reg         adm_ok_r;
always @* begin
    if (!d_id[0])   // request data: we are the server
        adm_ok_r = hit ? (ph[hs] == PH_SRECV && r_len[hs] == d_len && pport[hs] == d_sport &&
                          lport[hs] == d_dport && !bm[hs][a_seg] && a_ingnt)
                       : free_any;
    else            // response data: we are the client
        adm_ok_r = hit && ph[hs] == PH_CACT && pport[hs] == d_sport && lport[hs] == d_dport &&
                   (rx_on[hs] ? (r_len[hs] == d_len && !rx_done[hs] && !bm[hs][a_seg] && a_ingnt) : 1'b1);
end
assign adm_ok   = adm_ok_r;
assign adm_slot = hit ? hs : fs;

// commit (on tlast; same cycle as admission when tlast is beat 6)
wire          c_new  = (adm_req && cmt_req) ? !hit : ak_new;
wire [SL-1:0] cs     = cmt_slot;
wire [NSEG-1:0] c_bit = 1'b1 << (d_off >> SEG_LG);
wire          c_first = c_new || !rx_on[cs];
wire [NSEG-1:0] c_bm  = (c_first ? {NSEG{1'b0}} : bm[cs]) | c_bit;
wire [LW-1:0] c_rcvd  = (c_first ? {LW{1'b0}} : r_rcvd[cs]) + d_plen;
wire [NSEG-1:0] c_need = ~({NSEG{1'b1}} << ((d_len + SEG_BYTES - 1) >> SEG_LG));
wire          c_cmpl  = &(c_bm | ~c_need);
wire          c_go    = cmt_req && cmt_ok && (c_new || !dead[cs]);

// grant scheduling (draft §4, §5): registered; one message per peer; SRPT by bytes still to arrive
reg  [NRPC-1:0] elig, cand, act_r;
reg  [LW-1:0]   rem [0:NRPC-1];
reg  [2:0]      gprio_r [0:NRPC-1];
reg  [SL:0]     rank [0:NRPC-1];
reg  [SL:0]     ncand, mact;
reg  [3:0]      slv;
reg             bt;
always @* begin
    for (gi = 0; gi < NRPC; gi = gi + 1) begin
        elig[gi] = v[gi] && !dead[gi] && rx_on[gi] && rx_sch[gi] && !rx_done[gi] && (r_gnt[gi] < r_len[gi]);
        rem[gi]  = r_len[gi] - r_rcvd[gi];
    end
    for (gi = 0; gi < NRPC; gi = gi + 1) begin
        cand[gi] = elig[gi];
        for (gj = 0; gj < NRPC; gj = gj + 1) begin
            bt = elig[gj] && ((rem[gj] < rem[gi]) || (rem[gj] == rem[gi] && gj < gi));
            if (bt && peer[gj] == peer[gi]) cand[gi] = 1'b0;
        end
    end
    ncand = 0;
    for (gi = 0; gi < NRPC; gi = gi + 1) begin
        rank[gi] = 0;
        for (gj = 0; gj < NRPC; gj = gj + 1)
            if (cand[gj] && ((rem[gj] < rem[gi]) || (rem[gj] == rem[gi] && gj < gi))) rank[gi] = rank[gi] + 1'b1;
        ncand = ncand + cand[gi];
    end
end
always @* begin
    slv  = (cfg_sched_lvls == 0) ? 4'd1 : (cfg_sched_lvls > 8) ? 4'd8 : cfg_sched_lvls;
    mact = (ncand > OVERCOMMIT) ? OVERCOMMIT : ncand;
end
always @(posedge clk) begin
    for (ri = 0; ri < NRPC; ri = ri + 1) begin
        act_r[ri] <= cand[ri] && (rank[ri] < OVERCOMMIT);
        // draft §5.4: fewer messages than levels use the lowest levels; extras share level 0
        if (!cand[ri] || rank[ri] >= OVERCOMMIT)
            gprio_r[ri] <= 3'd0;
        else if (mact <= slv)
            gprio_r[ri] <= mact - 1 - rank[ri];
        else
            gprio_r[ri] <= (rank[ri] < slv - 1) ? slv - 1 - rank[ri] : 3'd0;
    end
end

// transmit selection
reg  [LW:0]     want_w;
reg  [LW-1:0]   want [0:NRPC-1];
reg  [NRPC-1:0] gneed, cneed, dneed;
reg  [LW-1:0]   seg_nw;
reg  [LW-1:0]   dkey, dbest;
reg             c_any, d_any;
reg  [SL-1:0]   c_ix, d_ix, rr;
reg  [SL-1:0]   q;
reg             alt;
always @* begin
    for (ti = 0; ti < NRPC; ti = ti + 1) begin
        want_w  = r_rcvd[ti] + RTT_BYTES;
        want[ti] = (want_w > r_len[ti]) ? r_len[ti] : want_w[LW-1:0];
        gneed[ti] = v[ti] && !dead[ti] && act_r[ti] && rx_on[ti] && rx_sch[ti] && !rx_done[ti] &&
                   (want[ti] > r_gnt[ti]);
        cneed[ti] = v[ti] && !dead[ti] &&
                   (gneed[ti] || p_regrant[ti] || p_resend[ti] || p_busy[ti] || p_start[ti] || p_needack[ti]);
        seg_nw   = (t_len[ti] - t_sent[ti] > SEGL) ? SEGL : t_len[ti] - t_sent[ti];
        dneed[ti] = v[ti] && !dead[ti] && tx_on[ti] &&
                   ((rs_s[ti] < rs_e[ti]) || (t_sent[ti] < t_len[ti] && t_sent[ti] + seg_nw <= t_gnt[ti]));
    end
    c_any = 1'b0; c_ix = 0;
    for (ti = NRPC - 1; ti >= 0; ti = ti - 1) begin
        q = rr + ti[SL-1:0];
        if (cneed[q]) begin c_any = 1'b1; c_ix = q; end
    end
    // sender SRPT (draft §8): fewest bytes left to send; retransmissions first
    d_any = 1'b0; d_ix = 0; dbest = {LW{1'b1}};
    for (ti = 0; ti < NRPC; ti = ti + 1) begin
        dkey = t_len[ti] - t_sent[ti];
        if (dneed[ti] && (!d_any || dkey < dbest)) begin d_any = 1'b1; d_ix = ti[SL-1:0]; dbest = dkey; end
    end
end

// resend range for the chosen control slot: first run of missing segments
reg  [LW-1:0]   x_exp, x_end;
reg  [31:0]     x_nexp;
reg  [NSEG-1:0] x_miss;
reg             x_found;
reg  [31:0]     x_f, x_g, x_off, x_len;
always @* begin
    x_exp  = rx_sch[c_ix] ? r_gnt[c_ix] : r_len[c_ix];
    x_nexp = (x_exp + SEG_BYTES - 1) >> SEG_LG;
    x_miss = ~({NSEG{1'b1}} << x_nexp) & ~bm[c_ix];
    x_found = 1'b0; x_f = 0;
    for (xi = NSEG - 1; xi >= 0; xi = xi - 1)
        if (x_miss[xi]) begin x_found = 1'b1; x_f = xi; end
    x_g = x_nexp;
    for (xi = NSEG - 1; xi >= 0; xi = xi - 1)
        if (bm[c_ix][xi] && xi > x_f && xi < x_nexp) x_g = xi;
    x_end = ((x_g << SEG_LG) > x_exp) ? x_exp : (x_g << SEG_LG);
    x_off = x_f << SEG_LG;
    x_len = x_end - x_off;
    if (!rx_on[c_ix]) begin x_found = 1'b1; x_off = 0; x_len = 32'hFFFF_FFFF; end
end

localparam [3:0] A_NONE = 4'd0, A_SQ = 4'd1, A_GRANT = 4'd2, A_RESEND = 4'd3, A_BUSY = 4'd4,
                 A_START = 4'd5, A_NEEDACK = 4'd6, A_DATA = 4'd7, A_CLR_REGR = 4'd8,
                 A_CLR_RES = 4'd9, A_CLR_START = 4'd10;

wire hi_busy = adm_req || cmt_req;
wire j_ev    = !hi_busy && eq_ne && !ak_v;
wire t_any   = c_any || sq_ne || d_any;
wire j_tx    = !hi_busy && !j_ev && tx_ready && t_any;
// a requester sees our registered ack one cycle late, so skip the cycle the ack is out
wire j_dd    = !hi_busy && !j_ev && !j_tx && dd_req && !dd_ack;
wire j_ld    = !hi_busy && !j_ev && !j_tx && !j_dd && (ld_req || fin_req) && !ld_ack && !fin_ack;
reg  tm_pend;
reg  [SL-1:0] tm_ix;
wire j_tm    = !hi_busy && !j_ev && !j_tx && !j_dd && !j_ld && tm_pend;

reg  [3:0]    t_act;
reg  [SL-1:0] t_s;
reg  [LW-1:0] t_poff, t_pl;
reg           use_sq;
always @* begin
    t_act = A_NONE; t_s = c_ix; use_sq = 1'b0;
    t_poff = 0; t_pl = 0;
    tx_valid = 1'b0;
    tx_type = T_BUSY; tx_daddr = peer[c_ix]; tx_sport = lport[c_ix]; tx_dport = pport[c_ix];
    tx_id = {rid[c_ix][63:1], role[c_ix]}; tx_prio = 3'd7; tx_off = 0; tx_len = 0; tx_plen = 0;
    tx_pf = 0; tx_cver = cfg_cver; tx_slot = c_ix;
    if (j_tx) begin
        use_sq = sq_ne && (!c_any || alt);
        if (use_sq) begin
            t_act = A_SQ;
            tx_valid = 1'b1;
            tx_type = sq_type[sq_h]; tx_daddr = sq_addr[sq_h]; tx_sport = sq_sp[sq_h];
            tx_dport = sq_dp[sq_h]; tx_id = sq_id[sq_h];
        end else if (c_any) begin
            if (gneed[c_ix] || (p_regrant[c_ix] && rx_on[c_ix] && !rx_done[c_ix] && r_gnt[c_ix] != 0)) begin
                t_act = A_GRANT; tx_valid = 1'b1; tx_type = T_GRANT;
                tx_off = gneed[c_ix] ? want[c_ix] : r_gnt[c_ix];
                tx_pf = gprio_r[c_ix];
            end else if (p_regrant[c_ix]) begin
                t_act = A_CLR_REGR;
            end else if (p_resend[c_ix]) begin
                if (x_found && !rx_done[c_ix]) begin
                    t_act = A_RESEND; tx_valid = 1'b1; tx_type = T_RESEND;
                    tx_off = x_off; tx_len = x_len;
                    tx_pf = rx_sch[c_ix] ? gprio_r[c_ix] : cut_prio(cfg_cutoffs, r_len[c_ix]);
                end else
                    t_act = A_CLR_RES;
            end else if (p_busy[c_ix]) begin
                t_act = A_BUSY; tx_valid = 1'b1; tx_type = T_BUSY;
            end else if (p_start[c_ix]) begin
                if (tx_on[c_ix] && tx_sch[c_ix]) begin
                    t_act = A_START; tx_valid = 1'b1; tx_type = T_START; tx_len = t_len[c_ix];
                end else
                    t_act = A_CLR_START;
            end else begin
                t_act = A_NEEDACK; tx_valid = 1'b1; tx_type = T_NEED_ACK;
            end
        end else begin
            t_act = A_DATA; t_s = d_ix; tx_valid = 1'b1; tx_type = T_DATA; tx_slot = d_ix;
            tx_daddr = peer[d_ix]; tx_sport = lport[d_ix]; tx_dport = pport[d_ix];
            tx_id = {rid[d_ix][63:1], role[d_ix]};
            if (rs_s[d_ix] < rs_e[d_ix]) begin
                t_poff = rs_s[d_ix]; tx_pf = 8'd1; tx_prio = rs_prio[d_ix];
            end else begin
                t_poff = t_sent[d_ix]; tx_pf = 8'd0; tx_prio = tx_sch[d_ix] ? t_prio[d_ix] : t_uprio[d_ix];
            end
            t_pl = (t_len[d_ix] - t_poff > SEGL) ? SEGL : t_len[d_ix] - t_poff;
            tx_off = t_poff; tx_len = t_len[d_ix]; tx_plen = t_pl; tx_cver = t_cver[d_ix];
        end
    end
end

// CUTOFFS received: new priorities for that peer
reg  [255:0]   n_cut;
reg            pm_hit;
reg  [PLG-1:0] pm_ix;
always @* begin
    for (pi = 0; pi < 8; pi = pi + 1) n_cut[32*pi +: 32] = hb32(eh, 28 + 4*pi);
    pm_hit = 1'b0; pm_ix = prr;
    for (pi = NPEER - 1; pi >= 0; pi = pi - 1)
        if (pv[pi] && paddr[pi] == eh_addr) begin pm_hit = 1'b1; pm_ix = pi; end
end

// peer cutoffs for a message being finalized
reg            fp_hit;
reg  [PLG-1:0] fp_ix;
always @* begin
    fp_hit = 1'b0; fp_ix = 0;
    for (fi = NPEER - 1; fi >= 0; fi = fi - 1)
        if (pv[fi] && paddr[fi] == peer[fin_slot]) begin fp_hit = 1'b1; fp_ix = fi; end
end

// state updates
reg  [TW-1:0] tcnt;
reg  [CW-1:0] cut_cool;
reg           cp_v;            // CUTOFFS owed to a sender with a stale version
reg  [31:0]   cp_addr;
reg  [15:0]   cp_sp, cp_dp;
reg           sp_v;
reg  [7:0]    sp_type;
reg  [31:0]   sp_addr;
reg  [15:0]   sp_sp, sp_dp;
reg  [63:0]   sp_id;
reg           sy_v;
reg  [639:0]  sy_hdr;
reg  [7:0]    sil;
reg  [LW-1:0] tmp_s, tmp_e;
reg  [32:0]   rq_end;
reg  [SL-1:0] t;
wire [SL-1:0] hslot = ld_handle[SL-1:0];

always @(posedge clk) begin
    dq_push <= 1'b0;
    ld_ack  <= 1'b0;
    fin_ack <= 1'b0;
    dd_ack  <= 1'b0;
    sp_v = 1'b0;
    sy_v = 1'b0;
    if (ab_valid && ab_ready) ab_valid <= 1'b0;
    if (rst) begin
        v <= 0; dead <= 0; dlv <= 0; load <= 0;
        ak_v <= 1'b0; ai <= 0; alt <= 1'b0; rr <= 0; prr <= 0; pv <= 0;
        eq_wp <= 0; eq_rp <= 0; sq_wp <= 0; sq_rp <= 0;
        next_id <= {cfg_id_base[63:2], 2'b10}; tcnt <= 0; tm_pend <= 1'b0; tm_ix <= 0; cut_cool <= 0; cp_v <= 1'b0;
        ab_valid <= 1'b0;
        for (i = 0; i < NRPC; i = i + 1) gen[i] <= 8'd0;
    end else begin
        // reclaim dead slots once nothing still uses their buffers
        for (i = 0; i < NRPC; i = i + 1)
            if (v[i] && dead[i] && !dlv[i] && !load[i] && !eg_lock[i] && !(ak_v && ak_slot == i))
                v[i] <= 1'b0;

        if (cut_cool != 0) cut_cool <= cut_cool - 1'b1;
        if (tcnt == TICK - 1) begin
            tcnt <= 0;
            if (!tm_pend) begin tm_pend <= 1'b1; tm_ix <= 0; end
        end else
            tcnt <= tcnt + 1'b1;

        // DATA admission / commit
        if (adm_req && !cmt_req) begin
            ak_v <= adm_ok; ak_slot <= adm_slot; ak_new <= !hit;
        end
        if (cmt_req) begin
            ak_v <= 1'b0;
            if (c_go) begin
                if (c_new) begin
                    v[cs] <= 1'b1; dead[cs] <= 1'b0; role[cs] <= 1'b1; ph[cs] <= PH_SRECV;
                    peer[cs] <= d_saddr; pport[cs] <= d_sport; lport[cs] <= d_dport;
                    rid[cs] <= {d_id[63:1], 1'b0}; gen[cs] <= gen[cs] + 1'b1;
                    dlv[cs] <= 1'b0; load[cs] <= 1'b0;
                    tx_on[cs] <= 1'b0; tx_sch[cs] <= 1'b0; rs_s[cs] <= 0; rs_e[cs] <= 0;
                    p_regrant[cs] <= 1'b0; p_resend[cs] <= 1'b0; p_busy[cs] <= 1'b0;
                    p_start[cs] <= 1'b0; p_needack[cs] <= 1'b0;
                end
                if (c_first) begin
                    rx_on[cs] <= 1'b1; rx_sch[cs] <= 1'b0; r_len[cs] <= d_len[LW-1:0]; r_gnt[cs] <= d_len[LW-1:0];
                end
                bm[cs] <= c_bm; r_rcvd[cs] <= c_rcvd;
                silent[cs] <= 0; nres[cs] <= 0;
                rx_done[cs] <= c_cmpl;
                if (c_cmpl) begin
                    dlv[cs] <= 1'b1;
                    dq_push <= 1'b1;
                    dq_handle <= {(c_new ? gen[cs] + 1'b1 : gen[cs]), cs};
                    dq_resp <= d_id[0];
                    dq_addr <= d_saddr; dq_rport <= d_sport; dq_lport <= d_dport;
                    dq_id <= {d_id[63:1], 1'b0}; dq_len <= d_len;
                    if (!d_id[0]) ph[cs] <= PH_SSERVE;
                    else if (!sq_full) begin   // client: response complete, acknowledge it
                        sp_v = 1'b1; sp_type = T_ACK; sp_addr = d_saddr;
                        sp_sp = d_dport; sp_dp = d_sport; sp_id = {d_id[63:1], 1'b0};
                    end
                end
                // piggybacked acknowledgment in a request packet
                if (!d_id[0] && d_ackid != 0 && !ev_full) begin
                    sy_v = 1'b1;
                    sy_hdr = 640'd0;
                    sy_hdr[8*11 +: 8] = T_ACK;
                    for (k = 0; k < 8; k = k + 1) sy_hdr[8*(20+k) +: 8] = d_ackid[63 - 8*k -: 8];
                    sy_hdr[8*27] = 1'b0;
                end
                // sender is using stale cutoffs (draft §5.2)
                if (d_cver != cfg_cver && cut_cool == 0 && !cp_v) begin
                    cp_v <= 1'b1; cp_addr <= d_saddr; cp_sp <= d_dport; cp_dp <= d_sport;
                end
            end
        end

        // received control packet
        else if (j_ev) begin
            if (!(eh_type == T_ACK && ai < ack_n)) begin
                eq_rp <= eq_rp + 1'b1;
                ai <= 0;
            end
            case (eh_type)
            T_GRANT: if (hit && tx_on[hs] && tx_sch[hs]) begin
                if (eh_f28 > t_gnt[hs]) t_gnt[hs] <= (eh_f28 > t_len[hs]) ? t_len[hs] : eh_f28[LW-1:0];
                t_prio[hs] <= eh_p32;
                silent[hs] <= 0; nres[hs] <= 0;
            end
            T_RESEND: if (hit) begin
                silent[hs] <= 0; nres[hs] <= 0;
                if (!tx_on[hs]) p_busy[hs] <= 1'b1;
                else begin
                    rq_end = {1'b0, eh_f28} + {1'b0, eh_f32};
                    tmp_s = (eh_f32 == 32'hFFFF_FFFF || eh_f28 >= t_sent[hs]) ? 0 :
                            (eh_f28[LW-1:0] & ~(SEGL - 1'b1));
                    tmp_e = (eh_f32 == 32'hFFFF_FFFF || rq_end >= t_sent[hs]) ? t_sent[hs] :
                            ((rq_end[LW-1:0] + SEGL - 1'b1) & ~(SEGL - 1'b1));
                    if (eh_f32 != 32'hFFFF_FFFF && eh_f28 >= t_sent[hs]) tmp_e = 0;
                    if (tmp_e > t_sent[hs]) tmp_e = t_sent[hs];
                    if (tmp_s < tmp_e) begin
                        rs_s[hs] <= (rs_s[hs] < rs_e[hs] && rs_s[hs] < tmp_s) ? rs_s[hs] : tmp_s;
                        rs_e[hs] <= (rs_s[hs] < rs_e[hs] && rs_e[hs] > tmp_e) ? rs_e[hs] : tmp_e;
                        rs_prio[hs] <= eh_p36;
                    end else if (tx_sch[hs] && t_gnt[hs] == 0)
                        p_start[hs] <= 1'b1;
                    else
                        p_busy[hs] <= 1'b1;
                    // the receiver asked for bytes it granted: that GRANT was lost, so honor it now
                    if (tx_sch[hs] && eh_f32 != 32'hFFFF_FFFF && rq_end > t_gnt[hs]) begin
                        t_gnt[hs]  <= (rq_end > t_len[hs]) ? t_len[hs] : rq_end[LW-1:0];
                        t_prio[hs] <= eh_p36;
                    end
                end
            end else if (!sq_full) begin
                sp_v = 1'b1; sp_type = T_UNKNOWN; sp_addr = eh_addr;
                sp_sp = eh_dport; sp_dp = eh_sport; sp_id = eh_id ^ 64'd1;
            end
            T_UNKNOWN: if (hit) begin
                if (!role[hs]) begin
                    // server lost the RPC: send everything again (draft §6.1)
                    if (tx_on[hs] && !rx_done[hs]) begin
                        t_sent[hs] <= 0; rs_s[hs] <= 0; rs_e[hs] <= 0;
                        if (tx_sch[hs]) begin t_gnt[hs] <= 0; p_start[hs] <= 1'b1; end
                        rx_on[hs] <= 1'b0; bm[hs] <= 0; r_rcvd[hs] <= 0;
                        silent[hs] <= 0; nres[hs] <= 0;
                    end
                end else
                    dead[hs] <= 1'b1;
            end
            T_BUSY: if (hit) begin silent[hs] <= 0; nres[hs] <= 0; end
            T_CUTOFFS: begin
                pv[pm_ix] <= 1'b1; paddr[pm_ix] <= eh_addr; pcut[pm_ix] <= n_cut; pver[pm_ix] <= hb16(eh, 60);
                if (!pm_hit) prr <= prr + 1'b1;
                for (i = 0; i < NRPC; i = i + 1)
                    if (v[i] && peer[i] == eh_addr) begin
                        t_uprio[i] <= cut_prio(n_cut, t_len[i]);
                        t_cver[i]  <= hb16(eh, 60);
                    end
            end
            T_NEED_ACK: if (eh_id[0]) begin
                if (hit && !rx_done[hs]) p_resend[hs] <= 1'b1;
                else if (!sq_full) begin
                    sp_v = 1'b1; sp_type = T_ACK; sp_addr = eh_addr;
                    sp_sp = eh_dport; sp_dp = eh_sport; sp_id = {eh_id[63:1], 1'b0};
                end
            end
            // an ACK only means something once our response is going out
            T_ACK: if (!eh_id[0] && hit && ph[hs] == PH_SSEND) dead[hs] <= 1'b1;
            T_START: if (eh_f28 != 0 && eh_f28 <= MAX_MSG) begin
                if (hit) begin
                    // a repeated START from a client must not keep a server RPC alive forever
                    if (!role[hs]) begin silent[hs] <= 0; nres[hs] <= 0; end
                    if (rx_on[hs]) begin
                        if (rx_done[hs]) ;
                        else if (r_gnt[hs] != 0) p_regrant[hs] <= 1'b1;
                        else p_busy[hs] <= 1'b1;
                    end else if (role[hs] == 1'b0 && ph[hs] == PH_CACT) begin
                        rx_on[hs] <= 1'b1; rx_sch[hs] <= 1'b1; r_len[hs] <= eh_f28[LW-1:0];
                        r_gnt[hs] <= 0; r_rcvd[hs] <= 0; bm[hs] <= 0; rx_done[hs] <= 1'b0;
                    end
                end else if (!eh_id[0] && free_any) begin
                    v[fs] <= 1'b1; dead[fs] <= 1'b0; role[fs] <= 1'b1; ph[fs] <= PH_SRECV;
                    peer[fs] <= eh_addr; pport[fs] <= eh_sport; lport[fs] <= eh_dport;
                    rid[fs] <= {eh_id[63:1], 1'b0}; gen[fs] <= gen[fs] + 1'b1;
                    dlv[fs] <= 1'b0; load[fs] <= 1'b0;
                    tx_on[fs] <= 1'b0; tx_sch[fs] <= 1'b0; rs_s[fs] <= 0; rs_e[fs] <= 0;
                    rx_on[fs] <= 1'b1; rx_sch[fs] <= 1'b1; rx_done[fs] <= 1'b0;
                    r_len[fs] <= eh_f28[LW-1:0]; r_gnt[fs] <= 0; r_rcvd[fs] <= 0; bm[fs] <= 0;
                    p_regrant[fs] <= 1'b0; p_resend[fs] <= 1'b0; p_busy[fs] <= 1'b0;
                    p_start[fs] <= 1'b0; p_needack[fs] <= 1'b0;
                    silent[fs] <= 0; nres[fs] <= 0;
                end
            end
            default: ;
            endcase
            if (eh_type == T_ACK && ai < ack_n) ai <= ai + 1'b1;
        end

        // transmit
        else if (j_tx) begin
            if (sq_ne && c_any) alt <= !alt;
            case (t_act)
            A_SQ:        sq_rp <= sq_rp + 1'b1;
            A_GRANT:     begin
                             if (tx_off[LW-1:0] > r_gnt[t_s]) r_gnt[t_s] <= tx_off[LW-1:0];
                             p_regrant[t_s] <= 1'b0;
                         end
            A_CLR_REGR:  p_regrant[t_s] <= 1'b0;
            A_RESEND,
            A_CLR_RES:   p_resend[t_s] <= 1'b0;
            A_BUSY:      p_busy[t_s] <= 1'b0;
            A_START,
            A_CLR_START: p_start[t_s] <= 1'b0;
            A_NEEDACK:   p_needack[t_s] <= 1'b0;
            A_DATA:      if (rs_s[t_s] < rs_e[t_s])
                             rs_s[t_s] <= rs_s[t_s] + SEGL;
                         else begin
                             t_sent[t_s] <= t_sent[t_s] + t_pl;
                             if (t_sent[t_s] + t_pl == t_len[t_s]) begin silent[t_s] <= 0; nres[t_s] <= 0; end
                         end
            default: ;
            endcase
            if (t_act != A_SQ && t_act != A_DATA) rr <= t_s + 1'b1;
        end

        // delivery finished
        else if (j_dd) begin
            dd_ack <= 1'b1;
            dlv[dd_slot] <= 1'b0;
            if (!role[dd_slot]) dead[dd_slot] <= 1'b1;   // client: RPC complete
        end

        // application send
        else if (j_ld) begin
            if (fin_req) begin
                fin_ack <= 1'b1;
                load[fin_slot] <= 1'b0;
                if (!dead[fin_slot]) begin
                    tx_on[fin_slot] <= 1'b1;
                    tx_sch[fin_slot] <= (t_len[fin_slot] > UNSCHED_MAX);
                    t_gnt[fin_slot] <= (t_len[fin_slot] > UNSCHED_MAX) ? {LW{1'b0}} : t_len[fin_slot];
                    p_start[fin_slot] <= (t_len[fin_slot] > UNSCHED_MAX);
                    t_sent[fin_slot] <= 0; rs_s[fin_slot] <= 0; rs_e[fin_slot] <= 0;
                    t_uprio[fin_slot] <= fp_hit ? cut_prio(pcut[fp_ix], t_len[fin_slot]) : 3'd7;
                    t_cver[fin_slot]  <= fp_hit ? pver[fp_ix] : 16'd0;
                    t_prio[fin_slot]  <= 3'd0;
                    silent[fin_slot] <= 0; nres[fin_slot] <= 0;
                    ph[fin_slot] <= role[fin_slot] ? PH_SSEND : PH_CACT;
                end
            end else begin
                ld_ack <= 1'b1;
                ld_ok  <= 1'b0;
                if (ld_mlen == 0 || ld_mlen > MAX_MSG) ;
                else if (!ld_resp) begin
                    if (nfree > SRV_RESERVE) begin
                        ld_ok <= 1'b1; ld_hout <= {gen[fs] + 1'b1, fs}; ld_id <= next_id;
                        next_id <= next_id + 64'd2;
                        v[fs] <= 1'b1; dead[fs] <= 1'b0; role[fs] <= 1'b0; ph[fs] <= PH_CLOAD;
                        peer[fs] <= ld_addr; pport[fs] <= ld_rport; lport[fs] <= ld_lport;
                        rid[fs] <= next_id; gen[fs] <= gen[fs] + 1'b1;
                        dlv[fs] <= 1'b0; load[fs] <= 1'b1; t_len[fs] <= ld_mlen[LW-1:0];
                        tx_on[fs] <= 1'b0; tx_sch[fs] <= 1'b0; rs_s[fs] <= 0; rs_e[fs] <= 0;
                        rx_on[fs] <= 1'b0; rx_sch[fs] <= 1'b0; rx_done[fs] <= 1'b0;
                        bm[fs] <= 0; r_rcvd[fs] <= 0; r_gnt[fs] <= 0; r_len[fs] <= 0;
                        p_regrant[fs] <= 1'b0; p_resend[fs] <= 1'b0; p_busy[fs] <= 1'b0;
                        p_start[fs] <= 1'b0; p_needack[fs] <= 1'b0;
                        silent[fs] <= 0; nres[fs] <= 0;
                    end
                end else if (v[hslot] && !dead[hslot] && role[hslot] && ph[hslot] == PH_SSERVE &&
                             gen[hslot] == ld_handle[SL+7:SL]) begin
                    ld_ok <= 1'b1; ld_hout <= ld_handle; ld_id <= rid[hslot];
                    ph[hslot] <= PH_SLOAD; load[hslot] <= 1'b1; t_len[hslot] <= ld_mlen[LW-1:0];
                end
            end
        end

        // timers: one slot per job
        else if (j_tm) begin
            t = tm_ix;
            tm_ix <= tm_ix + 1'b1;
            if (tm_ix == NRPC - 1) tm_pend <= 1'b0;
            if (v[t] && !dead[t] && !(ak_v && ak_slot == t)) begin
                sil = (silent[t] == 8'hFF) ? 8'hFF : silent[t] + 1'b1;
                if (!role[t] && ph[t] == PH_CACT) begin
                    if (tx_sch[t] && t_gnt[t] == 0 && !rx_done[t]) begin
                        if (sil >= RESEND_TICKS) begin p_start[t] <= 1'b1; nres[t] <= nres[t] + 1'b1; sil = 0; end
                    end else if (t_sent[t] >= t_gnt[t] && !rx_done[t] &&
                                 !(rx_on[t] && rx_sch[t] && r_rcvd[t] >= r_gnt[t])) begin
                        // draft §6.2: no RESEND while the missing bytes are ones we have not granted
                        if (sil >= RESEND_TICKS) begin
                            p_resend[t] <= 1'b1; nres[t] <= nres[t] + 1'b1; sil = 0;
                            if (rx_on[t] && rx_sch[t] && r_gnt[t] != 0) p_regrant[t] <= 1'b1;
                        end
                    end else
                        nres[t] <= 0;
                    if (nres[t] >= ABORT_RESENDS && !rx_done[t] && (!ab_valid || ab_ready)) begin
                        dead[t] <= 1'b1; ab_valid <= 1'b1; ab_id <= rid[t];
                    end
                end else if (role[t] && ph[t] == PH_SRECV) begin
                    if (r_rcvd[t] < (rx_sch[t] ? r_gnt[t] : r_len[t]) && sil >= RESEND_TICKS) begin
                        p_resend[t] <= 1'b1; nres[t] <= nres[t] + 1'b1; sil = 0;
                        if (rx_sch[t] && r_gnt[t] != 0) p_regrant[t] <= 1'b1;
                    end
                    if (nres[t] >= ABORT_RESENDS) dead[t] <= 1'b1;
                end else if (role[t] && ph[t] == PH_SSEND) begin
                    if (tx_sch[t] && t_gnt[t] == 0) begin
                        if (sil >= RESEND_TICKS) begin p_start[t] <= 1'b1; nres[t] <= nres[t] + 1'b1; sil = 0; end
                    end else if (t_sent[t] == t_len[t]) begin
                        if (sil >= NEEDACK_TICKS) begin p_needack[t] <= 1'b1; nres[t] <= nres[t] + 1'b1; sil = 0; end
                    end else if (sil >= RESEND_TICKS) begin
                        nres[t] <= nres[t] + 1'b1; sil = 0;
                    end
                    if (nres[t] >= ACK_GIVEUP) dead[t] <= 1'b1;
                end
                silent[t] <= sil;
            end
        end

        // queue writes
        if (cp_v && !sp_v && !sq_full) begin
            sp_v = 1'b1; sp_type = T_CUTOFFS; sp_addr = cp_addr; sp_sp = cp_sp; sp_dp = cp_dp; sp_id = 64'd0;
            cp_v <= 1'b0;
            cut_cool <= CUT_COOL;
        end
        if (ev_push || sy_v) begin
            eq_hdr[eq_wp[EQL-1:0]]  <= ev_push ? ev_hdr : sy_hdr;
            eq_addr[eq_wp[EQL-1:0]] <= ev_push ? ev_saddr : d_saddr;
            eq_len[eq_wp[EQL-1:0]]  <= ev_push ? ev_len : 16'd30;
            eq_wp <= eq_wp + 1'b1;
        end
        if (sp_v) begin
            sq_type[sq_wp[SQL-1:0]] <= sp_type; sq_addr[sq_wp[SQL-1:0]] <= sp_addr;
            sq_sp[sq_wp[SQL-1:0]] <= sp_sp; sq_dp[sq_wp[SQL-1:0]] <= sp_dp; sq_id[sq_wp[SQL-1:0]] <= sp_id;
            sq_wp <= sq_wp + 1'b1;
        end
    end
end

endmodule
