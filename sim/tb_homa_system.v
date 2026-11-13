`timescale 1ns/1ps
`define ERR(x) begin errors = errors + 1; if (errors <= 50) begin $write("%0t ERROR: ", $time); $display x; end end

// Three Homa endpoints on a simulated network. The network can lose, duplicate,
// delay (reorder) and flag packets bad, and serves higher priorities first like
// a switch. A monitor checks every packet on the wire against the draft and the
// grants that were actually delivered. Application models check every byte
// end to end and count executions per RPC.
module tb_sys;
parameter SEG    = 64;
parameter MAXM   = 1024;
parameter UNS    = 256;
parameter RTT    = 256;
parameter NRPC   = 8;
parameter K      = 2;
parameter NPEER  = 2;
parameter TICK   = 64;
parameter RTK    = 8;
parameter ABR    = 4;
parameter NAK    = 8;
parameter AGU    = 6;
parameter SEED   = 1;
parameter LOSS   = 0;      // per mille
parameter DUP    = 0;      // per mille
parameter BAD    = 0;      // per mille: frame error flagged on tlast
parameter JIT    = 0;      // extra random network delay, cycles
parameter NREQ   = 150;    // requests issued per endpoint
parameter MAXOUT = 2;      // outstanding requests per client
parameter STRICT = 1;      // every RPC must succeed exactly once
parameter SVC    = 60;     // max server think time, cycles (long values cause BUSY)
parameter APPST  = 40;     // app receive stall, of 256
parameter NETST  = 20;     // network egress stall, of 256
parameter SLV    = 3;      // grant priority levels
parameter GAP    = 100;    // mean cycles between requests per endpoint
parameter TRACE  = -1;     // print every packet of this RPC

localparam NE   = 3;
localparam SL   = $clog2(NRPC);
localparam PB   = 64 + SEG;
localparam PMAX = 512;
localparam RMAX = NE * NREQ + 8;
localparam [31:0] BASE = 32'h0A000001;
localparam [7:0] T_DATA = 16, T_GRANT = 17, T_RESEND = 18, T_UNKNOWN = 19, T_BUSY = 20,
                 T_CUTOFFS = 21, T_NEED_ACK = 23, T_ACK = 24, T_START = 25;

reg clk = 1'b0;
always #5 clk = ~clk;
reg rst = 1'b1;
integer seed = SEED;
integer errors = 0;
integer now = 0;
always @(posedge clk) now = now + 1;

// helpers
function [7:0] pbyte(input integer c, input [63:0] id, input integer j, input integer kind);
    reg [31:0] x;
    begin
        x = (id[31:0] ^ (c * 32'h01000193) ^ (kind * 32'h7F4A7C15)) * 32'h9E3779B1 + j * 32'h85EBCA77;
        x = x ^ (x >> 15);
        x = x * 32'h2C1B3C6D;
        pbyte = x[7:0] ^ x[31:24];
    end
endfunction

function integer resp_len(input integer ql, input [63:0] id);
    resp_len = ((ql * 37 + id[15:0] * 11) % MAXM) + 1;
endfunction

function [255:0] cfg_cut(input integer e);
    begin
        cfg_cut[0*32 +: 32] = MAXM; cfg_cut[1*32 +: 32] = MAXM; cfg_cut[2*32 +: 32] = MAXM;
        cfg_cut[3*32 +: 32] = UNS;  cfg_cut[4*32 +: 32] = UNS / 2 - e * (UNS / 32);
        cfg_cut[5*32 +: 32] = UNS / 4; cfg_cut[6*32 +: 32] = UNS / 8 + e; cfg_cut[7*32 +: 32] = 1;
    end
endfunction
function [15:0] cfg_ver(input integer e); cfg_ver = 16'h0100 + e; endfunction

function [2:0] cprio(input [255:0] c, input [31:0] len);
    integer q, f;
    begin
        cprio = 0; f = 0;
        for (q = 7; q >= 0; q = q - 1) if (!f && c[32*q +: 32] >= len) begin cprio = q; f = 1; end
    end
endfunction

// RPC scoreboard
reg  [63:0] r_id    [0:RMAX-1];
integer     r_cli   [0:RMAX-1];
integer     r_srv   [0:RMAX-1];
integer     r_ql    [0:RMAX-1];
integer     r_st    [0:RMAX-1];   // 0 not yet issued, 1 issued, 2 answered, 3 aborted
integer     r_exec  [0:RMAX-1];
integer     r_got   [0:RMAX-1];
integer     g_req   [0:RMAX-1];   // highest GRANT offset delivered to the client for its request
integer     g_resp  [0:RMAX-1];   // ... to the server for its response
reg  [15:0] r_cp    [0:RMAX-1];
reg  [15:0] r_sp    [0:RMAX-1];
integer     nr = 0;
integer     n_abort = 0, n_done = 0, n_reexec = 0, n_rej = 0;

function automatic integer find_rpc(input integer c, input [63:0] id);
    integer q;
    begin
        find_rpc = -1;
        for (q = 0; q < nr; q = q + 1)
            if (r_cli[q] == c && r_st[q] != 0 && r_id[q] == {id[63:1], 1'b0}) find_rpc = q;
    end
endfunction

// network pool
reg  [7:0]  pk   [0:PMAX*PB-1];
reg         p_v  [0:PMAX-1];
reg         p_busy [0:PMAX-1];
integer     p_src[0:PMAX-1];
integer     p_dst[0:PMAX-1];
integer     p_len[0:PMAX-1];
integer     p_rdy[0:PMAX-1];
integer     p_seq[0:PMAX-1];
reg  [2:0]  p_pri[0:PMAX-1];
reg         p_bad[0:PMAX-1];
integer     pseq = 0;
integer     n_pkts = 0, n_lost = 0, n_dupd = 0, n_bad = 0, n_full = 0;
integer     n_type [0:31];
integer     nq [0:2];
integer     pz;
initial begin
    for (pz = 0; pz < PMAX; pz = pz + 1) begin p_v[pz] = 0; p_busy[pz] = 0; end
    for (pz = 0; pz < 32; pz = pz + 1) n_type[pz] = 0;
    for (pz = 0; pz < 3; pz = pz + 1) nq[pz] = 0;
end

// capture buffers, one per endpoint
reg  [7:0]  cb [0:NE*PB-1];

function [31:0] g32(input integer a); g32 = {cb[a], cb[a+1], cb[a+2], cb[a+3]}; endfunction
function [15:0] g16(input integer a); g16 = {cb[a], cb[a+1]}; endfunction

// independent wire-format and protocol checks on a packet leaving endpoint e
task automatic check_pkt(input integer e, input integer n, input [31:0] daddr, input [2:0] pri);
    integer b, d, k, j, plen, cli;
    reg [7:0]   ty;
    reg [63:0]  id;
    reg [31:0]  mlen, off, gn;
    reg [15:0]  cv;
    reg [255:0] cc;
    reg         S, retr;
    begin
        b = e * PB;
        ty = cb[b + 11];
        id = {g32(b + 20), g32(b + 24)};
        S  = id[0];
        d  = daddr - BASE;
        if (ty < 32) n_type[ty] = n_type[ty] + 1;
        if (d < 0 || d >= NE || d == e) `ERR(("ep%0d: packet to bad address %h", e, daddr))
        for (j = 4; j < 20; j = j + 1)
            if (j != 11 && cb[b + j] != 0) `ERR(("ep%0d type %0d: reserved byte %0d = %h", e, ty, j, cb[b + j]))
        cli = S ? d : e;
        k = (ty == T_CUTOFFS) ? -2 : find_rpc(cli, id);
        if (k == TRACE && TRACE >= 0)
            $display("%0t TRACE send ep%0d->ep%0d type %0d S=%0d f28=%0d f48=%0d n=%0d pri=%0d", $time, e, d, ty, S, g32(b+28), g32(b+48), n, pri);
        if (ty != T_DATA && pri != 3'd7) `ERR(("ep%0d type %0d: control packet at priority %0d", e, ty, pri))
        case (ty)
        T_DATA: begin
            mlen = g32(b + 28); cv = g16(b + 42); retr = cb[b + 44]; off = g32(b + 48); plen = n - 52;
            if (g32(b + 32) != 0 || g32(b + 36) != 0 || g16(b + 40) != 0 || cb[b+45] != 0 || cb[b+46] != 0 || cb[b+47] != 0)
                `ERR(("ep%0d DATA: nonzero ack/reserved fields", e))
            if (off % SEG != 0 || mlen == 0 || mlen > MAXM || off >= mlen ||
                plen != ((mlen - off > SEG) ? SEG : mlen - off) || retr > 1)
                `ERR(("ep%0d DATA: bad framing len=%0d off=%0d plen=%0d retr=%0d", e, mlen, off, plen, retr))
            else if (k < 0)
                `ERR(("ep%0d DATA: unknown RPC id=%h S=%0d", e, id, S))
            else begin
                if (mlen != (S ? resp_len(r_ql[k], r_id[k]) : r_ql[k]))
                    `ERR(("ep%0d DATA rpc %0d: message length %0d wrong", e, k, mlen))
                for (j = 0; j < plen; j = j + 1)
                    if (cb[b + 52 + j] != pbyte(cli, r_id[k], off + j, S))
                        `ERR(("ep%0d DATA rpc %0d off %0d: wire byte %0d corrupt", e, k, off, j))
                gn = S ? g_resp[k] : g_req[k];
                if (mlen > UNS && off + plen > gn)
                    `ERR(("ep%0d DATA rpc %0d: sent %0d..%0d beyond grant %0d", e, k, off, off + plen, gn))
                if (!retr && mlen <= UNS &&
                    !((cv == 0 && pri == 7) || (cv == cfg_ver(d) && pri == cprio(cfg_cut(d), mlen))))
                    `ERR(("ep%0d DATA rpc %0d: unscheduled priority %0d with cutoff version %h", e, k, pri, cv))
                if (!retr && mlen > UNS && pri >= SLV)
                    `ERR(("ep%0d DATA rpc %0d: scheduled priority %0d not a grant level", e, k, pri))
            end
        end
        T_START: begin
            mlen = g32(b + 28);
            if (n != 32) `ERR(("ep%0d START_MSG length %0d", e, n))
            if (k < 0) `ERR(("ep%0d START_MSG: unknown RPC", e))
            else if (mlen <= UNS || mlen != (S ? resp_len(r_ql[k], r_id[k]) : r_ql[k]))
                `ERR(("ep%0d START_MSG rpc %0d: length %0d", e, k, mlen))
        end
        T_GRANT: begin
            off = g32(b + 28);
            if (n != 33) `ERR(("ep%0d GRANT length %0d", e, n))
            if (cb[b + 32] >= SLV) `ERR(("ep%0d GRANT priority %0d", e, cb[b + 32]))
            if (k < 0) `ERR(("ep%0d GRANT: unknown RPC", e))
            else begin
                mlen = S ? r_ql[k] : resp_len(r_ql[k], r_id[k]);   // the message the peer is sending
                if (mlen <= UNS || off > mlen || off == 0 || (off % SEG != 0 && off != mlen))
                    `ERR(("ep%0d GRANT rpc %0d: offset %0d for length %0d", e, k, off, mlen))
            end
        end
        T_RESEND: begin
            off = g32(b + 28); mlen = g32(b + 32);
            if (n != 37) `ERR(("ep%0d RESEND length %0d", e, n))
            if (off % SEG != 0 || mlen == 0 || (mlen != 32'hFFFFFFFF && off + mlen > MAXM))
                `ERR(("ep%0d RESEND off=%0d len=%0d", e, off, mlen))
        end
        T_CUTOFFS: begin
            cc = cfg_cut(e);
            if (n != 62) `ERR(("ep%0d CUTOFFS length %0d", e, n))
            for (j = 0; j < 8; j = j + 1)
                if (g32(b + 28 + 4*j) != cc[32*j +: 32]) `ERR(("ep%0d CUTOFFS[%0d] = %0d", e, j, g32(b + 28 + 4*j)))
            if (g16(b + 60) != cfg_ver(e)) `ERR(("ep%0d CUTOFFS version %h", e, g16(b + 60)))
        end
        T_ACK:      if (n != 30 || g16(b + 28) != 0 || S) `ERR(("ep%0d ACK length %0d S=%0d", e, n, S))
        T_NEED_ACK: if (n != 28 || !S) `ERR(("ep%0d NEED_ACK length %0d S=%0d", e, n, S))
        T_UNKNOWN, T_BUSY: if (n != 28) `ERR(("ep%0d type %0d length %0d", e, ty, n))
        default: `ERR(("ep%0d: unknown packet type %0d", e, ty))
        endcase
    end
endtask

// a GRANT reaching the sender raises the limit the monitor enforces
task automatic note_delivery(input integer p);
    integer b, k, cli;
    reg [63:0] id;
    reg [31:0] off;
    begin
        b = p * PB;
        // a RESEND names bytes the receiver wants, so it also authorizes them (covers a lost GRANT)
        if ((pk[b + 11] == T_GRANT || pk[b + 11] == T_RESEND) && !p_bad[p]) begin
            id  = {pk[b+20], pk[b+21], pk[b+22], pk[b+23], pk[b+24], pk[b+25], pk[b+26], pk[b+27]};
            off = {pk[b+28], pk[b+29], pk[b+30], pk[b+31]};
            if (pk[b + 11] == T_RESEND)
                off = ({pk[b+32], pk[b+33], pk[b+34], pk[b+35]} == 32'hFFFFFFFF) ? 0 :
                      off + {pk[b+32], pk[b+33], pk[b+34], pk[b+35]};
            cli = id[0] ? p_dst[p] : p_src[p];
            k = find_rpc(cli, id);
            if (k == TRACE && TRACE >= 0) $display("%0t TRACE deliver GRANT to ep%0d off %0d", $time, p_dst[p], off);
            if (k >= 0) begin
                if (id[0]) begin if (off > g_req[k]) g_req[k] = off; end
                else       begin if (off > g_resp[k]) g_resp[k] = off; end
            end
        end
    end
endtask

task automatic net_put(input integer e, input integer n, input [31:0] daddr, input [2:0] pri);
    integer p, j, copies, c;
    begin
        n_pkts = n_pkts + 1;
        copies = 1;
        if ({$random(seed)} % 1000 < LOSS) begin copies = 0; n_lost = n_lost + 1; end
        else if ({$random(seed)} % 1000 < DUP) begin copies = 2; n_dupd = n_dupd + 1; end
        for (c = 0; c < copies; c = c + 1) begin
            p = -1;
            for (j = PMAX - 1; j >= 0; j = j - 1) if (!p_v[j]) p = j;
            if (p < 0) n_full = n_full + 1;
            else begin
                for (j = 0; j < n; j = j + 1) pk[p*PB + j] = cb[e*PB + j];
                p_v[p] = 1; p_busy[p] = 0; p_src[p] = e; p_dst[p] = daddr - BASE; p_len[p] = n;
                p_pri[p] = pri; p_seq[p] = pseq; pseq = pseq + 1;
                p_rdy[p] = now + 20 + ((JIT > 0) ? {$random(seed)} % (JIT + 1) : 0) + c * ({$random(seed)} % 400);
                p_bad[p] = ({$random(seed)} % 1000 < BAD);
                if (p_bad[p]) begin
                    n_bad = n_bad + 1;
                    j = {$random(seed)} % n;                  // a flagged frame is also really damaged
                    pk[p*PB + j] = pk[p*PB + j] ^ (8'h01 << ({$random(seed)} % 8));
                end
                nq[p_dst[p]] = nq[p_dst[p]] + 1;
            end
        end
    end
endtask

// endpoints
reg  quiet = 1'b0;
integer issued [0:NE-1];
integer n_inflight = 0;

genvar g;
generate for (g = 0; g < NE; g = g + 1) begin : ep
    wire [63:0] ID_BASE = 64'h1234_0000_0000_0000 * (g + 1) + 64'd4 * SEED;
    reg  [63:0] in_d = 0; reg [7:0] in_k = 0; reg in_v = 0, in_l = 0, in_u = 0; reg [31:0] in_sa = 0;
    wire [63:0] o_d;   wire [7:0] o_k;  wire o_v, o_l;  reg o_r = 0;
    wire [31:0] o_da;  wire [2:0] o_p;  wire [15:0] o_len;
    reg         a_v = 0, a_resp = 0;  wire a_r;
    reg  [31:0] a_addr = 0, a_len = 0;  reg [15:0] a_rp = 0, a_lp = 0;  reg [SL+7:0] a_h = 0;
    reg  [63:0] a_d = 0;  reg a_dv = 0;  wire a_dr;
    wire        st_v, st_ok, st_resp;  wire [63:0] st_id;  wire [SL+7:0] st_h;
    wire [63:0] r_d;  wire [7:0] r_k;  wire r_v, r_l;  reg r_r = 0;
    wire        r_resp;  wire [31:0] r_addr, r_len;  wire [15:0] r_rp, r_lp;  wire [63:0] r_rid;  wire [SL+7:0] r_h;
    wire        ab_v;  wire [63:0] ab_id;  reg ab_r = 0;
    wire        drop;

    homa_endpoint #(
        .SEG_BYTES(SEG), .MAX_MSG(MAXM), .UNSCHED_MAX(UNS), .RTT_BYTES(RTT), .NRPC(NRPC),
        .OVERCOMMIT(K), .NPEER(NPEER), .TICK(TICK), .RESEND_TICKS(RTK), .ABORT_RESENDS(ABR),
        .NEEDACK_TICKS(NAK), .ACK_GIVEUP(AGU), .EVQ(4), .SLQ(8), .CUT_COOL(64)
    ) u (
        .clk(clk), .rst(rst),
        .cfg_cutoffs(cfg_cut(g)), .cfg_cver(cfg_ver(g)), .cfg_sched_lvls(SLV[3:0]), .cfg_id_base(ID_BASE),
        .s_net_tdata(in_d), .s_net_tkeep(in_k), .s_net_tvalid(in_v), .s_net_tlast(in_l), .s_net_tuser(in_u),
        .s_net_tready(), .s_net_saddr(in_sa),
        .m_net_tdata(o_d), .m_net_tkeep(o_k), .m_net_tvalid(o_v), .m_net_tlast(o_l), .m_net_tready(o_r),
        .m_net_daddr(o_da), .m_net_prio(o_p), .m_net_len(o_len),
        .s_app_valid(a_v), .s_app_ready(a_r), .s_app_resp(a_resp), .s_app_addr(a_addr), .s_app_rport(a_rp),
        .s_app_lport(a_lp), .s_app_handle(a_h), .s_app_len(a_len), .s_app_tdata(a_d), .s_app_tvalid(a_dv),
        .s_app_tready(a_dr), .st_valid(st_v), .st_ok(st_ok), .st_resp(st_resp), .st_id(st_id), .st_handle(st_h),
        .m_app_tdata(r_d), .m_app_tkeep(r_k), .m_app_tvalid(r_v), .m_app_tlast(r_l), .m_app_tready(r_r),
        .m_app_resp(r_resp), .m_app_addr(r_addr), .m_app_rport(r_rp), .m_app_lport(r_lp), .m_app_id(r_rid),
        .m_app_handle(r_h), .m_app_len(r_len),
        .ab_valid(ab_v), .ab_id(ab_id), .ab_ready(ab_r), .rx_drop(drop));

    // egress capture + monitor
    integer cn = 0, cj;
    reg [31:0] c_da; reg [2:0] c_pr; reg [15:0] c_len;
    always @(posedge clk) begin
        if (o_v && o_r) begin
            if (cn == 0) begin c_da = o_da; c_pr = o_p; c_len = o_len; end
            else if (c_da !== o_da || c_pr !== o_p || c_len !== o_len) `ERR(("ep%0d: sideband changed mid-packet", g))
            if (!o_l && o_k !== 8'hFF) `ERR(("ep%0d: partial tkeep mid-packet", g))
            for (cj = 0; cj < 8; cj = cj + 1)
                if (o_k[cj]) begin
                    if (cn < PB) cb[g*PB + cn] = o_d[8*cj +: 8];
                    cn = cn + 1;
                end else if ((o_k >> cj) != 0) `ERR(("ep%0d: tkeep %b not contiguous", g, o_k))
            if (o_l) begin
                if (cn != c_len || cn > PB) `ERR(("ep%0d: packet %0d bytes, sideband says %0d", g, cn, c_len))
                else begin
                    check_pkt(g, cn, c_da, c_pr);
                    net_put(g, cn, c_da, c_pr);
                end
                cn = 0;
            end
        end
        o_r <= ({$random(seed)} % 256) >= NETST;
    end

    // ingress: deliver queued packets, highest priority first
    integer ip = -1, ipos = 0, ib, iq, best;
    always @(posedge clk) begin
        if (in_v && in_l) begin p_v[ip] = 0; ip = -1; end
        if (ip < 0 && nq[g] > 0 && ({$random(seed)} % 8 != 0)) begin
            best = -1;
            for (iq = 0; iq < PMAX; iq = iq + 1)
                if (p_v[iq] && !p_busy[iq] && p_dst[iq] == g && p_rdy[iq] <= now &&
                    (best < 0 || p_pri[iq] > p_pri[best] || (p_pri[iq] == p_pri[best] && p_seq[iq] < p_seq[best])))
                    best = iq;
            if (best >= 0) begin ip = best; ipos = 0; p_busy[ip] = 1; nq[g] = nq[g] - 1; note_delivery(ip); end
        end
        if (ip >= 0) begin
            for (ib = 0; ib < 8; ib = ib + 1) begin
                in_d[8*ib +: 8] <= (ipos + ib < p_len[ip]) ? pk[ip*PB + ipos + ib] : 8'h00;
                in_k[ib]        <= (ipos + ib < p_len[ip]);
            end
            in_v  <= 1'b1;
            in_l  <= (ipos + 8 >= p_len[ip]);
            in_u  <= (ipos + 8 >= p_len[ip]) && p_bad[ip];
            in_sa <= BASE + p_src[ip];
            ipos = ipos + 8;
        end else
            in_v <= 1'b0;
    end

    // application send: responses first, then requests
    integer qk [0:63];  integer qt [0:63];  integer qh = 0, qtl = 0;
    integer sk [0:63];  integer stt[0:63];  reg [SL+7:0] sh [0:63];  integer shd = 0, stl = 0;
    integer ak, aw, anw, aj, ast = 0, n_out = 0;
    always @(posedge clk) begin
        if (a_v && a_r) a_v <= 1'b0;
        case (ast)
        0: if (!rst) begin
            if (shd != stl && stt[shd % 64] <= now) begin
                ak = sk[shd % 64]; a_resp <= 1'b1; a_h <= sh[shd % 64];
                a_len <= resp_len(r_ql[ak], r_id[ak]); anw = (resp_len(r_ql[ak], r_id[ak]) + 7) / 8;
                a_v <= 1'b1; ast = 1;
            end else if (qh != qtl && qt[qh % 64] <= now) begin
                ak = qk[qh % 64]; a_resp <= 1'b0; a_h <= 0;
                a_addr <= BASE + r_srv[ak]; a_rp <= r_sp[ak]; a_lp <= r_cp[ak];
                a_len <= r_ql[ak]; anw = (r_ql[ak] + 7) / 8;
                a_v <= 1'b1; ast = 1;
            end
        end
        1: if (st_v) begin
            if (st_resp != a_resp) `ERR(("ep%0d: status for the wrong kind of send", g))
            if (a_resp) begin
                shd = shd + 1;
                if (!st_ok) `ERR(("ep%0d: response for rpc %0d rejected", g, ak))
            end else if (st_ok) begin
                if (find_rpc(g, st_id) >= 0) `ERR(("ep%0d: id %h reused", g, st_id))
                if (st_id[0] || st_id == 0) `ERR(("ep%0d: client id %h not even/nonzero", g, st_id))
                r_id[ak] = st_id; r_st[ak] = 1;
                qh = qh + 1;
            end else begin
                n_rej = n_rej + 1;
                qt[qh % 64] = now + 50 + {$random(seed)} % 200;   // no free slot: retry later
            end
            // the payload follows even when rejected; the endpoint discards it
            aw = 0;
            ast = (anw == 0) ? 0 : 2;
        end
        2: begin
            if (a_dv && a_dr) aw = aw + 1;
            if (!(a_dv && !a_dr)) begin
                if (aw == anw) begin a_dv <= 1'b0; ast = 0; end
                else if ({$random(seed)} % 8 != 0) begin
                    for (aj = 0; aj < 8; aj = aj + 1)
                        a_d[8*aj +: 8] <= pbyte(a_resp ? r_cli[ak] : g, r_id[ak], 8*aw + aj, a_resp);
                    a_dv <= 1'b1;
                end else
                    a_dv <= 1'b0;
            end
        end
        endcase
    end

    // request generator
    integer ql, srv, nk;
    always @(posedge clk) begin
        if (!rst && !quiet && issued[g] < NREQ && n_out < MAXOUT && ({$random(seed)} % GAP == 0)) begin
            nk = nr; nr = nr + 1;
            srv = (g + 1 + {$random(seed)} % (NE - 1)) % NE;
            case ({$random(seed)} % 8)
            0: ql = 1 + {$random(seed)} % 16;
            1: ql = UNS;
            2: ql = (UNS + 1 > MAXM) ? MAXM : UNS + 1;
            3: ql = MAXM - {$random(seed)} % 4;
            4: ql = SEG * (1 + {$random(seed)} % 4);
            default: ql = 1 + {$random(seed)} % MAXM;
            endcase
            r_cli[nk] = g; r_srv[nk] = srv; r_ql[nk] = ql; r_st[nk] = 0; r_exec[nk] = 0; r_got[nk] = 0;
            g_req[nk] = 0; g_resp[nk] = 0; r_id[nk] = 0;
            r_cp[nk] = 16'h4000 + {$random(seed)} % 16; r_sp[nk] = 16'h0050 + {$random(seed)} % 4;
            qk[qtl % 64] = nk; qt[qtl % 64] = now; qtl = qtl + 1;
            issued[g] = issued[g] + 1;
            n_out = n_out + 1;
            n_inflight = n_inflight + 1;
        end
    end

    // application receive: check every byte
    reg  [7:0] mb [0:MAXM+7];
    integer mn = 0, mj, mk, mbad, mcli;
    reg [31:0] m_addr0, m_len0; reg [63:0] m_id0; reg m_resp0; reg [15:0] m_rp0, m_lp0; reg [SL+7:0] m_h0;
    always @(posedge clk) begin
        if (r_v && r_r) begin
            if (mn == 0) begin
                m_addr0 = r_addr; m_len0 = r_len; m_id0 = r_rid; m_resp0 = r_resp; m_rp0 = r_rp; m_lp0 = r_lp; m_h0 = r_h;
            end else if (m_addr0 !== r_addr || m_len0 !== r_len || m_id0 !== r_rid || m_resp0 !== r_resp)
                `ERR(("ep%0d: delivery sideband changed mid-message", g))
            for (mj = 0; mj < 8; mj = mj + 1)
                if (r_k[mj]) begin if (mn < MAXM + 8) mb[mn] = r_d[8*mj +: 8]; mn = mn + 1; end
            if (r_l) begin
                mcli = m_resp0 ? g : m_addr0 - BASE;
                mk = find_rpc(mcli, m_id0);
                if (mk < 0) `ERR(("ep%0d: delivered unknown message id=%h resp=%0d", g, m_id0, m_resp0))
                else begin
                    if (mn != m_len0 || mn != (m_resp0 ? resp_len(r_ql[mk], r_id[mk]) : r_ql[mk]))
                        `ERR(("ep%0d rpc %0d: delivered %0d bytes, sideband %0d", g, mk, mn, m_len0))
                    if (m_resp0 ? (m_rp0 != r_sp[mk] || m_lp0 != r_cp[mk] || m_addr0 != BASE + r_srv[mk])
                                : (m_rp0 != r_cp[mk] || m_lp0 != r_sp[mk] || r_srv[mk] != g))
                        `ERR(("ep%0d rpc %0d: wrong ports/addresses on delivery", g, mk))
                    mbad = -1;
                    for (mj = 0; mj < mn && mj < MAXM; mj = mj + 1)
                        if (mbad < 0 && mb[mj] !== pbyte(mcli, r_id[mk], mj, m_resp0)) mbad = mj;
                    if (mbad >= 0) `ERR(("ep%0d rpc %0d resp=%0d: byte %0d corrupt", g, mk, m_resp0, mbad))
                    if (m_resp0) begin
                        r_got[mk] = r_got[mk] + 1;
                        if (r_got[mk] > 1) `ERR(("ep%0d rpc %0d: response delivered twice", g, mk))
                        else if (r_st[mk] == 3) `ERR(("ep%0d rpc %0d: response delivered after abort", g, mk))
                        else begin r_st[mk] = 2; n_done = n_done + 1; n_inflight = n_inflight - 1; n_out = n_out - 1; end
                    end else begin
                        r_exec[mk] = r_exec[mk] + 1;
                        if (r_exec[mk] > 1) begin
                            n_reexec = n_reexec + 1;
                            $display("%0t note: rpc %0d (ep%0d->ep%0d) executed again (client state %0d)", $time, mk, r_cli[mk], r_srv[mk], r_st[mk]);
                        end
                        sk[stl % 64] = mk; sh[stl % 64] = m_h0; stt[stl % 64] = now + {$random(seed)} % (SVC + 1);
                        stl = stl + 1;
                    end
                end
                mn = 0;
            end
        end
        r_r <= ({$random(seed)} % 256) >= APPST;
    end

    // aborted client RPCs
    integer xk;
    always @(posedge clk) begin
        if (ab_v && ab_r) begin
            xk = find_rpc(g, ab_id);
            if (xk < 0) `ERR(("ep%0d: abort for unknown id %h", g, ab_id))
            else if (r_st[xk] != 1) `ERR(("ep%0d rpc %0d: abort in state %0d", g, xk, r_st[xk]))
            else begin r_st[xk] = 3; n_abort = n_abort + 1; n_inflight = n_inflight - 1; n_out = n_out - 1; end
        end
        ab_r <= ({$random(seed)} % 4) != 0;
    end
end endgenerate

// run
integer t, e, kk, n_unans;

integer last_prog = 0, last_n = 0;
always @(posedge clk) if (n_done + n_abort != last_n) begin last_n = n_done + n_abort; last_prog = now; end
initial begin
    for (e = 0; e < NE; e = e + 1) issued[e] = 0;
    repeat (10) @(posedge clk);
    rst = 1'b0;
    t = 0;
    while (t < 4000000 && now - last_prog < 300000 && !(issued[0] == NREQ && issued[1] == NREQ && issued[2] == NREQ && n_inflight == 0)) begin
        @(posedge clk); t = t + 1;
    end
    quiet = 1'b1;
    // let ACK / NEED_ACK finish and every slot drain
    repeat ((AGU + 3) * NAK * TICK + (ABR + 3) * RTK * TICK) @(posedge clk);
    n_unans = 0;
    for (kk = 0; kk < nr; kk = kk + 1) begin
        if (r_st[kk] < 2) n_unans = n_unans + 1;
        if (STRICT) begin
            if (r_st[kk] != 2) `ERR(("rpc %0d (len %0d, ep%0d->ep%0d) not answered: state %0d", kk, r_ql[kk], r_cli[kk], r_srv[kk], r_st[kk]))
            if (r_exec[kk] != 1) `ERR(("rpc %0d executed %0d times", kk, r_exec[kk]))
        end else if (r_st[kk] == 2 && r_exec[kk] < 1)
            `ERR(("rpc %0d answered but never executed", kk))
    end
    if (n_unans != 0) `ERR(("%0d RPCs neither answered nor aborted (stuck)", n_unans))
    if (STRICT && n_full != 0) `ERR(("network pool overflowed %0d times", n_full))
    if (ep[0].u.u_core.v !== 0 || ep[1].u.u_core.v !== 0 || ep[2].u.u_core.v !== 0)
        `ERR(("slots still in use after drain: %b %b %b", ep[0].u.u_core.v, ep[1].u.u_core.v, ep[2].u.u_core.v))
    $display("pkts %0d (DATA %0d GRANT %0d RESEND %0d UNKNOWN %0d BUSY %0d CUTOFFS %0d NEED_ACK %0d ACK %0d START %0d) lost %0d dup %0d bad %0d",
             n_pkts, n_type[16], n_type[17], n_type[18], n_type[19], n_type[20], n_type[21], n_type[23], n_type[24], n_type[25],
             n_lost, n_dupd, n_bad);
    $display("DONE seed=%0d loss=%0d dup=%0d bad=%0d: %0d rpcs, %0d answered, %0d aborted, %0d re-executed, %0d slot rejects, %0d cycles, %0d errors -> %0s",
             SEED, LOSS, DUP, BAD, nr, n_done, n_abort, n_reexec, n_rej, now, errors, errors == 0 ? "PASS" : "FAIL");
    $finish;
end

endmodule
