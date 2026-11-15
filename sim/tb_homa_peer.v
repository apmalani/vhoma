`timescale 1ns/1ps
`define ERR(x) begin errors = errors + 1; if (errors <= 60) begin $write("%0t ERROR: ", $time); $display x; end end

// One Homa endpoint driven by a scripted peer that builds and parses packets by
// the draft's byte layout. Checks each packet type and recovery path directly.
module tb_peer;
parameter SEED = 1;
localparam SEG = 64, MAXM = 1024, UNS = 256, RTT = 256, NRPC = 4, RES = 1, K = 2;
localparam TICK = 32, RTK = 4, ABR = 3, NAK = 4, AGU = 5, SLV = 2;
localparam SL = 2;
localparam [31:0] PEER = 32'h0A000063, PEER2 = 32'h0A000064;
localparam [15:0] DVER = 16'h0100;
localparam [7:0] T_DATA = 16, T_GRANT = 17, T_RESEND = 18, T_UNKNOWN = 19, T_BUSY = 20,
                 T_CUTOFFS = 21, T_NEED_ACK = 23, T_ACK = 24, T_START = 25;

reg clk = 1'b0;
always #5 clk = ~clk;
reg rst = 1'b1;
integer errors = 0;
integer seed = SEED;

localparam [255:0] DCUT = {32'd1, 32'd16, 32'd64, 32'd128, 32'd256, 32'd1024, 32'd1024, 32'd1024};

reg  [63:0] in_d = 0; reg [7:0] in_k = 0; reg in_v = 0, in_l = 0, in_u = 0; reg [31:0] in_sa = 0;
wire [63:0] o_d; wire [7:0] o_k; wire o_v, o_l; reg o_r = 1;
wire [31:0] o_da; wire [2:0] o_p; wire [15:0] o_len;
reg  a_v = 0, a_resp = 0; wire a_r;
reg  [31:0] a_addr = 0, a_len = 0; reg [15:0] a_rp = 0, a_lp = 0; reg [SL+7:0] a_h = 0;
reg  [63:0] a_d = 0; reg a_dv = 0; wire a_dr;
wire st_v, st_ok, st_resp; wire [63:0] st_id; wire [SL+7:0] st_h;
wire [63:0] r_d; wire [7:0] r_k; wire r_v, r_l; reg r_r = 1;
wire r_resp; wire [31:0] r_addr, r_len; wire [15:0] r_rp, r_lp; wire [63:0] r_rid; wire [SL+7:0] r_h;
wire ab_v; wire [63:0] ab_id; reg ab_r = 1;
wire drop;
integer ndrop = 0;
always @(posedge clk) if (drop) ndrop = ndrop + 1;

homa_endpoint #(
    .SEG_BYTES(SEG), .MAX_MSG(MAXM), .UNSCHED_MAX(UNS), .RTT_BYTES(RTT), .NRPC(NRPC), .SRV_RESERVE(RES),
    .OVERCOMMIT(K), .NPEER(2), .TICK(TICK), .RESEND_TICKS(RTK), .ABORT_RESENDS(ABR),
    .NEEDACK_TICKS(NAK), .ACK_GIVEUP(AGU), .EVQ(4), .SLQ(4), .CUT_COOL(16)
) dut (
    .clk(clk), .rst(rst), .cfg_cutoffs(DCUT), .cfg_cver(DVER), .cfg_sched_lvls(SLV[3:0]), .cfg_id_base(64'd0),
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

// packets out of the DUT
localparam QN = 256, QB = 128;
reg  [7:0]  oq   [0:QN*QB-1];
integer     oq_n [0:QN-1];
reg  [31:0] oq_a [0:QN-1];
reg  [2:0]  oq_p [0:QN-1];
reg         oq_v [0:QN-1];
integer     oq_cnt = 0, cn = 0, cj, oz;
initial for (oz = 0; oz < QN; oz = oz + 1) oq_v[oz] = 0;
always @(posedge clk) if (o_v && o_r) begin
    for (cj = 0; cj < 8; cj = cj + 1)
        if (o_k[cj]) begin if (cn < QB) oq[(oq_cnt % QN)*QB + cn] = o_d[8*cj +: 8]; cn = cn + 1; end
    if (o_l) begin
        if (cn != o_len) `ERR(("output packet %0d bytes, sideband %0d", cn, o_len))
        oq_n[oq_cnt % QN] = cn; oq_a[oq_cnt % QN] = o_da; oq_p[oq_cnt % QN] = o_p; oq_v[oq_cnt % QN] = 1;
        oq_cnt = oq_cnt + 1; cn = 0;
    end
end
function [7:0]  ob(input integer q, input integer o); ob = oq[q*QB + o]; endfunction
function [15:0] o16(input integer q, input integer o); o16 = {oq[q*QB+o], oq[q*QB+o+1]}; endfunction
function [31:0] o32(input integer q, input integer o); o32 = {o16(q, o), o16(q, o+2)}; endfunction
function [63:0] o64(input integer q, input integer o); o64 = {o32(q, o), o32(q, o+4)}; endfunction

// wait for the next unconsumed packet of a type (others stay queued); returns slot or -1
task wait_pkt(input [7:0] ty, input integer tmo, output integer q);
    integer t, z;
    begin
        q = -1;
        for (t = 0; t < tmo && q < 0; t = t + 1) begin
            for (z = 0; z < QN; z = z + 1)
                if (q < 0 && oq_v[z] && oq[z*QB + 11] == ty) q = z;
            if (q < 0) @(posedge clk);
        end
        if (q >= 0) oq_v[q] = 0;
    end
endtask
// wait for a GRANT for wid with exactly this offset (repeated GRANTs for older offsets are fine)
task wait_grant(input [63:0] wid, input [31:0] off, input integer tmo, output integer q);
    integer t, z;
    begin
        q = -1;
        for (t = 0; t < tmo && q < 0; t = t + 1) begin
            for (z = 0; z < QN; z = z + 1)
                if (q < 0 && oq_v[z] && oq[z*QB + 11] == T_GRANT && o64(z, 20) === wid && o32(z, 28) == off) q = z;
            if (q < 0) @(posedge clk);
        end
        if (q >= 0) oq_v[q] = 0;
    end
endtask
task flush_out;
    integer z;
    for (z = 0; z < QN; z = z + 1) oq_v[z] = 0;
endtask
function integer count_q(input [7:0] ty);
    integer z;
    begin
        count_q = 0;
        for (z = 0; z < QN; z = z + 1) if (oq_v[z] && oq[z*QB + 11] == ty) count_q = count_q + 1;
    end
endfunction

// packets into the DUT
reg [7:0] px [0:2047];
integer   pn;
task p_hdr(input [7:0] ty, input [15:0] sp, input [15:0] dp, input [63:0] id);
    integer j;
    begin
        for (j = 0; j < 80; j = j + 1) px[j] = 0;
        px[0] = sp[15:8]; px[1] = sp[7:0]; px[2] = dp[15:8]; px[3] = dp[7:0]; px[11] = ty;
        for (j = 0; j < 8; j = j + 1) px[20 + j] = id[63 - 8*j -: 8];
    end
endtask
task p32(input integer o, input [31:0] x); begin px[o] = x[31:24]; px[o+1] = x[23:16]; px[o+2] = x[15:8]; px[o+3] = x[7:0]; end endtask
task p16(input integer o, input [15:0] x); begin px[o] = x[15:8]; px[o+1] = x[7:0]; end endtask
task p64(input integer o, input [63:0] x); begin p32(o, x[63:32]); p32(o + 4, x[31:0]); end endtask
task send(input [31:0] from, input integer n);
    integer o, b;
    begin
        @(negedge clk);
        for (o = 0; o < n; o = o + 8) begin
            for (b = 0; b < 8; b = b + 1) begin
                in_d[8*b +: 8] = (o + b < n) ? px[o + b] : 8'h00;
                in_k[b] = (o + b < n);
            end
            in_v = 1; in_l = (o + 8 >= n); in_u = 0; in_sa = from;
            @(negedge clk);
        end
        in_v = 0; in_l = 0;
    end
endtask
function [7:0] pat(input [63:0] id, input integer j); pat = id[7:0] * 8'd31 + j * 8'd7 + (j >> 8); endfunction
task build_data(input [15:0] sp, input [15:0] dp, input [63:0] id, input [31:0] mlen,
                input [31:0] off, input [63:0] ackid, input [15:0] cver, input integer kind);
    integer j, pl;
    begin
        pl = (mlen - off > SEG) ? SEG : mlen - off;
        p_hdr(T_DATA, sp, dp, id);
        p32(28, mlen); p64(32, ackid); p16(42, cver); p32(48, off);
        for (j = 0; j < pl; j = j + 1) px[52 + j] = pat(id ^ kind, off + j);
        pn = 52 + pl;
    end
endtask
task send_data(input [31:0] from, input [15:0] sp, input [15:0] dp, input [63:0] id, input [31:0] mlen,
               input [31:0] off, input [63:0] ackid, input [15:0] cver, input integer kind);
    begin
        build_data(sp, dp, id, mlen, off, ackid, cver, kind);
        send(from, pn);
    end
endtask
task invert(input integer a, input integer b);   // flip bytes [a, b)
    integer j;
    for (j = a; j < b; j = j + 1) px[j] = ~px[j];
endtask

// application side
reg  [7:0]  db [0:MAXM-1];
integer     d_n = 0, d_cnt = 0, dj;
reg  [31:0] d_addr, d_len; reg [63:0] d_id; reg d_resp; reg [15:0] d_rp, d_lp; reg [SL+7:0] d_h;
// every delivered message is also kept here, so overlapping deliveries can be checked later
localparam LN = 16;
reg  [7:0]  lb [0:LN*MAXM-1];
reg  [63:0] l_id [0:LN-1]; reg l_resp [0:LN-1]; reg [31:0] l_addr [0:LN-1], l_len [0:LN-1];
reg  [15:0] l_rp [0:LN-1], l_lp [0:LN-1]; reg [SL+7:0] l_h [0:LN-1];
always @(posedge clk) if (r_v && r_r) begin
    if (d_n == 0) begin d_addr = r_addr; d_len = r_len; d_id = r_rid; d_resp = r_resp; d_rp = r_rp; d_lp = r_lp; d_h = r_h; end
    for (dj = 0; dj < 8; dj = dj + 1) if (r_k[dj]) begin
        if (d_n < MAXM) begin db[d_n] = r_d[8*dj +: 8]; lb[(d_cnt % LN)*MAXM + d_n] = r_d[8*dj +: 8]; end
        d_n = d_n + 1;
    end
    if (r_l) begin
        l_id[d_cnt % LN] = d_id; l_resp[d_cnt % LN] = d_resp; l_addr[d_cnt % LN] = d_addr; l_len[d_cnt % LN] = d_len;
        l_rp[d_cnt % LN] = d_rp; l_lp[d_cnt % LN] = d_lp; l_h[d_cnt % LN] = d_h;
        d_cnt = d_cnt + 1; d_n = 0;
    end
end
function integer find_d(input [63:0] id);
    integer z;
    begin
        find_d = -1;
        for (z = 0; z < LN && z < d_cnt; z = z + 1) if (l_id[(d_cnt - 1 - z) % LN] === id && find_d < 0) find_d = (d_cnt - 1 - z) % LN;
    end
endfunction
task wait_count(input integer c, input integer tmo);
    integer t;
    begin
        for (t = 0; t < tmo && d_cnt < c; t = t + 1) @(posedge clk);
        if (d_cnt < c) `ERR(("only %0d of %0d deliveries within %0d cycles", d_cnt, c, tmo))
    end
endtask
// check a logged message by id
task check_log(input r, input [31:0] addr, input [15:0] rp, input [15:0] lp, input [63:0] id, input [31:0] len, input integer kind);
    integer x, j, bad;
    begin
        x = find_d(id);
        if (x < 0) `ERR(("message %h was not delivered", id))
        else begin
            if (l_resp[x] !== r || l_addr[x] !== addr || l_rp[x] !== rp || l_lp[x] !== lp || l_len[x] !== len)
                `ERR(("delivery %h: resp=%0d addr=%h ports=%h/%h len=%0d", id, l_resp[x], l_addr[x], l_rp[x], l_lp[x], l_len[x]))
            bad = -1;
            for (j = 0; j < len; j = j + 1) if (bad < 0 && lb[x*MAXM + j] !== pat(id ^ kind, j)) bad = j;
            if (bad >= 0) `ERR(("delivery %h: byte %0d is %h want %h", id, bad, lb[x*MAXM + bad], pat(id ^ kind, bad)))
        end
    end
endtask
task wait_deliv(input integer c0, input integer tmo);
    integer t;
    begin
        for (t = 0; t < tmo && d_cnt == c0; t = t + 1) @(posedge clk);
        if (d_cnt == c0) `ERR(("no delivery within %0d cycles", tmo))
    end
endtask
task check_deliv(input r, input [31:0] addr, input [15:0] rp, input [15:0] lp, input [63:0] id, input [31:0] len, input integer kind);
    integer j, bad;
    begin
        if (d_resp !== r || d_addr !== addr || d_rp !== rp || d_lp !== lp || d_id !== id || d_len !== len)
            `ERR(("delivery: resp=%0d addr=%h ports=%h/%h id=%h len=%0d", d_resp, d_addr, d_rp, d_lp, d_id, d_len))
        bad = -1;
        for (j = 0; j < len; j = j + 1) if (bad < 0 && db[j] !== pat(id ^ kind, j)) bad = j;
        if (bad >= 0) `ERR(("delivery id=%h: byte %0d is %h want %h", id, bad, db[bad], pat(id ^ kind, bad)))
    end
endtask

reg [63:0] last_id;
reg        last_ok;
reg [SL+7:0] last_h;
task app_send(input r, input [31:0] addr, input [15:0] rp, input [15:0] lp, input [SL+7:0] h, input [31:0] len, input [63:0] pid);
    integer w, b;
    begin
        @(negedge clk);
        a_v = 1; a_resp = r; a_addr = addr; a_rp = rp; a_lp = lp; a_h = h; a_len = len;
        @(posedge clk); while (!a_r) @(posedge clk);
        @(negedge clk); a_v = 0;
        while (!st_v) @(posedge clk);
        last_ok = st_ok; last_id = st_id; last_h = st_h;
        @(negedge clk);
        for (w = 0; w < (len + 7) / 8; w = w + 1) begin
            for (b = 0; b < 8; b = b + 1) a_d[8*b +: 8] = pat((r ? pid : st_id) ^ (r ? 1 : 2), 8*w + b);
            a_dv = 1;
            @(posedge clk); while (!a_dr) @(posedge clk);
            @(negedge clk);
        end
        a_dv = 0;
    end
endtask

// check a DATA packet from the DUT: fields and payload
task check_data(input integer q, input [31:0] to, input [15:0] sp, input [15:0] dp, input [63:0] wid,
                input [31:0] mlen, input [31:0] off, input retr, input [2:0] pri, input [15:0] cver, input [63:0] pid, input integer kind);
    integer j, pl, bad;
    begin
        pl = (mlen - off > SEG) ? SEG : mlen - off;
        if (oq_a[q] !== to || o16(q, 0) !== sp || o16(q, 2) !== dp || o64(q, 20) !== wid || o32(q, 28) !== mlen ||
            o32(q, 48) !== off || ob(q, 44) !== retr || oq_p[q] !== pri || o16(q, 42) !== cver || oq_n[q] != 52 + pl)
            `ERR(("DATA: to=%h ports=%h/%h id=%h len=%0d off=%0d retr=%0d pri=%0d cver=%h n=%0d; want %h %h/%h %h %0d %0d %0d %0d %h %0d",
                  oq_a[q], o16(q,0), o16(q,2), o64(q,20), o32(q,28), o32(q,48), ob(q,44), oq_p[q], o16(q,42), oq_n[q],
                  to, sp, dp, wid, mlen, off, retr, pri, cver, 52 + pl))
        bad = -1;
        for (j = 0; j < pl; j = j + 1) if (bad < 0 && ob(q, 52 + j) !== pat(pid ^ kind, off + j)) bad = j;
        if (bad >= 0) `ERR(("DATA off %0d: payload byte %0d wrong", off, bad))
    end
endtask

task expect_none(input [7:0] ty, input integer cycles);
    begin
        repeat (cycles) @(posedge clk);
        if (count_q(ty) != 0) `ERR(("unexpected packet type %0d (%0d queued)", ty, count_q(ty)))
    end
endtask

function integer nslots(input dummy);
    integer z;
    begin nslots = 0; for (z = 0; z < NRPC; z = z + 1) nslots = nslots + dut.u_core.v[z]; end
endfunction

task section(input [8*40-1:0] name);
    $display("%0t  %0s  (errors so far %0d)", $time, name, errors);
endtask

integer q, q2, i, j, k, n0, off, g, base, f;
reg [63:0] cid;
reg [SL+7:0] h;
initial begin
    repeat (5) @(posedge clk);
    rst = 0;
    repeat (5) @(posedge clk);

    // 1: server, unscheduled request -> response -> RESEND -> NEED_ACK -> ACK frees the slot
    section("1 server unscheduled");
    n0 = d_cnt;
    send_data(PEER, 16'h1111, 16'h2222, 64'h100, 10, 0, 0, DVER, 2);
    wait_deliv(n0, 2000);
    check_deliv(0, PEER, 16'h1111, 16'h2222, 64'h100, 10, 2);
    h = d_h;
    app_send(1, 0, 0, 0, h, 20, 64'h100);
    if (!last_ok || last_id !== 64'h100) `ERR(("response send status ok=%0d id=%h", last_ok, last_id))
    wait_pkt(T_DATA, 2000, q);
    if (q < 0) `ERR(("no response DATA")) else check_data(q, PEER, 16'h2222, 16'h1111, 64'h101, 20, 0, 0, 7, 0, 64'h100, 1);
    p_hdr(T_RESEND, 16'h1111, 16'h2222, 64'h100); p32(28, 0); p32(32, 20); px[36] = 5; send(PEER, 37);
    wait_pkt(T_DATA, 2000, q);
    if (q < 0) `ERR(("no retransmission")) else check_data(q, PEER, 16'h2222, 16'h1111, 64'h101, 20, 0, 1, 5, 0, 64'h100, 1);
    wait_pkt(T_NEED_ACK, (NAK + 3) * TICK * 2, q);
    if (q < 0) `ERR(("no NEED_ACK"))
    else if (o64(q, 20) !== 64'h101 || oq_n[q] != 28 || oq_p[q] != 7) `ERR(("NEED_ACK id=%h n=%0d", o64(q, 20), oq_n[q]))
    p_hdr(T_ACK, 16'h1111, 16'h2222, 64'h100); p16(28, 0); send(PEER, 30);
    repeat (20) @(posedge clk);
    if (nslots(0) != 0) `ERR(("slot not freed by ACK"))
    flush_out;

    // 2: server, scheduled request: START_MSG -> GRANTs -> DATA within grants
    section("2 server scheduled");
    n0 = d_cnt;
    p_hdr(T_START, 16'h1111, 16'h2222, 64'h200); p32(28, 600); send(PEER, 32);
    g = 0; off = 0;
    for (i = 0; i < 60 && off < 600; i = i + 1) begin
        wait_pkt(T_GRANT, 3000, q);
        if (q < 0) begin `ERR(("no GRANT at offset %0d", off)) i = 99; end
        else begin
            if (o64(q, 20) !== 64'h201 || oq_n[q] != 33 || oq_p[q] != 7 || ob(q, 32) >= SLV)
                `ERR(("GRANT id=%h n=%0d pri=%0d field=%0d", o64(q, 20), oq_n[q], oq_p[q], ob(q, 32)))
            if (o32(q, 28) < g || o32(q, 28) > 600) `ERR(("GRANT offset %0d after %0d", o32(q, 28), g))
            if (o32(q, 28) > g) g = o32(q, 28);
            if (i == 2) begin   // at a quiet point, a duplicate START_MSG is answered with the current grant
                repeat (300) @(posedge clk);
                q2 = 0;
                while (q2 >= 0) begin
                    wait_pkt(T_GRANT, 1, q2);
                    if (q2 >= 0 && o32(q2, 28) > g) g = o32(q2, 28);
                end
                p_hdr(T_START, 16'h1111, 16'h2222, 64'h200); p32(28, 600); send(PEER, 32);
                wait_pkt(T_GRANT, 3000, q2);
                if (q2 < 0 || o32(q2, 28) != g) `ERR(("duplicate START_MSG: regrant %0d, want %0d", (q2 < 0) ? -1 : o32(q2, 28), g))
            end
            while (off < g) begin send_data(PEER, 16'h1111, 16'h2222, 64'h200, 600, off, 0, DVER, 2); off = off + SEG; end
        end
    end
    wait_deliv(n0, 3000);
    check_deliv(0, PEER, 16'h1111, 16'h2222, 64'h200, 600, 2);
    app_send(1, 0, 0, 0, d_h, 8, 64'h200);
    wait_pkt(T_DATA, 2000, q);
    if (q < 0) `ERR(("no response")) else check_data(q, PEER, 16'h2222, 16'h1111, 64'h201, 8, 0, 0, 7, 0, 64'h200, 1);
    p_hdr(T_ACK, 16'h1111, 16'h2222, 64'h200); send(PEER, 30);
    repeat (20) @(posedge clk);
    if (nslots(0) != 0) `ERR(("slot not freed after scheduled RPC"))
    flush_out;

    // 3: replies without an RPC: RESEND -> RPC_UNKNOWN, NEED_ACK -> ACK
    section("3 unknown RPCs");
    p_hdr(T_RESEND, 16'h3333, 16'h4444, 64'h300); p32(28, 0); p32(32, 32'hFFFFFFFF); send(PEER, 37);
    wait_pkt(T_UNKNOWN, 500, q);
    if (q < 0) `ERR(("no RPC_UNKNOWN"))
    else if (o64(q, 20) !== 64'h301 || o16(q, 0) !== 16'h4444 || o16(q, 2) !== 16'h3333 || oq_a[q] !== PEER || oq_n[q] != 28)
        `ERR(("RPC_UNKNOWN id=%h ports %h/%h", o64(q, 20), o16(q, 0), o16(q, 2)))
    p_hdr(T_NEED_ACK, 16'h5555, 16'h6666, 64'h401); send(PEER, 28);
    wait_pkt(T_ACK, 500, q);
    if (q < 0) `ERR(("no ACK for unknown NEED_ACK"))
    else if (o64(q, 20) !== 64'h400 || o16(q, 0) !== 16'h6666 || o16(q, 2) !== 16'h5555 || oq_n[q] != 30 || o16(q, 28) != 0)
        `ERR(("ACK id=%h ports %h/%h", o64(q, 20), o16(q, 0), o16(q, 2)))
    if (nslots(0) != 0) `ERR(("slot allocated by unknown-RPC packets"))
    flush_out;

    // 4: CUTOFFS set our unscheduled priority; stale version in DATA gets CUTOFFS back
    section("4 cutoffs");
    p_hdr(T_CUTOFFS, 16'h0, 16'h0, 64'h0);
    for (i = 0; i < 8; i = i + 1) p32(28 + 4*i, 1024 >> i);
    p16(60, 16'h0777); send(PEER, 62);
    repeat (10) @(posedge clk);
    app_send(0, PEER, 16'h0080, 16'h4001, 0, 100, 0);
    cid = last_id;
    if (!last_ok || cid[0] || cid == 0) `ERR(("request status ok=%0d id=%h", last_ok, cid))
    wait_pkt(T_DATA, 2000, q);
    // largest i with 1024>>i >= 100 is 3
    if (q < 0) `ERR(("no request DATA")) else check_data(q, PEER, 16'h4001, 16'h0080, cid, 100, 0, 0, 3, 16'h0777, cid, 2);
    n0 = d_cnt;
    send_data(PEER, 16'h0080, 16'h4001, cid | 1, 40, 0, 0, 16'h0001, 1);
    wait_pkt(T_CUTOFFS, 500, q);
    if (q < 0) `ERR(("no CUTOFFS for stale version"))
    else begin
        for (i = 0; i < 8; i = i + 1) if (o32(q, 28 + 4*i) !== ((DCUT >> (32*i)) & 256'hFFFFFFFF)) `ERR(("CUTOFFS[%0d]=%0d", i, o32(q, 28 + 4*i)))
        if (o16(q, 60) !== DVER || oq_n[q] != 62 || oq_a[q] !== PEER) `ERR(("CUTOFFS version %h n=%0d", o16(q, 60), oq_n[q]))
    end
    wait_deliv(n0, 2000);
    check_deliv(1, PEER, 16'h0080, 16'h4001, cid, 40, 0);
    wait_pkt(T_ACK, 500, q);
    if (q < 0 || o64(q, 20) !== cid) `ERR(("client did not ACK the response"))
    repeat (20) @(posedge clk);
    if (nslots(0) != 0) `ERR(("client slot not freed"))
    flush_out;

    // 5: silent server: RESEND(-1) every interval, then abort; BUSY keeps it alive
    section("5 client timeout");
    app_send(0, PEER2, 16'h0081, 16'h4002, 0, 30, 0);
    cid = last_id;
    wait_pkt(T_DATA, 2000, q);
    for (i = 0; i < ABR; i = i + 1) begin
        wait_pkt(T_RESEND, (RTK + 3) * TICK * 2, q);
        if (q < 0) `ERR(("RESEND %0d missing", i))
        else if (o64(q, 20) !== cid || o32(q, 28) !== 0 || o32(q, 32) !== 32'hFFFFFFFF || oq_a[q] !== PEER2)
            `ERR(("RESEND id=%h off=%0d len=%h", o64(q, 20), o32(q, 28), o32(q, 32)))
    end
    for (i = 0; i < (RTK + 3) * TICK * 3 && !(ab_v && ab_r); i = i + 1) @(posedge clk);
    if (!(ab_v && ab_id === cid)) `ERR(("no abort for silent server (id %h)", cid))
    repeat (20) @(posedge clk);
    if (nslots(0) != 0) `ERR(("aborted slot not freed"))
    flush_out;
    app_send(0, PEER2, 16'h0081, 16'h4002, 0, 30, 0);
    cid = last_id;
    for (i = 0; i < 4 * ABR; i = i + 1) begin
        wait_pkt(T_RESEND, (RTK + 3) * TICK * 2, q);
        if (q < 0) begin `ERR(("RESEND missing while BUSY")) i = 99; end
        p_hdr(T_BUSY, 16'h0081, 16'h4002, cid | 1); send(PEER2, 28);
        if (ab_v) `ERR(("aborted although the server answers BUSY"))
    end
    n0 = d_cnt;
    send_data(PEER2, 16'h0081, 16'h4002, cid | 1, 5, 0, 0, DVER, 1);
    wait_deliv(n0, 2000);
    check_deliv(1, PEER2, 16'h0081, 16'h4002, cid, 5, 0);
    repeat (20) @(posedge clk);
    flush_out;

    // 6: piggybacked ACK in DATA, and an ACK carrying extra acks
    section("6 acks");
    for (i = 0; i < 3; i = i + 1) begin
        n0 = d_cnt;
        send_data(PEER, 16'h1111, 16'h2222, 64'h500 + 2*i, 3, 0, 0, DVER, 2);
        wait_deliv(n0, 2000);
        app_send(1, 0, 0, 0, d_h, 3, 64'h500 + 2*i);
    end
    repeat (50) @(posedge clk);
    if (nslots(0) != 3) `ERR(("expected 3 server slots, have %0d", nslots(0)))
    n0 = d_cnt;
    send_data(PEER, 16'h1111, 16'h2222, 64'h506, 3, 0, 64'h500, DVER, 2);   // acks 0x500
    wait_deliv(n0, 2000);
    repeat (20) @(posedge clk);
    if (nslots(0) != 3) `ERR(("piggybacked ACK did not free 0x500 (%0d slots)", nslots(0)))
    p_hdr(T_ACK, 16'h1111, 16'h2222, 64'h502); p16(28, 1); p64(30, 64'h504); p16(38, 16'h2222); send(PEER, 40);
    repeat (20) @(posedge clk);
    if (nslots(0) != 1) `ERR(("ACK with extra ack left %0d slots", nslots(0)))
    app_send(1, 0, 0, 0, d_h, 3, 64'h506);
    p_hdr(T_ACK, 16'h1111, 16'h2222, 64'h506); send(PEER, 30);
    repeat (20) @(posedge clk);
    if (nslots(0) != 0) `ERR(("slots left after acks: %0d", nslots(0)))
    flush_out;

    // 7: scheduled request; RPC_UNKNOWN makes the client start over
    section("7 rpc_unknown restart");
    app_send(0, PEER, 16'h0082, 16'h4003, 0, 400, 0);
    cid = last_id;
    wait_pkt(T_START, 2000, q);
    if (q < 0 || o32(q, 28) != 400 || o64(q, 20) !== cid) `ERR(("no START_MSG"))
    p_hdr(T_GRANT, 16'h0082, 16'h4003, cid | 1); p32(28, 128); px[32] = 1; send(PEER, 33);
    for (i = 0; i < 2; i = i + 1) begin
        wait_pkt(T_DATA, 2000, q);
        if (q < 0) `ERR(("granted DATA %0d missing", i)) else check_data(q, PEER, 16'h4003, 16'h0082, cid, 400, i*SEG, 0, 1, 16'h0777, cid, 2);
    end
    expect_none(T_DATA, 200);
    p_hdr(T_UNKNOWN, 16'h0082, 16'h4003, cid | 1); send(PEER, 28);
    wait_pkt(T_START, 2000, q);
    if (q < 0) `ERR(("no START_MSG after RPC_UNKNOWN"))
    p_hdr(T_GRANT, 16'h0082, 16'h4003, cid | 1); p32(28, 400); px[32] = 0; send(PEER, 33);
    for (i = 0; i < 7; i = i + 1) begin
        wait_pkt(T_DATA, 2000, q);
        if (q < 0) `ERR(("DATA %0d after restart missing", i)) else check_data(q, PEER, 16'h4003, 16'h0082, cid, 400, i*SEG, 0, 0, 16'h0777, cid, 2);
    end
    n0 = d_cnt;
    send_data(PEER, 16'h0082, 16'h4003, cid | 1, 1, 0, 0, DVER, 1);
    wait_deliv(n0, 2000);
    repeat (20) @(posedge clk);
    flush_out;

    // 8: a lost GRANT: the receiver's RESEND for ungranted bytes releases them
    section("8 lost grant");
    app_send(0, PEER, 16'h0083, 16'h4004, 0, 300, 0);
    cid = last_id;
    wait_pkt(T_START, 2000, q);
    p_hdr(T_GRANT, 16'h0083, 16'h4004, cid | 1); p32(28, 256); px[32] = 1; send(PEER, 33);
    for (i = 0; i < 4; i = i + 1) wait_pkt(T_DATA, 2000, q);
    expect_none(T_DATA, 100);
    p_hdr(T_RESEND, 16'h0083, 16'h4004, cid | 1); p32(28, 256); p32(32, 44); px[36] = 1; send(PEER, 37);
    wait_pkt(T_DATA, 2000, q);
    if (q < 0) `ERR(("RESEND did not release ungranted bytes")) else check_data(q, PEER, 16'h4004, 16'h0083, cid, 300, 256, 1, 1, 16'h0777, cid, 2);
    // a GRANT past the end of the message must not stop the client asking for its response
    p_hdr(T_GRANT, 16'h0083, 16'h4004, cid | 1); p32(28, 5000); px[32] = 1; send(PEER, 33);
    flush_out;
    wait_pkt(T_RESEND, (RTK + 3) * TICK * 2, q);
    if (q < 0 || o64(q, 20) !== cid) `ERR(("no RESEND for the response after an oversized GRANT"))
    n0 = d_cnt;
    send_data(PEER, 16'h0083, 16'h4004, cid | 1, 1, 0, 0, DVER, 1);
    wait_deliv(n0, 2000);
    repeat (20) @(posedge clk);
    flush_out;

    // 9: client slots stop at NRPC - SRV_RESERVE; the reserved slot still takes requests
    section("9 slot reserve");
    for (i = 0; i < NRPC - RES; i = i + 1) begin
        app_send(0, PEER2, 16'h0090, 16'h4010, 0, 4, 0);
        if (!last_ok) `ERR(("request %0d refused", i));
    end
    app_send(0, PEER2, 16'h0090, 16'h4010, 0, 4, 0);
    if (last_ok) `ERR(("request accepted beyond the client limit"))
    n0 = d_cnt;
    send_data(PEER, 16'h1111, 16'h2222, 64'h700, 3, 0, 0, DVER, 2);
    wait_deliv(n0, 2000);
    app_send(1, 0, 0, 0, d_h, 3, 64'h700);
    p_hdr(T_ACK, 16'h1111, 16'h2222, 64'h700); send(PEER, 30);
    // let the silent-server requests abort
    for (i = 0; i < (ABR + 2) * (RTK + 2) * TICK * 2; i = i + 1) @(posedge clk);
    if (nslots(0) != 0) `ERR(("slots left after reserve test: %0d", nslots(0)))
    flush_out;

    // 11: malicious DATA for a request in progress: dropped, never corrupts it
    section("11 bad data");
    n0 = d_cnt; base = ndrop; k = 0;
    send_data(PEER, 16'h1111, 16'h2222, 64'h1100, 200, 64, 0, DVER, 2);          // seg 1 of A
    send_data(PEER, 16'h1111, 16'h2222, 64'h1102, 100, 0, 0, DVER, 2);           // B, seg 0 (next slot)
    build_data(16'h1111, 16'h2222, 64'h1100, 200, 0, 0, DVER, 2); invert(52, pn); pn = pn - 1; send(PEER, pn); k = k + 1;  // short
    build_data(16'h1111, 16'h2222, 64'h1100, 200, 0, 0, DVER, 2); for (j = pn; j < pn + 64; j = j + 1) px[j] = 8'h55;
        pn = pn + 64; send(PEER, pn); k = k + 1;                                  // long: must not spill into seg 1
    build_data(16'h9999, 16'h2222, 64'h1100, 200, 0, 0, DVER, 2); invert(52, pn); send(PEER, pn); k = k + 1;  // wrong client port
    build_data(16'h1111, 16'h9999, 64'h1100, 200, 0, 0, DVER, 2); invert(52, pn); send(PEER, pn); k = k + 1;  // wrong server port
    build_data(16'h1111, 16'h2222, 64'h1100, 200, 0, 0, DVER, 2); invert(52, pn); p32(48, MAXM); send(PEER, pn); k = k + 1;  // offset into next slot
    build_data(16'h1111, 16'h2222, 64'h1100, 200, 0, 0, DVER, 2); invert(52, pn); p32(48, 256); send(PEER, pn); k = k + 1;   // offset past end
    build_data(16'h1111, 16'h2222, 64'h1100, 200, 64, 0, DVER, 2); invert(52, pn); send(PEER, pn); k = k + 1;  // duplicate of seg 1
    repeat (10) @(posedge clk);
    if (ndrop - base != k) `ERR(("bad DATA: %0d drops, want %0d", ndrop - base, k))
    // a gap at seg 2 of a 5-segment request: RESEND asks for exactly that segment
    send_data(PEER, 16'h1111, 16'h2222, 64'h1104, 300, 0, 0, DVER, 2);
    send_data(PEER, 16'h1111, 16'h2222, 64'h1104, 300, 64, 0, DVER, 2);
    send_data(PEER, 16'h1111, 16'h2222, 64'h1104, 300, 192, 0, DVER, 2);
    send_data(PEER, 16'h1111, 16'h2222, 64'h1104, 300, 256, 0, DVER, 2);
    wait_pkt(T_RESEND, (RTK + 3) * TICK * 2, q);
    while (q >= 0 && o64(q, 20) !== 64'h1105) wait_pkt(T_RESEND, (RTK + 3) * TICK * 2, q);
    if (q < 0 || o32(q, 28) != 128 || o32(q, 32) != 64) `ERR(("RESEND for the gap: off=%0d len=%0d", (q < 0) ? -1 : o32(q, 28), (q < 0) ? -1 : o32(q, 32)))
    send_data(PEER, 16'h1111, 16'h2222, 64'h1104, 300, 128, 0, DVER, 2);
    send_data(PEER, 16'h1111, 16'h2222, 64'h1100, 200, 0, 0, DVER, 2);
    send_data(PEER, 16'h1111, 16'h2222, 64'h1100, 200, 128, 0, DVER, 2);
    send_data(PEER, 16'h1111, 16'h2222, 64'h1100, 200, 192, 0, DVER, 2);
    send_data(PEER, 16'h1111, 16'h2222, 64'h1102, 100, 64, 0, DVER, 2);
    wait_count(n0 + 3, 4000);
    check_log(0, PEER, 16'h1111, 16'h2222, 64'h1100, 200, 2);
    check_log(0, PEER, 16'h1111, 16'h2222, 64'h1102, 100, 2);
    check_log(0, PEER, 16'h1111, 16'h2222, 64'h1104, 300, 2);
    for (i = 0; i < 3; i = i + 1) begin
        app_send(1, 0, 0, 0, l_h[find_d(64'h1100 + 2*i)], 1, 64'h1100 + 2*i);
        p_hdr(T_ACK, 16'h1111, 16'h2222, 64'h1100 + 2*i); send(PEER, 30);
    end
    repeat (50) @(posedge clk);
    if (nslots(0) != 0) `ERR(("slots left after bad data: %0d", nslots(0)))
    flush_out;

    // 12: grants: SRPT, one message per peer, OVERCOMMIT, window and priorities
    section("12 grant policy");
    p_hdr(T_START, 16'h1111, 16'h2222, 64'h1202); p32(28, 400); send(PEER, 32);
    p_hdr(T_START, 16'h1111, 16'h2222, 64'h1200); p32(28, 900); send(PEER, 32);
    repeat (100) @(posedge clk);
    // one per peer: only the shorter one from PEER; one message active, so lowest level (0)
    wait_pkt(T_GRANT, 500, q);
    if (q < 0 || o64(q, 20) !== 64'h1203 || o32(q, 28) != 256 || ob(q, 32) != 0)
        `ERR(("first GRANT id=%h off=%0d pri=%0d, want 1203 256 0", (q < 0) ? 0 : o64(q, 20), (q < 0) ? -1 : o32(q, 28), (q < 0) ? -1 : ob(q, 32)))
    if (count_q(T_GRANT) != 0) `ERR(("granted a second message from the same peer"))
    flush_out;
    p_hdr(T_START, 16'h1111, 16'h2222, 64'h1204); p32(28, 500); send(PEER2, 32);
    repeat (100) @(posedge clk);
    wait_grant(64'h1205, 256, 500, q);
    // two active (OVERCOMMIT=2, 2 levels): the 500-byte one ranks below the 300-byte one -> level 0
    if (q < 0 || ob(q, 32) != 0) `ERR(("GRANT for second peer wrong"))
    send_data(PEER, 16'h1111, 16'h2222, 64'h1202, 400, 0, 0, DVER, 2);
    wait_grant(64'h1203, 320, 500, q);
    if (q < 0 || ob(q, 32) != 1) `ERR(("GRANT after data: pri=%0d, want offset 320 at level 1", (q < 0) ? -1 : ob(q, 32)))
    // a third, shortest message from a third peer takes an active place; the 500-byte one stops
    p_hdr(T_START, 16'h1111, 16'h2222, 64'h1206); p32(28, 280); send(32'h0A000065, 32);
    wait_grant(64'h1207, 256, 500, q);
    if (q < 0) `ERR(("third peer's short message not granted"))
    send_data(PEER2, 16'h1111, 16'h2222, 64'h1204, 500, 0, 0, DVER, 2);
    send_data(PEER2, 16'h1111, 16'h2222, 64'h1204, 500, 64, 0, DVER, 2);
    repeat (200) @(posedge clk);
    for (q = 0; q < QN; q = q + 1) if (oq_v[q] && ob(q, 11) == T_GRANT && o64(q, 20) === 64'h1205 && o32(q, 28) > 256)
        `ERR(("granted new bytes to a message outside the top OVERCOMMIT"))
    flush_out;
    // drain: finish everything with plain data and ACKs
    n0 = d_cnt;
    for (off = 64; off < 400; off = off + SEG) send_data(PEER, 16'h1111, 16'h2222, 64'h1202, 400, off, 0, DVER, 2);
    for (off = 0; off < 280; off = off + SEG) send_data(32'h0A000065, 16'h1111, 16'h2222, 64'h1206, 280, off, 0, DVER, 2);
    for (off = 128; off < 500; off = off + SEG) send_data(PEER2, 16'h1111, 16'h2222, 64'h1204, 500, off, 0, DVER, 2);
    for (off = 0; off < 900; off = off + SEG) send_data(PEER, 16'h1111, 16'h2222, 64'h1200, 900, off, 0, DVER, 2);
    wait_count(n0 + 4, 20000);
    check_log(0, PEER, 16'h1111, 16'h2222, 64'h1202, 400, 2);
    check_log(0, 32'h0A000065, 16'h1111, 16'h2222, 64'h1206, 280, 2);
    check_log(0, PEER2, 16'h1111, 16'h2222, 64'h1204, 500, 2);
    check_log(0, PEER, 16'h1111, 16'h2222, 64'h1200, 900, 2);
    repeat (100) @(posedge clk);
    flush_out;
    for (i = 0; i < NRPC; i = i + 1)
        if (dut.u_core.v[i] && dut.u_core.ph[i] == 3) begin
            app_send(1, 0, 0, 0, {dut.u_core.gen[i], i[SL-1:0]}, 1, dut.u_core.rid[i]);
            p_hdr(T_ACK, 16'h1111, 16'h2222, dut.u_core.rid[i]);
            send(dut.u_core.peer[i], 30);
        end
    repeat (100) @(posedge clk);
    if (nslots(0) != 0) `ERR(("slots left after grant test: %0d", nslots(0)))
    flush_out;

    // 13: sender SRPT and CUTOFFS arriving mid-message
    section("13 sender SRPT + cutoffs");
    o_r = 0;
    app_send(0, PEER, 16'h0091, 16'h4020, 0, 250, 0); cid = last_id;
    app_send(0, PEER, 16'h0091, 16'h4021, 0, 10, 0);
    p_hdr(T_CUTOFFS, 16'h0, 16'h0, 64'h0);
    for (i = 0; i < 8; i = i + 1) p32(28 + 4*i, (i < 6) ? 1024 : 8);   // 250 bytes -> priority 5
    p16(60, 16'h0888); send(PEER, 62);
    repeat (50) @(posedge clk);
    o_r = 1;
    g = -1; j = 0;
    for (i = 0; i < 5; i = i + 1) begin
        wait_pkt(T_DATA, 2000, q);
        if (q >= 0) begin
            if (o32(q, 28) == 10 && g < 0) g = i;
            if (o32(q, 28) == 250) begin j = q; end
        end
    end
    if (g < 0 || g > 2) `ERR(("short request sent as packet %0d, want within the first 3", g))
    if (o16(j, 42) !== 16'h0888 || oq_p[j] != 5) `ERR(("last DATA after CUTOFFS: version %h prio %0d", o16(j, 42), oq_p[j]))
    n0 = d_cnt;
    send_data(PEER, 16'h0091, 16'h4020, cid | 1, 1, 0, 0, DVER, 1);
    send_data(PEER, 16'h0091, 16'h4021, last_id | 1, 1, 0, 0, DVER, 1);
    repeat (200) @(posedge clk);
    flush_out;

    // 14: buffer locks: a slot freed while its data is still being sent or delivered
    section("14 buffer locks");
    n0 = d_cnt;
    send_data(PEER, 16'h1111, 16'h2222, 64'h1400, 3, 0, 0, DVER, 2);
    wait_deliv(n0, 2000);
    o_r = 0;
    app_send(1, 0, 0, 0, d_h, 250, 64'h1400);
    repeat (30) @(posedge clk);
    p_hdr(T_ACK, 16'h1111, 16'h2222, 64'h1400); send(PEER, 30);          // frees it mid-transmission
    n0 = d_cnt;
    send_data(PEER, 16'h1111, 16'h2222, 64'h1402, 3, 0, 0, DVER, 2);       // reuses a slot
    wait_deliv(n0, 2000);
    app_send(1, 0, 0, 0, d_h, 250, 64'h1402);
    o_r = 1;
    j = 0;
    for (i = 0; i < 8; i = i + 1) begin
        wait_pkt(T_DATA, 500, q);
        if (q >= 0 && o64(q, 20) === 64'h1403) j = j + 1;
        if (q >= 0) begin
            if (o64(q, 20) === 64'h1401) check_data(q, PEER, 16'h2222, 16'h1111, 64'h1401, 250, o32(q, 48), 0, 5, 16'h0888, 64'h1400, 1);
            else check_data(q, PEER, 16'h2222, 16'h1111, 64'h1403, 250, o32(q, 48), 0, 5, 16'h0888, 64'h1402, 1);
        end
    end
    if (j != 4) `ERR(("second response sent %0d of 4 packets", j))
    p_hdr(T_ACK, 16'h1111, 16'h2222, 64'h1402); send(PEER, 30);
    flush_out;
    r_r = 0;                                                                  // stall delivery
    for (off = 0; off < 300; off = off + SEG) send_data(PEER, 16'h1111, 16'h2222, 64'h1404, 300, off, 0, DVER, 2);
    repeat (20) @(posedge clk);
    p_hdr(T_UNKNOWN, 16'h1111, 16'h2222, 64'h1404); send(PEER, 28);        // client forgot it: dead while delivering
    for (off = 0; off < 300; off = off + SEG) send_data(PEER, 16'h1111, 16'h2222, 64'h1406, 300, off, 0, DVER, 3);
    repeat (50) @(posedge clk);
    n0 = d_cnt;
    r_r = 1;
    wait_count(n0 + 2, 3000);
    check_log(0, PEER, 16'h1111, 16'h2222, 64'h1404, 300, 2);
    check_log(0, PEER, 16'h1111, 16'h2222, 64'h1406, 300, 3);
    app_send(1, 0, 0, 0, l_h[find_d(64'h1404)], 1, 64'h1404);
    if (last_ok) `ERR(("response accepted for an RPC the client abandoned"))
    app_send(1, 0, 0, 0, l_h[find_d(64'h1406)], 1, 64'h1406);
    p_hdr(T_ACK, 16'h1111, 16'h2222, 64'h1406); send(PEER, 30);
    // an ACK with S=1 is not from the client and must not free a server RPC
    n0 = d_cnt;
    send_data(PEER, 16'h1111, 16'h2222, 64'h1408, 3, 0, 0, DVER, 2);
    wait_deliv(n0, 2000);
    app_send(1, 0, 0, 0, d_h, 3, 64'h1408);
    repeat (50) @(posedge clk);
    p_hdr(T_ACK, 16'h1111, 16'h2222, 64'h1409); send(PEER, 30);
    repeat (20) @(posedge clk);
    if (nslots(0) != 1) `ERR(("ACK with S=1 freed a server RPC"))
    p_hdr(T_ACK, 16'h1111, 16'h2222, 64'h1408); send(PEER, 30);
    repeat (50) @(posedge clk);
    if (nslots(0) != 0) `ERR(("slots left after lock test: %0d", nslots(0)))
    flush_out;

    // 15: peers that push: DATA past the grant, early ACK, repeated START_MSG
    section("15 pushy peers");
    p_hdr(T_START, 16'h1111, 16'h2222, 64'h1500); p32(28, 600); send(PEER, 32);
    wait_pkt(T_GRANT, 2000, q);
    if (q < 0 || o32(q, 28) != 256) `ERR(("no first GRANT of 256"))
    base = ndrop;
    send_data(PEER, 16'h1111, 16'h2222, 64'h1500, 600, 320, 0, DVER, 2);      // never granted
    send_data(PEER, 16'h1111, 16'h2222, 64'h1506, 100, MAXM, 0, DVER, 2);     // new RPC, offset past its end
    repeat (10) @(posedge clk);
    if (ndrop - base != 2) `ERR(("DATA beyond the grant or past the end was accepted (%0d drops)", ndrop - base))
    if (nslots(0) != 1) `ERR(("out-of-range first packet created an RPC"))
    n0 = d_cnt;
    g = 256; off = 0;
    while (off < 600) begin
        if (off < g) begin
            send_data(PEER, 16'h1111, 16'h2222, 64'h1500, 600, off, 0, DVER, 2);
            off = off + SEG;
        end else begin
            wait_pkt(T_GRANT, 2000, q);
            if (q < 0) begin `ERR(("grant stalled at %0d", g)) off = 600; end
            else if (o32(q, 28) > g) g = o32(q, 28);
        end
    end
    wait_deliv(n0, 3000);
    h = d_h;
    p_hdr(T_ACK, 16'h1111, 16'h2222, 64'h1500); send(PEER, 30);           // before any response
    repeat (20) @(posedge clk);
    if (nslots(0) != 1) `ERR(("ACK before the response freed the RPC"))
    app_send(1, 0, 0, 0, h, 4, 64'h1500);
    if (!last_ok) `ERR(("response refused after an early ACK"))
    p_hdr(T_ACK, 16'h1111, 16'h2222, 64'h1500); send(PEER, 30);
    repeat (20) @(posedge clk);
    if (nslots(0) != 0) `ERR(("ACK after the response did not free the RPC"))
    flush_out;
    // a client that only repeats START_MSG cannot hold a server slot forever
    p_hdr(T_START, 16'h1111, 16'h2222, 64'h1502); p32(28, 600); send(PEER, 32);
    for (i = 0; i < (ABR + 4) * 2; i = i + 1) begin
        repeat (RTK * TICK) @(posedge clk);
        p_hdr(T_START, 16'h1111, 16'h2222, 64'h1502); p32(28, 600); send(PEER, 32);
    end
    p_hdr(T_START, 16'h1111, 16'h2222, 64'h1504); p32(28, 600); send(PEER, 32);   // a fresh one: must get a slot
    repeat (20) @(posedge clk);
    f = 0;
    for (i = 0; i < NRPC; i = i + 1)
        if (dut.u_core.v[i] && dut.u_core.rid[i] == 64'h1502 && !dut.u_core.dead[i]) f = 1;
    if (f) `ERR(("repeated START_MSG kept a server RPC alive"))
    repeat ((ABR + 3) * (RTK + 2) * TICK * 2) @(posedge clk);
    if (nslots(0) != 0) `ERR(("slots left after pushy peers: %0d", nslots(0)))
    flush_out;

    // 10: fuzz: random packets, then the endpoint must still work and drain
    section("10 fuzz");
    for (f = 0; f < 3000; f = f + 1) begin
        pn = {$random(seed)} % 100;
        for (j = 0; j < pn; j = j + 1) px[j] = $random(seed);
        if ({$random(seed)} % 2) px[11] = 16 + {$random(seed)} % 10;
        if ({$random(seed)} % 2) begin px[20] = 0; px[21] = 0; px[22] = 0; px[23] = 0; px[24] = 0; px[25] = 0; px[26] = 0; px[27] = {$random(seed)} % 8; end
        if ({$random(seed)} % 3 == 0) begin px[28] = 0; px[29] = 0; px[30] = 0; px[31] = 1 + {$random(seed)} % 200; end
        if (pn > 0) send(({$random(seed)} % 2) ? PEER : PEER2, pn);
        if (d_n == 0 && d_cnt > 0 && d_resp == 0 && dut.u_core.v[d_h[SL-1:0]] && dut.u_core.ph[d_h[SL-1:0]] == 3)
            app_send(1, 0, 0, 0, d_h, 1, d_id);   // answer fuzz-made requests so their slots can drain
    end
    for (i = 0; i < (AGU + 3) * NAK * TICK * 2; i = i + 1) begin
        if (d_n == 0 && d_resp == 0 && dut.u_core.v[d_h[SL-1:0]] && dut.u_core.ph[d_h[SL-1:0]] == 3)
            app_send(1, 0, 0, 0, d_h, 1, d_id);
        @(posedge clk);
    end
    if (nslots(0) != 0) `ERR(("slots left after fuzz: %0d", nslots(0)))
    flush_out;
    n0 = d_cnt;
    send_data(PEER, 16'h1111, 16'h2222, 64'h900, 77, 0, 0, DVER, 2);
    send_data(PEER, 16'h1111, 16'h2222, 64'h900, 77, 64, 0, DVER, 2);
    wait_deliv(n0, 2000);
    check_deliv(0, PEER, 16'h1111, 16'h2222, 64'h900, 77, 2);

    $display("DONE tb_peer: %0d errors -> %0s", errors, errors == 0 ? "PASS" : "FAIL");
    $finish;
end

initial begin
    #50000000;
    $display("TIMEOUT"); $finish;
end

endmodule
