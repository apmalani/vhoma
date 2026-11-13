`timescale 1ns/1ps
`define ERR(x) begin errors = errors + 1; if (errors <= 40) $display x; end

// Packets are checked byte-for-byte against a reference built from the draft's
// layout in this file. Injected traffic includes reordering, duplicates, loss,
// malformed packets, slot exhaustion, delivery stalls and expiry during a packet.
module tb;
parameter SEG   = 64;
parameter MAXM  = 512;
parameter NSLOT = 4;
parameter TICK  = 400;
parameter AGE   = 3;
parameter SEED  = 1;
parameter ROUNDS = 40;
parameter NTX   = 300;
parameter PMAX  = 4000;
parameter MMAX  = 1200;

localparam NSEG = MAXM / SEG;
localparam PB   = 52 + SEG + 64;              // stored packet stride (room for oversize)
localparam [15:0] CVER = 16'hBEEF;
localparam EXPIRE_WAIT = TICK * (AGE + 3) + 200;

reg clk = 1'b0;
always #5 clk = ~clk;
reg rst = 1'b1;
integer seed = SEED;
integer errors = 0;

reg  [255:0] cutoffs;
reg          tx_sv;
wire         tx_sr;
reg  [31:0]  tx_saddr, tx_slen;
reg  [15:0]  tx_ssport, tx_sdport;
reg  [63:0]  tx_sid, tx_dd;
reg          tx_dv;
wire         tx_dr;
wire [63:0]  tx_md;
wire [7:0]   tx_mk;
wire         tx_mv, tx_ml;
reg          tx_mr;
wire [31:0]  tx_mdaddr;
wire [15:0]  tx_mlen;
wire [2:0]   tx_mprio;
wire         tx_err;

homa_tx_unsched #(.SEG_BYTES(SEG), .MAX_MSG(MAXM)) dut_tx (
    .clk(clk), .rst(rst),
    .s_msg_valid(tx_sv), .s_msg_ready(tx_sr), .s_msg_daddr(tx_saddr),
    .s_msg_sport(tx_ssport), .s_msg_dport(tx_sdport), .s_msg_id(tx_sid), .s_msg_len(tx_slen),
    .s_axis_tdata(tx_dd), .s_axis_tvalid(tx_dv), .s_axis_tready(tx_dr),
    .m_axis_tdata(tx_md), .m_axis_tkeep(tx_mk), .m_axis_tvalid(tx_mv), .m_axis_tlast(tx_ml),
    .m_axis_tready(tx_mr), .m_axis_daddr(tx_mdaddr), .m_axis_len(tx_mlen), .m_axis_prio(tx_mprio),
    .cfg_cutoffs(cutoffs), .cfg_cutoff_ver(CVER), .err_len(tx_err));

reg  [63:0]       rx_sd;
reg  [7:0]        rx_sk;
reg               rx_sv, rx_sl;
reg  [31:0]       rx_saddr;
wire              rx_sr;
wire [63:0]       rx_md, rx_mid;
wire [7:0]        rx_mk;
wire              rx_mv, rx_ml;
reg               rx_mr;
wire [31:0]       rx_maddr, rx_mlen;
wire [15:0]       rx_msport, rx_mdport;
wire              rx_drop;
wire [NSLOT-1:0]  rx_exp;

homa_rx_unsched #(.SEG_BYTES(SEG), .MAX_MSG(MAXM), .NSLOT(NSLOT), .TICK(TICK), .AGE_MAX(AGE)) dut_rx (
    .clk(clk), .rst(rst),
    .s_axis_tdata(rx_sd), .s_axis_tkeep(rx_sk), .s_axis_tvalid(rx_sv), .s_axis_tlast(rx_sl),
    .s_axis_tready(rx_sr), .s_axis_saddr(rx_saddr),
    .m_axis_tdata(rx_md), .m_axis_tkeep(rx_mk), .m_axis_tvalid(rx_mv), .m_axis_tlast(rx_ml),
    .m_axis_tready(rx_mr), .m_axis_saddr(rx_maddr), .m_axis_sport(rx_msport),
    .m_axis_dport(rx_mdport), .m_axis_id(rx_mid), .m_axis_len(rx_mlen),
    .ev_drop(rx_drop), .ev_expire(rx_exp));

// backpressure
integer tx_stall = 0, rx_stall = 0;
reg     rx_hold = 1'b0;
always @(negedge clk) begin
    tx_mr = ({$random(seed)} % 256) >= tx_stall;
    rx_mr = !rx_hold && (({$random(seed)} % 256) >= rx_stall);
end

integer ndrop = 0, nexp = 0, ntxerr = 0, ev_i;
always @(posedge clk) begin
    if (rx_drop) ndrop = ndrop + 1;
    for (ev_i = 0; ev_i < NSLOT; ev_i = ev_i + 1) if (rx_exp[ev_i]) nexp = nexp + 1;
    if (tx_err) ntxerr = ntxerr + 1;
    if (!rst && (^{tx_mv, rx_mv, rx_drop, rx_exp, tx_err, tx_sr, tx_dr}) === 1'bx)
        `ERR(("%0t X on control output", $time))
end

// messages and reference packets
reg [63:0] m_id    [0:MMAX-1];
reg [31:0] m_len   [0:MMAX-1];
reg [31:0] m_addr  [0:MMAX-1];
reg [15:0] m_sport [0:MMAX-1];
reg [15:0] m_dport [0:MMAX-1];
integer    m_got   [0:MMAX-1];
integer    m_exp   [0:MMAX-1];   // required delivery count; -1 = zero or one
integer    m_pk0   [0:MMAX-1];
integer    m_npk   [0:MMAX-1];
integer    nm = 0, last_m;

reg [7:0]  pk      [0:PMAX*PB-1];
integer    pk_len  [0:PMAX-1];
reg [31:0] pk_addr [0:PMAX-1];
integer    pk_msg  [0:PMAX-1];
integer    npk = 0;

integer    ord     [0:2*PMAX-1];
integer    nord;

function [7:0] pbyte(input [63:0] id, input [31:0] k);
    reg [31:0] x;
    begin
        x = (id[31:0] ^ {id[47:32], id[63:48]}) * 32'h9E3779B1 + k * 32'h85EBCA77;
        x = x ^ (x >> 15);
        x = x * 32'h2C1B3C6D;
        pbyte = x[7:0] ^ x[31:24];
    end
endfunction

function [2:0] ref_prio(input [31:0] len);
    integer i, found;
    begin
        ref_prio = 0; found = 0;
        for (i = 7; i >= 0; i = i - 1)
            if (!found && cutoffs[32*i +: 32] >= len) begin ref_prio = i; found = 1; end
    end
endfunction

// draft §3 byte layout, written independently of the RTL
task build(input integer k, input integer m, input [31:0] off, input [31:0] plen);
    integer b, j;
    reg [63:0] id;
    reg [31:0] L;
    begin
        b = k * PB; id = m_id[m]; L = m_len[m];
        for (j = 0; j < 52; j = j + 1) pk[b + j] = 8'd0;
        pk[b + 0] = m_sport[m][15:8];  pk[b + 1] = m_sport[m][7:0];
        pk[b + 2] = m_dport[m][15:8];  pk[b + 3] = m_dport[m][7:0];
        pk[b + 11] = 8'd16;                                       // Type = DATA
        for (j = 0; j < 8; j = j + 1) pk[b + 20 + j] = id[63 - 8*j -: 8];
        for (j = 0; j < 4; j = j + 1) pk[b + 28 + j] = L[31 - 8*j -: 8];
        pk[b + 42] = CVER[15:8];       pk[b + 43] = CVER[7:0];
        for (j = 0; j < 4; j = j + 1) pk[b + 48 + j] = off[31 - 8*j -: 8];
        for (j = 0; j < plen; j = j + 1) pk[b + 52 + j] = pbyte(id, off + j);
        pk_len[k] = 52 + plen; pk_addr[k] = m_addr[m]; pk_msg[k] = m;
    end
endtask

task build_msg_at(input integer m, input integer k0);
    reg [31:0] off, plen;
    integer k;
    begin
        off = 0; k = k0;
        while (off < m_len[m]) begin
            plen = (m_len[m] - off > SEG) ? SEG : m_len[m] - off;
            build(k, m, off, plen);
            k = k + 1; off = off + SEG;
        end
        m_npk[m] = k - k0;
    end
endtask

task new_msg(input [31:0] len, input [31:0] addr);
    reg [31:0] r1, r2;
    begin
        r1 = $random(seed); r2 = $random(seed);
        m_id[nm]    = {r1, nm[30:0], r2[0]};
        m_len[nm]   = len;
        m_addr[nm]  = addr;
        m_sport[nm] = r2[31:16];
        m_dport[nm] = r2[15:0] ^ 16'h5A5A;
        m_got[nm]   = 0;
        m_exp[nm]   = 1;
        m_pk0[nm]   = npk;
        build_msg_at(nm, npk);
        npk = npk + m_npk[nm];
        last_m = nm;
        nm = nm + 1;
    end
endtask

task rand_addr(output [31:0] a);
    a = 32'h0A000001 + ({$random(seed)} % 4);
endtask

task rand_len(output [31:0] L);
    reg [31:0] r;
    begin
        r = $random(seed);
        case (r[3:0])
        0: L = 1;
        1: L = SEG;
        2: L = (SEG + 1 <= MAXM) ? SEG + 1 : MAXM;
        3: L = MAXM;
        4: L = 4;
        5: L = 5;
        default: L = 1 + (r[31:4] % MAXM);
        endcase
    end
endtask

task copy_pkt(input integer dst, input integer src);
    integer j;
    begin
        for (j = 0; j < PB; j = j + 1) pk[dst*PB + j] = pk[src*PB + j];
        pk_len[dst] = pk_len[src]; pk_addr[dst] = pk_addr[src]; pk_msg[dst] = pk_msg[src];
    end
endtask

// copy with every payload byte inverted, so wrongly accepting it always corrupts data
task bad_copy(input integer dst, input integer src);
    integer j;
    begin
        copy_pkt(dst, src);
        for (j = 52; j < PB; j = j + 1) pk[dst*PB + j] = ~pk[dst*PB + j];
    end
endtask

task put32(input integer k, input integer at, input [31:0] v);
    integer j;
    for (j = 0; j < 4; j = j + 1) pk[k*PB + at + j] = v[31 - 8*j -: 8];
endtask

task reset_tables;
    begin nm = 0; npk = 0; end
endtask

// TX capture: byte-exact vs reference
integer cap_k = 0, cap_end = 0, cap_n = 0, cj;
always @(posedge clk) if (tx_mv && tx_mr) begin
    if (cap_k >= cap_end) begin
        `ERR(("%0t tx: unexpected packet beat", $time))
    end else begin
        if (tx_mdaddr !== pk_addr[cap_k] || tx_mlen !== pk_len[cap_k] ||
            tx_mprio !== ref_prio(m_len[pk_msg[cap_k]]))
            `ERR(("%0t tx pkt %0d: sideband addr=%h len=%0d prio=%0d want %h %0d %0d", $time, cap_k,
                  tx_mdaddr, tx_mlen, tx_mprio, pk_addr[cap_k], pk_len[cap_k], ref_prio(m_len[pk_msg[cap_k]])))
        if (!tx_ml && tx_mk !== 8'hFF)
            `ERR(("%0t tx pkt %0d: partial tkeep %b mid-packet", $time, cap_k, tx_mk))
        for (cj = 0; cj < 8; cj = cj + 1)
            if (tx_mk[cj]) begin
                if (cap_n >= pk_len[cap_k])
                    `ERR(("%0t tx pkt %0d: too long", $time, cap_k))
                else if (tx_md[8*cj +: 8] !== pk[cap_k*PB + cap_n])
                    `ERR(("%0t tx pkt %0d byte %0d: got %h want %h", $time, cap_k, cap_n,
                          tx_md[8*cj +: 8], pk[cap_k*PB + cap_n]))
                cap_n = cap_n + 1;
            end else if ((tx_mk >> (cj + 1)) != 0)
                `ERR(("%0t tx pkt %0d: non-contiguous tkeep %b", $time, cap_k, tx_mk))
        if (tx_ml) begin
            if (cap_n != pk_len[cap_k])
                `ERR(("%0t tx pkt %0d: length %0d want %0d", $time, cap_k, cap_n, pk_len[cap_k]))
            cap_k = cap_k + 1;
            cap_n = 0;
        end
    end
end

// RX monitor: scoreboard
reg  [7:0]  mon_buf [0:MAXM+63];
integer     mon_n = 0, mon_j, mon_m, mon_bad, ndeliv = 0;
reg  [31:0] mon_addr, mon_len;
reg  [63:0] mon_id;
reg  [15:0] mon_sp, mon_dp;
always @(posedge clk) if (rx_mv && rx_mr) begin
    if (mon_n == 0) begin
        mon_addr = rx_maddr; mon_id = rx_mid; mon_len = rx_mlen; mon_sp = rx_msport; mon_dp = rx_mdport;
    end else if (mon_addr !== rx_maddr || mon_id !== rx_mid || mon_len !== rx_mlen ||
                 mon_sp !== rx_msport || mon_dp !== rx_mdport)
        `ERR(("%0t rx: sideband changed mid-message", $time))
    if (!rx_ml && rx_mk !== 8'hFF)
        `ERR(("%0t rx: partial tkeep %b mid-message", $time, rx_mk))
    for (mon_j = 0; mon_j < 8; mon_j = mon_j + 1)
        if (rx_mk[mon_j]) begin
            if (mon_n < MAXM + 64) mon_buf[mon_n] = rx_md[8*mon_j +: 8];
            mon_n = mon_n + 1;
        end
    if (rx_ml) begin
        mon_m = -1;
        for (mon_j = 0; mon_j < nm; mon_j = mon_j + 1)
            if (m_addr[mon_j] == mon_addr && m_id[mon_j] == mon_id) mon_m = mon_j;
        if (mon_m < 0) begin
            `ERR(("%0t rx: delivered unknown message addr=%h id=%h len=%0d", $time, mon_addr, mon_id, mon_n))
        end else begin
            if (mon_n != m_len[mon_m] || mon_len != m_len[mon_m])
                `ERR(("%0t rx msg %0d: %0d bytes, sideband len %0d, want %0d", $time, mon_m, mon_n, mon_len, m_len[mon_m]))
            if (mon_sp !== m_sport[mon_m] || mon_dp !== m_dport[mon_m])
                `ERR(("%0t rx msg %0d: ports %h/%h want %h/%h", $time, mon_m, mon_sp, mon_dp, m_sport[mon_m], m_dport[mon_m]))
            mon_bad = -1;
            for (mon_j = 0; mon_j < mon_n && mon_j < MAXM; mon_j = mon_j + 1)
                if (mon_bad < 0 && mon_buf[mon_j] !== pbyte(m_id[mon_m], mon_j)) mon_bad = mon_j;
            if (mon_bad >= 0)
                `ERR(("%0t rx msg %0d len %0d: byte %0d is %h want %h", $time, mon_m, m_len[mon_m], mon_bad,
                      mon_buf[mon_bad], pbyte(m_id[mon_m], mon_bad)))
            m_got[mon_m] = m_got[mon_m] + 1;
            if (m_exp[mon_m] == 0 || (m_exp[mon_m] < 0 && m_got[mon_m] > 1) ||
                (m_exp[mon_m] > 0 && m_got[mon_m] > m_exp[mon_m]))
                `ERR(("%0t rx msg %0d: unexpected delivery #%0d (expected %0d)", $time, mon_m, m_got[mon_m], m_exp[mon_m]))
        end
        mon_n = 0;
        ndeliv = ndeliv + 1;
    end
end

// throughput probes
reg     perf_on = 1'b0, tx_in_msg = 1'b0, rx_in_msg = 1'b0;
integer tx_bub = 0, rx_bub = 0, tx_left = 0;
always @(posedge clk) if (perf_on) begin
    if (tx_in_msg && !tx_mv) tx_bub = tx_bub + 1;
    if (tx_mv && tx_mr) begin
        if (!tx_in_msg) begin tx_in_msg = 1'b1; tx_left = m_npk[0]; end
        if (tx_ml) begin tx_left = tx_left - 1; if (tx_left == 0) tx_in_msg = 1'b0; end
    end
    if (rx_in_msg && !rx_mv) rx_bub = rx_bub + 1;
    if (rx_mv && rx_mr) rx_in_msg = !rx_ml;
end

task tx_desc(input [31:0] addr, input [15:0] sp, input [15:0] dp, input [63:0] id, input [31:0] len);
    begin
        tx_sv = 1'b1; tx_saddr = addr; tx_ssport = sp; tx_sdport = dp; tx_sid = id; tx_slen = len;
        @(posedge clk); while (!tx_sr) @(posedge clk);
        @(negedge clk); tx_sv = 1'b0;
    end
endtask

task tx_payload(input [63:0] id, input [31:0] len, input integer gap);
    integer w, b;
    reg [63:0] word;
    begin
        for (w = 0; w < (len + 7) / 8; w = w + 1) begin
            for (b = 0; b < 8; b = b + 1) word[8*b +: 8] = pbyte(id, 8*w + b);
            while (({$random(seed)} % 256) < gap) begin tx_dv = 1'b0; @(negedge clk); end
            tx_dv = 1'b1; tx_dd = word;
            @(posedge clk); while (!tx_dr) @(posedge clk);
            @(negedge clk);
        end
        tx_dv = 1'b0;
    end
endtask

task tx_run(input integer m0, input integer m1, input integer gap);
    integer m, t;
    begin
        cap_k = m_pk0[m0]; cap_end = m_pk0[m1-1] + m_npk[m1-1]; cap_n = 0;
        for (m = m0; m < m1; m = m + 1) begin
            tx_desc(m_addr[m], m_sport[m], m_dport[m], m_id[m], m_len[m]);
            tx_payload(m_id[m], m_len[m], gap);
        end
        for (t = 0; t < 100000 && cap_k < cap_end; t = t + 1) @(negedge clk);
        if (cap_k != cap_end || cap_n != 0)
            `ERR(("%0t tx: captured %0d packets, want %0d", $time, cap_k - m_pk0[m0], cap_end - m_pk0[m0]))
    end
endtask

integer pause_beat = -1, pause_len = 0;
task inject(input integer k, input integer gap, input integer kmode);
    integer n, b, L, base, beat;
    reg [63:0] w;
    reg [7:0]  kp;
    begin
        L = pk_len[k]; base = k * PB; beat = 0;
        for (n = 0; n < L; n = n + 8) begin
            w = 64'd0; kp = 8'd0;
            for (b = 0; b < 8; b = b + 1)
                if (n + b < L) begin w[8*b +: 8] = pk[base + n + b]; kp[b] = 1'b1; end
            while (({$random(seed)} % 256) < gap) begin rx_sv = 1'b0; @(negedge clk); end
            if (beat == pause_beat) begin rx_sv = 1'b0; repeat (pause_len) @(negedge clk); end
            rx_sv = 1'b1; rx_sd = w; rx_sk = kp; rx_sl = (n + 8 >= L); rx_saddr = pk_addr[k];
            if (kmode == 1 && beat == 2 && !rx_sl) rx_sk = 8'h7F;
            if (kmode == 2 && rx_sl) rx_sk = (kp == 8'hFF) ? 8'hFE : kp << 1;   // same byte count, shifted
            @(negedge clk);
            beat = beat + 1;
        end
        rx_sv = 1'b0;
    end
endtask

task ord_msg(input integer m);
    integer k;
    for (k = m_pk0[m]; k < m_pk0[m] + m_npk[m]; k = k + 1) begin ord[nord] = k; nord = nord + 1; end
endtask

task ord_shuffle;
    integer i, j, t;
    for (i = nord - 1; i > 0; i = i - 1) begin
        j = {$random(seed)} % (i + 1);
        t = ord[i]; ord[i] = ord[j]; ord[j] = t;
    end
endtask

// duplicate random packets, never after the packet that completes their message
integer ndup;
task ord_add_dups(input integer pct);
    integer i, n, m, last;
    integer tmp [0:2*PMAX-1];
    begin
        n = 0; ndup = 0;
        for (i = 0; i < nord; i = i + 1) begin
            tmp[n] = ord[i]; n = n + 1;
            m = pk_msg[ord[i]];
            last = 1;
            begin : scan
                integer z;
                for (z = i + 1; z < nord; z = z + 1) if (pk_msg[ord[z]] == m) last = 0;
            end
            if (!last && ({$random(seed)} % 100) < pct) begin
                tmp[n] = ord[i]; n = n + 1; ndup = ndup + 1;
            end
        end
        for (i = 0; i < n; i = i + 1) ord[i] = tmp[i];
        nord = n;
    end
endtask

task inject_ord(input integer gap);
    integer i;
    for (i = 0; i < nord; i = i + 1) inject(ord[i], gap, 0);
endtask

task wait_check(input integer m0, input integer m1);
    integer t, m, pending;
    begin
        pending = 1;
        for (t = 0; t < 100000 && pending; t = t + 1) begin
            pending = 0;
            for (m = m0; m < m1; m = m + 1) if (m_exp[m] > 0 && m_got[m] < m_exp[m]) pending = 1;
            @(negedge clk);
        end
        repeat (300) @(negedge clk);
        for (m = m0; m < m1; m = m + 1)
            if ((m_exp[m] >= 0 && m_got[m] != m_exp[m]) || (m_exp[m] < 0 && m_got[m] > 1))
                `ERR(("%0t msg %0d len %0d: delivered %0d times, expected %0d", $time, m, m_len[m], m_got[m], m_exp[m]))
    end
endtask

task expect_drops(input integer d0, input integer n);
    begin
        repeat (4) @(negedge clk);
        if (ndrop - d0 != n) `ERR(("%0t expected %0d drops, saw %0d", $time, n, ndrop - d0))
    end
endtask

task expect_exp(input integer e0, input integer n);
    if (nexp - e0 != n) `ERR(("%0t expected %0d expirations, saw %0d", $time, n, nexp - e0))
endtask

task expect_idle;
    begin
        if (dut_rx.busy !== 0 || dut_rx.d_act !== 1'b0 || dut_rx.wp !== dut_rx.rp)
            `ERR(("%0t rx not idle: busy=%b done=%b d_act=%b", $time, dut_rx.busy, dut_rx.done, dut_rx.d_act))
    end
endtask

task section(input [8*40-1:0] name);
    begin
        if (errors != 0) begin
            $display("DONE SEG=%0d MAXM=%0d NSLOT=%0d: %0d errors -> FAIL", SEG, MAXM, NSLOT, errors);
            $finish;
        end
        $display("%0t  %0s", $time, name);
    end
endtask

integer L, a, t, r, m, k, s, d0, e0, nmsg, inc, lost;
integer lostm [0:MMAX-1];
reg [31:0] L32, A32;
initial begin
    for (t = 0; t < 8; t = t + 1) cutoffs[32*t +: 32] = MAXM >> t;
    tx_sv = 0; tx_dv = 0; tx_dd = 0; rx_sv = 0; rx_sl = 0; rx_sk = 0; rx_sd = 0; rx_saddr = 0;
    repeat (5) @(negedge clk);
    rst = 1'b0;
    repeat (5) @(negedge clk);

    // T1: TX packet format for every length (or many), with stalls; then loop back in order
    section("T1 tx format+loopback");
    tx_stall = 80; rx_stall = 80;
    reset_tables; d0 = ndrop;
    if (MAXM <= 1024) begin
        for (L = 1; L <= MAXM; L = L + 1) begin rand_addr(A32); new_msg(L, A32); end
    end else begin
        for (t = 0; t < NTX; t = t + 1) begin rand_len(L32); rand_addr(A32); new_msg(L32, A32); end
    end
    tx_run(0, nm, 60);
    // one message at a time: overload (slots all waiting on delivery) is covered by T7/T8/T10
    for (m = 0; m < nm; m = m + 1) begin
        nord = 0; ord_msg(m); inject_ord(40);
        for (t = 0; t < 100000 && m_got[m] == 0; t = t + 1) @(negedge clk);
    end
    wait_check(0, nm);
    expect_drops(d0, 0);

    // T1b: TX rejects bad lengths without losing sync
    section("T1b tx bad lengths");
    reset_tables; d0 = ntxerr;
    cap_k = 0; cap_end = 0;
    tx_desc(32'h0A000009, 16'h1, 16'h2, 64'h10, 32'd0);
    tx_desc(32'h0A000009, 16'h1, 16'h2, 64'h12, MAXM + 1);
    tx_payload(64'h12, MAXM + 1, 30);
    repeat (50) @(negedge clk);
    if (ntxerr - d0 != 2) `ERR(("%0t tx: %0d err_len pulses, want 2", $time, ntxerr - d0))
    if (cap_k != 0) `ERR(("%0t tx: emitted packets for a rejected message", $time))
    new_msg(SEG + 3 <= MAXM ? SEG + 3 : MAXM, 32'h0A000009);
    tx_run(0, nm, 30);

    // T2: shuffled packets, up to NSLOT messages at once
    section("T2 reorder");
    for (r = 0; r < ROUNDS; r = r + 1) begin
        reset_tables; d0 = ndrop; nord = 0;
        for (t = 0; t < NSLOT; t = t + 1) begin rand_len(L32); rand_addr(A32); new_msg(L32, A32); ord_msg(last_m); end
        ord_shuffle;
        inject_ord(({$random(seed)} % 4) * 30);
        wait_check(0, nm);
        expect_drops(d0, 0);
    end

    // T3: duplicates before completion are dropped, never corrupt, never double-deliver
    section("T3 duplicates");
    for (r = 0; r < ROUNDS; r = r + 1) begin
        reset_tables; d0 = ndrop; nord = 0;
        for (t = 0; t < NSLOT; t = t + 1) begin rand_len(L32); rand_addr(A32); new_msg(L32, A32); ord_msg(last_m); end
        ord_shuffle;
        ord_add_dups(35);
        inject_ord(30);
        wait_check(0, nm);
        expect_drops(d0, ndup);
    end

    // T4: a duplicate with different payload must not overwrite the first copy
    if (NSEG >= 2) begin
        section("T4 conflicting dup");
        reset_tables; d0 = ndrop;
        new_msg(2 * SEG, 32'h0A000001);
        inject(m_pk0[0], 0, 0);
        bad_copy(npk, m_pk0[0]);
        inject(npk, 0, 0);
        inject(m_pk0[0] + 1, 0, 0);
        wait_check(0, nm);
        expect_drops(d0, 1);
    end

    // T5: malformed packets are dropped and never corrupt valid data
    section("T5 malformed");
    reset_tables; d0 = ndrop;
    new_msg(NSEG >= 2 ? 2 * SEG : MAXM, 32'h0A000002);
    m = last_m; k = m_pk0[m]; s = npk; inc = 0;
    bad_copy(s, k); pk[s*PB + 11] = 8'd17;                        inject(s, 0, 0); inc = inc + 1;
    bad_copy(s, k); put32(s, 48, SEG / 2);                        inject(s, 0, 0); inc = inc + 1;
    bad_copy(s, k); put32(s, 48, ((m_len[m] + SEG - 1) / SEG) * SEG); inject(s, 0, 0); inc = inc + 1;
    bad_copy(s, k); put32(s, 48, MAXM);                           inject(s, 0, 0); inc = inc + 1;
    bad_copy(s, k); put32(s, 48, 32'hFFFFFFFF & ~(SEG - 1));      inject(s, 0, 0); inc = inc + 1;
    bad_copy(s, k); put32(s, 28, 0);                              inject(s, 0, 0); inc = inc + 1;
    bad_copy(s, k); put32(s, 28, MAXM + SEG);                     inject(s, 0, 0); inc = inc + 1;
    bad_copy(s, k); pk_len[s] = pk_len[s] - 5;                    inject(s, 0, 0); inc = inc + 1;
    bad_copy(s, k); pk_len[s] = pk_len[s] + 40;                   inject(s, 0, 0); inc = inc + 1;
    bad_copy(s, k); pk_len[s] = 20;                               inject(s, 0, 0); inc = inc + 1;
    bad_copy(s, k); pk_len[s] = 52;                               inject(s, 0, 0); inc = inc + 1;
    bad_copy(s, k);                                               inject(s, 0, 2); inc = inc + 1;
    if (pk_len[s] > 24) begin bad_copy(s, k);                     inject(s, 0, 1); inc = inc + 1; end
    if (m_npk[m] >= 2) begin
        inject(k, 0, 0);                                          // slot now allocated: hit path
        // self-consistent packet claiming a shorter message: only the length-match check stops it
        bad_copy(s, k + 1); put32(s, 28, m_len[m] - 1); pk_len[s] = pk_len[s] - 1;
                                                                  inject(s, 0, 0); inc = inc + 1;
        bad_copy(s, k + 1); pk_len[s] = pk_len[s] - 1;            inject(s, 0, 0); inc = inc + 1;
        bad_copy(s, k + 1); pk_len[s] = pk_len[s] + 64;           inject(s, 0, 0); inc = inc + 1;
        bad_copy(s, k + 1);                                       inject(s, 0, 2); inc = inc + 1;
        bad_copy(s, k + 1); pk[s*PB + 1] = ~pk[s*PB + 1];         inject(s, 0, 0); inc = inc + 1;
        bad_copy(s, k + 1); pk[s*PB + 3] = ~pk[s*PB + 3];         inject(s, 0, 0); inc = inc + 1;
        inject(k + 1, 0, 0);
    end else
        inject(k, 0, 0);
    wait_check(0, nm);
    expect_drops(d0, inc);

    // T5b: oversized packets must not spill into a received segment or the next slot
    if (NSEG >= 2) begin
        section("T5b overflow guard");
        reset_tables; d0 = ndrop; inc = 0;
        new_msg(MAXM, 32'h0A000003);                              // A -> slot 0
        new_msg(MAXM, 32'h0A000004);                              // B -> slot 1
        inject(m_pk0[0], 0, 0);
        inject(m_pk0[1], 0, 0);
        copy_pkt(npk, m_pk0[0] + NSEG - 1); pk_len[npk] = pk_len[npk] + 64;
        for (t = pk_len[npk] - 64; t < pk_len[npk]; t = t + 1) pk[npk*PB + t] = 8'hEE;
        inject(npk, 0, 0); inc = inc + 1;
        if (NSEG >= 3) begin
            inject(m_pk0[0] + NSEG - 1, 0, 0);
            copy_pkt(npk, m_pk0[0] + NSEG - 2); pk_len[npk] = pk_len[npk] + 64;
            for (t = pk_len[npk] - 64; t < pk_len[npk]; t = t + 1) pk[npk*PB + t] = 8'hEE;
            inject(npk, 0, 0); inc = inc + 1;
            for (k = 1; k < NSEG - 1; k = k + 1) inject(m_pk0[0] + k, 0, 0);
        end else
            inject(m_pk0[0] + 1, 0, 0);
        for (k = 1; k < NSEG; k = k + 1) inject(m_pk0[1] + k, 0, 0);
        wait_check(0, nm);
        expect_drops(d0, inc);
    end

    // T5c: same Rpcid from two different peers are separate messages
    section("T5c same id, two peers");
    reset_tables; d0 = ndrop;
    new_msg(MAXM, 32'h0A000005);
    new_msg(MAXM > 8 ? MAXM - 7 : MAXM, 32'h0A000006);
    m_id[1] = m_id[0];
    build_msg_at(1, m_pk0[1]);
    nord = 0; ord_msg(0); ord_msg(1); ord_shuffle; inject_ord(10);
    wait_check(0, nm);
    expect_drops(d0, 0);

    // T6: loss: incomplete messages are never delivered and their slots expire
    section("T6 loss + expiry");
    for (r = 0; r < ROUNDS / 4 + 1; r = r + 1) begin
        reset_tables; d0 = ndrop; e0 = nexp; nord = 0; lost = 0;
        for (t = 0; t < NSLOT; t = t + 1) begin rand_len(L32); rand_addr(A32); new_msg(L32, A32); lostm[t] = 0; end
        for (m = 0; m < nm; m = m + 1)
            for (k = m_pk0[m]; k < m_pk0[m] + m_npk[m]; k = k + 1)
                if (({$random(seed)} % 100) < 20) lostm[m] = lostm[m] + 1;
                else begin ord[nord] = k; nord = nord + 1; end
        for (m = 0; m < nm; m = m + 1) begin
            if (lostm[m] > 0) m_exp[m] = 0;
            if (lostm[m] > 0 && lostm[m] < m_npk[m]) lost = lost + 1;
        end
        ord_shuffle;
        inject_ord(20);
        wait_check(0, nm);
        repeat (EXPIRE_WAIT) @(negedge clk);
        expect_drops(d0, 0);
        expect_exp(e0, lost);
        expect_idle;
    end

    // T7: more messages than slots
    if (NSEG >= 2) begin
        section("T7 slot exhaustion");
        reset_tables; d0 = ndrop; e0 = nexp;
        for (t = 0; t < NSLOT + 2; t = t + 1) new_msg(2 * SEG, 32'h0A000010 + t);
        for (t = 0; t < NSLOT + 2; t = t + 1) inject(m_pk0[t], 10, 0);
        expect_drops(d0, 2);
        m_exp[NSLOT] = 0; m_exp[NSLOT + 1] = 0;
        for (t = 0; t < NSLOT; t = t + 1) inject(m_pk0[t] + 1, 10, 0);
        wait_check(0, nm);
        inject(m_pk0[NSLOT] + 1, 0, 0);
        inject(m_pk0[NSLOT + 1] + 1, 0, 0);
        repeat (EXPIRE_WAIT) @(negedge clk);
        wait_check(0, nm);
        expect_drops(d0, 2);
        expect_exp(e0, 2);
        expect_idle;
    end

    // T8: delivery stalled: complete messages wait (never expire), new ones are dropped
    section("T8 delivery stall");
    reset_tables; d0 = ndrop; e0 = nexp;
    rx_hold = 1'b1;
    nord = 0;
    for (t = 0; t < NSLOT; t = t + 1) begin rand_len(L32); new_msg(L32, 32'h0A000020 + t); ord_msg(last_m); end
    ord_shuffle; inject_ord(10);
    repeat (EXPIRE_WAIT) @(negedge clk);
    expect_exp(e0, 0);
    rand_len(L32); new_msg(L32, 32'h0A000030); m_exp[last_m] = 0;
    for (k = m_pk0[last_m]; k < m_pk0[last_m] + m_npk[last_m]; k = k + 1) inject(k, 0, 0);
    expect_drops(d0, m_npk[last_m]);
    rx_hold = 1'b0;
    wait_check(0, nm);
    expect_idle;

    // T9: a slot whose age has run out must not expire while its packet is still arriving
    if (NSEG >= 2) begin
        section("T9 expiry vs packet in flight");
        reset_tables; d0 = ndrop; e0 = nexp;
        new_msg(2 * SEG, 32'h0A000040);
        inject(m_pk0[0], 0, 0);
        pause_beat = 7; pause_len = TICK * (AGE + 3);
        inject(m_pk0[0] + 1, 0, 0);
        pause_beat = -1;
        wait_check(0, nm);
        expect_drops(d0, 0);
        expect_exp(e0, 0);
    end

    // T10: soak: random load, reorder, duplicates, loss, stalls, slot pressure
    section("T10 soak");
    for (r = 0; r < ROUNDS * 2; r = r + 1) begin
        reset_tables; d0 = ndrop; nord = 0;
        rx_stall = {$random(seed)} % 200;
        nmsg = 1 + {$random(seed)} % (2 * NSLOT);
        for (t = 0; t < nmsg; t = t + 1) begin rand_len(L32); rand_addr(A32); new_msg(L32, A32); lostm[t] = 0; end
        a = ({$random(seed)} % 3 == 0) ? 10 : 0;                  // loss percent
        for (m = 0; m < nm; m = m + 1)
            for (k = m_pk0[m]; k < m_pk0[m] + m_npk[m]; k = k + 1)
                if (({$random(seed)} % 100) < a) lostm[m] = 1;
                else begin ord[nord] = k; nord = nord + 1; end
        ord_shuffle;
        ord_add_dups({$random(seed)} % 40);
        for (m = 0; m < nm; m = m + 1)
            m_exp[m] = (nmsg <= NSLOT && !lostm[m]) ? 1 : (lostm[m] ? 0 : -1);
        inject_ord(({$random(seed)} % 4) * 25);
        wait_check(0, nm);
        if (nmsg <= NSLOT && a == 0) expect_drops(d0, ndup);
        repeat (EXPIRE_WAIT) @(negedge clk);
        expect_idle;
    end
    rx_stall = 80;

    // T11: duplicates arriving after delivery are delivered again (no ACK state)
    section("T11 dup after delivery");
    reset_tables; d0 = ndrop; e0 = nexp;
    new_msg(SEG < 5 ? SEG : 5, 32'h0A000050);
    inject(m_pk0[0], 0, 0);
    wait_check(0, nm);
    m_exp[0] = 2;                                                  // no ACK state yet: redelivered
    inject(m_pk0[0], 0, 0);
    wait_check(0, nm);
    if (NSEG >= 2) begin
        new_msg(2 * SEG, 32'h0A000051);
        inject(m_pk0[1], 0, 0); inject(m_pk0[1] + 1, 0, 0);
        wait_check(0, nm);
        inject(m_pk0[1], 0, 0);                                    // partial copy: expires, not delivered
        repeat (EXPIRE_WAIT) @(negedge clk);
        wait_check(0, nm);
        expect_exp(e0, 1);
    end
    expect_idle;

    // T12: line rate: no idle cycles inside a message on TX output or RX delivery
    section("T12 throughput");
    reset_tables;
    tx_stall = 0; rx_stall = 0;
    new_msg(MAXM, 32'h0A000060);
    perf_on = 1'b1; tx_bub = 0; rx_bub = 0;
    tx_run(0, nm, 0);
    nord = 0; ord_msg(0); inject_ord(0);
    wait_check(0, nm);
    perf_on = 1'b0;
    if (tx_bub != 0) `ERR(("%0t tx: %0d idle cycles inside a message", $time, tx_bub))
    if (rx_bub != 0) `ERR(("%0t rx: %0d idle cycles inside a delivery", $time, rx_bub))

    section("end");
    repeat (EXPIRE_WAIT) @(negedge clk);
    expect_idle;
    $display("DONE SEG=%0d MAXM=%0d NSLOT=%0d: %0d deliveries, %0d drops, %0d expirations, %0d errors -> %0s",
             SEG, MAXM, NSLOT, ndeliv, ndrop, nexp, errors, errors == 0 ? "PASS" : "FAIL");
    $finish;
end

initial begin
    #2000000000;
    $display("TIMEOUT");
    $finish;
end

endmodule
