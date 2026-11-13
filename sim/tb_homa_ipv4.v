`timescale 1ns/1ps
`define ERR(x) begin errors = errors + 1; if (errors <= 40) begin $write("%0t ERROR: ", $time); $display x; end end

// homa_ipv4: TX header fields and checksum checked independently; RX fed good,
// padded, truncated, corrupted and flagged-bad IPv4 packets, with backpressure.
module tb_ipv4;
parameter SEED = 1;
parameter N = 600;
localparam [31:0] ME = 32'hC0A80105;

reg clk = 0; always #5 clk = ~clk;
reg rst = 1;
integer errors = 0, seed = SEED;

reg  [63:0] tx_d = 0; reg [7:0] tx_k = 0; reg tx_v = 0, tx_l = 0; wire tx_r;
reg  [31:0] tx_da = 0; reg [2:0] tx_p = 0; reg [15:0] tx_len = 0;
wire [63:0] o_d; wire [7:0] o_k; wire o_v, o_l; reg o_r = 1;
reg  [63:0] rx_d = 0; reg [7:0] rx_k = 0; reg rx_v = 0, rx_l = 0, rx_u = 0; wire rx_r;
wire [63:0] h_d; wire [7:0] h_k; wire h_v, h_l, h_u; reg h_r = 1; wire [31:0] h_sa; wire rdrop;

homa_ipv4 dut (.clk(clk), .rst(rst), .cfg_addr(ME),
    .s_tx_tdata(tx_d), .s_tx_tkeep(tx_k), .s_tx_tvalid(tx_v), .s_tx_tlast(tx_l), .s_tx_tready(tx_r),
    .s_tx_daddr(tx_da), .s_tx_prio(tx_p), .s_tx_len(tx_len),
    .m_tx_tdata(o_d), .m_tx_tkeep(o_k), .m_tx_tvalid(o_v), .m_tx_tlast(o_l), .m_tx_tready(o_r),
    .s_rx_tdata(rx_d), .s_rx_tkeep(rx_k), .s_rx_tvalid(rx_v), .s_rx_tlast(rx_l), .s_rx_tuser(rx_u),
    .s_rx_tready(rx_r), .m_rx_tdata(h_d), .m_rx_tkeep(h_k), .m_rx_tvalid(h_v), .m_rx_tlast(h_l),
    .m_rx_tuser(h_u), .m_rx_tready(h_r), .m_rx_saddr(h_sa), .rx_drop(rdrop));

always @(negedge clk) begin o_r = ({$random(seed)} % 4) != 0; h_r = ({$random(seed)} % 4) != 0; end

reg [7:0] hp [0:2047];      // Homa packet being sent
reg [7:0] ip [0:2047];      // IPv4 packet captured / to inject
integer   ipn = 0, cj;
always @(posedge clk) if (o_v && o_r) begin
    for (cj = 0; cj < 8; cj = cj + 1) if (o_k[cj]) begin ip[ipn] = o_d[8*cj +: 8]; ipn = ipn + 1; end
    if (!o_l && o_k != 8'hFF) `ERR(("tx: partial tkeep mid-packet"))
end

function [15:0] ipsum(input integer dummy);
    integer j; reg [31:0] s;
    begin
        s = 0;
        for (j = 0; j < 20; j = j + 2) s = s + {ip[j], ip[j+1]};
        s = (s & 16'hFFFF) + (s >> 16); s = (s & 16'hFFFF) + (s >> 16);
        ipsum = ~s[15:0];
    end
endfunction

// output side of RX
reg [7:0] hb [0:2047];
integer   hn = 0, hpk = 0, hbadu = 0, hj;
always @(posedge clk) if (h_v && h_r) begin
    for (hj = 0; hj < 8; hj = hj + 1) if (h_k[hj]) begin hb[hn] = h_d[8*hj +: 8]; hn = hn + 1; end
    else if ((h_k >> hj) != 0) `ERR(("rx: tkeep %b not contiguous", h_k))
    if (!h_l && h_k != 8'hFF) `ERR(("rx: partial tkeep mid-packet"))
    if (h_l) begin hpk = hpk + 1; if (h_u) hbadu = hbadu + 1; end
end

task send_homa(input integer n, input [31:0] da, input [2:0] p);
    integer o, b;
    begin
        @(negedge clk);
        tx_da = da; tx_p = p; tx_len = n;
        for (o = 0; o < n; o = o + 8) begin
            for (b = 0; b < 8; b = b + 1) begin tx_d[8*b +: 8] = hp[o + b]; tx_k[b] = (o + b < n); end
            tx_v = 1; tx_l = (o + 8 >= n);
            @(posedge clk); while (!tx_r) @(posedge clk);
            @(negedge clk);
        end
        tx_v = 0; tx_l = 0;
    end
endtask

task inject(input integer n, input u, input integer holebeat);
    integer o, b;
    begin
        @(negedge clk);
        for (o = 0; o < n; o = o + 8) begin
            for (b = 0; b < 8; b = b + 1) begin rx_d[8*b +: 8] = ip[o + b]; rx_k[b] = (o + b < n); end
            if (o == 8 * holebeat) rx_k[0] = 1'b0;   // lane 0 marked empty mid-packet
            rx_v = 1; rx_l = (o + 8 >= n); rx_u = rx_l && u;
            @(posedge clk); while (!rx_r) @(posedge clk);
            @(negedge clk);
        end
        rx_v = 0; rx_l = 0; rx_u = 0;
    end
endtask

integer i, j, n, mode, t, pk0, bad0, d0, ndrop = 0, plen, want_out, want_bad, hole;
reg [31:0] da;
reg [2:0]  p;
always @(posedge clk) if (rdrop) ndrop = ndrop + 1;

initial begin
    repeat (4) @(posedge clk); rst = 0;
    for (i = 0; i < N; i = i + 1) begin
        n  = 28 + {$random(seed)} % 1100;
        da = $random(seed); p = $random(seed);
        for (j = 0; j < n; j = j + 1) hp[j] = $random(seed);
        ipn = 0;
        send_homa(n, da, p);
        for (t = 0; t < 2000 && ipn < n + 20; t = t + 1) @(posedge clk);
        repeat (3) @(posedge clk);
        // TX checks, independent of the RTL's header builder
        if (ipn != n + 20) `ERR(("tx: %0d bytes for a %0d byte Homa packet", ipn, n))
        if (ip[0] != 8'h45 || ip[1] != {p, 5'd0} || {ip[2], ip[3]} != n + 20 || ip[6] != 8'h40 || ip[7] != 0 ||
            ip[8] == 0 || ip[9] != 146 || {ip[12], ip[13], ip[14], ip[15]} != ME || {ip[16], ip[17], ip[18], ip[19]} != da)
            `ERR(("tx: bad IPv4 header for packet %0d", i))
        if (ipsum(0) != 0) `ERR(("tx: header checksum wrong"))
        for (j = 0; j < n; j = j + 1) if (ip[20 + j] != hp[j]) begin `ERR(("tx: payload byte %0d", j)) j = n; end

        // RX: mutate, then inject
        mode = {$random(seed)} % 12;
        {ip[16], ip[17], ip[18], ip[19]} = ME;
        {ip[12], ip[13], ip[14], ip[15]} = da;
        ip[10] = 0; ip[11] = 0; {ip[10], ip[11]} = ipsum(0);
        plen = n + 20; want_out = 1; want_bad = 0; hole = -1;
        case (mode)
        0: begin ip[9] = 17; want_out = 0; end                              // not Homa
        1: begin ip[11] = ip[11] ^ 8'h01; want_out = 0; end                 // bad checksum
        2: begin ip[16] = ip[16] ^ 8'h80; ip[10] = 0; ip[11] = 0; {ip[10], ip[11]} = ipsum(0); want_out = 0; end  // not for us
        3: begin ip[6] = 8'h20; ip[10] = 0; ip[11] = 0; {ip[10], ip[11]} = ipsum(0); want_out = 0; end           // fragment
        // trailing bytes: ok only if the frame ends on the beat that ends the output (beat j carries IP bytes 8j-4..8j+3)
        4: begin
            plen = n + 20 + 1 + {$random(seed)} % 30;
            for (j = n + 20; j < plen; j = j + 1) ip[j] = 8'hAA;
            want_bad = ((plen - 1) / 8 > (n + 23) / 8);
        end
        5: begin plen = 20 + 1 + {$random(seed)} % (n - 1); want_bad = 1; end    // truncated
        6: begin want_bad = 1; end                                           // frame error on tlast
        7: begin hole = 3 + {$random(seed)} % ((n + 20) / 8 - 3); want_bad = 1; end  // gap in tkeep
        8: begin hole = 3 + {$random(seed)} % ((n + 20) / 8 - 3); plen = n + 21; ip[n + 20] = 8'hAA; want_bad = 1; end  // gap hidden by an extra byte
        default: ;
        endcase
        hn = 0; pk0 = hpk; bad0 = hbadu; d0 = ndrop;
        if (plen <= 24 && mode == 5) want_out = 0;
        inject(plen, mode == 6, hole);
        for (t = 0; t < 200; t = t + 1) @(posedge clk);
        if (want_out) begin
            if (hpk != pk0 + 1) `ERR(("rx mode %0d: %0d packets out, want 1", mode, hpk - pk0))
            else begin
                if ((hbadu != bad0) != want_bad) `ERR(("rx mode %0d: tuser=%0d want %0d", mode, hbadu != bad0, want_bad))
                if (!want_bad) begin
                    if (hn != n) `ERR(("rx mode %0d: %0d bytes out, want %0d", mode, hn, n))
                    for (j = 0; j < hn && j < n; j = j + 1) if (hb[j] != hp[j]) begin `ERR(("rx: byte %0d", j)) j = n; end
                    if (h_sa !== da) `ERR(("rx: saddr %h want %h", h_sa, da))
                end
            end
        end else if (hpk != pk0 || ndrop != d0 + 1)
            `ERR(("rx mode %0d: should drop, got %0d packets, %0d drops", mode, hpk - pk0, ndrop - d0))
    end
    $display("DONE tb_ipv4: %0d packets, %0d errors -> %0s", N, errors, errors == 0 ? "PASS" : "FAIL");
    $finish;
end

endmodule
