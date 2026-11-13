// Homa wire constants and helpers (draft-ousterhout-tsvwg-homa-00 §3). Included inside modules.
// Header vectors hold byte i at [8*i +: 8], matching AXI-Stream lane order.
localparam [7:0] T_DATA = 8'd16, T_GRANT = 8'd17, T_RESEND = 8'd18, T_UNKNOWN = 8'd19,
                 T_BUSY = 8'd20, T_CUTOFFS = 8'd21, T_NEED_ACK = 8'd23, T_ACK = 8'd24,
                 T_START = 8'd25;

function [15:0] be16(input [15:0] x); be16 = {x[7:0], x[15:8]}; endfunction
function [31:0] be32(input [31:0] x); be32 = {x[7:0], x[15:8], x[23:16], x[31:24]}; endfunction

function [7:0]  hb8 (input [639:0] h, input integer o); hb8  = h[8*o +: 8]; endfunction
function [15:0] hb16(input [639:0] h, input integer o); hb16 = {h[8*o +: 8], h[8*o+8 +: 8]}; endfunction
function [31:0] hb32(input [639:0] h, input integer o);
    hb32 = {h[8*o +: 8], h[8*o+8 +: 8], h[8*o+16 +: 8], h[8*o+24 +: 8]};
endfunction
function [63:0] hb64(input [639:0] h, input integer o); hb64 = {hb32(h, o), hb32(h, o + 4)}; endfunction

// unscheduled priority: largest i with Cutoffs[i] >= length (draft §3.7); Cutoffs[i] in [32*i +: 32]
function [2:0] cut_prio(input [255:0] c, input [31:0] len);
    integer q;
    begin
        cut_prio = 3'd0;
        for (q = 0; q < 8; q = q + 1)
            if (c[32*q +: 32] >= len) cut_prio = q[2:0];
    end
endfunction
