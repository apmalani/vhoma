// Homa receive path: parses incoming packets, writes DATA payload into the
// receive buffer, and hands every other packet's header to the core as an event.
// Input starts at the Homa common header. Never stalls: bad packets are dropped.
module homa_rx_ingress #(
    parameter SEG_BYTES = 1024,
    parameter MAX_MSG   = 16384,
    parameter NRPC      = 8
) (
    input  wire                 clk,
    input  wire                 rst,

    input  wire [63:0]          s_tdata,
    input  wire [7:0]           s_tkeep,
    input  wire                 s_tvalid,
    input  wire                 s_tlast,
    input  wire                 s_tuser,      // packet is bad (e.g. FCS error), valid with tlast
    output wire                 s_tready,
    input  wire [31:0]          s_saddr,      // constant per packet

    // DATA admission, decided combinationally by the core on beat 6
    output wire                 adm_req,
    input  wire                 adm_ok,
    input  wire [$clog2(NRPC)-1:0] adm_slot,
    // DATA header fields: valid from beat 6 through tlast
    output wire [31:0]          d_saddr,
    output wire [15:0]          d_sport,
    output wire [15:0]          d_dport,
    output wire [63:0]          d_id,
    output wire [31:0]          d_len,
    output wire [31:0]          d_off,
    output wire [63:0]          d_ackid,
    output wire [15:0]          d_cver,
    output wire [15:0]          d_plen,
    // DATA commit on tlast
    output wire                 cmt_req,
    output wire                 cmt_ok,
    output wire [$clog2(NRPC)-1:0] cmt_slot,

    // other packets: header bytes 0..79 on tlast
    output wire                 ev_push,
    output wire [639:0]         ev_hdr,
    output wire [31:0]          ev_saddr,
    output wire [15:0]          ev_len,
    input  wire                 ev_full,

    output wire                 rb_we,
    output wire [$clog2(NRPC)+$clog2(MAX_MSG)-4:0] rb_waddr,
    output wire [63:0]          rb_wdata,

    output reg                  drop
);

`include "homa_defs.vh"

localparam SL     = $clog2(NRPC);
localparam SEG_LG = $clog2(SEG_BYTES);
localparam MSG_LG = $clog2(MAX_MSG);
localparam SEGW   = SEG_BYTES / 8;
localparam AW     = SL + MSG_LG - 3;
localparam BCM    = (SEGW + 7 > 10) ? SEGW + 7 : 10;   // saturates one beat past the longest DATA packet
localparam BW     = $clog2(BCM + 1);

function [3:0] ones(input [7:0] k);
    integer j;
    begin
        ones = 4'd0;
        for (j = 0; j < 8; j = j + 1) ones = ones + k[j];
    end
endfunction

assign s_tready = 1'b1;
wire [63:0] d = s_tdata;
wire        v = s_tvalid;

reg  [BW-1:0] bc;
reg  [639:0]  hdr;
reg           kbad;
reg           r_acc;
reg  [SL-1:0] r_slot;
reg  [31:0]   r_off;
reg  [31:0]   resid;

// fields from beats already captured; beat 6 offset comes straight off the bus
wire [7:0]  typ   = hdr[8*11 +: 8];
assign d_saddr = s_saddr;
assign d_sport = hb16(hdr, 0);
assign d_dport = hb16(hdr, 2);
assign d_id    = hb64(hdr, 20);
assign d_len   = hb32(hdr, 28);
assign d_ackid = hb64(hdr, 32);
assign d_cver  = hb16(hdr, 42);

wire        at6   = v && (bc == 6);
wire [31:0] off6  = be32(d[31:0]);
wire        sane6 = (typ == T_DATA) && (d_len != 0) && (d_len <= MAX_MSG) &&
                    (off6[SEG_LG-1:0] == 0) && (off6 < d_len);
assign adm_req = at6 && sane6;

wire          c_acc  = at6 ? (sane6 && adm_ok) : r_acc;
wire [SL-1:0] c_slot = at6 ? adm_slot : r_slot;
assign d_off    = at6 ? off6 : r_off;
assign cmt_slot = c_slot;

wire [3:0]  kc      = ones(s_tkeep);
wire        keep_ok = (s_tkeep != 0) && ((s_tkeep & (s_tkeep + 8'd1)) == 0) && !kbad && !s_tuser;
wire [31:0] nbytes  = {bc, 3'b000} + kc;
wire [31:0] rem     = d_len - d_off;
wire [31:0] plen    = (rem > SEG_BYTES) ? SEG_BYTES : rem;
assign d_plen = plen[15:0];

assign cmt_req = v && s_tlast && c_acc;
assign cmt_ok  = keep_ok && (nbytes == plen + 52);

// non-DATA packets: minimum lengths per draft §3
reg [7:0] minlen;
always @* begin
    case (typ)
    T_START:    minlen = 8'd32;
    T_GRANT:    minlen = 8'd33;
    T_RESEND:   minlen = 8'd37;
    T_UNKNOWN,
    T_BUSY,
    T_NEED_ACK: minlen = 8'd28;
    T_CUTOFFS:  minlen = 8'd62;
    T_ACK:      minlen = 8'd30;
    default:    minlen = 8'd0;
    endcase
end
// header of the current packet including this beat
wire [639:0] hdr_now = (bc < 10) ? (hdr & ~({576'd0, 64'hFFFF_FFFF_FFFF_FFFF} << (64*bc))) |
                                   ({576'd0, d} << (64*bc)) : hdr;
wire ctl_ok = (minlen != 0) && keep_ok && (nbytes >= minlen) && (bc >= 3);
assign ev_push  = v && s_tlast && (typ != T_DATA) && ctl_ok && !ev_full;
assign ev_hdr   = hdr_now;
assign ev_saddr = s_saddr;
assign ev_len   = (nbytes > 16'hFFFF) ? 16'hFFFF : nbytes[15:0];

// payload starts 4 bytes into beat 6; realign to 8-byte words
reg           fl_v;
reg  [AW-1:0] fl_addr;
reg  [31:0]   fl_data;
wire [31:0]   widx    = bc - 7;
wire [AW-1:0] r_base  = (r_slot << (MSG_LG - 3)) + (r_off >> 3);
wire          beat_we = v && r_acc && (bc >= 7) && (widx < SEGW);
assign rb_we    = fl_v || beat_we;
assign rb_waddr = fl_v ? fl_addr : r_base + widx;
assign rb_wdata = fl_v ? {32'd0, fl_data} : {d[31:0], resid};

always @(posedge clk) begin
    drop <= 1'b0;
    fl_v <= 1'b0;
    if (rst) begin
        bc    <= 0;
        r_acc <= 1'b0;
        kbad  <= 1'b0;
    end else if (v) begin
        if (bc < 10) hdr[64*bc +: 64] <= d;
        if (bc == 6) begin
            r_acc  <= sane6 && adm_ok;
            r_slot <= adm_slot;
            r_off  <= off6;
        end
        resid <= d[63:32];
        if (!s_tlast && s_tkeep != 8'hFF) kbad <= 1'b1;
        if (s_tlast) begin
            bc      <= 0;
            r_acc   <= 1'b0;
            kbad    <= 1'b0;
            drop    <= (typ == T_DATA) ? !(c_acc && cmt_ok) : !ev_push;
            // last beat may hold up to 4 more payload bytes: write them next cycle
            fl_v    <= c_acc && cmt_ok && (kc > 4);
            fl_addr <= (c_slot << (MSG_LG - 3)) + (d_off >> 3) + (bc - 6);
            fl_data <= d[63:32];
        end else if (bc != BCM) begin
            bc <= bc + 1'b1;
        end
    end
end

endmodule
