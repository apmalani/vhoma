// Homa sender, unscheduled path: splits each message into DATA packets
// (draft-ousterhout-tsvwg-homa-00 §3.1) and sends them without waiting for grants.
// Output starts at the Homa common header; IP framing is done downstream.
module homa_tx_unsched #(
    parameter SEG_BYTES = 1024,   // payload bytes per DATA packet: power of 2, 8..32768
    parameter MAX_MSG   = 16384   // unscheduled limit: longest message accepted, bytes
) (
    input  wire         clk,
    input  wire         rst,

    input  wire         s_msg_valid,
    output wire         s_msg_ready,
    input  wire [31:0]  s_msg_daddr,
    input  wire [15:0]  s_msg_sport,
    input  wire [15:0]  s_msg_dport,
    input  wire [63:0]  s_msg_id,        // Rpcid as sent; bit 0 is S
    input  wire [31:0]  s_msg_len,

    // message bytes: exactly ceil(len/8) beats after the descriptor, byte 0 in [7:0]
    input  wire [63:0]  s_axis_tdata,
    input  wire         s_axis_tvalid,
    output wire         s_axis_tready,

    output reg  [63:0]  m_axis_tdata,
    output reg  [7:0]   m_axis_tkeep,
    output reg          m_axis_tvalid,
    output reg          m_axis_tlast,
    input  wire         m_axis_tready,
    output wire [31:0]  m_axis_daddr,    // sideband, constant per packet
    output wire [15:0]  m_axis_len,      // Homa header + payload bytes
    output wire [2:0]   m_axis_prio,

    input  wire [255:0] cfg_cutoffs,     // Cutoffs[i] in [32*i +: 32]
    input  wire [15:0]  cfg_cutoff_ver,

    output reg          err_len          // pulse: len was 0 or > MAX_MSG; payload discarded
);

localparam [7:0]  TYPE_DATA = 8'd16;
localparam [15:0] HDR_BYTES = 16'd52;
localparam [15:0] SEG16     = SEG_BYTES;
localparam [31:0] MAX32     = MAX_MSG;
localparam [1:0]  IDLE = 2'd0, HEAD = 2'd1, PAY = 2'd2, DROP = 2'd3;

// elaboration fails here if a parameter is out of range
generate
    if (SEG_BYTES < 8 || SEG_BYTES > 32768 || (SEG_BYTES & (SEG_BYTES - 1)) != 0 || MAX_MSG < 1) begin : bad_parameter
        homa_tx_unsched_invalid_parameter u_fail ();
    end
endgenerate

function [15:0] be16(input [15:0] v); be16 = {v[7:0], v[15:8]}; endfunction
function [31:0] be32(input [31:0] v); be32 = {v[7:0], v[15:8], v[23:16], v[31:24]}; endfunction

reg  [1:0]  state;
reg  [2:0]  hb;
reg  [31:0] daddr, len, off, left;
reg  [15:0] sport, dport, cver, plen, pleft;
reg  [63:0] id;
reg  [2:0]  prio;
reg  [31:0] resid;
reg         first, flush;
reg  [2:0]  fbytes;

assign s_msg_ready   = (state == IDLE);
assign s_axis_tready = (state == DROP) || (state == PAY && !flush && m_axis_tready);
assign m_axis_daddr  = daddr;
assign m_axis_len    = HDR_BYTES + plen;
assign m_axis_prio   = prio;

// unscheduled priority: largest i with Cutoffs[i] >= message length (draft §3.7)
reg [2:0] new_prio;
integer i;
always @* begin
    new_prio = 3'd0;
    for (i = 0; i < 8; i = i + 1)
        if (cfg_cutoffs[32*i +: 32] >= s_msg_len) new_prio = i[2:0];
end

wire [15:0] first_plen = (s_msg_len > SEG16) ? SEG16 : s_msg_len[15:0];
wire [31:0] left_after = left - plen;
wire [15:0] next_plen  = (left_after > SEG16) ? SEG16 : left_after[15:0];
wire        last_in    = (pleft <= 16'd8);

reg [63:0] hdr;
always @* begin
    case (hb)
    3'd0:    hdr = {32'd0, be16(dport), be16(sport)};
    3'd1:    hdr = {32'd0, TYPE_DATA, 24'd0};
    3'd2:    hdr = {be32(id[63:32]), 32'd0};
    3'd3:    hdr = {be32(len), be32(id[31:0])};
    3'd4:    hdr = 64'd0;
    default: hdr = {32'd0, be16(cver), 16'd0};
    endcase
end

always @* begin
    m_axis_tdata  = hdr;
    m_axis_tkeep  = 8'hFF;
    m_axis_tvalid = 1'b0;
    m_axis_tlast  = 1'b0;
    if (state == HEAD) begin
        m_axis_tvalid = 1'b1;
    end else if (state == PAY) begin
        if (flush) begin
            m_axis_tdata  = {32'd0, resid};
            m_axis_tkeep  = ~(8'hFF << fbytes);
            m_axis_tvalid = 1'b1;
            m_axis_tlast  = 1'b1;
        end else begin
            // 52-byte header leaves payload 4 bytes past a beat boundary
            m_axis_tdata  = {s_axis_tdata[31:0], first ? be32(off) : resid};
            m_axis_tvalid = s_axis_tvalid;
            if (last_in && pleft <= 16'd4) begin
                m_axis_tkeep = ~(8'hFF << ({1'b0, pleft[2:0]} + 4'd4));
                m_axis_tlast = 1'b1;
            end
        end
    end
end

wire hs = m_axis_tvalid && m_axis_tready;

always @(posedge clk) begin
    err_len <= 1'b0;
    if (rst) begin
        state <= IDLE;
        flush <= 1'b0;
    end else begin
        case (state)
        IDLE: if (s_msg_valid) begin
            daddr <= s_msg_daddr;
            sport <= s_msg_sport;
            dport <= s_msg_dport;
            id    <= s_msg_id;
            len   <= s_msg_len;
            left  <= s_msg_len;
            cver  <= cfg_cutoff_ver;
            prio  <= new_prio;
            off   <= 32'd0;
            plen  <= first_plen;
            pleft <= first_plen;
            hb    <= 3'd0;
            if (s_msg_len == 32'd0 || s_msg_len > MAX32) begin
                err_len <= 1'b1;
                state   <= (s_msg_len == 32'd0) ? IDLE : DROP;
            end else
                state <= HEAD;
        end
        HEAD: if (hs) begin
            hb <= hb + 3'd1;
            if (hb == 3'd5) begin
                state <= PAY;
                first <= 1'b1;
            end
        end
        PAY: if (hs) begin
            if (!flush) begin
                resid <= s_axis_tdata[63:32];
                first <= 1'b0;
                pleft <= pleft - 16'd8;
                if (last_in && pleft > 16'd4) begin
                    flush  <= 1'b1;
                    fbytes <= pleft[2:0] - 3'd4;
                end
            end
            if (m_axis_tlast) begin
                flush <= 1'b0;
                off   <= off + plen;
                left  <= left_after;
                plen  <= next_plen;
                pleft <= next_plen;
                hb    <= 3'd0;
                state <= (left_after == 32'd0) ? IDLE : HEAD;
            end
        end
        DROP: if (s_axis_tvalid) begin
            left <= left - 32'd8;
            if (left <= 32'd8) state <= IDLE;
        end
        endcase
    end
end

endmodule
