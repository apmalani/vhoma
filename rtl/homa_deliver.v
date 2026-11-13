// Delivers completed messages from the receive buffer to the application, oldest first.
module homa_deliver #(
    parameter MAX_MSG = 16384,
    parameter NRPC    = 8
) (
    input  wire              clk,
    input  wire              rst,

    input  wire              dq_push,
    input  wire [$clog2(NRPC)+7:0] dq_handle,
    input  wire              dq_resp,
    input  wire [31:0]       dq_addr,
    input  wire [15:0]       dq_rport,
    input  wire [15:0]       dq_lport,
    input  wire [63:0]       dq_id,
    input  wire [31:0]       dq_len,
    output wire              dd_req,
    output wire [$clog2(NRPC)-1:0] dd_slot,
    input  wire              dd_ack,

    output wire [$clog2(NRPC)+$clog2(MAX_MSG)-4:0] rb_raddr,
    input  wire [63:0]       rb_rdata,

    output wire [63:0]       m_tdata,
    output wire [7:0]        m_tkeep,
    output wire              m_tvalid,
    output wire              m_tlast,
    input  wire              m_tready,
    output wire              m_resp,        // sideband, constant per message
    output wire [31:0]       m_addr,
    output wire [15:0]       m_rport,
    output wire [15:0]       m_lport,
    output wire [63:0]       m_id,
    output wire [$clog2(NRPC)+7:0] m_handle,      // for a request: pass back with the response
    output wire [31:0]       m_len
);

localparam SL     = $clog2(NRPC);
localparam MSG_LG = $clog2(MAX_MSG);
localparam AW     = SL + MSG_LG - 3;
localparam QW     = SL + 8 + 1 + 32 + 16 + 16 + 64 + 32;

// one entry per slot at most: a slot is queued once and stays locked until done
reg  [QW-1:0] q [0:NRPC-1];
reg  [SL:0]   wp, rp;
reg  [QW-1:0] cur;
reg           act, dd_p;
reg  [MSG_LG-2:0] left;   // words
reg  [AW-1:0] raddr;

wire q_ne  = (wp != rp);
wire start = !act && !dd_p && q_ne;
wire hs    = act && m_tready;
wire [QW-1:0]  head     = q[rp[SL-1:0]];
wire [SL+7:0]  h_handle = head[QW-1 -: SL+8];
wire [31:0]    h_len    = head[31:0];

assign {m_handle, m_resp, m_addr, m_rport, m_lport, m_id, m_len} = cur;
wire [SL-1:0] cslot = m_handle[SL-1:0];

assign rb_raddr = start ? (h_handle[SL-1:0] << (MSG_LG - 3)) : hs ? raddr + 1'b1 : raddr;
assign m_tdata  = rb_rdata;
assign m_tvalid = act;
assign m_tlast  = (left == 1);
assign m_tkeep  = (m_tlast && m_len[2:0] != 0) ? ~(8'hFF << m_len[2:0]) : 8'hFF;
assign dd_req   = dd_p;
assign dd_slot  = cslot;

always @(posedge clk) begin
    if (dq_push) q[wp[SL-1:0]] <= {dq_handle, dq_resp, dq_addr, dq_rport, dq_lport, dq_id, dq_len};
    if (rst) begin
        wp <= 0; rp <= 0; act <= 1'b0; dd_p <= 1'b0;
    end else begin
        if (dq_push) wp <= wp + 1'b1;
        raddr <= rb_raddr;
        if (start) begin
            cur  <= head;
            rp   <= rp + 1'b1;
            act  <= 1'b1;
            left <= (h_len + 7) >> 3;
        end else if (hs) begin
            left <= left - 1'b1;
            if (left == 1) begin act <= 1'b0; dd_p <= 1'b1; end
        end
        if (dd_ack) dd_p <= 1'b0;
    end
end

endmodule
