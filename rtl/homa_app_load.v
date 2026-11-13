// Application send path: takes a request or response, gets a slot from the core,
// writes the payload into the transmit buffer, then tells the core it is ready.
module homa_app_load #(
    parameter MAX_MSG = 16384,
    parameter NRPC    = 8
) (
    input  wire              clk,
    input  wire              rst,

    input  wire              s_valid,
    output wire              s_ready,
    input  wire              s_resp,        // 1: response, s_handle names the request
    input  wire [31:0]       s_addr,        // request: server address
    input  wire [15:0]       s_rport,       // request: server port
    input  wire [15:0]       s_lport,       // request: our port
    input  wire [$clog2(NRPC)+7:0] s_handle,
    input  wire [31:0]       s_len,
    // exactly ceil(len/8) beats follow each accepted descriptor, byte 0 in [7:0]
    input  wire [63:0]       s_tdata,
    input  wire              s_tvalid,
    output wire              s_tready,

    output reg               st_valid,      // pulse per descriptor
    output reg               st_ok,
    output reg               st_resp,
    output reg  [63:0]       st_id,
    output reg  [$clog2(NRPC)+7:0] st_handle,

    output wire              ld_req,
    output reg               ld_resp,
    output reg  [31:0]       ld_addr,
    output reg  [15:0]       ld_rport,
    output reg  [15:0]       ld_lport,
    output reg  [$clog2(NRPC)+7:0] ld_handle,
    output reg  [31:0]       ld_mlen,
    input  wire              ld_ack,
    input  wire              ld_ok,
    input  wire [$clog2(NRPC)+7:0] ld_hout,
    input  wire [63:0]       ld_id,
    output wire              fin_req,
    output wire [$clog2(NRPC)-1:0] fin_slot,
    input  wire              fin_ack,

    output wire              tb_we,
    output wire [$clog2(NRPC)+$clog2(MAX_MSG)-4:0] tb_waddr,
    output wire [63:0]       tb_wdata
);

localparam SL     = $clog2(NRPC);
localparam MSG_LG = $clog2(MAX_MSG);

localparam [2:0] IDLE = 3'd0, REQ = 3'd1, LOAD = 3'd2, FIN = 3'd3, SKIP = 3'd4;
reg  [2:0]    st;
reg  [SL-1:0] slot;
reg  [31:0]   left;      // payload bytes still to arrive
reg  [MSG_LG-3:0] w;

assign s_ready  = (st == IDLE);
assign ld_req   = (st == REQ);
assign fin_req  = (st == FIN);
assign fin_slot = slot;
assign s_tready = (st == LOAD) || (st == SKIP);
assign tb_we    = (st == LOAD) && s_tvalid;
assign tb_waddr = (slot << (MSG_LG - 3)) + w;
assign tb_wdata = s_tdata;

always @(posedge clk) begin
    st_valid <= 1'b0;
    if (rst) begin
        st <= IDLE;
    end else begin
        case (st)
        IDLE: if (s_valid) begin
            ld_resp <= s_resp; ld_addr <= s_addr; ld_rport <= s_rport; ld_lport <= s_lport;
            ld_handle <= s_handle; ld_mlen <= s_len; left <= s_len;
            st <= REQ;
        end
        REQ: if (ld_ack) begin
            st_valid <= 1'b1; st_ok <= ld_ok; st_resp <= ld_resp; st_id <= ld_id; st_handle <= ld_hout;
            slot <= ld_hout[SL-1:0];
            w <= 0;
            st <= (left == 0) ? IDLE : ld_ok ? LOAD : SKIP;
        end
        LOAD: if (s_tvalid) begin
            w <= w + 1'b1;
            left <= left - 32'd8;
            if (left <= 32'd8) st <= FIN;
        end
        FIN: if (fin_ack) st <= IDLE;
        SKIP: if (s_tvalid) begin
            left <= left - 32'd8;
            if (left <= 32'd8) st <= IDLE;
        end
        default: st <= IDLE;
        endcase
    end
end

endmodule
