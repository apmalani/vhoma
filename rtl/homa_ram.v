// Simple dual-port RAM: one write port, one registered read port.
module homa_ram #(
    parameter AW = 10,
    parameter DW = 64
) (
    input  wire          clk,
    input  wire          we,
    input  wire [AW-1:0] waddr,
    input  wire [DW-1:0] wdata,
    input  wire [AW-1:0] raddr,
    output reg  [DW-1:0] rdata
);

reg [DW-1:0] mem [0:(1<<AW)-1];

always @(posedge clk) begin
    if (we) mem[waddr] <= wdata;
    rdata <= mem[raddr];
end

endmodule
