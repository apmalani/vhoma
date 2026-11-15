# vhoma

My initial implementation of [Homa](https://datatracker.ietf.org/doc/draft-ousterhout-tsvwg-homa/)
in Verilog-2005: a minimal RPC transport endpoint for FPGAs, verified in simulation.

## What's here

- `rtl/homa_endpoint.v`: one endpoint. Requests and responses up to 16 KB; the first bytes go out
  unscheduled, the rest as the receiver grants them (shortest remaining first); priorities from
  CUTOFFS; recovery with RESEND, BUSY, NEED_ACK, ACK and UNKNOWN. 64-bit AXI-Stream on the network
  and application sides.
- `rtl/homa_ipv4.v`: IPv4 framing (IP protocol 146).
- `rtl/homa_tx_unsched.v`, `rtl/homa_rx_unsched.v`: standalone unscheduled-only sender and receiver.
- `sim/`: Icarus Verilog testbenches. The system test runs three endpoints over a network that
  loses, duplicates, reorders and corrupts packets, and checks every byte end to end.
- `syn/`: synthesis, placement and routing for a Lattice ECP5.

## Run

```
sh sim/run_full.sh    # full regression
sh sim/run_all.sh     # unscheduled sender and receiver
waterski run chaos    # replay a lossy run in the terminal (github.com/apmalani/waterski)
waterski run wave     # record build/wave.vcd: three FPGAs, one RPC each
sh syn/run.sh         # needs Yosys and nextpnr-ecp5 on PATH (YosysHQ OSS CAD Suite)
```

To see the recording in VaporView, open `build/wave.vcd` and load `sim/wave.json` with
"Load Vaporview Session".

## Status

Synthesized for an ECP5-5G 85F (LFE5UM5G-85F-8BG381) at the default parameters:

|          | LUT4   | FF    | Block RAM  |
|----------|--------|-------|------------|
| endpoint | 16,762 | 5,247 | 128 of 208 |
| IPv4     | 1,027  | 392   | 0          |

22 MHz after place and route. It has not run on a board, which also needs an Ethernet MAC around `homa_ipv4`.
