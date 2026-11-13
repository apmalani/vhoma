#!/bin/sh
# Runs the testbench across several parameter sets. Usage: sim/run_all.sh
set -e
cd "$(dirname "$0")/.."
mkdir -p build
run() {
    iverilog -g2005 -o build/tb_cfg.vvp "$@" sim/tb_homa_unsched.v rtl/homa_tx_unsched.v rtl/homa_rx_unsched.v
    vvp -n build/tb_cfg.vvp | grep -E "DONE|TIMEOUT"
}
run -Ptb.SEG=64   -Ptb.MAXM=512   -Ptb.NSLOT=4  -Ptb.TICK=400   -Ptb.SEED=1
run -Ptb.SEG=64   -Ptb.MAXM=512   -Ptb.NSLOT=4  -Ptb.TICK=400   -Ptb.SEED=7
run -Ptb.SEG=8    -Ptb.MAXM=8     -Ptb.NSLOT=2  -Ptb.TICK=64    -Ptb.AGE=2
run -Ptb.SEG=8    -Ptb.MAXM=64    -Ptb.NSLOT=2  -Ptb.TICK=200
run -Ptb.SEG=16   -Ptb.MAXM=512   -Ptb.NSLOT=16 -Ptb.TICK=6000  -Ptb.PMAX=9000 -Ptb.ROUNDS=15
run -Ptb.SEG=64   -Ptb.MAXM=4096  -Ptb.NSLOT=4  -Ptb.TICK=4000  -Ptb.NTX=60 -Ptb.ROUNDS=15
run -Ptb.SEG=1024 -Ptb.MAXM=16384 -Ptb.NSLOT=8  -Ptb.TICK=20000 -Ptb.NTX=40 -Ptb.PMAX=1500 -Ptb.ROUNDS=8
