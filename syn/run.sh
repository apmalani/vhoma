#!/bin/sh
# Synthesis, placement and routing for a Lattice ECP5-5G (LFE5UM5G-85F-8BG381) with the open-source
# flow: Yosys and nextpnr-ecp5 (e.g. from the YosysHQ OSS CAD Suite), on PATH. Target 156.25 MHz:
# 64 bits per cycle is 10 Gb/s. Outputs go to build/syn.
set -e
cd "$(dirname "$0")/.."
out=build/syn
mkdir -p $out
rtl=$(grep -v ipv4 sim/rtl_files.txt | tr '\n' ' ')

# resources: each design alone at its default parameters, ports left free
yosys -q -l $out/endpoint.log -p "read_verilog -I rtl $rtl; synth_ecp5 -top homa_endpoint; tee -q -o $out/endpoint.txt stat"
yosys -q -l $out/ipv4.log -p "read_verilog rtl/homa_ipv4.v; synth_ecp5 -top homa_ipv4; tee -q -o $out/ipv4.txt stat"

# timing: the endpoint in a pin-saving harness (syn/homa_top.v), placed and routed
yosys -q -l $out/top.log -p "read_verilog -I rtl $rtl syn/homa_top.v; synth_ecp5 -top homa_top -json $out/top.json"
nextpnr-ecp5 --um5g-85k --package CABGA381 --speed 8 --freq 156.25 --lpf-allow-unconstrained --timing-allow-fail \
    --json $out/top.json --report $out/report.json --log $out/pnr.log --quiet

for d in endpoint ipv4; do
    echo "$d:"
    grep -E '^ +[0-9]+ +[A-Z_0-9]+$' $out/$d.txt | sed 's/^ */  /'
done
grep -E 'Max frequency' $out/pnr.log | tail -1
