# README #

# Purpose #
This repository holds the verilog files for the BLDC1 project. It implements a dual loop digital PLL (DPLL) for a polygon using a brush-less 2.5in BLDC motor. The inner loop is a velocity control loop and the outer loop is a phase locked loop.

A discussion of the design is on my website at https://TremaineConsultingGroup.com in the blog section.

File Top.BLDC.v is the top level verilog file for implementing in the Xilinx Artix 7 using a
custom development board 024-00300-03

Files were bench tested using the the Veritakwin simulator.

## 
How do I get set up? ##
* Tool-chain environment Windows 10 64-bit
* Project: Xilinx Vivado 2018.3
* Simulation: Veritakwin (http://www.sugawara-systems.com/download.htm)

## Contribution guidelines ##
* Writing tests
* Code review
* Other guidelines

## Who do I talk to? ##
* Brian Tremaine btremaine@gmail.com
* (cell) 669-273-6035 