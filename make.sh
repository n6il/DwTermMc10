#!/bin/bash -x
AS="/Users/n6il/Development/Mc10/motorola-6800-assembler/bin/as1"
"${AS}" dwterm.asm \
	&&  "${AS}" dwterm.asm -l >dwterm.lst \
	|| exit 1
makewav -k -d0x4e20 -e0x4e20 -nDWTERM -odwterm.cas dwterm.s19
fixcas.py dwterm.cas dwterm.c10
cecb copy -f dwterm.s19 dwterm.bin -d0x4e20 -e0x4e20
zip dwterm-$(date +'%Y%m%d-%H%M%S').zip dwterm.*
