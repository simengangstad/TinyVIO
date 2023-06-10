file build/core0.elf
target remote localhost:2331
monitor reset
monitor halt
break main
load
c
layout asm
