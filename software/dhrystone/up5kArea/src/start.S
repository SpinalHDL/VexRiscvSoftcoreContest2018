	.section .init
	.globl _start
	.type _start,@function

_start:
.option push
.option norelax
	la gp, __global_pointer$
.option pop
	la sp, _sp


	/* Clear bss section */
	la a0, __bss_start
	la a1, __bss_end
	bgeu a0, a1, 2f
1:
	sw zero, (a0)
	addi a0, a0, 4
	bltu a0, a1, 1b
2:

	/* Call global constructors */
	la a0, __libc_fini_array
	call atexit
	call __libc_init_array

    //disable timer CMP
    li a0, 0x70018
    li a1, -1
    sw a1, 0(a0)

	call main
	li a0, 0xFF
	call putchar
mainDone:
    j mainDone


	.globl _init
	.globl _fini
_init:
_fini:
    ret

