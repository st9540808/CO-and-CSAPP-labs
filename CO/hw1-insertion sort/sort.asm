# insertion sort
.text
.globl main

main:
	j init

# $a0: start of array  $a1: end of element
sort:
	add $s1, $s0, 4 # get address of A[i] (expr1)
for:
	beq $s1, $a1, forend # jump to forend if &A[i]==end of address(expr2)
	# loop body
	add $t3, $zero, $s1 # $t3: address of A[j = i]
	
	while:
		sub $t7, $t3, $a0
		bltz $t7, while_end
		lw $t4, 0($t3)  # value of A[j]
		lw $t5, -4($t3) # value of A[j-1]
		slt $t2, $t4, $t5
		slt $t2, $t4, $t5
		slt $t2, $t4, $t5
		slt $t2, $t4, $t5
		beq $t2, $zero, while_end
		sw $t0, -4($a0)
		sw $t5, 0($t3)
		sw $t4, -4($t3)
		addi $t3, $t3, -4
		j while
while_end:
	add $t1, $zero, $s1
	addi $s1, $s1, 4 # get address of A[++i] (expr3)
	j for
forend:
	j exit

init:
	lui $s0, 0x1001
	add $s1, $zero, 3
	sw $s1, 0($s0)
	add $s1, $zero, 7
	sw $s1, 4($s0)
	add $s1, $zero, 10
	sw $s1, 8($s0)
	add $s1, $zero, 2
	sw $s1, 12($s0)
	add $s1, $zero, 9
	sw $s1, 16($s0)
	add $s1, $zero, 8
	sw $s1, 20($s0)
	add $s1, $zero, 6
	sw $s1, 24($s0)
	add $s1, $zero, 4
	sw $s1, 28($s0)
	add $s1, $zero, 5
	sw $s1, 32($s0)
	add $s1, $zero, 1
	sw $s1, 36($s0)
	addi $ra, $ra, -4
	add $a0, $zero, $s0
	addi $a1, $s0, 40
	j sort

exit:jr $ra
