; Copyright(c) 2023 Alexander Galilov, alexander.galilov@gmail.com
; This code is a preemptive multitasking DEMO for i8086 Turbo Assembler 2.01
;
; Permission is hereby granted, free of charge, to any person obtaining a copy of this software
; and associated documentation files(the "Software"), to deal in the Software without restriction,
; including without limitation the rights to use, copy, modify, merge, publish, distribute,
; sublicense, and /or sell copies of the Software, and to permit persons to whom the Software is
; furnished to do so, subject to the following conditions :
; The above copyright noticeand this permission notice shall be included in all copies or
; substantial portions of the Software.
;
; THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
; NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
; NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
; DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
; OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
;
; BUILD with Borland Turbo Assembler 2.01:
; D:\TASM\tasm demo.asm
; D:\TASM\tlink /t demo.obj
;
; RUN:
; demo.com
;

; ===== Macro command which saves all the required registers to stack
pushregs macro
		push	ax						; *((ss << 4) + (--sp)) = ax
		push	bx						; *((ss << 4) + (--sp)) = bx
		push	cx						; *((ss << 4) + (--sp)) = cx
		push	dx						; *((ss << 4) + (--sp)) = dx
		push	si						; *((ss << 4) + (--sp)) = si
		push	di						; *((ss << 4) + (--sp)) = di
		push	bp						; *((ss << 4) + (--sp)) = bp
		push	es						; *((ss << 4) + (--sp)) = es
		push	ds						; *((ss << 4) + (--sp)) = ds
endm

; ===== Macro command which loads the previously saved registers from stack
popregs macro
		pop		ds						; ds = *((ss << 4) + (sp++))
		pop		es						; es = *((ss << 4) + (sp++))
		pop		bp						; bp = *((ss << 4) + (sp++))
		pop		di						; di = *((ss << 4) + (sp++))
		pop		si						; si = *((ss << 4) + (sp++))
		pop		dx						; dx = *((ss << 4) + (sp++))
		pop		cx						; cx = *((ss << 4) + (sp++))
		pop		bx						; bx = *((ss << 4) + (sp++))
		pop		ax						; ax = *((ss << 4) + (sp++))
endm

		locals							; Enable of using @@local labels
; We use TINY memory model allowing 64K for code, data, and stack.
; All of CS, DS, and SS were set to the same value. This is the memory model
; that COM files use. 
		.model tiny
		.code							; Start the code segment.
		org 100h						; Offset for the code in tiny model.

Start:	jmp		Main					; Go to main code.

TimerVector			equ		8			; constant: Timer interrupt vector
TaskStackSize		equ		64			; constant: Size of the task stack, bytes
NTasks				equ		3			; constant: Number of tasks
SkipInterrupts		equ		20			; constant: How many timer ticks should be provided to each task.
TaskStacks			db		(TaskStackSize * NTasks) dup (?)	; The memory block for stacks
TaskStackPointers	dw		NTasks dup (?)	; Array of NTasks words to store task-specific stack pointers.
DefaultStackPointer	dw		?			; Word wariable to store a stack pointer for the main (startup) code.
CurrentTaskIndex	db		-1			; Byte variable to store currently executing task number.
AddedTaskCounter	db		0			; Byte variable used as counter of added tasks.
PrevTimerVector		dd		?			; Double word variable to save the original Timer interrupt vector here.
TimerCounter		dw		SkipInterrupts	; Counter used to count the nubler of timer ticks provided to each task.
; ===== Task1
Task1	proc
		mov		ax, 0b800h				; ax = 0xB800, a segment address of a text line 0
		mov		es, ax					; es = ax
		mov		ah, 04h					; ah = 04h 
		mov		al, 'A'					; al = 'A'
@@L1:
		mov		bx, 32					; bx = 32
		cli
@@repeat:
		sub		bx, 2					; bx -= 2
		mov		word ptr es:[bx], ax	; *((es << 4) + bx) = ax
		loopnz	@@repeat				; if bx != 0 then goto @@repeat
		sti
		inc		ah						; ah++
		and		ah, 07fh				; ah &= 0x07  clear 7-th bit in ah
		jmp		@@L1					; goto @@L1
Task1	endp
; ===== Task2
Task2	proc
		mov		ax, 0b80Ah				; ax = 0xB80A, a segment address of a text line 1
		mov		es, ax					; es = ax
		mov		ah, 02h					; ah = 02h
		mov		al, 'B'					; al = 'B'
@@L1:
		mov		bx, 32					; bx -= 2
		cli
@@repeat:
		sub		bx, 2					; bx -= 2
		mov		word ptr es:[bx], ax	; *((es << 4) + bx) = ax
		loopnz	@@repeat				; if bx != 0 then goto @@repeat
		sti
		inc		ah						; ah++
		and		ah, 07fh				; ah &= 0x07  clear 7-th bit in ah
		jmp		@@L1					; goto @@L1
Task2	endp
; ===== Task3
Task3	proc
		mov		ax, 0b814h				; ax = 0xB814, a segment address of a text line 2
		mov		es, ax					; es = ax
		mov		ah, 01h					; ah = 01h
		mov		al, 'C'					; al = 'C'
@@L1:
		mov		bx, 32					; bx -= 2
		cli
@@repeat:
		sub		bx, 2					; bx -= 2
		mov		word ptr es:[bx], ax	; *((es << 4) + bx) = ax
		loopnz	@@repeat				; if bx != 0 then goto @@repeat
		sti
		inc		ah						; ah++
		and		ah, 07fh				; ah &= 0x07  clear 7-th bit in ah
		jmp		@@L1					; goto @@L1
Task3	endp
; ===== Calculate the address to store a stack pointer for specific task
; Input: 
;     BL: task index
; Output:
;     BX: Task-specific Stack pointer address
GetTaskSPAddr proc
		; bx = bl * 2 + &TaskStackPointers
		xor		bh, bh					; bh = 0
		shl		bx, 1					; bx <<= 1
		add		bx, offset TaskStackPointers 	; bx += &TaskStackPointers
		ret								; return
GetTaskSPAddr endp
; ==== Adds a new task to multitasking manager
; Input:
;     DX: task offset
AddTask proc
		; Save the current task counter to BL for future usage.
		mov		bl, AddedTaskCounter	; bl = *AddedTaskCounter
		; Calculate the task stack pointer address.
		inc		AddedTaskCounter		; (*AddedTaskCounter)++
		mov		al, AddedTaskCounter	; al = *AddedTaskCounter
		mov		bh,	TaskStackSize		; bh = TaskStackSize
		mul		bh						; ax = al * bh
		add		ax, offset TaskStacks	; ax += &TaskStacks
		; Save the current value of SP to BP.
		mov		bp, sp					; bp = sp
		; Switch SP to the top of a task stack.
		mov		sp, ax					; sp = ax
		; Prepare the task stack to switch to the beginning of the task.
		pushf							; *((ss << 4) + (--sp)) = flags
		push	cs						; *((ss << 4) + (--sp)) = cs
		push	dx						; *((ss << 4) + (--sp)) = dx
		; Save reqired registers to be restored when switching to the task.
		pushregs						; put macro pushregs here
		; Save SP to a task-specific pointer variable.
		call	GetTaskSPAddr			; call GetTaskSPAddr
		mov		[bx], sp				; *(ds << 4) + bx) = sp
		; Restore SP from BP. Required to return back to a calling code.
		mov		sp, bp					; sp = bp
		ret								; return
AddTask endp
; ==== Time interrupt handler, the switching code. It is called every 55 ms.
TimerHandler proc
		; Save registers to a current stack.
		pushregs						; put macro pushregs here
		; Here we skip SkipInterrupts interrupts before swithching to another task.
		dec		TimerCounter			; *(TimerCounter)--
		jge		@@Pass					; if *TimerCounter == 0 goto @@Pass
		mov		TimerCounter, SkipInterrupts	; *TimerCounter = SkipInterrupts
		; Check if the handler interrupted the main code.
		cmp		CurrentTaskIndex, -1	; compare *CurrentTaskIndex to -1
		; If so, go to saving SP to DefaultStackPointer...
		je		@@SaveDefaultPointer	; if *CurrentTaskIndex == -1 goto @@SaveDefaultPointer
		; ...otherwise save SP to a task-specific pointer variable.
		mov		bl, CurrentTaskIndex	; bl = *CurrentTaskIndex
		call	GetTaskSPAddr			; call GetTaskSPAddr
		mov 	[bx], sp				; *((ds << 4) + bx) = sp
		jmp		@@NextTask				; goto @@NextTask
@@SaveDefaultPointer:
		mov		DefaultStackPointer, sp	; *DefaultStackPointer = sp
@@NextTask:
		; Select next task.
		inc		CurrentTaskIndex		; (*CurrentTaskIndex)++
		; Correct the task number: CurrentTaskIndex %= NTasks
		mov		ah, 0					; ah = 0
		mov		al, CurrentTaskIndex	; al = *CurrentTaskIndex
		mov		bl, NTasks				; bl = NTasks
		div		bl						; al = ax / bl , ah = ax % bl
		mov		CurrentTaskIndex, ah	; *CurrentTaskIndex = ah
		; Load SP from a task-specific pointer variable.
		mov		bl, ah					; bl = ah
		call	GetTaskSPAddr			; call GetTaskSPAddr
		mov		sp, [bx]				; sp = *((ds << 4_ + bx)
@@Pass:
		; Restore the registers from stack.
		popregs							; put macro popregs here
		; Prepare to call the default timer handler...
		pushf							; *((ss << 4) + (--sp)) = flags
		; ...call it.
		call dword ptr PrevTimerVector  ; call by far address segment:offset saved in PrevTimerVector
		; Return to the code pointed by CS:OFFSET saved in a current stack.
		; Flag register value is also saved in the stack and will be restored by IRET.
		iret							; Return from the interrupt.
TimerHandler endp
; ==== Startup code
Main:	; Add Task1 to switcher.
		mov		dx, offset Task1		; dx = &Task1
		call	AddTask					; call AddTask
		; Add Task2 to switcher.
		mov		dx, offset Task2		; dx = &Task2
		call	AddTask					; call Task2
		; Add Task3 to switcher.
		mov		dx, offset Task3		; dx = &Task3
		call	AddTask					; call Task3
		; Set the segment register ES to 0.
		xor		ax, ax					; ax = 0
		mov		es, ax					; es = ax
		; Save the current (system) timer handler address (segment:offset) to PrevTimerVector variable.
		mov		ax, word ptr es:[4 * TimerVector]	; ax = *((es << 4) + 4 * TimerVector)
		mov		word ptr PrevTimerVector, ax		; *PrevTimerVector = ax
		mov		ax, es:[4 * TimerVector + 2]		; ax = *((es << 4) + 4 * TimerVector + 2)
		mov		word ptr PrevTimerVector + 2, ax	;  *(PrevTimerVector + 2) = ax
		; Set the timer interrupt vector to a new handler TimerHandler.
		cli								; clear interrupt flag (disable interrupts)
		mov		word ptr es:[4 * TimerVector], offset TimerHandler	; *((es << 4) + 4 * TimerVector) = &TimerHandler
		mov		es:[4 * TimerVector + 2], cs		; *((es << 4) + 4 * TimerVector + 2) = cs
		sti								; set interrupt flag (enable interrupts)
		; Code stub. Do nothing.
@@Wait:	hlt								; halt CPU until some interrupt
		jmp		@@Wait					; goto @@wait
        end     Start					; Entry point marker
