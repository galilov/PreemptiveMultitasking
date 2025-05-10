use16									; Use 16-bit mode instructions.

TimerVector			=		8h			; constant: Timer interrupt vector
ThreadStackSize		=		128			; constant: Size of the Thread stack, bytes
NThreads			=		4			; constant: Max number of Threads

; ===== Macro command which saves all the required registers to stack
macro pushregs
{
		push	ax						; *((ss << 4) + (--sp)) = ax
		push	bx						; *((ss << 4) + (--sp)) = bx
		push	cx						; *((ss << 4) + (--sp)) = cx
		push	dx						; *((ss << 4) + (--sp)) = dx
		push	si						; *((ss << 4) + (--sp)) = si
		push	di						; *((ss << 4) + (--sp)) = di
		push	bp						; *((ss << 4) + (--sp)) = bp
		push	es						; *((ss << 4) + (--sp)) = es
		push	ds						; *((ss << 4) + (--sp)) = ds
}

; ===== Macro command which loads the previously saved registers from stack
macro popregs
{
		pop		ds						; ds = *((ss << 4) + (sp++))
		pop		es						; es = *((ss << 4) + (sp++))
		pop		bp						; bp = *((ss << 4) + (sp++))
		pop		di						; di = *((ss << 4) + (sp++))
		pop		si						; si = *((ss << 4) + (sp++))
		pop		dx						; dx = *((ss << 4) + (sp++))
		pop		cx						; cx = *((ss << 4) + (sp++))
		pop		bx						; bx = *((ss << 4) + (sp++))
		pop		ax						; ax = *((ss << 4) + (sp++))
}

; We use TINY memory model allowing 64K for code, data, and stack.
; All of CS, DS, and SS are set to the same value. This is the memory model
; that COM files use.
		org 100h						; Offset for the code in tiny model.

Start:	jmp		Main					; Go to main code.

ThreadStacks		db		(ThreadStackSize * NThreads) dup (0)	; The memory block for stacks
SavedSPs			dw		NThreads dup (0)	; Array of NThreads words to store Thread-specific SP (stack pointer) addresses.
MainThreadSP		dw		0			; Word variable to store a stack pointer for the main (startup) code.
CurrentThreadIndex	db		-1			; Byte variable to store currently executing Thread number.
AddedThreadCounter	db		0			; Byte variable used as counter of added Threads.
PrevTimerVector		dd		0			; Double word variable to save the original Timer interrupt vector here.
TickCounter			dw		0           ; Counter used to count the number of ticks since switcher is started.
MsgDone				db		'Done', 10, 13, '$'	; Message to print on exit.
Done				db		0			; Flag to indicate that the switcher is done.
; ===== Thread 1
Thread1:
		mov		ax, 0b800h				; ax = 0xB800, a segment address of a text line 0
		mov		es, ax					; es = ax
		mov		ah, 01h					; ah = 02h -- attribute to print "blue on black"
.Loop:
		mov		al, '0'					; al = '0' -- character to print
.NextDigit:
		mov		[es:0], ax				; print the character in ax at the very first
		inc		al						; al++ -- change the character
		cmp		al, '9'					; compare al to '9'
		jbe		.NextDigit				; if al <= '9' goto .NextDigit
		mov		al, '0'					; al = '0'
		jmp		.Loop					; goto .Loop

; ===== Thread 2
Thread2:
		mov ax, 0b800h					; ax = 0xB800, a segment address of a text line 0
		mov es, ax						; es = ax
		mov ah, 02h						; ah = 02h -- attribute to print "green on black"
.Loop:
		mov al, 'A'						; al = 'A' -- character to print
.NextChar:
		mov [es:2], ax					; print the character in ax at the second position
		inc al							; al++ -- change the character
		cmp al, 'Z'						; compare al to 'Z'
		jbe .NextChar					; if al <= 'Z' goto .NextChar
		jmp .Loop						; goto .Loop

; ===== Thread 3
Thread3:
		mov ax, 0b800h					; ax = 0xB800, a segment address of a text line 0
		mov es, ax						; es = ax
		mov ax, 0040h					; ah = 00h -- attribute to print "black on black", al = '@'
.Loop:
		mov [es:4], ax					; print the character '@' at the third position
		inc ah							; ah++ -- change the attribute
		jmp .Loop						; goto .Loop

; ===== Thread 4
Thread4:
		mov		dx, [TickCounter]
		mov		bx, 0100h				; line 1, column 0
		call	DumpHex					; dump the tick counter in hex
		cmp		[TickCounter], 100h		; compare *TickCounter to 100h
		jb		Thread4					; if *TickCounter < 100h goto Thread4
		jmp		TerminateSwitcher		; go to TerminateSwitcher


; ===== TerminateSwitcher
; This function restores the data segment, stack segment, timer vector, and main SP.
; It also sets the Done flag to 1 and returns from the interrupt to the main code.
TerminateSwitcher:
		xor		ax, ax
		mov		es, ax
		cli								; disable interrupts
		mov		ax, word [PrevTimerVector]		; ax = *PrevTimerVector
		mov		word [es:4 * TimerVector], ax	; *((es << 4) + 4 * TimerVector) = ax
		mov		ax, word [PrevTimerVector + 2]	; ax = *(PrevTimerVector + 2)
		mov		word [es:4 * TimerVector + 2], ax	; *((es << 4) + 4 * TimerVector + 2) = ax
		; restore main SP
		mov		sp, [MainThreadSP]		; sp = *MainThreadSP
		mov		[Done], 1				; *Done = 1
		popregs							; restore registers
		iret							; return from interrupt


; ===== Calculate the address to store a stack pointer for specific Thread
; Input:
;     BL: Thread index
; Output:
;     BX: Thread-specific Stack pointer address
GetSavedSPOffset:
		; bx = bl * 2 + &SavedSPs
		xor		bh, bh					; bh = 0
		shl		bx, 1					; bx <<= 1 -- multiply by 2 to get the index of the Thread stack pointer in SavedSPs array
		add		bx, SavedSPs	 		; bx += &SavedSPs
		ret								; return


; ==== Adds a new Thread to multithreading manager
; Input:
;     DX: Thread routine offset
; Uses:
;     AX, BX, DX, CX, SP
AddThread:
		; Save the current Thread counter to BL for future usage in GetThreadSPAddr call.
		mov		bl, [AddedThreadCounter]	; bl = *AddedThreadCounter
		; Calculate the new Thread stack pointer address.
		; Stack pointer must point to the top of the stack.
		mov 	al, bl					; al = bl
		inc		al						; al++
		mov		[AddedThreadCounter], al	; *AddedThreadCounter = al
		mov		bh,	ThreadStackSize		; bh = ThreadStackSize -- number of bytes in the stack
		mul		bh						; ax = al * bh -- select the top of the stack
		add		ax, ThreadStacks			; ax += &ThreadStacks -- now ax points to the top of the Thread stack
		; Save the current value of SP to CX.
		mov		cx, sp					; cx = sp
		; Switch SP to the top of a Thread stack.
		mov		sp, ax					; sp = ax
		; Prepare the Thread stack to switch to the beginning of the Thread with IRET instruction.
		pushf							; *((ss << 4) + (--sp)) = flags
		push	cs						; *((ss << 4) + (--sp)) = cs -- code segment
		push	dx						; *((ss << 4) + (--sp)) = dx -- Thread offset
		; Save reqired registers to be restored when switching to the Thread.
		pushregs						; use macro pushregs here
		; Save SP to the array of Thread-specific stack pointers.
		; SP will be restored from the array element when switching to the Thread.
		call	GetSavedSPOffset		; call GetSavedSPOffset, input parameter is BL
		; Now BX points to the Thread-specific stack pointer address, save SP to it.
		mov		[bx], sp				; *(ds << 4) + bx) = sp
		; Restore previously saved SP from CX. Required to return from 'AddThread'back to a calling code.
		mov		sp, cx					; sp = cx
		ret								; return


; ==== Time interrupt handler, switches threads. It is called every 55 ms.
TimerHandler:
		; Save registers to a current stack.
		pushregs						; use macro pushregs here
		; Set DS=CS to address our data
		mov		ax, cs					; ax = cs
		mov		ds, ax					; ds = ax
		; Increment the tick counter.
		inc		[TickCounter]			; *(TickCounter)++
		; Check if the handler interrupted the main code.
		cmp		[CurrentThreadIndex], -1; compare *CurrentThreadIndex to -1
		jne		.SaveThread				; if *CurrentThreadIndex != -1 goto .SaveThread
		; save main SP to MainThreadSP
		mov		[MainThreadSP], sp		; *MainThreadSP = sp
		jmp		short .NextThread			; goto .NextThread
.SaveThread:
		; save SP to a Thread-specific pointer variable.
		mov		bl, [CurrentThreadIndex]	; bl = *CurrentThreadIndex
		call	GetSavedSPOffset			; call GetSavedSPOffset
		mov 	[bx], sp				; *((ds << 4) + bx) = sp
.NextThread:
		; Select next Thread.
		; Correct the Thread number: CurrentThreadIndex %= NThreads
		xor		ah, ah					; ah = 0
		mov		al, [CurrentThreadIndex]	; al = *CurrentThreadIndex
		inc		al						; al++
		mov		bl, NThreads			; bl = NThreads
		div		bl						; al = ax / bl , ah = ax % bl
		mov		[CurrentThreadIndex], ah	; *CurrentThreadIndex = ah
		; Load SP from a Thread-specific pointer variable.
		mov		bl, ah					; bl = ah
		call	GetSavedSPOffset			; call GetSavedSPOffset
		mov		sp, [bx]				; sp = *((ds << 4) + bx)
		popregs							; Restore the registers from stack.
		jmp		dword [PrevTimerVector] ; far jump to segment:offset saved in PrevTimerVector


; ==== DumpHex
; Input:
;     DX: value to dump
;     BH: line number
;     BL: column number
; Output:
;     None
; Uses:
;     AX, BX, CX, DX, ES
; Description:
;     This function dumps the value in DX to the screen at the specified line and column.
;     It converts the value to a string of hex digits and stores them directly in the video memory.
DumpHex:
		mov		ax, 0b800h				; ax = 0xB800, a segment address of a text line 0
		mov		es, ax					; es = ax
		mov		al, 160					; al = 160 -- number of bytes per line (2 bytes per character)
		mul 	bh						; ax = bh * al -- calculate the offset of the line
		xor		bh, bh					; bh = 0
		shl		bx, 1					; bx <<= 1 -- multiply by 2 to get the index of the column
		add		bx, ax					; bx += ax -- add the offset of the line
		mov		cl, 12					; cl = 12 -- counter for the number of bits to shift
.DoLoop:
		mov		ax, dx					; ax = dx -- copy the value to ax
		shr		ax, cl					; shift right to get the next nibble
		and		al, 0fh					; mask the nibble
		cmp		al, 9					; compare the nibble to 9
		jbe		.Decimal				; if the nibble is less than 9, jump to .Decimal
		add		al, 7					; correct for hex
.Decimal:
		add		al, '0'					; convert to ascii
		mov		ah, 0eh					; character attribute 'yellow on black
		mov		[es:bx], ax				; store character and attribute
		add		bx, 2					; move to next position
		sub		cl, 4					; move to next nibble
		jge		.DoLoop					; if cl >= 0, jump to .DoLoop
		ret								; return


; ==== Startup code
Main:
		; Add Thread1 to switcher.
		mov		dx, Thread1				; dx = &Thread1
		call	AddThread
		; Add Thread2 to switcher.
		mov		dx, Thread2				; dx = &Thread2
		call	AddThread
		; Add Thread3 to switcher.
		mov		dx, Thread3				; dx = &Thread3
		call	AddThread
		; Add Thread4 to switcher.
		mov		dx, Thread4				; dx = &Thread4
		call	AddThread

		; Set the segment register ES to 0.
		xor		ax, ax					; ax = 0
		mov		es, ax					; es = ax
		; Save the current (system) timer handler address (segment:offset) to PrevTimerVector variable.
		mov		ax, [es:4 * TimerVector]	; ax = *((es << 4) + 4 * TimerVector)
		mov		word [PrevTimerVector], ax		; *PrevTimerVector = ax
		mov		ax, [es:4 * TimerVector + 2]	; ax = *((es << 4) + 4 * TimerVector + 2)
		mov		word [PrevTimerVector + 2], ax	;  *(PrevTimerVector + 2) = ax
		; Set the timer interrupt vector to a new handler TimerHandler.
		cli								; clear interrupt flag (disable interrupts)
		mov		word [es:4 * TimerVector], TimerHandler	; *((es << 4) + 4 * TimerVector) = &TimerHandler
		mov		word [es:4 * TimerVector + 2], cs	; *((es << 4) + 4 * TimerVector + 2) = cs
		sti								; set interrupt flag (enable interrupts)
		; Give chance to the timer interrupt to start the first Thread.
.Wait:
		wait							; wait for some interrupt
		; Check if the switcher is done.
		cmp		[Done], 1				; compare *Done to 1
		jne		.Wait					; if *Done != 1 goto .Wait
		; Print a message to the screen.
		mov ah, 9h						; ah = 9h -- print string function
		mov dx, MsgDone					; dx = &MsgDone
		int 21h							; call DOS interrupt
		; exit to DOS
		ret
