; File: FastLED.asm
; 12/26/2015

;############################# DESCIPTION ########################

; This is a rewrite of the FastLED6502 found at,
; https://github.com/FastLED/FastLED/blob/master/extras/FastLED6502.s65

; YouTube with my setup, https://www.youtube.com/watch?v=oHtNm04QEts

; However to simplify the code I'm not bit-banging the output but 
; rather using the shift register in the VIA.

; The LED driver is a LPD8806 and I used a strip with 60 LEDs...

; Below is a simple text description of the LPD8806 protocol. No known datasheet exists.

; Each LPD8806 has 6 PWM outputs and can therefore drive two RGB LEDs.
; It has two lines for SPI input (data & clock), and two lines for SPI output (data & clock).
; It is implemented in a very simple way:
; When it receives a zero byte, it resets its byteCount to 0, and writes a zero byte on the output.
; When it receives a byte with the high bit set (ie ORed with 0x80) and its byteCount < 6,
; it uses the lower 7 bits of the byte to set the output of one of the PWM outputs.
; It then increments byteCount to move to the next PWM output.
; If byteCount == 6, it just sends the byte to the output.
; When these chips are chained together, with the inputs of the second chip connected to the output
; of the first chip, they are quite easy to use. You just send a total of 3 bytes per LED, and
; bytes 1-3 go to the first one, 4-6 to the second, and so on. The byte order is GRB instead of RGB.

; The color value is 7 bits per color, for 128 * 128 * 128 = 2,097,152 colors.

;########################### PIN ASSIGNMENTS #####################

; Clock on LPD8806 is connected to CB1 (pin 18 ) on the 6522.
; Data In on LPD8806 is connected to CB2 (pin 19) on the 6522.

;#################################################################

	PW 80          ;Page Width (# of char/line)
	PL 60          ;Page Length for HP Laser
	INCLIST ON     ;Add Include files in Listing

;*********************************************
;Test for Valid Processor defined in -D option
;*********************************************
	IF USING_816
	ELSE
		EXIT  "Not Valid Processor: Use -DUSING_02, etc."
	ENDIF

	TITLE  "FastLED FastLED_01.asm"
	STTL

;########################### I/O addreses #####################
	
VIA_BASE		EQU $7FC0		; base address of VIA port on SXB
VIA_ORB			EQU VIA_BASE
VIA_IRB			EQU VIA_BASE
VIA_ORA			EQU VIA_BASE+1
VIA_IRA			EQU VIA_BASE+1
VIA_DDRB		EQU VIA_BASE+2
VIA_DDRA		EQU VIA_BASE+3
VIA_T1CLO		EQU VIA_BASE+4
VIA_T1CHI		EQU VIA_BASE+5
VIA_T1LLO		EQU VIA_BASE+6
VIA_T1LHI		EQU VIA_BASE+7
VIA_T2CLO		EQU VIA_BASE+8
VIA_T2CHI		EQU VIA_BASE+9
VIA_SR			EQU VIA_BASE+10
VIA_ACR			EQU VIA_BASE+11
VIA_PCR			EQU VIA_BASE+12
VIA_IFR			EQU VIA_BASE+13
VIA_IER			EQU VIA_BASE+14
VIA_ORANH		EQU VIA_BASE+15
VIA_IRANH		EQU VIA_BASE+15

PIA_BASE		EQU $7FA0		; base address of PIA port on SXB
PIA_ORA			EQU PIA_BASE
PIA_IRA			EQU PIA_BASE
PIA_DDRA		EQU PIA_BASE
PIA_CTRLA		EQU PIA_BASE+1
PIA_ORB			EQU PIA_BASE+2
PIA_IRB			EQU PIA_BASE+2
PIA_DDRB		EQU PIA_BASE+2
PIA_CTRLB		EQU PIA_BASE+3

;########################### Propeller addreses #####################

VGA_BASE		EQU $00			; "base address" of VGA, this address is sent to the propeller
VGA_PRINT		EQU VGA_BASE
VGA_COL			EQU VGA_BASE+$01
VGA_ROW			EQU VGA_BASE+$02
VGA_ROW_COLOR	EQU VGA_BASE+$03
VGA_ROW_BACK	EQU VGA_BASE+$04
VGA_AUTO_INC	EQU VGA_BASE+$05
VGA_FILL_CHAR	EQU VGA_BASE+$06
VGA_FILL_COL	EQU VGA_BASE+$07
VGA_FILL_BACK	EQU VGA_BASE+$08
VGA_SCROLL_UP	EQU VGA_BASE+$09
VGA_SCROLL_DN	EQU VGA_BASE+$0A

VGA_CUR1_X		EQU VGA_BASE+$10
VGA_CUR1_Y		EQU VGA_BASE+$11
VGA_CUR1_MODE	EQU VGA_BASE+$12
VGA_CUR2_X		EQU VGA_BASE+$13
VGA_CUR2_Y		EQU VGA_BASE+$14
VGA_CUR2_MODE	EQU VGA_BASE+$15

SID_BASE		EQU $20		; "base address" of SID emulation, this address is sent to the propeller
SID_FR1LO		EQU SID_BASE
SID_FR1HI		EQU SID_BASE+$01
SID_PW1LO		EQU SID_BASE+$02
SID_PW1HI		EQU SID_BASE+$03
SID_CR1			EQU SID_BASE+$04
SID_AD1			EQU SID_BASE+$05
SID_SR1			EQU SID_BASE+$06

SID_FR2LO		EQU SID_BASE+$07
SID_FR2HI		EQU SID_BASE+$08
SID_PW2LO		EQU SID_BASE+$09
SID_PW2HI		EQU SID_BASE+$0A
SID_CR2			EQU SID_BASE+$0B
SID_AD2			EQU SID_BASE+$0C
SID_SR2			EQU SID_BASE+$0D

SID_FR3LO		EQU SID_BASE+$0E
SID_FR3HI		EQU SID_BASE+$0F
SID_PW3LO		EQU SID_BASE+$10
SID_PW3HI		EQU SID_BASE+$11
SID_CR3			EQU SID_BASE+$12
SID_AD3			EQU SID_BASE+$13
SID_SR3			EQU SID_BASE+$14

SID_FCLO		EQU SID_BASE+$15
SID_FCHI		EQU SID_BASE+$16
SID_RESFIL		EQU SID_BASE+$17
SID_MODVOL		EQU SID_BASE+$18

;########################### Zero Page #####################

Red_Value		EQU $10		; Red value 0-127
Green_Value		EQU $11		; Green value 0-127
Blue_Value		EQU $12		; Blue value 0-127

RGB_Pos			EQU $13		; Which position in the "rainbow" are we on ?
NUM_LED			EQU $14		; How many RGB LEDs in the strip

;########################### Main Program #####################

	CHIP 65C02
	LONGI OFF
	LONGA OFF

	.STTL "FastLED"
	.PAGE
				ORG $0200
START
				JSR init_VIA				; Init VIA
				
				STZ RGB_Pos					; Reset position
				
MAIN_LOOP				
				JSR SendDataZero			; Reset LED strip.
				
				LDY RGB_Pos					; Get current pos
				INY							; Increase
				STY RGB_Pos					; Store next pos
				
				LDA #60						; I have 60 LEDs in my strip.
				STA NUM_LED
				
				
LED_LOOP		LDA FastLED_RainbowR, Y		; Get Red value from lookup table
				LSR							; We only use 7 bits, discard lowest bit.
				ORA #$80					; Set high bit for value, it should always be high.
				STA Red_Value				; Save value

				LDA FastLED_RainbowG, Y		; Same for Green
				LSR
				ORA #$80
				STA Green_Value

				LDA FastLED_RainbowB, Y		; And same for Blue
				LSR
				ORA #$80
				STA Blue_Value

				JSR SendData				; Shift it out.

				INY							; Next color
				
				DEC	NUM_LED					; More LEDs ?
				BNE LED_LOOP				; OK, continue


delay        	LDY #$80            ; Loop 128*256 times...
				LDX #$00
dloop1       	DEX
				BNE dloop1
				DEY
				BNE dloop1				
				
				JMP MAIN_LOOP

SendData      	LDX Green_Value     ; Green value for LED
				STX VIA_SR          ; Shift it...
				LDA #$04			; Test Bit 4
d_wait1       	BIT VIA_IFR			; Are we done yet ?
				BEQ d_wait1         ; Nope, continue...
				LDX Red_Value       ; Red value for LED
				STX VIA_SR          ; Shift it...
d_wait2     	BIT VIA_IFR			; Are we done yet ?
				BEQ d_wait2         ; Nope, continue...				
				LDX Blue_Value      ; Blue value for LED
				STX VIA_SR          ; Shift it...
d_wait3       	BIT VIA_IFR			; Are we done yet ?
				BEQ d_wait3         ; Nope, continue...				
				RTS

SendDataZero       		
				STZ VIA_SR          ; Reset LED driver.
				LDA #$04			; Test Bit 4
d_waitZ			BIT VIA_IFR			; Are we done yet ?
				BEQ d_waitZ         ; Nope, continue...
				RTS
				
init_VIA
				LDA VIA_ACR         ; Load ACR
				AND #$E3            ; Zero bit 4,3,2.
				ORA #$18            ; Shift out ($10) using Phi2 ($08).
				STA VIA_ACR
				RTS
				
;-------------------------------------------------------------------------
; FUNCTION NAME	: Event Hander re-vectors
;-------------------------------------------------------------------------
IRQHandler:
				PHA
				PLA
				RTI

badVec			; $FFE0 - IRQRVD2(134)
				PHP
				PHA
				LDA #$FF
				;clear Irq
				PLA
				PLP
				RTI

;########################### Data segment #####################

	DATA

; Rainbow RGB data taken from the original driver...	
	
FastLED_RainbowR
	BYTE $FF,$FD,$FA,$F8,$F5,$F2,$F0,$ED
	BYTE $EA,$E8,$E5,$E2,$E0,$DD,$DA,$D8
	BYTE $D5,$D2,$D0,$CD,$CA,$C8,$C5,$C2
	BYTE $C0,$BD,$BA,$B8,$B5,$B2,$B0,$AD
	BYTE $AB,$AB,$AB,$AB,$AB,$AB,$AB,$AB
	BYTE $AB,$AB,$AB,$AB,$AB,$AB,$AB,$AB
	BYTE $AB,$AB,$AB,$AB,$AB,$AB,$AB,$AB
	BYTE $AB,$AB,$AB,$AB,$AB,$AB,$AB,$AB
	BYTE $AB,$A6,$A1,$9C,$96,$91,$8C,$86
	BYTE $81,$7C,$76,$71,$6C,$66,$61,$5C
	BYTE $56,$51,$4C,$47,$41,$3C,$37,$31
	BYTE $2C,$27,$21,$1C,$17,$11,$0C,$07
	BYTE $00,$00,$00,$00,$00,$00,$00,$00
	BYTE $00,$00,$00,$00,$00,$00,$00,$00
	BYTE $00,$00,$00,$00,$00,$00,$00,$00
	BYTE $00,$00,$00,$00,$00,$00,$00,$00
	BYTE $00,$00,$00,$00,$00,$00,$00,$00
	BYTE $00,$00,$00,$00,$00,$00,$00,$00
	BYTE $00,$00,$00,$00,$00,$00,$00,$00
	BYTE $00,$00,$00,$00,$00,$00,$00,$00
	BYTE $00,$02,$05,$07,$0A,$0D,$0F,$12
	BYTE $15,$17,$1A,$1D,$1F,$22,$25,$27
	BYTE $2A,$2D,$2F,$32,$35,$37,$3A,$3D
	BYTE $3F,$42,$45,$47,$4A,$4D,$4F,$52
	BYTE $55,$57,$5A,$5C,$5F,$62,$64,$67
	BYTE $6A,$6C,$6F,$72,$74,$77,$7A,$7C
	BYTE $7F,$82,$84,$87,$8A,$8C,$8F,$92
	BYTE $94,$97,$9A,$9C,$9F,$A2,$A4,$A7
	BYTE $AB,$AD,$B0,$B2,$B5,$B8,$BA,$BD
	BYTE $C0,$C2,$C5,$C8,$CA,$CD,$D0,$D2
	BYTE $D5,$D8,$DA,$DD,$E0,$E2,$E5,$E8
	BYTE $EA,$ED,$F0,$F2,$F5,$F8,$FA,$FD
FastLED_RainbowG
	BYTE $00,$02,$05,$07,$0A,$0D,$0F,$12
	BYTE $15,$17,$1A,$1D,$1F,$22,$25,$27
	BYTE $2A,$2D,$2F,$32,$35,$37,$3A,$3D
	BYTE $3F,$42,$45,$47,$4A,$4D,$4F,$52
	BYTE $55,$57,$5A,$5C,$5F,$62,$64,$67
	BYTE $6A,$6C,$6F,$72,$74,$77,$7A,$7C
	BYTE $7F,$82,$84,$87,$8A,$8C,$8F,$92
	BYTE $94,$97,$9A,$9C,$9F,$A2,$A4,$A7
	BYTE $AB,$AD,$B0,$B2,$B5,$B8,$BA,$BD
	BYTE $C0,$C2,$C5,$C8,$CA,$CD,$D0,$D2
	BYTE $D5,$D8,$DA,$DD,$E0,$E2,$E5,$E8
	BYTE $EA,$ED,$F0,$F2,$F5,$F8,$FA,$FD
	BYTE $FF,$FD,$FA,$F8,$F5,$F2,$F0,$ED
	BYTE $EA,$E8,$E5,$E2,$E0,$DD,$DA,$D8
	BYTE $D5,$D2,$D0,$CD,$CA,$C8,$C5,$C2
	BYTE $C0,$BD,$BA,$B8,$B5,$B2,$B0,$AD
	BYTE $AB,$A6,$A1,$9C,$96,$91,$8C,$86
	BYTE $81,$7C,$76,$71,$6C,$66,$61,$5C
	BYTE $56,$51,$4C,$47,$41,$3C,$37,$31
	BYTE $2C,$27,$21,$1C,$17,$11,$0C,$07
	BYTE $00,$00,$00,$00,$00,$00,$00,$00
	BYTE $00,$00,$00,$00,$00,$00,$00,$00
	BYTE $00,$00,$00,$00,$00,$00,$00,$00
	BYTE $00,$00,$00,$00,$00,$00,$00,$00
	BYTE $00,$00,$00,$00,$00,$00,$00,$00
	BYTE $00,$00,$00,$00,$00,$00,$00,$00
	BYTE $00,$00,$00,$00,$00,$00,$00,$00
	BYTE $00,$00,$00,$00,$00,$00,$00,$00
	BYTE $00,$00,$00,$00,$00,$00,$00,$00
	BYTE $00,$00,$00,$00,$00,$00,$00,$00
	BYTE $00,$00,$00,$00,$00,$00,$00,$00
	BYTE $00,$00,$00,$00,$00,$00,$00,$00
FastLED_RainbowB
	BYTE $00,$00,$00,$00,$00,$00,$00,$00
	BYTE $00,$00,$00,$00,$00,$00,$00,$00
	BYTE $00,$00,$00,$00,$00,$00,$00,$00
	BYTE $00,$00,$00,$00,$00,$00,$00,$00
	BYTE $00,$00,$00,$00,$00,$00,$00,$00
	BYTE $00,$00,$00,$00,$00,$00,$00,$00
	BYTE $00,$00,$00,$00,$00,$00,$00,$00
	BYTE $00,$00,$00,$00,$00,$00,$00,$00
	BYTE $00,$00,$00,$00,$00,$00,$00,$00
	BYTE $00,$00,$00,$00,$00,$00,$00,$00
	BYTE $00,$00,$00,$00,$00,$00,$00,$00
	BYTE $00,$00,$00,$00,$00,$00,$00,$00
	BYTE $00,$02,$05,$07,$0A,$0D,$0F,$12
	BYTE $15,$17,$1A,$1D,$1F,$22,$25,$27
	BYTE $2A,$2D,$2F,$32,$35,$37,$3A,$3D
	BYTE $3F,$42,$45,$47,$4A,$4D,$4F,$52
	BYTE $55,$5A,$5F,$64,$6A,$6F,$74,$7A
	BYTE $7F,$84,$8A,$8F,$94,$9A,$9F,$A4
	BYTE $AA,$AF,$B4,$B9,$BF,$C4,$C9,$CF
	BYTE $D4,$D9,$DF,$E4,$E9,$EF,$F4,$F9
	BYTE $FF,$FD,$FA,$F8,$F5,$F2,$F0,$ED
	BYTE $EA,$E8,$E5,$E2,$E0,$DD,$DA,$D8
	BYTE $D5,$D2,$D0,$CD,$CA,$C8,$C5,$C2
	BYTE $C0,$BD,$BA,$B8,$B5,$B2,$B0,$AD
	BYTE $AB,$A9,$A6,$A4,$A1,$9E,$9C,$99
	BYTE $96,$94,$91,$8E,$8C,$89,$86,$84
	BYTE $81,$7E,$7C,$79,$76,$74,$71,$6E
	BYTE $6C,$69,$66,$64,$61,$5E,$5C,$59
	BYTE $55,$53,$50,$4E,$4B,$48,$46,$43
	BYTE $40,$3E,$3B,$38,$36,$33,$30,$2E
	BYTE $2B,$28,$26,$23,$20,$1E,$1B,$18
	BYTE $16,$13,$10,$0E,$0B,$08,$06,$03
				
	ENDS

;-----------------------------
;
;		Reset and Interrupt Vectors (define for 265, 816/02 are subsets)
;
;-----------------------------

Shadow_VECTORS	SECTION OFFSET $7EE0
								;65C816 Interrupt Vectors
								;Status bit E = 0 (Native mode, 16 bit mode)
				DW badVec		; $FFE0 - IRQRVD4(816)
				DW badVec		; $FFE2 - IRQRVD5(816)
				DW badVec		; $FFE4 - COP(816)
				DW badVec		; $FFE6 - BRK(816)
				DW badVec		; $FFE8 - ABORT(816)
				DW badVec		; $FFEA - NMI(816)
				DW badVec		; $FFEC - IRQRVD(816)
				DW badVec		; $FFEE - IRQ(816)
								;Status bit E = 1 (Emulation mode, 8 bit mode)
				DW badVec		; $FFF0 - IRQRVD2(8 bit Emulation)(IRQRVD(265))
				DW badVec		; $FFF2 - IRQRVD1(8 bit Emulation)(IRQRVD(265))
				DW badVec		; $FFF4 - COP(8 bit Emulation)
				DW badVec   	; $FFF6 - IRQRVD0(8 bit Emulation)(IRQRVD(265))
				DW badVec		; $FFF8 - ABORT(8 bit Emulation)
								; Common 8 bit Vectors for all CPUs
				DW badVec		; $FFFA -  NMIRQ (ALL)
				DW START		; $FFFC -  RESET (ALL)
				DW IRQHandler	; $FFFE -  IRQBRK (ALL)
	ENDS

vectors	SECTION OFFSET $FFE0
								;65C816 Interrupt Vectors
								;Status bit E = 0 (Native mode, 16 bit mode)
				DW badVec		; $FFE0 - IRQRVD4(816)
				DW badVec		; $FFE2 - IRQRVD5(816)
				DW badVec		; $FFE4 - COP(816)
				DW badVec		; $FFE6 - BRK(816)
				DW badVec		; $FFE8 - ABORT(816)
				DW badVec		; $FFEA - NMI(816)
				DW badVec		; $FFEC - IRQRVD(816)
				DW badVec		; $FFEE - IRQ(816)
								;Status bit E = 1 (Emulation mode, 8 bit mode)
				DW badVec		; $FFF0 - IRQRVD2(8 bit Emulation)(IRQRVD(265))
				DW badVec		; $FFF2 - IRQRVD1(8 bit Emulation)(IRQRVD(265))
				DW badVec		; $FFF4 - COP(8 bit Emulation)
				DW badVec		; $FFF6 - IRQRVD0(8 bit Emulation)(IRQRVD(265))
				DW badVec		; $FFF8 - ABORT(8 bit Emulation)
								; Common 8 bit Vectors for all CPUs
				DW badVec		; $FFFA -  NMIRQ (ALL)
				DW START		; $FFFC -  RESET (ALL)
				DW IRQHandler	; $FFFE -  IRQBRK (ALL)
	ENDS
	END
