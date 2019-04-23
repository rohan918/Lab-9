; Print.s
; Student names: Austin Rath and Rohan Narayanan
; Last modification date: change this to the last modification date or look very silly
; Runs on LM4F120 or TM4C123
; EE319K lab 7 device driver for any LCD
;
; As part of Lab 7, students need to implement these LCD_OutDec and LCD_OutFix
; This driver assumes two low-level LCD functions
; ST7735_OutChar   outputs a single 8-bit ASCII character
; ST7735_OutString outputs a null-terminated string 


;OutDec	
REM  EQU 4
;OutFix locals
One  EQU 0
point EQU 4	
Tenth EQU 8
Hund  EQU 12
Thou  EQU 16


    IMPORT   ST7735_OutChar
    IMPORT   ST7735_OutString
    EXPORT   LCD_OutDec
    EXPORT   LCD_OutFix
	

    AREA    |.text|, CODE, READONLY, ALIGN=2
    THUMB

  

;-----------------------LCD_OutDec-----------------------
; Output a 32-bit number in unsigned decimal format
; Input: R0 (call by value) 32-bit unsigned number
; Output: none
; Invariables: This function must not permanently modify registers R4 to R11
LCD_OutDec

	PUSH {R4, LR}
	SUBS SP, #8
	
	MOV R1, #10
	UDIV R2, R0, R1
    MUL R2, R2, R1
	SUB R3, R0, R2
	STR R3, [SP, #REM]
	UDIV R2, R2, R1
	MOV R0, R2
	CMP R0, #0
	BEQ done
	BL LCD_OutDec
done
	 LDR R1, [SP, #REM]
	 ADD R1, #0X30
	 MOV R0, R1
	 BL ST7735_OutChar
	 MOV R0, #0
	 ADD SP, #8
	 POP {R4, LR}
	

      BX  LR
;* * * * * * * * End of LCD_OutDec * * * * * * * *

; -----------------------LCD _OutFix----------------------
; Output characters to LCD display in fixed-point format
; unsigned decimal, resolution 0.001, range 0.000 to 9.999
; Inputs:  R0 is an unsigned 32-bit number
; Outputs: none
; E.g., R0=0,    then output "0.000 "
;       R0=3,    then output "0.003 "
;       R0=89,   then output "0.089 "
;       R0=123,  then output "0.123 "
;       R0=9999, then output "9.999 "
;       R0>9999, then output "*.*** "
; Invariables: This function must not permanently modify registers R4 to R11
LCD_OutFix
	 PUSH {R4, LR}
     SUBS SP, #24
	 ;set all to 0 initially plus decimal point
	 MOV R1, #0
	 STR R1, [SP, #One]
	 STR R1, [SP, #Tenth]
	 STR R1, [SP, #Hund]
	 STR R1, [SP, #Thou]
	 MOV R1, #0x2E
	 STR R1, [SP, #point]
	 MOV R1, #9999
	 CMP R0, R1
	 BHI stars
	 
	 
	 MOV R1, #10
	 
	 UDIV R2, R0, R1
	 MUL R3, R2, R1
	 SUB R3, R0, R3
	 ADD R3, #0x30
	 STR R3, [SP, #Thou]
	 MOV R0, R2
	 
	 UDIV R2, R0, R1
	 MUL R3, R2, R1
	 SUB R3, R0, R3
	 ADD R3, #0x30
	 STR R3, [SP, #Hund]
	 MOV R0, R2
	 
	 UDIV R2, R0, R1
	 MUL R3, R2, R1
	 SUB R3, R0, R3
	 ADD R3, #0x30
	 STR R3, [SP, #Tenth]
	 MOV R0, R2
	 
	 UDIV R2, R0, R1
	 MUL R3, R2, R1
	 SUB R3, R0, R3
	 ADD R3, #0x30
	 STR R3, [SP, #One]
	 MOV R0, R2
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 B skip
stars
	 MOV R1, #0x2A
	 STR R1, [SP, #One]
	 STR R1, [SP, #Tenth]
	 STR R1, [SP, #Hund]
	 STR R1, [SP, #Thou]
skip
     LDR R0, [SP, #One]
	 BL ST7735_OutChar
	 LDR R0, [SP, #point]
	 BL ST7735_OutChar
	 LDR R0, [SP, #Tenth]
	 BL ST7735_OutChar
	 LDR R0, [SP, #Hund]
	 BL ST7735_OutChar
	 LDR R0, [SP, #Thou]
	 BL ST7735_OutChar
	 ADD SP, #24
	 POP {R4, LR}
     BX   LR
 
     ALIGN
;* * * * * * * * End of LCD_OutFix * * * * * * * *

     ALIGN                           ; make sure the end of this section is aligned
     END                             ; end of file
