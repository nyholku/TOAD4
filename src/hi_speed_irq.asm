; This file tuned from the C file hi_speed_irq.c compiled with SDCC to asm first
; 1) removed r0x00,r0x01 from ".registers udata ovr" section
; and introduced them to 'idata' section after _g as:
;  r0x00 db 0x00
;  r0x01 db 0x00
; question: does that quarantee that they are in the same section?
; also removed saving/restoring them in the interrupt prologue/epilogue
; 2) RETFIE -> RETFIX 0x01 (fast return)
; 3) 4 x "MOTOR.nco += MOTOR.speed;" replace:
;	MOVF	(_g + 6), W, B
;	ADDWF	(_g + 4), W, B
;	MOVWF	r0x00
;	MOVF	(_g + 7), W, B
;	ADDWFC	(_g + 5), W, B
;	MOVWF	r0x01
;	MOVF	r0x00, W
;	MOVWF	(_g + 4), B
;	MOVF	r0x01, W
;	MOVWF	(_g + 5), B
; with:
;   MOVF    (_g + 6), W, B
;   ADDWF   (_g + 4), F, B
;   MOVF    (_g + 7), W, B
;   ADDWFC  (_g + 5), F, B
; 4) 4 x "if (MOTOR.nco_hi_bit) {...." replace:
;  	BTFSS	(_g + 5), 7, B
;	BRA	_00114_DS_
; with:
;   BNC _00114_DS_
; 5) 4 x "MOTOR.nco_hi_bit=0;" remove
;	BCF	(_g + 5), 7, B
; 6) TODO: optimize pro/epilogues





;--------------------------------------------------------
; File Created by SDCC : free open source ANSI-C Compiler
; Version 3.2.0 #8008 (Jul  6 2012) (Mac OS X i386)
; This file was generated Wed Jun 25 15:01:33 2014
;--------------------------------------------------------
; PIC16 port for the Microchip 16-bit core micros
;--------------------------------------------------------
	list	p=18f4550

	radix dec

;--------------------------------------------------------
; public variables in this module
;--------------------------------------------------------
	global	_g
	global	_high_priority_interrupt_service

;--------------------------------------------------------
; extern variables in this module
;--------------------------------------------------------
	extern	_SPPDATAbits
	extern	_SPPCFGbits
	extern	_SPPEPSbits
	extern	_SPPCONbits
	extern	_UFRMLbits
	extern	_UFRMHbits
	extern	_UIRbits
	extern	_UIEbits
	extern	_UEIRbits
	extern	_UEIEbits
	extern	_USTATbits
	extern	_UCONbits
	extern	_UADDRbits
	extern	_UCFGbits
	extern	_UEP0bits
	extern	_UEP1bits
	extern	_UEP2bits
	extern	_UEP3bits
	extern	_UEP4bits
	extern	_UEP5bits
	extern	_UEP6bits
	extern	_UEP7bits
	extern	_UEP8bits
	extern	_UEP9bits
	extern	_UEP10bits
	extern	_UEP11bits
	extern	_UEP12bits
	extern	_UEP13bits
	extern	_UEP14bits
	extern	_UEP15bits
	extern	_PORTAbits
	extern	_PORTBbits
	extern	_PORTCbits
	extern	_PORTDbits
	extern	_PORTEbits
	extern	_LATAbits
	extern	_LATBbits
	extern	_LATCbits
	extern	_LATDbits
	extern	_LATEbits
	extern	_TRISAbits
	extern	_TRISBbits
	extern	_TRISCbits
	extern	_TRISDbits
	extern	_TRISEbits
	extern	_OSCTUNEbits
	extern	_PIE1bits
	extern	_PIR1bits
	extern	_IPR1bits
	extern	_PIE2bits
	extern	_PIR2bits
	extern	_IPR2bits
	extern	_EECON1bits
	extern	_RCSTAbits
	extern	_TXSTAbits
	extern	_T3CONbits
	extern	_CMCONbits
	extern	_CVRCONbits
	extern	_ECCP1ASbits
	extern	_ECCP1DELbits
	extern	_BAUDCONbits
	extern	_CCP2CONbits
	extern	_CCP1CONbits
	extern	_ADCON2bits
	extern	_ADCON1bits
	extern	_ADCON0bits
	extern	_SSPCON2bits
	extern	_SSPCON1bits
	extern	_SSPSTATbits
	extern	_T2CONbits
	extern	_T1CONbits
	extern	_RCONbits
	extern	_WDTCONbits
	extern	_HLVDCONbits
	extern	_OSCCONbits
	extern	_T0CONbits
	extern	_STATUSbits
	extern	_FSR2Hbits
	extern	_BSRbits
	extern	_FSR1Hbits
	extern	_FSR0Hbits
	extern	_INTCON3bits
	extern	_INTCON2bits
	extern	_INTCONbits
	extern	_TBLPTRUbits
	extern	_PCLATHbits
	extern	_PCLATUbits
	extern	_STKPTRbits
	extern	_TOSUbits
	extern	_SPPDATA
	extern	_SPPCFG
	extern	_SPPEPS
	extern	_SPPCON
	extern	_UFRML
	extern	_UFRMH
	extern	_UIR
	extern	_UIE
	extern	_UEIR
	extern	_UEIE
	extern	_USTAT
	extern	_UCON
	extern	_UADDR
	extern	_UCFG
	extern	_UEP0
	extern	_UEP1
	extern	_UEP2
	extern	_UEP3
	extern	_UEP4
	extern	_UEP5
	extern	_UEP6
	extern	_UEP7
	extern	_UEP8
	extern	_UEP9
	extern	_UEP10
	extern	_UEP11
	extern	_UEP12
	extern	_UEP13
	extern	_UEP14
	extern	_UEP15
	extern	_PORTA
	extern	_PORTB
	extern	_PORTC
	extern	_PORTD
	extern	_PORTE
	extern	_LATA
	extern	_LATB
	extern	_LATC
	extern	_LATD
	extern	_LATE
	extern	_TRISA
	extern	_TRISB
	extern	_TRISC
	extern	_TRISD
	extern	_TRISE
	extern	_OSCTUNE
	extern	_PIE1
	extern	_PIR1
	extern	_IPR1
	extern	_PIE2
	extern	_PIR2
	extern	_IPR2
	extern	_EECON1
	extern	_EECON2
	extern	_EEDATA
	extern	_EEADR
	extern	_RCSTA
	extern	_TXSTA
	extern	_TXREG
	extern	_RCREG
	extern	_SPBRG
	extern	_SPBRGH
	extern	_T3CON
	extern	_TMR3L
	extern	_TMR3H
	extern	_CMCON
	extern	_CVRCON
	extern	_ECCP1AS
	extern	_ECCP1DEL
	extern	_BAUDCON
	extern	_CCP2CON
	extern	_CCPR2L
	extern	_CCPR2H
	extern	_CCP1CON
	extern	_CCPR1L
	extern	_CCPR1H
	extern	_ADCON2
	extern	_ADCON1
	extern	_ADCON0
	extern	_ADRESL
	extern	_ADRESH
	extern	_SSPCON2
	extern	_SSPCON1
	extern	_SSPSTAT
	extern	_SSPADD
	extern	_SSPBUF
	extern	_T2CON
	extern	_PR2
	extern	_TMR2
	extern	_T1CON
	extern	_TMR1L
	extern	_TMR1H
	extern	_RCON
	extern	_WDTCON
	extern	_HLVDCON
	extern	_OSCCON
	extern	_T0CON
	extern	_TMR0L
	extern	_TMR0H
	extern	_STATUS
	extern	_FSR2L
	extern	_FSR2H
	extern	_PLUSW2
	extern	_PREINC2
	extern	_POSTDEC2
	extern	_POSTINC2
	extern	_INDF2
	extern	_BSR
	extern	_FSR1L
	extern	_FSR1H
	extern	_PLUSW1
	extern	_PREINC1
	extern	_POSTDEC1
	extern	_POSTINC1
	extern	_INDF1
	extern	_WREG
	extern	_FSR0L
	extern	_FSR0H
	extern	_PLUSW0
	extern	_PREINC0
	extern	_POSTDEC0
	extern	_POSTINC0
	extern	_INDF0
	extern	_INTCON3
	extern	_INTCON2
	extern	_INTCON
	extern	_PRODL
	extern	_PRODH
	extern	_TABLAT
	extern	_TBLPTRL
	extern	_TBLPTRH
	extern	_TBLPTRU
	extern	_PCL
	extern	_PCLATH
	extern	_PCLATU
	extern	_STKPTR
	extern	_TOSL
	extern	_TOSH
	extern	_TOSU

;--------------------------------------------------------
;	Equates to used internal registers
;--------------------------------------------------------
STATUS	equ	0xfd8
PCLATH	equ	0xffa
PCLATU	equ	0xffb
BSR	equ	0xfe0
FSR0L	equ	0xfe9
FSR0H	equ	0xfea
FSR1L	equ	0xfe1
FSR2L	equ	0xfd9
POSTDEC1	equ	0xfe5
PREINC1	equ	0xfe4
PRODL	equ	0xff3
PRODH	equ	0xff4


	idata
_g	db	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	db	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	db	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	db	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
;
r0x00 db 0x00
r0x01 db 0x00
;
; Internal registers
.registers	udata_ovr	0x0000


usbram5	udata

;--------------------------------------------------------
; global & static initialisations
;--------------------------------------------------------
; ; Starting pCode block for absolute section
; ;-----------------------------------------
S_stepperirq_ivec_0x1_high_priority_interrupt_service	code	0X000008
ivec_0x1_high_priority_interrupt_service:
	GOTO	_high_priority_interrupt_service

; I code from now on!
; ; Starting pCode block
S_stepperirq__high_priority_interrupt_service	code
_high_priority_interrupt_service:
;	.line	67; stepperirq.c	void high_priority_interrupt_service() __interrupt(1) {
;	.line	69; stepperirq.c	LED_PIN = 1;
	BSF	_LATBbits, 4
	MOVFF	STATUS, POSTDEC1
	MOVFF	BSR, POSTDEC1
	MOVWF	POSTDEC1
	MOVFF	PRODL, POSTDEC1
	MOVFF	PRODH, POSTDEC1
	MOVFF	FSR0L, POSTDEC1
	MOVFF	FSR0H, POSTDEC1
	MOVFF	PCLATH, POSTDEC1
	MOVFF	PCLATU, POSTDEC1
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
;	.line	71; stepperirq.c	if (PIR1bits.TMR2IF) {
	BTFSS	_PIR1bits, 1
	BRA	_00146_DS_
;	.line	72; stepperirq.c	PIR1bits.TMR2IF = 0;
	BCF	_PIR1bits, 1
	BANKSEL	_g
;	.line	75; stepperirq.c	g.irq_flag = 0;
	BCF	_g, 0, B
;	.line	81; stepperirq.c	STEP_OUTPUT_PORT |= STEP_OUTPUT_ALL;
	MOVLW	0x0f
	IORWF	_LATD, F
;	.line	118; stepperirq.c	MOTOR.nco += MOTOR.speed;
	MOVF	(_g + 6), W, B
	ADDWF	(_g + 4), F, B
	MOVF	(_g + 7), W, B
	ADDWFC	(_g + 5), F, B
;	.line	119; stepperirq.c	if (MOTOR.steps) {
	MOVFF	(_g + 8), r0x00
	MOVF	r0x00, W
	BZ	_00113_DS_
;	.line	120; stepperirq.c	if (MOTOR.nco_hi_bit) { // speed NCO overflowed and we have steps left => generate pulse
	BNC	_00114_DS_
;	.line	121; stepperirq.c	MOTOR.steps--;
	DECF	r0x00, F
	MOVF	r0x00, W
	MOVWF	(_g + 8), B
;	.line	122; stepperirq.c	STEP_OUTPUT = 0;// STEP signal = 0 generates a rising edge on next interrupt
	BCF	_LATDbits, 0
	BRA	_00114_DS_
_00113_DS_:
;	.line	124; stepperirq.c	} else if (MOTOR.has_next) { // check if there more steps in the queue
	BTFSS	(_g + 9), 0, B
	BRA	_00114_DS_
;	.line	125; stepperirq.c	MOTOR.has_next = 0;// signals to higher level that we have consumed the next 'step' so to speak
	BCF	(_g + 9), 0, B
;	.line	126; stepperirq.c	MOTOR.steps = MOTOR.next_steps;
	MOVFF	(_g + 12), r0x00
	MOVF	r0x00, W
	MOVWF	(_g + 8), B
;	.line	127; stepperirq.c	MOTOR.speed = MOTOR.next_speed;
	MOVFF	(_g + 10), r0x00
	MOVFF	(_g + 11), r0x01
	MOVF	r0x00, W
	MOVWF	(_g + 6), B
	MOVF	r0x01, W
	MOVWF	(_g + 7), B
;	.line	130; stepperirq.c	if (MOTOR.next_dir) {
	BTFSS	(_g + 9), 1, B
	BRA	_00108_DS_
;	.line	131; stepperirq.c	DIR_OUTPUT=1;
	BSF	_LATCbits, 0
	BRA	_00114_DS_
_00108_DS_:
;	.line	133; stepperirq.c	DIR_OUTPUT=0;
	BCF	_LATCbits, 0
_00114_DS_:
;	.line	136; stepperirq.c	MOTOR.has_next = 1; // FIXME: THIS LINE JUST FOR TESTING
	BSF	(_g + 9), 0, B
;	.line	118; stepperirq.c	MOTOR.nco += MOTOR.speed;
	MOVF	(_g + 15), W, B
	ADDWF	(_g + 13), F, B
	MOVF	(_g + 16), W, B
	ADDWFC	(_g + 14), F, B
	MOVWF	(_g + 14), B
;	.line	119; stepperirq.c	if (MOTOR.steps) {
	MOVFF	(_g + 17), r0x00
	MOVF	r0x00, W
	BZ	_00123_DS_
;	.line	120; stepperirq.c	if (MOTOR.nco_hi_bit) { // speed NCO overflowed and we have steps left => generate pulse
	BNC	_00124_DS_
;	.line	121; stepperirq.c	MOTOR.steps--;
	DECF	r0x00, F
	MOVF	r0x00, W
	MOVWF	(_g + 17), B
;	.line	122; stepperirq.c	STEP_OUTPUT = 0;// STEP signal = 0 generates a rising edge on next interrupt
	BCF	_LATDbits, 1
	BRA	_00124_DS_
_00123_DS_:
;	.line	124; stepperirq.c	} else if (MOTOR.has_next) { // check if there more steps in the queue
	BTFSS	(_g + 18), 0, B
	BRA	_00124_DS_
;	.line	125; stepperirq.c	MOTOR.has_next = 0;// signals to higher level that we have consumed the next 'step' so to speak
	BCF	(_g + 18), 0, B
;	.line	126; stepperirq.c	MOTOR.steps = MOTOR.next_steps;
	MOVFF	(_g + 21), r0x00
	MOVF	r0x00, W
	MOVWF	(_g + 17), B
;	.line	127; stepperirq.c	MOTOR.speed = MOTOR.next_speed;
	MOVFF	(_g + 19), r0x00
	MOVFF	(_g + 20), r0x01
	MOVF	r0x00, W
	MOVWF	(_g + 15), B
	MOVF	r0x01, W
	MOVWF	(_g + 16), B
;	.line	130; stepperirq.c	if (MOTOR.next_dir) {
	BTFSS	(_g + 18), 1, B
	BRA	_00118_DS_
;	.line	131; stepperirq.c	DIR_OUTPUT=1;
	BSF	_LATCbits, 1
	BRA	_00124_DS_
_00118_DS_:
;	.line	133; stepperirq.c	DIR_OUTPUT=0;
	BCF	_LATCbits, 1
_00124_DS_:
;	.line	136; stepperirq.c	MOTOR.has_next = 1; // FIXME: THIS LINE JUST FOR TESTING
	BSF	(_g + 18), 0, B
;	.line	118; stepperirq.c	MOTOR.nco += MOTOR.speed;
	MOVF	(_g + 24), W, B
	ADDWF	(_g + 22), F, B
	MOVF	(_g + 25), W, B
	ADDWFC	(_g + 23), F, B
;	.line	119; stepperirq.c	if (MOTOR.steps) {
	MOVFF	(_g + 26), r0x00
	MOVF	r0x00, W
	BZ	_00133_DS_
;	.line	120; stepperirq.c	if (MOTOR.nco_hi_bit) { // speed NCO overflowed and we have steps left => generate pulse
	BNC	_00134_DS_
;	.line	121; stepperirq.c	MOTOR.steps--;
	DECF	r0x00, F
	MOVF	r0x00, W
	MOVWF	(_g + 26), B
;	.line	122; stepperirq.c	STEP_OUTPUT = 0;// STEP signal = 0 generates a rising edge on next interrupt
	BCF	_LATDbits, 2
	BRA	_00134_DS_
_00133_DS_:
;	.line	124; stepperirq.c	} else if (MOTOR.has_next) { // check if there more steps in the queue
	BTFSS	(_g + 27), 0, B
	BRA	_00134_DS_
;	.line	125; stepperirq.c	MOTOR.has_next = 0;// signals to higher level that we have consumed the next 'step' so to speak
	BCF	(_g + 27), 0, B
;	.line	126; stepperirq.c	MOTOR.steps = MOTOR.next_steps;
	MOVFF	(_g + 30), r0x00
	MOVF	r0x00, W
	MOVWF	(_g + 26), B
;	.line	127; stepperirq.c	MOTOR.speed = MOTOR.next_speed;
	MOVFF	(_g + 28), r0x00
	MOVFF	(_g + 29), r0x01
	MOVF	r0x00, W
	MOVWF	(_g + 24), B
	MOVF	r0x01, W
	MOVWF	(_g + 25), B
;	.line	130; stepperirq.c	if (MOTOR.next_dir) {
	BTFSS	(_g + 27), 1, B
	BRA	_00128_DS_
;	.line	131; stepperirq.c	DIR_OUTPUT=1;
	BSF	_LATCbits, 2
	BRA	_00134_DS_
_00128_DS_:
;	.line	133; stepperirq.c	DIR_OUTPUT=0;
	BCF	_LATCbits, 2
_00134_DS_:
;	.line	136; stepperirq.c	MOTOR.has_next = 1; // FIXME: THIS LINE JUST FOR TESTING
	BSF	(_g + 27), 0, B
;	.line	118; stepperirq.c	MOTOR.nco += MOTOR.speed;
	MOVF	(_g + 33), W, B
	ADDWF	(_g + 31), F, B
	MOVF	(_g + 34), W, B
	ADDWFC	(_g + 32), F, B
;	.line	119; stepperirq.c	if (MOTOR.steps) {
	MOVFF	(_g + 35), r0x00
	MOVF	r0x00, W
	BZ	_00143_DS_
;	.line	120; stepperirq.c	if (MOTOR.nco_hi_bit) { // speed NCO overflowed and we have steps left => generate pulse
	BNC	_00144_DS_
;	.line	121; stepperirq.c	MOTOR.steps--;
	DECF	r0x00, F
	MOVF	r0x00, W
	MOVWF	(_g + 35), B
;	.line	122; stepperirq.c	STEP_OUTPUT = 0;// STEP signal = 0 generates a rising edge on next interrupt
	BCF	_LATDbits, 3
	BRA	_00144_DS_
_00143_DS_:
;	.line	124; stepperirq.c	} else if (MOTOR.has_next) { // check if there more steps in the queue
	BTFSS	(_g + 36), 0, B
	BRA	_00144_DS_
;	.line	125; stepperirq.c	MOTOR.has_next = 0;// signals to higher level that we have consumed the next 'step' so to speak
	BCF	(_g + 36), 0, B
;	.line	126; stepperirq.c	MOTOR.steps = MOTOR.next_steps;
	MOVFF	(_g + 39), r0x00
	MOVF	r0x00, W
	MOVWF	(_g + 35), B
;	.line	127; stepperirq.c	MOTOR.speed = MOTOR.next_speed;
	MOVFF	(_g + 37), r0x00
	MOVFF	(_g + 38), r0x01
	MOVF	r0x00, W
	MOVWF	(_g + 33), B
	MOVF	r0x01, W
	MOVWF	(_g + 34), B
;	.line	130; stepperirq.c	if (MOTOR.next_dir) {
	BTFSS	(_g + 36), 1, B
	BRA	_00138_DS_
;	.line	131; stepperirq.c	DIR_OUTPUT=1;
	BSF	_LATBbits, 6
	BRA	_00144_DS_
_00138_DS_:
;	.line	133; stepperirq.c	DIR_OUTPUT=0;
	BCF	_LATBbits, 6
_00144_DS_:
;	.line	136; stepperirq.c	MOTOR.has_next = 1; // FIXME: THIS LINE JUST FOR TESTING
	BSF	(_g + 36), 0, B
_00146_DS_:
	MOVFF	PREINC1, FSR2L
	MOVFF	PREINC1, PCLATU
	MOVFF	PREINC1, PCLATH 
	MOVFF	PREINC1, FSR0H
	MOVFF	PREINC1, FSR0L
	MOVFF	PREINC1, PRODH
	MOVFF	PREINC1, PRODL
	MOVF	PREINC1, W
	MOVFF	PREINC1, BSR
	MOVFF	PREINC1, STATUS
;	.line	157; stepperirq.c	LED_PIN = 0;
	BCF	_LATBbits, 4
	RETFIE	0x01



; Statistics:
; code size:	  478 (0x01de) bytes ( 0.36%)
;           	  239 (0x00ef) words
; udata size:	    0 (0x0000) bytes ( 0.00%)
; access size:	    2 (0x0002) bytes


	end
