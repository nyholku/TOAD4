; NOTE: USEFUL DEBUG TOOL HERE: http://www.microchip.com/forums/m596618.aspx
;--------------------------------------------------------
; File: hi_speef_irq.asm 
;--------------------------------------------------------
;
	list	p=18f4550
;
	radix dec
;
;--------------------------------------------------------
; public variables in this module
;--------------------------------------------------------
;
	global	_g_irq_flags;
	global  _g_stepper_states
	global	_high_priority_interrupt_service
;
;--------------------------------------------------------
; extern variables in this module
;--------------------------------------------------------
;
	extern	_LATBbits
	extern	_LATCbits
	extern	_LATDbits
	extern	_PIR1bits
	extern	_LATD
;
;--------------------------------------------------------
;	Equates to used internal registers
;--------------------------------------------------------
;


STATUS	equ	0xfd8
PCL	equ	0xff9
PCLATH	equ	0xffa
PCLATU	equ	0xffb
WREG	equ	0xfe8
BSR	equ	0xfe0
FSR0L	equ	0xfe9
FSR0H	equ	0xfea
FSR1L	equ	0xfe1
FSR2L	equ	0xfd9
INDF0	equ	0xfef
POSTINC0	equ	0xfee
POSTDEC1	equ	0xfe5
PREINC1	equ	0xfe4
PRODL	equ	0xff3
PRODH	equ	0xff4

;
;--------------------------------------------------------
;	Global variables
;--------------------------------------------------------
;
	udata 0x600
;
_g_stepper_states	res 64
_g_irq_flags 		res 1
;
;
;--------------------------------------------------------
;	Todo
;--------------------------------------------------------
;

;TODO:
;define motor at 0x600
;degine motor stuct in C
;define syncflags somewhere
;move _g to udata (save ROM space)

;
;--------------------------------------------------------
;	Define values to access the global state
;--------------------------------------------------------
;

#define _g_irq_flags 0

#define motor_size 16
#define MOTOR_X (_g_stepper_states + 0 * motor_size)
#define MOTOR_Y (_g_stepper_states + 1 * motor_size)
#define MOTOR_Z (_g_stepper_states + 2 * motor_size)
#define MOTOR_4 (_g_stepper_states + 3 * motor_size)

#define nco 0
#define speed 2
#define next_speed 4
#define steps 6
#define next_steps 7
#define flags 8
#define has_next_bit 0
#define next_dir_bit 1

;
;--------------------------------------------------------
;	TOAD4 step/dir outputs
;--------------------------------------------------------
;

#define STEP_X_PORT _LATDbits
#define STEP_X_BIT 0
#define DIR_X_PORT _LATCbits
#define DIR_X_BIT 0

#define STEP_Y_PORT _LATDbits
#define STEP_Y_BIT 1
#define DIR_Y_PORT _LATCbits
#define DIR_Y_BIT 1

#define STEP_Z_PORT _LATDbits
#define STEP_Z_BIT 2
#define DIR_Z_PORT _LATCbits
#define DIR_Z_BIT 2

#define STEP_4_PORT _LATDbits
#define STEP_4_BIT 3
#define DIR_4_PORT _LATBbits
#define DIR_4_BIT 6

;
;--------------------------------------------------------
;	Actual interrupt code
;--------------------------------------------------------
;
; This is the 'official'interrupt routine entry point, just a jump to the real stuff really
; 
ivec_0x1_high_priority_interrupt_service: code	0X000008
	GOTO	_high_priority_interrupt_service
;
; Here we define a macro to do the actual step pulse generation 
;
STEP_GENERATOR_MACRO macro motor,step_out_port,step_out_bit,dir_out_port,dir_out_bit
;
	local no_steps_left
	local pulse_gen_done
	local next_dir_reverse
;
;	MOTOR.nco += MOTOR.speed;
;
	MOVF	(motor + speed + 0), W, B
	ADDWF	(motor + nco + 0), F, B
	MOVF	(motor + speed + 1), W, B
	ADDWFC	(motor + nco + 1), F, B
;
;
;	if (MOTOR.steps) {
;
	MOVF	(motor + steps), W
	BZ	no_steps_left
;	
;	if (MOTOR.nco_hi_bit) { // speed NCO overflowed and we have steps left => generate pulse
;
	BNC	pulse_gen_done
;
;	MOTOR.steps--;
;
	DECF	(motor + steps), F
;
;	STEP_OUTPUT = 0;// STEP signal = 0 generates a rising edge on next interrupt
;
	BCF	step_out_port, step_out_bit
	BRA	pulse_gen_done
;
no_steps_left:
;
;		} else if (MOTOR.has_next) { // check if there more steps in the queue
;
	BTFSS	(motor + flags), has_next_bit, B
	BRA	pulse_gen_done
;
;	MOTOR.has_next = 0;// signals to higher level that we have consumed the next 'step' so to speak
;
	BCF	(motor + flags), has_next_bit, B
;
;	MOTOR.steps = MOTOR.next_steps;
;
	MOVFF	(motor + next_steps), (motor + steps)
;
;	MOTOR.speed = MOTOR.next_speed;
;
	MOVFF	(motor + next_speed + 0), (motor + speed + 0)
	MOVFF	(motor + next_speed + 1), (motor + speed + 1)
;
;	if (MOTOR.next_dir) {
;
	BTFSS	(motor + flags), next_dir_bit, B
	BRA	next_dir_reverse
;
;		DIR_OUTPUT=1;
;
	BSF	dir_out_port, dir_out_bit
	BRA	pulse_gen_done
	
next_dir_reverse:
;
;		DIR_OUTPUT=0;
;
	BCF	dir_out_port, dir_out_bit
;
pulse_gen_done:
;
;	MOTOR.has_next = 1; // FIXME: THIS LINE JUST FOR TESTING
;
;	BSF	(motor + flags), has_next_bit, B

	endm
;
; ---------
;
S_stepperirq__high_priority_interrupt_service	code
;
_high_priority_interrupt_service:

;
	BANKSEL	_g_stepper_states;
;
;	LED_PIN = 1;
;
	BSF	_LATBbits, 4 ;// FIXME: THIS LINE JUST FOR TESTING
;
; FIXME, basically following is unnecessary also as this is the only hi priority interrupt....
;	if (PIR1bits.TMR2IF) {
;
	BTFSS	_PIR1bits, 1
	BRA	_00146_DS_
;
;	.line	72; stepperirq.c	PIR1bits.TMR2IF = 0;
;
	BCF	_PIR1bits, 1
;
;	STEP_OUTPUT_PORT |= STEP_OUTPUT_ALL;
;
	MOVLW	0x0f
	IORWF	_LATD, F	
;
	STEP_GENERATOR_MACRO MOTOR_X, STEP_X_PORT, STEP_X_BIT, DIR_X_PORT, DIR_X_BIT	
;
	STEP_GENERATOR_MACRO MOTOR_Y, STEP_Y_PORT, STEP_Y_BIT, DIR_Y_PORT, DIR_Y_BIT	
;
	STEP_GENERATOR_MACRO MOTOR_Z, STEP_Z_PORT, STEP_Z_BIT, DIR_Z_PORT, DIR_Z_BIT	
;
	STEP_GENERATOR_MACRO MOTOR_4, STEP_4_PORT, STEP_4_BIT, DIR_4_PORT, DIR_4_BIT	
;
_00146_DS_:
;	.line	157; stepperirq.c	LED_PIN = 0;
	BCF	_LATBbits, 4 ;// FIXME: THIS LINE JUST FOR TESTING
;
;
;
	RETFIE	0x01
;
	end
