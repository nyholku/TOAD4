; NOTE: USEFUL DEBUG TOOL HERE: http://www.microchip.com/forums/m596618.aspx
;--------------------------------------------------------
; File: hi_speef_irq.asm
;--------------------------------------------------------
;
	list	p=18f45k50
;
	radix dec
;
;--------------------------------------------------------
; public variables in this module
;--------------------------------------------------------
;
	global  _g_stepper_states
	global	_g_busy_flags
	global	_g_busy_bits
	global	_g_ready_flags
	global	_g_ready_bits
;
	global	_high_priority_interrupt_service
	global	_g_pwm_out
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
	extern	_PIR1
	extern	_TMR0L
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
#define motor_size 20

_g_stepper_states	res 4*motor_size
_g_busy_flags		res 1
_g_busy_bits		res 1
_g_ready_flags		res 1
_g_ready_bits		res 1
_g_pwm_out			res	1
;
;
;--------------------------------------------------------
;	Todo
;--------------------------------------------------------
;
;
;--------------------------------------------------------
;	Define values to access the global state
;--------------------------------------------------------
;

#define MOTOR_X (_g_stepper_states + 0 * motor_size)
#define MOTOR_Y (_g_stepper_states + 1 * motor_size)
#define MOTOR_Z (_g_stepper_states + 2 * motor_size)
#define MOTOR_4 (_g_stepper_states + 3 * motor_size)

#define nco 0
#define speed 2
#define next_speed 4
#define steps 6
#define next_steps 7

#define flags 9
#define update_pos_bit 0
#define next_dir_bit 1
; next four are obsolete
#define ready_bit 4
#define ready2_bit ; 5
#define not_busy_bit 6
#define not_busy2_bit 7

#define ready_mask 10
;#define busy_mask 11



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
;ivec_0x1_high_priority_interrupt_service: code	0X000008
ivec_0x1_high_priority_interrupt_service: code	0X000808
	GOTO	_high_priority_interrupt_service
;
; Here we define a macro to do the actual step pulse generation
;
STEP_GENERATOR_MACRO macro motor_flag_bit,motor,step_out_port,step_out_bit,dir_out_port,dir_out_bit,DEBUG
;
	local not_busy
	local all_done
	local no_steps_left
	local next_dir_reverse
	local no_next
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
	BZ		no_steps_left
;
;	if (MOTOR.nco_hi_bit) { // speed NCO overflowed and we have steps left => generate pulse
;
	BNC		all_done
;
;	MOTOR.steps--;
;
	DECF	(motor + steps), F
;
;	STEP_OUTPUT = 0;// STEP signal = 0 generates a rising edge on next interrupt
;
	BCF		step_out_port, step_out_bit
	BRA		all_done
;
no_steps_left:
;
;	check if not ready i.e. queue processing has pushed more our way
;
	BCF		(_g_busy_flags), motor_flag_bit, B
;
	MOVF	_g_busy_bits, W, B
	IORWF	_g_ready_bits, W, B
	ANDWF	(motor + ready_mask), W, B
	BNZ		not_busy
;
;
	IF DEBUG==1
	BCF	_LATBbits,4
	ENDIF
;
	BSF 	_PIR1, 2   	; // trig the queue processing with sw-interrupt
	BSF		(motor + flags), update_pos_bit, B
	BSF		(_g_busy_flags), motor_flag_bit, B
	BSF		(_g_ready_flags), motor_flag_bit, B
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
	BRA		next_dir_reverse
;
;		DIR_OUTPUT=1;
;
	BSF		dir_out_port, dir_out_bit
	BRA		all_done
;

not_busy:
	IF DEBUG==1
	BSF	_LATBbits,4
	ENDIF
;
	BRA		all_done

;
next_dir_reverse:
;
;		DIR_OUTPUT=0;
;
	BCF		dir_out_port, dir_out_bit
;
all_done:
;
	endm
;
; ---------
;
S_stepperirq__high_priority_interrupt_service	code
;
_high_priority_interrupt_service:
;	MOVFF   BSR, POSTDEC1
;
	BANKSEL	_g_stepper_states;
;
;;;	BSF	_LATBbits, 4 ;// FIXME: THIS LINE JUST FOR TESTING
;
; FIXME, basically following is unnecessary also as this is the only hi priority interrupt....
	BTFSS	_PIR1bits, 1
	BRA	_00146_DS_
;
;
	BCF	_PIR1bits, 1
;
;	Turn ON all step outputs (=> rising edge if on previous interrupt the output was set OFF)
;
	MOVLW	0x0f
	IORWF	_LATD, F
;
	MOVFF	_g_busy_flags , _g_busy_bits
	MOVFF	_g_ready_flags , _g_ready_bits
;
	STEP_GENERATOR_MACRO 0, MOTOR_X, STEP_X_PORT, STEP_X_BIT, DIR_X_PORT, DIR_X_BIT, 0
;
	STEP_GENERATOR_MACRO 1, MOTOR_Y, STEP_Y_PORT, STEP_Y_BIT, DIR_Y_PORT, DIR_Y_BIT, 0
;
	STEP_GENERATOR_MACRO 2, MOTOR_Z, STEP_Z_PORT, STEP_Z_BIT, DIR_Z_PORT, DIR_Z_BIT, 0
;
	STEP_GENERATOR_MACRO 3, MOTOR_4, STEP_4_PORT, STEP_4_BIT, DIR_4_PORT, DIR_4_BIT, 0
;
_00146_DS_:
;
;
;
	MOVF    _g_pwm_out, W, B
	SUBWF   _TMR0L, W
	BC      pwmout0
	BSF     _LATBbits, 3
	BRA     pwmout1
pwmout0:
	BCF     _LATBbits, 3
pwmout1:

;
;	MOVFF   PREINC1, BSR
	RETFIE	0x01
;
;
	end
;
