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
	global	_g_not_empty_flags
	global	_g_pop_flags
	global	_g_debug_count
;
	global	_high_priority_interrupt_service
	global	_g_hipri_int_flags
	global	_g_lopri_int_flags

;
;--------------------------------------------------------
; extern variables in this module
;--------------------------------------------------------
;
	extern	_LATBbits
	extern	_LATCbits
	extern	_LATDbits
	extern	_PIR1bits
	extern	_LATA
	extern	_LATD
	extern	_PIR1
	extern	_TMR0L
;
;--------------------------------------------------------
;	Equates to used internal registers
;--------------------------------------------------------
;


STATUS		equ	0xfd8
PCL		equ	0xff9
PCLATH		equ	0xffa
PCLATU		equ	0xffb
WREG		equ	0xfe8
BSR		equ	0xfe0
FSR0L		equ	0xfe9
FSR0H		equ	0xfea
FSR1L		equ	0xfe1
FSR2L		equ	0xfd9
INDF0		equ	0xfef
POSTINC0	equ	0xfee
POSTDEC1	equ	0xfe5
PREINC1		equ	0xfe4
PRODL		equ	0xff3
PRODH		equ	0xff4

;
;--------------------------------------------------------
;	Global variables
;--------------------------------------------------------
;
	udata 0x600
;
#define motor_size 20

_g_stepper_states	res 5*motor_size
_g_busy_flags		res 1
_g_busy_bits		res 1
_g_ready_flags		res 1
_g_not_empty_flags  	res 1
_g_hipri_int_flags  	res 1
_g_lopri_int_flags  	res 1
_g_pop_flags  		res 1
_g_step_out		res 2
_g_dir_out			res 2
_g_debug_count		res 1
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
#define MOTOR_A (_g_stepper_states + 3 * motor_size)
#define MOTOR_B (_g_stepper_states + 4 * motor_size)

#define nco 0
#define speed 2
#define next_speed 4
#define steps 6
#define next_steps 7

#define flags 9
#define update_pos_bit 0
#define next_dir_bit 1
#define next_forward_bit 4
#define next_reverse_bit 5

#define group_mask 10
;#define busy_mask 11



;
;--------------------------------------------------------
;	TOAD4 step/dir outputs
;--------------------------------------------------------
;

#define STEP_X_BIT 0
#define DIR_X_BIT 0

#define STEP_Y_BIT 1
#define DIR_Y_BIT 1

#define STEP_Z_BIT 2
#define DIR_Z_BIT 2

#define STEP_A_BIT 3
#define DIR_A_BIT 3

#define STEP_B_BIT 4
#define DIR_B_BIT 4

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
STEP_GENERATOR_MACRO macro motor_flag_bit,motor,step_out_bit,dir_out_bit,DEBUG
;
	local not_busy
	local all_done
	local no_steps_left
	local next_forward
	local next_reverse
	local no_next
	local no_int
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
;	STEP_OUTPUT = 1; // STEP signal = 1 generates a rising edge on next interrupt
;
	BSF		_g_step_out, step_out_bit, B
	BRA		all_done
;
no_steps_left:
;
;	check if not ready i.e. queue processing has pushed more our way
;
	BCF		(_g_busy_flags), motor_flag_bit, B
;
	MOVF	_g_hipri_int_flags, W, B
	ANDWF	(motor + group_mask), W, B
	BNZ		all_done
;
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
	BTFSC	(motor + flags), next_forward_bit, B
	BRA	next_forward
	BTFSC	(motor + flags), next_reverse_bit, B
	BRA	next_reverse
;
	BRA		all_done
;
;		DIR_OUTPUT=1;
;
next_forward
	CLRF		(motor+nco+1), B
	BSF		_g_dir_out, dir_out_bit
	BRA		all_done
;
next_reverse
	CLRF		(motor+nco+1), B
	BCF		_g_dir_out, dir_out_bit
	BRA		all_done
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
;
	BANKSEL	_g_stepper_states;
;
; Clear the interrupt flag CCP1IF
;
	BCF	_PIR1bits, 2
;
;	Update all outputs at once (step pulses are delayed by one interrupt period)
;
	MOVF    (_g_step_out + 0), W, B ; _g_step_out[0] was updated on previous interrupt so it is delayed
	IORWF   (_g_step_out + 1), W, B ; lengthen the pulses by or'ing the two last outputs
	MOVWF   _LATD
;
	MOVFF   (_g_step_out + 0), (_g_step_out + 1)
	CLRF    (_g_step_out + 0), B
;
	MOVFF   (_g_dir_out + 1), (_LATA) ; FIXME, shouldn't this be after STEP_GENERATOR_MACRO so that DIR gets updated asap
	MOVFF   (_g_dir_out + 0), (_g_dir_out + 1)
;
; Update flags
;
	MOVF    _g_ready_flags, W, B
	IORWF	_g_busy_flags, W, B
	MOVWF   _g_hipri_int_flags, B
;	MOVFF	_g_ready_flags , _g_ready_bits
;
	STEP_GENERATOR_MACRO 0, MOTOR_X, STEP_X_BIT, DIR_X_BIT, 0
;
	STEP_GENERATOR_MACRO 1, MOTOR_Y, STEP_Y_BIT, DIR_Y_BIT, 0
;
	STEP_GENERATOR_MACRO 2, MOTOR_Z, STEP_Z_BIT, DIR_Z_BIT, 0
;
	STEP_GENERATOR_MACRO 3, MOTOR_A, STEP_A_BIT, DIR_A_BIT, 0
;
	STEP_GENERATOR_MACRO 4, MOTOR_B, STEP_B_BIT, DIR_B_BIT, 0
;
;
_00146_DS_:
;
	RETFIE	0x01
;
;
	end
;
