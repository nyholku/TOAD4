;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;  BootLoader.                                                             ;;
;;  Copyright (C) 2007 Diolan ( http://www.diolan.com )                     ;;
;;                                                                          ;;
;;  This program is free software: you can redistribute it and/or modify    ;;
;;  it under the terms of the GNU General Public License as published by    ;;
;;  the Free Software Foundation, either version 3 of the License, or       ;;
;;  (at your option) any later version.                                     ;;
;;                                                                          ;;
;;  This program is distributed in the hope that it will be useful,         ;;
;;  but WITHOUT ANY WARRANTY; without even the implied warranty of          ;;
;;  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           ;;
;;  GNU General Public License for more details.                            ;;
;;                                                                          ;;
;;  You should have received a copy of the GNU General Public License       ;;
;;  along with this program.  If not, see <http://www.gnu.org/licenses/>    ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Copyright (c) 2015 Kustaa Nyholm / SpareTimeLabs
; - modified NOT to use Extended Instruction Set (for compatibility with SDCC)
; - extensively optimized to still fit in the 2 kB boot block
;
;-----------------------------------------------------------------------------
; XTEA Encoding / Decoding
;-----------------------------------------------------------------------------
	#include "mpasmx.inc"
;-----------------------------------------------------------------------------
; Constants
;-----------------------------------------------------------------------------
; Access to boot_cmd
CODE_SIZE_OFFS		equ	5
CODE_START_OFFS		equ	6
;-----------------------------------------------------------------------------
; Global Variables
;-----------------------------------------------------------------------------
	extern	boot_cmd
	extern	boot_rep
;-----------------------------------------------------------------------------
; Local Variables
;-----------------------------------------------------------------------------
BOOT_DATA	UDATA
_sum	res	4	; long sum  LSB first
tmp1	res	4	; long tmp1 LSB first
tmp2	res	4	; long tmp2 LSB first
iter	res	1	; XTEA iterations
bcntr	res	1	; Encoded/Decoded bytes counter
	global	_fsr
_fsr	res	4	; Temporary storage for FSR's
;-----------------------------------------------------------------------------
; XTEA Algorithm Definitions
;-----------------------------------------------------------------------------
XTEA_ITERATIONS		equ	0x40	; Number of iterations on x1,x2
;-----------------------------------------------------------------------------
XTEA_KEYS_SECTION	CODE_PACK	0x001C	; Place ROM constants after vectors
#ifndef XTEA_KEY
XTEA_KEY_VAL	db	"0123456789ABCDEF"	; default 16 byte Key
#else
XTEA_KEY_VAL    db  XTEA_KEY
#endif
;-----------
DELTA		db	0xB9,0x79,0x37,0x9E	; Delta
MINUS_DELTA	db	0x47,0x86,0xC8,0x61	; (-)Delta
DELTA_ITER	db	0x40,0x6e,0xde,0x8d	; Delta*XTEA_ITERATIONS
;-----------------------------------------------------------------------------
; START
;-----------------------------------------------------------------------------
XTEA_CODE	CODE
	GLOBAL	xtea_encode
	GLOBAL	xtea_decode
;-----------------------------------------------------------------------------
;       xtea_encode
;-----------------------------------------------------------------------------
; DESCR :
; INPUT : boot_cmd
; OUTPUT: xtea_block
; NOTES :
;-----------------------------------------------------------------------------
xtea_encode:
	rcall	xtea_init				; Set TBLPTR, bcnt
	lfsr	FSR2, boot_rep + CODE_START_OFFS		; FSR2=&x1[0]
	lfsr	FSR1, boot_rep + (CODE_START_OFFS + 4)	; FSR1=&x2[0]
	; while( bcnt-=8 )
xtea_encode_loop
	rcall	xtea_init_iters		; sum=0, iter=XTEA_ITERATIONS
	; while( iter )
xtea_encode_iter
	; x1 += ((x2<<4 ^ x2>>5) + x2) ^ ( sum + key[ sum&0x03 ] )
	rcall	calc_x1
	rcall	sum_key_xor_add
	; sum += DELTA
	movlw	LOW(DELTA)
	rcall	add_sum_const
	; x2 += ((x1<<4 ^ x1>>5) + x1) ^ ( sum + key[ (sum>>11)&0x03 ] )
	rcall	calc_x2
	rcall	sum_key_xor_add

	decfsz	iter,f
	bra	xtea_encode_iter

	rcall	after_iters
	bnz	xtea_encode_loop

	; Restore FSR1, FSR2
	GLOBAL	restore_fsr1_fsr2
restore_fsr1_fsr2
	movff	_fsr, FSR1L
	movff	_fsr + 1, FSR1H
	movff	_fsr + 2, FSR2L
	movff	_fsr + 3, FSR2H

	return


;-----------------------------------------------------------------------------
;       xtea_decode
;-----------------------------------------------------------------------------
; DESCR :
; INPUT :
; OUTPUT:
; NOTES :
;-----------------------------------------------------------------------------
xtea_decode:
	rcall	xtea_init				; Set TBLPTR, bcnt
	lfsr	FSR1, boot_cmd + CODE_START_OFFS	; FSR1=&x1[0]
	lfsr	FSR2, boot_cmd + (CODE_START_OFFS + 4)	; FSR2=&x2[0]

	; while( bcnt-=8 )
xtea_decode_loop
	rcall	xtea_init_iters		; sum=0, iter=XTEA_ITERATIONS
	movlw	LOW(DELTA_ITER)
	rcall	add_sum_const		; sum = DELTA * XTEA_ITERATIONS

	; while( iter )
xtea_decode_iter
	; x2 -= ((x1<<4 ^ x1>>5) + x1) ^ ( sum + key[ (sum>>11)&0x03 ] )
	rcall	calc_x2
	rcall	sum_key_xor_sub

	; sum += -DELTA
	movlw	LOW(MINUS_DELTA)
	rcall	add_sum_const

	; x1 -= ((x2<<4 ^ x2>>5) + x2) ^ ( sum + key[ sum&0x03 ] )
	rcall	calc_x1
	rcall	sum_key_xor_sub

	decfsz	iter,f
	bra	xtea_decode_iter

	rcall	after_iters
	bnz	xtea_decode_loop

	bra	restore_fsr1_fsr2
;-----------------------------------------------------------------------------
; Local Functions
;-----------------------------------------------------------------------------
;
; Init Encoder/Decoder
; Assume TBLPTRU=0
xtea_init
	; Prepare Table Pointer to access KEY
	clrf	TBLPTRH

	movf	boot_cmd + CODE_SIZE_OFFS, W
	andlw	0x38
	movwf	boot_cmd + CODE_SIZE_OFFS	; size8 &= 0x38
	movwf	bcntr				; bcnt = size8

	; Store FSR1,FSR2
	GLOBAL  store_fsr1_fsr2
store_fsr1_fsr2
	movff	FSR1L, _fsr
	movff	FSR1H, _fsr + 1
	movff	FSR2L, _fsr + 2
	movff	FSR2H, _fsr + 3
	return
;
; Init next 8 byte Encode/Decode
;
xtea_init_iters
	clrf	_sum
	clrf	_sum + 1
	clrf	_sum + 2
	clrf	_sum + 3	; _sum = 0
	movlw	XTEA_ITERATIONS
	movwf	iter		; iter = XTEA_ITERATIONS
	return

;
; After 8 byte Encode/Decode
;
after_iters
	rcall	addfsr_FSR1_4
	rcall	addfsr_FSR1_4
	rcall	addfsr_FSR2_4
	rcall	addfsr_FSR2_4
	; bcnt -= 8
	movlw	0x08
	subwf	bcntr,f		; bcntr -=8
	return			; Z will be set if bcntr==0

calc_x1
	rcall	subfsr_FSR1_4
	rcall	addfsr_FSR2_4
	rcall	load_shift_xor_add
	movf	_sum,W
	return

;
addfsr_FSR1_4
 	rcall	addfsr_FSR1_2
addfsr_FSR1_2
 	movf	POSTINC1, W
 	movf	POSTINC1, W
 	return
;
addfsr_FSR2_4
 	rcall	addfsr_FSR2_2
addfsr_FSR2_2
 	movf	POSTINC2, W
 	movf	POSTINC2, W
 	return
;
calc_x2
	rcall	addfsr_FSR1_4
 	rcall	subfsr_FSR2_4
load_shift_xor_add
	rcall	load_shift	; tmp1=tmp2=*FSR2; tmp1<<=4 ; tmp2=>>5
	rcall	xor_tmp1_tmp2	; tmp1 = tmp1 ^ tmp2
	rcall	add_tmp1_FSR2	; tmp1 = tmp1 + *FSR2
	movf	_sum + 1, W
	rrncf	WREG, W
	rrncf	WREG, W
	rrncf	WREG, W		; W = sum>>11
	return

sum_key_xor_add
	rcall	sum_key		; tmp2 = sum + key[ w ]
	rcall	xor_tmp1_tmp2	; tmp1 = tmp1 ^ tmp2
	bra	add_FSR1_tmp1	; *FSR1 = *FSR1 + tmp1

sum_key_xor_sub
	rcall	sum_key		; tmp2 = sum + key[ w ]
	rcall	xor_tmp1_tmp2	; tmp1 = tmp1 ^ tmp2
	bra	sub_FSR1_tmp1	; *FSR1 = *FSR1 + tmp1

; tmp1=tmp2=*FSR2; tmp1<<4; tmp2>>5
load_shift
	movff	INDF2, tmp1
	movff	POSTINC2, tmp2
	movff	INDF2, tmp1 + 1
	movff	POSTINC2, tmp2 + 1
	movff	INDF2, tmp1 + 2
	movff	POSTINC2, tmp2 + 2
	movff	INDF2, tmp1 + 3
	movff	POSTINC2, tmp2 + 3
	rcall	subfsr_FSR2_4
	; tmp1 <<= 4
	rcall	tmp1_shft_l_2
	rcall	tmp1_shft_l_2
	; tmp2 >>= 5
	rcall	tmp2_shft_r_2
	rcall	tmp2_shft_r_1
tmp2_shft_r_2
	rcall	tmp2_shft_r_1
tmp2_shft_r_1
	bcf	STATUS,C        ; Carry=0
	rrcf	tmp2 + 3,f
	rrcf	tmp2 + 2,f
	rrcf	tmp2 + 1,f
	rrcf	tmp2 + 0,f
	return
;
tmp1_shft_l_2
	rcall   tmp1_shft_l_1
tmp1_shft_l_1
	bcf	STATUS,C        ; Carry=0
	rlcf	tmp1 + 0,f
	rlcf	tmp1 + 1,f
	rlcf	tmp1 + 2,f
	rlcf	tmp1 + 3,f
	return
;

; (*FSR1) = (*FSR1) + tmp1
add_FSR1_tmp1
	movf	tmp1, W
	addwf	POSTINC1
	movf	tmp1 + 1, W
	addwfc	POSTINC1
	movf	tmp1 + 2, W
	addwfc	POSTINC1
	movf	tmp1 + 3, W
	addwfc	POSTINC1
_xtea_1
subfsr_FSR1_4
 	rcall	subfsr_FSR1_2
subfsr_FSR1_2
 	movf	POSTDEC1, W
 	movf	POSTDEC1, W
	return

; (*FSR1) = (*FSR1) - tmp1
sub_FSR1_tmp1
	movf	tmp1, W
	subwf	POSTINC1
	movf	tmp1 + 1, W
	subwfb	POSTINC1
	movf	tmp1 + 2, W
	subwfb	POSTINC1
	movf	tmp1 + 3, W
	subwfb	POSTINC1
	bra	_xtea_1

; tmp1 = tmp1 + *FSR2
add_tmp1_FSR2
	movf	POSTINC2, W
	addwf	tmp1,f
	movf	POSTINC2, W
	addwfc	tmp1 + 1,f
	movf	POSTINC2, W
	addwfc	tmp1 + 2,f
	movf	POSTINC2, W
	addwfc	tmp1 + 3,f
subfsr_FSR2_4
 	rcall 	subfsr_FSR2_2
subfsr_FSR2_2
 	movf	POSTDEC2, W
 	movf	POSTDEC2, W
	return

; tmp1 = tmp1 ^ tmp2
xor_tmp1_tmp2
	movf	tmp2, W
	xorwf	tmp1,f
	movf	tmp2 + 1, W
	xorwf	tmp1 + 1,f
	movf	tmp2 + 2, W
	xorwf	tmp1 + 2,f
	movf	tmp2 + 3, W
	xorwf	tmp1 + 3,f
	return

; tmp2 = sum + key[ (w&0x03) ]
sum_key
	andlw   0x03            ; w = sum & 0x03
	; XTEA_KET must be located in first 256 bytes of code ROM
	rlncf	WREG
	rlncf	WREG		; W *=4
	addlw	LOW(XTEA_KEY_VAL)
	movwf	TBLPTRL		; TBLPTR = &key[ w ]

	lfsr	FSR0, tmp2
	bra		add_FSR0_sum_TBLPTR

; sum = sum + *((rom*)w)
add_sum_const
	movwf	TBLPTRL		; TBLPTR = &const
	lfsr	FSR0,_sum
; *FSR0 = sum + *TBLPTR
add_FSR0_sum_TBLPTR
	tblrd*+
	movf	_sum, W
	addwf	TABLAT, W
	movwf	POSTINC0
	tblrd*+
	movf	_sum + 1, W
	addwfc	TABLAT, W
	movwf	POSTINC0
	tblrd*+
	movf	_sum + 2, W
	addwfc	TABLAT, W
	movwf	POSTINC0
	tblrd*+
	movf	_sum + 3, W
	addwfc	TABLAT, W
	movwf	POSTINC0
	return
;-----------------------------------------------------------------------------
	END
