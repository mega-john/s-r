/*
 * ServoRevers.c
 *
 * Created: 24.12.2013 16:29:58
 *  Author: estarcev
 *  
 *  в протеусе при частоте 4,8мгц
 *  1,0мс - 600
 *  1,5мс - 900
 *  2,0мс - 1200
 *  в протеусе при частоте 9,6мгц
 *  1,0мс - 1200
 *  1,5мс - 1800
 *  2,0мс - 2400
 */

#include <inttypes.h>
//#include <stdbool.h>
#include <avr/io.h>
#include <avr/interrupt.h>
//#include <avr/eeprom.h>
//#include <avr/wdt.h>
//#include "EventQueue.h"

//#define FREQUENSY_4_8
#define DIVIDER				8


#define MIN_PWM_PULSE		1000
#define NUTRAL_PWM_PULSE	1500
#define MAX_PWM_PULSE		2000

#ifdef FREQUENSY_4_8
	#define FREQUENSY_VALUE	4.8
#else
	#define FREQUENSY_VALUE	9.6
#endif

#define PWM_MIN				(MIN_PWM_PULSE * FREQUENSY_VALUE / DIVIDER)		// number of pulses in 1000 uS at 4.8MHz with /8 prescailer = 1000 * 4.8 / 8 = 600  | 1200
#define PWM_NEUTRAL			(NUTRAL_PWM_PULSE * FREQUENSY_VALUE / DIVIDER)	// number of pulses in 1500 uS at 4.8MHz with /8 prescailer = 1500 * 4.8 / 8 = 900  | 1800
#define PWM_MAX				(MAX_PWM_PULSE * FREQUENSY_VALUE / DIVIDER)		// number of pulses in 2000 uS at 4.8MHz with /8 prescailer = 2000 * 4.8 / 8 = 1200 | 2400
#define INV_PWM_VALUE		(PWM_MAX + PWM_MIN)

#define PWM_INPUT_PIN		PCINT0
#define OUT_NORMAL_PIN		PORTB3
#define OUT_INVERSE_PIN		PORTB4

#define sb(port, bit) (port |= _BV(bit))	//set bit
#define cb(port, bit) (port &= ~_BV(bit))	//clear bit
#define tb(port, bit) (port ^= _BV(bit))	//toggle bit
#define compare_bit(byte1, byte2, bit) ((!(_SFR_BYTE(byte1) & _BV(bit))) && (_SFR_BYTE(byte2) & _BV(bit)))
#define CheckBit(port, bit) (port & _BV(bit))

//#define TIMEOUT_MS(t)  ((uint32_t)t  * 30 / 256)    //4.8MHz / (8 prescailer * 256 full timer cycle * 1000 since we are counting in ms)
//#define TIMEOUT_10US(t)  ((uint32_t)t  * 6 / 256)

typedef enum statuses
{
    CAPTURE_INPUT_PULSE = 0,
    MEASURE_PULSE_LEN,
    WORK,
    NONE
}statuses;

typedef union
{
    struct
    {
        uint8_t l;
        uint8_t h;
    };
    uint16_t val;
} timeUnion;

volatile statuses status = NONE;
volatile uint8_t tcnth = 0;
volatile uint16_t inputPulseLen = 0;
volatile uint16_t inversePulseLen = 0;//инвертированный импульс
uint8_t outNormalReady = 0;
uint8_t outInverseReady = 0;

void __jumpMain     (void) __attribute__ ((naked)) __attribute__ ((section (".init0")));

void __jumpMain(void)
{
    asm volatile ( ".set __stack, %0" :: "i" (RAMEND) );
    asm volatile ( "clr __zero_reg__" );        // r1 set to 0
    asm volatile ( "rjmp main");                   // jump to main()
}


void __inline__ startTimer()
{
    // Reset counter counters
    tcnth = 0;
    TCNT0 = 0;
    
    // Run timer at 4.8MHz/8 = 600 kHz
    // This gives 1.667 uSec timer tick, 426.667 uSec timer interval
    // Almost 28 seconds with additional 16bit SW timer value
    // Run timer at 9.6MHz/8 = 1200 kHz
    // This gives 0.8333 uSec timer tick, 213.333 uSec timer interval
    // Almost 14 seconds with additional 16bit SW timer value
    if(DIVIDER == 8)
    {
        TCCR0B = 0 << CS02 | 1 << CS01 | 0 << CS00; // run timer with prescailer f/8
    }
    else
    {
        TCCR0B = 0 << CS02 | 0 << CS01 | 1 << CS00; // run timer with prescailer f/1
    }
    TIMSK0 = 1 << TOIE0;
}

void __inline__ stopTimer()
{
    TCCR0B = 0;
}

void __inline__ startPWMInput()
{
	// Use PCINT1 pin as input
	PCMSK = 1 << PWM_INPUT_PIN;
	// Enable Pin Change interrupt
	GIMSK |= 1 << PCIE;
}

void __inline__ stopPWMInput()
{
    cb(GIMSK, PCIE);
}

//timer0 overflow interrupt
ISR(TIM0_OVF_vect, ISR_NAKED)
{
    // Increment high byte of the HW counter
    //tcnth++;
     asm(
     "push	r24             \n"
     "lds	r24, (tcnth)    \n"
     "subi	r24, 0xFF       \n"
     "sts	(tcnth), r24    \n"
     "pop	r24             \n"
     "reti                  \n"    
     );

}

// Pin Change interrupt
ISR(PCINT0_vect)
{
	static timeUnion measureTime;

	if(CheckBit(PINB, PWM_INPUT_PIN)) // On raising edge just capture current timer value
	{
    	measureTime.val = 0;
    	tcnth = 0;
    	TCNT0 = 0;
	}
	else // On failing edge calculate pulse length
	{
    	// It may happen that Pin Change Interrupt occurs at the same time as timer overflow
    	// Since timer overflow interrupt has lower priority let's do its work here (increment tcnth)
   	    measureTime.h = tcnth;
	    measureTime.l = TCNT0;
    	if(CheckBit(TIFR0, TOV0))
    	{
        	measureTime.h++;
    	}

    	inputPulseLen = measureTime.val;
        status = MEASURE_PULSE_LEN;
	}
}

int main(void)
{
    timeUnion workTime;

    // Set up ports
    PORTB = 1 << PWM_INPUT_PIN | 1 << OUT_NORMAL_PIN | 1 << OUT_INVERSE_PIN ; // OUTs switched off, pull-up for PWM_INPUT_PIN
    DDRB = 0 << PWM_INPUT_PIN | 1 << OUT_NORMAL_PIN | 1 << OUT_INVERSE_PIN; // output mode for OUT pins, input mode for PWM_INPUT_PIN pin

	cb(PORTB, OUT_NORMAL_PIN);
	cb(PORTB, OUT_INVERSE_PIN);
    	
    status = NONE;
    sei();
    	
    while(1)
    {
        switch(status)
        {
            case NONE:
                startPWMInput();
                startTimer();
                status = CAPTURE_INPUT_PULSE;
            case CAPTURE_INPUT_PULSE:
            break;
            case MEASURE_PULSE_LEN:
                stopPWMInput();
                stopTimer();
                inversePulseLen = INV_PWM_VALUE - inputPulseLen;
                sb(PORTB, OUT_NORMAL_PIN);
                sb(PORTB, OUT_INVERSE_PIN);
                outInverseReady = 0;
                outNormalReady = 0;
                status = WORK;
                startTimer();
            break;
            case WORK:
            {
                workTime.h = tcnth;
                workTime.l = TCNT0;
                if(workTime.val >= inputPulseLen)
                {
                    cb(PORTB, OUT_NORMAL_PIN);
                    outNormalReady = 1;
                }
                if(workTime.val >= inversePulseLen)
                {
                    cb(PORTB, OUT_INVERSE_PIN);
                    outInverseReady = 1;
                }
                if(outNormalReady && outInverseReady)
                {
                    stopTimer();
                    status = NONE;
                }
            }
            break;
        }
    }
}