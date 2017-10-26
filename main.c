// main.c
// 
// A simple blinky program for ATtiny85
// Connect red LED at pin 2 (PB3)
//
// electronut.in

#include <avr/io.h>
#include <util/delay.h>
 
#define MIN_START_DC 60
#define MIN_CONT_DC 30
#define MAX_DC 255

#define SETPOINT_BOTTOM 30
#define SETPOINT_TOP    40

// PID constants
#define Kp 8 // e=30 ==> d=255

#define PWM
// #define INTERNAL_TEMP // MCU sensor or external on PB2 (ADC1) chan
// #define SPI_DEBUG // output values over SPI
// #define SPI_SLAVE_SELECT // output a slave select signal on PB0

#if defined(SPI_SLAVE_SELECT) && defined(PWM)
#error PWM and SPI Slave Select use the same pin: disable one of them
#endif // SPI_SLAVE_SELECT && PWM

#ifdef SPI_DEBUG

static const uint8_t usi_low = (1 << USIWM0) | (1 << USITC);
static const uint8_t usi_hi = (1 << USIWM0) | (1 << USITC) | (1 << USITC);

static void send_bytes(uint8_t buf[], uint8_t len)
{
#ifdef SPI_SLAVE_SELECT
	PORTB &= ~(1 << PB0); // slave select
#endif // SPI_SLAVE_SELECT

	for (int i = 0; i < len; ++i) {
		USIDR = buf[i];

		USICR |= (1 << USITC);
		__builtin_avr_delay_cycles(4);
		USIDR <<= 1;
		USICR |= (1 << USITC);
		__builtin_avr_delay_cycles(4);

		USICR |= (1 << USITC);
		__builtin_avr_delay_cycles(4);
		USIDR <<= 1;
		USICR |= (1 << USITC);
		__builtin_avr_delay_cycles(4);

		USICR |= (1 << USITC);
		__builtin_avr_delay_cycles(4);
		USIDR <<= 1;
		USICR |= (1 << USITC);
		__builtin_avr_delay_cycles(4);

		USICR |= (1 << USITC);
		__builtin_avr_delay_cycles(4);
		USIDR <<= 1;
		USICR |= (1 << USITC);
		__builtin_avr_delay_cycles(4);

		USICR |= (1 << USITC);
		__builtin_avr_delay_cycles(4);
		USIDR <<= 1;
		USICR |= (1 << USITC);
		__builtin_avr_delay_cycles(4);

		USICR |= (1 << USITC);
		__builtin_avr_delay_cycles(4);
		USIDR <<= 1;
		USICR |= (1 << USITC);
		__builtin_avr_delay_cycles(4);

		USICR |= (1 << USITC);
		__builtin_avr_delay_cycles(4);
		USIDR <<= 1;
		USICR |= (1 << USITC);
		__builtin_avr_delay_cycles(4);

		USICR |= (1 << USITC);
		__builtin_avr_delay_cycles(4);
		USIDR <<= 1;
		USICR |= (1 << USITC);
		__builtin_avr_delay_cycles(4);
	}
#ifdef SPI_SLAVE_SELECT
	PORTB |= (1 << PB0);
#endif // SPI_SLAVE_SELECT
}

#endif // SPI_DEBUG
 

int main (void)
{

#ifdef PWM
  DDRB |= (1 << DDB0);
#endif // PWM

#ifdef SPI_DEBUG
  DDRB |= (1 << DDB2) | (1 << DDB1) // SPI: USCK, D0, Slave Select
#ifdef SPI_SLAVE_SELECT 
	  | (1 << DDB0)
#endif // SPI_SLAVE_SLECT
	  ;

  USICR = (1 << USIWM0);
#endif // SPI_DEBUG

#ifdef PWM
  // PWM
  TCCR0A |= (1 << COM0A1) | (1 << WGM01) | (1 << WGM00); // non-inverted fast PWM
  //TCCR0A |= (1 << COM0A1) | (1 << COM0A0) | (1 << WGM01) | (1 << WGM00); // inverted fast PWM

  // Timer is configured when it is turned on and off below
  //TCCR0B |= (1 << CS00); // fclkio
  //TCCR0B |= (1 << CS01); // fclkio / 8
  //TCCR0B |= (1 << CS01) | (1 << CS00); // fclkio / 64
  //TCCR0B |= (1 << CS02); // fclkio / 256
  //TCCR0B |= (1 << CS02) | (1 << CS00); // fclkio / 1024

  OCR0A = 0; // duty cycle
#endif // PWM

  int8_t s = SETPOINT_BOTTOM;

  uint8_t prev_d = 0;

  while (1) {
 
#ifdef INTERNAL_TEMP
    ADMUX = 0b1111; // temperature
    ADMUX |= (1 << REFS1); // 1.1V ref
#else // !INTERNAL_TEMP
    ADMUX = 0b0001; // PB2 input (ADC1)
    ADMUX |= (1 << REFS2) | (1 << REFS1); // 2.56V ref
#endif // !INTERNAL_TEMP

    ADCSRA |= (1 << ADEN);

    // set prescaler to div64: 125kHz ADC clock at 8MHz
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1);

    ADCSRA &= ~(1 << ADIF);
    ADCSRA |= (1 << ADSC); // start conversion
    while (!(ADCSRA & (1 << ADIF))); // wait

    uint8_t result[2];
    result[0] = ADCL;
    result[1] = ADCH;

    uint16_t t = result[1];
    t <<= 8;
    t |= result[0];

#ifdef INTERNAL_TEMP

#define C_TO_A 12
    int8_t temp = (int16_t)t / C_TO_A;

#else // !INTERNAL_TEMP

    // 1.0V -- 50C, 10mV/C, 10-bit
    // 1024 => 2.56V
    // 4 counts/C
    // counts = 200 + 4 * degrees
    // degrees = (counts - 200) / 4
    // 400 -- 1.0V  -- 50C ==> 4

    int8_t temp = ((int16_t)t - 200)/4;
#endif // !INTERNAL_TEMP

    int16_t e = (int16_t)temp - s; // error
    int16_t c; // control
    uint8_t d = 0; // duty cycle

    if (e > 0) {
	c = Kp * e;
	s = SETPOINT_BOTTOM;
    } else {
	c = 0;
	s = SETPOINT_TOP;
    }

    c = (c <= MAX_DC) ? c : MAX_DC;
    if (c > 0) {
	if (d == 0) {
    	    d = c >= MIN_START_DC ? c : MIN_START_DC;
	} else {
	    d = c >= MIN_CONT_DC ? c : MIN_CONT_DC;
	}
    } else {
	d = c;
    }
 
    if (d == 0) {
	// we have to turn off the timer, because otherwise we still
	// get a pulse on every roll-over of the timer.
    	TCNT0 = 0; // will cause output to go HIGH
    	//__builtin_avr_delay_cycles(1024); // wait for one timer clock cycle
    	__builtin_avr_delay_cycles(8192); // wait for one timer clock cycle (8 MHz /1024)
    	TCCR0B &= ~((1 << CS02) | (1 << CS01) | (1 << CS00)); // turn off timer
    } else {
	OCR0A = d;   
	if (prev_d == 0) {
		TCCR0B |= (1 << CS02) | (1 << CS00); // re-enable the timer (CLK / 1024)
    		_delay_ms(2000);
	}
    }
    prev_d = d;

#ifdef SPI_DEBUG
#if 0
    send_bytes(result, 2);
    send_bytes(&d, 1);
#endif
#endif // SPI_DEBUG

    _delay_ms(250);
  }
 
  return 0;
}
