
#define F_CPU 16000000L

#define BAUD 9600    // Define baud rate

#define VREF 5// Reference voltage



#include <stdbool.h>

#include <util/setbaud.h>

#include <avr/io.h>

#include <util/delay.h>

#include <stdlib.h>

#include <avr/interrupt.h>

#include <stdint.h>

#include <string.h>

#include <math.h>





#define ADC_PIN       1 // Defining which analog pin to read, We are reading ADC1, thus 1

#define            LED_PIN         PB0



//Declaring variables

float curr_Deg;

float curr_Vel;

float curr_Volt;

float prev_Vol;

float prev_Deg;

float prev_Vel;

float vel_Out;

float vel_1;

float vel_2;

float filterVel_1;

float filterVel_2;

float filterVel;

//Tested saturation values

float sat_High = 120;

float sat_Low = 100;

float velocity[9];

float calcVel;

int i=0;



//volatile uint8_t tot_overflow;  // Uses of interrupt

volatile uint16_t time_count;  // Uses of interrupt

// Initializing UART

void uart_init (void)

{

	UBRR0H = UBRRH_VALUE; // enabled by setbaud.h

	UBRR0L = UBRRL_VALUE;
	

	UCSR0B = (1<<TXEN0)|(1<<RXEN0);                                                // enable receiver and transmitter

	UCSR0C = (0<<USBS0)|(1<<UCSZ00)|(1<<UCSZ01);   // set frame format: 1 stop bit, 8bit data

}


// Data transmit function
void uart_transmit (unsigned char data) //(unsigned char data)

{
	while (!(UCSR0A & (1<<UDRE0)));                // wait for empty transmit buffer

	UDR0 = data;                                   // load data in the register, which sends it
}


// Data receive function

unsigned char uart_receive (void)

{
	while ( !(UCSR0A & (1<<RXC0)) );               // wait for data to be received

	return UDR0;                                   // get and return 8-bit data from buffer
}



// Float transmit function
void printfloat(float number)
{
	uint8_t *p = (uint8_t *)&number;

	uart_transmit(p[0]);

	uart_transmit(p[1]);

	uart_transmit(p[2]);

	uart_transmit(p[3]);
}



// Initializing ADC
void InitADC()
{
	ADMUX|=(1<<REFS0); // Sets 5v internal source as reference voltage

	ADCSRA|=(1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); // ADEN enables the ADC, ADPS0~2 sets pre-scaler
}



// Function that reads the ADC value from the designated channel
float ReadADC(uint8_t ADCchannel)
{

	//select ADC channel with safety mask

	ADMUX = (ADMUX & 0xF0) | (ADCchannel & 0x0F);

	//single conversion mode

	ADCSRA |= (1<<ADSC);

	// wait until ADC conversion is complete

	while( ADCSRA & (1<<ADSC) );

	return ADC;

}



// Function that converts the voltage reading into degrees
float Vol2D(float volt)
{

	if (volt>=0.4834 && volt<=0.714)
	{
		return 79*volt + 120.74;
	}


	else
	{
		return 79*volt-241.48;
	}
}



// Function that calculates the angular velocity
float D2Vel(float curr_Deg, float prev_Deg)
{
	return (curr_Deg-prev_Deg)/0.01; //We are sampling every 1ms
}


// Apply filter (Butterworth filter)
float butter(float t, float t_step1, float t_step2, float tf_step1, float tf_step2)
{

float b1 = 0.005543; // wn = 25/( (1/dt)/2)

float b2 = 0.01109; // wn = 25/( (1/dt)/2)

float b3 = b1;

float a1 = 1;

float a2 = -1.779; // wn = 25/( (1/dt)/2)

float a3 = 0.8008; // wn = 25/( (1/dt)/2)

float Out = (b1*t + b2*t_step1 + b3*t_step2 - a2*tf_step1 - a3*tf_step2);


// 4th order filter
	//float b0 = 0.0102;
	//
	//float b1 = 0.0408;
//
	//float b2 = 0.0613;
//
	//float b3 = b1;
	//
	//float b4 = b0;
	//
	//
	//float a0 = 1.0;
//
	//float a1 = -1.9684;
//
	//float a2 = 1.7359;
	//
	//float a3 = -0.7245;
	//
	//float a4 = 0.1204;
	//
//
	//float Out = (b1*t + b2*t_step1 + b3*t_step2 - a2*tf_step1 - a3*tf_step2);

	
	return Out;

}



// Saturation function (this will fix the issue of discontinuities occurring at breaking point and 180/-180 degrees)
float vel_after_sat(float curr_Deg,float prev_Deg, float sat_Low,float sat_High,float prev_Vel)
{
	if ((prev_Deg <= sat_Low && curr_Deg >= sat_Low) || (prev_Deg >= sat_High && curr_Deg <= sat_High))
	{

		return prev_Vel;
	}

	return D2Vel(curr_Deg,prev_Deg);
}



void timer0(void)
{
	OCR0A = 155; //interrupts happen when the counter hits this value

	TCNT0 = 0;

	TIMSK0 = TIMSK0 | (1<<OCIE0A);   //enable compare match interrupt

	TCCR0B |= (1<< CS02) | (1 << CS00);

	TCCR0A =(1<<WGM01);  //CTC

	sei();
}



ISR(TIMER0_COMPA_vect)
{
	//time_count++;

	curr_Volt = (float)VREF/1024 * ReadADC(ADC_PIN);

	curr_Deg = Vol2D(curr_Volt);

	//curr_Vel = D2Vel(curr_Deg,prev_Deg);

	curr_Vel = vel_after_sat(curr_Deg,prev_Deg,sat_High,sat_Low,prev_Vel); //calculate the current velocity

	//apply filter to smoothen the output velocity

	//After outputting the filtered velocity, set the variables as the previous time step

	prev_Deg = curr_Deg;

	prev_Vel = curr_Vel;

	filterVel = butter(curr_Vel,vel_1,vel_2,filterVel_1,filterVel_2);


	calcVel = -log(1-((filterVel*(80/22))/4000));


	//printfloat(curr_Deg+120);
	//printfloat(curr_Volt);
	printfloat(calcVel);

	vel_2 = vel_1;

	vel_1 = curr_Vel;

	filterVel_2 = filterVel_1;

	filterVel_1 = filterVel;

	TCNT0 = 0; //reset counter
}



int main(void)
{

	InitADC(); //Initializing the Analog-Digital Converter

	//Power up the Hall Effect sensor

	DDRC = (1 << PINC0);

	PORTC = (1 << PINC0);

	

	prev_Vol = (float)VREF/1024*ReadADC(ADC_PIN); //read the initial voltage before timer starts

	prev_Deg = Vol2D(prev_Vol);

	//timer1();

	timer0();

	uart_init(); // Initializing UART

	while(1);

	{


	}

	return 0;

}

