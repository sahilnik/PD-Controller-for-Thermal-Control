/* Includes */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

/* Defines */
#define F_CPU 16000000
#define BAUD_RATE 9600
#define FOSC 16000000
#define TX_Buffer 255

/* Prototype Function declarations */
void initPowerControl(void);
void PWM(float input);
void initUSART(void);
void USART_Transmit(unsigned char data);
void StoreSerial(char ToWrite);
void SerialWriteNumber(void);
void PrintDelay (float varA, float varB, float varC, float varD);
void initADC(void);
void AnalogToTemp(void);
void CalculateAvgTemp(float temp);
void PDController(float measured_state);

/* Variable definitions */
static float duty = 0.0;
static float prev_duty = 0.0;

char SerialBuffer[TX_Buffer];
static uint8_t ReadPosition = 0;
static uint8_t WritePosition = 0;

static float Target_Temperature = 0.0;
static float Initial_Temperature = 0.0;
static float raw_temperature = 0.0;
static float true_temperature = 0.0;
static float Avg_temperature = 0.0;

char StringArray[100];
volatile static int chararray = 0;

static int lockcount = 0;
static int printcount = 0;

/*	------------------------------------------------------------------------------------------------------------------------------------------------------------------
	Main function
	------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/
int main(void) {
/*	Initialize PWM, ADC, and USART registers */
	initPowerControl();
	initUSART();
	initADC();

	while(1) {
/*		Read temperature */
		AnalogToTemp();
/*		Using counter to add delays with use of _delay_ms */
		if (lockcount < 200) {	
			lockcount++;
		}
/*		Wait for temperature reading to stabalize before setting Initial temperature
		Set Target temperature to 25 degrees above Initial */
		if (lockcount >= 50 && lockcount < 51) {
			Initial_Temperature = Avg_temperature;
			Target_Temperature = Initial_Temperature + 25.0;
		}
/*		Once Initial Temperature is set, run PD Controller to start controlling PWM */
		if (lockcount >= 200) {
			lockcount = 500;
			PDController(Avg_temperature);
		}
/*		Using counter to add delay to USART transmit without using _delay_ms */
		if (printcount < 8000) {
			printcount++;
		} else if (printcount >= 8000) {
			PrintDelay(Initial_Temperature,Target_Temperature,Avg_temperature,duty);
			printcount = 0;
		}
	}
	return 0;
}

/*	------------------------------------------------------------------------------------------------------------------------------------------------------------------
	Name:			initPowerControl()
	Input:			void
	Description:	Initialize Power Control registers to allow for CTC timer and use of Compare and Match to generate PWM 
	------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/
void initPowerControl(void) {
	int E2 = 6; //M2 Speed Control
	
/*	Set to fast PWM mode with OCR0A as TOP*/
	TCCR0A |= (1<<COM0A1)|(1<<WGM01)|(1<<WGM00);
/*	Set prescaler = 0*/
	TCCR0B |= (1<<CS00);
/*	Init counter. Start at 0 for Compare and Match */
	TCNT0 = 0;
/*	Set the compare value to control duty cycle. 127 is half of 8 bit maximum (255). 
	Bulb is rated for 6 VDC, therefore will use 127 as maximum duty cycle */
	OCR0A = 0;
	//OCR0B = 100;
/*	Set M2 control pin as output
	Port D, Pin 6 controls M2 speed */
	DDRD |= (1 << E2);
}

/*	------------------------------------------------------------------------------------------------------------------------------------------------------------------
	Name:			PWM(float input)
	Input:			Integer value of percentage of duty cycle
	Description:	Set PWM duty cycle of OCR0A based on integer input
	------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/
void PWM(float input) {
/*	Take input from 0% to 100% as integer 
	Calculate double value of duty cycle based on integer input
	and round double down to nearest integer */
	const float maxdutycycle = 128.0;
	duty = floor((double)(maxdutycycle*input*0.01));

/*	Maximum OCR0A is 128
	If duty cycle exceeds hardware limitations, set to 128 or 0 with respect to input */	
	if (duty > 128) {
		duty = 128;
	} else if (duty < 0) {
		duty = 0;
	}
	OCR0A = duty;
}

/*	------------------------------------------------------------------------------------------------------------------------------------------------------------------
	Name:			initUSART(void)
	Input:			None
	Description:	Initialize USART registers, set baud rate, and enable serial communication
	------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/
void initUSART(void) {
/*	Baud Rate = 9600, Clock speed is 16 MHz, Baud = 103 for 0.2% error
	Baud = ((ClockSpeed/BaudRate/16) - 1)
	Set baud */
	UBRR0H = (unsigned char)(103>>8);
	UBRR0L = (unsigned char)(103);
/*	Enable transmitter and receiver */
	UCSR0B = (1<<RXEN0)|(1<<TXEN0); //|(1<<TXCIE0)
/*	Set frame format: 8 bit data, 1 stop bit */
	UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);

}

/*	------------------------------------------------------------------------------------------------------------------------------------------------------------------
	Name:			USART_Transmit(unsigned char data)
	Input:			unsigned character to be transmitted	
	Description:	Waits for empty transmission buffer and places data into UDR0 buffer to be transmitted
	------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/
void USART_Transmit(unsigned char data) {
	
	if (chararray == 0) {
/*		Do nothing and wait for empty transmit buffer */
		while ( !( UCSR0A & (1<<UDRE0)) ) {
		}
/*		Put data into buffer, sends the data */
		UDR0 = data;
		
	} else if (chararray == 1) {
		
		if (WritePosition < sizeof(StringArray)) {
			WritePosition++;
		}
		
		if (ReadPosition != WritePosition) {
			UDR0 = SerialBuffer[ReadPosition];
			ReadPosition++;
				
			if (ReadPosition >= TX_Buffer) {
				ReadPosition = 0;
			}
		} else if (ReadPosition == WritePosition) {
			ReadPosition = 0;
			WritePosition = 0;
		}
		chararray = 0;
	}

}

/*	------------------------------------------------------------------------------------------------------------------------------------------------------------------
	Name:			StoreSerial(char ToWrite)
	Input:			Character elements to be written
	Description:	Writes bit to buffer array. Resets array position once buffer limit is reached/exceeded
	------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/

void StoreSerial(char ToWrite) {
	SerialBuffer[WritePosition] = ToWrite;
	USART_Transmit(SerialBuffer[WritePosition]);
	
	if (WritePosition >= TX_Buffer) {
		WritePosition = 0;
	}
}

/*	------------------------------------------------------------------------------------------------------------------------------------------------------------------
	Name:			SerialWrite(void)
	Input:			None	
	Description:	Sends each element of StringArray to be stored and transmitted
	------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/
void SerialWrite(void) {
	
	chararray = 1;
	/*	Increment through character area so all elements are sent to StoreSerial(); */
	for (uint8_t n = 0; n < strlen(StringArray); n++) {
		StoreSerial(StringArray[n]);
	}

	/*	Send empty bit until bit is set */
	if (UCSR0A & (1<<UDRE0)) {
		UDR0 = 0;
	}
}

/*	------------------------------------------------------------------------------------------------------------------------------------------------------------------
	Name:			PrintDelay (float varA, float varB, float varC, float varD)
	Input:			Floating value(s) to be printed 
	Description:	Provides float value within sprintf function to assign to StringArray 
	------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/
void PrintDelay (float varA, float varB, float varC, float varD) {
	sprintf(StringArray, "Initial Temperature = %0.2f Celsius\n\rTarget Temperature = %0.2f\n\rCurrent Temperature = %0.2f\n\rPWM Duty Cycle = %0.2f\n\r\n\r", varA, varB, varC, varD);
	SerialWrite();
}

/*	------------------------------------------------------------------------------------------------------------------------------------------------------------------
	Name:			initADC()
	Input:			None
	Description:	Initialize ADC registers 
	------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/
void initADC() {
/*	Initialize ADC registers to allow for ADC and set prescaler */
	ADMUX = (1 << REFS0) | (1 << MUX0);
	ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2);
/*	Set ADC1 to active */
	DIDR0 = (1 << ADC1D);
	
	AnalogToTemp();
}

/*	------------------------------------------------------------------------------------------------------------------------------------------------------------------
	Name:			AnalogToTemp()
	Input:			None
	Description:	Reads raw voltage from thermister and converts it to Celsius (true_temperature)
	------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/
void AnalogToTemp() {
	float Thermister;
	float Resistance;
	
	ADCSRA |= (1 << ADSC);
	raw_temperature = ADC;
/*	calculate the resistance */
	Resistance = ((10230000/raw_temperature) - 10000);
/*	calculate natural log of resistance */
	Thermister = log(Resistance);	
	/* Steinhart-Hart Thermistor Equation: */
	/*  Temperature in Kelvin = 1 / (A + B[ln(R)] + C[ln(R)]^3)		*/
	/*  where A = 0.001129148, B = 0.000234125 and C = 8.76741*10^-8  */
	Thermister = 1 / (0.001129148 + (0.000234125 * Thermister) + (0.0000000876741 * Thermister * Thermister * Thermister));
	true_temperature = Thermister - 273.15;/* convert kelvin to °C */
	
	CalculateAvgTemp(true_temperature);
}

/*	------------------------------------------------------------------------------------------------------------------------------------------------------------------
	Name:			CalculateAvgTemp (float temp)
	Input:			Temperature reading in Celsius
	Description:	Stores temperature reading in array and averages it for more stable reading
					Breaks out of for loop every iteration so new data can be read in for average
	------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/
void CalculateAvgTemp (float temp) {
/*	Setup array to calculate moving average of temperature reading */
	static float AvgTempArray[5];
	static float sum = 0.0;
	static int p = 0;
	static int q = 0;
	static int ArrayFillCheck = 0;
	
	for (q = p; q <= 4; q++) {
/*		If q = 4 then the array has been filled up */
		if (q == 4){
			ArrayFillCheck = 1;
		}
		if (ArrayFillCheck == 1){
/*			Logic to use a moving average. Subtract/Add the first element that is being replaced from the sum. 
			add new value to sum. Sum would be subtracted or added depending on if new element is negative or positive */
			if (AvgTempArray[q] <= 0 && sum <= 0 ){
				sum = sum - AvgTempArray[q];
				} else if (AvgTempArray[q] <= 0 && sum > 0 ){
				sum = sum + AvgTempArray[q];
				} else if (AvgTempArray[q] > 0 && sum > 0 ){
				sum = sum - AvgTempArray[q];
				} else if (AvgTempArray[q] > 0 && sum <= 0 ){
				sum = sum + AvgTempArray[q];
			}
		}
/*		Store temperature reading in array and calculate sum and average */
		AvgTempArray[q] = temp;
		sum = sum + AvgTempArray[q]; 
		Avg_temperature = sum / (4+1);
		break;
	}
/*	For loop uses external variable, p, as counter reference since for loop is being broken every iteration */
	p = q; p++;
	if (p > 4){ 
		p = 0; 
	}
}

/*	------------------------------------------------------------------------------------------------------------------------------------------------------------------
	Name:			PDController(float measured_state)
	Input:			Averaged Temperature reading
	Description:	PD Controller to provide control input to PWM based on initial temperature,
					current temperature, and desired temperature
	------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/
void PDController(float measured_state) {
/*	Variable setup */
	static float Kp = 45;
	static float Kd = 0.25;
	static float derivative = 0.0;
	static float control_signal = 0.0;
	static float error_signal = 0.0; 
	static float prev_error = 0.0;
	
/*	PD Calculations for Proportional and Derivative gains */
	error_signal = Target_Temperature - measured_state;
	derivative = error_signal - prev_error;
	control_signal = (Kp*error_signal) + (Kd*derivative);
/*	Send control signal to PWM function to change OCR0A value */
	PWM(control_signal);
/*	Observe previous error to update derivative calculations */
	prev_error = error_signal;
}

