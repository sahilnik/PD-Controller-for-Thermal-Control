# PD-Controller-for-Thermal-Control
Running on a Romeo V1.3 Microcontroller with a Mega328p.
NOTE: Programmed in C to setup MCU registers.
MCU controls an incandescent bulb through PWM output. 
A thermister connected via ADC output provides real time temperature reading of the system (bulb with thermister pressed against it). 
The Mega328p controls PWM output to increase brightness (and thus heat) of the bulb to reach desired temperature 
and maintain that temperature while minimizing rise time and overshoot. 
USART on Mega328p set up to transmit temperature readings and PWM output to serial terminal.
