/*
* Fuctions: 				sensor_update, motion_pin_config, interface_pin_config, left_encoder_pin_config, right_encoder_pin_config, lcd_port_config
*									adc_pin_config, servo1_pin_config, servo2_pin_config, servo3_pin_config, port_init, buzzer_on, buzzer_off, beep, timer5_init
*									timer1_init, adc_init, left_position_encoder_interrupt_init, right_position_encoder_interrupt_init, ISR, ADC_Conversion, 
*									print_sensor, velocity, motion_set, forward, back, left, right, soft_left, soft_right, soft_left_2, soft_right_2, stop, update_current_node
*									angle_rotate, atpos2l, atpos2r, forward_mm, back_mm, left_degrees, right_degrees, soft_left_degrees, soft_right_degrees, 
*									soft_left_2_degrees, soft_right_2_degrees, rotate_to_position, Sharp_GP2D12_estimation, move_row, move_col, init_devices,
*									linear_distance_mm, send_signal, read_sensor, PID, move, servo_1, servo_2, servo_3, servo_1_free, servo_2_free, servo_3_free,
*									gripper_close, gripper_open, fruit, initial, drop, move_x2, linear_distance_mm_x2, sleft, sright, forward_mm_x2, backward_mm_x2,
*									pluck, camerapro, dist1
* Global Variables:	ShaftCountLeft, ShaftCountRight, adj
*/
//**********************************************************************************************
//**********************************************************************************************
#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "lcd.c"
//**********************************************************************************************
//**********************************************************************************************
unsigned char ADC_Conversion(unsigned char Ch);
//to keep track of left position encoder 
unsigned long int ShaftCountLeft = 0; 
//to keep track of right position encoder;
unsigned long int ShaftCountRight = 0; 
//to keep track of Front Sharp Sensor
unsigned char Front_Sharp_Sensor = 0;
//to keep track of Front IR Sensor 
unsigned char Front_IR_Sensor = 0;
//to keep track of Left White Line Sensor
unsigned char Left_black_line = 0;
//to keep track of Center White Line Sensor
unsigned char Center_black_line = 0;
//to keep track of Right White Line Sensor
unsigned char Right_black_line = 0;
//to store received data from UDR1
unsigned char data;
//Current Node of the Bot
unsigned char sNode = 0;
//Current Position of the Bot
int pos = 0;
//PID Control Variable
float control;
//Variable s
float s;
//Proportional Gain
float pGain = 170;
//Integral Gain
float iGain =  0.2;
//Differential Gain
float dGain =  110;
//Integral accumulator
int32_t eInteg = 0;
//Previous Error
int32_t ePrev  =0; 
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			sensor_update
* Input:     						None
* Output:    					None 
* Logic:     						Updates the value of Left_black_line, Center_black_line, Right_black_line, Front_Sharp_Sensor,
*										Front_IR_Sensor & Prints it on LCD
* Example Call:   			sensor_update();
*/
void sensor_update()
{
	Left_black_line = ADC_Conversion(3);	
	Center_black_line = ADC_Conversion(2);
	Right_black_line = ADC_Conversion(1);	
	Front_Sharp_Sensor = ADC_Conversion(11);
	Front_IR_Sensor = ADC_Conversion(6);
	print_sensor(1,1,3);
	print_sensor(1,5,2);
	print_sensor(1,9,1);
	print_sensor(2,4,11);
	print_sensor(2,8,6);
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			motion_pin_config
* Input:     						None
* Output:    					None 
* Logic:     						Function to configure ports to enable robot's motion
* Example Call:   			motion_pin_config();
*/
void motion_pin_config(void)
{
	DDRA = DDRA | 0x0F;
	PORTA = PORTA & 0xF0;
	DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
	PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			interface_pin_config
* Input:     						None
* Output:    					None 
* Logic:     						Function to configure ports to enable robot's interface
* Example Call:   			interface_pin_config();
*/
void interface_pin_config (void)
{
	DDRL = DDRL | 0x40;
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			left_encoder_pin_config
* Input:     						None
* Output:    					None 
* Logic:     						Function to configure INT4 (PORTE 4) pin as input for the left position encoder
* Example Call:   			left_encoder_pin_config();
*/
void left_encoder_pin_config(void)
{
	DDRE = DDRE & 0xEF;
	PORTE = PORTE | 0x10;
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			right_encoder_pin_config
* Input:     						None
* Output:    					None 
* Logic:     						Function to configure INT5 (PORTE 5) pin as input for the right position encoder
* Example Call:   			right_encoder_pin_config();
*/
void right_encoder_pin_config(void)
{
	DDRE = DDRE & 0xDF;
	PORTE = PORTE | 0x20;
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			lcd_port_config
* Input:     						None
* Output:    					None 
* Logic:     						Function to configure LCP PORT pin as input for the right position encoder
* Example Call:   			lcd_port_config();
*/
void lcd_port_config (void)
{
	DDRC = DDRC | 0xF7;
	PORTC = PORTC & 0x80;
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			adc_pin_config
* Input:     						None
* Output:    					None 
* Logic:     						Function to configure ADC pin
* Example Call:   			adc_pin_config();
*/
void adc_pin_config (void)
{
	DDRF = 0x00; 
	PORTF = 0x00;
	DDRK = 0x00;
	PORTK = 0x00;
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			servo1_pin_config
* Input:     						None
* Output:    					None 
* Logic:     						Configure PORTB 5 pin for servo motor 1 operation
* Example Call:   			servo1_pin_config();
*/
void servo1_pin_config (void)
{
	DDRB  = DDRB | 0x20;
	PORTB = PORTB | 0x20;
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			servo2_pin_config
* Input:     						None
* Output:    					None 
* Logic:     						Configure PORTB 6 pin for servo motor 2 operation
* Example Call:   			servo2_pin_config();
*/
void servo2_pin_config (void)
{
	DDRB  = DDRB | 0x40;
	PORTB = PORTB | 0x40;
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			servo3_pin_config
* Input:     						None
* Output:    					None 
* Logic:     						Configure PORTB 7 pin for servo motor 3 operation
* Example Call:   			servo3_pin_config();
*/
void servo3_pin_config (void)
{
	DDRB  = DDRB | 0x80;
	PORTB = PORTB | 0x80;
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			port_init
* Input:     						None
* Output:    					None 
* Logic:     						Function to initialize ports
* Example Call:   			port_init();
*/
void port_init(void)
{
	motion_pin_config();
	left_encoder_pin_config();
	right_encoder_pin_config();
	lcd_port_config();
	adc_pin_config();
	servo1_pin_config();
	servo2_pin_config();
	servo3_pin_config();
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			buzzer_on 
* Input:     						None
* Output:    					None 
* Logic:     						Function to turns on the buzzer
* Example Call:   			buzzer_on ();
*/
void buzzer_on (void)
{
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore | 0x08;
	PORTC = port_restore;
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			buzzer_off 
* Input:     						None
* Output:    					None 
* Logic:     						Function to turns on the buzzer
* Example Call:   			buzzer_off ();
*/
void buzzer_off (void)
{
	unsigned char port_restore = 0;
	port_restore = PINC;
 	port_restore = port_restore & 0xF7;
 	PORTC = port_restore;
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			beep 
* Input:     						t: defines the number of times you want buzzer to beep
* Output:    					None 
* Logic:     						Function to beep buzzer for desired number of time
* Example Call:   			beep(2);
*/
void beep(int t)
{
    for (int i = 0; i < t; i++)
    {
        buzzer_on();
        _delay_ms(100);
        buzzer_off();
        _delay_ms(100);
    }
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			timer5_init 
* Input:     						None
* Output:    					None 
* Logic:     						Timer 5 initialized in PWM mode for velocity control
*										Prescale:256
*										PWM 8bit fast, TOP=0x00FF
*										Timer Frequency:225.000Hz
* Example Call:   			timer5_init();
*/
void timer5_init()
{
	TCCR5B = 0x00;	
	TCNT5H = 0xFF;
	TCNT5L = 0x01;	
	OCR5AH = 0x00;
	OCR5AL = 0xFF;	
	OCR5BH = 0x00;
	OCR5BL = 0xFF;	
	OCR5CH = 0x00;
	OCR5CL = 0xFF;	
	TCCR5A = 0xA9;
	TCCR5B = 0x0B;
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			timer1_init 
* Input:     						None
* Output:    					None 
* Logic:     						TIMER1 initialization in 10 bit fast PWM mode
*										Prescale:256
*										PWM 10bit fast, TOP=0x03FF
*										Actual Value: 52.25Hz
* Example Call:   			timer1_init();
*/
void timer1_init(void)
{
 	TCCR1B = 0x00;
 	TCNT1H = 0xFC;
 	TCNT1L = 0x01;	
 	OCR1AH = 0x03;
 	OCR1AL = 0xFF;	
 	OCR1BH = 0x03;
 	OCR1BL = 0xFF;	
 	OCR1CH = 0x03;
 	OCR1CL = 0xFF;	
 	ICR1H  = 0x03;	
 	ICR1L  = 0xFF;
 	TCCR1A = 0xAB;
 	TCCR1C = 0x00;
 	TCCR1B = 0x0C;
}	
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			adc_init 
* Input:     						None
* Output:    					None 
* Logic:     						Initialize ADC
* Example Call:   			adc_init();
*/
void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;
	ADMUX = 0x20;
	ACSR = 0x80;
	ADCSRA = 0x86;
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			left_position_encoder_interrupt_init 
* Input:     						None
* Output:    					None 
* Logic:     						Initialize Left Position Encoder
* Example Call:   			left_position_encoder_interrupt_init();
*/
void left_position_encoder_interrupt_init(void) //Interrupt 4 enable
{
	cli();
	EICRB = EICRB | 0x02;
	EIMSK = EIMSK | 0x10;
	sei();
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			right_position_encoder_interrupt_init 
* Input:     						None
* Output:    					None 
* Logic:     						Initialize Right Position Encoder
* Example Call:   			right_position_encoder_interrupt_init();
*/
void right_position_encoder_interrupt_init(void)
{
	cli();
	EICRB = EICRB | 0x08;
	EIMSK = EIMSK | 0x20;
	sei();
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			ISR(INT5_vect) 
* Input:     						INT5_vect
* Output:    					None 
* Logic:     						ISR for right position encoder
* Example Call:   			
*/
ISR(INT5_vect)
{
	ShaftCountRight++;
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			ISR(INT4_vect) 
* Input:     						INT4_vect
* Output:    					None 
* Logic:     						ISR for left position encoder
* Example Call:   			
*/
ISR(INT4_vect)
{
	ShaftCountLeft++;
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			ADC_Conversion 
* Input:     						Ch: channel number for ADC Conversion
* Output:    					unsigned char: Converted ADC_VALUE
* Logic:     						Function For ADC Conversion
* Example Call:   			ADC_Conversion(2);
*/
unsigned char ADC_Conversion(unsigned char Ch) 
{
	unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;  			
	ADMUX= 0x20| Ch;	   		
	ADCSRA = ADCSRA | 0x40;
	while((ADCSRA&0x10)==0);
	a=ADCH;
	ADCSRA = ADCSRA|0x10;
	ADCSRB = 0x00;
	return a;
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			ADC_Conversion 
* Input:     						channel: channel number for ADC Conversion
*										row: row in the LCD
*										column: Column in LCD
* Output:    					None
* Logic:     						Function To Print Sesor Values At Desired Row And Coloumn Location on LCD
* Example Call:   			print_sensor(1, 1, 2);
*/
void print_sensor(char row, char column,unsigned char channel)
{
	unsigned char ADC_Value = ADC_Conversion(channel);
	lcd_print(row, column, ADC_Value, 3);
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			velocity 
* Input:     						left_motor: speed of left motor
*										right_motor: speed of right motr
* Output:    					None
* Logic:     						Function To set Velocity of the motors
* Example Call:   			velocity(180, 180);
*/
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			motion_set 
* Input:     						Direction: Set the direction of bot to mve
* Output:    					None
* Logic:     						Function used for setting motor's direction
* Example Call:   			motion_set(0x06);
*/
void motion_set(unsigned char Direction)
{
	unsigned char PortARestore = 0;

	Direction &= 0x0F;
	PortARestore = PORTA;
	PortARestore &= 0xF0;
	PortARestore |= Direction;
	PORTA = PortARestore;
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			forward 
* Input:     						None
* Output:    					None
* Logic:     						Function to set both wheels forward
* Example Call:   			forward();
*/
void forward(void)
{
	motion_set(0x06);
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			back 
* Input:     						None
* Output:    					None
* Logic:     						Function to set both wheels backward
* Example Call:   			back();
*/
void back(void)
{
	motion_set(0x09);
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			left 
* Input:     						None
* Output:    					None
* Logic:     						Function to move Left wheel backward, Right wheel forward
* Example Call:   			left();
*/
void left(void)
{
	velocity(180,180);
	motion_set(0x05);
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			right 
* Input:     						None
* Output:    					None
* Logic:     						Function to move Left wheel forward, Right wheel backward
* Example Call:   			right();
*/
void right(void)
{
	velocity(180,180);
	motion_set(0x0A);
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			soft_left 
* Input:     						None
* Output:    					None
* Logic:     						Function to move Left wheel stationary, Right wheel forward
* Example Call:   			soft_left();
*/
void soft_left(void)
{
	motion_set(0x04);
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			soft_right 
* Input:     						None
* Output:    					None
* Logic:     						Function to move Left wheel forward, Right wheel is stationary
* Example Call:   			soft_right();
*/
void soft_right(void) //Left wheel forward, Right wheel is stationary
{
	motion_set(0x02);
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			stop 
* Input:     						None
* Output:    					None
* Logic:     						Function to stop both the wheels
* Example Call:   			stop();
*/
void stop(void)
{
	motion_set(0x00);
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			update_current_node 
* Input:     						row: in which row bot is present
*										col: in which rolumn bot is present
* Output:    					None
* Logic:     						Function to update current node value
* Example Call:   			update_current_node(2, 2);
*/
void update_current_node(int row, int col) 
{
    sNode = (row * 7) + col; 
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			angle_rotate 
* Input:     						Degrees: to which degrees bot to rotate
* Output:    					None
* Logic:     						Function used for turning robot by specified degrees
* Example Call:   			angle_rotate(90);
*/
void angle_rotate(unsigned int Degrees)
{
	float ReqdShaft = 0;
	unsigned long int ReqdShaftCount = 0;

	ReqdShaft = (float)Degrees / 5.000;
	ReqdShaftCount = (unsigned long int)ReqdShaft;
	
	ShaftCountRight = 0;
	ShaftCountLeft = 0;

	while (1)
	{
		sensor_update();	
		if ((ShaftCountRight >= ReqdShaftCount) | (ShaftCountLeft >= ReqdShaftCount))
			break;
	}
	stop();
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			atpos2l 
* Input:     						None
* Output:    					None
* Logic:     						Function used to for turning robot by minimising error Left
* Example Call:   			atpos2l();
*/
void atpos2l()
{
	unsigned char sen;
	sen = ADC_Conversion(2);
	while(sen<35)
	{
		sen = ADC_Conversion(2);
		sleft();
	}
	stop();
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			atpos2lr
* Input:     						None
* Output:    					None
* Logic:     						Function used to for turning robot by minimising error Right
* Example Call:   			atpos2r();
*/
void atpos2r()
{
	unsigned char sen2 ;
	
	sen2 = ADC_Conversion(2);
	while(sen2<35)
	{
		sen2 = ADC_Conversion(2);
		sright();
	}
	stop();
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			forward_mm 
* Input:     						DistanceInMM: distance to move bot
* Output:    					None
* Logic:     						Function used to move bot forward for specified distance(on black line)
* Example Call:   			forward_mm(100);
*/
void forward_mm(unsigned int DistanceInMM)
{
	forward();
	linear_distance_mm(DistanceInMM);
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			back_mm 
* Input:     						DistanceInMM: distance to move bot
* Output:    					None
* Logic:     						Function used to move bot backward for specified distance(on black line)
* Example Call:   			back_mm(100);
*/
void back_mm(unsigned int DistanceInMM)
{
	back();
	linear_distance_mm(DistanceInMM);
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			left_degrees 
* Input:     						Degrees: Rotate degrees
* Output:    					None
* Logic:     						Function used to rotate bot through left for specified degrees
* Example Call:   			left_degrees(90);
*/
void left_degrees(unsigned int Degrees)
{
	left();
	angle_rotate(Degrees-30);
	atpos2l();
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			right_degrees 
* Input:     						Degrees: Rotate degrees
* Output:    					None
* Logic:     						Function used to rotate bot through right for specified degrees
* Example Call:   			right_degrees(90);
*/
void right_degrees(unsigned int Degrees)
{
	right();
	angle_rotate(Degrees-30);
	atpos2r();
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			soft_left_degrees 
* Input:     						Degrees: Rotate degrees
* Output:    					None
* Logic:     						Function used to rotate bot through left (soft_left) for specified degrees
* Example Call:   			soft_left_degrees(90);
*/
void soft_left_degrees(unsigned int Degrees)
{
	soft_left();
	Degrees = Degrees * 2;
	angle_rotate(Degrees);
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			soft_right_degrees 
* Input:     						Degrees: Rotate degrees
* Output:    					None
* Logic:     						Function used to rotate bot through right (soft_right) for specified degrees
* Example Call:   			soft_right_degrees(90);
*/
void soft_right_degrees(unsigned int Degrees)
{
	soft_right();
	Degrees = Degrees * 2;
	angle_rotate(Degrees);
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			rotate_to_position 
* Input:     						Degrees: Rotate degrees
* Output:    					None
* Logic:     						Function used to rotate bot to a specified position
* Example Call:   			rotate_to_position(90);
*/
void rotate_to_position(int degrees)
{
	int move_degrees = pos - degrees;

    if (move_degrees == 0)
    {
        pos = degrees;
    }
	else if (move_degrees > 180)
    {
        move_degrees = move_degrees - 180;
        right_degrees((unsigned int) move_degrees);
    }
    else if (move_degrees > 0)
    {
        left_degrees((unsigned int) move_degrees);
    }
    else if (move_degrees >= -180)
    {
        move_degrees = - move_degrees;
        right_degrees((unsigned int) move_degrees);
    }
    else
    {
        move_degrees = - move_degrees - 180;
        left_degrees((unsigned int) move_degrees);
    }
    pos = degrees;
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			Sharp_GP2D12_estimation 
* Input:     						adc_reading: Reading from sharp sensor
* Output:    					unsigned int: returns a value of sharp sensor converted
* Logic:     						Estimate the value of  Sharp Sensor to Distance
* Example Call:   			Sharp_GP2D12_estimation(90);
*/
unsigned int Sharp_GP2D12_estimation(unsigned char adc_reading)
{
	float distance;
	unsigned int distanceInt;
	distance = (int)(10.00*(2799.6*(1.00/(pow(adc_reading,1.1546)))));
	distanceInt = (int)distance;
	if(distanceInt>800)
	{
		distanceInt=800;
	}
	return distanceInt;
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			move_row
* Input:     				dNode: destination node
* Output:    				int: return value after reaching desired location
* Logic:     				Moves to the desired row of the destination node
* Example Call:   			move_row(12);
*/
int move_row(int dNode)
{
    int sRow=0,sCol=0,dRow=0;

    sRow = sNode / 7;
    sCol = sNode % 7;
    dRow = dNode / 7;

    if (dRow > sRow)
    {
        rotate_to_position(0); 

        while(1)
        {
			sensor_update();
			if (sRow == dRow)
            {
				break;
			}
            forward_mm(390);
            update_current_node(++sRow, sCol);
        }
    }
    else if (dRow < sRow)
    {
        rotate_to_position(180);

        while(1)
        {	
			sensor_update();
			if (sRow == dRow)
            {
				break;
			}
			forward_mm(390);
            update_current_node(--sRow, sCol);
        }
    }
    return 1;
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			move_col
* Input:     				dNode: destination node
* Output:    				int: return value after reaching desired location
* Logic:     				Moves to the desired column of the destination node
* Example Call:   			move_col(12);
*/
int move_col(int dNode)
{
    int sRow=0,sCol=0,dCol=0;
	
    sRow = sNode / 7;
    sCol = sNode % 7;
    dCol = dNode % 7;

    if (dCol > sCol)
    {
        rotate_to_position(90);
        while(1)
        {	
			sensor_update();
			if (sCol == dCol)
			{
                break;
			}
            forward_mm(390);
            update_current_node(sRow, ++sCol);
        }
    }
    else if (dCol < sCol)
    {
        rotate_to_position(270);
        while(1)
        {
			sensor_update();
			if (sCol == dCol)
			{
                break;
			}
			forward_mm(390);
            update_current_node(sRow, --sCol);
        }
    }
    return 1;
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			init_devices
* Input:     				None
* Output:    				None
* Logic:     				Function to initialize all the devices
* Example Call:   			init_devices();
*/
void init_devices()
{
	cli(); 										//Clears the global interrupt
	port_init();  								//Initializes all the ports
	left_position_encoder_interrupt_init();		//Left
	right_position_encoder_interrupt_init();
	adc_init();
	timer5_init();
	timer1_init();
	lcd_set_4bit();
	lcd_init();
	uart2_init();								//Initailize UART1 for serial communiaction
	interface_pin_config();
	sei(); 										//Enables the global interrupt 
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			linear_distance_mm
* Input:     				DistanceInMM: distance to be moved
* Output:    				None
* Logic:     				Function to move linear Forward or Backward Certain Distance
* Example Call:   			linear_distance_mm(100);
*/
void linear_distance_mm(unsigned int DistanceInMM)
{
    unsigned long int ReqdShaftCount = 0;
    ReqdShaftCount = (unsigned long int) DistanceInMM / 5.882;
    ShaftCountRight = 0;
    ShaftCountLeft = 0;
    while (1)
    {   
        move();
		if (ShaftCountRight > ReqdShaftCount && ShaftCountLeft > ReqdShaftCount)
		{
			break;
		}		
    }
    stop();
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			send_signal
* Input:     				None
* Output:    				None
* Logic:     				Function to send signal to R-Pi
* Example Call:   			send_signal(100);
*/
void send_signal()
{
	PORTL = 0x40;
	_delay_ms(100);
	PORTL = 0x00;
	_delay_ms(100);
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			read_sensor
* Input:     				None
* Output:    				float: returns the average value of the sensor
* Logic:     				Function calculates & returns the average value of Black_Line_Sensor
* Example Call:   			read_sensor();
*/
float read_sensor()
{
    uint16_t right,middle,left;
    uint8_t sensor1,sensor2, sensor3;
    float avgSensor = 0.0;
    right=ADC_Conversion(1);
    if(right>0x10)
    {
        sensor1 = 1;  
    }
    else
    {
        sensor1 = 0;
    }
    middle=ADC_Conversion(2);
    if(middle>0x10)
    {
        sensor2 = 1;  
    }
    else
    {
        sensor2 = 0;     
    }
    left=ADC_Conversion(3);
    if(left>0x10)
    {
        sensor3 = 1; 
    }
    else
    {
        sensor3 = 0;
    }
    if(sensor1==0 && sensor2==0 && sensor3==0)
    {
        return 0xFF;
    }
    avgSensor = (float) sensor1*1 + sensor2*2 + sensor3*3  ;
    avgSensor = (float) avgSensor / (sensor1 + sensor2 + sensor3 );
    return avgSensor;
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			PID
* Input:     				cur_value: current value of sensor
*							req_value: required value of sensor
* Output:    				float: returns the PID correction for the sensor
* Logic:     				Function calculates & returns the PID processed signal used for black line following
* Example Call:   			read_sensor();
*/
float PID(float cur_value,float req_value)
{
    float pid;
    float error;
    error = req_value - cur_value;
    pid = (pGain * error)  + (iGain * eInteg) + (dGain * (error - ePrev));
    eInteg += error;
    ePrev = error;
    return pid;
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			move
* Input:     				None
* Output:    				None
* Logic:     				Function used to move the value on Black line & Incurs the use of 
*							PID for error correction
* Example Call:   			move();
*/
void move()
{
    float sprev;
	sensor_update();
    s = read_sensor();
    
    if(s==0xFF)
    {
        s = sprev;
    }
    control = PID(s,2.0);
    if(control > 510)
        control = 510;
    if(control < -510)
        control = -510;
    if(control > 0.0)
    {
        if(control>255)
        {
            velocity(255,control-255);
        }
        else
        {
            velocity(255,255-control);
        }
    }
    if(control <= 0.0)
    {
        if(control<-255)
        {
            velocity(-(control+255),255);
        }
        else
        {
            velocity(255+control,255);
        }
    }
    sprev=s;
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			servo_1
* Input:     				None
* Output:    				None
* Logic:     				Function to rotate Servo 1 by a specified angle in the multiples
*							of 1.86 degrees
* Example Call:   			servo_1(180);
*/
void servo_1(unsigned char degrees)
{
	float PositionPanServo = 0;
	PositionPanServo = ((float)degrees / 1.86) + 35.0;
	OCR1AH = 0x00;
	OCR1AL = (unsigned char) PositionPanServo;
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			servo_2
* Input:     				None
* Output:    				None
* Logic:     				Function to rotate Servo 2 by a specified angle in the multiples
*							of 1.86 degrees
* Example Call:   			servo_2(180);
*/
void servo_2(unsigned char degrees)
{
	float PositionTiltServo = 0;
	PositionTiltServo = ((float)degrees / 1.86) + 35.0;
	OCR1BH = 0x00;
	OCR1BL = (unsigned char) PositionTiltServo;
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			servo_3
* Input:     				None
* Output:    				None
* Logic:     				Function to rotate Servo 3 by a specified angle in the multiples
*							of 1.86 degrees
* Example Call:   			servo_3(180);
*/
void servo_3(unsigned char degrees)
{
	float PositionServo = 0;
	PositionServo = ((float)degrees / 1.86) + 35.0;
	OCR1CH = 0x00;
	OCR1CL = (unsigned char) PositionServo;
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			servo_1_free
* Input:     				None
* Output:    				None
* Logic:     				function is used to unlock the servo motors from the any angle
*							and make them free by giving 100% duty cycle at the PWM. This function can be used to
*							reduce the power consumption of the motor if it is holding load against the gravity.
* Example Call:   			servo_1_free();
*/
void servo_1_free (void)
{
	OCR1AH = 0x03;
	OCR1AL = 0xFF;
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			servo_2_free
* Input:     				None
* Output:    				None
* Logic:     				function is used to unlock the servo motors from the any angle
*							and make them free by giving 100% duty cycle at the PWM. This function can be used to
*							reduce the power consumption of the motor if it is holding load against the gravity.
* Example Call:   			servo_2_free();
*/
void servo_2_free (void)
{
	OCR1BH = 0x03;
	OCR1BL = 0xFF; //Servo 2 off
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			servo_3_free
* Input:     				None
* Output:    				None
* Logic:     				function is used to unlock the servo motors from the any angle
*							and make them free by giving 100% duty cycle at the PWM. This function can be used to
*							reduce the power consumption of the motor if it is holding load against the gravity.
* Example Call:   			servo_3_free();
*/
void servo_3_free (void) //makes servo 3 free rotating
{
	OCR1CH = 0x03;
	OCR1CL = 0xFF; //Servo 3 off
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			gripper_close
* Input:     				None
* Output:    				None
* Logic:     				function is used to close the gripper by rotating servo_3 to 180 degrees
* Example Call:   			gripper_close();
*/
void gripper_close()
{
	servo_3(180);
	_delay_ms(500);
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			gripper_open
* Input:     				None
* Output:    				None
* Logic:     				function is used to open the gripper by rotating servo_3 to 0 degrees
* Example Call:   			gripper_open();
*/
void gripper_open()
{
	servo_3(0);
	_delay_ms(500);
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			fruit
* Input:     				p: node of fruit
* Output:    				None
* Logic:     				function is used to rotate servo_2 to desired angle according to fruit position
* Example Call:   			fruit(8);
*/
void fruit(int p)
{
	switch(p)
	{
		case 1:
			servo_2(115);
			break;
		case 2:
			servo_2(112);
			break;
		case 3:
			servo_2(109);
			break;
		case 4:
			servo_2(106);
			break;
		case 5:
			servo_2(103);
			break;
		case 6:
			servo_2(100);
			break;
		case 7:
			servo_2(97);
			break;
		case 8:
			servo_2(94);
			break;
		case 9:
			servo_2(91);
			break;
		case 10:
			servo_2(88);
			break;
		case 11:
			servo_2(85);
			break;
		case 12:
			servo_2(82);
			break;
	}
	
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			initial
* Input:     				None
* Output:    				None
* Logic:     				function is used to set initial or reset motor positions
* Example Call:   			initial();
*/
void initial()
{
	servo_1(104);
	_delay_ms(500);
	servo_2(180);
	_delay_ms(500);
	servo_3(180);	
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			drop
* Input:     				box: box number of fruit
* Output:    				None
* Logic:     				function is used to drop the fruit to desired box
* Example Call:   			drop(2);
*/
void drop(int box)
{
	if(box == 1)
	{
		servo_1(10);
		_delay_ms(2000);
	}
	else if(box == 2)
	{
		servo_1(100);
		_delay_ms(2000);
	}
	else if(box == 3)
	{
		servo_1(200);
		_delay_ms(2000);
	}
	else
	{
		servo_1(60);
		_delay_ms(2000);
	}
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			move_x2
* Input:     				None
* Output:    				None
* Logic:     				function is used to move bot without lack line following
* Example Call:   			move_x2();
*/
void move_x2()
{
	sensor_update();
	read_sensor();
	velocity(255, 255);
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			linear_distance_mm_x2
* Input:     				DistanceInMM: Distance to be moved
* Output:    				None
* Logic:     				function is used to move bot without lack line following to particular distance forward or backward
* Example Call:   			linear_distance_mm_x2(100);
*/
void linear_distance_mm_x2(unsigned int DistanceInMM)
{
	float ReqdShaft = 0;
	unsigned long int ReqdShaftCount = 0;
	ReqdShaft = DistanceInMM / 7.358;
	ReqdShaftCount = (unsigned int) ReqdShaft;
	ShaftCountRight = 0;
	ShaftCountLeft = 0;
	while (1)
	{
		move_x2();
		if (ShaftCountRight > ReqdShaftCount && ShaftCountLeft > ReqdShaftCount)
		{
			break;
		}
	}
	stop();
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			sleft
* Input:     				None
* Output:    				None
* Logic:     				function is used to move bot slow_left
* Example Call:   			sleft();
*/
void sleft()
{
	velocity(150,150);
	motion_set(0x05);
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			sright
* Input:     				None
* Output:    				None
* Logic:     				function is used to move bot slow_left
* Example Call:   			sright();
*/
void sright()
{
	velocity(150,150);
	motion_set(0x0A);
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			forward_mm_x2
* Input:     				DistanceInMM: Distance to be moved
* Output:    				None
* Logic:     				function is used to move bot without lack line following to particular distance forward
* Example Call:   			forward_mm_x2();
*/
void forward_mm_x2(unsigned int DistanceInMM)
{
	forward();
	linear_distance_mm_x2(DistanceInMM);
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			backward_mm_x2
* Input:     				DistanceInMM: Distance to be moved
* Output:    				None
* Logic:     				function is used to move bot without lack line following to particular distance backward
* Example Call:   			backward_mm_x2();
*/
void backward_mm_x2(unsigned int DistanceInMM)
{
	back();
	linear_distance_mm_x2(DistanceInMM);
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			pluck
* Input:     				point: point of the fruit
*							box: box number of fruit
* Output:    				None
* Logic:     				function is used to Pluck the fruit from the tree
* Example Call:   			pluck(1,2);
*/
void pluck(int point, int box)
{
	initial();
	gripper_open();
	_delay_ms(1000);
	fruit(point);
	_delay_ms(1000);
	forward_mm_x2(10);
	_delay_ms(1000);
	gripper_close();
	_delay_ms(1000);
	backward_mm_x2(20);
	_delay_ms(500);
	servo_2(150);
	_delay_ms(500);
	drop(box);
	_delay_ms(2000);
	gripper_open();
	_delay_ms(500);
	initial();
}
void pluck1(int point, int box)
{
	initial();
	//backward_mm_x2(15);
	gripper_open();
	
	fruit(point);
	_delay_ms(500);
	//forward_mm_x2(10);
	_delay_ms(1000);
	gripper_close();
	_delay_ms(1000);
	//backward_mm_x2(15);
	_delay_ms(500);
	servo_2(150);
	_delay_ms(500);
	drop(box);
	_delay_ms(1000);
	gripper_open();
	_delay_ms(500);
	initial();
	_delay_ms(500);
	//backward_mm_x2(2);
	//_delay_ms(500);
	
}

void camerapro()
{
	unsigned char i = 0;
	initial();
	_delay_ms(1000);
	servo_2(80);
	_delay_ms(1000);
	for (i = 80; i <115; i++)
	{
		servo_2(i);
		_delay_ms(10);
		
	}	
}

void dist()
{
	unsigned char sharp;
	unsigned int value,sub;
	
		sharp = ADC_Conversion(11);
		value = Sharp_GP2D12_estimation(sharp);
		if (value>268)
			{
				sub = value - 268;
				forward_mm(sub);
				pluck1(3,1);
				backward_mm_x2(sub);
				
			}
	
}

void dist1()
{
	unsigned char sharp , s1;
	unsigned int value,sub,v1;
	s1 = ADC_Conversion(11);
	v1 = Sharp_GP2D12_estimation(s1);
	while(1)
	{sharp = ADC_Conversion(11);
	value = Sharp_GP2D12_estimation(sharp);
	if (value>269)
	{
		
		forward_mm(2);
		
		
	}
	
		if (value<270)
		{	
			forward_mm(2);
		}
		break;
	}
}

void uart2_init(void)
{
	UCSR2B = 0x00; //disable while setting baud rate
	UCSR2A = 0x00;
	UCSR2C = 0x06;
	UBRR2L = 0x5F; //set baud rate lo
	UBRR2H = 0x00; //set baud rate hi
	UCSR2B = 0x98;
}


SIGNAL(SIG_USART2_RECV) 		// ISR for receive complete interrupt
{
	data = UDR2; 				//making copy of data from UDR2 in 'data' variable

	UDR2 = data; 				//echo data back to PC

	if(data == 0x38) //ASCII value of 8
	{
		PORTA=0x06;  //forward
	}

	if(data == 0x32) //ASCII value of 2
	{
		PORTA=0x09; //back
	}

	if(data == 0x34) //ASCII value of 4
	{
		PORTA=0x05;  //left
	}

	if(data == 0x36) //ASCII value of 6
	{
		PORTA=0x0A; //right
	}

	if(data == 0x35) //ASCII value of 5
	{
		PORTA=0x00; //stop
	}

	if(data == 0x37) //ASCII value of 7
	{
		buzzer_on();
	}

	if(data == 0x39) //ASCII value of 9
	{
		buzzer_off();
	}

}

