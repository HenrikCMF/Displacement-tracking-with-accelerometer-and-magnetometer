/*
 Project kill mee
 Team 3
 */ 

 
 
#define F_CPU 16000000UL
#include "Compasslib.h" //compass library contains a shit ton of other libraries FYI
#define ADC_PIN 0
#define timeinterval 0.025 //regulatable timeinterval
#define DACCRES 16384 //16384 4096 - division factor for digital accelerometer

long acceleration (void);

volatile float acc, vel, dis, acc1, pitch, roll;
volatile float heading, disx, disy;
float toimer;
int main (void)
{
	
	DDRC=0x00; //make all PINC inputs
	DDRD=0x40; //make all PIND inputs except for pin 6, which is vibration motor
	PORTD = 0xBF; // Enable internal pull at pins except pin 6
	i2c_init(); //initialize I2C communication
	//LCD_init(); //initialize the LCD
	
	
	compass2_init(); //the initialization function for the compass
	
	
	uart_init();
	io_redirect();
	MMA8451_init_1();
	//normval normacc;
	float xcord, ycord, zcord;
	TCCR1B|=(1<<WGM12); //Timer mode CTC
	TIMSK1 |= (1 << OCIE1A); //enable interrupts on overflow of timer 1
	OCR1A=(16000000/1024)*timeinterval-1; // count to timeinterval- for more accuracy at higher frequencies, 
	//division factor should be adjusted
	get_data_accel_1(&xcord, &ycord, &zcord); //get acceleration values
	_delay_ms(500); //init time for accel
	get_data_accel_1(&xcord, &ycord, &zcord); //get acceleration values
	xcord=((xcord)/DACCRES);
	ycord=((ycord)/DACCRES);
	zcord=((zcord)/DACCRES);
	acc1=sqrt(xcord*xcord+ycord*ycord+zcord*zcord); //get the magnitude of gravity vector
	sei();
	TCCR1B|=(1<<CS10)|(1<<CS12); //prescaler 1024 & start timer
	
	get_data_accel_1(&xcord, &ycord, &zcord); //get acceleration values
	printf("acc1=%f\n", acc1);
	while(1)
	{
		_delay_ms(300); //suitably long delay for there to be enough time for the interrupt to occur 
		//frequently enough to give accurate readings
		heading=compasshead(pitch, roll); //compass function- it requires that pitch and roll have been measured, or at least defined as 0
		printf("distance=%f\n", dis); //print current measured distance
		printf("displacement=%f\n", sqrt(disx*disx+disy*disy));//print current measured displacement
		

		
	}
}
		
	


ISR (TIMER1_COMPA_vect) //interrupt triggers every 0.025sec
{
	static float a0=0, v0=0, dis0=0; 
	float acc2, accchange;
	float accxnorm, accynorm, accmag;
	static int count=0;
	float xcord, ycord, zcord;
	get_data_accel_1(&xcord, &ycord, &zcord); //get acceleration values
	
	xcord=((xcord)/DACCRES); //convert values to unit g
	ycord=((ycord)/DACCRES);
	zcord=((zcord)/DACCRES);
	
	if(sqrt(xcord*xcord+ycord*ycord+zcord*zcord)<1.1) //if acceleration is less than slightly above 1G, save measurement as pitch and roll
	{
		accmag=sqrt(xcord*xcord+ycord*ycord+zcord*zcord);
		accxnorm=xcord/accmag;
		accynorm=ycord/accmag;
		pitch=asin(accxnorm);
		roll=-asin(accynorm/cos(pitch))+2*M_PI;
	}
	accchange=sqrt(xcord*xcord+ycord*ycord+zcord*zcord)-acc1; //find change in acceleration
	if (accchange<0.02) //if acceleration change is less than 0.02, assume noise
	{
		accchange=0;	
	}
	if((accchange)<0.005) //if there is no acceleration for 50*0.025 Sec
	{
		count++;
		if(count>50)
		{
				accchange=0; //assume person is standing still
				a0=0;
				v0=0;
				printf("Resetted\n");
				count=0;
		}
	
	}

	
	acc=a0+(accchange)*9.8; //convert from g to m/s^2
	

	vel=acc*timeinterval+v0; //get linear velocity

	
	v0=vel;
	
	dis=vel*timeinterval+dis0;
	
	disx=vel*timeinterval*cos(heading*M_PI/180)+disx; //sum displacement along north/south axis
	disy=vel*timeinterval*sin(heading*M_PI/180)+disy; //sum displacement along east/west axis
	
	dis0=dis;
}

