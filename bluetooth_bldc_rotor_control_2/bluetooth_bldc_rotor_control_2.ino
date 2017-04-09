#include<Servo.h>

Servo m1,m2,m3,m4;
long throttle;
int i=0;
unsigned char c,d,x;

void initialize()
{
	
	Serial.print("Arming the motor! \n");
	delay(3000);
	
	Serial.print("Setting Low speed! \n");
	m1.write(20);
	m2.write(20);
	m3.write(20);
	m4.write(20);
	delay(4000);
	
	Serial.print("Setting High speed! \n");
	//m1.write(179);
	//m2.write(179);
	//m3.write(179);
	//m4.write(179);
	delay(4000);
	
	//    m1.write(20);
	//    m1.write(20);
	Serial.print("MOTOR IS READY! \n");
	
}

void setup()
{
	Serial.begin(9600);
	m1.attach(6);
	m2.attach(9);
	m3.attach(10);
	m4.attach(11);
	initialize();
	i=0;
}

void refresh()
{
	m1.write(throttle);
	m2.write(throttle);
	m3.write(throttle);
	m4.write(throttle);
}

void plint(unsigned char a,unsigned char b)
{
	Serial.print((int)a);
	Serial.print(" ");
	Serial.println((int)b);
}

void loop()
{
	if(Serial.available())
	{
		x=Serial.read();
		if(x>=11)
		{
			throttle=x-11;
			throttle=(throttle*180)/100;
			refresh();
		}
		else if(x>=2 && x<=10)
		{
			
		}
		else if(x==0 || x==1)
		{
			
		}
		Serial.println(x);
	}
}
