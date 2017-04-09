#include<Servo.h>
#include<SoftwareSerial.h>

Servo m1,m2,m3,m4;
SoftwareSerial BT(4,5);
long throttle;
int i=0;
String a;
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
 BT.begin(115200);
	m1.attach(6);
	m2.attach(9);
	m3.attach(10);
	m4.attach(11);
	initialize();
	i=0;
}

void refresh()
{
//      if(throttle<30)
//      {
//	m1.write(throttle);
//	m2.write(throttle);
//      }
//      else
//      {
//      m1.write(30+150*((throttle-30)/120));
//      m2.write(30+150*((throttle-30)/120));
//      }	
  m1.write(throttle);
  m2.write(throttle);
  m3.write(throttle+12);
	m4.write(throttle+12);
}


void loop()
{
	if(BT.available())
	{
		x=BT.read();
//    Serial.print("Bluetooth read data : ");
    Serial.print(x);
		if(x>=11)
		{
			throttle=x-11;
			throttle=(throttle*180)/100;
//                 Serial.print(" Setting thrust to : ");
//                 Serial.print(throttle);
			refresh();
		}
		else if(x>=2 && x<=10)
		{
//			Serial.print(" Settting direction to : ");
//      switch(x)
//      {
//        case 2:a="center";break;
//        case 3:a="Up";break;
//        case 4:a="Up right";break;
//        case 5:a="Right";break;
//        case 6:a="Down right";break;
//        case 7:a="Down";break;
//        case 8:a="Down left";break;
//        case 9:a="Left";break;
//        case 10:a="Up left";break;
//        }
//        Serial.print(a);
		}
		else if(x==0 || x==1)
		{
//    if(x==0)
//			Serial.print(" GPS LOCK disabled");
//     else
//      Serial.print(" GPS LOCK enabled");
		}
   Serial.println();
	}
}
