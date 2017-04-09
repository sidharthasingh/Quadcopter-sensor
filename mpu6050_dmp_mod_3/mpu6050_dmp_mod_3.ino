#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include<Servo.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	#include "Wire.h"
#endif
#define MAX_ADDON_VALUE 30
#define MIN_ADDON_VALUE 0
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

Servo m1,m2,m3,m4;
int thrust=0;
int iii=0;
byte m1_v=0,m2_v=0,m3_v=0,m4_v=0;
int A=0,B=0;
int temp;
bool blinkState = false;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
	mpuInterrupt = true;
}

void normalize()
{
	if(m1_v<MIN_ADDON_VALUE)
		m1_v=MIN_ADDON_VALUE;
	else if(m1_v>MAX_ADDON_VALUE)
		m1_v=MAX_ADDON_VALUE;
	if(m2_v<MIN_ADDON_VALUE)
		m2_v=MIN_ADDON_VALUE;
	else if(m2_v>MAX_ADDON_VALUE)
		m2_v=MAX_ADDON_VALUE;
	if(m3_v<MIN_ADDON_VALUE)
		m3_v=MIN_ADDON_VALUE;
	else if(m3_v>MAX_ADDON_VALUE)
		m3_v=MAX_ADDON_VALUE;
	if(m4_v<MIN_ADDON_VALUE)
		m4_v=MIN_ADDON_VALUE;
	else if(m4_v>MAX_ADDON_VALUE)
		m4_v=MAX_ADDON_VALUE;
	if(thrust+m1_v>=150)
		m1_v=150-thrust;
	if(thrust+m2_v>=150)
		m2_v=150-thrust;
}

void refresh()
{
	m1.write(thrust+m1_v);
	m2.write(thrust+m2_v);
	m3.write(thrust+m3_v);
	m4.write(thrust+m4_v);
}

void getBTvalue()
{
	if(Serial.available())
	{
		temp=Serial.read();
		if(temp>=11)
		{
			thrust=((temp-11)*180)/100;
		}
		else if(temp>=2 && temp<=10)
		{
			//code for setting direction
		}
		else if(temp==0 || temp==1)
		{
			//code for gps lock or 'hovering'
		}
		Serial.println();
		Serial.println(temp);
	}
}

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

	Serial.print("MOTOR IS READY! \n");
}


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
	// join I2C bus (I2Cdev library doesn't do this automatically)
	#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
		Wire.begin();
	TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
	#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
		Fastwire::setup(400, true);
	#endif
	
	// initialize serial communication
	Serial.begin(9600);

	m1.attach(6);
	m2.attach(9);
	m3.attach(10);
	m4.attach(11);
	
	initialize();
	
	while (!Serial); // wait for Leonardo enumeration, others continue immediately
	
	// initialize device
	Serial.println(F("Initializing I2C devices..."));
	mpu.initialize();
	
	// verify connection
	Serial.println(F("Testing device connections..."));
	Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
	
	// load and configure the DMP
	Serial.println(F("Initializing DMP..."));
	devStatus = mpu.dmpInitialize();
	
	// supply your own gyro offsets here, scaled for min sensitivity
	mpu.setXGyroOffset(220);
	mpu.setYGyroOffset(76);
	mpu.setZGyroOffset(-85);
	mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
	
	// make sure it worked (returns 0 if so)
	if (devStatus == 0) {
		// turn on the DMP, now that it's ready
		Serial.println(F("Enabling DMP..."));
		mpu.setDMPEnabled(true);
		
		// enable Arduino interrupt detection
		Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
		attachInterrupt(0, dmpDataReady, RISING);
		mpuIntStatus = mpu.getIntStatus();
		
		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		Serial.println(F("DMP ready! Waiting for first interrupt..."));
			dmpReady = true;
		
		// get expected DMP packet size for later comparison
		packetSize = mpu.dmpGetFIFOPacketSize();
	} else {
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		Serial.print(F("DMP Initialization failed (code "));
		Serial.print(devStatus);
		Serial.println(F(")"));
	}
}

void loop()
{
	getBTvalue();

	// if programming failed, don't try to do anything
	if (!dmpReady) return;
	
	// wait for MPU interrupt or extra packet(s) available
	while (!mpuInterrupt && fifoCount < packetSize)
	{
		//Other codes here.
	}
	
	// reset interrupt flag and get INT_STATUS byte
	mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus();
	
	// get current FIFO count
	fifoCount = mpu.getFIFOCount();
	
	// check for overflow (this should never happen unless our code is too inefficient)
	if ((mpuIntStatus & 0x10) || fifoCount == 1024) 
	{
		// reset so we can continue cleanly
		mpu.resetFIFO();
		Serial.println(F("FIFO overflow!"));
		
		// otherwise, check for DMP data ready interrupt (this should happen frequently)
	} 

	else if (mpuIntStatus & 0x02) 
	{
		// wait for correct available data length, should be a VERY short wait
		while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
		
		// read a packet from FIFO
		mpu.getFIFOBytes(fifoBuffer, packetSize);
		
		fifoCount -= packetSize;
		
		// display Euler angles in degrees
		while(iii<1400)
		{
			mpu.dmpGetQuaternion(&q, fifoBuffer);
			mpu.dmpGetGravity(&gravity, &q);
			mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
			iii++;
			Serial.println(iii);
			delay(10);
		}
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
		A=ypr[1]*180/M_PI-4;
		B=ypr[2]*180/M_PI;
		Serial.print("ypr\t");
		Serial.print(A);
		Serial.print("\t");
		Serial.println(B);
		Serial.print("\t");
		if(B>1)
		{
			m2_v=A/2;
			m4_v=(-1*A)/2;
		}
		else if(A<-1)
		{
			m2_v=(-1*A)/2;
			m4_v=A/2;
		}
		if(A>1)
		{
			m1_v=B/2;;
			m3_v=(-1*B)/2;
		}
		else if(B<-1)
		{
			m1_v=(-1*B)/2;
			m3_v=B/2;
		}
		normalize();
		refresh();
	}
}
