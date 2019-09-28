#include "Wire.h"
#include "MPU6050.h"
#include "math.h"

#define sampleTime  0.005

MPU6050 mpu;

int16_t accY, accZ, gyroX;
volatile int motorPower, gyroRate;
volatile float accAngle, gyroAngle, currentAngle, prevAngle = 0, error, prevError = 0, errorSum = 0;
float Speed, Kif=0, Kpf=0, Kdf=0, Ki=0, Kp=0, Kd=0,targetAngle=-32;

void setMotors(int MotorSpeed)
{
  if (MotorSpeed > 0)
  {
    Speed = MotorSpeed;
    Speed=Speed+50;
    if(Speed>255)
    {
    Speed=255;
    }

    //Speed = map(Speed, 0, 255, 255, 0);
    analogWrite(5, Speed);
    digitalWrite(6, LOW);
    analogWrite(3, Speed);
    digitalWrite(11, LOW);
    
    Serial.print(Kp,4);
    Serial.print(",");
    Serial.print(Ki,4);
    Serial.print(",");
    Serial.println(Kd,4);


//    Serial.print("Kp=");
//    Serial.print(Kp,4);
//    Serial.print(" Ki= ");
//    Serial.print(Ki,4);
//    Serial.print(" Kd= ");
//    Serial.print(Kd,4);
//    Serial.print(" Speed= ");
//    Serial.print(Speed);
//    Serial.print(" currentAngle= ");
//    Serial.print(currentAngle);
//    Serial.print(" targetAngle ");
//    Serial.print(targetAngle);
//    Serial.print("   Sensor Values: ");
//    Serial.print(accY);
//    Serial.print("  ");
//    Serial.print(accZ);
//    Serial.print("  ");
//    Serial.print(gyroX);    
//    Serial.println(" Forward");
  }
  else if (MotorSpeed < 0)
  {
    Speed = (MotorSpeed) + 255;
    Speed = map(Speed, 255,0,0,255);
    Speed=Speed+60;
    if(Speed>255)
    {
      Speed=255;
    }
    analogWrite(6,Speed);
    digitalWrite(5,LOW);
    analogWrite(11,Speed);
    digitalWrite(3,LOW);
    
    Serial.print(Kp,4);
    Serial.print(",");
    Serial.print(Ki,4);
    Serial.print(",");
    Serial.println(Kd,4);


//    Serial.print("Kp=");
//    Serial.print(Kp,4);
//    Serial.print(" Ki= ");
//    Serial.print(Ki,4);
//    Serial.print(" Kd= ");
//    Serial.print(Kd,4);
//    Serial.print(" Speed= ");
//    Serial.print(Speed);
//    Serial.print(" currentAngle= ");
//    Serial.print(currentAngle);
//    Serial.print(" targetAngle ");
//    Serial.print(targetAngle);
//    Serial.print("   Sensor Values: ");
//    Serial.print(accY);
//    Serial.print("  ");
//    Serial.print(accZ);
//    Serial.print("  ");
//    Serial.print(gyroX);
//    Serial.println(" Backward");    
}
  
  else 
  {
    analogWrite(6, LOW);
    digitalWrite(5, LOW);
    analogWrite(3, LOW);
    digitalWrite(11, LOW);

    Serial.print(Kp,4);
    Serial.print(",");
    Serial.print(Ki,4);
    Serial.print(",");
    Serial.println(Kd,4);


//    Serial.print("Kp=");
//    Serial.print(Kp,4);
//    Serial.print(" Ki= ");
//    Serial.print(Ki,4);
//    Serial.print(" Kd= ");
//    Serial.print(Kd,4);
//    Serial.print(" Speed= ");
//    Serial.print(Speed);
//    Serial.print(" currentAngle= ");
//    Serial.print(currentAngle);
//    Serial.print(" targetAngle ");
//    Serial.print(targetAngle);
//    Serial.print("   Sensor Values: ");
//    Serial.print(accY);
//    Serial.print("  ");
//    Serial.print(accZ);
//    Serial.print("  ");
//    Serial.print(gyroX);    
//    Serial.println("---------------------------Neutral----------------------------");  
  }
}

void init_PID()
{
  // initialize Timer1
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B
  // set compare match register to set sample time 5ms
  OCR1A = 9999;
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 bit for prescaling by 8
  TCCR1B |= (1 << CS11);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();          // enable global interrupts
}

void setup()
{
  Serial.begin(9600); 
  // set the motor control and PWM pins to output mode
  //pinMode(A0, INPUT);   
  //pinMode(A1, INPUT);
  //pinMode(A2, INPUT);
  
  //pinMode(5, OUTPUT);
  //pinMode(6, OUTPUT);
  //pinMode(3, OUTPUT);
  DDRD|=B01101000;
  //pinMode(11, OUTPUT);
  DDRB|=B00001000;

  //Serial.println("Hardware is configured. Sensor Data is as follows:");
  // initialize the MPU6050 and set offset values

  mpu.initialize();
  mpu.setYAccelOffset(8);
  mpu.setZAccelOffset(16374);
  mpu.setXGyroOffset(14);
  // initialize PID sampling loop
  init_PID();
}

void loop()
{
  // read acceleration and gyroscope values
  accY = mpu.getAccelerationY();
  accZ = mpu.getAccelerationZ();
  gyroX = mpu.getRotationX();

//  Serial.print("Sensor Values: ");
//  Serial.print(accY);
//  Serial.print("  ");
//  Serial.print(accZ);
//  Serial.print("  ");
//  Serial.println(gyroX);
 
  
  Kpf = analogRead(A0);
  Kif = analogRead(A1);
  Kdf = analogRead(A2);

//Kp=Kpf;
//Ki=Kif;
//Kd=Kdf;

//Serial.print(Kp);
//Serial.print(" ");
//Serial.print(Ki);
//Serial.print(" ");
//Serial.println(Kd);


  Kp = Kpf/100;
  Ki = Kif/1000;
  Kd = Kdf/1000;

//  Kp = map(Kpf, 0, 1023, 0, 25000) / 100; //0 - 250
//  Ki = map(Kif, 0, 1023, 0, 100000) / 100; //0 - 1000
//  Kd = map(Kdf, 0, 1023, 0, 1000) / 100; //0 - 5

  //targetAngle = map(targetAngle, 0, 1023, -10, -20);

  // set motor power after constraining it
  motorPower = constrain(motorPower, -255, 255);
  setMotors(motorPower);
}

// The ISR will be called every 5 milliseconds

ISR(TIMER1_COMPA_vect)
{
  // calculate the angle of inclination
  accAngle = atan2(accY, accZ) * RAD_TO_DEG;
  gyroRate = map(gyroX, -32768, 32767, -250, 250);
  gyroAngle = (float)gyroRate * sampleTime;
  currentAngle = 0.9934 * (prevAngle + gyroAngle) + 0.0066 * (accAngle);

  error = currentAngle - targetAngle;
  errorSum = errorSum + error;
  errorSum = constrain(errorSum, -300, 300);
  //calculate output from P, I and D values
  motorPower = Kp * (error) + Ki * (errorSum) * sampleTime - Kd * (currentAngle - prevAngle) / sampleTime;
  prevAngle = currentAngle;
}

