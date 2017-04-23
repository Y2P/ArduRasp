
#include <Event.h>
#include <Timer.h>

#include <Arduino.h>
Timer timer2;
Timer sensor;
Timer yayInfo; // This timer is defined for string information request, PERIODICALLY


String inputString = "";         // a string to hold incoming data
char buf[20];                      // Buffer for incoming data
boolean stringComplete = false;  // whether the string is complete
float measured_val= 0; // Global Variable for passing incoming value
float lastval = 0;
float unitSpeedUpdate_Dec = 1.05; // Unit speed change 
float unitSpeedUpdate_Acc = 1.1; // Unit speed change 
float SpringFreeAcc = 1.1;
float unitSpeedUpdate_Acc_L1 = 0.05;
float unitSpeedUpdate_Acc_L2 = 0.04;
float unitSpeedUpdate_Dec_L2 = 0.08;
float unitSpeedUpdate_Dec_L1 = 0.1;
float range = 6;

float maxscale = 0.9;      // 



const byte trigPin1 = 4;
const byte echoPin1 = 3;

const byte trigPin2 = 13;
const byte echoPin2 = 12;

const byte trigPin3 = 8;
const byte echoPin3 = 9;

const byte Mf1 = 5;
const byte Mr1 = 6;

const byte Mf2 = 11;
const byte Mr2 = 10;

const byte DirSel = 7;
const byte yay = 2;


float KP1 =10;
float KD1 =0;
float KP2 =10;
float KD2 =0;
float followdist = 10;
float midspeed = 90;
float sum_par,sum_diff;
float delta_sense3, delta_diff, delta_sense1;
float old_sense3, old_diff, old_sense1;
float leftspeed,rightspeed;
float scaler2 = 0.4;
float scaler1 = 0.5;
float delta_speed;
float delta_speed2;

long duration1;
float distance1;
long duration2;
float distance2;
long duration3;
float distance3;
float sense1=0;
float sense2=0;
float sense3=0;
float alpha = 0.05;
boolean toggle0 = 0;
int dir = 0;

void yayRequest()
{
  Serial.print(digitalRead(yay),BIN);
  Serial.print(" ");
  Serial.print(measured_val,DEC);
  Serial.print(" ");
  Serial.print(scaler1,DEC);
  Serial.print(" ");
  Serial.println(scaler2,DEC);

  delay(10);
  inputString = "";
}


void setup() {
  // put your setup code here, to run once:
  pinMode(trigPin1, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin3, INPUT);
  pinMode(Mf1, OUTPUT);
  pinMode(Mr1, OUTPUT);
  pinMode(Mf2, OUTPUT);
  pinMode(Mr2, OUTPUT);
  pinMode(DirSel, INPUT_PULLUP);
  pinMode(2, INPUT_PULLUP);
//  attachInterrupt(yay_interrupt, yayRequest, FALLING);

  Serial.begin(9600);
 // timer2.every(1000,send_data);
  sensor.every(5,measure);
  yayInfo.every(500,yayRequest);
}

void loop() {
  // put your main code here, to run repeatedly:
// Spring polling is done here
dir = digitalRead(DirSel);
if(measured_val > 15)
{
  if( digitalRead(yay) == 0) 
  {
  
      // Basic Prototype Decision making implemented here
        if((lastval - measured_val) < -range)
        {
          if(maxscale > scaler1 && maxscale > scaler2)
          {
            // Linear Update 
            scaler1 = scaler1 + unitSpeedUpdate_Acc_L1;
            scaler2 = scaler2 + unitSpeedUpdate_Acc_L2;
            // Muliplier update
            //scaler1 = scaler1 * unitSpeedUpdate_Acc;
           // scaler2 = scaler2 * unitSpeedUpdate_Acc;
          }
        }
        else if((lastval - measured_val) > range)
        {
          if(0.1 < scaler1 && 0.1 < scaler2)
          {
            // Linear Update 
            scaler1 = scaler1 - unitSpeedUpdate_Dec_L1;
            scaler2 = scaler2 - unitSpeedUpdate_Dec_L2;
             // Muliplier update
            //scaler1 = scaler1 / unitSpeedUpdate_Dec;
            // scaler2 = scaler2 / unitSpeedUpdate_Dec;
          }
        }
      }
    // Update Scale parameter according to the last distance information  
    
    
    else if( digitalRead(yay) == 1) 
    {
      // When spring is free
      if(maxscale > scaler1 && maxscale > scaler2)
          {
            // Linear update
            scaler1 = scaler1 + unitSpeedUpdate_Acc_L1;
            scaler2 = scaler2 + unitSpeedUpdate_Acc_L2;
            // Multiplier update
           // scaler1 = scaler1 * SpringFreeAcc;
         //   scaler2 = scaler2 * SpringFreeAcc;
          }
    }
}
else 
{
  scaler1 = 0.25;
  scaler2 = 0.2;
}

sensor.update();
yayInfo.update();
//timer2.update();
delta_sense1 = sense1 - old_sense1;
old_sense1 = sense1;
delta_sense3 = sense3 - old_sense3;
old_sense3 = sense3;
delta_speed = KP1*(sense1 - followdist) + KD1*(delta_sense1);
delta_speed2 = KP2*(sense3 - followdist) + KD2*(delta_sense3);
if (delta_speed > midspeed)  delta_speed = midspeed;
if (delta_speed < -midspeed)  delta_speed = -midspeed;
if (delta_speed2 > midspeed)  delta_speed2 = midspeed;
if (delta_speed2 < -midspeed)  delta_speed2 = -midspeed;
if (dir)
{
leftspeed = scaler2*( midspeed - delta_speed2 );
rightspeed = scaler2*(midspeed + delta_speed2 );
 }
else
{
  leftspeed = scaler1*( midspeed + delta_speed );
  rightspeed = scaler1*(midspeed - delta_speed );
  }
Motordrive(dir,leftspeed,rightspeed);

}
void measure() { //Sensor Measurement
  // Clears the trigPin
  if (dir)
  {
  digitalWrite(trigPin3, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin3, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin3, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration3 = pulseIn(echoPin3, HIGH);
  
  // Calculating the distance
  if (duration3 < (40 / 0.034)) //limit the data
  {
    distance3 = duration3 * 0.034 / 2;
  }

  // LPF
  sense3 = (1-alpha)*sense3+alpha*distance3;
  
  }
else
{
digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration1 = pulseIn(echoPin1, HIGH);
  
  // Calculating the distance
  if (duration1 < (40 / 0.034)) //limit the data
  {
    distance1 = duration1 * 0.034 / 2;
  }

  // LPF
  sense1 = (1-alpha)*sense1+alpha*distance1;
}
  delay(1);
}
void send_data()  {
  Serial.print("Sensor1 >   ");
  Serial.println(sense1);
    Serial.print("Sensor2 >  ");
  Serial.println(sense2);
    Serial.print("Sensor3 >  ");
  Serial.println(sense3);
   
  }

void Motordrive(int dir2, float velocity_L, float velocity_R){
 if (velocity_L <20)velocity_L=20;
 if (velocity_R <20)velocity_R=20;
 if (velocity_L >255)velocity_L=255;
 if (velocity_R >255)velocity_R=255;
// MOTORLAR BURADA SUSTURULDU

 if (dir2) { //not yapılacak ipi kontrol ediyor
  analogWrite(Mr1,velocity_L);
  analogWrite(Mr2,velocity_R*0.97);
  analogWrite(Mf1,0);
  analogWrite(Mf2,0);
 }
else {
  analogWrite(Mf1,velocity_R);
  analogWrite(Mf2,velocity_L*0.97);
  analogWrite(Mr1,0);
  analogWrite(Mr2,0);
  }
}
/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true; 
      break;
    }
  }
    lastval = measured_val;
//  Serial.println(inputString);
    inputString.toCharArray(buf, 20);
    measured_val = atof(buf);

}



