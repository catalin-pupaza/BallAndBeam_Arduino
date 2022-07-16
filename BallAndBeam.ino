
#include <Wire.h>
#include <Servo.h>

///////////////////////Inputs/outputs///////////////////////
const int trigPinSensor = 2;
const int echoPinSensor = 3;

Servo myservo;  // create servo object to control a servo, later attatched to D9
///////////////////////////////////////////////////////


////////////////////////Variables///////////////////////
int Read = 0;
float distance = 0.0;
float elapsedTime, time, timePrev;        //Variables for time control
float distance_previous_error, distance_error;
int period = 50;  //Refresh rate period of the loop is 50ms
///////////////////////////////////////////////////////


///////////////////PID constants///////////////////////
float kp=20; //-------------------------------------------------------------------Mine was 8
float ki=0.2; //------------------------------------------------------------------Mine was 0.2
float kd=1000; //------------------------------------------------------------------Mine was 3100
float distance_setpoint = 10;           //Should be the distance from sensor to the middle of the bar in cm
float PID_p, PID_i, PID_d, PID_total;
///////////////////////////////////////////////////////



void setup() {
  //analogReference(EXTERNAL);
  Serial.begin(9600);  
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  myservo.write(125); //Put the servo at angle 125, in balance

  pinMode(trigPinSensor, OUTPUT);
  pinMode(echoPinSensor, INPUT);
 
  time = millis();
}
///////////////////////////////////////////////////////


void loop() {
  if (millis() > time+period)
  {
    time = millis();    
    distance = get_dist();   
    distance_error = distance_setpoint - distance;   
    PID_p = kp * distance_error;
    float dist_diference = distance_error - distance_previous_error;     
    PID_d = kd*((distance_error - distance_previous_error)/period);
      
    if(-5 < distance_error && distance_error < 5)
    {
      PID_i = PID_i + (ki * distance_error);
    }
    else
    {
      PID_i = 0;
    }
  
    PID_total = PID_p + PID_i + PID_d;  
    PID_total = map(PID_total, -150, 150, 0, 150);
  
    //if(PID_total < 20){PID_total = 20;}
    //if(PID_total > 150) {PID_total = 150; } 
    PID_total = PID_total+30;
    
    myservo.write(PID_total);  
    distance_previous_error = distance_error;
  }
}




float get_dist()
{
  float durationSensor, distanceSensor;
  
  digitalWrite(trigPinSensor, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinSensor, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinSensor, LOW);

  durationSensor = pulseIn(echoPinSensor, HIGH);
  distanceSensor = (durationSensor*.0343)/2;
  Serial.print("DistanceSensor: ");
  if(distanceSensor >34){ distanceSensor = 34;}
      
  Serial.println(distanceSensor);
  return(distanceSensor);
}
