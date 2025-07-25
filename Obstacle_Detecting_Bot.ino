#include <LiquidCrystal.h>
int mSpeed=3;
int dir1=2;
int dir2=4;
int trigPin=5;
int echoPin=6;

LiquidCrystal lcd(7,8,9,10,11,12);

float time;
float speedOfSound=343;
float distance;

float milliOld;
float milliNew;
float dt;
float area;

float mSpeedVal=255;
float mSpeed_actual;

float error=0;
float errorOld;
float dError;
float slope;

float Kp=0.1;
float Kd=10;
float Ki=0.00001;

void setup()
{
  Serial.begin(9600);
  lcd.begin(16,2);
  pinMode(echoPin, INPUT); 
  pinMode(mSpeed, OUTPUT);
  pinMode(dir1, OUTPUT);
  pinMode(dir2, OUTPUT);
  pinMode(trigPin, OUTPUT);
  milliNew=millis();
}

void loop()
{
  lcd.setCursor(0,0);
  lcd.print("OBJECT DISTANCE IS");
  float mSpeed_target;
  digitalWrite(trigPin,LOW);
  delayMicroseconds(10);
  digitalWrite(trigPin,HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin,LOW);
  time=pulseIn(echoPin,HIGH);
  delay(25);
  distance=speedOfSound*time/20000;
  Serial.print("DISTANCE OF OBJECT IS:");
  Serial.println(distance);

  lcd.setCursor(0,1);
  lcd.print(distance);
  lcd.print(" IN CM");
  delay(500);
  lcd.clear();
  
  analogWrite(mSpeed,mSpeedVal);
  digitalWrite(dir1,HIGH);
  digitalWrite(dir2,LOW);
  
  milliOld=milliNew;
  milliNew=millis();
  dt=milliNew - milliOld;
  Serial.print("CHANGE IN TIME IS:");
  Serial.println(dt);
  
  
  if(distance<=100){
    lcd.setCursor(0,0);
    lcd.print("OBJECT DETECTED");
    
    mSpeed_target=0;
    
    errorOld=error;
    error=mSpeed_target - mSpeed_actual;
    dError=error-errorOld;
  
    slope=dError/dt;
    area=area+(dError*dt);
    
    mSpeedVal=mSpeedVal+Kp*error+Kd*slope+Ki*area;
    
    if(mSpeedVal<0){
      mSpeedVal=0;
    }
    
    analogWrite(mSpeed,mSpeedVal);
    digitalWrite(dir1,HIGH);
    digitalWrite(dir2,LOW);
    mSpeed_actual=mSpeedVal;
    Serial.print("SPEED OF MOTOR IS:");
    Serial.println(mSpeedVal);
    Serial.println();
    
    lcd.setCursor(0,1);
    lcd.print(mSpeedVal);
    lcd.print(" rpm");
    
    delay(100);
  }else{
    
    lcd.setCursor(0,0);
    lcd.print("OBJECT UNIDENTIFIED");
    
    mSpeed_target=255;
    errorOld=error;
    error=mSpeed_target - mSpeed_actual;
    dError=error-errorOld;
  
    slope=dError/dt;
    area=area+(dError*dt);
    
    mSpeedVal=mSpeedVal+Kp*error+Kd*slope+Ki*area;
    
    if(mSpeedVal>255){
      mSpeedVal=255;
    }
    
    analogWrite(mSpeed,mSpeedVal);
    mSpeed_actual=mSpeedVal;
    digitalWrite(dir1,HIGH);
    digitalWrite(dir2,LOW);
    Serial.print("SPEED OF MOTOR IS:");
    Serial.println(mSpeedVal);
    Serial.println();
    
    lcd.setCursor(0,1);
    lcd.print("MOTOR SPEED ");
    lcd.print(mSpeedVal);
    lcd.print("rpm");
    delay(100);
  }
}
