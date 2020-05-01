/*
 * Created by Vasilakis Michalis // 12-12-2014 ver.1
 * Project: Control RC Car via Bluetooth with Android Smartphone
 * http://www.ardumotive.com/arduino-car
 * More information at www.ardumotive.com
 */

 
#include <SoftwareSerial.h>

// arduino>>bluetooth
// D11 (as RX)   >>>  Tx
// D12 (as TX)  >>>  Rx
SoftwareSerial bluetooth(12, 11); // RX, TX

//L293 Connection   
  const int motorAEN  = 2;  // Enable 1 of 298
  const int motorA1  = 3;  // Pin  2 of L293
  const int motorA2  = 4;  // Pin  7 of L293
  const int motorBEN  = 5;  // Enable 1 of 298
  const int motorB1  = 7; // Pin 10 of L293
  const int motorB2  = 6;  // Pin 14 of L293
  const int TiltValue = 500; // Ohm value of tilt
  const int Tolerance = 50;
  
//Leds connected to Arduino UNO Pin 12
  const int frontlight  = 9;
  const int backlight  = 13;
//Buzzer / Speaker to Arduino UNO Pin 3
  const int buzzer = 8 ;   
//Bluetooth (HC-06 JY-MCU) State pin on pin 2 of Arduino
  const int BTState = 10;
//Leds connected to Arduino UNO Pin 12
  const boolean islog  = true;
//Motor direction pins
  int sensorRightPin = 0;
  int sensorLeftPin = 1;
  int sensorRightValue; 
  int sensorLeftValue; 
  boolean motorARightTilt = false;
  boolean motorALeftTilt = false;
  
//Calculate Battery Level
  const float maxBattery = 8.0;// Change value to your max battery voltage level! 
  int perVolt;                 // Percentage variable 
  float voltage = 0.0;         // Read battery voltage
  int level;
// Use it to make a delay... without delay() function!
  long previousMillis = -1000*10;// -1000*10=-10sec. to read the first value. If you use 0 then you will take the first value after 10sec.  
  long interval = 1000*10;       // interval at which to read battery voltage, change it if you want! (10*1000=10sec)
  unsigned long currentMillis;   //unsigned long currentMillis;
//Useful Variables
  char state;
  int vSpeed=0;     // Default speed, from 0 to 255

void setup() {
    // Set pins as outputs:
    pinMode(motorA1, OUTPUT);
    pinMode(motorA2, OUTPUT);
    pinMode(motorB1, OUTPUT);
    pinMode(motorB2, OUTPUT);
    pinMode(frontlight, OUTPUT); 
    pinMode(backlight, OUTPUT); 
    pinMode(BTState, INPUT);    
    // Initialize serial communication at 38400 bits per second:
    Serial.begin(38400); 
    bluetooth.begin(38400);

    //init_motor;
          analogWrite(motorAEN, vSpeed); analogWrite(motorBEN, vSpeed);
    Serial.println("Sistem Initialised");
}

void loop() {
  //Stop car when connection lost or bluetooth disconnected
     if(digitalRead(BTState)==LOW) { 
      //Serial.println("No signal"); 
      //state='S'; 
      }

  //Save income data to variable 'state'
    if(bluetooth.available() > 0){     
      state = bluetooth.read();
      //Serial.print(state);   
    }
    sensorRightValue = analogRead(sensorRightPin);
    sensorLeftValue = analogRead(sensorLeftPin);
    //Serial.print(sensorRightValue); Serial.print("-");Serial.println(sensorLeftValue);    //Első balra, max 500-ig(250 - 250 közép)
  

  switch (state) {
    case 'S':
      if (islog) {
        //Serial.println("Stop the car");
      }
      //vSpeed = 0;
      analogWrite(motorAEN, vSpeed); analogWrite(motorBEN, vSpeed);
      digitalWrite(motorA1, LOW); digitalWrite(motorA2, LOW);
      digitalWrite(motorB1, LOW); digitalWrite(motorB2, LOW);
      break;
    case 'F':
      if (islog) {
        Serial.println("Go forward");
      }
            vSpeed = 0;
      analogWrite(motorAEN, vSpeed); analogWrite(motorBEN, vSpeed);
      digitalWrite(motorA1, HIGH); digitalWrite(motorA2, LOW);
      digitalWrite(motorB1, LOW); digitalWrite(motorB2, LOW); 
      break;
    case 'G':
      if (islog) {
        Serial.println("Go forward left");
      }
      digitalWrite(motorA1, HIGH); digitalWrite(motorA2, LOW);
      digitalWrite(motorB1, HIGH); digitalWrite(motorB2, LOW);    
      break;
    case 'I':
      if (islog) {
        Serial.println("Go forward right");
      }
      digitalWrite(motorA1, HIGH); digitalWrite(motorA2, LOW);
      digitalWrite(motorB1, LOW); digitalWrite(motorB2, HIGH);    
      break;
    case 'B':
      if (islog) {
        Serial.println("Go backward");
      }
                  vSpeed = 255;
      analogWrite(motorAEN, vSpeed); analogWrite(motorBEN, vSpeed);
      digitalWrite(motorA1, LOW); digitalWrite(motorA2, HIGH);
      digitalWrite(motorB1, LOW); digitalWrite(motorB2, LOW);    
      break;
    case 'H':
      if (islog) {
        Serial.println("Go backward left");
      }
      digitalWrite(motorA1, LOW); digitalWrite(motorA2, HIGH);
      digitalWrite(motorB1, HIGH); digitalWrite(motorB2, LOW);    
      break;
    case 'J':
      if (islog) {
        Serial.println("Go backward right");
      }
      digitalWrite(motorA1, LOW); digitalWrite(motorA2, HIGH);
      digitalWrite(motorB1, LOW); digitalWrite(motorB2, HIGH);    
      break;
    case 'L':
      if (islog) {
        Serial.println("Go turn left");
      }
      if (motorALeftTilt) {
        digitalWrite(motorA1, LOW); digitalWrite(motorA2, LOW);
      }
      digitalWrite(motorB1, HIGH); digitalWrite(motorB2, LOW);    
      break;
    case 'R':
      if (islog) {
        Serial.println("Go turn right");
      }
      digitalWrite(motorA1, LOW); digitalWrite(motorA2, LOW);
      digitalWrite(motorB1, LOW); digitalWrite(motorB2, HIGH);       
      break;
    case 'W':
      if (islog) {
        Serial.println("Turn front leds on");
      }
      digitalWrite(frontlight, HIGH); 
      break;
    case 'w':
      if (islog) {
        Serial.println("Turn front leds off");
      }
      digitalWrite(frontlight, LOW); 
      break;
    case 'U':
      if (islog) {
        Serial.println("Turn back leds on");
      }
      digitalWrite(backlight, HIGH); 
      break;
    case 'u':
      if (islog) {
        Serial.println("Turn back leds off");
      }
      digitalWrite(backlight, LOW); 
      break;
    case 'V':
      if (islog) {
        Serial.println("Turn horn on");
      }
      tone(buzzer, 1000);
      break;
    case 'v':
      if (islog) {
        Serial.println("Turn horn off");
      }
      noTone(buzzer); 
      break;
    case '0':
      vSpeed=0;
      analogWrite(motorAEN, vSpeed); analogWrite(motorBEN, vSpeed);
      if (islog) {
        Serial.print("Speed to "); Serial.println(vSpeed);
      }
      break;
    case '1':
      vSpeed=150;
      analogWrite(motorAEN, vSpeed); analogWrite(motorBEN, vSpeed);
      if (islog) {
        Serial.print("Speed to "); Serial.println(vSpeed);
      }
      break;
    case '2':
      vSpeed=170;
      analogWrite(motorAEN, vSpeed); analogWrite(motorBEN, vSpeed);
      if (islog) {
        Serial.print("Speed to "); Serial.println(vSpeed);
      }
      break;
    case '3':
      vSpeed=190;
      analogWrite(motorAEN, vSpeed); analogWrite(motorBEN, vSpeed);    
      if (islog) {
        Serial.print("Speed to "); Serial.println(vSpeed);
      }
      break;
    case '4':
      vSpeed=210;
      analogWrite(motorAEN, vSpeed); analogWrite(motorBEN, vSpeed);    
      if (islog) {
        Serial.print("Speed to "); Serial.println(vSpeed);
      }
      break;
    case '5':
      vSpeed=230;
      analogWrite(motorAEN, vSpeed); analogWrite(motorBEN, vSpeed);    
      if (islog) {
        Serial.print("Speed to "); Serial.println(vSpeed);
      }
      break;
    case '6':
      vSpeed=250;
      analogWrite(motorAEN, vSpeed); analogWrite(motorBEN, vSpeed);    
      if (islog) {
        Serial.print("Speed to "); Serial.println(vSpeed);
      }
      break;
    case '7':
      vSpeed=255;
      analogWrite(motorAEN, vSpeed); analogWrite(motorBEN, vSpeed);    
      if (islog) {
        Serial.print("Speed to "); Serial.println(vSpeed);
      }
     case '8':
      vSpeed=300;
      analogWrite(motorAEN, vSpeed); analogWrite(motorBEN, vSpeed);    
      if (islog) {
        Serial.print("Speed to "); Serial.println(vSpeed);
      }
      break;
    default:
      // statements
      break;
  }

 /* if (analogRead(sensorRightPin) >= TiltValue) {
    motorARightTilt = true;
  } else {
    motorARightTilt = false;
  }
  if (analogRead(sensorLeftPin) >= TiltValue) {
    motorALeftTilt = true;
  } else {
    motorALeftTilt = false;
  }*/
  

  /***********************Battery*****************************/
  //Read battery voltage every 10sec.
    /*currentMillis = millis();
    if(currentMillis - (previousMillis) > (interval)) {
       previousMillis = currentMillis; 
       //Read voltage from analog pin A0 and make calibration:
       voltage = (analogRead(A0)*5.015 / 1024.0)*11.132;
       //Calculate percentage...
       perVolt = (voltage*100)/ maxBattery;
       if      (perVolt<=75)               { level=0; }
       else if (perVolt>75 && perVolt<=80) { level=1; }    //        Battery level
       else if (perVolt>80 && perVolt<=85) { level=2; }    //Min ------------------------   Max
       else if (perVolt>85 && perVolt<=90) { level=3; }    //    | 0 | 1 | 2 | 3 | 4 | 5 | >
       else if (perVolt>90 && perVolt<=95) { level=4; }    //    ------------------------
       else if (perVolt>95)                { level=5; }   
       Serial.println(level);    
    }*/
}

/*void init_motor() {

  sensorRightValue = analogRead(sensorRightPin);
  sensorLeftValue = analogRead(sensorLeftPin);

  while (abs(sensorRightValue-sensorLeftValue) > Tolerance) {
    if (sensorRightValue>sensorLeftValue) {
      analogWrite(motorAEN, vSpeed);
      digitalWrite(motorA1, HIGH); digitalWrite(motorA2, LOW); 
    } else if (sensorRightValue>sensorLeftValue) {
      analogWrite(motorAEN, vSpeed);
      digitalWrite(motorA1, LOW); digitalWrite(motorA2, HIGH); 
    }
    sensorRightValue = analogRead(sensorRightPin);
    sensorLeftValue = analogRead(sensorLeftPin); 
  }
      
  digitalWrite(motorA1, LOW); digitalWrite(motorA2, LOW); 
}

    */
