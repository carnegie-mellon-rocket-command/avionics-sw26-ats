#include <Servo.h> 

Servo myservo;  // create servo object to control a servo 
                // a maximum of eight servo objects can be created 
 
int pos = 0;    // variable to store the servo position 
int servPin = 6; // Chose pin at random
int potPin = A5; //AKA pin 19

void setup() 
{ 
  Serial.begin(9600);
  // myservo.write(93);
  myservo.attach(servPin);  // attaches the servo on pin 20 
  Serial.println("Initialized");
  myservo.write(180);
  Serial.println(myservo.read());
  delay(2000);
} 
 

void loop() 
{
  // int potVal = analogRead(potPin); //Read analogue value from potentiometer
  // Serial.println(potVal);
  //int pos = potVal / 1023.0 * 180; // Turn potVal into a percentage of 130 degrees (Max travel range)
  // myservo.write(180); // Send position data to servo
  // Serial.println(myservo.read());
  
  // delay(3000); // Repeat
  
  // myservo.write(13);
  // Serial.println(myservo.read());

  delay(3000);
} 