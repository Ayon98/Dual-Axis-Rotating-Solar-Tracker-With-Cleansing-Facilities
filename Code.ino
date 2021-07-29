#include <Stepper.h> // include Stepper library 
#include <SoftwareSerial.h>
SoftwareSerial mySerial(0,1); // Rx, Tx

int ldrlt = 0; //LDR top lef
int lvl0;
int ldrrt = 1; //LDR top rigt 
int lvl1;
int ldrld = 2; //LDR down left
int lvl2;
int ldrrd = 3; //ldr down rigt 
int lvl3;
// Number of steps per output rotation
const int STEPS_PER_REV = 200;
const int HALF_STEP=100;

// set up the motor pins
int enA = 10;
int in1 = 9;
int in2 = 8;

String value = "";
int ledpin=13;
float x = 0;
float y = 0;

Stepper stepper_hor(STEPS_PER_REV, 8, 9, 10, 11);
Stepper stepper_vert(HALF_STEP, 4,5,6,7);

void setup()
{
  Serial.begin(9600);
  pinMode(ldrlt,INPUT);  //sets the pin for input
  pinMode(ldrrt,INPUT);
  pinMode(ldrld,INPUT);
  pinMode(ldrrd,INPUT);
  stepper_hor.setSpeed (30); //RPM horizontal motor
  stepper_vert.setSpeed (10); //RPM vertical motor
}

void loop()
{
  int lt = analogRead(ldrlt); // top left
  int rt = analogRead(ldrrt); // top right
  int ld = analogRead(ldrld); // down left
  int rd = analogRead(ldrrd); // down rigt
  
  int dtime = 2000;
  int tol = 10;
  
  int avt = (lt + rt) / 2; // average value top
  int avd = (ld + rd) / 2; // average value down
  int avl = (lt + ld) / 2; // average value left
  int avr = (rt + rd) / 2; // average value right

  int dvert = avt - avd; // check the diffirence of up and down
  int dhoriz = avl - avr;// check the diffirence og left and rigt
  
  
  Serial.print("average top :");
  Serial.println(avt);
  Serial.print("average down :");
  Serial.println(avd);
  Serial.print("average left :");
  Serial.println(avl);
  Serial.print("average right :");
  Serial.println(avr);
  
    
  if (-1*tol > dvert || dvert > tol)  // check if the diffirence is in the tolerance else change vertical angle
  {
    if (avt > avd)
    {
      Serial.println("horizontal motor : clockwise(+ve)");
      stepper_hor.step(-STEPS_PER_REV/100);  //If top avg > down avg, motor will have 2 steps i.e. 1.8*2=3.6 degree clockwise(+ve)
    }
    else if (avt < avd)
    {
      Serial.println("horizontal motor : counter-clockwise(-ve)");
      stepper_hor.step(STEPS_PER_REV/100);   //If down avg > top avg, motor will have 2 steps i.e. 1.8*2=3.6 degree counter-clockwise(-ve)
    }
  }
  else
    {
      Serial.println("horizontal motor : No rotation");
    }
  if (-1*tol > dhoriz || dhoriz > tol) // check if the diffirence is in the tolerance else change horizontal angle
  {
  if (avl > avr)
  {
    Serial.println("vertical motor : counter-clockwise(-ve)");
    stepper_vert.step(STEPS_PER_REV/100);  //If left avg > right avg, motor will have 2 steps i.e. 1.8*2=3.6 degree counter-clockwise(-ve)
  }
  else if (avl < avr)
  {
    Serial.println("vertical motor : clockwise(+ve)");
    stepper_vert.step(-STEPS_PER_REV/100);   //If right avg > left avg, motor will have 2 steps i.e. 1.8*2=3.6 degree clockwise(-ve)
  }
  }
   else
    {
      Serial.println("horizontal motor : No rotation");
    }

    while(mySerial.available()) {
    char character = mySerial.read();
    Serial.println(character);

    if(character=='x' or character=='y')
    {
      Serial.print(character);
      Serial.print("=");
      Serial.print(value);

      if(character=='x')
        x=value.toFloat();
        // turn the wheel one direction to the speed of x
      else
        y=value.toFloat();
        //turn the wheel in the other direction to the speed of y

      x=0;
      y=0;
      value="";
    }
    else
    {
      value.concat(character);
    }
  }
  lvl0 = analogRead(light0);
  lvl1 = analogRead(light1);
  lvl2 = analogRead(light2);
  lvl3 = analogRead(light3);

  Serial.println(lvl0);
  Serial.print("|");
  Serial.println(lvl1);
  Serial.print("|");
  Serial.println(lvl2);
  Serial.print("|");
  Serial.println(lvl3);
   delay(dtime);
}
