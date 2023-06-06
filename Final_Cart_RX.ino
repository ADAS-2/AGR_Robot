//RX_CODE
#include <SPI.h>
#include <nRF24L01.h>
#include "RF24.h"
#include <Arduino.h>
#include <Wire.h>
#include <TFLI2C.h>

TFLI2C tflI2C;//TF LUNO Lidar sensor 

int16_t tfDist;//variable to store the distance
int16_t tfAddr = TFL_DEF_ADR; //address of LUNO 

int ENA = 9;             //enable pin for motorA 
int ENB = 9;             //enable pin for motorB                                   
int MotorA1 = 4;         //motor pin for first motor                                  
int MotorA2 = 5;         //motor pin for first motor 
int MotorB1 = 6;         //motor pin for second motor 
int MotorB2 = 7;         //motor pin for second motor 

int angle_values[50];    //first fifty values  are storing for check the direction to take(left or right)
int current_value=0;     //intialising the current value to zero
int calibrate_value;     //calibrate value is the difference between current value and previous value

RF24 radio(8, 10); //cse=10,ce=8 these are nrf pins

const byte addresses [][6] = {"00001", "00002"};  //Setting the two addresses. One for transmitting and one for receiving
float Heading;
int old_value = 0;
float rf_value=0;
int flag =0;
int weight = 0 ;

void setup() 
{
  // put your setup code here, to run once:
  pinMode(3, OUTPUT);//NRF - Pairing
  pinMode(2, OUTPUT); // Object detection
  //pinMode(1, INPUT_PULLUP);
  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(addresses[0]);     //Setting the address at which we will send the data
  radio.openReadingPipe(1, addresses[1]); //Setting the address at which we will receive the data
  radio.setPALevel(RF24_PA_MIN);//You can set it as minimum or maximum depending on the distance between the transmitter and receiver. 
  radio.startListening(); //receiver is on
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(MotorA1, OUTPUT);
  pinMode(MotorA2, OUTPUT);
  pinMode(MotorB1, OUTPUT);
  pinMode(MotorB2, OUTPUT);
  Wire.begin();
  NRF_INIT();  //to intialise the nrf
}


void loop() 
{

delay(100);
   if(flag == 1) //when pairing is done,then flag is set
   {
     digitalWrite(3, HIGH); //after pairing green led is glown
    NRF_RX();  //calling nrf initialization function
   } 
  if (tflI2C.getData(tfDist, tfAddr)) //get data from luno lidar senasor
   {
    float distMeters = tfDist / 100.0; // Convert centimeters to meters
    Serial.println(distMeters);  //printing the distance in meters
    stop();
    if(distMeters>1) //if distance is greater than 1 meter
     {
       digitalWrite(2, LOW); //greater than 1 meter Led is off
     }
    else
     {
       digitalWrite(2, HIGH);  // object detected means Red led is glown
       }
    if(distMeters>1)// Always check the distance between human and cart  
    {
         
          Serial.println("motor about to start");

        //moveForward(timeInSeconds*15000);
        moveForward(1500);  //to move .1 meter 

          Serial.println("motor stop");
     // digitalWrite(Led, LOW);
    //put your main code here, to run repeatedly
        rf_value = Heading;
          //Serial.println(Heading);
          angle_values[current_value]=rf_value;  //magnetometer values storing in array

          if(current_value>0)
          {
              calibrate_value= angle_values[current_value]-angle_values[current_value-1];   //calibrate value is the difference between current value and previous value
              // delay(1000);
              if(calibrate_value>10)
              {
               right();  //turn right 
                  delay(1000);
                stop();
                   delay(1000);
              }
  
             if(calibrate_value<-5 )
             {
               left();  //turn left
                  delay(50);
               stop();
                  delay(50);
                        
             }
              
          }

          if(current_value>50) //if the current value is greater than 50 means again start the value from zero
              current_value=0;
              current_value++;

    }
    else
    {
      
      stop();  //stop the motor
       delay(50);
      
    }
    
    }
      delay(50);
 }

void right()
{
        digitalWrite(MotorA1, LOW);  //right
        digitalWrite(MotorA2, HIGH);
        digitalWrite(MotorB1, LOW);
        digitalWrite(MotorB2, HIGH);
        analogWrite(ENA, 100);
        analogWrite(ENB, 100);       
}
void left()
{
       
        digitalWrite(MotorA1, HIGH);  //left
        digitalWrite(MotorA2, LOW);
        digitalWrite(MotorB1, HIGH);
        digitalWrite(MotorB2, LOW);
        analogWrite(ENA, 100);
        analogWrite(ENB, 100);
     

}
void stop()
{
          digitalWrite(MotorA1, LOW);  //stop
          digitalWrite(MotorA2, LOW);
          digitalWrite(MotorB1, LOW);
          digitalWrite(MotorB2, LOW);
          analogWrite(ENA, 0);
          analogWrite(ENB, 0);
}     

void forword()
{
      digitalWrite(MotorA1, HIGH);  //forword
      digitalWrite(MotorA2, LOW);
      digitalWrite(MotorB1, LOW);
      digitalWrite(MotorB2, HIGH);
      analogWrite(ENA, 100);
      analogWrite(ENB, 100);
}

// Function to move the robot forward for a specified time duration
bool moveForward(unsigned long duration) {
  
  forword();
  Serial.println("before delay");
  delay(duration);  // Delay for the specified duration
  Serial.println("after delay");
  stop();
  return true;  // Return true to indicate successful movement
}


void NRF_INIT()
{
  if(flag==0)
  {
  Serial.println("NRF_INIT") ;
  delay(5);
  radio.startListening();   //nrf receiver on
  while(!radio.available());
    Serial.println("NRF_INIT_Data_avalaible") ;
    char rcv_msg[32] = {0} ;
    radio.read(&rcv_msg, sizeof(rcv_msg));   //receiving data from radio 
    Serial.println(rcv_msg) ; //printing data on serial monitor

    delay(5); 
    radio.stopListening() ;   //nrf transmission on
    char send_msg[]="nrfstarted";  //transmitting 'nrfstarted' through nrf
    radio.write(&send_msg,sizeof(send_msg));   //sending data through nrf radio
    Serial.println(send_msg) ;
  }
  flag = 1;
}

void NRF_RX( )
{
  delay(5);  
  radio.startListening(); //nrf receiver on
  if(radio.available())   //checking data availability
  {
    radio.read(&Heading, sizeof(Heading));  //receiving data from nrf 
   // digitalWrite(led,HIGH);  //indication om nf successful data receive
    Serial.println(Heading) ; //printing data on serial monitor
    //Serial.println("Rx_fn") ;
  } 
}
