//TX_CODE
#include<SPI.h>
#include<Arduino.h>
#include<Wire.h>
#include<nRF24L01.h>
#include<RF24.h>
#include<HMC5883L_Simple.h>

 //Create a compass
HMC5883L_Simple Compass;
RF24 radio(8, 10);  //ce,csn pin

const byte address[][6] ={"00001","00002"};  //address to write data
 
//char data_rx[32] = {0} ;
char tx_data='h';    //sending 'h' through nrf
int flag=0;

void setup()
{
  Serial.begin(9600) ;  //setting the baudrate for 9600
  radio.begin();
  radio.openWritingPipe(address[1]) ; //open pipe for writing the data
  radio.openReadingPipe(1,address[0]) ;  //open pipe for reading the data
  radio.setPALevel(RF24_PA_MIN);  //You can set it as minimum or maximum depending on the distance between the transmitter and receiver. 
  Wire.begin();  //to intialise the i2c communication
  Compass.SetDeclination(-0, 23, 'W');  //to set the magnetometer intialisation
  Compass.SetSamplingMode(COMPASS_SINGLE);
  Compass.SetScale(COMPASS_SCALE_130);
  Compass.SetOrientation(COMPASS_HORIZONTAL_X_NORTH);//To give the direction of x-axis taken as heading values of magnetometer
}
void loop()
{
  float heading = Compass.GetHeadingDegrees();  //storing the degrees into one variable that is heading 
  //  radio.write(&heading, sizeof(heading));
  Serial.print("Heading: \t");
  Serial.println( heading );//printing the heading values
    NRF_INIT();  //calling nrf initialization function 
 
  delay(50);
  if(flag==1)
  {
  NRF_TX(heading);    //calling NRF_transmission function 
  }
}

void NRF_INIT()  //function to intialise the NRF
{
  if(flag==0)
  {
   //  Serial.println("init fn") ;
  delay(5);
  radio.stopListening() ;  //nrf transmission on
  char data_tx[]="Hello";   //transmitting 'hello' through nrf
  // radio.openWritingPipe(data_1) ;  //open writing pipe to write data
  radio.write(&data_tx,sizeof(data_tx)); //sending data through nrf radio
   Serial.println(data_tx) ;
  // flag=1;
  delay(5);
  radio.startListening();   //nrf receiver on
  while(!radio.available());
  //  if (radio.available())    //checking data availability
  //   { 
    //  Serial.println("RX_data_avalable") ;
     char data_rx[32] = {0} ;
     //radio.openReadingPipe(1,data_2) ;  //open reading pipe to read data
    radio.read(&data_rx, sizeof(data_rx));  //receiving data from radio 
    Serial.println(data_rx) ; //printing data on serial monitor
   // radio.stopListening();   // receiver off
 // }
  }
  flag=1;
}

void NRF_TX(float tx_data)                    //function to transmit the NRF
{
  delay(5);
  radio.stopListening() ;       //transmission on 
  //radio.openWritingPipe(data) ;       //writing pipe to write data 
  radio.write(&tx_data,sizeof(tx_data));    //sending data through radio
 // digitalWrite(led,HIGH);     //indication of data transmission successful 
  Serial.println(tx_data) ;     //printing data on serial monitor
}

