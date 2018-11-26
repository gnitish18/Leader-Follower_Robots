#include <XBee.h>

XBee xbee = XBee();
XBeeResponse response = XBeeResponse();

Rx16Response ZBRx16 = Rx16Response();
Rx64Response ZBRx64 = Rx64Response();
#define MAX_FRAME_DATA_SIZE 110

void setup() 
{
  Serial.flush();
  Serial.begin(9600);
  xbee.setSerial(Serial);
}

void loop() 
{
 xbee.readPacket();
 
 if (xbee.getResponse().isAvailable()) 
 {  
       Serial.print("receiving ");
       Serial.print(xbee.getResponse().getApiId());Serial.print(" ");
       Serial.print(RX_16_RESPONSE);Serial.print(" ");
       Serial.println(RX_64_RESPONSE);
       //if((xbee.getResponse().getApiId()==RX_16_RESPONSE)||(xbee.getResponse().getApiId()==RX_64_RESPONSE)) 
        {
          //if (xbee.getResponse().getApiId()==RX_16_RESPONSE) 
          {
            xbee.getResponse().getRx16Response(ZBRx16);
            Serial.println(ZBRx16.getData(0));
            xbee.getResponse().getRx64Response(ZBRx64);
            Serial.println(ZBRx64.getData(0));
            /*
            Serial.print("lpos=");Serial.print(ZBRx16.getData(0));Serial.print(" ");
            Serial.print("lrot=");Serial.print(ZBRx16.getData(1));Serial.print(" ");
            Serial.print("lrpm=");Serial.print(ZBRx16.getData(0));Serial.print(" ");
            Serial.print("rpos=");Serial.print(ZBRx16.getData(1));Serial.print(" ");
            Serial.print("rrot=");Serial.print(ZBRx16.getData(0));Serial.print(" ");
            Serial.print("rrpm=");Serial.println(ZBRx16.getData(1));*/
          } 
          /*else 
          {
            xbee.getResponse().getRx64Response(ZBRx64);
            Serial.print("lpos=");Serial.print(ZBRx64.getData(0));Serial.print(" ");
            Serial.print("lrot=");Serial.print(ZBRx64.getData(1));Serial.print(" ");
            Serial.print("lrpm=");Serial.print(ZBRx64.getData(0));Serial.print(" ");
            Serial.print("rpos=");Serial.print(ZBRx64.getData(1));Serial.print(" ");
            Serial.print("rrot=");Serial.print(ZBRx64.getData(0));Serial.print(" ");
            Serial.print("rrpm=");Serial.println(ZBRx64.getData(1));
          }*/
        }       
 }
}
