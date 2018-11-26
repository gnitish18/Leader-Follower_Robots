#include <XBee.h>

XBee xbee = XBee();
XBeeResponse response = XBeeResponse();
ZBRxResponse rx = ZBRxResponse();
ModemStatusResponse msr = ModemStatusResponse();

// Specify the address of the remote XBee (this is the SH + SL)
XBeeAddress64 addr64_Leader  = XBeeAddress64(0x000000000000, 0x00000000FFFF); 
XBeeAddress64 addr64_Follower  = XBeeAddress64(0x0013A200, 0x415177B1);  
XBeeAddress64 addr64_Laptop  = XBeeAddress64(0x0013A200, 0x41517A82);

void setup() 
{
 Serial.begin(115200);
 Serial.flush();
 xbee.setSerial(Serial); 
}

void loop() 
{
  xbee.readPacket();
  if (xbee.getResponse().isAvailable()) 
  {
    if(xbee.getResponse().getApiId()==ZB_RX_RESPONSE)
    {
      /*
      Serial.print("receiving ");
      Serial.print(xbee.getResponse().getApiId());Serial.print(" ");
      Serial.print(ZB_RX_RESPONSE);Serial.print(" ");
      Serial.println(MODEM_STATUS_RESPONSE);*/
      /*
      Serial.print("Acknowledge ");
      Serial.print(rx.getOption());Serial.print(" ");
      Serial.println(ZB_PACKET_ACKNOWLEDGED);*/
      
      xbee.getResponse().getZBRxResponse(rx);
      Serial.print(rx.getData(0));Serial.print(" ");
      Serial.print(rx.getData(1));Serial.print(" ");
      Serial.println(rx.getData(2));
    }      
 }
}
