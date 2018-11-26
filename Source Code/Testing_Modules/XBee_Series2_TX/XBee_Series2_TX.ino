#include <XBee.h>

XBee xbee = XBee();
ZBTxStatusResponse txStatus = ZBTxStatusResponse();

int i=0;

// Specify the address of the remote XBee (this is the SH + SL)
XBeeAddress64 addr64_Broadcast  = XBeeAddress64(0x00000000, 0x0000FFFF); 
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
  // Create an array for holding the data you want to send.
  uint8_t payload[] = {i, 255, 128 };
  // Create a TX Request
  ZBTxRequest zbTx = ZBTxRequest(addr64_Follower, payload, sizeof(payload));
  
  // Send your request
  xbee.send(zbTx);
  Serial.print("Sending ");
  Serial.println(i);
  i=i+1;
  /*if (xbee.readPacket(5000)) 
  {
    // got a response!
    // should be a znet tx status             
    if (xbee.getResponse().getApiId() == ZB_TX_STATUS_RESPONSE) 
    {
      xbee.getResponse().getZBTxStatusResponse(txStatus);
      // get the delivery status, the fifth byte
      if (txStatus.getStatus() == SUCCESS) 
      {
        // success.  time to celebrate
        Serial.println("Success");
      } 
      else 
      {
        // the remote XBee did not receive our packet. is it powered on?
        Serial.println("Failure");
      }
    }     
  }*/
  delay(50);
}
