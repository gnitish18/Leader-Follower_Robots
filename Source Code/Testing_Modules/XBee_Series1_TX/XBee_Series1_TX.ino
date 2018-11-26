#include <XBee.h>

XBee xbee = XBee();
//ZBRxResponse ZBRx16 = ZBRxResponse();
TxStatusResponse txStatus = TxStatusResponse();

int i=0;

// Specify the address of the remote XBee (this is the SH + SL)
XBeeAddress64 addr64_0  = XBeeAddress64(0x000000000000, 0x00000000FFFF); 
XBeeAddress64 addr64_11 = XBeeAddress64(0x000000000000, 0x0013A200415177B1);
XBeeAddress64 addr64_1  = XBeeAddress64(0x0013A200, 0x415177B1); 
XBeeAddress64 addr64_22 = XBeeAddress64(0x000000000000, 0x0013A20041517A82); 
XBeeAddress64 addr64_2  = XBeeAddress64(0x0013A200, 0x41517A82);

void setup() 
{
 Serial.begin(9600);
 Serial.flush();
 xbee.setSerial(Serial);
}

void loop() 
{
  // Create an array for holding the data you want to send.
  uint8_t payload[] = {i, 255, 128 };
  // Create a TX Request
  Tx64Request zbTx1 = Tx64Request(addr64_1, payload, sizeof(payload));
  Tx64Request zbTx2 = Tx64Request(addr64_2, payload, sizeof(payload));

  // Send your request
  xbee.send(zbTx1);
  xbee.send(zbTx2);
  Serial.print("Sending ");
  Serial.println(i);
  i=i+1;
  /*if (xbee.readPacket(5000)) 
  {
    // got a response!
    // should be a znet tx status             
    if (xbee.getResponse().getApiId() == TX_STATUS_RESPONSE) 
    {
      xbee.getResponse().getTxStatusResponse(txStatus);
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
  delay(200);
}
