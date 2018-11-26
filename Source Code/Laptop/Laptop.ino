#include <XBee.h>

XBee xbee = XBee();
XBeeResponse response = XBeeResponse();
ZBRxResponse rx = ZBRxResponse();
//ModemStatusResponse msr = ModemStatusResponse();

/*
XBeeAddress64 addr64_Leader  = XBeeAddress64(0x00000000, 0x0000FFFF);
XBeeAddress64 addr64_Follower  = XBeeAddress64(0x0013A200, 0x41517A82);
XBeeAddress64 addr64_Laptop  = XBeeAddress64(0x0013A200, 0x415177B1);
*/

int sxp=0,syp=0,sx1=0,sy1=0,stheta1=0,sx2=0,sy2=0,stheta2=0,sx3=0,sy3=0;
double sx=0,sy=0,stheta=0;
int xp=0,yp=0,x1=0,y1=0,theta1=0,x2=0,y2=0,theta2=0,x3=0,y3=0;
double x=0,y=0,theta=0;

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
      sxp=rx.getData(0);
      sx3=rx.getData(1);
      sx2=rx.getData(2);
      sx1=rx.getData(3);
      syp=rx.getData(4);
      sy3=rx.getData(5);
      sy2=rx.getData(6);
      sy1=rx.getData(7);
      stheta2=rx.getData(8);
      stheta1=rx.getData(9);
      xp=rx.getData(10);
      x3=rx.getData(11);
      x2=rx.getData(12);
      x1=rx.getData(13);
      yp=rx.getData(14);
      y3=rx.getData(15);
      y2=rx.getData(16);
      y1=rx.getData(17);
      theta2=rx.getData(18);
      theta1=rx.getData(19);

      if(sxp==0)
      {
        sx=sx3*10000+sx2*100+sx1;
      }
      else
      {
        sx=-1*(sx3*10000+sx2*100+sx1);
      }
      if(syp==0)
      {
        sy=sy3*10000+sy2*100+sy1;
      }
      else
      {
        sy=-1*(sy3*10000+sy2*100+sy1);
      }
      stheta=(stheta2*100.0+stheta1)/100.0;

      if(xp==0)
      {
        x=x3*10000+x2*100+x1;
      }else
      {
        x=-1*(x3*10000+x2*100+x1);
      }
      if(yp==0)
      {
        y=y3*10000+y2*100+y1;
      }
      else
      {
        y=-1*(y3*10000+y2*100+y1);
      }
      theta=(theta2*100.0+theta1)/100.0;
      
      Serial.print(sx);Serial.print(" ");
      Serial.print(sy);Serial.print(" ");
      Serial.print(stheta);Serial.print(" ");
      Serial.print(x);Serial.print(" ");
      Serial.print(y);Serial.print(" ");
      Serial.println(theta);
    }      
 }
}
