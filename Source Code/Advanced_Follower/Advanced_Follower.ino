#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <XBee.h>

XBee xbee = XBee();
ZBTxStatusResponse txStatus = ZBTxStatusResponse();
XBeeResponse response = XBeeResponse();
ZBRxResponse rx = ZBRxResponse();
//ModemStatusResponse msr = ModemStatusResponse();

XBeeAddress64 addr64_Leader  = XBeeAddress64(0x00000000, 0x0000FFFF); 
XBeeAddress64 addr64_Follower  = XBeeAddress64(0x0013A200, 0x41517A82);
XBeeAddress64 addr64_Laptop  = XBeeAddress64(0x0013A200, 0x415177B1);  


#define RADIUS 25.75
#define LENGTH 172
#define TICKSPERREV 30

#define UINT_MAX 4294967295

int rm1=8, rm2=9, rme=10, lm1=12, lm2=13, lme=11;

float dist=0;
int reached=0;

int setlpos=0,setlrpm=0,setrpos=0,setrrpm=0;
int ldif=0,lid=0,ldd=0,lprev=0,rdif=0,rid=0,rdd=0,rprev=0;
int lsp=155,rsp=155;
int lpre_ocr=0,rpre_ocr=0;
long line=0,pre=0,start=0;
volatile int lpos=0,rpos=0;
float rrot=0,lrot=0;
volatile unsigned long lctr = 0, rctr = 0, lpulse = 0, rpulse = 0, leftTicks = 0, rightTicks = 0, leftTicksPrev = 0, rightTicksPrev = 0;
volatile unsigned long lrpm, rrpm, duration, prev = 0, cur = 0, t = 0;

unsigned long prevPositionComputeTime = 0, prevWheelComputeTime = 0, prevIntegrationTime = 0;
double prevX = 0, prevY = 0, xc = 0, yc = 0, theta = 0, wl = 0, wr = 0;
int sxp=0,syp=0,sx1=0,sy1=0,stheta1=0,sx2=0,sy2=0,stheta2=0,sx3=0,sy3=0;
double sx=0,sy=0,stheta=0;
int xp=0,yp=0,x1=0,y1=0,theta1=0,x2=0,y2=0,theta2=0,x3=0,y3=0;

uint8_t payload[] = {sxp,sx3,sx2,sx1,syp,sy3,sy2,sy1,stheta2,stheta1,xp,x3,x2,x1,yp,y3,y2,y1,theta2,theta1};
ZBTxRequest zbTx = ZBTxRequest(addr64_Laptop, payload, sizeof(payload));

void setup() 
{
  sei();    
  EIMSK|=(1<<INT0)|(1<<INT1);   
  EICRA|=(1<<ISC01)|(1<<ISC00)|(1<<ISC11)|(1<<ISC10); 
  Serial.flush();
  Serial.begin(115200);
  xbee.setSerial(Serial);
  pinMode (rm1, OUTPUT);
  pinMode (rm2, OUTPUT);
  pinMode (rme, OUTPUT);
  pinMode (lm1, OUTPUT);
  pinMode (lm2, OUTPUT);
  pinMode (lme, OUTPUT);
  pinMode (2,INPUT);
  pinMode (3,INPUT);
}

void lmf()
{
  lpos=1;
  rpos=0;
  analogWrite(lme,lsp);
  digitalWrite(lm1,HIGH);
  digitalWrite(lm2,LOW);
}

void rmf()
{
  rpos=1;
  lpos=0;
  analogWrite(rme,rsp);
  digitalWrite(rm1,HIGH);
  digitalWrite(rm2,LOW);
}

void lmb()
{
  lpos=2;
  analogWrite(lme,lsp);
  digitalWrite(lm1,LOW);
  digitalWrite(lm2,HIGH);
}

void rmb()
{
  rpos=2;
  analogWrite(rme,rsp);
  digitalWrite(rm1,LOW);
  digitalWrite(rm2,HIGH);
}

void lmh()
{
  lpos=0;
  analogWrite(lme,lsp);
  digitalWrite(lm1,LOW);
  digitalWrite(lm2,LOW);
}

void rmh()
{
  rpos=0;
  analogWrite(rme,rsp);
  digitalWrite(rm1,LOW);
  digitalWrite(rm2,LOW);
}

void leftturn()
{
  lpos=2;
  rpos=1;
  analogWrite(lme,80);
  digitalWrite(lm2,HIGH);
  digitalWrite(lm1,LOW);
  analogWrite(rme,80);
  digitalWrite(rm1,HIGH);
  digitalWrite(rm2,LOW);
}

void rightturn()
{
  rpos=2;
  lpos=1;
  analogWrite(lme,80);
  digitalWrite(lm1,HIGH);
  digitalWrite(lm2,LOW);
  analogWrite(rme,80);
  digitalWrite(rm2,HIGH);
  digitalWrite(rm1,LOW);
}

void halt()
{
  lpos=rpos=0;
  digitalWrite(lme,LOW);
  digitalWrite(rme,LOW);
  digitalWrite(lm1,LOW);
  digitalWrite(lm2,LOW);
  digitalWrite(rm2,LOW);
  digitalWrite(rm1,LOW);
}

void forward()
{
  lpos=rpos=1;
  analogWrite(lme,lsp);
  digitalWrite(lm1,HIGH);
  digitalWrite(lm2,LOW);
  analogWrite(rme,rsp);
  digitalWrite(rm1,HIGH);
  digitalWrite(rm2,LOW);
}

void reverse()
{
  lpos=rpos=2;
  analogWrite(lme,lsp);
  digitalWrite(lm2,HIGH);
  digitalWrite(lm1,LOW);
  analogWrite(rme,rsp);
  digitalWrite(rm2,HIGH);
  digitalWrite(rm1,LOW);
}

void orient(float st)
{
  if((st-theta)>0.1)
    {
      if((st-theta)<=PI)
      {
        while((theta<(st-0.1))||(theta>(st+0.1)))
          {
            leftturn();
            if (millis() - pre > 50) 
            {
              computePosition();
              pre = millis();
            }
            Serial.print(st);Serial.print(" ");
            Serial.println(theta);
          }
          halt();
      }
      else if((st-theta)>PI)
      {
        while((theta<(st-0.1))||(theta>(st+0.1)))
          {
            rightturn();
            if (millis() - pre > 50) 
            {
              computePosition();
              pre = millis();
            }
            Serial.print(st);Serial.print(" ");
            Serial.println(theta);
          }
          halt();
      }
    }
    
    else if((st-theta)<(-0.1))
    {
      if((st-theta)>=PI)
      {
        while((theta<(st-0.1))||(theta>(st+0.1)))
          {
            leftturn();
            if (millis() - pre > 50) 
            {
              computePosition();
              pre = millis();
            }
            Serial.print(st);Serial.print(" ");
            Serial.println(theta);
          }
          halt();
      }
      else if((st-theta)<PI)
      {
        while((theta<(st-0.1))||(theta>(st+0.1)))
          {
            rightturn();
            if (millis() - pre > 50) 
            {
              computePosition();
              pre = millis();
            }
            Serial.print(st);Serial.print(" ");
            Serial.println(theta);
          }
          halt();
      }
    }
    reached=1;
}

float setang(float sx,float sy)
{
  float ang;
  if(((sy-yc)>=0)&&((sx-xc)>=0))
  {
    ang=atan((sy-yc)/(sx-xc));
  }
  else if(((sy-yc)>=0)&&((sx-xc)<=0))
  {
    ang=PI+atan((sy-yc)/(sx-xc));
  }
  else if(((sy-yc)<=0)&&((sx-xc)<=0))
  {
    ang=PI+atan((sy-yc)/(sx-xc));
  }
  else if(((sy-yc)<=0)&&((sx-xc)>=0))
  {
    ang=2*PI+atan((sy-yc)/(sx-xc));
  }
  return ang;    
}

long preTime=0,sendTime=0;

void reach(float sx,float sy,float st=100)
{
  computePosition();
  float inix=xc,iniy=yc;
  orient(setang(sx,sy));
  dist=sqrt(((sy-yc)*(sy-yc))+((sx-xc)*(sx-xc)));
  float d=0.0;
    
  while(d<(dist))
  {
    if (millis() - preTime > 50) 
    {
      computePosition();
      preTime = millis();
    }
    if (millis() - sendTime > 250) 
    {
      payload_update();
      xbee.send(zbTx);
      sendTime = millis();
    }
    d=sqrt((yc-iniy)*(yc-iniy)+(xc-inix)*(xc-inix));
    forward();
    if(d>=(dist))
    {
      halt();
      break;
    }
    Serial.print(dist);Serial.print(" ");
    Serial.println(d);
  }
  halt();
  if(st!=100)
    orient(st);
  reached=1;
}

unsigned long getElapsedTime(unsigned long prevTime) 
{
  unsigned long currentTime = micros();
  if (currentTime < prevTime)
    return UINT_MAX - prevTime + currentTime;
  return currentTime - prevTime;
}

void computeAngularVelocities() 
{
  unsigned long dt_omega = getElapsedTime(prevWheelComputeTime); // in microseconds
  prevWheelComputeTime = micros();

  float c = (2 * PI) / ((TICKSPERREV * dt_omega) / 1000000.0); // ticks to rad/s conversion factor

  wl = (leftTicks - leftTicksPrev) * c;
  wr = (rightTicks - rightTicksPrev) * c;

  leftTicksPrev = leftTicks;
  rightTicksPrev = rightTicks;
}

void computePosition()
{
  computeAngularVelocities();
  unsigned long dt_integration = getElapsedTime(prevIntegrationTime);
  prevIntegrationTime = micros();
  
  float dt = dt_integration / 1000000.0;
  float Vl = wl * RADIUS;
  float Vr = wr * RADIUS;
  float v = (Vr + Vl) / 2.0;
  float w = (Vr - Vl) / LENGTH;
  /*
  float xNext = xc + dt * v*(2 + cos(dt*w / 2))*cos(theta + dt * w / 2) / 3;
  float yNext = yc + dt * v*(2 + cos(dt*w / 2))*sin(theta + dt * w / 2) / 3;
  */
  float thetaNext = theta + dt * w;
  float xNext = xc + (dt * v*cos(thetaNext))/10.0;
  float yNext = yc + (dt * v*sin(thetaNext))/10.0;
  
  //float thetaNext = theta + dt * w;
  float distance = sqrt(xc * xc + yc * yc);

  xc = xNext;
  yc = yNext;
  theta = thetaNext;

  if(theta>2*PI)
    theta=(theta-(2*PI));
  else if(theta<0)
    theta=(theta+(2*PI));

  if(xc<0)
  {
    x1=(int(-1*xc))%100;
    x2=(int(-1*xc/100))%100;
    x3=(int(-1*xc/10000))%100;
    xp=1;
  }
  else
  {
    x1=(int(xc))%100;
    x2=(int(xc/100))%100;
    x3=(int(xc/10000))%100;
    xp=0;
  }

  if(yc<0)
  {
    y1=(int(-1*yc))%100;
    y2=(int(-1*yc/100))%100;
    y3=(int(-1*yc/10000))%100;
    yp=1;
  }
  else
  {
    y1=(int(yc))%100;
    y2=(int(yc/100))%100;
    y3=(int(yc/10000))%100;
    yp=0;
  }

  theta1=(int(theta*100))%100;
  theta2=(int(theta*100))/100;
}

void lpid()
{
  ldif=setlrpm-lrpm;
  //Serial.println(ldif);
  //lid+=ldif;
  //ldd=ldif-lprev;
  //lprev=ldif;
  lsp=lpre_ocr+((255.0/400.0)*(ldif));
  lpre_ocr=lsp;
  if(lsp>255)
    lsp=255;
}

void rpid()
{
  rdif=setrrpm-rrpm;
  //Serial.println(rdif);
  //rid+=rdif;
  //rdd=rdif-rprev;
  //rprev=rdif;
  rsp=rpre_ocr+((255.0/400.0)*(rdif));
  rpre_ocr=rsp;
  if(rsp>255)
    rsp=255;
}

void payload_update()
{
  payload[0] = xp;
  payload[1] = x3;
  payload[2] = x2;
  payload[3] = x1;
  payload[4] = yp;
  payload[5] = y3;
  payload[6] = y2;
  payload[7] = y1;
  payload[8] = theta2;
  payload[9] = theta1;
}

void loop() 
{
  lrot=lctr/30.0;
  rrot=rctr/30.0;
  
  xbee.readPacket();
 
  if (xbee.getResponse().isAvailable())
  if(xbee.getResponse().getApiId()==ZB_RX_RESPONSE) 
  {
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
    
    Serial.print(sx);Serial.print(" ");
    Serial.print(sy);Serial.print(" ");
    Serial.println(stheta);

    reach(sx,sy);
    payload_update();
    xbee.send(zbTx);
  }
  
  if (millis() - prevPositionComputeTime > 50) 
  {
    computePosition();
    prevPositionComputeTime = millis();
  }
  
  if( millis() - start > 200)
  {
    t=millis()-start;
    start=millis();
    lrpm=((lpulse/30.0)*60.0*1000.0)/t;
    rrpm=((rpulse/30.0)*60.0*1000.0)/t;
    lpulse=rpulse=0;
    /*Serial.print(lrot);
    Serial.print(" ");
    Serial.println(rrot);*/
  }  
}

ISR(INT1_vect)
{
  lctr++;
  lpulse++;
  if(lpos==2)
    leftTicks--;
  else
    leftTicks++;
  delay(10);
}

ISR(INT0_vect)
{
  rctr++;
  rpulse++;
  if(rpos==2)
    rightTicks--;
  else
    rightTicks++;
  delay(10);
}
