#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <XBee.h>

XBee xbee = XBee();
ZBTxStatusResponse txStatus = ZBTxStatusResponse();

#define RADIUS 25.75
#define LENGTH 166
#define TICKSPERREV 30

#define UINT_MAX 4294967295

XBeeAddress64 addr64_Broadcast  = XBeeAddress64(0x00000000, 0x0000FFFF); 
XBeeAddress64 addr64_Follower  = XBeeAddress64(0x0013A200, 0x41517A82);
XBeeAddress64 addr64_Laptop  = XBeeAddress64(0x0013A200, 0x415177B1);  

int eleft,left,centre,right,eright;
int rm1=8, rm2=9, rme=10, lm1=12, lm2=13, lme=11;

float dist=0;
int reached=0;

int lsp=155,rsp=155;
long line=0,pre=0,start=0;
volatile int lpos=0,rpos=0;
float rrot=0,lrot=0;
volatile unsigned long lctr = 0, rctr = 0, lpulse = 0, rpulse = 0;
long leftTicks = 0, rightTicks = 0, leftTicksPrev = 0, rightTicksPrev = 0;
volatile unsigned long lrpm, rrpm, duration, prev = 0, cur = 0, t = 0;

unsigned long prevPositionComputeTime = 0, prevWheelComputeTime = 0, prevIntegrationTime = 0;
double prevX = 0, prevY = 0, xc = 0, yc = 0, theta = 0, wl = 0, wr = 0;
int xp=0,x1=0,x2=0,x3=0,yp=0,y1=0,y2=0,y3=0,theta1=0,theta2=0;

uint8_t payload[] = {xp,x3,x2,x1,yp,y3,y2,y1,theta2,theta1};
ZBTxRequest zbTx = ZBTxRequest(addr64_Follower, payload, sizeof(payload));

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

void lm()
{
  lpos=1;
  rpos=0;
  analogWrite(lme,lsp);
  digitalWrite(lm1,HIGH);
  digitalWrite(lm2,LOW);
  digitalWrite(rm1,HIGH);
  digitalWrite(rm2,HIGH);
}

void rm()
{
  rpos=1;
  lpos=0;
  analogWrite(rme,rsp);
  digitalWrite(rm1,HIGH);
  digitalWrite(rm2,LOW);
  digitalWrite(lm1,HIGH);
  digitalWrite(lm2,HIGH);
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
  digitalWrite(lm1,LOW);
  digitalWrite(lm2,LOW);
  digitalWrite(rm2,LOW);
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
    //reached=1;
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
  payload_update();
  lrot=lctr/30.0;
  rrot=rctr/30.0;
  if(reached==0)
    {
      reached=0;
      payload_update();
      xbee.send(zbTx);
      reach(60.0,14.0);
      payload_update();
      xbee.send(zbTx);
      reach(20.0,22.0);
      payload_update();
      xbee.send(zbTx);
      reach(0.0,60.0);
      payload_update();
      xbee.send(zbTx);
      reach(-20.0,22.0);
      payload_update();
      xbee.send(zbTx);
      reach(-60.0,14.0);
      payload_update();
      xbee.send(zbTx);
      reach(-32.0,-18.0);
      payload_update();
      xbee.send(zbTx);
      reach(-38.0,-60.0);
      payload_update();
      xbee.send(zbTx);
      reach(0.0,-42.0);
      payload_update();
      xbee.send(zbTx);
      reach(38.0,-60.0);
      payload_update();
      xbee.send(zbTx);
      reach(32.0,-18.0);
      payload_update();
      xbee.send(zbTx);
      reach(60.0,14.0);
      payload_update();
      xbee.send(zbTx);
      //reach(0.0,0.0);
    }
  
  if(reached==1)
    Serial.println("reached");
    
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
