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

// Specify the address of the remote XBee (this is the SH + SL)
XBeeAddress64 addr64_Broadcast  = XBeeAddress64(0x00000000, 0x0000FFFF); 
XBeeAddress64 addr64_Follower  = XBeeAddress64(0x0013A200, 0x41517A82);
XBeeAddress64 addr64_Laptop  = XBeeAddress64(0x0013A200, 0x415177B1);  


int eleft,left,centre,right,eright;
int rm1=8, rm2=9, rme=10, lm1=12, lm2=13, lme=11;

int lsp=155,rsp=155;
long line=0,pre=0,start=0;
volatile int lpos=0,rpos=0;
float rrot=0,lrot=0;
volatile unsigned long lctr = 0, rctr = 0, lpulse = 0, rpulse = 0;
long leftTicks = 0, rightTicks = 0, leftTicksPrev = 0, rightTicksPrev = 0;
volatile unsigned long lrpm, rrpm, duration, prev = 0, cur = 0, t = 0;

unsigned long prevPositionComputeTime = 0, prevWheelComputeTime = 0, prevIntegrationTime = 0;
double prevX = 0, prevY = 0, xc = 0, yc = 0, theta = 0, wl = 0, wr = 0;
int xp,x1,x2,x3,yp,y1,y2,y3,theta1,theta2,v;

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
  analogWrite(lme,lsp);
  digitalWrite(lm2,HIGH);
  digitalWrite(lm1,LOW);
  analogWrite(rme,rsp);
  digitalWrite(rm1,HIGH);
  digitalWrite(rm2,LOW);
}

void rightturn()
{
  rpos=2;
  lpos=1;
  analogWrite(lme,lsp);
  digitalWrite(lm1,HIGH);
  digitalWrite(lm2,LOW);
  analogWrite(rme,rsp);
  digitalWrite(rm2,HIGH);
  digitalWrite(rm1,LOW);
}

void brake()
{
  lpos=rpos=0;
  digitalWrite(lm1,HIGH);
  digitalWrite(lm2,HIGH);
  digitalWrite(rm2,HIGH);
  digitalWrite(rm1,HIGH);
}

void halt()
{
  lpos=rpos=0;
  digitalWrite(lme,LOW);
  digitalWrite(rme,LOW);
}

void track()
{
  eleft=analogRead(0)>400?1:0;
  left=analogRead(1)>400?1:0;
  centre=analogRead(2)>400?1:0;
  right=analogRead(3)>400?1:0;
  eright=analogRead(4)>400?1:0;
  line=eleft*10000+left*1000+centre*100+right*10+eright;
  
  A:
  if((line==11011)||(line==10001))
    {
      forward();
      pre=line;
    }
  else if((line==10111)||(line==10011))
    {
      lm();
      pre=line;
    }
  else if((line==11101)||(line==11001))
    {
      rm();
      pre=line;
    }
  else if((line==01111)||(line==00111)||(line==00011))
    {
      rightturn();
      pre=line;
    }
  else if((line==11110)||(line==11100)||(line==11000))
    {
      leftturn();
      pre=line; 
    }
  else if(line==11111)
    {
      line=pre;
      goto A;
    }
  else if(line==00000)
    {
      brake();
    }
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
  float V = (Vr + Vl) / 2.0;
  float w = (Vr - Vl) / LENGTH;

  v=V;
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

void loop() 
{
  uint8_t payload[] = {lrpm,rrpm,lpos,rpos,xp,x3,x2,x1,yp,y3,y2,y1,theta2,theta1,v};
  track();
  lrot=lctr/30.0;
  rrot=rctr/30.0;

  if (millis() - prevPositionComputeTime > 50) 
  {
    computePosition();
    prevPositionComputeTime = millis();
  }
  
  if( millis() - start > 100)
  {
    t=millis()-start;
    start=millis();
    lrpm=((lpulse/30.0)*60.0*1000.0)/t;
    rrpm=((rpulse/30.0)*60.0*1000.0)/t;
    lpulse=rpulse=0;
    /*Serial.print(lrot);
    Serial.print(" ");
    Serial.println(rrot);*/
    /*
    Serial.println(" ");
    Serial.print("x=");Serial.print(xc);Serial.print(" ");
    Serial.print("y=");Serial.print(yc);Serial.print(" ");
    Serial.print("theta=");Serial.println(theta);Serial.println(" ");
    *//*
    Serial.print("lpos=");Serial.print(lpos);Serial.print(" ");
    Serial.print("lrot=");Serial.print(lrot);Serial.print(" ");
    Serial.print("lrpm=");Serial.print(lrpm);Serial.print(" ");
    Serial.print("rpos=");Serial.print(rpos);Serial.print(" ");
    Serial.print("rrot=");Serial.print(rrot);Serial.print(" ");
    Serial.print("rrpm=");Serial.print(rrpm);Serial.println(" ");*/
    ZBTxRequest zbTx = ZBTxRequest(addr64_Follower, payload, sizeof(payload));
    xbee.send(zbTx);
    Serial.println("Sending ");
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
