#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <XBee.h>

XBee xbee = XBee();
ZBRxResponse ZBRx16 = ZBRxResponse();

#define RADIUS 25.75
#define LENGTH 166
#define TICKSPERREV 30

#define POSITION_COMPUTE_INTERVAL 50
#define SEND_INTERVAL 100

#define UINT_MAX 4294967295

int eleft,left,centre,right,eright;
int rm1=8, rm2=9, rme=10, lm1=12, lm2=13, lme=11;

int lsp=155,rsp=155;
long line=0,pre=0,start=0;
volatile int lpos=0,rpos=0;
float rrot=0,lrot=0;
volatile unsigned long lctr = 0, rctr = 0, lpulse = 0, rpulse = 0, leftTicks = 0, rightTicks = 0, leftTicksPrev = 0, rightTicksPrev = 0;
volatile unsigned long lrpm, rrpm, duration, prev = 0, cur = 0, t = 0;

unsigned long prevPositionComputeTime = 0, prevWheelComputeTime = 0, prevIntegrationTime = 0;
double prevX = 0, prevY = 0, xc = 0, yc = 0, theta = 0, wl = 0, wr = 0;

void setup() 
{
  sei();    
  EIMSK|=(1<<INT0)|(1<<INT1);   
  EICRA|=(1<<ISC01)|(1<<ISC00)|(1<<ISC11)|(1<<ISC10); 
  Serial.flush();
  Serial.begin(9600);
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
  float v = (Vr + Vl) / 2.0;
  float w = (Vr - Vl) / LENGTH;
  /*
  float xNext = xc + dt * v*(2 + cos(dt*w / 2))*cos(theta + dt * w / 2) / 3;
  float yNext = yc + dt * v*(2 + cos(dt*w / 2))*sin(theta + dt * w / 2) / 3;
  */
  float thetaNext = theta + dt * w;
  float xNext = xc + dt * v*cos(thetaNext);
  float yNext = yc + dt * v*sin(thetaNext);
  
  //float thetaNext = theta + dt * w;
  float distance = sqrt(xc * xc + yc * yc);

  xc = xNext;
  yc = yNext;
  theta = thetaNext;
}

void loop() 
{
  uint8_t payload[] = {lpos,lrot,lrpm,rpos,rrot,rrpm};
  forward();
  //track();
  lrot=lctr/30.0;
  rrot=rctr/30.0;
  
  XBeeAddress64 addr64 = XBeeAddress64(0x000000000000, 0x00000000FFFF);
  
  if (millis() - prevPositionComputeTime > POSITION_COMPUTE_INTERVAL) 
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
    Serial.print(lrot);
    Serial.print(" ");
    Serial.println(rrot);
    //Serial.println(" ");
    /*Serial.print("x=");Serial.print(xc/10.0);Serial.print(" ");
    Serial.print("y=");Serial.print(yc/10.0);Serial.print(" ");
    Serial.print("theta=");Serial.println(theta);Serial.println(" ");
    /*
    Serial.print("lpos=");Serial.print(lpos);Serial.print(" ");
    Serial.print("lrot=");Serial.print(lrot);Serial.print(" ");
    Serial.print("lrpm=");Serial.print(lrpm);Serial.print(" ");
    Serial.print("rpos=");Serial.print(rpos);Serial.print(" ");
    Serial.print("rrot=");Serial.print(rrot);Serial.print(" ");
    Serial.print("rrpm=");Serial.print(rrpm);Serial.println(" ");*/
    ZBTxRequest zbTx = ZBTxRequest(addr64, payload, sizeof(payload));
    //xbee.send(zbTx);
    //Serial.println("Sending");
  }  
}

ISR(INT1_vect)
{
  lctr++;
  lpulse++;
  leftTicks++;
  delay(10);
}

ISR(INT0_vect)
{
  rctr++;
  rpulse++;
  rightTicks++;
  delay(10);
}
