#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <XBee.h>

XBee xbee = XBee();
//SoftwareSerial XBee(0,1); // RX, TX
ZBRxResponse ZBRx16 = ZBRxResponse();

#define RADIUS 51.5
#define LENGTH 166
#define TICKSPERREV 30

#define POSITION_COMPUTE_INTERVAL 50
#define SEND_INTERVAL 100

#define UINT_MAX 4294967295

int rm1=8, rm2=9, rme=10, lm1=12, lm2=13, lme=11;

int setlpos=0,setlrot=0,setlrpm=0,setrpos=0,setrrot=0,setrrpm=0;
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

  float c = 2 * PI / (TICKSPERREV * dt_omega / 1000000.0); // ticks to rad/s conversion factor

  wl = (leftTicks - leftTicksPrev) * c;
  wr = (rightTicks - rightTicksPrev) * c;

  leftTicksPrev = leftTicks;
  rightTicksPrev = rightTicks;
  
  prevWheelComputeTime = micros();
}

void computePosition()
{
  computeAngularVelocities();
  unsigned long dt_integration = getElapsedTime(prevIntegrationTime);
  
  float dt = dt_integration / 1000000.0;
  float Vl = wl * RADIUS;
  float Vr = wr * RADIUS;
  float v = (Vr + Vl) / 2.0;
  float w = (Vr - Vl) / LENGTH;
  /*
  float xNext = xc + dt * v*(2 + cos(dt*w / 2))*cos(theta + dt * w / 2) / 3;
  float yNext = yc + dt * v*(2 + cos(dt*w / 2))*sin(theta + dt * w / 2) / 3;
  */
  float xNext = xc + dt * v*(2+ cos(dt*w))*cos(theta + dt * w) / 3;
  float yNext = yc + dt * v*(2+ cos(dt*w))*sin(theta + dt * w) / 3;
  
  float thetaNext = theta + dt * w;
  float distance = sqrt(xc * xc + yc * yc);

  xc = xNext;
  yc = yNext;
  theta = thetaNext;
  prevIntegrationTime = micros();
}

void lpid()
{
  ldif=setlrpm-lrpm;
  //Serial.println(dif);
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

void loop() 
{
  lrot=lctr/30.0;
  rrot=rctr/30.0;
  
  xbee.readPacket();
 
  if (xbee.getResponse().isAvailable())
  {
    Serial.print("receiving ");
    xbee.getResponse().getZBRxResponse(ZBRx16);
    {
      setlpos=ZBRx16.getData(0);
      setlrot=ZBRx16.getData(1);
      setlrpm=ZBRx16.getData(2);
      setrpos=ZBRx16.getData(3);
      setrrot=ZBRx16.getData(4);
      setrrpm=ZBRx16.getData(5);
      Serial.print(setlpos);Serial.print(" ");
      Serial.print(setlrot);Serial.print(" ");
      Serial.print(setlrpm);Serial.print(" ");
      Serial.print(setrpos);Serial.print(" ");
      Serial.print(setrrot);Serial.print(" ");
      Serial.println(setrrpm);
      lpid();
      rpid();
      lm();
      rm();
    }
  }
  
  if (millis() - prevPositionComputeTime > POSITION_COMPUTE_INTERVAL) 
  {
    computePosition();
    prevPositionComputeTime = millis();
  }
  
  if( millis() - start > 20)
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
