#include<avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>
#include <XBee.h>
#include <SoftwareSerial.h>

XBee xbee = XBee();

SoftwareSerial XBee(0,1); // RX, TX

ZBRxResponse ZBRx16 = ZBRxResponse();

int eleft,left,centre,right,eright;
int rm1 =8,rm2 =9,rme =10,lm1 =12,lm2 =13,lme =11;
long line=0,pre=0,start=0;
int lctr=0,rctr=0,lpulse=0,rpulse=0,lrpm,rrpm,duration,prev=0,cur=0,t=0;
volatile int lpos=0,rpos=0;
float rrot=0,lrot=0;
int lsp=255,rsp=255;

void lm()
{
  lpos=1;
  rpos=0;
  analogWrite(lme,lsp);
  digitalWrite(lm1,HIGH);
  digitalWrite(lm2,LOW);
  digitalWrite(rme,LOW);
}


void rm()
{
  rpos=1;
  lpos=0;
  analogWrite(rme,rsp);
  digitalWrite(rm1,HIGH);
  digitalWrite(rm2,LOW);
  digitalWrite(lme,LOW);
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

void halt()
{
  lpos=rpos=0;
  digitalWrite(lme,LOW);
  digitalWrite(rme,LOW);
}

void track()
{
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
  else if((line==01111)||(line==00111))
    {
      rightturn();
      pre=line;
    }
  else if((line==11110)||(line==11100))
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
      halt();
    }
}

ISR(INT1_vect)
{
  lctr++;
  lpulse++;
  delay(10);
}


ISR(INT0_vect)
{
  rctr++;
  rpulse++;
  delay(10);
}

void setup() 
{
  sei();    
  EIMSK|=(1<<INT0)|(1<<INT1);   
  EICRA|=(1<<ISC01)|(1<<ISC00)|(1<<ISC11)|(1<<ISC10); 
  Serial.begin(9600);
  Serial.flush();
  XBee.begin(9600);
  pinMode (rm1, OUTPUT);
  pinMode (rm2, OUTPUT);
  pinMode (rme, OUTPUT);
  pinMode (lm1, OUTPUT);
  pinMode (lm2, OUTPUT);
  pinMode (lme, OUTPUT);
  pinMode (2,INPUT);
  pinMode (3,INPUT);
}

void loop() 
{
  uint8_t payload[] = {lpos,lrot,lrpm,rpos,rrot,rrpm};
  eleft=analogRead(0)>400?1:0;
  left=analogRead(1)>400?1:0;
  centre=analogRead(2)>400?1:0;
  right=analogRead(3)>400?1:0;
  eright=analogRead(4)>400?1:0;
  line=eleft*10000+left*1000+centre*100+right*10+eright;
  lrot=lctr/30.0;
  rrot=rctr/30.0;
  //Serial.print(lrot);
  //Serial.print(" ");
  //Serial.println(rrot);
  XBeeAddress64 addr64 = XBeeAddress64(0x000000000000, 0x00000000FFFF); 
  if( millis() - start > 2000)
  {
    t=millis()-start;
    start=millis();
    lrpm=((lpulse/30.0)*60.0*1000.0)/t;
    rrpm=((rpulse/30.0)*60.0*1000.0)/t;
    lpulse=rpulse=0;
    /*
    Serial.print("lpos=");Serial.print(lpos);Serial.print(" ");
    Serial.print("lrot=");Serial.print(lrot);Serial.print(" ");
    Serial.print("lrpm=");Serial.print(lrpm);Serial.print(" ");
    Serial.print("rpos=");Serial.print(rpos);Serial.print(" ");
    Serial.print("rrot=");Serial.print(rrot);Serial.print(" ");
    Serial.print("rrpm=");Serial.print(rrpm);Serial.println(" ");*/
    ZBTxRequest zbTx = ZBTxRequest(addr64, payload, sizeof(payload));
    xbee.send(zbTx);
    Serial.println("Sending");
  }
  
}
