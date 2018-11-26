#include<avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>

int lm1 =12,lm2 =13,lme =11,rm1 =8,rm2 =9,rme =10;
int lctr=0,rctr=0,t;
int lsp=255,lsetrpm=50,lrpm=0,rsp=255,rsetrpm=50,rrpm=0;
int ldif=0,lid=0,ldd=0,lprev=0,rdif=0,rid=0,rdd=0,rprev=0;
float lki=1,lkd=1,lkp=1,rki=1,rkd=1,rkp=1;
float lrot=0,rrot=0;
volatile long lpulse,rpulse;
int lpre_ocr=0,rpre_ocr=0;
unsigned long start;

/*
int getParam()  
{
  char param, cmd;
  if(!Serial.available())    return 0;
  delay(10);                  
  param = Serial.read();                              // get parameter byte
  if(!Serial.available())    return 0;
  cmd = Serial.read();                                // get command byte
  Serial.flush();                                     // clean serial buffer
  switch (param) 
  {  
    case 's':                                        // adjust speed more
              if(cmd=='+')  
              {
                speed_req += 20;
                if(speed_req>60)   speed_req=60;
              }
              if(cmd=='-')    
              {
                speed_req -= 20;
                if(speed_req<0)   speed_req=0;
              }
              break;
    case 'r':                                         // adjust speed slowly 
              if(cmd=='+')  
              {
                speed_req += 5;
                if(speed_req>60)   speed_req=60;
              }
              if(cmd=='-')    
              {
                speed_req -= 5;
                if(speed_req<0)   speed_req=0;
              }
              break;
    case 'a':                                        // adjust direction
              if(cmd=='+')
              {
                digitalWrite(InA1, LOW);
                digitalWrite(InB1, HIGH);
                pos=2;
              }
              if(cmd=='-')   
              {
                digitalWrite(InA1, HIGH);
                digitalWrite(InB1, LOW);
                pos=1;
              }
              break;
    case 'o':                                        // type "oo" to stop motor
              digitalWrite(InA1, LOW);
              digitalWrite(InB1, LOW);
              speed_req = 0;
              pos=0;
              break;
    case 'p':                                       //adjust proportional gain
              if(cmd=='+')
              {
                Kp += 0.1;
              }
              if(cmd=='-')   
              {
                Kp -= 0.1;
              }
              break;
    case 'd':                                     //adjust derivative gain
              if(cmd=='+')
              {
                Kd += 0.1;
              }
              if(cmd=='-')   
              {
                Kd -= 0.1;
              }
              break;
    default: 
              Serial.println("ERROR");
  }
}*/

void lpid()
{
  ldif=lsetrpm-lrpm;
  //Serial.println(dif);
  //lid+=ldif;
  //ldd=ldif-lprev;
  //lprev=ldif;
  lsp=lpre_ocr+((255.0/400.0)*(ldif));
  lpre_ocr=lsp;
  if(lsp>255)
    lsp=255;
  if(lsp<0)
    lsp=0;
}

void rpid()
{
  rdif=rsetrpm-rrpm;
  //Serial.println(rdif);
  //rid+=rdif;
  //rdd=rdif-rprev;
  //rprev=rdif;
  rsp=rpre_ocr+((255.0/400.0)*(rdif));
  rpre_ocr=rsp;
  if(rsp>255)
    rsp=255;
  if(rsp<0)
    rsp=0;
}

void lm()
{
  analogWrite(lme,lsp);
  digitalWrite(lm1,HIGH);
  digitalWrite(lm2,LOW);
}

void rm()
{
  analogWrite(rme,rsp);
  digitalWrite(rm1,HIGH);
  digitalWrite(rm2,LOW);
}

void halt()
{
  digitalWrite(lm1,LOW);
  digitalWrite(lm2,LOW);
  digitalWrite(rm1,LOW);
  digitalWrite(rm2,LOW);
}

ISR(INT0_vect)
{
  rctr++;
  rpulse++;
}

ISR(INT1_vect)
{
  lctr++;
  lpulse++;
}
/*
void lisr()
{
  lctr++;
  lpulse++;
  Serial.print(lctr);
  Serial.print(" ");
}

void risr()
{
  rctr++;
  rpulse++;
  Serial.print(lctr);
  Serial.print(" ");
}
*/ 
void setup() 
{
  Serial.begin(9600);
  sei();   
  EIMSK|=(1<<INT0)|(1<<INT1);   
  EICRA|=(1<<ISC01)|(1<<ISC00)|(1<<ISC11)|(1<<ISC10);  
  pinMode (lm1, OUTPUT);
  pinMode (lm2, OUTPUT);
  pinMode (lme, OUTPUT);
  pinMode (rm1, OUTPUT);
  pinMode (rm2, OUTPUT);
  pinMode (rme, OUTPUT);
  //attachInterrupt(digitalPinToInterrupt(3), lisr , RISING);
  //attachInterrupt(digitalPinToInterrupt(2), risr , RISING);
  pinMode (2,INPUT);
  pinMode (3,INPUT);
}
void loop() 
{
  if(millis() - start > 200)
  {
    lrot=lctr/30.0;
    rrot=rctr/30.0;
    t=millis()-start;
    start=millis();
    lrpm=((lpulse/30.0)*60.0*1000.0)/t;
    rrpm=((rpulse/30.0)*60.0*1000.0)/t;
    lsetrpm=20;
    rsetrpm=30;
    lpid();
    rpid();
    lpulse=0;
    rpulse=0;
    /*
    Serial.print(lrot);Serial.print(" ");
    Serial.print(rrot);Serial.print(" ");*/
    Serial.print(lrpm);Serial.print(" ");
    Serial.println(rrpm);
  }
  lm();  
  rm();  
}
