#include "src/AX12A.h"
#define DirectionPin   (10u)
#define BaudRate      (1000000ul)
#define JOINT_1    5u
#define JOINT_2    3u
#define JOINT_3    4u
#define speed1    50
#define L1 0.094
#define L2 0.11
#define L1_OFFSET 204
#define L2_OFFSET 512
#define RAD2DEG 57.2957
#define DEG2DIGITAL 3.4133
#define DISCRETIZARE 10U
#define DISCRETIZARE_MICA 3U

struct postura{
  int q1;
  int q2;
};

void setup() {
  // put your setup code here, to run once
ax12a.begin(BaudRate, DirectionPin, &Serial);
}

void loop() 
{

//  DrawCh(0.02, 0.17, 'T');
//  DrawCh(0.05, 0.17, 'V');

  delay(1500);
  goToPoint(0.02, 0.17);
  delay(1750);

  DrawTable(0.02,0.17);

  pencilUp();
  delay(150);
  goToPoint(0.023,0.167);
  delay(150);

  
  DrawX(0.023,0.167);
  Draw0(0.049,0.167);

  DrawX(0.023,0.154);
  Draw0(0.023,0.141);
//
  DrawX(0.036,0.154);
  Draw0(0.049,0.141);
//
  DrawX(0.049,0.154);

  pencilUp();
  drawLine(0.017,0.151,0.063,0.151);
  drawLine(0.063,0.151,0.017,0.151);
  drawLine(0.017,0.151,0.063,0.151);
  //DrawCh(0.03, 0.17, 'P');
  // put your main code here, to run repeatedly:
  //ax12a.ledStatus(3,ON);
  //pencilUp();
  //pencilDown();
  //goToZero();
  //delay(3000);
  //goToPoint(0.1,0.11);
  //delay(3000);
  //pencilDown();
  //delay(3000);
  //goToPoint(0.1,0.20);
  //delay(3000);
  //drawLine(0.03,0.14,0.08,0.14);
  //delay(3000);
  //pencilUp();
  //drawLine(0.055,0.16,0.055,0.14);
  //delay(3000);
  //drawLine(0.065,0.18, 0.065, 0.16);
  //delay(3000);
  //goToPoint(0.03,0.15);
  //delay(3000);
  //pencilDown();
  //goToPoint(0.06,0.15);
  //delay(3000);

  //pencilUp();
  //goToPoint(0.045,0.15);  
  //delay(3000);
  //pencilDown();
  //goToPoint(0.045,0.14);
  //delay(3000);

}

//void postura1(){
  //ax12a.moveSpeed(joint1,412,50);
  //ax12a.moveSpeed(joint2,412,50); 
  //ax12a.move(joint3,673,speed1);
//}
//
//void postura2(){
//  ax12a.moveSpeed(joint1,612,50);
//  ax12a.moveSpeed(joint2,612,50); 
//  //ax12a.moveSpeed(joint3,673,speed1);
//}

postura getPosture (double xe, double ye){
  double L= sqrt(xe*xe + ye*ye);
  postura tempPost;
  tempPost.q1 = (int)((atan(ye/ xe) - acos((L2*L2 - L*L - L1* L1) / ((-2) * L1* L)))*RAD2DEG * DEG2DIGITAL);
  tempPost.q2 = (int)((acos( (L*L - L1*L1 - L2* L2) / (2 * L1* L2)))*RAD2DEG * DEG2DIGITAL);
  tempPost.q1 += L1_OFFSET;
  tempPost.q2 += L2_OFFSET;
  if(tempPost.q1 < 0 || tempPost.q2 < 0)
    {
      tempPost.q1 = -tempPost.q1;
      tempPost.q2 = -tempPost.q2;
    }
   if(tempPost.q1 > 1023 || tempPost.q2 >1023)
    {
      tempPost.q1 = -tempPost.q1;
      tempPost.q2 = -tempPost.q2;
    }
  return  tempPost;
}

void drawLine(double xs, double ys, double xf, double yf ){
  double dx = xf - xs;
  double pasX = dx/(double)DISCRETIZARE;
  double dy = yf - ys;
  double pasY = dy/(double)DISCRETIZARE;

  goToPoint(xs, ys);
  delay(100);
  pencilDown();
  for(int i = 0;i<=DISCRETIZARE; i++)
  {
      goToPoint(xs+pasX*i, ys + pasY*i);
      delay(50);
  }
}

void drawSmallLine(double xs, double ys, double xf, double yf ){
  double dx = xf - xs;
  double pasX = dx/(double)DISCRETIZARE_MICA;
  double dy = yf - ys;
  double pasY = dy/(double)DISCRETIZARE_MICA;

  goToPoint(xs, ys);
  delay(100);
  pencilDown();
  for(int i = 0;i<=DISCRETIZARE_MICA; i++)
  {
      goToPoint(xs+pasX*i, ys + pasY*i);
      delay(50);
  }
}

void goToZero(){
  goToPoint(0.,0.);
}

void goToPoint(double xe, double ye)
{
  postura post1 = getPosture(xe, ye);
  postura postCurr;
  ax12a.moveSpeed(JOINT_1, post1.q1, 150);
  ax12a.moveSpeed(JOINT_2, post1.q2, 150);
  /*do{
    postCurr = readCurrentPosture();
  }while(!isPostureReach(postCurr.q1,postCurr.q2,post1.q1,post1.q2));*/
}

postura readCurrentPosture(){
  postura tempPost;
  tempPost.q1 = ax12a.readPosition(JOINT_1);
  tempPost.q2 = ax12a.readPosition(JOINT_2);
  return tempPost;
}

void pencilUp(){
  ax12a.moveSpeed(JOINT_3, 440, 100);
  delay(1000);
}

void pencilDown(){
  ax12a.moveSpeed(JOINT_3, 490, 100);
  delay(500);
}
bool isPostureReach(int currq1, int currq2, int finishq1, int finishq2){
  bool result = false;
  if(currq1 == finishq1 || currq2== finishq2)
    result=true;
  return result;
}

void DrawCh(double x, double y, char ch)
{
 if(ch=='T')
 {
   pencilUp();
   drawLine(x,y,x+0.03,y);
   delay(500);
   pencilUp();
   drawLine(x+0.015,y,x+0.015,y-0.04);
   delay(300);
 } 

  if(ch=='V')
 {
   pencilUp();
   drawLine(x,y,x+0.01,y-0.03);
   delay(50);
   //pencilUp();
   drawLine(x+0.01,y-0.03,x+0.02,y);
   delay(300);
 } 

 if(ch=='P')
 {
   pencilUp();
   drawLine(x,y,x+0.03,y);
   drawLine(x+0.03,y, x+0.03, y-0.03);
   drawLine(x+0.03,y-0.03, x, y-0.03);
   drawLine(x,y-0.03, x, y);
   
   delay(300);
 } 
}
void DrawTable(double x, double y)
{
  //linie sus
   pencilUp();
   drawLine(x,y,x+0.04,y);
   delay(150);
   //linie dreapta
   drawLine(x+0.04,y,x+0.04,y-0.04);
   delay(150);
   //linie jos
   drawLine(x+0.04,y-0.04,x,y-0.04);
   delay(150);
   //linie stanga
   drawLine(x,y-0.04,x,y);
   delay(150);
   
   pencilUp();
   drawLine(x+0.013,y,x+0.013,y-0.04);
   delay(150);
   
   pencilUp();
   goToPoint(x+0.026,y);
   delay(75);
   drawLine(x+0.026,y,x+0.026,y-0.04);
   delay(150);
   
   pencilUp();
   drawLine(x,y-0.013,x+0.04,y-0.013);
   delay(150);
   
   pencilUp();
   drawLine(x,y-0.026,x+0.04,y-0.026);
   delay(150);
}

void DrawX(double x, double y){
   
   pencilUp();
   drawSmallLine(x,y,x+0.007,y-0.007);
   delay(100);
   pencilUp();
   drawSmallLine(x+0.007,y,x,y-0.007);
   delay(100);
}
void Draw0(double x,double y){
  //linie sus
   pencilUp();
   drawSmallLine(x,y,x+0.007,y);
   delay(10);
   //linie dreapta
   drawSmallLine(x+0.007,y,x+0.007,y-0.007);
   delay(10);
   //linie jos
   drawSmallLine(x+0.007,y-0.007,x,y-0.007);
   delay(10);
   //linie stanga
   drawSmallLine(x,y-0.007,x,y);
   delay(10);
}
