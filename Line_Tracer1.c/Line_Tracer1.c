#include <stdio.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <termio.h>
#include <softPwm.h>
#include <opencv2/opencv.hpp>
#include <iostream>

//////////////// Image Processing //////////////
using namespace cv;
using namespace std; 

#define IMG_Width     640
#define IMG_Height    480

#define ASSIST_BASE_LINE 320
#define ASSIST_BASE_WIDTH 60

int guide_width1 = 50;
int guide_height1 = 20;
int guide_center = IMG_Width/2;    

//////////////// GPIO MOTOR CONTROL //////////////
#define ENA 6  //Physerial 
#define IN1 4  //Physerial 
#define IN2 5  //Physerial 

#define ENB 0  //Physerial 
#define IN3 2  //Physerial 
#define IN4 3  //Physerial 


#define MAX_PWM_DUTY 100

/////////// Ultrasonic Sensor ////////

#define TRIG 21
#define ECHO 22

/////////// Serial Com. //////////
#define baud_rate 115200

void sig_Handler(int sig);

int getch(void)
{

int ch;
struct termios buf;
struct termios save;

tcgetattr(0, &save);
buf = save;
buf.c_lflag &= ~(ICANON|ECHO);
buf.c_cc[VMIN] = 1;
buf.c_cc[VTIME] = 0;
tcsetattr(0, TCSAFLUSH, &buf);
ch = getchar();
tcsetattr(0, TCSAFLUSH, &save);
return ch;
}


int GPIO_control_setup(void)
{
   if(wiringPiSetup() == -1)
   {
      printf("wiringPi Setup error !\n");
      return -1;
   }
   
   pinMode(ENA, OUTPUT);
   pinMode(IN1, OUTPUT);
   pinMode(IN2, OUTPUT);
   
   pinMode(ENB, OUTPUT);
   pinMode(IN3, OUTPUT);
   pinMode(IN4, OUTPUT);
   
   pinMode(TRIG, OUTPUT);
   pinMode(ECHO, INPUT);
   
   softPwmCreate(ENA, 1, MAX_PWM_DUTY);
   softPwmCreate(ENB, 1, MAX_PWM_DUTY);
   
   softPwmWrite(ENA, 0);
   softPwmWrite(ENB, 0);
   return 0;
}

void motor_control_r(int pwm)
{
   
   if(pwm>0)
   {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(ENA, pwm);
   }
   
   else if(pwm==0)
   {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      digitalWrite(ENA, 0);
   }
   
   else
   {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      softPwmWrite(ENA, -pwm);
   }
   
}

void motor_control_l(int pwm)
{
   
   if(pwm>0)
   {
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      digitalWrite(ENB, pwm);
   }
   
   else if(pwm==0)
   {
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      digitalWrite(ENB, 0);
   }
   
   else
   {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      softPwmWrite(ENA, -pwm);
   }
   
}

float ultrasonic_sensor(void)
{
   long start_time, end_time;
   long temp_time1, temp_time2;
   int duration;
   float distance;
   
   digitalWrite(TRIG, LOW);
   delayMicroseconds(5);
   digitalWrite(TRIG, HIGH);
   delayMicroseconds(10);
   digitalWrite(TRIG, LOW);
   
   delayMicroseconds(200);    // wait for burst signal. 40kHz x 8 = 8x25us = 200
   
   //printf("200msec \n");
   temp_time1 = micros();
   
   
   while(digitalRead(ECHO) == LOW)   // wait untiol ECHO pin is HIGH
   {
      temp_time2 = micros();
      duration = temp_time2 - temp_time1;
      if(duration>1000) return -1;
   }
   
  
   
   start_time = micros();
   
   //printf("echo signal high \n");

   while(digitalRead(ECHO) == HIGH)   // wait untiol ECHO pin is LOW
   {
      temp_time2 = micros();
      duration = temp_time2 - temp_time1;
      if(duration > 2000) return -1;
   }
   end_time = micros();
   
   //printf("echo signal low \n");
   
   duration = end_time - start_time;
   
   distance = duration / 58;

   
   return distance;
}


Mat Canny_Edge_Detection(Mat img)
{
  Mat mat_blur_img, mat_canny_img;
  blur(img, mat_blur_img, Size(3,3));  
  Canny(mat_blur_img, mat_canny_img, 225, 280,3);
   
  return mat_canny_img;  
}

Mat Draw_Guide_Line(Mat img)
{
	Mat result_img;
	img.copyTo(result_img);
	rectangle(result_img, Point(50, ASSIST_BASE_LINE - ASSIST_BASE_WIDTH), Point(IMG_Width-50, ASSIST_BASE_LINE + ASSIST_BASE_WIDTH),Scalar(0,255,0), 1, LINE_AA);
		 
	line(result_img, Point(guide_center-guide_width1,ASSIST_BASE_LINE), Point(guide_center,ASSIST_BASE_LINE), Scalar(0,255,255),1,0);
	line(result_img, Point(guide_center,ASSIST_BASE_LINE), Point(guide_center+guide_width1,ASSIST_BASE_LINE), Scalar(0,255,255),1,0);
		
	line(result_img, Point(guide_center-guide_width1,ASSIST_BASE_LINE - guide_height1), Point(guide_center-guide_width1,ASSIST_BASE_LINE + guide_height1), Scalar(0,255,255),1,0);
	line(result_img, Point(guide_center+guide_width1,ASSIST_BASE_LINE - guide_height1), Point(guide_center+guide_width1,ASSIST_BASE_LINE + guide_height1), Scalar(0,255,255),1,0);
		
	line(img, Point(IMG_Width/2, ASSIST_BASE_LINE - guide_height1*1), Point(IMG_Width/2, ASSIST_BASE_LINE + guide_height1*1), Scalar(255,255,255), 2,0);
	 
	return result_img;
}

int main(void)
{
   
   int fd;
   int pwm_r =0;
   int pwm_l =0;
   unsigned char test;
   
   ///////////////////////////////////////////////////////////////////////////////////////////////
   //////////////////////////////////////OpenCV 변수 선언///////////////////////////////////////////
   int img_width, img_height;
   
   img_width = 640;
   img_height = 480;
   
    
	  
   Mat mat_image_org_color;
   Mat mat_image_org_color_Overlay;
   Mat mat_image_org_gray;
   Mat mat_image_org_gary_result;
   Mat mat_image_canny_edge;
   Mat mat_image_line_image = Mat(IMG_Height, IMG_Width, CV_8UC1, Scalar(0));
   Mat image;

   Scalar GREEN(0,255,0);
   Scalar RED(0,0,255);
   Scalar BLUE(255,0,0);
   Scalar YELLOW(0,255,255);
   //////////////////////////////////////////////////////////////////////////////////////
    
    mat_image_org_color = imread("/home/pi/Team_aMAP/vlf /Line_Tracer/Line_Tracer1.c/line_1.jpg");
    mat_image_org_color.copyTo(mat_image_org_color_Overlay);
    
    if (mat_image_org_color.empty())
      {
        cerr << "Empty Video \n";
        return -1;
      }
    
    img_width = mat_image_org_color.size().width;
    img_height = mat_image_org_color.size().height;
    

    
 
    
    
    namedWindow("Display window", CV_WINDOW_NORMAL);
    resizeWindow("Display window", img_width,img_height);
    moveWindow("Display window", 10, 10);
    
    namedWindow("Gray Image window", CV_WINDOW_NORMAL);
    resizeWindow("Gray Image window", img_width,img_height);
    moveWindow("Gray Image window", 700, 10);
    
    namedWindow("Canny Edge Image window", CV_WINDOW_NORMAL);
    resizeWindow("Canny Edge Image window", img_width,img_height);
    moveWindow("Canny Edge Image window", 10, 500);
   
   
   
   if(GPIO_control_setup() == -1)
   {
      
      return -1;
   }

   signal(SIGINT, sig_Handler);
   test = 'B';
   
   while(1)
   {
	  cvtColor(mat_image_org_color, mat_image_org_gray, CV_RGB2GRAY );               // color to gray conversion
      threshold(mat_image_org_gray, mat_image_canny_edge, 200, 255, THRESH_BINARY);
      mat_image_canny_edge = Canny_Edge_Detection(mat_image_org_gray); 
      
      mat_image_org_color_Overlay = Draw_Guide_Line(mat_image_org_color);
      
      vector<Vec4i> linesP;
      HoughLinesP(mat_image_canny_edge, linesP, 1, CV_PI/192,65,65,50);
      printf("Line Number : %3d\n", linesP.size());
      for(int i=0; i<linesP.size(); i++)
      {
          Vec4i L=linesP[i];
          //int cx1 = linesP[i][0];
          //int cy1 = linesP[i][1];
          //int cx2 = linesP[i][2];
          //int cy2 = linesP[i][3];
          
          line(mat_image_line_image, Point(L[0], L[1]), Point(L[2], L[3]), Scalar(255), 3, LINE_AA);
          
          line(mat_image_org_color_Overlay, Point(L[0], L[1]), Point(L[2], L[3]), Scalar(0,0,255), 3, LINE_AA);
          printf("L :[%3d,%3d], [%3d,%3d] \n", L[0],L[1],L[2],L[3]);
          //printf("H :[%3d,%3d], [%3d,%3d] \n", cx1,cy1,cx2,cy2);

      }
          printf("\n\n\n");      
      
      
      
      imshow("Display window", mat_image_org_color_Overlay);
      //imshow("Display window", mat_image_org_color);
      //imshow("Gray Image window", mat_image_org_gray);
      imshow("Gray Image window", mat_image_line_image);

      imshow("Canny Edge Image window", mat_image_canny_edge);    
      if(waitKey(10) > 0)
      break;     
   }
   
   return 0;
}
void sig_Handler(int sig)
{
   printf("\n\n\n\nProgram and Motor Stop\n\n\n");
   motor_control_r(0);
   motor_control_l(0);
   exit(0);
}
