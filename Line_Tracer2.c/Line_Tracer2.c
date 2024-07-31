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

using namespace cv;
using namespace std;

// Image Processing
#define IMG_Width 640
#define IMG_Height 480
#define ASSIST_BASE_LINE 320
#define ASSIST_BASE_WIDTH 30

int guide_width1 = 50;
int guide_height1 = 20;
int guide_center = IMG_Width / 2;
int line_center = -1;

// GPIO Motor Control
#define ENA 6  //Physerial 
#define IN1 4  //Physerial 
#define IN2 5  //Physerial 

#define ENB 0  //Physerial 
#define IN3 2  //Physerial 
#define IN4 3  //Physerial 

#define MAX_PWM_DUTY 100
#define PWM_BASE 20
#define P_gain 0.5

// Ultrasonic Sensor
#define TRIG 21
#define ECHO 22

// Serial Communication
#define baud_rate 115200

void sig_Handler(int sig);

int getch(void) {
    int ch;
    struct termios buf;
    struct termios save;

    tcgetattr(0, &save);
    buf = save;
    buf.c_lflag &= ~(ICANON | ECHO);
    buf.c_cc[VMIN] = 1;
    buf.c_cc[VTIME] = 0;
    tcsetattr(0, TCSAFLUSH, &buf);
    ch = getchar();
    tcsetattr(0, TCSAFLUSH, &save);
    return ch;
}

int GPIO_control_setup(void) {
    if (wiringPiSetup() == -1) {
        printf("wiringPi Setup error!\n");
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

void motor_control_r(int pwm) {
    if (pwm > 0) {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        softPwmWrite(ENA, pwm);
    } else if (pwm == 0) {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        softPwmWrite(ENA, 0);
    } else {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        softPwmWrite(ENA, -pwm);
    }
}

void motor_control_l(int pwm) {
    if (pwm > 0) {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        softPwmWrite(ENB, pwm);
    } else if (pwm == 0) {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);
        softPwmWrite(ENB, 0);
    } else {
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        softPwmWrite(ENB, -pwm);
    }
}

float ultrasonic_sensor(void) {
    long start_time, end_time;
    long temp_time1, temp_time2;
    int duration;
    float distance;

    digitalWrite(TRIG, LOW);
    delayMicroseconds(5);
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);

    delayMicroseconds(200); 

    temp_time1 = micros();

    while (digitalRead(ECHO) == LOW) {
        temp_time2 = micros();
        duration = temp_time2 - temp_time1;
        if (duration > 1000) return -1;
    }

    start_time = micros();

    while (digitalRead(ECHO) == HIGH) {
        temp_time2 = micros();
        duration = temp_time2 - temp_time1;
        if (duration > 2000) return -1;
    }
    end_time = micros();

    duration = end_time - start_time;
    distance = duration / 58;

    return distance;
}

Mat region_of_interest(Mat img, Point *points) {
    Mat img_mask = Mat::zeros(img.rows, img.cols, CV_8UC1);

    const Point* pt[1] = { points };
    int npt[] = { 4 };

    fillPoly(img_mask, pt, npt, 1, Scalar(255), LINE_8);

    Mat masked_img;
    bitwise_and(img, img_mask, masked_img);

    return masked_img;
}

Mat Canny_Edge_Detection(Mat img) {
    Mat mat_blur_img, mat_canny_img;
    blur(img, mat_blur_img, Size(3, 3));
    Canny(mat_blur_img, mat_canny_img, 225, 280,3);

    return mat_canny_img;
}

Mat Draw_Guide_Line(Mat img) {
    Mat result_img;
    img.copyTo(result_img);

    rectangle(result_img, Point(50, ASSIST_BASE_LINE - ASSIST_BASE_WIDTH), Point(IMG_Width - 50, ASSIST_BASE_LINE + ASSIST_BASE_WIDTH), Scalar(0, 255, 0), 1, LINE_AA);

    line(result_img, Point(guide_center - guide_width1, ASSIST_BASE_LINE), Point(guide_center, ASSIST_BASE_LINE), Scalar(0, 255, 255), 1, 0);
    line(result_img, Point(guide_center, ASSIST_BASE_LINE), Point(guide_center + guide_width1, ASSIST_BASE_LINE), Scalar(0, 255, 255), 1, 0);

    line(result_img, Point(guide_center - guide_width1, ASSIST_BASE_LINE - guide_height1), Point(guide_center - guide_width1, ASSIST_BASE_LINE + guide_height1), Scalar(0, 255, 255), 1, 0);
    line(result_img, Point(guide_center + guide_width1, ASSIST_BASE_LINE - guide_height1), Point(guide_center + guide_width1, ASSIST_BASE_LINE + guide_height1), Scalar(0, 255, 255), 1, 0);

    line(result_img, Point(IMG_Width / 2, ASSIST_BASE_LINE - guide_height1 * 1), Point(IMG_Width / 2, ASSIST_BASE_LINE + guide_height1 * 1), Scalar(255, 255, 255), 2, 0);
    line(result_img, Point(line_center, ASSIST_BASE_LINE - guide_height1 *1.2), Point(line_center, ASSIST_BASE_LINE + guide_height1 * 1.2), Scalar(0, 0, 255), 2, 0);

    return result_img;
}

void line_tracer_motor_control(int line_center) {
    int pwm_r, pwm_l;
    int steer_error = 0;
    steer_error = line_center - IMG_Width / 2;

    pwm_r = PWM_BASE + steer_error * P_gain;
    pwm_l = PWM_BASE - steer_error * P_gain;

    motor_control_r(pwm_r);
    motor_control_l(pwm_l);
}

int main(void) {
    int fd;
    int pwm_r = 0;
    int pwm_l = 0;
    unsigned char test;

    // OpenCV variables
    int img_width, img_height;
    Point points[4];
    int no_label;
    int c_x, c_y;
    int c_x_sum = 0;

    img_width = 640;
    img_height = 480;
    c_x = c_y = 0;

    Mat mat_image_org_color_Overlay;
    Mat mat_image_org_color;
    Mat mat_image_org_gray;
    Mat mat_image_canny_edge;
    Mat mat_image_roi_canny_edge;
    Mat mat_image_line_image = Mat(IMG_Height, IMG_Width, CV_8UC1, Scalar(0));
    Mat img_labels, stats, centroids;

    Scalar GREEN(0, 255, 0);
    Scalar RED(0, 0, 255);
    Scalar BLUE(255, 0, 0);
    Scalar YELLOW(0, 255, 255);

    mat_image_org_color = imread("/home/pi/Team_aMAP/vlf /Line_Tracer/Line_Tracer2.c/line_1.jpg");
    if (mat_image_org_color.empty()) {
        cerr << "Error loading image \n";
        return -1;
    }
    mat_image_org_color.copyTo(mat_image_org_color_Overlay);

    img_width = mat_image_org_color.size().width;
    img_height = mat_image_org_color.size().height;

    namedWindow("Display window", WINDOW_NORMAL);
    resizeWindow("Display window", img_width, img_height);
    moveWindow("Display window", 10, 10);

    namedWindow("ROI Image window", WINDOW_NORMAL);
    resizeWindow("ROI Image window", img_width, img_height);
    moveWindow("ROI Image window", 700, 10);

    namedWindow("Line Image window", WINDOW_NORMAL);
    resizeWindow("Line Image window", img_width, img_height);
    moveWindow("Line Image window", 10, 500);

    if (GPIO_control_setup() == -1) {
        return -1;
    }

    signal(SIGINT, sig_Handler);

    points[0] = Point(0, ASSIST_BASE_LINE - ASSIST_BASE_WIDTH);
    points[1] = Point(0, ASSIST_BASE_LINE + ASSIST_BASE_WIDTH);
    points[2] = Point(IMG_Width, ASSIST_BASE_LINE + ASSIST_BASE_WIDTH);
    points[3] = Point(IMG_Width, ASSIST_BASE_LINE - ASSIST_BASE_WIDTH);

    while (1) {
        cvtColor(mat_image_org_color, mat_image_org_gray, COLOR_RGB2GRAY);
        threshold(mat_image_org_gray, mat_image_canny_edge, 200, 255, THRESH_BINARY);
        mat_image_canny_edge = Canny_Edge_Detection(mat_image_org_gray);

        mat_image_roi_canny_edge =  region_of_interest(mat_image_canny_edge, points);

        vector<Vec4i> linesP;
        HoughLinesP(mat_image_roi_canny_edge, linesP, 1, CV_PI/192,30,30 ,40);
        printf("Line Number: %3d\n", linesP.size());  
        for (int i = 0; i < linesP.size(); i++) {
            Vec4i L = linesP[i];
            line(mat_image_line_image, Point(L[0], L[1]), Point(L[2], L[3]), Scalar(255), 3, LINE_AA);
            //printf("L: [%3d,%3d], [%3d,%3d]\n", L[0], L[1], L[2], L[3]);  
        }

        no_label = connectedComponentsWithStats(mat_image_line_image, img_labels, stats, centroids, 8, CV_32S);
        printf("no label: %3d\n", no_label);  
        c_x_sum = 0; 
        for (int i = 1; i < no_label; i++) {
            int area = stats.at<int>(i, CC_STAT_AREA);
            c_x = centroids.at<double>(i, 1);
            c_y = centroids.at<double>(i, 1);
            //printf("centroid [%4d %3d, %3d]\n", area, c_x, c_y);  
            c_x_sum += c_x;
        }
        line_center = (no_label > 1) ? c_x_sum / (no_label - 1) : IMG_Width / 2; 
        printf("Centroid Center: %3d\n", line_center);  
        printf("\n\n\n");

        line_tracer_motor_control(line_center);

        mat_image_org_color_Overlay = Draw_Guide_Line(mat_image_org_color);

        imshow("Display window", mat_image_org_color_Overlay);
        imshow("ROI Image window", mat_image_roi_canny_edge);
        imshow("Line Image window", mat_image_line_image);

        if (waitKey(10) > 0) break;
    }

    return 0;
}

void sig_Handler(int sig) {
    printf("Caught signal %d\n", sig);
    exit(0);
}
