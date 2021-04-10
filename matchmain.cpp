#include<iostream>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/core/core.hpp>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
using namespace std;
using namespace cv;
#define COM_NAME "/dev/ttyUSB0"
int count = 0;
int set_opt(int,int,int,char,int);
Point2f gc[1];
float gr[1];
int cc;
int ss;
char buff[1];
int counter = 0;


int print_px_value(Mat& im)
{
    int counter = 0;
    int rowNumber = im.rows;  //行数
    int colNumber = im.cols * im.channels();  //列数 x 通道数=每一行元素的个数

    //双重循环，遍历所有的像素值
    for (int i = 0; i < rowNumber; i++)  //行循环
    {
        uchar* data = im.ptr<uchar>(i);  //获取第i行的首地址
        for (int j = 0; j < colNumber; j++)   //列循环
        {
            //data[j] = data[j] / div * div + div / 2;
            //cout << (int)data[j] << endl;
            if( data[j] == 255) counter += 1;

        }  //行处理结束
    }
    //cout<<counter<<endl;
    return counter;//white
}

int jugment_first()
{
    VideoCapture Capture(2);
    Mat src, hsv, ROI;
    int tip;
    while(tip != 1)
    {
        Capture >> src;

        int w = src.cols;
        int h = src.rows;
        cvtColor(src, hsv, COLOR_BGR2HSV);

        cv::Mat combine = cv::Mat::zeros(w, h, src.type());
        Mat ROI = src(Range((src.rows / 5)*2, (src.rows / 5)*3), Range((src.cols/5)*2, (src.cols/5)*3));//起始位ROI区域

        //红灯HSV参数， 两组， 实测择优使用
        //inRange(roired, Scalar(0, 160, 254), Scalar(180, 192, 255), roired);//red light
        //起始位 红灯像素点>120 stop
        Mat roired;
        inRange(ROI, Scalar(31, 128, 100), Scalar(58, 167, 158), roired);//未测 min-pass
        //cout<<"count:"<<print_px_value(roired)<<endl;


        //inRange(, Scalar(24, 67, 254), Scalar(73, 120, 255), ) //同时筛黄和绿
        //绿灯+闪黄灯-white > 200（190）共三次
        //红灯<100
        //黄or绿 > 100
        //取决于ROI大小和位置，需实测
        //green and yellow 90~110


        if(print_px_value(roired) < 200)//
        {
            tip = 1;
            cout<<"pass"<<endl;
            break;
        }
    }
    return 0;
}




int jugment_second()
{
    //实测
}




int jugment_RED()         //未知错误需修改
{
    VideoCapture capture (2);
    Mat src, hsv, dstg;
    int counter = 0;
    while(counter =! 1)
    {

        capture >> src;

        cvtColor(src, hsv, COLOR_BGR2HSV);
        //inRange(hsv, Scalar(55, 184, 170), Scalar(89, 245, 255), dstg);//in
        inRange(hsv, Scalar(55, 183, 50), Scalar(70, 255, 170), dstg);//out

        int pc[1];
        vector<Vec3f>circles;
        HoughCircles(dstg, circles, HOUGH_GRADIENT, 1.55, 15, 70, 60, 10, 40);
        for (size_t i = 0; i < circles.size(); i++)
        {
            Vec3f c = circles[i];
            //yuan
            //circle(src, Point(c[0], c[1]), c[2], Scalar(77, 255, 255), 2, LINE_AA);
            circle(src, Point(c[0], c[1]), c[2], Scalar(77, 255, 255), 2, LINE_AA);
            Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);
            pc[0] = radius;

        }
        cout<<pc[0]<<endl;

        if(pc[0] > 0 && pc[0] < 100)
        {
            cout<<"1"<<endl;
        }
        else cout<<"0"<<endl;
//        imshow("hsv", dstg);
//        imshow("capture",src);
//        waitKey(1);
        if(pc[0] > 0 && pc[0] < 100)
        {
            counter++;
        }

    }
    imshow("hsv", dstg);
    imshow("capture",src);
    waitKey(1);
    return jugment_RED();//返回值使用方式
}
int jugment_GREEN()
{
    VideoCapture capture (2);
    Mat src, hsv, dstg;
    int counter = 0;
    while(counter < 2000)
    {
        counter++;

        capture >> src;

        cvtColor(src, hsv, COLOR_BGR2HSV);
        //inRange(hsv, Scalar(55, 184, 170), Scalar(89, 245, 255), dstg);//in
        inRange(hsv, Scalar(55, 183, 50), Scalar(70, 255, 170), dstg);//out

        int pc[1];
        vector<Vec3f>circles;
        HoughCircles(dstg, circles, HOUGH_GRADIENT, 1.55, 15, 70, 60, 10, 40);
        for (size_t i = 0; i < circles.size(); i++)
        {
            Vec3f c = circles[i];
            //yuan
            //circle(src, Point(c[0], c[1]), c[2], Scalar(77, 255, 255), 2, LINE_AA);
            circle(src, Point(c[0], c[1]), c[2], Scalar(77, 255, 255), 2, LINE_AA);
            Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);
            pc[0] = radius;

        }
        cout<<pc[0]<<endl;
        if(pc[0] > 0 && pc[0] < 100)
        {
            cout<<"1"<<endl;
        }
        else cout<<"0"<<endl;
        imshow("hsv", dstg);
        imshow("capture",src);
        char(key)=(char)waitKey(1);
        if(key==27)
            break;
    }
    return jugment_GREEN();//返回值使用方式
}

int main()
{
    //read( )
    //if( != )
    //{
    //    return read()
    //}


    int fd;
    char len;
    char buffer1[512];
    char buffer2[512];
     unsigned char buff1[1];
      unsigned char buff2[1];
    //buff[0] = 0x10;
    buff1[0] = '8';//go
    buff2[0] = '9';//stop
    char *uart_out = "Please input,waiting\n";
    int x,y;
    fd = open(COM_NAME, O_RDWR | O_NOCTTY |O_NDELAY);
    if(fd < 0){
        perror(COM_NAME);
                cout<<"!!!"<<endl;
                return -1;
            }
            memset(buffer1,0,sizeof(buffer1));
            memset(buffer2,0,sizeof(buffer2));
            set_opt(fd, 115200, 8, 'N', 1);
            //buff[0] = '9';
            //write(fd, buff, sizeof(buffer));
            jugment_GREEN();
            jugment_RED();

            for(int n = 0; n < 1; n++)
            {
                if(jugment_GREEN() == 1 && jugment_RED() == 1)//cc = 1, ss = 1
                {
                    write(fd, buff1, sizeof(buffer1));
                    cout<<buff1<<endl;
                }
                else if(jugment_GREEN() == 0 && jugment_RED() == 0)//cc = 0, ss = 0
                {
                    write(fd, buff1, sizeof(buffer1));
                    cout<<buff1<<endl;
                }
                else if(jugment_GREEN() == 1 && jugment_RED() == 0)//cc = 0, ss = 0
                {
                    write(fd, buff1, sizeof(buffer1));
                    cout<<buff1<<endl;
                }
                else if(jugment_GREEN() == 0 && jugment_RED() == 1)//cc = 0, ss = 0
                {
                    write(fd, buff2, sizeof(buffer2));
                    cout<<buff2<<endl;
                }
            }

            while(1){
        //             while((len = read(fd, buffer, 512))>0){
        //            buffer[len+1] = '\0';
        //              write(fd,buffer,strlen(buffer));
                      write(fd,buff1,sizeof(buff1));
                      x=sizeof(buff1);
                    len=read(fd,buffer1,sizeof(buffer1));
                    y=strlen(buffer1);
                    memset(buffer1,0,strlen(buffer1));
                    len = 0;

                    write(fd,buff2,sizeof(buff2));
                    x=sizeof(buff2);
                  len=read(fd,buffer2,sizeof(buffer2));
                  y=strlen(buffer2);
                  memset(buffer2,0,strlen(buffer2));
                  len = 0;
            }
            return 0;
}










int set_opt(int fd,int comspeed,int comBits,char comEvent,int comStop)
{
        struct termios newtio,oldtio;
        if( tcgetattr(fd,&oldtio) !=0){
           perror("SetupSerial 1");
           return -1;
        }
        bzero(&newtio, sizeof(newtio));
        newtio.c_cflag |= CLOCAL | CREAD;
        newtio.c_cflag &= ~CSIZE;

        switch(comBits){
        case 7:
            newtio.c_cflag |= CS7;
            break;
        case 8:
            newtio.c_cflag |= CS8;
            break;
        }

        switch (comEvent) {
        case 'N':
            newtio.c_cflag &= PARENB;
                   break;
               }
               switch (comspeed) {
               case 115200:
                   cfsetispeed(&newtio, B115200);
                   cfsetospeed(&newtio, B115200);
                   break;
               default:
                   cfsetispeed(&newtio, B9600);
                   cfsetospeed(&newtio, B9600);
                   break;
               }
                  if(comStop == 1){
                      newtio.c_cflag &= ~CSTOPB;
                      newtio.c_cc[VTIME] = 100;
                      newtio.c_cc[VMIN] = 0;
                      tcflush(fd,TCIFLUSH);
                  }
                  if((tcsetattr(fd,TCSANOW,&newtio))!=0){
                      perror("comset error");
                      return -1;
                  }else{
                      cout<<"set done!"<<endl;
                  }
                  return 0;
              }
