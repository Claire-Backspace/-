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


int jugment_start()
{
    VideoCapture capture (2);
    Mat src, hsv, thr, thred;
    Mat dst_green, dst_yellow;

    int w = src.cols;
    int h = src.rows;
    int green;
    int yellow;

    //cout<<"cols:"<<src.cols<<endl;//640
    //cout<<"rows:"<<src.rows<<endl;//360



    int n = 0;
    while(1)
    {
        n++;
        capture >> src;
        cv::Mat combine = cv::Mat::zeros(w, h, src.type());
        Mat ROI = src(Range((src.rows / 5)*2, (src.rows / 5)*3), Range((src.cols/5)*2, (src.cols/5)*3));//起始位ROI区域
        //640/5 = 128
        //360/5 = 72
        //128 * 72 = 9216

        cvtColor(ROI, ROI, COLOR_BGR2HSV);
        Mat roig, roiy;
        Mat roi = ROI.clone();
        inRange(roi, Scalar(60, 8, 188), Scalar(100, 255, 255), roig);
        //inRange(ROI, Scalar(17, 125, 132), Scalar(32, 245, 255), roiy);//outside
        inRange(roi, Scalar(60, 130, 193), Scalar(72, 255, 255), roiy);
        green = print_px_value(roig);//HSV二值化后白色（绿色）像素点个数
        yellow = print_px_value(roiy);//HSV二值化后白色（黄色）像素点计数
        //inRange(, Scalar(31, 128, 100), Scalar(58, 167, 158)//min pass//max red(yellow) to green//middle green(yellow) to red;


        print_px_value(roig);
        print_px_value(roiy);


        //213*120=25560
        //cout<<"count-green:"<<print_px_value(roig)<<endl;
        cout<<"count-yellow:"<<print_px_value(roiy)<<endl;
        imshow("src", src);
        imshow("roi", roiy);
        waitKey(1);

    }

    return (green + yellow);

}



int jugment1()
{
    VideoCapture capture (2);
    Mat src, hsv, thr, thred;
    Mat dst_green, dst_yellow;
    //while(counter < 6)
    //{ counter ++;
    while(counter < 1)
    {
        counter ++;

        capture >> src;
        //cvtColor(src, thr, COLOR_BGR2GRAY);
        cvtColor(src, hsv, COLOR_BGR2HSV);
        //inRange(hsv, Scalar(55, 184, 170), Scalar(89, 245, 255), dst_green);
        inRange(hsv, Scalar(55, 183, 50), Scalar(70, 255, 170), dst_green);//green2
        //inRange(hsv, Scalar(17, 125, 132), Scalar(32, 245, 255), dst_yellow);

        vector<Vec3f>circles;
        HoughCircles(dst_green, circles, HOUGH_GRADIENT, 1.55, 15, 100, 70, 10, 40);
        for (size_t i = 0; i < circles.size(); i++)
        {
            Vec3f c = circles[i];
            //yuan
            //circle(src, Point(c[0], c[1]), c[2], Scalar(77, 255, 255), 2, LINE_AA);
            circle(src, Point(c[0], c[1]), c[2], Scalar(77, 255, 255), 2, LINE_AA);
            Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);

//            while(i < 1)
//            {
                gc[i] = center;
                gr[i] = radius;
        }

        if(gr[0] == 0)
        {
            cc = 1;
            cout<<"cc:"<< cc <<endl;
        }
        else if(gr[0] != 0)
        {
            ss = 1;
            cout<<"ss:"<< ss <<endl;
        }
}
    imshow("src", src);
            waitKey(0);
//}
    return cc;
}


int jugment2()
{
    VideoCapture capture (2);
    Mat src, hsv, thr, thred;
    Mat dst_green, dst_yellow;
    int m = 0;
    //while(counter < 6)
    //{ counter ++;
    while(1)
    {
        counter ++;

        capture >> src;
        //cvtColor(src, thr, COLOR_BGR2GRAY);
        cvtColor(src, hsv, COLOR_BGR2HSV);
        //inRange(hsv, Scalar(55, 184, 170), Scalar(89, 245, 255), dst_green);
        inRange(hsv, Scalar(55, 183, 50), Scalar(70, 255, 170), dst_green);//green2
        inRange(hsv, Scalar(17, 125, 132), Scalar(32, 245, 255), dst_yellow);
        //threshold(thr, thred, 30, 255, 0);
        //threshold(thr, thred, 100, 255, 1);
        //blur(thred, thred, Size(15, 15));
        //imshow("thred", thred);


//        Point2f gc[1];
//        float gr[1];
//        Point2f yc[1];
//        float yr[1];
        vector<Vec3f>circles;
        HoughCircles(dst_green, circles, HOUGH_GRADIENT, 1.55, 15, 100, 20, 5, 17);
        for (size_t i = 0; i < circles.size(); i++)
        {
            Vec3f c = circles[i];
            //yuan
            //circle(src, Point(c[0], c[1]), c[2], Scalar(77, 255, 255), 2, LINE_AA);
            circle(src, Point(c[0], c[1]), c[2], Scalar(77, 255, 255), 2, LINE_AA);
            Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);

//            while(i < 1)
//            {
                gc[i] = center;
                gr[i] = radius;





//            }
        }
        imshow("src", src);
        waitKey(1);

        if(gr[0] == 0)
        {
            cc = 1;
            cout<<"cc:"<< cc <<endl;
        }
        else if(gr[0] != 0)
        {
            ss = 1;
            cout<<"ss:"<< ss <<endl;
        }
        cout<<"gr:"<< gr[0] <<endl;
        cout<<"ss:"<< ss <<endl;
        cout<<"cc:"<< cc <<endl;
//        if(gr[0] > 0)
//        {
//            ss = 1;
//        }
//        else if(gr[0] == 0)
//        {
//            cc = 1;
//        }

//        imshow("capture",src);
//        //imshow("hsv", tog);
//        char(key)=(char)waitKey(1);


//        if(key==27)
//            break;
                //return 0;
    }

    //return cc;



    //}
                return ss;
}


int jugment_a()
{


    VideoCapture capture (2);
    Mat src, hsv, thr, thred;
    Mat dst_green, dst_yellow;

    int w = src.cols;
    int h = src.rows;
    //cout<<"cols:"<<src.cols<<endl;//480
    //cout<<"rows:"<<src.rows<<endl;//360


    while(counter < 20)
    {
        capture >> src;

        cv::Mat combine = cv::Mat::zeros(w, h, src.type());
        Mat ROI = src(Range(src.rows / 3, (src.rows / 3)*2), Range(src.cols/3, (src.cols/3)*2));
        cvtColor(ROI, ROI, COLOR_BGR2HSV);
        inRange(ROI, Scalar(60, 8, 188), Scalar(100, 255, 255), ROI);

    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////

int shot( int &w)
{
    if(w % 2 ==0)
        cout<<"stop"<<endl;
    else if(w % 2 != 0)
        cout<<"go"<<endl;
}




int main()
{
    int WAY;
    Mat src;
    int w = src.cols;
    int h = src.rows;
    while(1)
    {
        VideoCapture capture (2);
        capture >> src;
        Mat HSV, dst;
        cvtColor(src, HSV, COLOR_BGR2HSV);
        cv::Mat combine = cv::Mat::zeros(w, h, gray.type());
        Mat ROI = HSV(Range(src.rows / 3, (src.rows / 3)*2), Range(src.cols/3, (src.cols/3)*2));
        Mat roired = ROI.clone();
        inRange(ROI, Scalar(24, 67, 254), Scalar(73, 120, 255), ROI);//yellow and green
        inRange(roired, Scalar(0, 160, 254), Scalar(180, 192, 255), roired);//red light
        imshow("roi", roired);
        imshow("dst", ROI);
        //print_px_value(dst);
        cout<<"count:"<<print_px_value(roired)<<endl;
        //起始位 红灯像素点>70 stop
        //green and yellow 90~110
        waitKey(1);
    }

//    int fd;
//    char len;
//    char buffer1[512];
//    char buffer2[512];
//     unsigned char buff1[1];
//      unsigned char buff2[1];
//    //buff[0] = 0x10;
//    buff1[0] = '8';//go
//    buff2[0] = '9';//stop


//    char *uart_out = "Please input,waiting\n";
//    int x,y;
//    fd = open(COM_NAME, O_RDWR | O_NOCTTY |O_NDELAY);
//    if(fd < 0){
//        perror(COM_NAME);
//                cout<<"!!!"<<endl;
//                return -1;
//            }
//            memset(buffer1,0,sizeof(buffer1));
//            memset(buffer2,0,sizeof(buffer2));
//            ////////////////////////set_opt(fd, 115200, 8, 'N', 1);
//            //buff[0] = '9';
//            //write(fd, buff, sizeof(buffer));
            //jugment1();
            //jugment2();

//            jugment_start();
//            if (jugment_start() > 370)
//            {
//                WAY++;
//            }

//            shot(WAY);

//            if (jugment_start() > 390)
//            {
//                WAY++;
//            }
//            shot(WAY);


            //jugment_start()




            //cout<<"jugment_star:"<<jugment_star()<<endl;
            //int n = 0;//
//            while(n < 1)
//            {
//                read(fd, buff,sizeof(buffer));

//                write(fd, buff, sizeof(buffer));
////                if(read != 0)
//                cout<<buff<<endl;
//                //return(fd, sign, sizeof(sign));
//                n++;
//            }
            //return(fd, buff, sizeof(buff))

        //    write(fd,uart_out, 1);


//            cout<<"jugment1"<<jugment1()<<endl;
//            cout<<"jugment2"<<jugment2()<<endl;
//            cout<<"cc:"<<cc<<endl;
//            cout<<"ss:"<<ss<<endl;
//            //while(){
//            for(int n = 0; n < 1; n++)
//            {
//                if(jugment1() == 1 && jugment2() == 1)//cc = 1, ss = 1
//                {
//                    write(fd, buff1, sizeof(buffer1));
//                    cout<<buff1<<endl;
//                }
//                else if(jugment1() == 0 && jugment2() == 0)//cc = 0, ss = 0
//                {
//                    write(fd, buff1, sizeof(buffer1));
//                    cout<<buff1<<endl;
//                }
//                else if(jugment1() == 1 && jugment2() == 0)//cc = 0, ss = 0
//                {
//                    write(fd, buff1, sizeof(buffer1));
//                    cout<<buff1<<endl;
//                }
//                else if(jugment1() == 0 && jugment2() == 1)//cc = 0, ss = 0
//                {
//                    write(fd, buff2, sizeof(buffer2));
//                    cout<<buff2<<endl;
//                }
//            }
            //}


//            while(1){
//        //             while((len = read(fd, buffer, 512))>0){
//        //            buffer[len+1] = '\0';
//        //              write(fd,buffer,strlen(buffer));
//                      write(fd,buff1,sizeof(buff1));
//                      x=sizeof(buff1);
//                    len=read(fd,buffer1,sizeof(buffer1));
//                    y=strlen(buffer1);
//                    memset(buffer1,0,strlen(buffer1));
//                    len = 0;

//                    write(fd,buff2,sizeof(buff2));
//                    x=sizeof(buff2);
//                  len=read(fd,buffer2,sizeof(buffer2));
//                  y=strlen(buffer2);
//                  memset(buffer2,0,strlen(buffer2));
//                  len = 0;
//////                    ssize_t read(int fd, void *buf, size_t count);
//////                    返回值：成功返回读取的字节数，出错返回-1并设置errno，如果在调read之前已到达文件末尾，则这次read返回0
////                    //ssize_t write(int fd, const void *buf, size_t count);
////                    //返回值：成功返回写入的字节数，出错返回-1并设置errno

//    return 0;
//}
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
