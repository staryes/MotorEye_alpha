#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

#include <stdio.h>      // standard input / output functions
#include <stdlib.h>
#include <string.h>     // string function definitions
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions


 int main( int argc, char** argv )
 {
    /* Open File Descriptor */
    int USB = open( "/dev/ttyACM0", O_RDWR| O_NONBLOCK | O_NDELAY );

    /* Error Handling */
    if ( USB < 0 )
    {
      //cout << "Error " << errno << " opening " << "/dev/ttyUSB0" << ": " << strerror (errno) << endl;
        perror("USB ");
    }

    /* *** Configure Port *** */
    struct termios tty;
    memset (&tty, 0, sizeof tty);

    /* Error Handling */
    if ( tcgetattr ( USB, &tty ) != 0 )
    {
      //cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << endl;
      perror("tcgerattr ");
    }
    
    /* Set Baud Rate */
    cfsetospeed (&tty, B115200);
    cfsetispeed (&tty, B115200);

    /* Setting other Port Stuff */
    tty.c_cflag     &=  ~PARENB;        // Make 8n1
    tty.c_cflag     &=  ~CSTOPB;
    tty.c_cflag     &=  ~CSIZE;
    tty.c_cflag     |=  CS8;
    tty.c_cflag     &=  ~CRTSCTS;       // no flow control
    tty.c_lflag     =   0;          // no signaling chars, no echo, no canonical processing
    tty.c_oflag     =   0;                  // no remapping, no delays
    tty.c_cc[VMIN]      =   0;                  // read doesn't block
    tty.c_cc[VTIME]     =   5;                  // 0.5 seconds read timeout

    tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines
    tty.c_iflag     &=  ~(IXON | IXOFF | IXANY);// turn off s/w flow ctrl
    tty.c_lflag     &=  ~(ICANON | ECHO | ECHOE | ISIG); // make raw
    tty.c_oflag     &=  ~OPOST;              // make raw

    /* Flush Port, then applies attributes */
    tcflush( USB, TCIFLUSH );

    if ( tcsetattr ( USB, TCSANOW, &tty ) != 0)
    {
        cout << "Error " << errno << " from tcsetattr" << endl;
    }
   
    VideoCapture cap1(1); //capture the video1 from webcam1
    VideoCapture cap2(2); //capture the video2 from webcam2
    

    if ( !cap1.isOpened() )  // if not success, exit program
    {
         cout << "Cannot open the web cam1" << endl;
         return -1;
    }

    if ( !cap2.isOpened() )  // if not success, exit program
    {
         cout << "Cannot open the web cam2" << endl;
         return -1;
    }

    
    namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

 int iLowH = 170;
 int iHighH = 179;

 int iLowS = 100; 
 int iHighS = 255;

 int iLowV = 60;
 int iHighV = 255;

 //Create trackbars in "Control" window
 createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
 createTrackbar("HighH", "Control", &iHighH, 179);

 createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
 createTrackbar("HighS", "Control", &iHighS, 255);

 createTrackbar("LowV", "Control", &iLowV, 255);//Value (0 - 255)
 createTrackbar("HighV", "Control", &iHighV, 255);

 // int iLastX = -1; 
 //int iLastY = -1;

 //Capture a temporary image from the camera
 Mat imgTmp1;
 Mat imgTmp2;
 cap1.read(imgTmp1);
 cap2.read(imgTmp2);
 
 //Create a black image with the size as the camera output
 // Mat imgLines = Mat::zeros( imgTmp1.size(), CV_8UC3 );
 Mat img1Cir = Mat::zeros( imgTmp1.size(), CV_8UC3 );
 Mat img2Cir = Mat::zeros( imgTmp2.size(), CV_8UC3 );

    while (true)
    {
        Mat imgOriginal;

        bool bSuccess = cap1.read(imgOriginal); // read a new frame from video

	Mat imgOriginal2;

	bool bSuccess2 = cap2.read(imgOriginal2);


	if (!bSuccess || !bSuccess2) //if not success, break loop
        {
             cout << "Cannot read a frame from video stream" << endl;
             break;
        }

	img1Cir = Mat::zeros( imgTmp1.size(), CV_8UC3 );
	img2Cir = Mat::zeros( imgTmp2.size(), CV_8UC3 );

  Mat img1HSV;
  Mat img2HSV;

  cvtColor(imgOriginal, img1HSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
  cvtColor(imgOriginal2, img2HSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
  
  Mat img1Thresholded;
  Mat img2Thresholded;
  
  inRange(img1HSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), img1Thresholded); //Threshold the image1
  inRange(img2HSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), img2Thresholded); //Threshold the image2
  
  //morphological opening (removes small objects from the foreground)
  erode(img1Thresholded, img1Thresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
  dilate( img1Thresholded, img1Thresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

  //morphological closing (removes small holes from the foreground)
  dilate( img1Thresholded, img1Thresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
  erode(img1Thresholded, img1Thresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

  //morphological opening (removes small objects from the foreground)
  erode(img2Thresholded, img2Thresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
  dilate( img2Thresholded, img2Thresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

  //morphological closing (removes small holes from the foreground)
  dilate( img2Thresholded, img2Thresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
  erode(img2Thresholded, img2Thresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

  //Calculate the moments of the thresholded image
  Moments oImg1Moments = moments(img1Thresholded);
  Moments oImg2Moments = moments(img2Thresholded);
  
  double dCam1M01 = oImg1Moments.m01;
  double dCam1M10 = oImg1Moments.m10;
  double dCam1Area = oImg1Moments.m00;

  double dCam2M01 = oImg2Moments.m01;
  double dCam2M10 = oImg2Moments.m10;
  double dCam2Area = oImg2Moments.m00;
  
  // if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero 
  if (dCam1Area > 10000 && dCam2Area > 10000)
  {
   //calculate the position of the ball
   int Cam1posX = dCam1M10 / dCam1Area;
   int Cam2posX = dCam2M10 / dCam2Area;

   int Cam1posY = dCam1M01 / dCam1Area;
   int Cam2posY = dCam2M01 / dCam2Area;

   if (Cam1posX >= 0 && Cam2posX >= 0)
   {
    //Draw a red line from the previous point to the current point
     // line(imgLines, Point(posX, posY), Point(iLastX, iLastY), Scalar(0,0,255), 2);
     circle(img1Cir, Point(Cam1posX, Cam1posY), 3, Scalar(255,255,255), 1, 8, 0);
     circle(img2Cir, Point(Cam2posX, Cam2posY), 3, Scalar(255,255,255), 1, 8, 0);

    /* *** WRITE *** */

     unsigned char cmd[] = {'c', 'c', '\r', '\0'};

     
     if(Cam1posX < 225) //255-30
     {
       cout << "(" << Cam1posX <<  ") <--";
       cmd[0] = 'a';
     }
     else if(Cam1posX >285) //255+30
     {  
       cout << "(" << Cam1posX <<  ") -->";
       cmd[0] = 'd';
     }
     else
     {
       cout << "(" << Cam1posX <<  ") -x-";
       cmd[0]='s';
     }
     
     if(Cam2posX < 225)
       {
	 cout << "(" << Cam2posX <<  ") <--" << endl;
	 cmd[1]='z';
       }
     else if(Cam2posX >285)
       {
	 cout << "(" << Cam2posX <<  ") -->" << endl;
	 cmd[1]='c';
       }
     else
       {
	 cout << "(" << Cam2posX <<  ") -x-" << endl;
	 cmd[1]='x';
       }
     
    int n_written = write( USB, cmd, sizeof(cmd) -1 );

     
   }

   //   iLastX = posX;
   //iLastY = posY;
  }

  //imshow("Thresholded Image1", img1Thresholded); //show the thresholded image
  //imshow("Thresholded Image2", img2Thresholded);
  
  imgOriginal = imgOriginal + img1Cir;
  imgOriginal2 = imgOriginal2 + img2Cir;
  imshow("Original", imgOriginal); //show the original image
  imshow("Original2", imgOriginal2);
  
        if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
       {
	 cout << "esc key is pressed by user" << endl;
            break; 
       }
    }

        close(USB);

   return 0;
}
