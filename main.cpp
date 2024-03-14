/*
 * untitled.cxx
 * 
 * Copyright 2023  <kevinzhu@raspberrypi>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 * 
 * 
 */



#include "src/TauCam.h"


#include <wiringPi.h>
#include <wiringSerial.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
//#include <raspicam/raspicam.h>
#include <opencv2/opencv.hpp>

#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>

using namespace cv;

int main(void)
{    
    VideoCapture cap; 
    Mat undistorted_image;
    
    Mat mtx = (Mat_<float>(3,3) << 2293.741, 0, 335.2158, 0.0, 2261.6832, 240.568, 0, 0, 1 );
    Mat dist = (Mat_<float>(1,5) <<  -5.27983,32.7776,0.01023,-0.12483,-124.9291   );
    Mat newmtx = (Mat_<float>(3,3) << 1813.00647,0,295.09372,0,1678.07117,243.4453,0,0,1);
    
    
    if(!cap.open(0))
      return 0;
      
    Ptr<Tracker> tracker = TrackerMedianFlow::create();
    
    Rect2d bbox(287,23,86,320);
    Mat frame; 
    cap >> frame; 
    undistort(frame,undistorted_image,mtx,dist);
    rectangle(frame,bbox, Scalar(255,0,0),2,1) ;
    
    imshow("Tracking",undistorted_image);
    tracker->init(undistorted_image,bbox);
    
    for (;;) {
        Mat frame; 
        cap >> frame; 
        undistort(frame,undistorted_image,mtx,dist);
        rectangle(undistorted_image,bbox, Scalar(255,0,0),2,1) ;       
        bool ok = tracker->update(undistorted_image,bbox);
        
        if (ok) {
            
            
            rectangle(undistorted_image,bbox,Scalar( 255,0,0),2,1);
            
        }
        else {
            
        }
        
        imshow("Tracking",undistorted_image);
        
        int k = waitKey(1);
        
        if(k==27) {
            break;
        }
    }
      
      
      
      
      
      
  //  for(;;) {
   //     Mat frame; 
   //     cap >> frame;
        
        //undistort(frame,
   //     undistort(frame,undistorted_image,mtx,dist);
   //     if(frame.empty() ) break;
   //     imshow("hello",undistorted_image);
  //      if ((waitKey(10)%256) == 27) break;
        
  //  }
    
    
      
    int fd;
    unsigned int nextTime;
    
    if(fd=serialOpen("/dev/serial0",115200)<0) {
        
        printf("bad\n");
        return 0;
    }
    
    if(wiringPiSetup()== -1 ) {
        printf("bad\n");
        return 0;
    }
    sleep(1);
    nextTime = millis() + 300;
       
  //  for (unsigned char i = 70; i < 256; i++) {
  //      if (millis() > nextTime) {
  //                fflush(stdout); 
      //  serialPutchar(fd,i);
  //      nextTime+=300;
  //      usleep(1000000);  
            
            
   //     }
        
   //     delay(3);
        
    //    while(serialDataAvail(fd)) {
            
    //        serialGetchar(fd);
   //         fflush(stdout);
   //     }

    //}
    
     
    
    
    
    
    sleep(1);
    uint16_t hi = 0;
    uint16_t temp = 500;
    TauCam taucam;
    taucam.SetIntegrationTime3d(hi,temp);
  //  taucam.GetId();
    std::cout<<"Start now" <<std::endl;
  //  taucam.GetPointCloud();
  
  while (true) {
    taucam.FillDepth();
    for (int i =0; i < 10;i++) {
        for (int j = 0; j <10;j++) {
        //printf("%f\n",taucam.depth_buffer[j+75 + (29+i)*160]);
    }
    printf("\n");
    }
    
    std::cout<<"end" <<std::endl;
    usleep(100000);
}
    std::cout << 'i' << std::endl;
    
    return 0;

}

