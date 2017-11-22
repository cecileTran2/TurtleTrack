/*
   g++ -o detect detect.cpp `pkg-config --cflags --libs opencv` -std=c++11
*/


#include <opencv2/opencv.hpp>
#include <vector>
#include <array>

#define NB_COLORS 4

int main(int argc, char* argv[]) {

  cv::VideoCapture cap(0); // open the default camera
  if(!cap.isOpened())      // check if we succeeded
    return -1;

  std::array<cv::Scalar,NB_COLORS> colors = {{cv::Scalar(255,0,0),
					      {0,255,0},
					      {0,0,255},
					      {255,255,0}}};
  cv::Mat hsv;
  cv::Mat detection; 
  cv::Mat cleaned; 
  cv::Mat frame;
  cv::namedWindow("detection",1);
  cv::namedWindow("clean-up",1);
  cv::namedWindow("contours",1);
  

  // This is a "blue" color range.
  cv::Scalar hsv_min(100, 100,  50);
  cv::Scalar hsv_max(110, 256, 256);

  cv::Mat open_elem  = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3));
  cv::Mat close_elem = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(9,9));

  for(;;) {
    cap >> frame;                               // get a new frame from camera
    cv::cvtColor(frame, hsv, CV_BGR2HSV);       // convert it to HSV
    cv::inRange(hsv,hsv_min,hsv_max,detection); // select in-range pixels

    cv::imshow("detection", detection);

    cv::erode (detection,detection,open_elem);  // opening...
    cv::dilate(detection,detection,open_elem);  // ...
    cv::dilate(detection,detection,close_elem); // closing...
    cv::erode (detection,detection,close_elem); // ...

    cv::imshow("clean-up", detection);

    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i>               hierarchy;
    cv::findContours(detection, 
		     contours, hierarchy, 
		     CV_RETR_TREE, 
		     CV_CHAIN_APPROX_SIMPLE, 
		     cv::Point(0, 0));
    
    
    cv::Mat display = cv::Mat::zeros(detection.size(), CV_8UC3 );
    for(int i = 0; i< contours.size(); i++)
      cv::drawContours(display, contours, 
      		       i, colors[i%NB_COLORS],
      		       3);
    cv::imshow("contours", display);
    
    if(cv::waitKey(30) >= 0) break;
  }
  // the camera will be deinitialized automatically in VideoCapture destructor
  return 0;
}
