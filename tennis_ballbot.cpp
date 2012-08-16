/*
 * tennis_ballbot.cpp
 *
 *  Created on: Sep 09, 2011
 *      Author: ankush, ryanjulian, hhu
 */

#include <X11/keysym.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <time.h>
#include "camera.h"

using namespace cv;
using namespace std;


#define DISPLAY_CAPTURE             0 // Display camera capture
#define DISPLAY_PIPELINE            0 // Display all steps of pipeline
#define RECORD                      1 // Record video
#define VERBOSE                     1 // Lots of printing

// Note the channels are NOT RGB.
// Also note that H is in [0, 179] and the rest are [0,255]
// I-Bird Pink HSV
#define TARGET_H_LOW_P        (130) //130
#define TARGET_S_LOW_P        (50) //20
#define TARGET_V_LOW_P        (90) //80
#define TARGET_H_HIGH_P       (150) //150
#define TARGET_S_HIGH_P       (255) //255
#define TARGET_V_HIGH_P       (255) //255

#define TARGET_H_LOW_G        (24) //130
#define TARGET_S_LOW_G        (55) //20
#define TARGET_V_LOW_G        (88) //80
#define TARGET_H_HIGH_G       (68) //150
#define TARGET_S_HIGH_G       (255) //255
#define TARGET_V_HIGH_G       (255) //255

#define TARGET_H_LOW_W        (90)
#define TARGET_S_LOW_W        (60)
#define TARGET_V_LOW_W        (150)
#define TARGET_H_HIGH_W       (110)
#define TARGET_S_HIGH_W       (180)
#define TARGET_V_HIGH_W       (200)

// Camera driver config values
#define CAM_BRIGHTNESS      (0.4)
#define CAM_CONTRAST        (0.11)
#define CAM_SATURATION      (0.11)
#define CAM_GAIN            (1.0)

#define ERODE_LEVEL         (1)
#define DILATE_LEVEL        (3)

// ====== Static Variables ============
const double pi = 3.141592654;
Mat img;
int ffillMode    = 1;
int loDiff       = 45;
int upDiff       = 45;
int connectivity = 8;
int isColor      = true;
bool useMask     = false;
int newMaskVal   = 255;
Point2d horizonPt;
Mat cross = getStructuringElement(MORPH_CROSS, Size(5,5));

RNG rng;
camera cam;
VideoWriter record;
int n = 0;
char filename[200];

Size roi_size, roiExpansionRate, roiZoomRate;
Point2d lastBallLocation;

vector<Mat> frames;

// ====== Function Declarations =======
int searchFrame(Mat &frame, Mat &temp, Mat &dest);


struct bgr
{
  /** Structure for accessing individual pixels from 
      a 3 channel image (Mat) */
  unsigned char b,g,r;
  bgr(unsigned char b0, unsigned char g0, unsigned char r0) {
    r = r0;
    b = b0;
    g = g0;
  }

  bgr()
  {}
  
  bool operator==(const bgr &other)
  {
    return (other.r == r && other.b == b && other.g == g);
  }
};

struct ballContour
{
  /**Structure for representing the ball candidates.
   @param:
    1. contour : the contour of the candidate
    2. pixelPosition: the position of the center of the candidate in the image
    3. sizeMeasure : place holder for some measure of size of the ball
                     currently being used for storing the number of pixels
                     which fall in the color range.
  */
  vector<Point> contour;
  Point pixelPosition;
  double sizeMeasure;
  
  ballContour(double size0) {
    sizeMeasure = size0;
  }
  ballContour() {}
  void operator= (const ballContour & other) {
    contour = other.contour;
    pixelPosition = other.pixelPosition;
    sizeMeasure = other.sizeMeasure;
  }
};

Mat getContourPixels(vector <Point> contour, Mat &src) {
  /**Returns the pixels in the actual image which fall insdie 
     the contour.
   * @param: contour - vector of points which form a contour
   *        src     - source Mat from which the pixels need to be extracted.
   */
  Rect R = boundingRect(Mat(contour));
  Mat outMat(Size(R.width, R.height), src.type());
  outMat = src(Range(R.y, R.y + R.height),
         Range(R.x, R.x + R.width));
  return outMat;
}

/**Returns a vector of contours (which are vectors of points) of the SMALL, CIRCULAR blobs
 * found in input.*/
vector <ballContour> doContours(Mat & input)
{
  /** Returns a vector of ball candidates' contours
      found in the given input image.
      @param
          input : the source image
      @return
          vector of candidate contours
  */
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  vector<ballContour> returnCandidates;
  findContours(input, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
  if (!contours.empty() && !hierarchy.empty()) {
    int idx = 0;
    for( ;idx >= 0; idx = hierarchy[idx][0]) {
      double contArea = contourArea(Mat(contours[idx]));
        if (contArea > 0) { // used to be 0
          // parameters for ellipse fit onto the contour
          RotatedRect contourEllipse;
          float majorAxis = 0.0, minorAxis = 0.0, area = 0.0,
          delta = 1, r = 3;
        if( contours[idx].size() >= 6 ) {
            try {  
              //fit an ellipse to the contour
              //cout << (int) contours[idx].size() << endl;
              contourEllipse = fitEllipse(Mat(contours[idx]));
              majorAxis = contourEllipse.size.height;
              minorAxis = contourEllipse.size.width;
              area = (majorAxis * minorAxis * pi) / 4.0;
            
              /*filter based on shape:
               * 1. calculate delta = |contour_area - ellipse_area|/contour_area
               * 2. r = Major_axis/Minor_axis
               *
               *  for circular ball, we expect: delta -->0(+) && r --> 1(+)
               */
              delta = (abs(contArea - area))/contArea;
              r = majorAxis/minorAxis;
            }
          catch (...) {continue;}
        }
    
        //if (delta < 1.2) {
          ballContour candidate;
          candidate.contour = contours[idx];
          candidate.pixelPosition = contourEllipse.center;
          returnCandidates.push_back(candidate);
        //}
      }
    }
  }
  return returnCandidates;
}

vector <ballContour> findWindow(Mat & input)
{
  /** Returns a vector of ball candidates' contours
      found in the given input image.
      @param
          input : the source image
      @return
          vector of candidate contours
  */
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  vector<ballContour> returnWindows;
  findContours(input, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
  if (!contours.empty() && !hierarchy.empty()) {
    int idx = 0;
    for( ;idx >= 0; idx = hierarchy[idx][0]) {
      double contArea = contourArea(Mat(contours[idx]));
        if (contArea > 50) { // used to be 0
          // parameters for ellipse fit onto the contour
          RotatedRect contourEllipse;
          float majorAxis = 0.0, minorAxis = 0.0, area = 0.0,
          delta = 1, r = 3;
        if( contours[idx].size() >= 6 ) {
            try {  
              //fit an ellipse to the contour
              //cout << (int) contours[idx].size() << endl;
              //contourEllipse = fitEllipse(Mat(contours[idx])); 
              contourEllipse = minAreaRect(Mat(contours[idx]));
              majorAxis = contourEllipse.size.height;
              minorAxis = contourEllipse.size.width;
              //area = (majorAxis * minorAxis * pi) / 4.0;
              area = (majorAxis * minorAxis);
            
              /*filter based on shape:
               * 1. calculate delta = |contour_area - ellipse_area|/contour_area
               * 2. r = Major_axis/Minor_axis
               *
               *  for circular ball, we expect: delta -->0(+) && r --> 1(+)
               */
              delta = (abs(contArea - area))/contArea;
              r = majorAxis/minorAxis;
            }
          catch (...) {continue;}
        }
    
        //if (delta < 0.7) { //&& r<3
          ballContour windows;
          windows.contour = contours[idx];
          windows.pixelPosition = contourEllipse.center;
          returnWindows.push_back(windows);
        //}
      }
    }
  }
  return returnWindows;
}

bool withinBounds(int n, int m, int nMax, int mMax) {
  /**Returns TRUE iff 0<=n(+-1)<nMAX && 0<= m(+-)1<mMax.*/
  return ((n-1 >= 0)
    && (n+1 < nMax)
    && (m-1 >= 0)
    && (m+1 < mMax));
}

/**Finds connected components in the input image img.
   The similarity is based on color and intensity of neighbouring pixels.
   Filters the connected components based on size and color (here color bounds are loose).
  @param:
      1. img : The input image
  @return:
      2. maskOut: the mask (single channel, binary image) representing 
                  the connected components. The connected compoenets are
                  filtered on the size. "Appropriate sized" blobs are kept,
                  others discarded.*/
Mat floodFillPostprocess( Mat& img) {

  Mat maskOut( img.rows+2, img.cols+2, CV_8UC1, Scalar::all(0) );
  Mat mask( img.rows+2, img.cols+2, CV_8UC1, Scalar::all(0) );
  Mat maskLocal( img.rows+2, img.cols+2, CV_8UC1, Scalar::all(0));
  Scalar lo = Scalar(loDiff, loDiff, loDiff),
  up = Scalar(upDiff, upDiff, upDiff);
  int flags = connectivity + (newMaskVal << 8) + CV_FLOODFILL_FIXED_RANGE;

  for( int y = 0; y < img.rows; y++ ) {
    for( int x = 0; x < img.cols; x++ ) { 
       if (withinBounds(x, y, img.cols, img.rows)) {
         if(mask.at<uchar>(y+1, x+1) == 0 && mask.at<uchar>(y-1, x-1) == 0) {
            maskLocal = Mat::zeros(mask.size(), mask.type());
            int area;
            Scalar newVal( rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));
            area = floodFill(img, maskLocal, Point(x,y), newVal, 0, lo, up, flags);
            bitwise_or(mask, maskLocal, mask);
        
            if(area>0 && area < 800) {
                bitwise_or(maskOut, maskLocal, maskOut);
            }
        } else { continue; }
      }
    }
  }
  return maskOut;
}

void processNewFrame(Mat &frame, Mat &temp, Mat &dest) {

#if DISPLAY_CAPTURE
  imshow("Captured Frame", frame);
#endif
  
  searchFrame(frame, temp, dest);
  
}
  
int searchFrame(Mat &frame, Mat &frameHSV, Mat &colorRangeMask) {

    //Mat ballFound = Mat(frame.size(), frame.type());

    Mat windowRangeMask(frame.size(), frame.type());
    Mat pinkRangeMask(frame.size(), frame.type());
    Mat greenRangeMask(frame.size(), frame.type());

    //Color filtering
    cvtColor(frame, frameHSV, CV_RGB2HSV);
    inRange(frameHSV,
          Scalar(TARGET_H_LOW_P, TARGET_S_LOW_P, TARGET_V_LOW_P, 0),
          Scalar(TARGET_H_HIGH_P, TARGET_S_HIGH_P, TARGET_V_HIGH_P, 0),
          pinkRangeMask);
    inRange(frameHSV,
          Scalar(TARGET_H_LOW_G, TARGET_S_LOW_G, TARGET_V_LOW_G, 0),
          Scalar(TARGET_H_HIGH_G, TARGET_S_HIGH_G, TARGET_V_HIGH_G, 0),
          greenRangeMask);

    dilate(pinkRangeMask,pinkRangeMask,cross,Point(-1,-1),DILATE_LEVEL);
    dilate(greenRangeMask,greenRangeMask,cross,Point(-1,-1),DILATE_LEVEL);
    
    bitwise_and(pinkRangeMask,greenRangeMask,colorRangeMask);

    /*inRange(frameHSV,
          Scalar(TARGET_H_LOW_W, TARGET_S_LOW_W, TARGET_V_LOW_W, 0),
          Scalar(TARGET_H_HIGH_W, TARGET_S_HIGH_W, TARGET_V_HIGH_W, 0),
          windowRangeMask);*/
    //bitwise_or(birdRangeMask, windowRangeMask, colorRangeMask);
  //pyrMeanShiftFiltering(colorRangeMask, dst, 4, 20, 2);
#if DISPLAY_PIPELINE
    imshow("Color Filtered", colorRangeMask);
#endif

  // Flood fill
  //Mat mask = floodFillPostprocess(colorRangeMask);
#if DISPLAY_PIPELINE
//    imshow("Flood filled", colorRangeMask);
#endif

  // Erode and Dilate
  //erode(colorRangeMask,colorRangeMask,cross,Point(-1,-1), ERODE_LEVEL);
#if DISPLAY_PIPELINE
    //imshow("Eroded", colorRangeMask);
#endif

  //dilate(colorRangeMask,colorRangeMask,cross,Point(-1,-1),DILATE_LEVEL);
#if DISPLAY_PIPELINE
    //imshow("Erode + Dilate", colorRangeMask);
#endif
    
    Vector<ballContour> candidates = doContours(colorRangeMask);

    if(candidates.empty()) {
        /*Vector<ballContour> window = findWindow(windowRangeMask);
        if (!window.empty()) {
            ellipse( frame, window[0].pixelPosition, Size(10,10),
                0, 0, 360, Scalar(0,255,0), CV_FILLED, 8, 0);
        }*/
        ellipse( frame, Point(160,120), Size(10,10),
                0, 0, 360, Scalar(0,255,0), CV_FILLED, 8, 0);
#if DISPLAY_PIPELINE
    imshow("Result", frame);
#endif
    } else {

        int bx = candidates[0].pixelPosition.x;
        int by = candidates[0].pixelPosition.y;
        
        /*unsigned int i;
        for(i=0; i < candidates.size(); i++) {
            ellipse( frame, candidates[i].pixelPosition, Size(10,10),
                    0, 0, 360, Scalar(255,0,0), CV_FILLED, 8, 0);
        }*/

        ellipse( frame, candidates[0].pixelPosition, Size(10,10),
                    0, 0, 360, Scalar(0,0,255), CV_FILLED, 8, 0);

        /*Vector<ballContour> window = findWindow(windowRangeMask);
        if (!window.empty()) {
            printf("#%d,%d,%d,%d\n",bx, by, window[0].pixelPosition.x, window[0].pixelPosition.y);
            ellipse( frame, window[0].pixelPosition, Size(10,10),
                0, 0, 360, Scalar(0,255,0), CV_FILLED, 8, 0);
        }*/
        
        ellipse( frame, Point(160,120), Size(10,10),
                0, 0, 360, Scalar(0,255,0), CV_FILLED, 8, 0);
        printf("#%d,%d,%d,%d\n",bx, by, 0, 0);


#if DISPLAY_PIPELINE
    imshow("Result", frame);
#endif
    }

#if RECORD
  //record << (frame);
#endif

  return 0;
  
}

int main( int argc, char** argv ) {
  
  Mat frame;
  Size frame_size;    
  double cam_brightness, cam_contrast, cam_saturation, cam_gain;

  // Open and configure camera
  VideoCapture cap(0);
  if( !cap.isOpened() ) {
    cout << "Camera open failed." << endl;
    return -1;
  }

  // Set camera parameters
  cap.set(CV_CAP_PROP_BRIGHTNESS, CAM_BRIGHTNESS);
  cap.set(CV_CAP_PROP_CONTRAST, CAM_CONTRAST);
  cap.set(CV_CAP_PROP_SATURATION, CAM_SATURATION);
  //cap.set(CV_CAP_PROP_HUE, CAM_HUE); // Doesn't exist for our camera
  cap.set(CV_CAP_PROP_GAIN, CAM_GAIN);
  //cap.set(CV_CAP_PROP_EXPOSURE, CAM_EXPOSURE); // Not supported
  cap.set(CV_CAP_PROP_FRAME_WIDTH, 320);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
  cap.set(CV_CAP_PROP_FPS, 30);
  cap.set(CV_CAP_PROP_FORMAT, 0);
//  cap.set(CV_CAP_PROP_FOURCC, 0);

  // Get camera parameters to make sure they were set correctly
  frame_size = Size(cap.get(CV_CAP_PROP_FRAME_WIDTH), cap.get(CV_CAP_PROP_FRAME_HEIGHT));
  cam_brightness = cap.get(CV_CAP_PROP_BRIGHTNESS);
  cam_contrast = cap.get(CV_CAP_PROP_CONTRAST);
  cam_saturation = cap.get(CV_CAP_PROP_SATURATION);
  cam_gain = cap.get(CV_CAP_PROP_GAIN);

#if VERBOSE
  printf("Opened %u by %u camera stream.\n", frame_size.width, frame_size.height);
  printf("Brightness: %f\n", cam_brightness);
  printf("Contrast: %f\n", cam_contrast);
  printf("Saturation: %f\n", cam_saturation);
  printf("Gain: %f\n", cam_gain);
#endif

  // open video recording
#if RECORD
  record = VideoWriter("tennis_ballbot.mjpg", CV_FOURCC('M','J','P','G'), 25, frame_size, true);
  if( !record.isOpened() ) {
    printf("Recording failed to open!\n");
    return -1;
  }
#endif
  int stop = 0;
  double framerate;
  clock_t prev_time, new_time;
    
  prev_time = clock();

  cap >> frame;
  Mat frameHSV(frame.size(), frame.type());
  Mat colorRangeMask(frame.size(), frame.type());

  while(1) {

    cap >> frame;

    cout.flush();

    new_time = clock();
    framerate = CLOCKS_PER_SEC/((float)(new_time - prev_time));
    prev_time = new_time;
    

#if VERBOSE
    cout << "Framerate: " << framerate << "\n";
    //printf("Framerate: %f\n", framerate);
    //fprintf(stdout, "Framerate: %f\n", framerate);
#endif
 
    processNewFrame( frame, frameHSV, colorRangeMask );
    //if(waitKey(5) == (0x100000 + 'q')) {
    int c = waitKey(5);
    printf("c = %d\n", c);
    if(c != -1) {
        break;
    }
    
#if RECORD
    frames.push_back(*(new Mat));
    frame.copyTo(frames[frames.size()-1]);
#endif
    /*if(frames.size() > 1) {
      printf("frame pointers = %d", frames[frames.size()-2]);
    }*/
    stop++;
#if RECORD
    if(stop > 80) {
      break;
    }
#endif
  }
  
  cap.release();
 
#if RECORD
  unsigned int i=0;
  for (i=0; i < frames.size(); i++) {
    record << (frames[i]);
  }
#endif

  return 0;

}

