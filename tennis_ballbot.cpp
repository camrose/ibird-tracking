/*
 * tennis_ballbot.cpp
 *
 *  Created on: Sep 09, 2011
 *      Author: ankush, ryanjulian, hhu
 */

#include <X11/keysym.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
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
#define DISPLAY_PIPELINE            1 // Display all steps of pipeline
#define RECORD                      1 // Record video
#define VERBOSE                     1 // Lots of printing

// Note the channels are NOT RGB.
// Also note that H is in [0, 179] and the rest are [0,255]
// I-Bird Pink HSV
#define TARGET_H_LOW        (130)
#define TARGET_S_LOW        (20) 
#define TARGET_V_LOW        (80)
#define TARGET_H_HIGH       (150)
#define TARGET_S_HIGH       (255)
#define TARGET_V_HIGH       (255) 

// Camera driver config values
#define CAM_BRIGHTNESS      (0.4)
#define CAM_CONTRAST        (0.11)
#define CAM_SATURATION      (0.11)
#define CAM_GAIN            (1.0)

#define ERODE_LEVEL         (1)
#define DILATE_LEVEL        (1)

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

Size roi_size, roiExpansionRate, roiZoomRate;
Point2d lastBallLocation;

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
        if (contArea > 0) {
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
    
        if (r < 3 && delta < 0.7) {
          ballContour candidate;
          candidate.contour = contours[idx];
          candidate.pixelPosition = contourEllipse.center;
          returnCandidates.push_back(candidate);
        }
      }
    }
  }
  return returnCandidates;
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

    //Color filtering
    cvtColor(frame, frameHSV, CV_RGB2HSV);
    inRange(frameHSV,
          Scalar(TARGET_H_LOW, TARGET_S_LOW, TARGET_V_LOW, 0),
          Scalar(TARGET_H_HIGH, TARGET_S_HIGH, TARGET_V_HIGH, 0),
          colorRangeMask);
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
    imshow("Eroded", colorRangeMask);
#endif

  //dilate(colorRangeMask,colorRangeMask,cross,Point(-1,-1),DILATE_LEVEL);
#if DISPLAY_PIPELINE
    imshow("Erode + Dilate", colorRangeMask);
#endif

    Vector<ballContour> candidates = doContours(colorRangeMask);

    if(candidates.empty()) { 
#if DISPLAY_PIPELINE
    imshow("Result", frame);
#endif
    } else {

        unsigned int i;
        for(i = 0; i < candidates.size(); i++) {
            //printf("Ball %d found at %d, %d\n", i, candidates[i].pixelPosition.x, candidates[i].pixelPosition.y);
            printf("#%d,%d\n",candidates[i].pixelPosition.x, candidates[i].pixelPosition.y);
        }

#if DISPLAY_PIPELINE
        ellipse( frame, candidates[0].pixelPosition, Size(10,10),
            0, 0, 360, Scalar(0,0,255), CV_FILLED, 8, 0);
        imshow("Result", frame);
#endif
    }

#if RECORD
  record << (frame);
#endif

  return 0;
  
}

int main( int argc, char** argv ) {
  
  Mat frame;
  Size frame_size;    
  double cam_brightness, cam_contrast, cam_saturation, cam_gain;

  // Open and configure camera
  VideoCapture cap(1);
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
//  cap.set(CV_CAP_PROP_FPS, 30);
//  cap.set(CV_CAP_PROP_FORMAT, 0);
//  cap.set(CV_CAP_PROP_FOURCC, "UYUV");

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
    if(waitKey(5) == (0x100000 + 'q')) {
        break;
    }
  }
  
  cap.release();

  return 0;

}

