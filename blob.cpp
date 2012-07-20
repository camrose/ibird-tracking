// Test code for color blob tracking
// Adapted from http://www.aishack.in/

#include "cv.h"
#include "highgui.h"

#include <stdlib.h>
#include <stdio.h>

// Declarations
IplImage* ColorThresholdImage(IplImage* img);

// Code Body
int main(void) {

    // Attempt to initialize capture device
    CvCapture* capture = NULL;
    capture = cvCaptureFromCAM(0);    
    if(capture == NULL) {
        printf("Could not initialize capture device\n");
        return -1;
    }

    // Set up display windows
    cvNamedWindow("video");
    cvNamedWindow("thresh");
    
    IplImage* capturedFrame = NULL; // BGR 8-bit?
    IplImage* thresholded = NULL; // Binary
    //CvMoments *moments = (CvMoments*) malloc(sizeof(CvMoments)); // Try static alloc first
    CvMoments moments;
    double moment10, moment01, area;
    int posX, posY;
    
    while(true) {
            
        cvGrabFrame(capture); // Store internally
        capturedFrame = cvRetrieveFrame(capture); // Decode - Don't modify returned data!
        if(capturedFrame == NULL) {
            printf("Frame capture failed.\n");
            break;
        }
        
        thresholded = ColorThresholdImage(capturedFrame);
        
        cvMomentsS(imgYellowThresh, &moments, true); // true for binary image
        moment10 = cvGetSpatialMoment(&moments, 1, 0);
        moment01 = cvGetSpatialMoment(&moments, 0, 1);
        area = cvGetCentralMoment(&moments, 0, 0);
        posX = (int) (moment10/area);
        posY = (int) (moment01/area);
        
    }
    
}

IplImage* ColorThresholdImage(IplImage* img) {

    // Create temporary and return image
    IplImage* imgHSV = cvCreateImage(cvGetSize(img), 8, 3); // Size, depth, channels    
    IplImage* imgThreshed = cvCreateImage(cvGetSize(img), 8, 1);
    
    // Threshold operation
    cvCvtColor(img, imgHSV, CV_BGR2HSR);    
    CvScalar lowBound = cvScalar(20, 100, 100);
    CvScalar highBound = cvScalar(30, 255, 255);
    cvInRangeS(imgHSV, lowBound, highBound, imgThreshed);
    
    // Cleanup
    cvReleaseImage(&imgHSV);
    return imgThreshed;
    
}
