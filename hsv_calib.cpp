#include <stdio.h>
#include "cv.h"
#include "cxcore.h"
#include "highgui.h"


void mouseCallback(
        int event, int x, int y, int flags, void* param
        );

CvRect box;
bool drawing_box = false;


void drawBox( IplImage* img) {
    cvRectangle (
            img,
            cvPoint(box.x,box.y),
            cvPoint(box.x+box.width,box.y+box.height),
            CV_RGB(0x00,0xFF,0x00)
            );
}

void drawCircle( IplImage* img) {
    cvCircle (
            img,
            cvPoint(box.x+box.width/2,box.y+box.height/2),
            sqrt(box.width*box.width + box.height*box.height)/2,
            CV_RGB(0x00,0xFF,0x00)
            );
}

int main( int argc, char* argv[] )
{
    if(argc != 2) {
        printf("Wrong number of arguments\n");
        return 1;
    }

    box = cvRect(-1,-1,0,0);

    IplImage* originalImage = cvLoadImage(argv[1]);
    if (!originalImage) {
        printf("Cannot open input image: %s\n", argv[1]);
        return -1;
    }

    IplImage* imageParam = cvCloneImage( originalImage );
    IplImage* imageOut = cvCloneImage( originalImage );

    cvNamedWindow("HSV Calib");

    cvSetMouseCallback(
            "HSV Calib",
            mouseCallback,
            (void*) imageParam
            );

    while( 1 ) {
        cvCopyImage( imageParam, imageOut );
        if( drawing_box ) {
            drawBox( imageOut);
            //drawCircle( temp, box );
        }
        cvShowImage( "HSV Calib", imageOut );
        char key = cvWaitKey(10);
        if( key == 27 )
            break;
        else if ( key == 'r') {
            cvCopyImage( originalImage, imageParam );
        }
    }
    // Be tidy
    //
    cvReleaseImage( &imageParam );
    cvReleaseImage( &imageOut );
    cvReleaseImage( &originalImage );
    cvDestroyWindow( "HSV Calib" );
}


void mouseCallback(
        int event, int x, int y, int flags, void* param
        ) {
    IplImage* image = (IplImage*) param;
    switch( event ) {
        case CV_EVENT_MOUSEMOVE:
            {
                if( drawing_box ) {
                    box.width  = x-box.x;
                    box.height = y-box.y;
                }
            }
            break;
        case CV_EVENT_LBUTTONDOWN:
            {
                drawing_box = true;
                box = cvRect(x, y, 0, 0);
            }
            break;
        case CV_EVENT_LBUTTONUP:
            {
                drawing_box = false;
                if(box.width<0) {
                    box.x+=box.width;
                    box.width *=-1;
                }
                if(box.height<0) {
                    box.y+=box.height;
                    box.height*=-1;
                }
                if((box.width<=1) || (box.height<=1)){
                    return;
                }

                if(flags & CV_EVENT_FLAG_CTRLKEY){
                    IplImage* resImage = cvCreateImage( cvGetSize(image), IPL_DEPTH_8U, 3);
                    cvSetImageROI(image, box);
                    cvResize(image, resImage);
                    cvResetImageROI(image);
                    cvCopy(resImage, image);
                    cvReleaseImage(&resImage);
                }
                else{
                    CvScalar mean, stdDev;
                    IplImage* hsvImage = cvCreateImage( cvGetSize(image), IPL_DEPTH_8U, 3);
                    cvCvtColor(image, hsvImage, CV_BGR2HSV);
                    //drawCircle( image, box );
                    cvSetImageROI(hsvImage, box);
                    cvAvgSdv(hsvImage, &mean, &stdDev);
                    printf("\n%s\n", "Hue:");
                    printf("Mean: %f, Standard Deviation: %f\n", mean.val[0]*2, stdDev.val[0]*2);
                    printf("%s\n", "Saturation:");
                    printf("Mean: %f, Standard Deviation: %f\n", mean.val[1], stdDev.val[1]);
                    printf("%s\n", "Value:");
                    printf("Mean: %f, Standard Deviation: %f\n", mean.val[2], stdDev.val[2]);
                    cvResetImageROI(image);
                    cvReleaseImage(&hsvImage);
                }

            }
            break;
    }
}




