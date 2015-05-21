#include <cv.h>
#include "nonfree/features2d.hpp"
#include "nonfree/nonfree.hpp"
#include <highgui_c.h>
#include <stdio.h>
#include <stdarg.h>
#include "cvutil.h"

/*Function///////////////////////////////////////////////////////////////

Name:       cvShowManyImages

Purpose:    This is a function illustrating how to display more than one
               image in a single window using Intel OpenCV

Parameters: char *title: Title of the window to be displayed
            int nArgs:   Number of images to be displayed
            ...:         IplImage*, which contains the images

Language:   C++

The method used is to set the ROIs of a Single Big image and then resizing
and copying the input images on to the Single Big Image.

This function does not stretch the image...
It resizes the image without modifying the width/height ratio..

This function can be called like this:

cvShowManyImages("Images", 2, 0, img1, img2);
or
cvShowManyImages("Images", 5, 3, img2, img2, img3, img4, img5);

This function can display upto 12 images in a single window.
It does not check whether the arguments are of type IplImage* or not.
The maximum window size is 700 by 660 pixels.
Does not display anything if the number of arguments is less than
    one or greater than 12.

If you pass a pointer that is not IplImage*, Error will occur.
Take care of the number of arguments you pass, and the type of arguments,
which should be of type IplImage* ONLY.

Idea was from BettySanchi of OpenCV Yahoo! Groups.

If you have trouble compiling and/or executing
this code, I would like to hear about it.

You could try posting on the OpenCV Yahoo! Groups
[url]http://groups.yahoo.com/group/OpenCV/messages/ [/url]


Parameswaran,
Chennai, India.

cegparamesh[at]gmail[dot]com

...
///////////////////////////////////////////////////////////////////////*/

void cvShowManyImages(char* title, int nArgs, int nChannels, ...) {

    // img - Used for getting the arguments
    IplImage *img;

    // DispImage - the image in which input images are to be copied
    IplImage *DispImage;

    int size;
    int i;
    int m, n;
    int x, y;

    // w - Maximum number of images in a row
    // h - Maximum number of images in a column
    int w, h;

    // scale - How much we have to resize the image
    float scale;
    int max;

    // If the number of arguments is lesser than 0 or greater than 12
    // return without displaying
    if(nArgs <= 0) {
        printf("Number of arguments too small....\n");
        return;
    }
    else if(nArgs > 12) {
        printf("Number of arguments too large....\n");
        return;
    }
    // Determine the size of the image,
    // and the number of rows/cols
    // from number of arguments
    else if (nArgs == 1) {
        w = h = 1;
        size = 300;
    }
    else if (nArgs == 2) {
        w = 2; h = 1;
        size = 300;
    }
    else if (nArgs == 3 || nArgs == 4) {
        w = 2; h = 2;
        size = 300;
    }
    else if (nArgs == 5 || nArgs == 6) {
        w = 3; h = 2;
        size = 200;
    }
    else if (nArgs == 7 || nArgs == 8) {
        w = 4; h = 2;
        size = 200;
    }
    else {
        w = 4; h = 3;
        size = 150;
    }

    // Create a new image with nChannels channels
    // nChannels = 0 indicates the image is in grayscale
    DispImage = cvCreateImage( cvSize(60 + size*w, 40 + size*h), 8, nChannels );
    //DispImage = cvCreateImage( cvSize(100 + size*w, 80 + size*h), 8, nChannels );

    // Used to get the arguments passed
    va_list args;
    va_start(args, nArgs);

    // Loop for nArgs number of arguments
    for (i = 0, m = 20, n = 20; i < nArgs; i++, m += (20 + size)) {

        // Get the Pointer to the IplImage
        img = va_arg(args, IplImage*);

        // Check whether it is NULL or not
        // If it is NULL, release the image, and return
        if(img == 0) {
            printf("Invalid arguments");
            cvReleaseImage(&DispImage);
            return;
        }

        // Find the width and height of the image
        x = img->width;
        y = img->height;

        // Find whether height or width is greater in order to resize the image
        max = (x > y)? x: y;

        // Find the scaling factor to resize the image
        scale = (float) ( (float) max / size );

        // Used to Align the images
        if( i % w == 0 && m!= 20) {
            m = 20;
            n+= 20 + size;
        }

        // Set the image ROI to display the current image
        cvSetImageROI(DispImage, cvRect(m, n, (int)( x/scale ), (int)( y/scale )));

        // Resize the input image and copy the it to the Single Big Image
        cvResize(img, DispImage);

        // Reset the ROI in order to display the next image
        cvResetImageROI(DispImage);
    }

    // Create a new window, and show the Single Big Image
    cvNamedWindow( title, 1 );
    cvShowImage( title, DispImage);

    cvWaitKey(0);
    cvDestroyWindow(title);

    // End the number of arguments
    va_end(args);

    // Release the Image Memory
    cvReleaseImage(&DispImage);
}


///////////////////////////////////////////////////////////////////////*/
void cvShowTwoImages(char* title, int nChannels, IplImage* img1, IplImage* img2) {

    // DispImage - the image in which input images are to be copied
    IplImage *DispImage;

    // Getting the images sizes and determining the output image sizes
    int width = img1->width + img2->width;
    int height = (img1->height > img2->height) ? img1->height : img2->height;

    // Create a new image with nChannels channels
    // nChannels = 0 indicates the image is in grayscale
    DispImage = cvCreateImage( cvSize(60 + width, 40 + height), 8, nChannels );

	// Check whether it is NULL or not
	// If it is NULL, release the images, and return
	if(img1 == 0 || img2 == 0) {
		printf("Invalid arguments");
		cvReleaseImage(&img1);
		cvReleaseImage(&img2);
		return;
	}

	// Set the image ROI to display the 1st image
	cvSetImageROI(DispImage, cvRect(19, 19, img1->width, img1->height));

    // Resize the 1st image and copy the it to the Single Big Image
    cvResize(img1, DispImage);

	// Reset the ROI in order to display the 2nd image
	cvResetImageROI(DispImage);

	// Set the image ROI to display the 2nd image
	cvSetImageROI(DispImage, cvRect(39+img1->width, 19, img2->width, img2->height));

    // Resize the 2nd image and copy the it to the Single Big Image
    cvResize(img2, DispImage);

    // Reset the ROI in order to display the next image
    cvResetImageROI(DispImage);

    // Create a new window, and show the Single Big Image
    cvNamedWindow(title, 1);
    cvShowImage(title, DispImage);
    cvWaitKey(0);
    cvDestroyWindow(title);
    cvReleaseImage(&DispImage);
}
