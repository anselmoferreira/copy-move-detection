#include <stdio.h>
#include <stdlib.h>
#include <highgui.h>
#include <cv.h>
#include "our-method.cpp"

int main(int argc, char** argv){

	if(argc != 3){
        printf("*********************************************\n*  Usage: ./copiacolagem [input] [output]   *\n*********************************************\n");
        return 0;
	}

    // Applying the CMF Method
	IplImage* outimg = ourmethod(argv[1]);

	// Saving the detection map
	cvSaveImage(argv[2], outimg);
    return 0;
}
