/* -----------------------------------------------------------------
 * cvutil.h
 * -----------------------------------------------------------------
 * This library contains some complementary
 * functions to the OpenCV library.
 * -----------------------------------------------------------------
 */

/* This function is used to show several images in the same window */
/*extern void cvShowManyImages(char* title, int nArgs, int nChannels, ...);*/
extern void cvShowTwoImages(char* title, int nChannels, IplImage* img1, IplImage* img2);
extern void cvShowManyImages(char* title, int nArgs, int nChannels, ...);
