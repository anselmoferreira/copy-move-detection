#include <stdio.h>
#include <stdlib.h>
#include "math.h"
#include "highgui.h"
#include <cv.h>
//#include "surf.h"
#include "cvutil.h"
#include "our-method.h"
#include "surf-new.cpp"
// ------------------------------------ //
//               STRUCTURES             //
// ------------------------------------ //

// Structure used to create the lexicographical matrix
typedef struct element posmat;
struct element {
	float *desc;	// descriptors
	int cx;			// X position of the center of the patch
	int cy;			// Y position of the center of the patch
	int group;		// group ("0" for source or "1" for destination)
};

// Structure Pairs
typedef struct structpair elpair;
struct structpair {
	int x1, y1;		//(x1, y1) pair
	int x2, y2; 	//(x2, y2) pair
	int subgroup;	// subgroup number (distance-based)
	elpair *next;		// pointer to the next pair
};

// Structure used to create groups of SURF keypoints (orientation-based)
typedef struct keygroup group;
struct keygroup {
	elpair *first;	// pointer to the first pair
	int nsubgroups; // number of subgroups (distance-based)
	int numpairs;	// total number of pairs
};

// Shows the image on the screen
void showImageNew(char winame[], IplImage* img) {
	cvNamedWindow(winame, CV_WINDOW_AUTOSIZE);
	cvShowImage(winame, img);
	cvWaitKey(0);
	cvReleaseImage(&img);
	cvDestroyWindow(winame);
}

// Global variables
CvScalar elimg, elimg2;

// ------------------------------------ //
//          STRUCTURE FUNCTIONS         //
// ------------------------------------ //

// Allocates the array of groups
group *allocGroupArray(int numintervals){
	if (360%numintervals != 0){
		printf("Number of intervals has to be a divisor of 360!\n");
		exit(1);
	}

	group *g = (group*) malloc(numintervals*sizeof(group));	// array of groups

	// Initializations
	for (int i = 0; i < numintervals; i++){
		g[i].first = NULL;
		g[i].nsubgroups = 0;
		g[i].numpairs = 0;
	}
	return g;
}

// Deallocates the array of groups
void deallocGroupArray(group *g, int numintervals){
	//for (int i = 0; i < numintervals; i++){
		//elpair *el = g[i].first;
		//elpair *elaux = el;

		//for (int j = 0; j < g[i].numpairs; j++){
			//elaux = elaux->next;
			//free(el);
			//el = elaux;
		//}

		//free(elaux);
		//free(el);
	//}
	free(g);
}

// Adds a pair to specific group (i-th element)
void addPair(group *g, int i, int dist, int x1, int y1, int x2, int y2){
	// Instantiating the new pair
	elpair *p = (elpair*) malloc(sizeof(elpair));

	// normal order of the pair
	p->x1 = x1; p->y1 = y1;
	p->x2 = x2; p->y2 = y2;

	if (g[i].numpairs == 0){ // the first element inside the group
		p->next = NULL;
		g[i].first = p;
		p->subgroup = g[i].nsubgroups+1;
		g[i].nsubgroups++;
	}
	else {
		elpair *el = g[i].first;
		elpair *pre = g[i].first;
		while (el != NULL){
			// If the pair was already added (in the inverse order)
			if ((el->x1 == x2 && el->y1 == y2) ||
					(el->x2 == x1 && el->y2 == y1)){
				break;
			}
			int flag = 0;
			if (euclidean_dist_new(el->x1, el->y1, x1, y1) <= dist &&
					euclidean_dist_new(el->x2, el->y2, x2, y2) <= dist){
				flag = 1;
			}
			else if (euclidean_dist_new(el->x1, el->y1, x2, y2) <= dist &&
						euclidean_dist_new(el->x2, el->y2, x1, y1) <= dist){
				flag = 1;
				// inverting the order of the pair
				p->x1 = x2; p->y1 = y2;
				p->x2 = x1; p->y2 = y1;
			}

			if (flag){
				// Adding the pair to an existing subgroup
				if (el == g[i].first){	// first pair
					p->next = el;
					g[i].first = p;
				}
				else {	// subsequent pairs
					pre->next = p;
					p->next = el;
				}
				p->subgroup = el->subgroup;
				break;
			}
			// New subgroup
			else if (el->next == NULL){
				el->next = p;
				p->next = NULL;
				p->subgroup = g[i].nsubgroups+1;	// new subgroup number
				g[i].nsubgroups++;	// updating the total number of subgroups
				break;
			}
			pre = el;		// predecessor pair
			el = el->next;	// successor pair
		} // end-WHILE

	} // end-ELSE

	g[i].numpairs++;	// updating the number of pairs inside the group i
}

// Computes the total number of subgroups
int getTotalNumSubGroups(group *g, int numintervals){
	int totalsg = 0;
	for(int i = 0; i < numintervals; i++){
		totalsg += g[i].nsubgroups;
	}
	return totalsg;
}

// Computes the boundaries for each subgroup and gets the number of correspondences in them
void getSGBoundsNumCorresp(group *g, int *bound, int *bound2, int *npairssg,
		int numintervals, int height, int width){
	int count = 0;

	// Checking groups
	for(int i = 0; i < numintervals; i++){
		elpair *el = g[i].first;

		// Checking subgroups
		for(int j = 1; j <= g[i].nsubgroups; j++){
			// Bounds for group
			int left = width-1, up = height-1;
			int right = 0, bottom = 0;

			// Bounds for the correspondent group
			int left2 = width-1, up2 = height-1;
			int right2 = 0, bottom2 = 0;

			int count2 = 0;		// counter for the pairs in each subgroup

			while (el != NULL){
				// First subgroup
				if(el->x1 < left)
					left = el->x1;	// left boundary
				if(el->y1 < up)
					up = el->y1;	// up boundary
				if(el->x1 > right)
					right = el->x1;	// right boundary
				if(el->y1 > bottom)
					bottom = el->y1;// bottom boundary

				// Second subgroup
				if(el->x2 < left2)
					left2 = el->x2;	// left boundary
				if(el->y2 < up2)
					up2 = el->y2;	// up boundary
				if(el->x2 > right2)
					right2 = el->x2;// right boundary
				if(el->y2 > bottom2)
					bottom2 = el->y2;// bottom boundary

				count2++;		// updating the number of correspondences in the subgroup
				el = el->next;	// next pair
				if (el == NULL || el->subgroup != j)
					break;		// next pair is from another subgroup
			}

			// Filling the arrays of boundaries
			// Subgroup
			bound[count] = left; bound[count+1] = up;
			bound[count+2] = right; bound[count+3] = bottom;

			// Correspondent subgroup
			bound2[count] = left2; bound2[count+1] = up2;
			bound2[count+2] = right2; bound2[count+3] = bottom2;
			count += 4;

			// Filling the array of number of correspondences in each subgroup
			npairssg[(int)(count/4) - 1] = count2;
		}
		free(el);
	}
}

// Updates groups boundaries
void updateGroups(IplImage* img, int *bound, int *bound2, int ngroups, int growth){
	for (int i = 0; i < 4*ngroups; i += 4){
		// Expanding bounds of the first group
		if (bound[i] - growth >= 0)
			bound[i] -= growth;			// X1
		else bound[i] -= bound[i];

		if (bound[i+1] - growth >= 0)
			bound[i+1] -= growth;		// Y1
		else bound[i+1] -= bound[i+1];

		if (bound[i+2] + growth < img->height)
			bound[i+2] += growth;				// X2
		else bound[i+2] += img->height-bound[i+2]-1;

		if (bound[i+3] + growth < img->width)
			bound[i+3] += growth;				// Y2
		else bound[i+3] += img->height-bound[i+3]-1;

		// Expanding bounds of the correspondent group
		if (bound2[i] - growth >= 0)
			bound2[i] -= growth;			// X1
		else bound2[i] -= bound2[i];

		if (bound2[i+1] - growth >= 0)
			bound2[i+1] -= growth;		// Y1
		else bound2[i+1] -= bound2[i+1];

		if (bound2[i+2] + growth < img->height)
			bound2[i+2] += growth;				// X2
		else bound2[i+2] += img->height-bound2[i+2]-1;

		if (bound2[i+3] + growth < img->width)
			bound2[i+3] += growth;				// Y2
		else bound2[i+3] += img->height-bound2[i+3]-1;
	}
}

// ------------------------------------ //
//         AUXILIARY FUNCTIONS          //
// ------------------------------------ //

// Paints a circle area of the image around a center (cx, cy)
void paintImageCircleArea2(IplImage* img, int bord, int cx, int cy){
	// Creating the circle mask
	IplImage* mask = cvCreateImage(cvSize(2*bord+1, 2*bord+1), IPL_DEPTH_8U, 1);
	cvCircle(mask, cvPoint(bord, bord), bord, CV_RGB(255, 255, 255), -1, 8, 0);

	// Offsets
	int offsetX = cx-bord;	// offset for the mask (x)
	int offsetY = cy-bord;	// offset for the mask (y)

	// Painting pixels
	for (int i = cx-bord; i <= cx+bord; i++){
		for (int j = cy-bord; j <= cy+bord; j++){
			elimg = cvGet2D(mask, i-offsetX, j-offsetY);
			if (elimg.val[0] == 255){
				elimg = cvGet2D(img, i, j);
				for(int c = 0; c < img->nChannels; c++){
					//if (c > 0)
						//el.val[c] = 0;
					//else
						elimg.val[c] = 255;
				}
				cvSet2D(img, i, j, elimg);
			}
		} //end-FOR
	} //end-FOR

	// Releasing mask image
	cvReleaseImage(&mask);
}

// Adds information about an element (patch)
void addPatchLexMat(posmat *lexmat, int pos, float *desc, int cx, int cy, int group){
	lexmat[pos].desc = desc;	// normal histogram
	lexmat[pos].cx = cx;		// x coordinate od the patch
	lexmat[pos].cy = cy;		// y coordinate od the patch
	lexmat[pos].group = group; 	// group
}

// Computes the distance between descriptors
float descriptorDistance(float *desc1, float *desc2, int size){
	float d = 0;
	for (int i = 0; i < size; i++){
		d += (desc1[i] - desc2[i])*(desc1[i] - desc2[i]);
	}
	return sqrt(d);
}

// Draw a box (polygon) and fills it with the specified color (RGB)
void fillMinRect(CvArr* img, CvBox2D box) {
	CvPoint2D32f pointsf[4];
	cvBoxPoints(box, pointsf);
	CvPoint pointsi[4];

    for(int i = 0; i < 4; i++){
        pointsi[i] = cvPointFrom32f(pointsf[i]);
    }

    CvPoint* countours[1] = {pointsi,};
    int countours_n[1] = {4,};
    cvFillPoly(img, countours, countours_n, 1, cvScalar(255, 255, 255));

    //free(countours); // this doesn't seem necessary and might generate bugs
}

// Enhance boundaries by finding the minimum rectangle that surrounds each group
void enhanceBoundaries(IplImage* map, IplImage* img, IplImage* map_enhanced,
		int *bound, int *bound2, int *nDetGroup, int thndet, int nGroups, int factor){

	// Looking for contours in the map image
	for (int i = 0; i < 4*nGroups; i += 4){
		// Checking the number of detections inside the group
		if (nDetGroup[(int)i/4] < thndet)
			continue;

		// Group (source) ----------------------------------------------
		// Taking the number of points in the contour
		int nPoints = 0;
		if (bound[i+2]%2 == 0)
			bound[i+2]--;	// bound correction for X
		if (bound[i+3]%2 == 0)
			bound[i+3]--;	// bound correction for Y

		for(int x = bound[i]/factor; x <= bound[i+2]/factor; x++){
			for(int y = bound[i+1]/factor; y <= bound[i+3]/factor; y++){
				elimg = cvGet2D(map, x, y);
				if (elimg.val[0] == 255)
					nPoints++;
			}
		}

		// Group (correspondence) --------------------------------------
		// Taking the number of points in the contour
		int nPoints2 = 0;
		if (bound2[i+2]%2 == 0)
			bound2[i+2]--;	// bound2 correction for X
		if (bound2[i+3]%2 == 0)
			bound2[i+3]--;	// bound2 correction for Y

		for(int x = bound2[i]/factor; x <= bound2[i+2]/factor; x++){
			for(int y = bound2[i+1]/factor; y <= bound2[i+3]/factor; y++){
				elimg = cvGet2D(map, x, y);
				if (elimg.val[0] == 255)
					nPoints2++;
			}
		}

		if (nPoints < thndet || nPoints2 < thndet)
			continue;

		// Group (source) ----------------------------------------------
		// Getting contour points
		CvPoint *pointArray = (CvPoint*) malloc(nPoints*sizeof(CvPoint));
		int count = 0;

		for(int x = bound[i]/factor; x <= bound[i+2]/factor; x++){
			for(int y = bound[i+1]/factor; y <= bound[i+3]/factor; y++){
				elimg = cvGet2D(map, x, y);
				if (elimg.val[0] == 255){
					pointArray[count].x = y;
					pointArray[count].y = x;
					count++;
				}
			}
		}

		// Calculating the minimum box around the points
		CvMat pointMat;	// matrix of points
		CvBox2D32f box;	// minimum box
		pointMat = cvMat(1, nPoints, CV_32SC2, pointArray);
		box = cvMinAreaRect2(&pointMat, 0);

		// Drawing and filling the minimum rectangle (box) on the image
		fillMinRect(map_enhanced, box);

		// Memory deallocation
		free(pointArray);

		// Group (correspondence) --------------------------------------
		// Getting contour points
		CvPoint *pointArray2 = (CvPoint*) malloc(nPoints2*sizeof(CvPoint));
		count = 0;

		for(int x = bound2[i]/factor; x <= bound2[i+2]/factor; x++){
			for(int y = bound2[i+1]/factor; y <= bound2[i+3]/factor; y++){
				elimg = cvGet2D(map, x, y);
				if (elimg.val[0] == 255){
					pointArray2[count].x = y;
					pointArray2[count].y = x;
					count++;
				}
			}
		}

		// Calculating the minimum box around the points
		pointMat = cvMat(1, nPoints2, CV_32SC2, pointArray2);
		box = cvMinAreaRect2(&pointMat, 0);

		// Drawing and filling the minimum rectangle (box) on the image
		fillMinRect(map_enhanced, box);

		// Memory deallocation
		free(pointArray2);
	}
}

// ------------------------------------ //
//           SORTING FUNCTIONS          //
// ------------------------------------ //

// QuickSort (used in conjunction with RadixSort)
void quickSort(posmat *lexmat, int pos, int left, int right) {
      int i = left, j = right;
      float pivot = lexmat[(int)((left + right)/2)].desc[pos];
      posmat el;

      // partition
      while (i <= j) {
    	  while (lexmat[i].desc[pos] < pivot)
    		  i++;
    	  while (lexmat[j].desc[pos] > pivot)
    		  j--;
    	  if (i <= j) {
    		  el = lexmat[i];
    		  lexmat[i] = lexmat[j];
    		  lexmat[j] = el;
    		  i++; j--;
		}
      }

      // recursion
      if (left < j)
            quickSort(lexmat, pos, left, j);
      if (i < right)
            quickSort(lexmat, pos, i, right);
}

// RadixSort (used in conjunction with QuickSort to sort the lexicog. matrix)
void radixSort(posmat *lexmat, int size, int ncol){
	quickSort(lexmat, 0, 0, size-1);
	for(int i = 1; i < ncol; i++){
		for (int j = 0; j < size-1; j++){
			int begin = j, end = 0;
			while(j < size-1 && (lexmat[j].desc[i-1] == lexmat[j+1].desc[i-1])){
				end++;
				j++;
			}
			quickSort(lexmat, i, begin, begin+end);
		}
	}
}

// Selection Sort for simple array sorting
void selectionSort(float *array, int size){
	int min;
	for (int i = 0; i < size-1; i++){
		min = i;
		for (int j = i+1; j < size; j++) {
		  if(array[j] < array[min]) {
			min = j;
		  }
		}
		if (i != min) {
			// Swapping elements
			float swap = array[i];
			array[i] = array[min];
			array[min] = swap;
		}
	}
}

// ------------------------------------ //
//         DESCRIPTOR FUNCTIONS         //
// ------------------------------------ //

// Calculates the patch's descriptor (whole concentric circles)
float* getDesc1(IplImage* img, int radius, int cx, int cy){
	int factor = radius/4;

	// Creating the circle mask 0 (minimum radius)
	IplImage* mask0 = cvCreateImage(cvSize(2*radius+1, 2*radius+1), IPL_DEPTH_8U, 1);
	cvCircle(mask0, cvPoint(radius, radius), radius-3*factor,
			CV_RGB(255, 255, 255), -1, 8, 0);

	// Creating the circle mask 1
	IplImage* mask1 = cvCreateImage(cvSize(2*radius+1, 2*radius+1), IPL_DEPTH_8U, 1);
	cvCircle(mask1, cvPoint(radius, radius), radius-2*factor,
			CV_RGB(255, 255, 255), -1, 8, 0);

	// Creating the circle mask 2
	IplImage* mask2 = cvCreateImage(cvSize(2*radius+1, 2*radius+1), IPL_DEPTH_8U, 1);
	cvCircle(mask2, cvPoint(radius, radius), radius-factor,
			CV_RGB(255, 255, 255), -1, 8, 0);

	// Creating the circle mask 3 (maximum radius)
	IplImage* mask3 = cvCreateImage(cvSize(2*radius+1, 2*radius+1), IPL_DEPTH_8U, 1);
	cvCircle(mask3, cvPoint(radius, radius), radius, CV_RGB(255, 255, 255), -1, 8, 0);

	// Memory allocation
	float *desc = (float*) calloc(4, sizeof(float));

	// Iterating over the patch
	int offsetX = cx-radius;	// offset for x
	int offsetY = cy-radius;	// offset for y
	int total0 = 0, total1 = 0, total2 = 0, total3 = 0;

	for (int i = cx-radius; i <= cx+radius; i++){
		for (int j = cy-radius; j <= cy+radius; j++){
			// Computing values for mask 1
			elimg = cvGet2D(mask3, i-offsetX, j-offsetY);
			if (elimg.val[0] != 255)
				continue;	// if the mask value is 0, do not consider this position

			elimg = cvGet2D(img, i, j);
			float contval = 0;
			for (int c = 0; c < img->nChannels; c++){
				contval += elimg.val[c];
			}
			desc[3] += contval;
			total3++;

			// Computing values for mask 2
			elimg = cvGet2D(mask2, i-offsetX, j-offsetY);
			if (elimg.val[0] != 255)
				continue;	// if the mask value is 0, do not consider this position
			desc[2] += contval;
			total2++;

			// Computing values for mask 3
			elimg = cvGet2D(mask1, i-offsetX, j-offsetY);
			if (elimg.val[0] != 255)
				continue;	// if the mask value is 0, do not consider this position
			desc[1] += contval;
			total1++;

			// Computing values for mask 4
			elimg = cvGet2D(mask0, i-offsetX, j-offsetY);
			if (elimg.val[0] != 255)
				continue;	// if the mask value is 0, do not consider this position
			desc[0] += contval;
			total0++;
		}
	}

	// Calculating the mean of each circle
	desc[0] /= (total0);
	desc[1] /= (total1);
	desc[2] /= (total2);
	desc[3] /= (total3);

	// Releasing masks
	cvReleaseImage(&mask0);
	cvReleaseImage(&mask1);
	cvReleaseImage(&mask2);
	cvReleaseImage(&mask3);

	return desc;
}

// Calculates the patch's descriptor (whole concentric circles)
// The mean of each channel is taken individually
float* getDesc2(IplImage* img, int radius, int cx, int cy){
	int factor = radius/4;

	// Creating the circle mask 0 (minimum radius)
	IplImage* mask0 = cvCreateImage(cvSize(2*radius+1, 2*radius+1), IPL_DEPTH_8U, 1);
	cvCircle(mask0, cvPoint(radius, radius), radius-3*factor,
			CV_RGB(255, 255, 255), -1, 8, 0);

	// Creating the circle mask 1
	IplImage* mask1 = cvCreateImage(cvSize(2*radius+1, 2*radius+1), IPL_DEPTH_8U, 1);
	cvCircle(mask1, cvPoint(radius, radius), radius-2*factor,
			CV_RGB(255, 255, 255), -1, 8, 0);

	// Creating the circle mask 2
	IplImage* mask2 = cvCreateImage(cvSize(2*radius+1, 2*radius+1), IPL_DEPTH_8U, 1);
	cvCircle(mask2, cvPoint(radius, radius), radius-factor,
			CV_RGB(255, 255, 255), -1, 8, 0);

	// Creating the circle mask 3 (maximum radius)
	IplImage* mask3 = cvCreateImage(cvSize(2*radius+1, 2*radius+1), IPL_DEPTH_8U, 1);
	cvCircle(mask3, cvPoint(radius, radius), radius, CV_RGB(255, 255, 255), -1, 8, 0);

	// Memory allocation
	float *desc = (float*) calloc(12, sizeof(float));

	// Iterating over the patch
	int offsetX = cx-radius;	// offset for x
	int offsetY = cy-radius;	// offset for y
	int total0 = 0, total1 = 0, total2 = 0, total3 = 0;

	for (int i = cx-radius; i <= cx+radius; i++){
		for (int j = cy-radius; j <= cy+radius; j++){
			// Computing values for mask 1
			elimg = cvGet2D(mask3, i-offsetX, j-offsetY);
			if (elimg.val[0] != 255)
				continue;	// if the mask value is 0, do not consider this position

			elimg2 = cvGet2D(img, i, j);
			desc[3] += elimg2.val[0];	// Blue
			desc[7] += elimg2.val[1];	// Green
			desc[11] += elimg2.val[2];	// Red
			total3++;

			// Computing values for mask 2
			elimg = cvGet2D(mask2, i-offsetX, j-offsetY);
			if (elimg.val[0] != 255)
				continue;	// if the mask value is 0, do not consider this position
			desc[2] += elimg2.val[0];	// Blue
			desc[6] += elimg2.val[1];	// Green
			desc[10] += elimg2.val[2];	// Red
			total2++;

			// Computing values for mask 3
			elimg = cvGet2D(mask1, i-offsetX, j-offsetY);
			if (elimg.val[0] != 255)
				continue;	// if the mask value is 0, do not consider this position
			desc[1] += elimg2.val[0];	// Blue
			desc[5] += elimg2.val[1];	// Green
			desc[9] += elimg2.val[2];	// Red
			total1++;

			// Computing values for mask 4
			elimg = cvGet2D(mask0, i-offsetX, j-offsetY);
			if (elimg.val[0] != 255)
				continue;	// if the mask value is 0, do not consider this position
			desc[0] += elimg2.val[0];	// Blue
			desc[4] += elimg2.val[1];	// Green
			desc[8] += elimg2.val[2];	// Red
			total0++;
		}
	}

	// Calculating the mean of each circle and each channel
	//		Blue	----		Green	    ----	 Red
	desc[0] /= (total0); desc[4] /= (total0); desc[8] /= (total0);
	desc[1] /= (total1); desc[5] /= (total1); desc[9] /= (total1);
	desc[2] /= (total2); desc[6] /= (total2); desc[10] /= (total2);
	desc[3] /= (total3); desc[7] /= (total3); desc[11] /= (total3);

	// Releasing masks
	cvReleaseImage(&mask0);
	cvReleaseImage(&mask1);
	cvReleaseImage(&mask2);
	cvReleaseImage(&mask3);

	return desc;
}

// Calculates the normalized patch's descriptor (whole concentric circles)
// The mean of each channel is taken individually
float* getDesc2Norm(IplImage* img, int radius, int cx, int cy){
	int factor = radius/4;

	// Creating the circle mask 0 (minimum radius)
	IplImage* mask0 = cvCreateImage(cvSize(2*radius+1, 2*radius+1), IPL_DEPTH_8U, 1);
	cvCircle(mask0, cvPoint(radius, radius), radius-3*factor,
			CV_RGB(255, 255, 255), -1, 8, 0);

	// Creating the circle mask 1
	IplImage* mask1 = cvCreateImage(cvSize(2*radius+1, 2*radius+1), IPL_DEPTH_8U, 1);
	cvCircle(mask1, cvPoint(radius, radius), radius-2*factor,
			CV_RGB(255, 255, 255), -1, 8, 0);

	// Creating the circle mask 2
	IplImage* mask2 = cvCreateImage(cvSize(2*radius+1, 2*radius+1), IPL_DEPTH_8U, 1);
	cvCircle(mask2, cvPoint(radius, radius), radius-factor,
			CV_RGB(255, 255, 255), -1, 8, 0);

	// Creating the circle mask 3 (maximum radius)
	IplImage* mask3 = cvCreateImage(cvSize(2*radius+1, 2*radius+1), IPL_DEPTH_8U, 1);
	cvCircle(mask3, cvPoint(radius, radius), radius, CV_RGB(255, 255, 255), -1, 8, 0);

	// Memory allocation
	float *desc = (float*) calloc(12, sizeof(float));

	// Iterating over the patch
	int offsetX = cx-radius;	// offset for x
	int offsetY = cy-radius;	// offset for y
	int total0 = 0, total1 = 0, total2 = 0, total3 = 0;

	// getting the max and min values
	float max = 0, min = 255;
	for (int i = cx-radius; i <= cx+radius; i++){
		for (int j = cy-radius; j <= cy+radius; j++){
			elimg = cvGet2D(mask3, i-offsetX, j-offsetY);
			if (elimg.val[0] == 255){
				elimg = cvGet2D(img, i, j);
				// MAX
				if (max < elimg.val[0])
					max = elimg.val[0];
				if (max < elimg.val[1])
					max = elimg.val[1];
				if (max < elimg.val[2])
					max = elimg.val[2];
				// MIN
				if (min > elimg.val[0])
					min = elimg.val[0];
				if (min > elimg.val[1])
					min = elimg.val[1];
				if (min > elimg.val[2])
					min = elimg.val[2];
			}
		}
	}

	for (int i = cx-radius; i <= cx+radius; i++){
		for (int j = cy-radius; j <= cy+radius; j++){
			// Computing values for mask 1
			elimg = cvGet2D(mask3, i-offsetX, j-offsetY);
			if (elimg.val[0] != 255)
				continue;	// if the mask value is 0, do not consider this position

			elimg2 = cvGet2D(img, i, j);
			desc[3] += (elimg2.val[0]-min)/(max-min);	// Blue
			desc[7] += (elimg2.val[1]-min)/(max-min);	// Green
			desc[11] += (elimg2.val[2]-min)/(max-min);	// Red
			total3++;

			// Computing values for mask 2
			elimg = cvGet2D(mask2, i-offsetX, j-offsetY);
			if (elimg.val[0] != 255)
				continue;	// if the mask value is 0, do not consider this position
			desc[2] += (elimg2.val[0]-min)/(max-min);	// Blue
			desc[6] += (elimg2.val[1]-min)/(max-min);	// Green
			desc[10] += (elimg2.val[2]-min)/(max-min);	// Red
			total2++;

			// Computing values for mask 3
			elimg = cvGet2D(mask1, i-offsetX, j-offsetY);
			if (elimg.val[0] != 255)
				continue;	// if the mask value is 0, do not consider this position
			desc[1] += (elimg2.val[0]-min)/(max-min);	// Blue
			desc[5] += (elimg2.val[1]-min)/(max-min);	// Green
			desc[9] += (elimg2.val[2]-min)/(max-min);	// Red
			total1++;

			// Computing values for mask 4
			elimg = cvGet2D(mask0, i-offsetX, j-offsetY);
			if (elimg.val[0] != 255)
				continue;	// if the mask value is 0, do not consider this position
			desc[0] += (elimg2.val[0]-min)/(max-min);	// Blue
			desc[4] += (elimg2.val[1]-min)/(max-min);	// Green
			desc[8] += (elimg2.val[2]-min)/(max-min);	// Red
			total0++;
		}
	}

	// Calculating the mean of each circle and each channel
	//		Blue	----		Green	    ----	 Red
	desc[0] /= (total0); desc[4] /= (total0); desc[8] /= (total0);	// mask 4
	desc[1] /= (total1); desc[5] /= (total1); desc[9] /= (total1);	// mask 3
	desc[2] /= (total2); desc[6] /= (total2); desc[10] /= (total2);	// mask 2
	desc[3] /= (total3); desc[7] /= (total3); desc[11] /= (total3);	// mask 1

	// Releasing masks
	cvReleaseImage(&mask0);
	cvReleaseImage(&mask1);
	cvReleaseImage(&mask2);
	cvReleaseImage(&mask3);

	return desc;
}

// Calculates the patch's descriptor (whole concentric circles)
// The mean, median, max and min values of each channel are taken individually
float* getDesc3(IplImage* img, int radius, int cx, int cy){
	int factor = radius/4;

	// Creating the circle mask 0 (minimum radius)
	IplImage* mask0 = cvCreateImage(cvSize(2*radius+1, 2*radius+1), IPL_DEPTH_8U, 1);
	cvCircle(mask0, cvPoint(radius, radius), radius-3*factor,
			CV_RGB(255, 255, 255), -1, 8, 0);

	// Creating the circle mask 1
	IplImage* mask1 = cvCreateImage(cvSize(2*radius+1, 2*radius+1), IPL_DEPTH_8U, 1);
	cvCircle(mask1, cvPoint(radius, radius), radius-2*factor,
			CV_RGB(255, 255, 255), -1, 8, 0);

	// Creating the circle mask 2
	IplImage* mask2 = cvCreateImage(cvSize(2*radius+1, 2*radius+1), IPL_DEPTH_8U, 1);
	cvCircle(mask2, cvPoint(radius, radius), radius-factor,
			CV_RGB(255, 255, 255), -1, 8, 0);

	// Creating the circle mask 3 (maximum radius)
	IplImage* mask3 = cvCreateImage(cvSize(2*radius+1, 2*radius+1), IPL_DEPTH_8U, 1);
	cvCircle(mask3, cvPoint(radius, radius), radius, CV_RGB(255, 255, 255), -1, 8, 0);

	// Memory allocation
	float *desc = (float*) malloc(24*sizeof(float));
	for (int i = 0; i < 12; i++){
		desc[i] = 0;
	}

	// Auxiliary matrix to compute the medians per circle and channel
	int totmax = (radius*2+1)*(radius*2+1);
	float **matmed = (float**) malloc(12*sizeof(float*));
	for (int i = 0; i < 12; i++){
		matmed[i] = (float*) calloc(totmax, sizeof(float));
	}

	// Iterating over the patch
	int offsetX = cx-radius;	// offset for x
	int offsetY = cy-radius;	// offset for y
	int total0 = 0, total1 = 0, total2 = 0, total3 = 0;

	for (int i = cx-radius; i <= cx+radius; i++){
		for (int j = cy-radius; j <= cy+radius; j++){
			// Computing values for mask 1
			elimg = cvGet2D(mask3, i-offsetX, j-offsetY);
			if (elimg.val[0] != 255)
				continue;	// if the mask value is 0, do not consider this position

			elimg2 = cvGet2D(img, i, j);

			// Mean
			desc[3] += elimg2.val[0];	// Blue
			desc[7] += elimg2.val[1];	// Green
			desc[11] += elimg2.val[2];	// Red

			// Median
			matmed[3][total3] = elimg2.val[0];	// Blue
			matmed[7][total3] = elimg2.val[1];	// Green
			matmed[11][total3] = elimg2.val[2];// Red
			total3++;

			// Computing values for mask 2
			elimg = cvGet2D(mask2, i-offsetX, j-offsetY);
			if (elimg.val[0] != 255)
				continue;	// if the mask value is 0, do not consider this position

			// Mean
			desc[2] += elimg2.val[0];	// Blue
			desc[6] += elimg2.val[1];	// Green
			desc[10] += elimg2.val[2];	// Red

			// Median
			matmed[2][total2] = elimg2.val[0];	// Blue
			matmed[6][total2] = elimg2.val[1];	// Green
			matmed[10][total2] = elimg2.val[2];// Red
			total2++;

			// Computing values for mask 3
			elimg = cvGet2D(mask1, i-offsetX, j-offsetY);
			if (elimg.val[0] != 255)
				continue;	// if the mask value is 0, do not consider this position

			// Mean
			desc[1] += elimg2.val[0];	// Blue
			desc[5] += elimg2.val[1];	// Green
			desc[9] += elimg2.val[2];	// Red

			// Median
			matmed[1][total1] = elimg2.val[0];	// Blue
			matmed[5][total1] = elimg2.val[1];	// Green
			matmed[9][total1] = elimg2.val[2];	// Red
			total1++;

			// Computing values for mask 4
			elimg = cvGet2D(mask0, i-offsetX, j-offsetY);
			if (elimg.val[0] != 255)
				continue;	// if the mask value is 0, do not consider this position

			// Mean
			desc[0] += elimg2.val[0];	// Blue
			desc[4] += elimg2.val[1];	// Green
			desc[8] += elimg2.val[2];	// Red

			// Median
			matmed[0][total0] = elimg2.val[0];	// Blue
			matmed[4][total0] = elimg2.val[1];	// Green
			matmed[8][total0] = elimg2.val[2];	// Red
			total0++;
		}
	}

	// Calculating the MEAN of each circle and each channel
	//		Blue
	desc[0] /= (total0);
	desc[1] /= (total1);
	desc[2] /= (total2);
	desc[3] /= (total3);

	//		Green
	desc[4] /= (total0);
	desc[5] /= (total1);
	desc[6] /= (total2);
	desc[7] /= (total3);

	//		Red
	desc[8] /= (total0);
	desc[9] /= (total1);
	desc[10] /= (total2);
	desc[11] /= (total3);

	// Calculating the MEDIAN of each circle and each channel
	//		Blue
	selectionSort(matmed[0], total0);
	selectionSort(matmed[1], total1);
	selectionSort(matmed[2], total2);
	selectionSort(matmed[3], total3);
	desc[12] = matmed[0][(int)total0/2];
	desc[13] = matmed[1][(int)total1/2];
	desc[14] = matmed[2][(int)total2/2];
	desc[15] = matmed[3][(int)total3/2];

	//		Green
	selectionSort(matmed[4], total0);
	selectionSort(matmed[5], total1);
	selectionSort(matmed[6], total2);
	selectionSort(matmed[7], total3);
	desc[16] = matmed[4][(int)total0/2];
	desc[17] = matmed[5][(int)total1/2];
	desc[18] = matmed[6][(int)total2/2];
	desc[19] = matmed[7][(int)total3/2];

	//		Red
	selectionSort(matmed[8], total0);
	selectionSort(matmed[9], total1);
	selectionSort(matmed[10], total2);
	selectionSort(matmed[11], total3);
	desc[20] = matmed[8][(int)total0/2];
	desc[21] = matmed[9][(int)total1/2];
	desc[22] = matmed[10][(int)total2/2];
	desc[23] = matmed[11][(int)total3/2];

	// Deallocating memory
	for (int i = 0; i < 12; i++){
		free(matmed[i]);
	}
	free(matmed);

	// Releasing masks
	cvReleaseImage(&mask0);
	cvReleaseImage(&mask1);
	cvReleaseImage(&mask2);
	cvReleaseImage(&mask3);

	return desc;
}

// ------------------------------------ //
//        CMF DETECTION FUNCTION        //
// ------------------------------------ //

// Scans the final Lexicographical Matrix towards similar patches
void detect(IplImage* img, IplImage* map, posmat *lexmat, int *ndet, int bord,
		int size, int descsize, int neigh, float tsim, float mindist){

	for (int i = 0; i < size-1; i++){
		float dist, magnitude;
		int j = i+1;
		while (j < size && j <= i+neigh){
			// Checking whether the blocks are from the same region (group)
			if (lexmat[i].group == lexmat[j].group){
				j++;
				continue;
			}

			// Calculating the magnitude (physical distance) between patches
			magnitude = euclidean_dist_new(lexmat[i].cx, lexmat[i].cy,
					lexmat[j].cx, lexmat[j].cy);
			if (magnitude <= mindist){
				j++;
				continue;
			}

			// Computing the distance between histograms
			dist = descriptorDistance(lexmat[i].desc, lexmat[j].desc, descsize);

			// Checking the threshold
			if (dist <= tsim){
				CvScalar el = cvGet2D(map, lexmat[i].cx, lexmat[i].cy);
				if (el.val[0] != 255){
					(*ndet)++;	// updating the number of detections in the group
					paintImageCircleArea2(map, bord, lexmat[i].cx, lexmat[i].cy);
					paintImageCircleArea2(map, bord, lexmat[j].cx, lexmat[j].cy);
				}
			}
			j++;
		}
	}
}

// Performs the pyramidal decomposition and returns an array of images
IplImage** pyrDecompose(IplImage* img, int numlevels){
	IplImage** arrayimg = (IplImage**) malloc(numlevels * sizeof(IplImage*));
	arrayimg[0] = img;	// first level is the original image

	for (int i = 1; i < numlevels; i++){
		if (i == 1){
			arrayimg[1] = cvCreateImage(cvSize(img->width/2, img->height/2),
					img->depth, img->nChannels);
			cvPyrDown(img, arrayimg[1], CV_GAUSSIAN_5x5);
		}
		else {
			arrayimg[i] = cvCreateImage(cvSize(arrayimg[i-1]->width/2,
					arrayimg[i-1]->height/2), img->depth, img->nChannels);
			cvPyrDown(arrayimg[i-1], arrayimg[i], CV_GAUSSIAN_5x5);
		}
	}

	return arrayimg;
}

// Performs the votation scheme to find the final image map
void votationMap(IplImage** arrayimg, IplImage* map_final, int numlevels, int thpyr){
	for (int n = 1; n < numlevels; n++){
		IplImage* aux = cvCreateImage(cvSize(arrayimg[n]->width,
				arrayimg[n]->height), 8, 3);
		cvCopy(arrayimg[n], aux, NULL);
		cvReleaseImage(&arrayimg[n]);

		arrayimg[n] = cvCreateImage(cvSize(arrayimg[0]->width,
				arrayimg[0]->height), 8, 3);
		cvResize(aux, arrayimg[n]);
		cvReleaseImage(&aux);
	}

	for (int i = 0; i < arrayimg[0]->height; i++){
		for (int j = 0; j < arrayimg[0]->width; j++){
			elimg = cvGet2D(arrayimg[0], i, j);

			if (elimg.val[0] != 0){
				int count = 1;
				for (int n = 1; n < numlevels; n++){
					elimg2 = cvGet2D(arrayimg[n], i, j);
					if (elimg2.val[0] != 0)
						count++;
				}

				if (count >= thpyr){
					elimg = cvGet2D(map_final, i, j);
					elimg.val[0] = 255;
					cvSet2D(map_final, i, j, elimg);
				}
			}
		}
	}
}

// ------------------------------------ //
//            MAIN FUNCTION             //
// ------------------------------------ //

IplImage* ourmethod(char imgpath[]){
	// Reading the image -------------------------------------------------------------------
	IplImage* img_orig = cvLoadImage(imgpath);

	// Converting to HSV -------------------------------------------------------------------
	IplImage* img = cvCreateImage(cvSize(img_orig->width, img_orig->height), 8, 3);
	cvCvtColor(img_orig, img, CV_BGR2HSV);	// RGB, BGR, GRAY, HSV, YCrCb, XYZ, Lab, Luv, HLS
	cvReleaseImage(&img_orig);

	// Converting to Grayscale -------------------------------------------------------------
	IplImage* img_gray = cvCreateImage(cvSize(img->width, img->height), 8, 1);
	cvCvtColor(img, img_gray, CV_BGR2GRAY);

	// Important variables for Detection part ----------------------------------------------
	int psize = 9; 			// patch size
	int bord = psize/2;		// border size
	int desctype = '3';		// descriptor type
	int neigh = 5;			// number of neighboring lines to check
	float tsim = 0.05;		// threshold distance to evaluate similarity
	float mindist = 30;		// minimum physical distance between similar patches
	int thndet = 3;			// minimum number of detections in the group
	int thnpairs = 3;		// NEW: minimum number of pairs of keypoints in a subgroup
	int numlevels = 3;		// number of levels in the pyramidal decomposition
	int thpyr = 2;			// minimum number of detection through the scales

	// Important variables for SURF part ---------------------------------------------------
	int numpairs;			// number of keypoint pairs - not a threshold
	int thessian = 0;		// hessian threshold (0 for the max. number of keypoints)
	float thfactor = 0.6;	// factor threshold to compare two keypoints
	int tphy = 50;			// minimum physical distance between keypoints
	int tphy2 = 50;			// maximum distance between a keypoint and its closest
								// matching pair in a subgroup
	int numintervals = 20;	// number of orientation intervals
	int growth = 10;		// subgroup size maximum growth in each direction

	// ------------------------------------------------------------------ //
	// PART I - SURF correspondences detection and groups of keypoints	  //
	// ------------------------------------------------------------------ //

	// Extracting SURF keyppoints and correspondences --------------------------------------
	int **pairs = getSURFPointsNew(img_gray, &numpairs, thessian, thfactor, tphy);

	// Creating groups of keypoint correspondences
	group *g = allocGroupArray(numintervals);	// groups of keypoints

	for (int i = 0; i < numpairs*2; i+=2){
		// Computing the angle between the correspondent keypoints
		float dx = abs(pairs[1][i] - pairs[0][i]);
		float dy = abs(pairs[1][i+1] - pairs[0][i+1]);
		float angle = (atan(dy/dx)*180.0)/M_PI;
		if (angle < 0){	// it seems that this is unnecessary
			angle += 360.0;	// atan can return negative angles
		}

		// Adding the keypoint to a group
		int ngroup = (int)(angle*numintervals)/360;
		addPair(g, ngroup, tphy2, pairs[0][i], pairs[0][i+1], pairs[1][i],
				pairs[1][i+1]);
	}

	// Deallocating memory
	free(pairs[0]); free(pairs[1]); free(pairs);

	// Getting the total number of subgroups
	int totalsg = getTotalNumSubGroups(g, numintervals);

	// Getting subgroups boundaries and the number of correspondences in each subgroup
	int *bound = (int*) malloc((4*totalsg)*sizeof(int)); // bounds for original subgroups
	int *bound2 = (int*) malloc((4*totalsg)*sizeof(int));// bounds for correspondent subgroups
	int *npairssg = (int*) malloc(totalsg*sizeof(int));	// number of correspondences in each subgroup
	getSGBoundsNumCorresp(g, bound, bound2, npairssg, numintervals,
			img_gray->height, img_gray->width);

	// Updating keypoints groups (expanding each according to the growth rate)
	updateGroups(img, bound, bound2, totalsg, growth);

	// Freeing structure array
	deallocGroupArray(g, numintervals);

	// Releasing the gray image
	cvReleaseImage(&img_gray);

	// ------------------------------------------------------------------- //
	// PART II - Traditional CMF Detection method over the groups found    //
	// ------------------------------------------------------------------- //

	// Using pyramid decomposition and getting an array of images/map
	IplImage** arrayimg = pyrDecompose(img, numlevels);

	// Running the method for each image level
	for (int n = 0; n < numlevels; n++){
		// Checking forgeries in each group;
		IplImage* map = cvCreateImage(cvSize(arrayimg[n]->width, arrayimg[n]->height), 8, 3);
		cvZero(map);	// Map Image
		int *nDetGroup = (int*) calloc(totalsg, sizeof(int));	// number of detections per group
		int factor = (int) powf(2.0, (float)n);	// factor to resize the group

		// Changing the distance by a factor
		mindist /= factor;

		for (int k = 0; k < 4*totalsg; k += 4){
			// Checking if the subgroups have a minimum number of correspondence pairs
			if (npairssg[(int)(k/4)] < thnpairs)
				continue;

			// Finding the number of lines of the lexicographical matrix (group 1)
			int g1height = (int)(bound[k+2]-bound[k]+1)/factor;
			int g1width = (int)(bound[k+3]-bound[k+1]+1)/factor;
			int size1 = (g1height-psize+1)*(g1width-psize+1);

			// Finding the number of lines of the lexicographical matrix (group 2)
			int g2height = (int)(bound2[k+2]-bound2[k]+1)/factor;
			int g2width = (int)(bound2[k+3]-bound2[k+1]+1)/factor;
			int size2 = (g2height-psize+1)*(g2width-psize+1);

			// Checking whether the group has at least the size of the sliding window
			if (g1height < psize || g1width < psize ||
					g2height < psize || g2width < psize){
				continue;
			}

			// Allocating the final lexicographical matrix -----------------------------------------
			int size = size1 + size2;
			posmat *lexmat = (posmat*) malloc(size*sizeof(posmat));

			// Iterating over the image ------------------------------------------------------------
			int next = 0;	// stores the next position of lexmat to be filled
			int descsize = 0;	// descriptor size

			// Group 1 (source)
			int boundX = (int)bound[k]/factor;
			int boundY = (int)bound[k+1]/factor;
			for (int i = bord+boundX; i < boundX+g1height-bord; i++) {
				for (int j = bord+boundY; j < boundY+g1width-bord; j++) {
					float *desc;
					if (img->nChannels > 1){
						switch (desctype){
						case '1':
							descsize = 4;
							desc = getDesc1(arrayimg[n], bord, i, j);
							break;
						case '2':
							descsize = 12;
							desc = getDesc2(arrayimg[n], bord, i, j);
							break;
						case '3':
							descsize = 12;
							desc = getDesc2Norm(arrayimg[n], bord, i, j);
							break;
						case '4':
							descsize = 24;
							desc = getDesc3(arrayimg[n], bord, i, j);
							break;
						}
					}
					else {
						descsize = 4;
						desc = getDesc1(arrayimg[n], bord, i, j);
					}
					addPatchLexMat(lexmat, next, desc, i, j, 0);
					next++;
				}
			}

			// Group 2 (correspondence)
			int boundX2 = (int)bound2[k]/factor;
			int boundY2 = (int)bound2[k+1]/factor;
			for (int i = bord+boundX2; i < boundX2+g2height-bord; i++) {
				for (int j = bord+boundY2; j < boundY2+g2width-bord; j++) {
					float *desc;
					if (img->nChannels > 1){
						switch (desctype){
						case '1':
							descsize = 4;
							desc = getDesc1(arrayimg[n], bord, i, j);
							break;
						case '2':
							descsize = 12;
							desc = getDesc2(arrayimg[n], bord, i, j);
							break;
						case '3':
							descsize = 12;
							desc = getDesc2Norm(arrayimg[n], bord, i, j);
							break;
						case '4':
							descsize = 24;
							desc = getDesc3(arrayimg[n], bord, i, j);
							break;
						}
					}
					else {
						descsize = 4;
						desc = getDesc1(arrayimg[n], bord, i, j);
					}
					addPatchLexMat(lexmat, next, desc, i, j, 1);
					next++;
				}

			}


			// Sorting the lexicographical matrix --------------------------------------------------
			radixSort(lexmat, size, descsize);

			// Looking for similar patches ---------------------------------------------------------
			int nDet = 0;
			detect(arrayimg[n], map, lexmat, &nDet, bord, size, descsize, neigh,
					tsim, mindist);

			// Updating the number of detections in the current group ------------------------------
			nDetGroup[(int)(k/4)] += nDet;	// OLHAR AQUI!!!

			// Lexmat deallocation
			for (int i = 0; i < size; i++){
				free(lexmat[i].desc);
			}
			free(lexmat);

		}

		// Drawing the new enhanced boundaries
		IplImage* map_enhanced = cvCreateImage(cvSize(arrayimg[n]->width,
				arrayimg[n]->height), 8, 3);
		cvZero(map_enhanced);	// Enhanced Map Image
		enhanceBoundaries(map, arrayimg[n], map_enhanced, bound, bound2,
				nDetGroup, thndet, totalsg, factor);

		// Releasing images
		cvReleaseImage(&map);
		cvReleaseImage(&arrayimg[n]);
		free(nDetGroup);

		// The arrayimg[n] now points to map_enhanced
		arrayimg[n] = map_enhanced;

	}// end-FOR

	// Calculating the final map by a votation scheme
	IplImage* map_final = cvCreateImage(cvSize(arrayimg[0]->width,
			arrayimg[0]->height), 8, 1);
	cvZero(map_final);	// Enhanced Map Image
	votationMap(arrayimg, map_final, numlevels, thpyr);

	// Deallocating memory
	free(bound);
	free(bound2);
	free(npairssg);

	// Releasing Map Images
	for (int n = 0; n < numlevels; n++){
		cvReleaseImage(&arrayimg[n]);
	}
	free(arrayimg);

	return map_final;
}
