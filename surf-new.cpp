//#include <cv.h>
//#include <compat.hpp>
#include <highgui.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include "surf-new.h"
//#include "nonfree/features2d.hpp"
//#include "nonfree/nonfree.hpp"

using namespace std;
//using namespace cv;

//cv::initModule_nonfree();

static CvScalar colors[] = { { { 0, 0, 255 } }, { { 0, 128, 255 } }, { { 0,
			255, 255 } }, { { 0, 255, 0 } }, { { 255, 128, 0 } }, { { 255, 255,
			0 } }, { { 255, 0, 0 } }, { { 255, 0, 255 } },
			{ { 255, 255, 255 } } };

// Calculates the Euclidean Distance between two points (x1, y1) and (x2, y2)
float euclidean_dist_new(int x1, int y1, int x2, int y2){
	return sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
}

// Compares two surf descriptors
double compareSURFDescriptorsNew(float* d1, float* d2, double best, int length) {
	double total_cost = 0;
	assert( length % 4 == 0 );
	for (int i = 0; i < length; i += 4) {
		double t0 = d1[i] - d2[i];
		double t1 = d1[i + 1] - d2[i + 1];
		double t2 = d1[i + 2] - d2[i + 2];
		double t3 = d1[i + 3] - d2[i + 3];
		total_cost += t0 * t0 + t1 * t1 + t2 * t2 + t3 * t3;
		if (total_cost > best)
			break;	// stop if it's higher than best, because these descriptors are not similar
	}
	return total_cost;
}

// Uses a Naive Nearest Neighbor search to find correspondences between keypoint descriptors
int naiveNearestNeighborNew(float* vec, int laplacian, CvSeq* image_keypoints,
		CvSeq* image_descriptors, float thfactor, int indice) {
	int length = (int) (image_descriptors->elem_size / sizeof(float));
	int i, neighbor = -1;
	double d, dist1 = 1e6, dist2 = 1e6;
	CvSeqReader reader, kreader;
	cvStartReadSeq(image_keypoints, &kreader, 0);
	cvStartReadSeq(image_descriptors, &reader, 0);

	for (i = 0; i < image_descriptors->total; i++) {
		CvSURFPoint* kp = (CvSURFPoint*) kreader.ptr;
		float* mvec = (float*) reader.ptr;
		CV_NEXT_SEQ_ELEM(kreader.seq->elem_size, kreader);
		CV_NEXT_SEQ_ELEM(reader.seq->elem_size, reader);

		if (i == indice || laplacian != kp->laplacian)
			continue;

		d = compareSURFDescriptorsNew(vec, mvec, dist2, length);
		if (d < dist1) {
			dist2 = dist1;
			dist1 = d;
			neighbor = i;
		} else if (d < dist2)
			dist2 = d;
	}

	if (dist1 < thfactor * dist2)
		return neighbor;
	return -1;
}

// Finds pairs using an exhaustive search approach
void findPairsNew(CvSeq* imageKeypoints, CvSeq* imageDescriptors,
		vector<int>& ptpairs, float threshold_factor) {
	CvSeqReader reader, kreader;
	cvStartReadSeq(imageKeypoints, &kreader);
	cvStartReadSeq(imageDescriptors, &reader);

	for (int i = 0; i < imageDescriptors->total; i++) {
		CvSURFPoint* kp = (CvSURFPoint*) kreader.ptr;
		float* descriptor = (float*) reader.ptr;
		CV_NEXT_SEQ_ELEM(kreader.seq->elem_size, kreader);
		CV_NEXT_SEQ_ELEM(reader.seq->elem_size, reader);
		int nearest_neighbor = naiveNearestNeighborNew(descriptor, kp->laplacian,
				imageKeypoints, imageDescriptors, threshold_factor, i);
		if (nearest_neighbor >= 0) {
			ptpairs.push_back(i);
			ptpairs.push_back(nearest_neighbor);
		}
	}
}

// Shows the SURF correspondences in the image
void getSURFCorrespondences(CvSeq* imageKeypoints, vector<int>& ptpairs,
		int *pairs1, int *pairs2, int *numpairs, float thdist){

	int count = 0;
	for (int i = 0; i < (int) ptpairs.size(); i += 2) {
		CvSURFPoint* r1 = (CvSURFPoint*) cvGetSeqElem(imageKeypoints, ptpairs[i]);
		CvSURFPoint* r2 = (CvSURFPoint*) cvGetSeqElem(imageKeypoints, ptpairs[i+1]);
		if ((r1->pt.x != r2->pt.x || r1->pt.y != r2->pt.y) &&
				(euclidean_dist_new(cvRound(r1->pt.x), cvRound(r1->pt.y),
						cvRound(r2->pt.x), cvRound(r2->pt.y)) > thdist)){
			pairs1[count] = cvRound(r1->pt.y);
			pairs1[count+1] = cvRound(r1->pt.x);
			pairs2[count] = cvRound(r2->pt.y);
			pairs2[count+1] = cvRound(r2->pt.x);
			count += 2;	// updating the number of pairs
		}
	}

	*numpairs = count/2;
}

// Shows the SURF keypoints in the image
void getSURFKeypoints(CvSeq* imageKeypoints){
	for (int i = 0; i < imageKeypoints->total; i++) {
		CvSURFPoint* r = (CvSURFPoint*) cvGetSeqElem(imageKeypoints, i);
		CvPoint center;
		center.x = cvRound(r->pt.x);
		center.y = cvRound(r->pt.y);
	}
}

// Extracts and gets all surf keypoints (not used in patchmatch)
int **getSURFPointsNew(IplImage* image,	int *numpairs, float thhessian,
		float thfactor, float thdist) {
	// Variables
	vector<int> ptpairs;
	CvMemStorage* storage = cvCreateMemStorage(0);

	// Instantiating keypoints and descriptors arrays
	CvSeq *imageKeypoints, *imageDescriptors = 0;

	// Defining SURF parameters (threshold and type of descriptor)
	CvSURFParams params = cvSURFParams(thhessian, 1);

	// Performing the SURF extraction
	cvExtractSURF(image, 0, &imageKeypoints, &imageDescriptors, storage, params);

	// Finding correspondences
	findPairsNew(imageKeypoints, imageDescriptors, ptpairs, thfactor);

	// Creating arrays
	int **pairs = (int**) malloc(2*sizeof(int*));
	pairs[0] = (int*) malloc((2*ptpairs.size())*sizeof(int));
	pairs[1] = (int*) malloc((2*ptpairs.size())*sizeof(int));

	// getting SURF correspondences and taking the correspondent pairs
	getSURFCorrespondences(imageKeypoints, ptpairs, pairs[0],
		pairs[1], numpairs, thdist);

	// Releasing structures - these instructions break the method for large images
	cvClearSeq(imageKeypoints);
	cvClearSeq(imageDescriptors);
	cvReleaseMemStorage(&storage);

	return pairs;
}
