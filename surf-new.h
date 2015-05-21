/* -----------------------------------------------------------------
 * surf-new.h
 * -----------------------------------------------------------------
 * This library contains methods to extract SURF keypoints from
 * an image, and to get only the surf correspondences.
 * -----------------------------------------------------------------
 */

/* -----------------------------
 *  Defining the SURF functions
 * ----------------------------- */

using namespace std;

extern int **getSURFPointsNew(IplImage* image, int *numpairs, float thhessian,
		float thfactor, float thdist);
extern float euclidean_dist_new(int x1, int y1, int x2, int y2);
