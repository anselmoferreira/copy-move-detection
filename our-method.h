/* -----------------------------------------------------------------
 * our-method.h
 * -----------------------------------------------------------------
 * This library contains our method to detect copy-move forgeries.
 * The method uses SURF correspondences and a circular block to
 * detect rotated and resized duplicated patches.
 * -----------------------------------------------------------------
 */

/* -----------------------------
 *  Defining the functions
 * ----------------------------- */
extern IplImage* ourmethod(char imgpath[]);
