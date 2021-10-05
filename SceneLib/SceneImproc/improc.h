// Image processing functions

// Should this be a class with constants defined at initialisation?
const double BESTPATCHSEARCHFRACTION = 0.5;  /* The fraction of the width and
				   	        height of a total image which
						we will search for a best
						patch in find_best_patch */

const double CORRTHRESH2 = 0.4;      /* Threshold for stereo correlation using 
		              	        and_correlate2 */
const int LINESEARCHWIDTH = 8;       /* Half-width of the strip we will
			       	        search for stereo matches */
const double CORRELATION_SIGMA_THRESHOLD = 10.0;
                                     /* Value for the standard deviation
					of the intensity values in a
					patch below which we deem it
					unsuitable for correlation */
const double NO_SIGMA = 3.0;         /* Number of standard deviations to search
					within images */

int evaluate_patch(Hor_Image *image, int u, int v, int BOXSIZE);
int find_best_patch(Hor_Image *image, int *ubest, int *vbest, double *evbest, 
		    int BOXSIZE, int U0, int V0);
int find_best_n_patches(Hor_Image *image, int n, int *ubest, int *vbest, 
                        double *evbest, int BOXSIZE, int U0, int V0);

int epipolar_search(Hor_Image *image, Hor_Image *patch, 
		    double m, double c, int *u, int *v, int BOXSIZE);
int elliptical_search(Hor_Image *image, Hor_Image *patch,
		      Hor_Matrix *PuInv,
		      int *u, int *v, double U0, double V0, int BOXSIZE);
int xor_elliptical_search_region(Hor_Image *image, Hor_Matrix *PuInv,
				 double U0, double V0, int BOXSIZE);
