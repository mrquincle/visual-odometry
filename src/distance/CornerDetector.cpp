/**
 * @brief 
 * @file CornerDetector.cpp
 *
 * This file is created at Almende B.V. It is open-source software and part of the Common 
 * Hybrid Agent Platform (CHAP). A toolbox with a lot of open-source tools, ranging from 
 * thread pools and TCP/IP components to control architectures and learning algorithms. 
 * This software is published under the GNU Lesser General Public license (LGPL).
 *
 * It is not possible to add usage restrictions to an open-source license. Nevertheless,
 * we personally strongly object to this software being used by the military, in factory 
 * farming, for animal experimentation, or anything that violates the Universal 
 * Declaration of Human Rights.
 *
 * Copyright © 2012 Anne van Rossum <anne@almende.com>
 *
 * @author  Anne C. van Rossum
 * @date    Oct 1, 2012
 * @project Replicator FP7
 * @company Almende B.V.
 * @case    modular robotics / sensor fusion
 */

/* **************************************************************************************
 * Configuration options
 * **************************************************************************************/

// Turn on/off saving images to disk
#define STORE_IMAGES		1

// Use "fast" corner detection (or Harris)
#define USE_FAST			1

/* **************************************************************************************
 * Includes and typedefs
 * **************************************************************************************/

#include <CornerDetector.h>
#include <iostream>
#include <sstream>
#include <CRawImage.h>
#include <cassert>

#include <convolve.h>
#include <fast/fast.h>

#if STORE_IMAGES == 0
#undef STORE_IMAGES
#endif

#if USE_FAST == 0
#undef USE_FAST
#endif

// pixel values are 8 bit fields (uint8_t would require <inttypes.h>)
#define PIXELTYPE unsigned char

using namespace std;

/* **************************************************************************************
 * Implementation of CornerDetector
 * **************************************************************************************/

CornerDetector::CornerDetector(): img(NULL), dx(NULL), dy(NULL), ddx(NULL), ddy(NULL),
		dxy(NULL), dH(NULL), dDisp(NULL), index(0) {

}

CornerDetector::~CornerDetector() {

}

/**
 * The image handed over will never be deallocated. However, when you provide a new
 * image with different dimensions, the internal data structures will be deleted and
 * build up again.
 */
void CornerDetector::SetImage(CRawImage *img) {
	assert (img->isMonochrome());
	if (this->img == NULL) {
		this->img = img;
		goto create_temps;
	}

	// we do not need to deallocate if new image is same size as old one
	if (img->getsize() == this->img->getsize()) {
		cout << "Images are same size as old ones: won't reallocate memory" << endl;
		return;
	}

	// delete all temporary images and create anew
	cout << "Delete old temporary images" << endl;
	delete [] dx;
	delete [] dy;
	delete [] ddx;
	delete [] ddy;
	delete [] dxy;
	delete [] dH;
	delete [] dDisp;
create_temps:
	cout << "Create new temporary images" << endl;
	dx = new CRawImage(img->getwidth(), img->getheight(), 1);
	dy = new CRawImage(img->getwidth(), img->getheight(), 1);
	ddx = new CRawImage(img->getwidth(), img->getheight(), 1);
	ddy = new CRawImage(img->getwidth(), img->getheight(), 1);
	dxy = new CRawImage(img->getwidth(), img->getheight(), 1);
	dH = new CRawImage(img->getwidth(), img->getheight(), 1);
	dDisp = new CRawImage(img->getwidth(), img->getheight(), 1);
}

/**
 * Add a corner, except if it is too close to the border. In that case it is not important,
 * because it can move out of the visual field. That would yield improper results.
 */
void CornerDetector::AddCorner(std::vector<Corner*> & corners, int i, int j) {
	const int margin = 10;
	if (i < margin) return;
	if (j < margin) return;
	if (i > (img->getwidth() - margin)) return;
	if (j > (img->getheight() - margin)) return;
	corners.push_back(new Corner(i,j));
}

/**
 * www.aishack.in/2010/05/the-shi-tomasi-corner-detector/
 *
 * Get the derivatives
 * http://www.csse.uwa.edu.au/~pk/research/matlabfns/Spatial/derivative5.m
 * Requires a 2D convolution filter.
 *
 * Use a Gaussian filter
 * http://www.csse.uwa.edu.au/~pk/research/matlabfns/Spatial/harris.m
 */
void CornerDetector::GetCorners(std::vector<Corner*> & corners) {
	cout << __func__ << ": start" << endl;
	if (!img) {
		cerr << __func__ << "First set image" << endl;
		assert(false);
	}
	assert (img->isMonochrome());

	stringstream f;
	string method;
	cout << "Detection" << endl;
#ifdef USE_FAST
	fast(corners);
	method = "fast";
#else
	harris(corners);
	method = "harris";
#endif

	DrawCorners(corners, dDisp);

#ifdef STORE_IMAGES
	f.clear(); f.str("");
	f << "corners_" << method << '_' << ++index << ".bmp";
	cout << __func__ << ": save " << f.str() << endl;
	dDisp->saveBmp(f.str().c_str());
#endif

	cout << __func__ << ": end" << endl;
}

/**
 * Harris corner detector, or Shi-Tomaso, etc.
 */
void CornerDetector::harris(std::vector<Corner*> &corners) {

#ifdef LOWER_HARRIS_ACCURACY
	//5-tap derivative coefficients for 2 derivatives
	unsigned char ntap = 5;
	float p[]  = {0.030320,  0.249724,  0.439911,  0.249724,  0.030320};
	float d1[] = {0.104550,  0.292315,  0.000000, -0.292315, -0.104550};
	float d2[] = {0.232905,  0.002668, -0.471147,  0.002668,  0.232905};
#else
	unsigned char ntap = 7;
    float p[]  = { 0.004711,  0.069321,  0.245410,  0.361117,  0.245410,  0.069321,  0.004711 };
    float d1[] = { 0.018708,  0.125376,  0.193091,  0.000000, -0.193091, -0.125376, -0.018708 };
    float d2[] = { 0.055336,  0.137778, -0.056554, -0.273118, -0.056554,  0.137778,  0.055336 };
#endif
	cout << __func__ << ": convolve" << endl;
	bool success;
	success = convolve2DSeparable(img->data, dx->data, img->getwidth(), img->getheight(), p, ntap, d1, ntap);
	assert (success);
	// we don't need dy but it would be d1,ntap,p,ntap
	cout << __func__ << ": convolve" << endl;
	success = convolve2DSeparable(img->data, dy->data, img->getwidth(), img->getheight(), d1, ntap, p, ntap);
	cout << __func__ << ": convolve" << endl;
	convolve2DSeparable(img->data, ddx->data, img->getwidth(), img->getheight(), p, ntap, d2, ntap);
	cout << __func__ << ": convolve" << endl;
	convolve2DSeparable(img->data, ddy->data, img->getwidth(), img->getheight(), d2, ntap, p, ntap);
	cout << __func__ << ": convolve" << endl;
	convolve2DSeparable(dx->data, dxy->data, img->getwidth(), img->getheight(), d1, ntap, p, ntap);

	// We now have the "structure tensor" http://en.wikipedia.org/wiki/Corner_detection
	// or in other wards the "Harris matrix"

	// I don't know why there is first a convolution according to the Shi and Tomasi
	// operator and afterwards a convolution with a Gaussian filter.
	// A 2D Gaussian filter http://www.librow.com/articles/article-9 is also separable
	// and it would be better to incorporate that at once in the Hessian itself
	// for now I just skip it
//#define HARRIS_STEPHEN

	// now we have to calculate \gamma_1 * \gamma_2 - k ( \gamma_1 + gamma_2 )^2
	// this does not need to calculate the eigenvalue decomposition of A, but only the determinant and the trace
	// it is also possible to avoid to have a parameter k, by using a harmonic mean of the eigenvalues

#ifdef HARRIS_STEPHEN
	// this one fails because we are working with chars
	float k = 0.04; // 0.04 to 0.15 according to wikipedia
	cout << __func__ << ": write dH" << endl;
	for (int i = 0; i < img->getsize(); ++i) {
		dH->data[i] = (ddx->data[i]*ddy->data[i] - dxy->data[i]*dxy->data[i]) \
				- k * (ddx->data[i]+ddy->data[i])*(ddx->data[i]+ddy->data[i]);
		if (ddx->data[i]*ddy->data[i] - dxy->data[i]*dxy->data[i] > 255) {
			cerr << "Error, reached limit: " << ddx->data[i]*ddy->data[i] - dxy->data[i]*dxy->data[i] << endl;
			cerr << " but minus " << k * (ddx->data[i]+ddy->data[i])*(ddx->data[i]+ddy->data[i]) << endl;
			cerr << " makes finally " << (int)dH->data[i] << endl;
		}
	}
#else
	unsigned char eps = 5;
	for (int i = 0; i < img->getsize(); ++i) {
		dH->data[i] = (ddx->data[i] * ddy->data[i] - dxy->data[i] * dxy->data[i]) \
				/ (ddx->data[i] + ddy->data[i] + eps);
	}
#endif
	// http://www.csse.uwa.edu.au/~pk/research/matlabfns/Spatial/nonmaxsuppts.m

	// Shi and Tomasi computes min(\gamma_1,\gamma_2) (the minimum of the eigenvalues)
	// which seems to be better, but is computationally more expensive than determining the determinant and trace

	int threshold = 80;
	for (int i = 1; i < img->getwidth()-1; ++i) {
		for (int j = 1; j < img->getheight()-1; ++j) {
			int total = dH->data[i+j*img->getwidth()];
			if (total > threshold) {
				AddCorner(corners,i,j);
			}
		}
	}
}

/**
 * Just small wrapper around default library. I haven't done some quality control yet.
 * Either with respect to default versus _nonmax versions. Or with respect to fast9, fast... versions.
 */
void CornerDetector::fast(std::vector<Corner*> &corners) {
	int numcorners;
	xy* xycorn;
	//xycorn = fast11_detect_nonmax(img->data, img->getwidth(), img->getheight(), img->getwidth(), 100, &numcorners);
	xycorn = fast11_detect(img->data, img->getwidth(), img->getheight(), img->getwidth(), 20, &numcorners);
	for (int p = 0; p < numcorners; ++p) {
		int i = xycorn[p].x;
		int j = xycorn[p].y;
		AddCorner(corners, i, j);
	}
}

void CornerDetector::DrawCorners(std::vector<Corner*> & corners, CRawImage *result) {
	const int white = 255;
	const int black = 0;

	// clear image
	for (int i = 0; i < img->getsize(); ++i) {
		result->data[i] = white;
	}

	// fill it with crosses
	for (unsigned int c = 0; c < corners.size(); ++c) {
		int i = corners[c]->x;
		int j = corners[c]->y;
		int cross = 4;
		for (int di = -cross; di < cross; ++di) {
			int dii = i+di;
			if (dii < 0) continue;
			if (dii >= img->getwidth()) continue;
			result->data[dii+j*img->getwidth()] = black;
		}
		for (int dj = -cross; dj < cross; ++dj) {
			int djj = j+dj;
			if (djj < 0) continue;
			if (djj >= img->getheight()) continue;
			result->data[i+djj*img->getwidth()] = black;
		}
	}
}
