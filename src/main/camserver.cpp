/**
 * Create by ... at Prague University.
 */
//#include "CImageServer.h"
#include "CCamera.h"
#include "CTimer.h"
#include <string>
#include <sstream>
#include <iostream>
#include <semaphore.h>
#include <vector>
#include <cassert>
#include <CornerDetector.h>

#include <iomanip>

char port[] = "10002"; 

CRawImage* image0;
CRawImage* image1;
CRawImage* image0gray;
CRawImage* image1gray;

//CImageServer* server;
sem_t imageSem;

//#define SPECIFIC_IMAGE

#define ENABLE_CAM

using namespace std;


// http://www.cise.ufl.edu/class/cap5416fa09/Assignments/Right.bmp
void specific_image() {
	CornerDetector detector;
	string home = string(getenv("HOME"));
	if (home.empty()) cerr << "Error: no $HOME env. variable" << endl;
	string path = home + "/myworkspace/stuttgart/controller/almende/visualodometry/data/";
	string imagename = "right.bmp";
	std::ostringstream filename; filename.clear();
	filename << path << setfill('0') << setw(8) << imagename;
	if (!image1->loadBmp(filename.str().c_str())) {
		cerr << "Couldn't load bmp file: \"" << filename.str() << '"' << endl;
		assert(false);
	}
	image1->saveNumberedBmp("image");
	image1->makeMonochrome();
	detector.SetImage(image1);
	std::vector<Corner*> corners;
	for (unsigned int i = 0; i < corners.size(); ++i) delete corners[i];
	corners.clear();
	detector.GetCorners(corners);
}

/**
 * The most primitive way of matching two points. Just check all values in a neighbourhood and
 * if the total distance between the values at the specific locations around the two corners is
 * small, it is considered the same point. This doesn't work at all... It is not rotation or
 * translation invariant, or able to capture light difference etc.
 */
bool match(Corner *c0, Corner *c1, CRawImage *img0, CRawImage *img1) {
	int region = 4;
	int dist = 0;
	for (int i = -region; i < region; ++i) {
		for (int j = -region; j < region; ++j) {
			dist += abs(img0->data[(j+c0->y)*img0->getwidth() + (i+c0->x)] - img1->data[(j+c1->y)*img1->getwidth() + (i+c1->x)]);
		}
	}
	return (dist < region*region*20); // 320
}

/**
 * This starts a separate binary forever, calling renewImage indefinitely.
 */
int main(int argc,char *argv[])
{
	if (argc < 2) {
		fprintf(stderr, "You need the camera file descriptor as argument\n");
		return EXIT_FAILURE;
	}
	char *devName = argv[1];

	sem_init(&imageSem,0,1);

	image0 = new CRawImage(640,480,3);
	image1 = new CRawImage(640,480,3);
	image0gray = new CRawImage(640,480,1);
	image1gray = new CRawImage(640,480,1);

#ifdef SPECIFIC_IMAGE
	specific_image();
	cout << "Just calculate corners for specific image" << endl;

	delete image1;
	return EXIT_SUCCESS;
#endif

#ifdef ENABLE_SERVER
	server = new CImageServer(&imageSem,image1);
	server->initServer(port);
#endif

#ifdef ENABLE_CAM
	CCamera* cam;
	cam = new CCamera(&imageSem);
	cam->init(devName,640,480);
#else
	int offset = 100;
#endif
	int numImages = 1000;

	// set path and filenames
	string home = string(getenv("HOME"));
	if (home.empty()) cerr << "Error: no $HOME env. variable" << endl;
//	string path = home + "/myworkspace/replicator/visualodometry/data/rep/";
	string path = home + "/mydata/kit_robot/";
	string extension = ".bmp";
	int imageIndex = 0;

	CornerDetector detector;
	std::vector<Corner*> corners0; corners0.clear();
	std::vector<Corner*> corners1; corners1.clear();
	while (true) {
#ifdef ENABLE_CAM
		cout << "Grab new image" << endl;
		cam->renewImage(image0);
		if (++imageIndex == numImages) break;
#else
		std::ostringstream filename; filename.clear();
		filename << path << setfill('0') << setw(8) << (imageIndex + offset) << extension;
		if (!image0->loadBmp(filename.str().c_str())) {
			cerr << "Couldn't load bmp file: \"" << filename.str() << '"' << endl;
			assert(false);
		}
#endif
		cam->renewImage(image1);

//		image0->saveNumberedBmp("left");
		image0->makeMonochrome(image0gray);
		detector.SetImage(image0gray);
		for (unsigned int i = 0; i < corners0.size(); ++i) delete corners0[i];
		corners0.clear();
		detector.GetCorners(corners0);


//		image1->saveNumberedBmp("right",false);
		image1->makeMonochrome(image1gray);
		image1->makeMonochrome();
		detector.SetImage(image1);
		for (unsigned int i = 0; i < corners1.size(); ++i) delete corners1[i];
		corners1.clear();
		detector.GetCorners(corners1);

		// stupid exhaustive enumeration over all corners (should've been organized in a spatial sense)
		std::vector<Corner*> matches;
		matches.clear(); // individual corners do not need to be deleted
		for (unsigned int i = 0; i < corners0.size(); ++i) {
			Corner *c0 = corners0[i];
			for (unsigned int j = 0; j < corners1.size(); ++j) {
				Corner *c1 = corners0[j];

				if (abs(c0->x - c1->x) > 10) continue;
				if (abs(c0->y - c1->y) > 10) continue;

				// match corners
				if (match(c0, c1, image0, image1)) {
					matches.push_back(c0);
				}
			}
		}

		CRawImage *match_img(image0gray);
		detector.DrawCorners(matches, match_img);
		match_img->saveNumberedBmp("stereo");

		// now calculate features around corners to find matches...

		// use corners close to each other to calculate the homography (projective mapping), there are only four
		// correspondences required for this

		// depict this by coloring the points if they are on the same plane with the same color

		// remove outliers with ransac

		// ? plane expansion, use a type of region growing to expand the plane over an area as large as possible

		// ? can we somehow use normal image based segmentation for this?
		// it is possible to go over all images around and check for a certain reprojection error

#ifdef OBJECT_RECOGNTION
		// use corners to get patches

		// use patches to calculate color descriptors

		// match color descriptors to a previously composed codebook with all(!) possible patches/descriptors

		// use a supervised learning mechanism to match sets of codebook vectors (bag-of-words) with the robot class
		// often an SVM

		// the codebook is normally generated "learnt" by an unsupervised method, such as k-means clustering
		// and as described above, requires all possible patches imaginable in the real-world
#endif

	}

#ifdef ENABLE_CAM
	delete cam;
#endif
	delete image1;
	sleep (1);
}
