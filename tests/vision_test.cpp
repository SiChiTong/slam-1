#include "munit.hpp"
#include "vision.hpp"


// TESTS
int testVision(void);
int testVisionInitCamera(void);
int testVisionInitFeatureDetector(void);
int testVisionInitFeatureExtractor(void);
int testVisionInitFeatureMatcher(void);
void testSuite(void);


int testVision(void)
{
    Vision vision;

    mu_check(vision.capture_index == 0);
    mu_check(vision.image_width == 0);
    mu_check(vision.image_height == 0);
    mu_check(vision.capture == NULL);

    return 0;
}

int testVisionInitCamera(void)
{
    Vision vision;
    cv::Mat image;

    // init
    vision.capture_index = 0;
    vision.image_width = 320;
    vision.image_height = 280;
    vision.initCamera();

    // loop
    for (int i = 0; i < 10; i++) {
        vision.capture->read(image);
        cv::imshow("test", image);
        cv::waitKey(1);
    }

    return 0;
}

int testVisionInitFeatureDetector(void)
{
	Vision vision;
    cv::Mat image;
	std::vector<cv::KeyPoint> key_pts;

	// feature detectors
	vision.initFeatureDetector("FAST");
	vision.initFeatureDetector("STAR");
	vision.initFeatureDetector("ORB");
	vision.initFeatureDetector("BRISK");
	vision.initFeatureDetector("MSER");
	vision.initFeatureDetector("GFTT");
	vision.initFeatureDetector("HARRIS");
	vision.initFeatureDetector("DENSE");
	vision.initFeatureDetector("SimpleBlob");

	// nonfree feature detectors
	vision.initFeatureDetector("SURF");
	vision.initFeatureDetector("SIFT");
	vision.initFeatureDetector("SURF");

    // init
    vision.capture_index = 0;
    vision.image_width = 320;
    vision.image_height = 280;
    vision.initCamera();

    // loop
	while (true) {
        vision.capture->read(image);
		vision.detectFeatures(image, key_pts);

        cv::imshow("test", image);
        cv::waitKey(1);
    }

    return 0;
}

int testVisionInitFeatureExtractor(void)
{

    return 0;
}

int testVisionInitFeatureMatcher(void)
{

    return 0;
}


void testSuite(void)
{
    mu_add_test(testVision);
    // mu_add_test(testVisionInitCamera);
	mu_add_test(testVisionInitFeatureDetector);
	// mu_add_test(testVisionInitFeatureExtractor);
	// mu_add_test(testVisionInitFeatureMatcher);
}

mu_run_tests(testSuite)
