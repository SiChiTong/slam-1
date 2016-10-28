#include "slam/utils/munit.hpp"
#include "slam/camera/camera.hpp"


#define CALIB_FILE "tests/data/calibration.yaml"

// TESTS
int testCamera(void);
int testCameraConfigure(void);
void testSuite(void);


int testCamera(void)
{
    slam::Camera camera;

    mu_check(camera.configured == false);
    mu_check(camera.capture == NULL);
    mu_check(camera.capture_index == 0);
    mu_check(camera.image_width == 0);
    mu_check(camera.image_height == 0);

    return 0;
}

int testCameraConfigure(void)
{
    int retval;
    cv::Mat image;
    slam::Camera camera;

    // configure camera with index, image dimensions
    retval = camera.configure(0, 320, 240);
    mu_check(retval == 0);
    camera.close();

    // configure camera with index, calibration file
    camera.configure(0, CALIB_FILE);
    mu_check(retval == 0);
    camera.close();

    return 0;
}

int testCameraGetFrame(void)
{
    slam::Camera camera;
    cv::Mat image;

    // configure camera with index, image dimensions
    camera.configure(0, 320, 240);
    for (int i = 0; i < 10; i++) {
        camera.getFrame(image);
        cv::imshow("test", image);
        cv::waitKey(1);
    }
    camera.close();

    return 0;
}

void testSuite(void)
{
    mu_add_test(testCamera);
    mu_add_test(testCameraConfigure);
    mu_add_test(testCameraGetFrame);
}

mu_run_tests(testSuite)
