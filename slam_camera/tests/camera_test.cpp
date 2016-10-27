#include "slam/utils/munit.hpp"
#include "slam/camera/camera.hpp"


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
    slam::Camera camera;
    cv::Mat image;

    // init
    camera.configure(0, 320, 240);

    // loop
    for (int i = 0; i < 10; i++) {
        camera.getFrame(image);
        cv::imshow("test", image);
        cv::waitKey(1);
    }

    return 0;
}

void testSuite(void)
{
    mu_add_test(testCamera);
    // mu_add_test(testCameraConfigure);
}

mu_run_tests(testSuite)
