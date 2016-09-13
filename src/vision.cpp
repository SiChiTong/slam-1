#include "vision.hpp"


Vision::Vision(void)
{
	cv::initModule_nonfree();

	this->initialized = false;

    this->capture_index = 0;
    this->image_width = 0;
    this->image_height = 0;
    this->capture = NULL;
}

int Vision::init(void)
{

    return 0;
}

int Vision::initCamera(void)
{
    // setup
    this->capture = new cv::VideoCapture(this->capture_index);

    // open camera
    if (this->capture->isOpened() == 0) {
        LOG_ERROR("failed to open webcam!");
        return -1;
    }

    // configure image resolution
    this->capture->set(CV_CAP_PROP_FRAME_WIDTH, image_width);
    this->capture->set(CV_CAP_PROP_FRAME_HEIGHT, image_height);
    LOG_INFO("camera initialized!");

    return 0;
}

int Vision::initFeatureDetector(std::string type)
{
	this->fdetector = cv::FeatureDetector::create(type);
	return 0;
}

int Vision::initFeatureExtractor(std::string type)
{
	this->fextractor = cv::DescriptorExtractor::create(type);
	return 0;
}

int Vision::initFeatureMatcher(std::string type)
{
	this->fmatcher = cv::DescriptorMatcher::create(type);
	return 0;
}

int Vision::detectFeatures(cv::Mat &image, std::vector<cv::KeyPoint> &key_pts)
{
	this->fdetector->detect(image, key_pts);
	cv::drawKeypoints(
		image,
		key_pts,
		image,
		cv::Scalar::all(-1),
		cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS
	);

	return 0;
}

int Vision::extractFeatures(
	cv::Mat &image,
	std::vector<cv::KeyPoint> &key_pts,
	cv::Mat &descriptors
)
{
	this->fextractor->compute(image, key_pts, descriptors);
	return 0;
}

int Vision::matchFeaturesBF(cv::Mat d1, cv::Mat d2)
{
	float ratio;
	std::vector<cv::DMatch> good_matches;
	std::vector< std::vector<cv::DMatch> > matches;

	// setup
	ratio = 0.8;

	// match
	this->fmatcher->knnMatch(d1, d2, matches, 2);

	// ratio test as in Lowe's paper; can be tuned
	for (size_t i = 0; i < matches.size(); i++) {
		if (matches[i][0].distance < ratio * matches[i][1].distance) {
			good_matches.push_back(matches[i][0]);
		}

		if (good_matches.size() > 10) {
			break;
		}
	}

	return 0;
}

int Vision::matchFeaturesFlann(cv::Mat d1, cv::Mat d2)
{
	double dist;
	double max_dist;
	double min_dist;
	std::vector<cv::DMatch> matches;
	std::vector<cv::DMatch> good_matches;

	// setup
	dist = 0.0;
	max_dist = 0.0;
	min_dist = 100.0;

	// match
	this->fmatcher->match(d1, d2, matches);

	// find min max distances between keypoints
	for (int i = 0; i < d1.rows; i++) {
		dist = matches[i].distance;
		min_dist = (dist < min_dist) ? dist : min_dist;
		max_dist = (dist > max_dist) ? dist : max_dist;
	}

	// filter only good matches
	for (int i = 0; i < d1.rows; i++) {
		if (matches[i].distance <= std::max(2 * min_dist, 0.05)) {
			good_matches.push_back(matches[i]);
		}
	}

	return 0;
}

// bool match_sort(const std::vector<cv::DMatch> &m1_vec, const std::vector<cv::DMatch> &m2_vec)
// {
//     for (size_t i = 0; i < m1_vec.size() && i < m2_vec.size(); i++) {
//         if (m1_vec[i].distance > m2_vec[i].distance) {
//             return false;
//         } else if (m1_vec[i].distance < m2_vec[i].distance) {
//             return true;
//         }
//     }
//
//     return false;
// }

// static void feature_matching_prototype(cv::Mat &image)
// {
//     // setup
//     // cv::Mat tag_image = cv::imread("tests/data/tags/tag.jpg");
//     // pyrDown(tag_image, tag_image, cv::Size(tag_image.cols / 2, tag_image.rows / 2));
//     // pyrDown(tag_image, tag_image, cv::Size(tag_image.cols / 2, tag_image.rows / 2));
//     // pyrDown(tag_image, tag_image, cv::Size(tag_image.cols / 2, tag_image.rows / 2));
//
//
//     // detect keypoints
//     std::vector<cv::KeyPoint> keypoints_1;
//     std::vector<cv::KeyPoint> keypoints_2;
//
//     // cv::Ptr<cv::FeatureDetector> feature_detector;
//     // feature_detector = cv::FeatureDetector::create("ORB");
//     // feature_detector->detect(tag_image, keypoints_1);
//     // feature_detector->detect(image, keypoints_2);
//
//     cv::SurfFeatureDetector feature_detector(200);
//     feature_detector.detect(tag_image, keypoints_1);
//     feature_detector.detect(image, keypoints_2);
//
//
//     // extract descriptors
//     // cv::Mat descriptors_1;
//     // cv::Mat descriptors_2;
//
//     // cv::Ptr<cv::DescriptorExtractor> feature_extractor;
//     // feature_extractor = cv::DescriptorExtractor::create("ORB");
//
//     // cv::Ptr<cv::DescriptorExtractor> feature_extractor;
//     // feature_extractor = cv::DescriptorExtractor::create("SURF");
//
//     // feature_extractor->compute(tag_image, keypoints_1, descriptors_1);
//     // feature_extractor->compute(image, keypoints_2, descriptors_2);
//
//
//     // match features
//     // if (!descriptors_1.empty() && !descriptors_2.empty()) {
//     //     std::vector< std::vector<cv::DMatch> > matches;
//     //
//     //     cv::BFMatcher feature_matcher;
//     //     feature_matcher.knnMatch(descriptors_1, descriptors_2, matches, 2);
//     //
//     //     // ratio test as in Lowe's paper; can be tuned
//     //     float ratio = 0.8;
//     //     std::vector<cv::DMatch> good_matches;
//     //     for (size_t i = 0; i < matches.size(); i++) {
//     //         if (matches[i][0].distance < ratio * matches[i][1].distance) {
//     //             good_matches.push_back(matches[i][0]);
//     //             // printf("%f %f\n", matches[i][0].distance, matches[i][1].distance);
//     //         }
//     //
//     //         if (good_matches.size() > 10) {
//     //             break;
//     //         }
//     //     }
//     //
//     //
//     //     // draw matches
//     //     if (matches.size() < 1) {
//     //         std::cout << "No matches!" << std::endl;
//     //     } else {
//     //         cv::Mat m;
//     //         cv::drawMatches(
//     //             tag_image,
//     //             keypoints_1,
//     //             image,
//     //             keypoints_2,
//     //             good_matches,
//     //             m,
//     //             cv::Scalar::all(-1),
//     //             cv::Scalar::all(-1),
//     //             std::vector<char>(),
//     //             cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS
//     //
//     //         );
//     //         cv::imshow("test", m);
//     //         // cv::waitKey(100000);
//     //         cv::waitKey(1000);
//     //     }
//     // }
//
// }
