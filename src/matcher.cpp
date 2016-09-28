#include "matcher.hpp"


//     cv::drawKeypoints(
//         image,
//         key_pts,
//         image,
//         cv::Scalar(0, 0, 255),
//         cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS
//     );

// int Camera::matchFeaturesBF(cv::Mat d1, cv::Mat d2)
// {
//     float ratio;
//     std::vector<cv::DMatch> good_matches;
//     std::vector< std::vector<cv::DMatch> > matches;
//
//     // setup
//     ratio = 0.8;
//
//     // match
//     this->fmatcher->knnMatch(d1, d2, matches, 2);
//
//     // ratio test as in Lowe's paper; can be tuned
//     for (size_t i = 0; i < matches.size(); i++) {
//         if (matches[i][0].distance < ratio * matches[i][1].distance) {
//             good_matches.push_back(matches[i][0]);
//         }
//
//         if (good_matches.size() > 10) {
//             break;
//         }
//     }
//
//     return 0;
// }
//
// int Camera::matchFeaturesFlann(cv::Mat d1, cv::Mat d2)
// {
//     double dist;
//     double max_dist;
//     double min_dist;
//     std::vector<cv::DMatch> matches;
//     std::vector<cv::DMatch> good_matches;
//
//     // setup
//     dist = 0.0;
//     max_dist = 0.0;
//     min_dist = 100.0;
//
//     // match
//     this->fmatcher->match(d1, d2, matches);
//
//     // find min max distances between keypoints
//     for (int i = 0; i < d1.rows; i++) {
//         dist = matches[i].distance;
//         min_dist = (dist < min_dist) ? dist : min_dist;
//         max_dist = (dist > max_dist) ? dist : max_dist;
//     }
//
//     // filter only good matches
//     for (int i = 0; i < d1.rows; i++) {
//         if (matches[i].distance <= std::max(2 * min_dist, 0.05)) {
//             good_matches.push_back(matches[i]);
//         }
//     }
//
//     return 0;
// }
