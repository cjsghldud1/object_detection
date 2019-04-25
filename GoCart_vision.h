//
// Created by hoit on 19. 4. 2.
//



#ifndef UNTITLED3_GOCART_VISION_H
#define UNTITLED3_GOCART_VISION_H

#include <opencv2/opencv.hpp>
#include <OpenNI.h>
#include <list>

namespace cv
{
    void Filter_anisotropic_diffusion(Mat &src, Mat &dst, float alpha, float K, int niters);


}
//simple_cluster backup
/*
void simple_cluster(std::vector<cv::Point> data, std::vector<cv::Point> &clusters,
                    int dist_threshold = 100);
*/

void simple_cluster(std::list<cv::Point> data, std::vector<cv::Point> &clusters,
                    int dist_threshold = 100);

void show_img(const std::string& winname, cv::Mat &img);

void img_capture(cv::Mat &img, int key);

void draw_keypoint(const std::string& winname, cv::Mat &img,
        std::vector< cv::KeyPoint > keypoints, int scale_factor = 1);

void draw_rectangle( cv::Mat &pimg, cv::Point2f mean, cv::Point2f p1, cv::Point2f p2);
#endif //UNTITLED3_GOCART_VISION_H
void match_img(cv::Mat &src,std::vector< cv::KeyPoint > &keypoints0, cv::Mat &gray, std::vector< cv::KeyPoint > &keypoints1,
              std::vector<cv::DMatch> &good_matches, cv::Mat &img_matches);