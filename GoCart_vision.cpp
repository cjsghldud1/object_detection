//
// Created by hoit on 19. 4. 2.
//

#include <opencv2/opencv.hpp>
#include <OpenNI.h>
#include <time.h>
#include <list>
#include "apriltag.h"
#include <apriltag_pose.h>
#include "tag36h11.h"
#include "common/getopt.h"

namespace cv
{
    void Filter_anisotropic_diffusion(Mat &src, Mat &dst, float alpha, float K, int niters)
    {
        Size size = src.size();
        int type = src.type();
        int w = size.width;
        int h = size.height;

        dst.create(size, type );
    }
}

//simple_cluster backup
/*void simple_cluster(std::vector<cv::Point> data, std::vector<cv::Point> &clusters,
                    int dist_threshold = 100)
{
    int dist, min, min_num;
    int sumx = 0, sumy = 0;
    std::vector<cv::Point> temp;


    while(data.size())
    {

        sumx = 0;
        sumy = 0;

        temp.push_back(data[0]);
        data.erase(data.begin());

        ////////make a cluster named temp
        for(int i = 0; i < data.size(); i++) {

            ///////////find the closest point
            for (int j = 0; j < data.size(); j++) {
                for (int k = 0; k < temp.size(); k++) {

                    dist = (temp[k].x - data[j].x) * (temp[k].x - data[j].x) +
                           (temp[k].y - data[j].y) * (temp[k].y - data[j].y);

                    if (dist < dist_threshold*dist_threshold) {
                        temp.push_back(data[j]);
                        data.erase(data.begin() + j);
                        j--;
                        break;
                    }
                }
            }

//            //////////push the closest point to temp
//            if (min < dist_threshold) {
//                min = dist_threshold;
//                temp.push_back(data[min_num]);
//                data.erase(data.begin() + min_num);
//                i--;
//            }
        }

        //////////push a centerpoint of the cluster 'temp' to 'clusters'
        for(int i=0;i < temp.size();i++)
        {
            sumx += temp[i].x;
            sumy += temp[i].y;
        }

        clusters.push_back(cv::Point(sumx/temp.size(), sumy/temp.size()));
        temp.clear();
    }


}*/






void simple_cluster(std::list<cv::Point> data, std::vector<cv::Point> &clusters,
                    int dist_threshold = 100)
{
    int dist, min, min_num;
    int sumx = 0, sumy = 0;
    bool brk_flag = false;
    cv::Point pt_data;
    std::vector<cv::Point> temp;
    std::list<cv::Point>::iterator iter;




    while(data.size())
    {

        sumx = 0;
        sumy = 0;

        iter = data.begin();
        temp.push_back(*iter);
        data.erase(iter);

        ////////make a cluster named temp
        for(int i = 0; i < data.size(); i++) {

            iter = data.begin();
            ///////////find the closest point
            for (int j = 0; j < data.size(); j++) {

                for (int k = 0; k < temp.size(); k++) {
                    pt_data = *iter;
                    dist = (temp[k].x - pt_data.x) * (temp[k].x - pt_data.x) +
                           (temp[k].y - pt_data.y) * (temp[k].y - pt_data.y);

                    if (dist < dist_threshold*dist_threshold) {
                        temp.push_back(*iter);
                        iter = data.erase(iter);
                        j--;
                        brk_flag = true;
                        break;
                    }


                }
                if ( brk_flag == false)  iter++;
                brk_flag = false;

            }

            //////////push the closest point to temp
//            if (min < dist_threshold) {
//                min = dist_threshold;
//                temp.push_back(data[min_num]);
//                data.erase(data.begin() + min_num);
//                i--;
//            }
        }

        //////////push a centerpoint of the cluster 'temp' to 'clusters'
        for(int i=0;i < temp.size();i++)
        {
            sumx += temp[i].x;
            sumy += temp[i].y;
        }

        clusters.push_back(cv::Point(sumx/temp.size(), sumy/temp.size()));
        temp.clear();
    }


}







void show_img(const std::string& winname, cv::Mat &img)
{
    cv::namedWindow(winname, cv::WINDOW_NORMAL);
    cv::imshow(winname, img);
}







void img_capture(cv::Mat &img, int pressed_key)
{
    const time_t t = time(NULL);
    struct tm *date;
    int i_time;
    std::string s_time;

    if (pressed_key == 99) // which means 'c'
    {
        printf("@@@@@@@@@@@@@@@@   img_captured!   @@@@@@@@@@@@@@@@");
        time_t t = time(NULL);
        date = localtime(&t);
        i_time = date->tm_hour * 10000 + date->tm_min * 100 + date->tm_sec;
        s_time = std::to_string(i_time) + ".png";
        cv::imwrite(s_time, img);
    }
}







void draw_keypoint( cv::Mat &img, std::vector< cv::KeyPoint > keypoints, int scale_factor = 1)
{
    std::vector<cv::Point2f> points;

    for (int i(0); i < keypoints.size(); i++) {
        points.push_back(keypoints[i].pt * scale_factor);
    }

    for (int i(0); i < points.size(); i++) {
        cv::circle(img, points[i], 2, cv::Scalar(0, 0, 255));
    }
    printf("\nNumber of keypoints: %d\n", points.size());

}






void draw_rectangle( cv::Mat &pimg, cv::Point2f mean, cv::Point2f p1, cv::Point2f p2)
{

    cv::circle(pimg, cv::Point2f(  mean.x, mean.y ) , 10, cv::Scalar(0, 0, 255),10);
    cv::line(pimg, p1,  cv::Point2f( p2.x, p1.y), cv::Scalar(128, 128, 0), 4);
    cv::line(pimg, cv::Point2f( p2.x, p1.y), p2, cv::Scalar(128, 128, 0), 4);
    cv::line(pimg, p2, cv::Point2f( p1.x, p2.y), cv::Scalar(128, 128, 0), 4);
    cv::line(pimg, cv::Point2f( p1.x, p2.y), p1, cv::Scalar(128, 128, 0), 4);

}






void match3D_img(cv::Mat &src,std::vector< cv::KeyPoint > &keypoints0, cv::Mat &gray,
                 std::vector< cv::KeyPoint > &keypoints1, std::vector<cv::DMatch> &good_matches, cv::Mat &img_matches)
{

    drawMatches(src, keypoints0, gray, keypoints1,
                good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);


    if (good_matches.size() > 4) {
        //-- Localize the object
        std::vector<cv::Point2f> obj;
        std::vector<cv::Point2f> scene;


        for (int i = 0; i < good_matches.size(); i++) {
            //-- Get the keypoints from the good matches
            obj.push_back(keypoints0[good_matches[i].queryIdx].pt);
            scene.push_back(keypoints1[good_matches[i].trainIdx].pt);
        }

        cv::Mat H = findHomography(obj, scene, CV_RANSAC, 10);

        //-- Get the corners from the image_1 ( the object to be "detected" )
        std::vector<cv::Point2f> obj_corners(4);
        obj_corners[0] = cvPoint(0, 0);
        obj_corners[1] = cvPoint(src.cols, 0);
        obj_corners[2] = cvPoint(src.cols, src.rows);
        obj_corners[3] = cvPoint(0, src.rows);
        std::vector<cv::Point2f> scene_corners(4);

        perspectiveTransform(obj_corners, scene_corners, H);

        //-- Draw lines between the corners (the mapped object in the scene - image_2 )
        line(img_matches, scene_corners[0] + cv::Point2f(src.cols, 0), scene_corners[1] + cv::Point2f(src.cols, 0),
             cv::Scalar(0, 255, 0), 4);
        line(img_matches, scene_corners[1] + cv::Point2f(src.cols, 0), scene_corners[2] + cv::Point2f(src.cols, 0),
             cv::Scalar(0, 255, 0), 4);
        line(img_matches, scene_corners[2] + cv::Point2f(src.cols, 0), scene_corners[3] + cv::Point2f(src.cols, 0),
             cv::Scalar(0, 255, 0), 4);
        line(img_matches, scene_corners[3] + cv::Point2f(src.cols, 0), scene_corners[0] + cv::Point2f(src.cols, 0),
             cv::Scalar(0, 255, 0), 4);

    }


    else
    {
        printf("\nMatches less than 4 Found!!  %d\n", good_matches.size() );

        return;

    }
}






void template_matching(cv::Mat &image_src, cv::Mat &template_src, std::vector<cv::Mat> &dst,
                       std::vector<cv::Rect> &dst_info)
{
    cv::Mat matching_result_img;
    std::list<cv::Point> good_match;
    std::vector<cv::Point> clusters;
    cv::Point max_pnt;
    cv::Point min_pnt;
    double maxval;
    double minval;

    cv::matchTemplate(image_src, template_src, matching_result_img, cv::TM_CCOEFF_NORMED);
    cv::minMaxLoc(matching_result_img, &minval, &maxval,&min_pnt, &max_pnt,cv::Mat());
    printf("    %f   %d,%d",maxval,max_pnt.x ,max_pnt.y);


    if( maxval > 0.35)
    {
        cv::normalize(matching_result_img, matching_result_img, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());
        for (int i = 0; i < matching_result_img.rows; i++) {
            for (int j = 0; j < matching_result_img.cols; j++) {
                if (matching_result_img.at<float>(i, j) > 0.90)
                    good_match.push_back(cv::Point(j, i));
            }
        }
//
    }

    cv::cvtColor(matching_result_img,matching_result_img, cv::COLOR_GRAY2RGB);

//            for(int i = 0; i < dumM_range.size(); i++)
//                cv::circle(dummy,dumM_range[i],5, cv::Scalar(0, 0, 255),2);
//

    if (good_match.size() > 0) {
        simple_cluster(good_match, clusters, 100);

        for(int i = 0; i < clusters.size(); i++)
        {
            cv::circle(matching_result_img, clusters[i], 10, cv::Scalar(0, 255, 0), 2);

            cv::Rect temp(clusters[i].x, clusters[i].y, template_src.cols, template_src.rows);

            dst_info.push_back(temp);

            dst.push_back(image_src(temp));

        }

    }

    show_img("template_matching_result", matching_result_img);



    show_img("img_keypoint", image_src);


}






/*void template_matching(cv::Mat &img_src, cv::Mat &template_src, std::vector<cv::Mat> &dst )
{
    cv::Mat matching_result_img;
    std::list<cv::Point> good_match;
    std::vector<cv::Point> clusters;
    cv::Point max_pnt;
    cv::Point min_pnt;
    double maxval;
    double minval;

    cv::matchTemplate(img_src, template_src, matching_result_img, cv::TM_CCOEFF_NORMED);
    cv::minMaxLoc(matching_result_img, &minval, &maxval,&min_pnt, &max_pnt,cv::Mat());
    printf("    %f   %d,%d",maxval,max_pnt.x ,max_pnt.y);


    if( maxval > 0.35)
    {
        cv::normalize(matching_result_img, matching_result_img, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());
        for (int i = 0; i < matching_result_img.rows; i++) {
            for (int j = 0; j < matching_result_img.cols; j++) {
                if (matching_result_img.at<float>(i, j) > 0.90)
                    good_match.push_back(cv::Point(j, i));
            }
        }
//
    }

    cv::cvtColor(matching_result_img,matching_result_img, cv::COLOR_GRAY2RGB);

//            for(int i = 0; i < dumM_range.size(); i++)
//                cv::circle(dummy,dumM_range[i],5, cv::Scalar(0, 0, 255),2);
//

    if (good_match.size() > 0) {
        simple_cluster(good_match, clusters, 100);

        for(int i = 0; i < clusters.size(); i++)
        {
            cv::circle(matching_result_img, clusters[i], 10, cv::Scalar(0, 255, 0), 2);

            cv::Rect rect(clusters[i].x, clusters[i].y, template_src.cols, template_src.rows);

            dst.push_back(img_src(rect));

        }

    }

    show_img("template_matching_result", matching_result_img);



    show_img("img_keypoint", img_src);


}*/







void feature_matching(std::vector<cv::KeyPoint> &keypoints0, cv::Mat &descriptors0, cv::Mat &template_src,
                      cv::Mat &roi, std::vector<cv::KeyPoint> &keypoints1,
                      std::vector<cv::DMatch> &matches)
{
    std::vector<cv::DMatch> good_matches;
    cv::Mat descriptors1;
    cv::Mat match_img;


    cv::ORB orb;
    cv::Ptr< cv::DescriptorMatcher > matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");


    orb(roi, cv::Mat(), keypoints1, descriptors1, false);
    if (descriptors0.rows > 0 && descriptors1.rows > 0) {
        matcher->match(descriptors0, descriptors1, matches);
    }


    printf("\nkeypoint1: %d\n", keypoints1.size());
    if (matches.size() > 3) {
        double max_dist = 0;
        double min_dist = 100;
        double mean = 0;

        //-- Quick calculation of max and min distances between keypoints
        for (int j = 0; j < descriptors0.rows; j++) {

            if (matches[j].distance < min_dist) min_dist = matches[j].distance;
        }

        printf("-- Min dist : %f \n", min_dist);

        for (int k = 0; k < descriptors0.rows; k++) {
            if (matches[k].distance < 60) {
                good_matches.push_back(matches[k]);
                mean += matches[k].distance;
            }
        }

        mean /= good_matches.size();


        printf("\n mean_good_matches: %f", mean);
    }

    if (good_matches.size() > 0)
    {
        printf("%d                           dfafasdfsdf",good_matches.size());


        drawMatches(template_src, keypoints0, roi, keypoints1,
                    good_matches, match_img, cv::Scalar::all(-1), cv::Scalar::all(-1),
                    std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
        show_img("feature_Matching", match_img);
    }

}





void feature_matching(std::vector<cv::KeyPoint> &keypoints0, cv::Mat &descriptors0, cv::Mat &template_src,
                      cv::Mat image_src, std::vector<cv::Mat> &roi, std::vector<cv::Rect> &roi_info,
                      std::vector<cv::KeyPoint> &Best_keypoints1, std::vector<cv::DMatch> &Best_matches)
{
    float mean_good_matches = 1000;
    int max_good_matches = 0;
    int good_roi;
    std::vector<cv::KeyPoint> keypoints1;
    std::vector<cv::DMatch> good_matches;
    std::vector<cv::DMatch> matches;

    cv::Mat descriptors1;
    cv::Mat match_img;
    cv::Mat best_matches_img;

    cv::ORB orb;
    cv::Ptr< cv::DescriptorMatcher > matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

    for(int i = 0; i < roi.size();i++) {
        keypoints1.clear();
        descriptors1.release();
        matches.clear();

        orb(roi[i], cv::Mat(), keypoints1, descriptors1, false);
        if (descriptors0.rows > 0 && descriptors1.rows > 0) {
            matcher->match(descriptors0, descriptors1, matches);
        }


        printf("\nkeypoint1: %d\n", keypoints1.size());
        if (matches.size() > 3) {
            float max_dist = 0;
            float min_dist = 100;
            float mean = 0;

            std::vector<cv::DMatch> good_matches;
            //-- Quick calculation of max and min distances between keypoints
            for (int j = 0; j < descriptors0.rows; j++) {
                float dist = matches[j].distance;
                if (dist < min_dist) min_dist = dist;
                if (dist > max_dist) max_dist = dist;
            }

            printf("-- Min dist : %f \n", min_dist);

            for (int k = 0; k < descriptors0.rows; k++) {
                if (matches[k].distance < 60) {
                    good_matches.push_back(matches[k]);
                    mean += matches[k].distance;
                }
            }

            mean /= good_matches.size();

            if (mean_good_matches > mean)
            {
                Best_matches.clear();
                Best_keypoints1.clear();
                mean_good_matches = mean;
                Best_matches = good_matches;
                Best_keypoints1 = keypoints1;
                good_roi = i;
                good_matches.clear();
            }

            printf("\n mean_good_matches: %f", mean_good_matches);
        }

    }



    if (Best_matches.size() > 0)
    {
        printf("%d                           dfafasdfsdf",Best_matches.size());


        drawMatches(template_src, keypoints0, roi[good_roi], Best_keypoints1,
                    Best_matches, best_matches_img, cv::Scalar::all(-1), cv::Scalar::all(-1),
                    std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
        show_img("feature_Matching", best_matches_img);

    }


    if (Best_matches.size() > 0) {
        draw_rectangle(image_src, cv::Point2f(roi_info[good_roi].x + roi_info[good_roi].width / 2, roi_info[good_roi].y + roi_info[good_roi].height / 2),
                       cv::Point2f(roi_info[good_roi].x, roi_info[good_roi].y),
                       cv::Point2f(roi_info[good_roi].x + roi_info[good_roi].width, roi_info[good_roi].y + roi_info[good_roi].height));
        show_img("Matching", image_src);
    }
}


apriltag_pose_t apriltag_matching(cv::Mat image_src, apriltag_detector_t *td )
{
    cv::Mat gray;
    apriltag_pose_t pose;

    cv::cvtColor(image_src, gray, cv::COLOR_RGB2GRAY);

    image_u8_t im = { .width = gray.cols,
            .height = gray.rows,
            .stride = gray.cols,
            .buf = gray.data
    };

    zarray_t *detections = apriltag_detector_detect(td, &im);
    std::cout << zarray_size(detections) << " tags detected" << std::endl;

    // Draw detection outlines
    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        line(image_src, cv::Point(det->p[0][0], det->p[0][1]),
             cv::Point(det->p[1][0], det->p[1][1]),
             cv::Scalar(0, 0xff, 0), 2);
        line(image_src, cv::Point(det->p[0][0], det->p[0][1]),
             cv::Point(det->p[3][0], det->p[3][1]),
             cv::Scalar(0, 0, 0xff), 2);
        line(image_src, cv::Point(det->p[1][0], det->p[1][1]),
             cv::Point(det->p[2][0], det->p[2][1]),
             cv::Scalar(0xff, 0, 0), 2);
        line(image_src, cv::Point(det->p[2][0], det->p[2][1]),
             cv::Point(det->p[3][0], det->p[3][1]),
             cv::Scalar(0xff, 0, 0), 2);

        std::stringstream ss;
        ss << det->id;
        cv::String text = ss.str();
        int fontface = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
        double fontscale = 1.0;
        int baseline;
        cv::Size textsize = cv::getTextSize(text, fontface, fontscale, 2,
                                    &baseline);
        putText(image_src, text, cv::Point(det->c[0]-textsize.width/2,
                                   det->c[1]+textsize.height/2),
                fontface, fontscale, cv::Scalar(0xff, 0x99, 0), 2);


        apriltag_detection_info_t info;
        info.det = det;
        info.tagsize = 0.071; // Size of square inside of dashed-line
        info.fx = 514.60723;  // Calculate with MRPT camera calib. tool
        info.fy = 512.72978;
        info.cx = 332.26685;
        info.cy = 238.38761;



        double err = estimate_tag_pose(&info, &pose);
//        printf("X: %lf \nY: %lf \nZ: %lf\n", pose.t->data[0]*1000,pose.t->data[1]*1000,pose.t->data[2]*1000);

        /*for(int i = 0; i < 9; i++) {
            printf("%lf\t", pose.R->data[i]);
            if (i%3 == 2)
                printf("\n");
        }*/

//            printf("depth: %ld  \n", d_frame.at<uint16_t>(Point(round(det->c[0]),
//                                                                round(det->c[1]))));


    }


    imshow("Tag Detections", image_src);
    return pose;
//        d_frame.release();


}