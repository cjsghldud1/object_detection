//
// Created by hoit on 19. 4. 2.
//

#include <opencv2/opencv.hpp>
#include <OpenNI.h>
#include <time.h>
#include <list>

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

    if (pressed_key == 1048675) // which means 'c'
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

void match_img(cv::Mat &src,std::vector< cv::KeyPoint > &keypoints0, cv::Mat &gray, std::vector< cv::KeyPoint > &keypoints1,
               std::vector<cv::DMatch> &good_matches, cv::Mat &img_matches)
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