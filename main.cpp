/*****************************************************************************
*                                                                            *
*  OpenNI 2.x Alpha                                                          *
*  Copyright (C) 2012 PrimeSense Ltd.                                        *
*                                                                            *
*  This file is part of OpenNI.                                              *
*                                                                            *
*  Licensed under the Apache License, Version 2.0 (the "License");           *
*  you may not use this file except in compliance with the License.          *
*  You may obtain a copy of the License at                                   *
*                                                                            *
*      http://www.apache.org/licenses/LICENSE-2.0                            *
*                                                                            *
*  Unless required by applicable law or agreed to in writing, software       *
*  distributed under the License is distributed on an "AS IS" BASIS,         *
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  *
*  See the License for the specific language governing permissions and       *
*  limitations under the License.                                            *
*                                                                            *
*****************************************************************************/
#include <OpenNI.h>
#include <iostream>
#include <time.h>
#include <list>
#include "Viewer.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "apriltag.h"
#include <apriltag_pose.h>
#include "tag36h11.h"
#include "common/getopt.h"
#include "GoCart_vision.h"


int main(int argc, char** argv)
{
    openni::Status rc = openni::STATUS_OK;

    openni::Device device;
    openni::VideoStream depth, color;
    const char* deviceURI = openni::ANY_DEVICE;
    if (argc > 1)
    {
        deviceURI = argv[0];
        std::cout<<"f"<<std::endl;
    }

    rc = openni::OpenNI::initialize();

    printf("After initialization:\n%s\n", openni::OpenNI::getExtendedError());
    printf("%d", deviceURI);
    std::cout<<std::endl<<deviceURI<<std::endl;
    rc = device.open(deviceURI);
    if (rc != openni::STATUS_OK)
    {
        printf("SimpleViewer: Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
        openni::OpenNI::shutdown();
        return 1;
    }





    rc = depth.create(device, openni::SENSOR_DEPTH);
    if (rc == openni::STATUS_OK)
    {
        rc = depth.start();
        if (rc != openni::STATUS_OK)
        {
            printf("SimpleViewer: Couldn't start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
            depth.destroy();
        }
    }
    else
    {
        printf("SimpleViewer: Couldn't find depth stream:\n%s\n", openni::OpenNI::getExtendedError());
    }





    rc = color.create(device, openni::SENSOR_COLOR);
    if (rc == openni::STATUS_OK)
    {
        rc = color.start();
        if (rc != openni::STATUS_OK)
        {
            printf("SimpleViewer: Couldn't start color stream:\n%s\n", openni::OpenNI::getExtendedError());
            color.destroy();
        }
    }
    else
    {
        printf("SimpleViewer: Couldn't find color stream:\n%s\n", openni::OpenNI::getExtendedError());
    }





    if (!depth.isValid() || !color.isValid())
    {
        printf("SimpleViewer: No valid streams. Exiting\n");
        openni::OpenNI::shutdown();
        return 2;
    }



    SampleViewer sampleViewer("Hello World", device, depth, color);

    rc = sampleViewer.init(argc, argv);
    if (rc != openni::STATUS_OK)
    {
        openni::OpenNI::shutdown();
        return 3;
    }


    openni::CameraSettings* camset = color.getCameraSettings();
    openni::VideoMode cammode = color.getVideoMode();
    openni::SensorInfo const * caminfo = device.getSensorInfo(openni::SENSOR_COLOR);

    caminfo->getSupportedVideoModes();


//    camset->setAutoWhiteBalanceEnabled(false);
    camset->setAutoExposureEnabled(false);


    openni::VideoFrameRef color_img;
    color.readFrame(&color_img);

    /*openni::VideoMode img_param;
    img_param = color_img.getVideoMode();


    printf("%d", (int)img_param.getPixelFormat());
    int aa = (int)img_param.getPixelFormat();
    std::cout<< aa;*/
    //OpenNI: BGR888

    getopt_t *getopt = getopt_create();

    getopt_add_bool(getopt, 'h', "help", 0, "Show this help");
    getopt_add_bool(getopt, 'd', "debug", 1, "Enable debugging output (slow)");
    getopt_add_bool(getopt, 'q', "quiet", 0, "Reduce output");
    getopt_add_string(getopt, 'f', "family", "tag36h11", "Tag family to use");
    getopt_add_int(getopt, 't', "threads", "1", "Use this many CPU threads");
    getopt_add_double(getopt, 'x', "decimate", "2.0", "Decimate input image by this factor");
    getopt_add_double(getopt, 'b', "blur", "0.0", "Apply low-pass blur to input");
    getopt_add_bool(getopt, '0', "refine-edges", 1, "Spend more time trying to align edges of tags");

    if (!getopt_parse(getopt, argc, argv, 1) ||
        getopt_get_bool(getopt, "help")) {
        printf("Usage: %s [options]\n", argv[0]);
        getopt_do_usage(getopt);
        exit(0);
    }

    printf("1111111111");
    // Initialize camera
//    VideoCapture cap(CV_CAP_OPENNI);
//    if (!cap.isOpened()) {
//        cerr << "Couldn't open video capture device" << endl;
//        return -1;
//    }




    // Initialize tag detector with options
    apriltag_family_t *tf = nullptr;
    const char *famname = getopt_get_string(getopt, "family");
    if (!strcmp(famname, "tag36h11")) {
        printf("1111");
        tf = tag36h11_create();
    } /*else if (!strcmp(famname, "tag25h9")) {
        tf = tag25h9_create();
    } else if (!strcmp(famname, "tag16h5")) {
        tf = tag16h5_create();
    } else if (!strcmp(famname, "tagCircle21h7")) {
        tf = tagCircle21h7_create();
    } else if (!strcmp(famname, "tagCircle49h12")) {
        tf = tagCircle49h12_create();
    } else if (!strcmp(famname, "tagStandard41h12")) {
        tf = tagStandard41h12_create();
    } else if (!strcmp(famname, "tagStandard52h13")) {
        tf = tagStandard52h13_create();
    } else if (!strcmp(famname, "tagCustom48h12")) {
        tf = tagCustom48h12_create();
    }*/ else {
        printf("Unrecognized tag family name. Use e.g. \"tag36h11\".\n");
        exit(-1);
    }

    printf("3333333333333333333");
    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    td->quad_decimate = getopt_get_double(getopt, "decimate");
    td->quad_sigma = getopt_get_double(getopt, "blur");
    td->nthreads = getopt_get_int(getopt, "threads");
    td->debug = getopt_get_bool(getopt, "debug");
    td->refine_edges = getopt_get_bool(getopt, "refine-edges");


    apriltag_pose_t pose;
////////////////////////////////////////////////////////




    auto *colorPix = new openni::RGB888Pixel[color_img.getHeight()*color_img.getWidth()];
    memcpy(colorPix, color_img.getData(), color_img.getHeight()*color_img.getWidth()*sizeof(uint8_t)*3);

    cv::Mat src;
    cv::Mat srcM;
    cv::Mat img1(color_img.getHeight(), color_img.getWidth(), CV_8UC3, colorPix);
    cv::Mat tmp;
    cv::Mat cimg;
    cv::Mat oimg;
    cv::Mat gray;
    cv::Mat dummy;
    cv::Mat descriptors0;

    cv::Mat best_matches_img;

    std::vector<cv::Mat> roi;
    std::vector<cv::Rect> roi_info;

    cv::cvtColor(img1,img1, cv::COLOR_BGR2RGB);
    cv::imshow("tset",img1);
    cv::moveWindow("tset",0,0);


    cv::FlannBasedMatcher matcherf;
    cv::Ptr< cv::DescriptorMatcher > matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

    cv::GoodFeaturesToTrackDetector GFTTdetector( 1024, 0.1, 1, 3, false);// num_pnt,quality_threshold,mindistance,mask_size,harris
    cv::FastFeatureDetector FASTdetector(10,true); //threshold,??clustering maybe
    cv::ORB orb;
    cv::BRISK Dbrisk;
    cv::Ptr< cv::DescriptorExtractor > extractor = cv::DescriptorExtractor::create("BRIEF");



    /*orb.set("WTA_K",4);
    orb.set("patchSize",50);
    orb.set("edgeThreshold",50);*/
/*    orb.
    (int nfeatures=500, float scaleFactor=1.2f, int nlevels=8, int edgeThreshold=31, int firstLevel=0, int WTA_K=4, int scoreType=cv::ORB::HARRIS_SCORE, int patchSize=31);


    cv::ORB::
    cv::OrbFeatureDetector::
    cv::Ptr< cv::DescriptorExtractor > extractor = cv::DescriptorExtractor::create("ORB");*/



    std::vector< cv::KeyPoint > keypoints0;
    std::vector< cv::KeyPoint > keypoints1;
    std::vector< cv::KeyPoint > Best_keypoints0;
    std::vector< cv::KeyPoint > Best_keypoints1;
    std::vector<cv::Point2f> points_start;
    std::vector<cv::Point2f> points_now;
    std::vector< cv::DMatch> matches;


    time_t start, end;
    int key;
    int old_key;
    int count_frame = 0;

    float fps = 0;
    double ftime_diff;



    openni::VideoStream **streams;
    streams = new openni::VideoStream*[2];
    streams[0] = &depth;
    streams[1] = &color;

    cv::waitKey(0);

    cv::cvtColor(img1,img1, cv::COLOR_BGR2RGB);
    cv::cvtColor(img1,gray, cv::COLOR_RGB2GRAY);
    cv::destroyWindow("tset");
    printf("111111111111111111\n");



    src = cv::imread("src_head.png");
//    cv::cvtColor(src,src, cv::COLOR_RGB2GRAY);
//    cv::blur( src, src, cv::Size(3,3) );
//        cv::cvtColor(gray, gray, cv::COLOR_RGB2GRAY);
//        cv::medianBlur(gray, gray, 3);
//        cv::GaussianBlur(gray, gray, cv::Size(3,3), 0 );
//        cv::blur(gray, gray, cv::Size(7,7));
    cv::Size2f size(0,0);
//    cv::resize(src,src,size,0.3,0.3);
//    GFTTdetector.detect(src, keypoints0);
//            FASTdetector.detect(src, keypoints0);
    orb(src, cv::Mat(), keypoints0, descriptors0, false);


/*    cv::SimpleBlobDetector::Params params;
//    params.filterByColor = true;
    params.filterByCircularity = true;
    params.minCircularity = 0;
    params.filterByConvexity = true;
    params.minConvexity = 0;
    params.minThreshold = 0;
    params.maxThreshold = 255;
    params.filterByArea = true;
    params.minArea = 10;
    params.filterByInertia = true;
    params.minInertiaRatio = 0.01;


    printf("%f %f %f", params.thresholdStep,params.minThreshold,params.maxThreshold);
    cv::SimpleBlobDetector SBD(params);
    SBD.detect(src,keypoints0);*/


//    Dbrisk(src, cv::Mat(), keypoints0, descriptors0, false );
//
    draw_keypoint(src, keypoints0);
    show_img("src_keypoint", src);

    srcM = cv::imread("src_head.png");
//    cv::resize(srcM,srcM,size,0.1,0.1);
    show_img("srcM", srcM);


//    cv::waitKey(0);






    time(&start);
    while(true) {

        old_key = key;
        key = cv::waitKey(1);
        if (key == -1)
            key = old_key;
        printf("%d",key);
        color.readFrame(&color_img);


        colorPix = new openni::RGB888Pixel[color_img.getHeight() * color_img.getWidth()];
        memcpy(colorPix, color_img.getData(), color_img.getHeight() * color_img.getWidth() * sizeof(uint8_t) * 3);
        img1.data = (uchar *) colorPix;


        cv::cvtColor(img1, gray, cv::COLOR_BGR2RGB);


        img_capture(gray, key);


//        cv::cvtColor(gray, gray, cv::COLOR_RGB2GRAY);
//        cv::medianBlur(gray, gray, 3);
//        cv::GaussianBlur(gray, gray, cv::Size(3,3), 0 );
//        cv::blur(gray, gray, cv::Size(7,7));
//        cv::bilateralFilter(img1, gray,9,75,75);


//        GFTTdetector.detect(gray, keypoints1);
//        FASTdetector.detect(gray, keypoints1);
//        orb(gray, cv::Mat(), keypoints1, descriptors1, false);
//        Dbrisk(gray, cv::Mat(), keypoints1, descriptors1, false );
//        printf("\n%d", keypoints0.size());
//        printf("\n%d", keypoints1.size());




//        draw_keypoint(gray, keypoints1);

        show_img("img_keypoint", gray);


        if (key == (int) 'm') {
            printf("\n\nTEMPLATE+FEATURE\n");
            template_matching(gray, srcM, roi, roi_info);
            feature_matching(keypoints0, descriptors0, srcM, gray, roi, roi_info, keypoints1, matches);
        }
        else if (key == (int) 'a') {
            printf("\n\nAPRILTAG_MATCHING\n");
            pose = apriltag_matching(gray, td);
            if (pose.t != nullptr)
                printf("X: %lf \nY: %lf \nZ: %lf\n", pose.t->data[0]*1000,pose.t->data[1]*1000,pose.t->data[2]*1000);
        }
        else if (key == (int) 't') {
            printf("\n\nTEMPLATE MATCHING\n");
            template_matching(gray, srcM, roi, roi_info);
        }
        else if(key == (int)'f') {
            printf("\n\nFEATURE MATCHING\n");
            feature_matching(keypoints0, descriptors0, srcM, gray, keypoints1, matches);
        }
        else if(key == 27) {
            printf("\nGOOD-BYE\n");
            break;
        }
        else{
            printf("/////////////////////////////////\n");
            printf("////  a: apriltag_matching   ////\n");
            printf("////  t: template_matching   ////\n");
            printf("////  f: feature_matching    ////\n");
            printf("////  m: template + feature  ////\n");
            printf("/////////////////////////////////\n");

        }

         /*mean_good_matches = 1000;
        max_good_matches = 0;
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
                double max_dist = 0;
                double min_dist = 100;
                double mean = 0;
                std::vector<cv::DMatch> good_matches;
                //-- Quick calculation of max and min distances between keypoints
                for (int j = 0; j < descriptors0.rows; j++) {
                    double dist = matches[i].distance;
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
                    Best_keypoints0.clear();
                    Best_keypoints1.clear();
                    mean_good_matches = mean;
                    Best_matches = good_matches;
                    Best_keypoints0 = keypoints0;
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


            drawMatches(srcM, keypoints0, roi[good_roi], Best_keypoints1,
                        Best_matches, best_matches_img, cv::Scalar::all(-1), cv::Scalar::all(-1),
                        std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
            show_img("feature_Matching", best_matches_img);
            cv::waitKey(10);
        }

        if (clusters.size() > 0) {
            draw_rectangle(gray, cv::Point2f(clusters[good_roi].x + srcM.cols / 2, clusters[good_roi].y + srcM.rows / 2),
                           cv::Point2f(clusters[good_roi].x, clusters[good_roi].y),
                           cv::Point2f(clusters[good_roi].x + srcM.cols, clusters[good_roi].y + srcM.rows));
            show_img("Matching", gray);
        }*/


        free(img1.data);
        gray.release();
        roi.clear();
        roi_info.clear();
        keypoints1.clear();
        matches.clear();

//        free(gray.data);

        count_frame++;
        time(&end);
        ftime_diff = difftime(end,start);
        fps = count_frame/ftime_diff;
        printf("\nControl loop Rate: \t\t\t\t\t  %lf\n\n\n\n", fps);


    }




    /*while(true) {

        key = cv::waitKey(1);
        if (key == 1048675) // which means 'c'
        {
            printf("img_captured!");
            time_t t = time(NULL);
            date = localtime(&t);
            i_time = date->tm_hour * 10000 + date->tm_min * 100 + date->tm_sec;
            s_time = std::to_string(i_time)+ ".png";
            cv::imwrite(s_time, gray);
        }


//        color_img.release();
        color.readFrame(&color_img);

        rc = openni::OpenNI::waitForAnyStream(streams, 2, &changedIndex);
        if (rc != openni::STATUS_OK)
        {
            printf("Wait failed\n");
            exit(0);
        }


        tmp = gray.clone();

        cv::blur( tmp, tmp, cv::Size(3,3) );

        colorPix = new openni::RGB888Pixel[color_img.getHeight() * color_img.getWidth()];
        memcpy(colorPix, color_img.getData(), color_img.getHeight() * color_img.getWidth() * sizeof(uint8_t) * 3);
        img1.data = (uchar *) colorPix;
//        free(colorPix);
        cv::cvtColor(img1,img1, cv::COLOR_BGR2RGB);
        cv::cvtColor(img1,gray, cv::COLOR_RGB2GRAY);

        cv::blur( tmp, tmp, cv::Size(3,3) );
        cv::blur( img1, img1, cv::Size(3,3) );


//        GFTTdetector.detect(tmp, keypoints0);
//        FASTdetector.detect(tmp, keypoints0);
        orb( tmp, cv::Mat(), keypoints0, descriptors0 );


        points_start.clear();
        for (int i(0); i < keypoints0.size(); i++) {
            points_start.push_back(keypoints0[i].pt * scale_factor);
        }

        points_now = points_start;

//            std::cout << "Keypoint size " << keypoints0.size() << std::endl;

        std::vector<cv::Point2f> points1;
        std::vector<unsigned char> status;
        std::vector<float> errr;
        cv::Size winSize(7, 7);
        cv::TermCriteria termcrit = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 10, 0.01);

        if (points_now.size()) {
            cv::calcOpticalFlowPyrLK(tmp, gray, points_now, points1, status, errr, winSize, 3, termcrit, 0, 1e-3f);
            matcher->match(descriptors0, descriptors1, matches);
        }
        points_now.clear();
        std::vector<cv::Point2f> points_tmp(points_start);
        points_start.clear();

        max_u = tmp.cols - min_uv;
        max_v = tmp.rows - min_uv;
        for (int i = 0; i < status.size(); i++) {
            cv::Point2f &pt = points1[i];
            if (status[i] != 0
                && pt.x > min_uv
                && pt.y > min_uv
                && pt.x < max_u
                && pt.y < max_v) {
                points_start.push_back(points_tmp[i]);
                points_now.push_back(pt);
            }
        }

        cimg = gray.clone();
        cv::imshow("oimg", img1);
        free(img1.data);
//        free(gray.data);

        cv::cvtColor( cimg, cimg, CV_GRAY2BGR );
        for (int i(0); i < points_start.size(); i++) {
            cv::circle(cimg, points_now[i], 2, cv::Scalar(0, 0, 255));
            cv::line(cimg, points_now[i], points_start[i], cv::Scalar(0, 255, 255));
        }


        cv::imshow("cimg", cimg);

        printf("\nEnd of show\n");
    }*/



    sampleViewer.run();





}
