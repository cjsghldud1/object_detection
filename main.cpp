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


    auto *colorPix = new openni::RGB888Pixel[color_img.getHeight()*color_img.getWidth()];
    memcpy(colorPix, color_img.getData(), color_img.getHeight()*color_img.getWidth()*sizeof(uint8_t)*3);

    cv::Mat src;
    cv::Mat srcM;
    cv::Mat img1(color_img.getHeight(), color_img.getWidth(), CV_8UC3, colorPix);
    cv::Mat tmp;
    cv::Mat cimg;
    cv::Mat oimg;
    cv::Mat gray;
    cv::Mat roi;
    cv::Mat dummy;
    cv::Mat descriptors0;
    cv::Mat descriptors1;
    cv::Mat best_matches_img;
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

    std::vector<cv::DMatch> Best_matches;
    time_t start, end;
    struct tm *date;
    int scale_factor = 1;
    int changedIndex;
    int min_uv;
    int max_u;
    int max_v;
    int key;
    int count_frame = 0;
    int good_roi;
    int max_good_matches;
    float mean_good_matches;
    float fps = 0;
    float mean_cols = 320;
    float mean_rows = 240;
    double ftime_diff;

    cv::Point dumM;
    cv::Point dumm;
    std::vector<cv::Point> dumM_range;

    std::list<cv::Point> template_match;
    std::vector<cv::Point> clusters;
    double dummM;
    double dummm;


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

        key = cv::waitKey(1);

//        color_img.release();
        color.readFrame(&color_img);



        colorPix = new openni::RGB888Pixel[color_img.getHeight() * color_img.getWidth()];
        memcpy(colorPix, color_img.getData(), color_img.getHeight() * color_img.getWidth() * sizeof(uint8_t) * 3);
        img1.data = (uchar *) colorPix;
//        free(colorPix);






        cv::cvtColor(img1, gray, cv::COLOR_BGR2RGB);


        img_capture(gray, key);


//        cv::cvtColor(gray, gray, cv::COLOR_RGB2GRAY);
//        cv::medianBlur(gray, gray, 3);
//        cv::GaussianBlur(gray, gray, cv::Size(3,3), 0 );
//        cv::blur(gray, gray, cv::Size(7,7));
        cv::cvtColor(img1, img1, cv::COLOR_BGR2RGB);
//        cv::bilateralFilter(img1, gray,9,75,75);






//        GFTTdetector.detect(gray, keypoints1);
//        FASTdetector.detect(gray, keypoints1);
//        orb(gray, cv::Mat(), keypoints1, descriptors1, false);
//        Dbrisk(gray, cv::Mat(), keypoints1, descriptors1, false );
//        printf("\n%d", keypoints0.size());
//        printf("\n%d", keypoints1.size());




//        draw_keypoint(gray, keypoints1);

        show_img("img_keypoint", gray);


//        draw_keypoint(gray, keypoints1);

        //        cv::cvtColor( src, src, CV_RGB2GRAY );
        std::vector<cv::Point2f> points1;
        std::vector<unsigned char> status;
        std::vector<float> errr;
        cv::Size winSize(7, 7);



        cv::matchTemplate(gray, srcM, dummy, cv::TM_CCOEFF_NORMED);
        cv::minMaxLoc(dummy, &dummm, &dummM,&dumm, &dumM,cv::Mat());
        printf("    %f   %d,%d",dummM,dumM.x ,dumM.y);


        if( dummM > 0.35)
        {
            cv::normalize(dummy, dummy, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());
            for (int i = 0; i < dummy.rows; i++) {
                for (int j = 0; j < dummy.cols; j++) {
                    if (dummy.at<float>(i, j) > 0.90)
                        template_match.push_back(cv::Point(j, i));
                }
            }
//
        }

        cv::cvtColor(dummy,dummy, cv::COLOR_GRAY2RGB);

//            for(int i = 0; i < dumM_range.size(); i++)
//                cv::circle(dummy,dumM_range[i],5, cv::Scalar(0, 0, 255),2);
//
        std::vector<cv::Mat> roi;
        if (template_match.size() > 0) {
            simple_cluster(template_match, clusters, 100);

            for(int i = 0; i < clusters.size(); i++)
            {
                cv::circle(dummy, clusters[i], 10, cv::Scalar(0, 255, 0), 2);

                cv::Rect rect(clusters[i].x, clusters[i].y, srcM.cols, srcM.rows);

                roi.push_back(gray(rect));

            }

        }

        show_img("dummy", dummy);



        show_img("img_keypoint", gray);
        mean_good_matches = 1000;
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


            drawMatches(src, keypoints0, roi[good_roi], Best_keypoints1,
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
        }




        good_roi = 0;
        free(img1.data);
        roi.clear();
        dumM_range.clear();
        template_match.clear();
        clusters.clear();
        descriptors1.release();
        matches.clear();
        Best_matches.clear();
//        free(gray.data);

        count_frame++;
        time(&end);
        ftime_diff = difftime(end,start);
        fps = count_frame/ftime_diff;
        printf("\nEnd of show  \t\t\t\t\t  %lf\n", fps);


    }




	sampleViewer.run();





}
