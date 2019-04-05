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
    std::vector<cv::Point2f> points_start;
    std::vector<cv::Point2f> points_now;
    std::vector< cv::DMatch> matches;
    std::vector<cv::DMatch> good_matches;
    time_t start, end;
    struct tm *date;
    int scale_factor = 1;
    int changedIndex;
    int min_uv;
    int max_u;
    int max_v;
    int key;
    int count_frame = 0;
    float fps = 0;
    float mean_cols = 320;
    float mean_rows = 240;
    double ftime_diff;

    cv::Point dumM;
    cv::Point dumm;
    std::vector<cv::Point> dumM_range;
    double dummM;
    double dummm;


    openni::VideoStream **streams;
    streams = new openni::VideoStream*[2];
    streams[0] = &depth;
    streams[1] = &color;

    cv::waitKey(0);
    cv::namedWindow("Good Matches & Object detection", cv::WINDOW_AUTOSIZE);
    cv::moveWindow("Good Matches & Object detection",0,0);
    cv::cvtColor(img1,img1, cv::COLOR_BGR2RGB);
    cv::cvtColor(img1,gray, cv::COLOR_RGB2GRAY);
    cv::destroyWindow("tset");
    printf("111111111111111111\n");



    src = cv::imread("GC120_front_scaled.png");
//    cv::cvtColor(src,src, cv::COLOR_RGB2GRAY);
//    cv::blur( src, src, cv::Size(3,3) );
//        cv::cvtColor(gray, gray, cv::COLOR_RGB2GRAY);
//        cv::medianBlur(gray, gray, 3);
//        cv::GaussianBlur(gray, gray, cv::Size(3,3), 0 );
//        cv::blur(gray, gray, cv::Size(7,7));
    cv::Size2f size(0,0);
    cv::resize(src,src,size,0.3,0.3);
//    GFTTdetector.detect(src, keypoints0);
//            FASTdetector.detect(src, keypoints0);
    orb(src, cv::Mat(), keypoints0, descriptors0, false);
//    Dbrisk(src, cv::Mat(), keypoints0, descriptors0, false );
//
    draw_keypoint("src_keypoint", src, keypoints0);

    srcM = cv::imread("GC120_head.png");
    cv::resize(srcM,srcM,size,0.1,0.1);
    cv::imshow("srcM", srcM);


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
        orb(gray, cv::Mat(), keypoints1, descriptors1, false);
//        Dbrisk(gray, cv::Mat(), keypoints1, descriptors1, false );
//        printf("\n%d", keypoints0.size());
//        printf("\n%d", keypoints1.size());




        draw_keypoint("img_keypoint", gray, keypoints1);




        //        cv::cvtColor( src, src, CV_RGB2GRAY );
        std::vector<cv::Point2f> points1;
        std::vector<unsigned char> status;
        std::vector<float> errr;
        cv::Size winSize(7, 7);


        if (descriptors0.rows > 0 && descriptors1.rows > 0) {

            matcher->match(descriptors0, descriptors1, matches);
//        matcherf.match( descriptors0, descriptors1, matches  );
            printf("\n%d", matches.size());
            if (matches.size() > 9) {
                double max_dist = 0;
                double min_dist = 100;
                double mean = 0;
                //-- Quick calculation of max and min distances between keypoints
                for (int i = 0; i < descriptors0.rows; i++) {
                    double dist = matches[i].distance;
                    mean += dist;
                    if (dist < min_dist) min_dist = dist;
                    if (dist > max_dist) max_dist = dist;
                }

//                printf("-- Max dist : %f \n", max_dist);
                printf("-- Min dist : %f \n", min_dist);
//                printf("-- Mean dist : %f \n", mean / matches.size());
                //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )


                for (int i = 0; i < descriptors0.rows; i++) {
                    if (matches[i].distance < 35) { good_matches.push_back(matches[i]); }
                    }
                printf("\n%d", good_matches.size());
            }



            cv::Mat img_matches;
            match_img(src, keypoints0,  gray, keypoints1,  good_matches, img_matches);


            mean_cols = 0;
            mean_rows = 0;
            for (int i = 0; i < good_matches.size(); i++)
            {
                //-- Get the keypoints from the good matches
                mean_cols += keypoints1[good_matches[i].trainIdx].pt.x;
                mean_rows += keypoints1[good_matches[i].trainIdx].pt.y;
            }

            mean_cols /= good_matches.size();
            mean_rows /= good_matches.size();

            cv::Rect rect(int(mean_cols - 160 < 0 ? 0 : mean_cols - 160), int(mean_rows - 120 < 0 ? 0 : mean_rows - 120),
                              int(mean_cols + 160 > 640 ? 640 - mean_cols + 160 : 320),
                              int(mean_rows + 120 > 480 ? 480 - mean_rows + 120 : 240));

            draw_rectangle(img_matches, cv::Point2f(src.cols + mean_cols, mean_rows),cv::Point2f(src.cols + rect.x, rect.y),cv::Point2f(src.cols + rect.x, rect.y) + cv::Point2f(rect.width, rect.height));






            cv::matchTemplate(gray, srcM, dummy, cv::TM_CCOEFF_NORMED);
//            cv::normalize( dummy, dummy, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() );
            for(int i =0; i <dummy.rows;i++)
            {
                for(int j = 0; j <dummy.cols; j++)
                {
                    if (dummy.at<float>(i,j)  > 0.4)
                        dumM_range.push_back(cv::Point(j,i));
                }
            }
//
            cv::minMaxLoc(dummy, &dummm, &dummM,&dumm, &dumM,cv::Mat());
            printf("    %f   %d,%d",dummM,dumM.x ,dumM.y);
            cv::cvtColor(dummy,dummy, cv::COLOR_GRAY2RGB);

            for(int i = 0; i < dumM_range.size(); i++)
                cv::circle(dummy,dumM_range[i],10, cv::Scalar(0, 0, 255),10);

            cv::imshow("dummy", dummy);
            draw_rectangle(img_matches, cv::Point2f(src.cols + srcM.cols/2 + dumM.x, srcM.rows/2 + dumM.y),
                           cv::Point2f(src.cols + dumM.x, dumM.y),
                           cv::Point2f(src.cols + srcM.cols + dumM.x, srcM.rows + dumM.y));
            /*draw_rectangle(img_matches, cv::Point2f(src.cols + srcM.cols/2 + dumm.x, srcM.rows/2 + dumm.y),
                           cv::Point2f(src.cols + dumm.x, dumm.y),
                           cv::Point2f(src.cols + srcM.cols + dumm.x, srcM.rows + dumm.y));
*/

            cv::imshow("Good Matches & Object detection", img_matches);
            cv::moveWindow("Good Matches & Object detection", 0, 0);

        }

        free(img1.data);
        good_matches.clear();
        dumM_range.clear();
        descriptors1.release();
        matches.clear();
//        free(gray.data);

        count_frame++;
        time(&end);
        ftime_diff = difftime(end,start);
        fps = count_frame/ftime_diff;
        printf("\nEnd of show  \t\t\t\t\t  %lf\n", fps);


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