/* Copyright (C) 2013-2016, The Regents of The University of Michigan.
All rights reserved.

This software was developed in the APRIL Robotics Lab under the
direction of Edwin Olson, ebolson@umich.edu. This software may be
available under alternative licensing terms; contact the address above.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the Regents of The University of Michigan.
*/

#include <iostream>
#include <hiredis.h>

#include "opencv2/opencv.hpp"

#include "common/homography.h"
#include "apriltag.h"
#include "tag36h11.h"
#include "tag36h10.h"
#include "tag36artoolkit.h"
#include "tag25h9.h"
#include "tag25h7.h"
#include "common/getopt.h"

using namespace std;
using namespace cv;

void cameraPoseFromHomography(const Mat& H, Mat& pose)
{
    pose = Mat::eye(3, 4, CV_32FC1);      // 3x4 matrix, the camera pose
    float norm1 = (float)norm(H.col(0));  
    float norm2 = (float)norm(H.col(1));  
    float tnorm = (norm1 + norm2) / 2.0f; // Normalization value

    Mat p1 = H.col(0);       // Pointer to first column of H
    Mat p2 = pose.col(0);    // Pointer to first column of pose (empty)

    cv::normalize(p1, p2);   // Normalize the rotation, and copies the column to pose

    p1 = H.col(1);           // Pointer to second column of H
    p2 = pose.col(1);        // Pointer to second column of pose (empty)

    cv::normalize(p1, p2);   // Normalize the rotation and copies the column to pose

    p1 = pose.col(0);
    p2 = pose.col(1);

    Mat p3 = p1.cross(p2);   // Computes the cross-product of p1 and p2
    Mat c2 = pose.col(2);    // Pointer to third column of pose
    p3.copyTo(c2);       // Third column is the crossproduct of columns one and two

    pose.col(3) = H.col(2) / tnorm;  //vector t [R|t] is the last column of pose
}

int main(int argc, char *argv[])
{

    redisContext *c;
    redisReply *reply;
    const char *hostname =  "127.0.0.1";
    int port = 6379;

    struct timeval timeout = { 1, 500000 }; // 1.5 seconds
    c = redisConnectWithTimeout(hostname, port, timeout);
    if (c == NULL || c->err) {
        if (c) {
            printf("Connection error: %s\n", c->errstr);
            redisFree(c);
        } else {
            printf("Connection error: can't allocate redis context\n");
        }
        exit(1);
    }

    getopt_t *getopt = getopt_create();

    getopt_add_bool(getopt, 'h', "help", 0, "Show this help");
    getopt_add_bool(getopt, 'd', "debug", 0, "Enable debugging output (slow)");
    getopt_add_bool(getopt, 'q', "quiet", 0, "Reduce output");
    getopt_add_string(getopt, 'f', "family", "tag36h11", "Tag family to use");
    getopt_add_int(getopt, '\0', "border", "1", "Set tag family border size");
    getopt_add_int(getopt, '\0', "th", "500", "Set target image height");
    getopt_add_int(getopt, '\0', "tw", "500", "Set target image width");

    getopt_add_int(getopt, 't', "threads", "4", "Use this many CPU threads");
    getopt_add_double(getopt, 'x', "decimate", "1.0", "Decimate input image by this factor");
    getopt_add_double(getopt, 'b', "blur", "0.0", "Apply low-pass blur to input");
    getopt_add_bool(getopt, '0', "refine-edges", 1, "Spend more time trying to align edges of tags");
    getopt_add_bool(getopt, '1', "refine-decode", 0, "Spend more time trying to decode tags");
    getopt_add_bool(getopt, '2', "refine-pose", 0, "Spend more time trying to precisely localize tags");

    if (!getopt_parse(getopt, argc, argv, 1) ||
            getopt_get_bool(getopt, "help")) {
        printf("Usage: %s [options]\n", argv[0]);
        getopt_do_usage(getopt);
        exit(0);
    }

    // Initialize camera
    VideoCapture cap(0);
    if (!cap.isOpened()) {
        cerr << "Couldn't open video capture device" << endl;
        return -1;
    }

    // Initialize tag detector with options
    apriltag_family_t *tf = NULL;
    const char *famname = getopt_get_string(getopt, "family");
    if (!strcmp(famname, "tag36h11"))
        tf = tag36h11_create();
    else if (!strcmp(famname, "tag36h10"))
        tf = tag36h10_create();
    else if (!strcmp(famname, "tag36artoolkit"))
        tf = tag36artoolkit_create();
    else if (!strcmp(famname, "tag25h9"))
        tf = tag25h9_create();
    else if (!strcmp(famname, "tag25h7"))
        tf = tag25h7_create();
    else {
        printf("Unrecognized tag family name. Use e.g. \"tag36h11\".\n");
        exit(-1);
    }
    tf->black_border = getopt_get_int(getopt, "border");

    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    td->quad_decimate = getopt_get_double(getopt, "decimate");
    td->quad_sigma = getopt_get_double(getopt, "blur");
    td->nthreads = getopt_get_int(getopt, "threads");

    td->debug = getopt_get_bool(getopt, "debug");
    td->refine_edges = getopt_get_bool(getopt, "refine-edges");
    td->refine_decode = getopt_get_bool(getopt, "refine-decode");
    td->refine_pose = getopt_get_bool(getopt, "refine-pose");

    Mat frame, gray;
    Mat undist;
    // have we seen the board? 
    bool initialized = false;
    float board_width = 0.0f;
    float board_height = 0.0f;
    vector<Vec2f> target_loc;
    auto th = getopt_get_int(getopt,"th");
    auto tw = getopt_get_int(getopt,"tw");
    cv::Mat out_image(th,tw,CV_8UC3);
    BackgroundSubtractorMOG2 bgs(360,36,false);

    while (true) {
        bool new_detection = false;
        cap >> frame;
        cvtColor(frame, gray, COLOR_BGR2GRAY);

        // Make an image_u8_t header for the Mat data
        image_u8_t im = { .width = gray.cols,
            .height = gray.rows,
            .stride = gray.cols,
            .buf = gray.data
        };

        zarray_t *detections = apriltag_detector_detect(td, &im);
        auto num_detect = zarray_size(detections);
        //cout << num_detect << " tags detected" << endl;

        if ( target_loc.size() == 16 && num_detect > 0 && num_detect < 4) {

            std::vector<Vec2f> target_pts,src_pts;
            for (int i = 0; i < num_detect; i++) {
                apriltag_detection_t *det;
                zarray_get(detections, i, &det);
                auto id = det->id;
                for(int j=0; j < 4; j++) {
                    src_pts.emplace_back(det->p[j][0],det->p[j][1]);
                    target_pts.push_back(target_loc[4*id+j]);
                }
            }
            //cout << src_pts.size() << '\t' << target_pts.size() << endl;
            //for(int i=0; i < num_detect*4; i++)
            //    cout << src_pts[i] << endl << target_pts[i] << endl;
            Mat sMat,tMat;
            Mat(src_pts).convertTo(sMat, CV_32FC2);
            Mat(target_pts).convertTo(tMat, CV_32FC2);
            auto T = findHomography(sMat,tMat);
            warpPerspective(frame,out_image,T,out_image.size());
            new_detection = true;
        }
        if(true && zarray_size(detections) == 4) {

            std::vector<Vec2f> src_pts;
            cv::Mat H(3,3,CV_32F);

            for (int i = 0; i < zarray_size(detections); i++) {
                apriltag_detection_t *det;
                zarray_get(detections, i, &det);
                //cout << det->id << '\t' << det->c[0] << '\t' << det->c[1] << '\t';
                for(int j=0; j < 9; j++) {
                    H.at<float>(j) = det->H->data[j];
                    //cout << det->H->data[j] << ", ";
                }
                H /= H.at<float>(8);
                //cout <<endl << H << endl;
                //cout << endl;
                src_pts.emplace_back(det->c[0],det->c[1]);

            }
            Mat R,Q,Qx,Qy,Qz;
            std::vector<Vec2f> target_pts = {{0,0}, {tw,0}, {0,th}, {tw,th}};
            auto T = getPerspectiveTransform(src_pts,target_pts);
            auto angl = RQDecomp3x3(T,R,Q,Qx,Qy,Qz);
            //cout << angl << endl;
            //cout << Qx << '\n' <<  Qy  << '\n' << Qz << endl;
            Mat angl2;
            cameraPoseFromHomography(T,angl2);
            //cout << angl2 <<endl;
            //cout << T <<endl;
            Mat invT;
            invert(T,invT);
            vector<Vec2f> corner_p;
            for(int i=0; i < 4; i++) {
                apriltag_detection_t *det;
                zarray_get(detections, i, &det);
                for(int j=0; j < 4; j++) {
                    corner_p.emplace_back(det->p[j][0],det->p[j][1]);
                }
            }
            auto out_p = corner_p;
            perspectiveTransform(corner_p,out_p,T);
            //for(auto & p : out_p)
            //    cout << p << endl;
            target_loc = out_p;
            warpPerspective(frame,out_image,T,out_image.size());
            new_detection = true;

        }

        redisCommand(c,"SET %s %s", "maze::detections", to_string(num_detect).c_str());

        if(num_detect) {
            cv::Mat fg_mask;
            bgs(out_image,fg_mask);
            Moments m = moments(fg_mask, false);
            Point2f p1(m.m10/m.m00, m.m01/m.m00);
            Point2f var(sqrt(m.m20/m.m00 - p1.x*p1.x), sqrt(m.m02/m.m00-p1.y*p1.y));

            imshow("hah",out_image);
            cv::circle(fg_mask,p1,10,128,3);
            imshow("hah2",fg_mask);
            var.x /= tw;
            var.y /= th;
            p1.x /= tw;
            p1.y /= th;
            redisCommand(c,"SET %s %s", "maze::x", to_string(p1.x).c_str());
            redisCommand(c,"SET %s %s", "maze::y", to_string(p1.y).c_str());
            redisCommand(c,"SET %s %s", "maze::stdx", to_string(var.x).c_str());
            redisCommand(c,"SET %s %s", "maze::stdy", to_string(var.y).c_str());

            cout <<  p1 <<"\t" <<  var << endl;


        }
        // Draw detection outlines
        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);
            line(frame, Point(det->p[0][0], det->p[0][1]),
                     Point(det->p[1][0], det->p[1][1]),
                     Scalar(0, 0xff, 0), 2);
            line(frame, Point(det->p[0][0], det->p[0][1]),
                     Point(det->p[3][0], det->p[3][1]),
                     Scalar(0, 0, 0xff), 2);
            line(frame, Point(det->p[1][0], det->p[1][1]),
                     Point(det->p[2][0], det->p[2][1]),
                     Scalar(0xff, 0, 0), 2);
            line(frame, Point(det->p[2][0], det->p[2][1]),
                     Point(det->p[3][0], det->p[3][1]),
                     Scalar(0xff, 0, 0), 2);

            stringstream ss;
            ss << det->id;
            String text = ss.str();
            int fontface = FONT_HERSHEY_SCRIPT_SIMPLEX;
            double fontscale = 1.0;
            int baseline;
            Size textsize = getTextSize(text, fontface, fontscale, 2,
                                            &baseline);
            putText(frame, text, Point(det->c[0]-textsize.width/2,
                                       det->c[1]+textsize.height/2),
                    fontface, fontscale, Scalar(0xff, 0x99, 0), 2);
        }
        zarray_destroy(detections);

        imshow("Tag Detections", frame);
        if (waitKey(1) == 'q')
            break;
    }

    apriltag_detector_destroy(td);
    if (!strcmp(famname, "tag36h11"))
        tag36h11_destroy(tf);
    else if (!strcmp(famname, "tag36h10"))
        tag36h10_destroy(tf);
    else if (!strcmp(famname, "tag36artoolkit"))
        tag36artoolkit_destroy(tf);
    else if (!strcmp(famname, "tag25h9"))
        tag25h9_destroy(tf);
    else if (!strcmp(famname, "tag25h7"))
        tag25h7_destroy(tf);
    getopt_destroy(getopt);

    return 0;
}
