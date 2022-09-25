#pragma once

#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include "camera_model/CameraFactory.h"

#include "utility/tic_toc.h"

using namespace std;
using namespace camodocal;
using namespace Eigen;

bool inBorder(const cv::Point2f &pt);

void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);
void reduceVector(vector<int> &v, vector<uchar> status);

class FeatureTracker
{
  public:
    FeatureTracker();

    void readImage(const cv::Mat &_img,double _cur_time, bool pub_this_frame, int equalize);

    void setMask();

    void addPoints();

    bool updateID(unsigned int i);

    void showUndistortion(const string &name);

    void rejectWithF();

    void undistortedPoints();

    cv::Mat mask;

    cv::Mat prev_img, cur_img, forw_img;
    vector<cv::Point2f> n_pts;
    vector<cv::Point2f> prev_pts, cur_pts, forw_pts;
    vector<cv::Point2f> prev_un_pts, cur_un_pts;
    vector<cv::Point2f> pts_velocity;
    vector<int> ids;
    vector<int> track_cnt;
    map<int, cv::Point2f> cur_un_pts_map;
    map<int, cv::Point2f> prev_un_pts_map;
    camodocal::CameraPtr m_camera;
    double cur_time;
    double prev_time;

	int row;
	int col;
	double focal_length;
	int max_cnt;
	int min_dist;
	double f_threshold;

    static int n_id;
};
