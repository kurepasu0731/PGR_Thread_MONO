#ifndef PGROPENCV_H
#define PGROPENCV_H


#define PGR_PARAMETER_FILE "./parameter.ini"

#define DOT_SIZE 150
#define A_THRESH_VAL -5
#define DOT_THRESH_VAL_MIN 50  // ドットノイズ弾き
#define DOT_THRESH_VAL_MAX 500 // エッジノイズ弾き
#define DOT_THRESH_VAL_BRIGHT 100 //ドット点の明るさ
#define RESIZESCALE 1.0


#pragma once

#include <FlyCapture2.h>
#include <opencv2\opencv.hpp>
#include <boost/thread/thread.hpp>
#include <cstdio>
#include <iostream>
#include "Timer.h"
#include "criticalSection.h"

class TPGROpenCV
{
private:
	FlyCapture2::Error			fc2Error;
	FlyCapture2::BusManager		fc2BusMgr;
	FlyCapture2::PGRGuid		fc2Guid;
	FlyCapture2::CameraInfo		fc2CamInfo;
	FlyCapture2::Camera			fc2Cam;
	FlyCapture2::Property		fc2Prop;
	FlyCapture2::PixelFormat	fc2PixelFormat;
	FlyCapture2::Image			fc2Image;

	FlyCapture2::ColorProcessingAlgorithm fc2CPA;
	unsigned int				numCameras;
	unsigned int				useCamIndex;

	float						ShutterSpeed;
	float						Gain;
	float						Gamma;
	float						Brightness;
	float						Framerate;
	unsigned int				Wb_Red;
	unsigned int				Wb_Blue;
	cv::Mat						fc2Mat;
	boost::shared_ptr<imgSrc> imgsrc;

	float						delay;
	
	void loadParameters();

	//**ドット検出関連**//
	//float RESIZESCALE;
	//double A_THRESH_VAL;
	//int DOT_THRESH_VAL_MIN;  // ドットノイズ弾き
	//int DOT_THRESH_VAL_MAX; // エッジノイズ弾き

	//膨張処理のカーネル
	cv::Mat element;

	std::vector<cv::Point> dots;
	std::vector<int> data;

	void calCoG_dot_v0(cv::Mat &src, cv::Point& sum, int& cnt, cv::Point& min, cv::Point& max, cv::Point p);

	bool getDots(cv::Mat &src, std::vector<cv::Point> &dots, double C, int dots_thresh_min, int dots_thresh_max, float resizeScale, cv::Mat &drawimage);


public:
	TPGROpenCV(int _useCameraIndex);
	~TPGROpenCV();
	int init( FlyCapture2::PixelFormat _format = FlyCapture2::PIXEL_FORMAT_BGR, int ColorProcessingAlgorithm = FlyCapture2::ColorProcessingAlgorithm::HQ_LINEAR);
	void PrintBuildInfo();
	void PrintError( FlyCapture2::Error error );
	void PrintCameraInfo( FlyCapture2::CameraInfo* pCamInfo );
	void InitCameraParameter();
	int PixelFormatInOpenCV();
	int start();
	int queryFrame();
	int stop();
	int release();
	std::string windowNameCamera;

	// パラメータをセットするメソッド
	void setShutterSpeed(float shutterSpeed);
	void setGain(float gain);
	void setWhiteBalance(int r, int b);
	void setPixelFormat( FlyCapture2::PixelFormat format );
	void setColorProcessingAlgorithm( FlyCapture2::ColorProcessingAlgorithm algorithm );
	void setGamma(float gamma);
	void setBrightness(float brightness);
	void setFrameRate(float framerate);

	// パラメータを取得するメソッド
	float getShutterSpeed();
	float getGain();
	void getWhiteBalance(int &r, int &b);
	float getFramerate();
	void showCapImg(std::string winname, cv::Mat cap = cv::Mat());	//撮影画像を表示
	void CameraCapture(cv::Mat &image);			// 撮影画像をMatで取得

	//cv::Mat getVideo(){ return fc2Mat; };
	cv::Mat getVideo();
	cv::Mat getResultVideo();

	//**ドット検出関連**//
	void setDotsParameters(double AthreshVal, int DotThreshValMin, int DotThreshValMax, float resizeScale);
	int getDotsCount();
	void getDotsData(std::vector<int> &data);

	Timer tm;

protected:
	boost::thread thread;
	mutable boost::mutex mutex;

	//スレッド処理
	void threadFunction();

	bool quit;
	bool running;

	//! スレッド間の共有クラス
	boost::shared_ptr<criticalSection> critical_section;

};

#endif