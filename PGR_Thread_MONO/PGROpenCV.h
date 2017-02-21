#ifndef PGROPENCV_H
#define PGROPENCV_H


#define PGR_PARAMETER_FILE "./parameter.ini"

#define DOT_SIZE 150
#define A_THRESH_VAL -5
#define DOT_THRESH_VAL_MIN 50  // �h�b�g�m�C�Y�e��
#define DOT_THRESH_VAL_MAX 500 // �G�b�W�m�C�Y�e��
#define DOT_THRESH_VAL_BRIGHT 100 //�h�b�g�_�̖��邳
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

	//**�h�b�g���o�֘A**//
	//float RESIZESCALE;
	//double A_THRESH_VAL;
	//int DOT_THRESH_VAL_MIN;  // �h�b�g�m�C�Y�e��
	//int DOT_THRESH_VAL_MAX; // �G�b�W�m�C�Y�e��

	//�c�������̃J�[�l��
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

	// �p�����[�^���Z�b�g���郁�\�b�h
	void setShutterSpeed(float shutterSpeed);
	void setGain(float gain);
	void setWhiteBalance(int r, int b);
	void setPixelFormat( FlyCapture2::PixelFormat format );
	void setColorProcessingAlgorithm( FlyCapture2::ColorProcessingAlgorithm algorithm );
	void setGamma(float gamma);
	void setBrightness(float brightness);
	void setFrameRate(float framerate);

	// �p�����[�^���擾���郁�\�b�h
	float getShutterSpeed();
	float getGain();
	void getWhiteBalance(int &r, int &b);
	float getFramerate();
	void showCapImg(std::string winname, cv::Mat cap = cv::Mat());	//�B�e�摜��\��
	void CameraCapture(cv::Mat &image);			// �B�e�摜��Mat�Ŏ擾

	//cv::Mat getVideo(){ return fc2Mat; };
	cv::Mat getVideo();
	cv::Mat getResultVideo();

	//**�h�b�g���o�֘A**//
	void setDotsParameters(double AthreshVal, int DotThreshValMin, int DotThreshValMax, float resizeScale);
	int getDotsCount();
	void getDotsData(std::vector<int> &data);

	Timer tm;

protected:
	boost::thread thread;
	mutable boost::mutex mutex;

	//�X���b�h����
	void threadFunction();

	bool quit;
	bool running;

	//! �X���b�h�Ԃ̋��L�N���X
	boost::shared_ptr<criticalSection> critical_section;

};

#endif