#include <FlyCapture2.h>
#include <opencv2\opencv.hpp>
#include <iostream>
#include <cstdio>
#include <atltime.h>


#include "PGROpenCV.h"

#define CAMERA_WIDTH 1920
#define CAMERA_HEIGHT 1200

#define DOT_SIZE 150
#define A_THRESH_VAL -5
#define DOT_THRESH_VAL_MIN 20  // ドットノイズ弾き
#define DOT_THRESH_VAL_MAX 500 // エッジノイズ弾き



void calCoG_dot_v0(cv::Mat &src, cv::Point& sum, int& cnt, cv::Point& min, cv::Point& max, cv::Point p) 
{
	if (src.at<uchar>(p)) {
		sum += p; cnt++;
		src.at<uchar>(p) = 0;
		if (p.x<min.x) min.x = p.x;
		if (p.x>max.x) max.x = p.x;
		if (p.y<min.y) min.y = p.y;
		if (p.y>max.y) max.y = p.y;

		if (p.x - 1 >= 0) calCoG_dot_v0(src, sum, cnt, min, max, cv::Point(p.x-1, p.y));
		if (p.x + 1 < CAMERA_WIDTH) calCoG_dot_v0(src, sum, cnt, min, max, cv::Point(p.x + 1, p.y));
		if (p.y - 1 >= 0) calCoG_dot_v0(src, sum, cnt, min, max, cv::Point(p.x, p.y - 1));
		if (p.y + 1 < CAMERA_HEIGHT) calCoG_dot_v0(src, sum, cnt, min, max, cv::Point(p.x, p.y + 1));
	}
}

bool init_v0(cv::Mat &src) 
{
	//cv::Mat origSrc = src.clone();
	//cv::Mat resized;
	//cv::resize(src, resized, cv::Size(), RESIZESCALE, RESIZESCALE);

	////適応的閾値処理
	//cv::adaptiveThreshold(resized, resized, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 7, A_THRESH_VAL);
	////膨張処理
	//cv::dilate(resized, resized, cv::Mat());

	//cv::Mat ptsImg = cv::Mat::zeros( CAMERA_HEIGHT, CAMERA_WIDTH, CV_8UC1); //原寸大表示用
	//cv::resize(resized, ptsImg, cv::Size(), 1/RESIZESCALE, 1/RESIZESCALE);
	cv::Mat ptsImgColor; 
	cv::cvtColor(src, ptsImgColor, CV_GRAY2BGR);

	//処理時間計測
	CFileTime cTimeStart_, cTimeEnd_;
	CFileTimeSpan cTimeSpan_;

	cTimeStart_ = CFileTime::GetCurrentTime();// 現在時刻


	cv::Point sum, min, max, p;
	int cnt;
	std::vector<cv::Point> dots;
	for (int i = 0; i < src.rows; i++) {
		for (int j = 0; j < src.cols; j++) {
			if (src.at<uchar>(i, j)) {
				sum = cv::Point(0, 0); cnt = 0; min = cv::Point(j, i); max = cv::Point(j, i);
				calCoG_dot_v0(src, sum, cnt, min, max, cv::Point(j, i));
				if (cnt>DOT_THRESH_VAL_MIN && max.x - min.x < DOT_THRESH_VAL_MAX && max.y - min.y < DOT_THRESH_VAL_MAX) {
					dots.push_back(cv::Point(sum.x / cnt, sum.y / cnt));
					//dots.push_back(cv::Point((int)((float)(sum.x / cnt) / RESIZESCALE + 0.5), (int)((float)(sum.y / cnt) / RESIZESCALE + 0.5)));
				}
			}
		}
	}

	cTimeEnd_ = CFileTime::GetCurrentTime();           // 現在時刻
	cTimeSpan_ = cTimeEnd_ - cTimeStart_;

	std::cout << std::to_string(cTimeSpan_.GetTimeSpan()/10000) << std::endl;

	//cv::rectangle(ptsImg, cv::Point(CamWidth / 4, CamHeight / 4), cv::Point(CamWidth * 3 / 4, CamHeight * 3 / 4), cv::Scalar(255, 0, 0), 5, 4);
	// OpenGL用に予めRGB用のデータ作成
	//cv::rectangle(ptsImg, cv::Point(CamWidth / 4, CamHeight / 4), cv::Point(CamWidth * 3 / 4, CamHeight * 3 / 4), cv::Scalar(0, 0, 255), 5, 4);
	std::vector<cv::Point>::iterator it = dots.begin();
	
	bool k = (dots.size()==DOT_SIZE);
	//for (int i=0; it != dots.end(); i++,++it) {
	//	if (i && i%MarkersWidth == 0) {
	//		if ((*it).y <= (*(it - 1)).y) k = false;
	//	} else {
	//		if (i && (*it).x <= (*(it - 1)).x) k = false;
	//	}
	//}
	//cv::Scalar co = k ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
	// OpenGL用に予めRGB用のデータ作成
	cv::Scalar color = k ? cv::Scalar(255, 0, 0) : cv::Scalar(0, 255, 0);
	for (it = dots.begin(); it != dots.end(); ++it) {
		cv::circle(ptsImgColor, *it, 3, color, 2);
	}

	//if (k) {
	//	it = dots.begin();
	//	for (int i = 0; it != dots.end(); ++it) {
	//		marker_u[i / MarkersWidth][i%MarkersWidth] = *it;
	//		marker_s[i / MarkersWidth][i%MarkersWidth] = true;
	//		i++;
	//	}
	//	
	//	flag = 1;
	//	std::cout << "init complete!" << std::endl;
	//	
	//}

	cv::Mat resize_src, resize_pts;
	//cv::resize(origSrc, resize_src, cv::Size(), 0.5, 0.5);
	cv::resize(ptsImgColor, resize_pts, cv::Size(), 1.0, 1.0);

	//cv::imshow("src", resize_src);
	cv::imshow("result", resize_pts);

	return k;
}


std::vector<cv::Point2f> corners;
//コーナー点を検出(srcはモノクロ)
cv::Mat detectCorner(const cv::Mat &src)
{
	//コーナー検出
	cv::goodFeaturesToTrack(src, corners, 100, 0.01, 50);
	//高精度化
	//cv::cornerSubPix(src, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03));



	//描画したものを返す
	cv::Mat drawimage = cv::Mat(src.rows, src.cols, CV_8UC3);
	//memcpy(drawimage.data, src.data, drawimage.rows * drawimage.cols * drawimage.channels());
	cv::cvtColor(src, drawimage, CV_GRAY2BGR);

	//描画
	for(int i = 0; i < corners.size(); i++)
	{
		cv::circle(drawimage, corners[i], 1, cv::Scalar(0, 0, 255), 3);
	}
	return drawimage;

}

//適応的閾値のテスト(srcはモノクロ)
//普通の二値化と比較
void adaptiveThresholdTest(const cv::Mat &src)
{
		//適応的閾値処理の確認//
		cv::Mat adapBinImg, binImg;
		cv::adaptiveThreshold(src, adapBinImg, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 7, A_THRESH_VAL);
		//cv::threshold(src, binImg, 150, 255, cv::THRESH_BINARY);

		cv::Mat resizeAdap, resizeBin;
		cv::resize(adapBinImg, resizeAdap, cv::Size(adapBinImg.cols * 0.5, adapBinImg.rows * 0.5));
		//cv::resize(binImg, resizeBin, cv::Size(binImg.cols * 0.5, binImg.rows * 0.5));

		//cv::imshow("Threshold", resizeBin);
		cv::imshow("adaptiveThreshold", resizeAdap);
		//適応的閾値処理の確認//
}

int main( int argc, char* argv[] )
{
	//! スレッド間の共有クラス
	//boost::shared_ptr<criticalSection> critical_section = boost::shared_ptr<criticalSection> (new criticalSection);

	//boost::shared_ptr<imgSrc> imgsrc = boost::shared_ptr<imgSrc>(new imgSrc);
	//imgsrc->image = cv::Mat::zeros(CAMERA_HEIGHT, CAMERA_WIDTH, CV_8UC3);


	// 起動したいカメラインデックスを指定
	TPGROpenCV	pgrOpenCV(0);
	cv::Mat cap;

	// initialization
	pgrOpenCV.init(FlyCapture2::PIXEL_FORMAT_RGB8, FlyCapture2::HQ_LINEAR); //**
	// start capturing
	pgrOpenCV.start();
	for( ;; ) {
		pgrOpenCV.tm.restart();

		//pgrOpenCV.queryFrame();
		//critical_section->getImageSource(imgsrc);
		cv::Mat frame = pgrOpenCV.getVideo();//元画像
		cv::Mat result = pgrOpenCV.getResultVideo();//ドット検出結果

		//適応的閾値処理と普通の二値化の比較
		//adaptiveThresholdTest(pgrOpenCV.getVideo());
		if(!frame.empty())
		{
			//cv::Mat currFrameGray;
			//cv::cvtColor(frame, currFrameGray, CV_RGB2GRAY);
			//init_v0(frame);
			//適応的二値化
			//adaptiveThresholdTest(currFrameGray);

			pgrOpenCV.showCapImg(frame);
			pgrOpenCV.showCapImg(result);

		}

		pgrOpenCV.tm.elapsed();

		//int count = pgrOpenCV.getDotsCount();
		//std::cout << "count:" << count << std::endl;
		//std::vector<int> data;
		//pgrOpenCV.getDotsData(data);
		//for(int i = 0; i < data.size(); i++)
		//{
		//	std::cout << data[i] << std::endl;
		//}

		//int* dotsdata = pgrOpenCV.getDotsData();
		//for(int i = 0; i < count*2; i++)
		//{
		//	std::cout << dotsdata[i] << std::endl;
		//}


		//コーナー点を検出
		//pgrOpenCV.showCapImg(detectCorner(pgrOpenCV.getVideo()));
		//ドット検出
		//init_v0(pgrOpenCV.getVideo());
		//ノーマル
		//pgrOpenCV.showCapImg(pgrOpenCV.getVideo());
		
		if( cv::waitKey( 10 ) == ' ' ) {
			break;
		}

	}

	// stop capturing
	pgrOpenCV.stop();

	// finalization
	pgrOpenCV.release();

	return 0;
}