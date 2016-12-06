#include <boost/thread/thread.hpp>
#include <opencv2\opencv.hpp>

#define CAMERA_WIDTH 1920
#define CAMERA_HEIGHT 1200

struct imgSrc
{
	cv::Mat image;//元画像
	cv::Mat result_image;//ドット検出に使った画像とドット検出結果
	int dotsCount; //検出されたドットの数
	std::vector<int> dots_data;//ドットの座標データ
};

class criticalSection
{
public:
	criticalSection()
	{
		imgsrc = boost::shared_ptr<imgSrc>(new imgSrc);
		imgsrc->image = cv::Mat::zeros(CAMERA_HEIGHT, CAMERA_WIDTH, CV_8UC3);//**
		imgsrc->result_image = cv::Mat::zeros(CAMERA_HEIGHT, CAMERA_WIDTH, CV_8UC3);

		//image = cv::Mat::zeros(CAMERA_HEIGHT, CAMERA_WIDTH, CV_8UC3);
	}

	~criticalSection(){};


	inline void setImageSource(boost::shared_ptr<imgSrc> _src)
	{
		boost::upgrade_lock<boost::shared_mutex> up_lock(image_mutex);
		boost::upgrade_to_unique_lock<boost::shared_mutex> write_lock(up_lock);
		if(!_src->image.empty())
		{
			imgsrc = _src;
		}
	}

	inline bool getImageSource(boost::shared_ptr<imgSrc>& _src)
	{
		boost::shared_lock<boost::shared_mutex> read_lock(image_mutex);
		_src = imgsrc;
		return true;
	}

	//inline cv::Mat getImage()
	//{
	//	boost::shared_lock<boost::shared_mutex> read_lock(image_mutex);
	//	return image;
	//}

	//inline void setImage(const cv::Mat &img)
	//{
	//	boost::upgrade_lock<boost::shared_mutex> up_lock(image_mutex);
	//	boost::upgrade_to_unique_lock<boost::shared_mutex> write_lock(up_lock);
	//	if(!img.empty())
	//	{
	//		image = img;
	//	}
	//}

private:
	boost::shared_mutex image_mutex;
	boost::shared_ptr<imgSrc> imgsrc;

	//cv::Mat image;


};
