/****************************************************************************
目的：    定义视觉识别二维码估计相机位姿类
编写时间： 2018.08.06
作者：    Yibin Woo
版本:     V1.0
版权：    武汉大学GNSS中心
****************************************************************************/
#ifndef _QRCODESOLVE_H_
#define _QRCODESOLVE_H_

#include <eigen3/Eigen/Core>
#include <ceres/ceres.h>
class QRCodeSolve
{
  private:
	std::vector<cv::Point3f> m_markerCorners3d;
	std::vector<cv::Point2f> m_markerCorners2d;
	std::vector<cv::Point3f> m_markerinterCorners3d;
	std::vector<cv::Point2f> m_markerinterCorners2d;
	std::vector<cv::Point3f> m_markerinterCorners3d4cal;

	std::vector<cv::Point3f> m_squareCorners3d;
	std::vector<cv::Point2f> m_squareCorners2d;

	cv::Size2f markerSize;
	cv::Size2f markerSizeinWorld;

	cv::Mat camMatrix;
	cv::Mat distCoeff;

	cv::Mat originimg;
	cv::Mat m_Mask;
	int img_height;
	int img_width;

	int imgthr[7] = {227, 207, 187, 167, 147, 127, 107};

	//
	FILE *results, *errorimg;

  public:
	char imgdir[1000];
	int imgindex = 0;

	double Q_wc[4] = {0.0, 0.0, 0.0, 0.0};
	double T_wc[3] = {0.0, 0.0, 0.0};

  public:
	//输入图像路径和输出文件路径
	QRCodeSolve(char imgdir_[1000], char outputdir_[1000], int cameratype, int markertype, cv::Size2f markerSize_ = cv::Size2f(100.0, 100.0), cv::Size2f markerSizeinWorld_ = cv::Size2f(500.0, 500.0));
	~QRCodeSolve();
	//获取两点间距离
	template <typename T>
	double getdist(T pointa, T pointb);

	float perimeter(std::vector<cv::Point2f> a);
	void findMarkerCandidates(const std::vector<std::vector<cv::Point>> &contours, std::vector<Marker> &detectedMarkers);
	void detectMarkers(const cv::Mat &grayscale, std::vector<Marker> &detectedMarkers);

	void warpPerspectivePoints(std::vector<cv::Point2f> src, std::vector<cv::Point2f> &dst, cv::Mat M);
	void getintercorners(cv::Mat img, Marker &m, std::vector<cv::Point2f> &intercorners);
	void estiMatePosition(std::vector<Marker> &detectedMarkers, GPSTIME gpst);
	bool markerDetection(cv::Mat img, GPSTIME gpst);
	bool CalVar(Eigen::Matrix3f R_cw, Eigen::Vector3f T_wc, Eigen::Vector3f P_w, std::vector<cv::Point3f> allworldcorners3d, std::vector<cv::Point2f> allcorners);
	bool cv2double(const cv::Mat Rvec, const cv::Mat Tvec);

	void optimize(std::vector<cv::Point2f> allcorners, std::vector<cv::Point3f> allworldcorners3d);
	//int getimgindex();
};

#endif