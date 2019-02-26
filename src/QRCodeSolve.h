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
	int markertype;

	cv::Mat camMatrix;
	cv::Mat distCoeff;

	cv::Mat camMatrix2;
	cv::Mat distCoeff2;

	cv::Mat originimg, previmg;
	cv::Mat m_Mask;
	std::vector<cv::Point2f> pre_pts;
	int img_height;
	int img_width;

	int imgthr[9] = {247, 227, 207, 187, 167, 147, 127, 107, 87};

	//
	FILE *results, *errorimg;

  public:
	char imgdir[1000];
	int imgindex = 0;

	double Q_wc[4] = {0.0, 0.0, 0.0, 0.0};
	double T_wc[3] = {0.0, 0.0, 0.0};

  public:
	//输入图像路径和输出文件路径
	QRCodeSolve(std::string config_path, char outputdir_[1000]);
	~QRCodeSolve();
	void readpara(std::string config_path);
	//获取两点间距离
	template <typename T>
	double getdist(T pointa, T pointb);

	float perimeter(std::vector<cv::Point2f> a);
	void findMarkerCandidates(const std::vector<std::vector<cv::Point>> &contours, std::vector<Marker> &detectedMarkers);
	void detectMarkers(const cv::Mat &grayscale, std::vector<Marker> &detectedMarkers);

	void warpPerspectivePoints(std::vector<cv::Point2f> src, std::vector<cv::Point2f> &dst, cv::Mat M);
	void getintercorners(cv::Mat img, Marker &m, std::vector<cv::Point2f> &intercorners);
	bool estiMatePosition(std::vector<Marker> &detectedMarkers, double gpst);
	bool markerDetection(cv::Mat img, double gpst);
	bool CalVar(Eigen::Matrix3d R_cw, Eigen::Vector3d T_wc, Eigen::Vector3d P_w, std::vector<cv::Point3d> allworldcorners3d, std::vector<cv::Point2f> allcorners);
	bool cv2double(const cv::Mat Rvec, const cv::Mat Tvec);

	void optimize(std::vector<cv::Point2f> allcorners, std::vector<cv::Point3f> allworldcorners3d);
	void NED2BLH(Eigen::Vector3d origBLH, Eigen::Vector3d NED, Eigen::Vector3d &calBLH);

	void distortion(const Eigen::Vector2d &p_u, Eigen::Vector2d &d_u);
	bool m_undistortPoints(std::vector<cv::Point2f> points, std::vector<cv::Point2f> &points_un);
	//int getimgindex();
};

#endif