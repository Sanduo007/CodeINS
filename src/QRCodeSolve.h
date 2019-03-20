/****************************************************************************
Ŀ�ģ�    �����Ӿ�ʶ���ά��������λ����
��дʱ�䣺 2018.08.06
���ߣ�    Yibin Woo
�汾:     V1.0
��Ȩ��    �人��ѧGNSS����
****************************************************************************/
#ifndef _QRCODESOLVE_H_
#define _QRCODESOLVE_H_

#include <eigen3/Eigen/Core>
#include <ceres/ceres.h>
#include <fstream>

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
	Eigen::Vector3d origBLH;

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

	std::ofstream results, errorimg;
	std::string evaluate_file_dir;
	std::ifstream evaluatefile;
	std::ofstream evaluate_outfile, time_info;

  public:
	std::string imgdir_str;
	int imgindex = 0;

	double Q_wc[4] = {0.0, 0.0, 0.0, 0.0};
	double T_wc[3] = {0.0, 0.0, 0.0};

  public:
	//����ͼ��·��������ļ�·��
	QRCodeSolve(std::string config_path, std::string outputdir_);
	~QRCodeSolve();
	void readpara(std::string config_path);
	//��ȡ��������
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

	void distortion(const Eigen::Vector2d &p_u, Eigen::Vector2d &d_u);
	bool m_undistortPoints(std::vector<cv::Point2f> points, std::vector<cv::Point2f> &points_un);
	bool m_caljacobian(Eigen::Matrix3d R_wcEg, Eigen::Vector3d T_wcEg, std::vector<cv::Point3f> allworldcorners3d, Eigen::MatrixXd &m_rjacobianEg, Eigen::MatrixXd &m_tjacobianEg);
	bool evaluatePOSE(std::vector<Marker> &detectedMarkers, double gpst);
	//int getimgindex();
};

#endif