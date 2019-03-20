#include <time.h>
#include <ceres/ceres.h>

#include <omp.h>
#include <opencv2/core/eigen.hpp>

#include "Marker.h"
#include "Const.h"
#include "Struct.h"
#include "QRCodeSolve.h"
#include "CostStruct.h"
#include "Utility.h"
#include "tic_toc.h"

#define EVALUATE

using namespace std;
using namespace cv;

QRCodeSolve::QRCodeSolve(string config_path, string outputdir_)
{
	readpara(config_path);

	char stime[32];

	double blockside2d = 0.0;
	double blockside3d = 0.0;
	Point2f temppoint(0.0, 0.0);
	time_t tt = time(0);
	int shep = strftime(stime, sizeof(stime), "%Y%m%d%H%M", localtime(&tt)); //get current UTC time

	m_markerCorners3d.push_back(Point3f(-markerSizeinWorld.width / 2.0, -markerSizeinWorld.height / 2.0, 0));
	m_markerCorners3d.push_back(Point3f(markerSizeinWorld.width / 2.0, -markerSizeinWorld.height / 2.0, 0)); //�������
	m_markerCorners3d.push_back(Point3f(markerSizeinWorld.width / 2.0, markerSizeinWorld.height / 2.0, 0));
	m_markerCorners3d.push_back(Point3f(-markerSizeinWorld.width / 2.0, markerSizeinWorld.height / 2.0, 0));

	m_markerCorners2d.push_back(Point2f(0, 0));
	m_markerCorners2d.push_back(Point2f(markerSize.width - 1, 0));
	m_markerCorners2d.push_back(Point2f(markerSize.width - 1, markerSize.height - 1)); //˳ʱ��
	m_markerCorners2d.push_back(Point2f(0, markerSize.height - 1));

	blockside2d = (markerSize.width - 1) / 7.0;
	blockside3d = markerSizeinWorld.width / 7.0;

	if (markertype == OLDMARKER)
	{
		m_markerinterCorners2d.push_back(Point2f(blockside2d, blockside2d)); //˳ʱ��
		m_markerinterCorners2d.push_back(Point2f(blockside2d * 6, blockside2d));
		m_markerinterCorners2d.push_back(Point2f(blockside2d * 6, blockside2d * 6));
		m_markerinterCorners2d.push_back(Point2f(blockside2d, blockside2d * 6));
		m_markerinterCorners2d.push_back(Point2f(blockside2d * 3, blockside2d * 2));
		m_markerinterCorners2d.push_back(Point2f(blockside2d * 4, blockside2d * 2));
		m_markerinterCorners2d.push_back(Point2f(blockside2d * 4, blockside2d * 3));
		m_markerinterCorners2d.push_back(Point2f(blockside2d * 5, blockside2d * 3));
		m_markerinterCorners2d.push_back(Point2f(blockside2d * 5, blockside2d * 4));
		m_markerinterCorners2d.push_back(Point2f(blockside2d * 4, blockside2d * 4));
		m_markerinterCorners2d.push_back(Point2f(blockside2d * 3, blockside2d * 4));
		m_markerinterCorners2d.push_back(Point2f(blockside2d * 2, blockside2d * 4));
		m_markerinterCorners2d.push_back(Point2f(blockside2d * 2, blockside2d * 3));
		m_markerinterCorners2d.push_back(Point2f(blockside2d * 3, blockside2d * 3));
		m_markerinterCorners2d.push_back(Point2f(blockside2d * 3, blockside2d * 5));
		m_markerinterCorners2d.push_back(Point2f(blockside2d * 4, blockside2d * 5));
		m_markerinterCorners2d.push_back(Point2f(blockside2d * 4, blockside2d * 6));
		m_markerinterCorners2d.push_back(Point2f(blockside2d * 3, blockside2d * 6));

		for (int i = 0; i < m_markerinterCorners2d.size(); i++)
		{
			temppoint.x = (m_markerinterCorners2d[i].x - 3.5 * blockside2d) * blockside3d / blockside2d;
			temppoint.y = (m_markerinterCorners2d[i].y - 3.5 * blockside2d) * blockside3d / blockside2d;
			m_markerinterCorners3d.push_back(Point3f(temppoint.x, temppoint.y, 0.0));
		}
	}
	else if (markertype == NEWMARKER)
	{
		m_markerinterCorners2d.push_back(Point2f(blockside2d, blockside2d)); //˳ʱ��
		m_markerinterCorners2d.push_back(Point2f(blockside2d * 6, blockside2d));
		m_markerinterCorners2d.push_back(Point2f(blockside2d * 6, blockside2d * 6));
		m_markerinterCorners2d.push_back(Point2f(blockside2d, blockside2d * 6));

		m_markerinterCorners2d.push_back(Point2f(blockside2d * 3, blockside2d * 3));
		m_markerinterCorners2d.push_back(Point2f(blockside2d * 4, blockside2d * 3));
		m_markerinterCorners2d.push_back(Point2f(blockside2d * 4, blockside2d * 4));
		m_markerinterCorners2d.push_back(Point2f(blockside2d * 3, blockside2d * 4));

		m_markerinterCorners2d.push_back(Point2f(blockside2d * 4, blockside2d * 5));
		m_markerinterCorners2d.push_back(Point2f(blockside2d * 5, blockside2d * 5));
		m_markerinterCorners2d.push_back(Point2f(blockside2d * 5, blockside2d * 6));
		m_markerinterCorners2d.push_back(Point2f(blockside2d * 4, blockside2d * 6));

		m_markerinterCorners2d.push_back(Point2f(blockside2d * 2, blockside2d * 5));
		m_markerinterCorners2d.push_back(Point2f(blockside2d * 3, blockside2d * 5));
		m_markerinterCorners2d.push_back(Point2f(blockside2d * 3, blockside2d * 6));
		m_markerinterCorners2d.push_back(Point2f(blockside2d * 2, blockside2d * 6));

		for (int i = 0; i < m_markerinterCorners2d.size(); i++)
		{
			temppoint.x = (m_markerinterCorners2d[i].x - 3.5 * blockside2d) * blockside3d / blockside2d;
			temppoint.y = (m_markerinterCorners2d[i].y - 3.5 * blockside2d) * blockside3d / blockside2d;
			m_markerinterCorners3d.push_back(Point3f(temppoint.x, temppoint.y, 0.0));
		}
	}
#ifdef EVALUATE
	evaluatefile.open(evaluate_file_dir);
	evaluate_outfile.open(imgdir_str + "evaluate_outfile.txt");
	if (!evaluatefile.is_open())
	{
		std::cout << "Open Evaluate File ERROR!" << std::endl;
	}
#else
	results.open(imgdir_str + "POSEResultsRyaw_noransac" + string(stime) + ".txt"); // = fopen(string(imgdir_str) + string(filename), "w+");
	errorimg.open(outputdir_ + "errorimg" + string(stime) + ".txt");				//= fopen(string(outputdir_) + string(errorimgname), "w+");
	time_info.open(imgdir_str + "TimeInfo.txt");
	if (!results.is_open() || !errorimg.is_open() || !time_info.is_open())
	{
		std::cout << "create file failed!" << std::endl;
	}
#endif

	// if (cameratype == GUIDANCE)
	// {
	// 	//old_version
	// 	// camMatrix = (Mat_<double>(3, 3) << 240.2721, 1.5735, 163.4041, 0.000, 238.7327, 118.7457, 0.000, 0.000, 1.000); //����ڲ�
	// 	// distCoeff = (Mat_<double>(5, 1) << -0.0123, 0.2247, -0.0090, 0.0018, -0.4472);
	// 	//1206
	// 	camMatrix = (Mat_<double>(3, 3) << 233.6475, 0.000, 164.4622, 0.000, 235.9954, 120.2638, 0.000, 0.000, 1.000); //����ڲ�
	// 	distCoeff = (Mat_<double>(5, 1) << 0.0342, -0.0193, -0.00133, 4.5599e-04, 0.007);
	// 	img_height = 240;
	// 	img_width = 320;
	// }
	// else if (cameratype == X3)
	// {
	// 	camMatrix = (Mat_<double>(3, 3) << 752.6946, 0.7260, 640.3954, 0, 738.9115, 356.1570, 0.000, 0.000, 1.000);
	// 	distCoeff = (Mat_<double>(5, 1) << -0.1362, 0.1201, -0.0036, -0.0018, -0.0282);

	// 	img_height = 360;
	// 	img_width = 640;
	// }
	// else if (cameratype == MYNTEYE)
	// {
	// 	camMatrix = (Mat_<double>(3, 3) << 391.0981, 0.000, 374.2491, 0.000, 390.5271, 238.1774, 0.000, 0.000, 1.000); //calib by my self
	// 	distCoeff = (Mat_<double>(5, 1) << -0.3010, 0.0857, -0.0013, -5.0467e-04, -0.0107);

	// 	//distCoeff = (Mat_<double>(5, 1) << -0.3010, 0.0857, -7.12e-04, -4.18e-04, -0.0107);

	// 	camMatrix2 = (Mat_<double>(3, 3) << 369.07822115562487397, 0.0, 392.09630381779828667, 0.0, 371.96824907254500658, 220.98562290666671970, 0.0, 0.0, 1.0);
	// 	distCoeff2 = (Mat_<double>(5, 1) << -0.25532845038098667, 0.04714091848806595, 0.00190513557523387, -0.00088014317456589, 0.0); //get from mynt SDK

	// 	img_height = 480;
	// 	img_width = 752;
	// }
}
void QRCodeSolve::readpara(string config_path)
{

	double B, L, H;
	cv::FileStorage fsSettings(config_path, cv::FileStorage::READ);
	if (!fsSettings.isOpened())
	{
		std::cerr << "ERROR: Wrong path to settings" << std::endl;
		return;
	}

	img_height = fsSettings["img_height"];
	img_width = fsSettings["img_width"];

	markertype = fsSettings["MARKER_TYPE"];

	fsSettings["camMatrix"] >> camMatrix;
	fsSettings["distCoeff"] >> distCoeff;
	cout << camMatrix << endl;
	cout << distCoeff << endl;
	markerSize.height = fsSettings["MARKER_SIZE"];
	markerSize.width = fsSettings["MARKER_SIZE"];

	markerSizeinWorld.height = fsSettings["MARKER_SIZE_WORLD"];
	markerSizeinWorld.width = fsSettings["MARKER_SIZE_WORLD"];
	B = fsSettings["MARKER_POS_B"];
	L = fsSettings["MARKER_POS_L"];
	H = fsSettings["MARKER_POS_H"];
	origBLH = Eigen::Vector3d(B, L, H);

	fsSettings["IMAGE_DIR"] >> imgdir_str;
	// strcpy(imgdir, imgdir_str.c_str());

	fsSettings["EVALUATE_FILE_DIR"] >> evaluate_file_dir;
}

QRCodeSolve::~QRCodeSolve()
{
	results.close();
	errorimg.close();
	evaluatefile.close();

	originimg.release();
	previmg.release();
	m_Mask.release();
}

float QRCodeSolve::perimeter(vector<Point2f> a)
{
	float sum = 0, dx, dy;

	for (size_t i = 0; i < a.size(); ++i)
	{
		size_t i2 = (i + 1) % a.size(); //ȡģ��ǿ���պϵĻ�·

		dx = a[i].x - a[i2].x;
		dy = a[i].y - a[i2].y;

		sum += sqrt(dx * dx + dy * dy);
	}

	return sum;
}
void QRCodeSolve::findMarkerCandidates(const vector<vector<Point>> &contours, vector<Marker> &detectedMarkers)
{
	float m_minContourLengthAllowed = 30.0;

	vector<Point2i> approxCurve;
	vector<Marker> possibleMarkers;
	Mat contour_display = Mat(img_height, img_width, CV_8UC1);
	contour_display = originimg.clone();

	Point vec;
	float squaredDistance = 0.0;

	for (size_t i = 0; i < contours.size(); ++i)
	{
		approxPolyDP(contours[i], approxCurve, double(contours[i].size()) * 0.15, true); //����.approxCurve����Ϻ�Ķ�����������㼯

		if (approxCurve.size() != 4)
			continue;

		if (!isContourConvex(approxCurve)) // �ж��ǲ���͹�����
			continue;

		float minDist = 1e10; //ÿ����ϵ�������Ӧ�����ֵ�Ƚ�!(���)
		for (int i = 0; i < 4; ++i)
		{
			vec = approxCurve[i] - approxCurve[(i + 1) % 4];
			squaredDistance = vec.dot(vec);
			minDist = min(minDist, squaredDistance); //ȡ������Сֵ
		}

		if (minDist < m_minContourLengthAllowed)
			continue;

		Marker m;
		for (int i = 0; i < 4; ++i)
		{
			m.points.push_back(Point2f(approxCurve[i].x, approxCurve[i].y));
		}
		//imshow("contour", contour_display);
		//waitKey(1);
		Point2f v1 = m.points[1] - m.points[0];
		Point2f v2 = m.points[2] - m.points[0];
		double o = (v1.x * v2.y) - (v1.y * v2.x); //���Σ�������ж�,������ұ�Ӧ�ý����ڶ����͵��ĸ����ﵽЧ��
		if (o < 0.0)
		{
			swap(m.points[1], m.points[3]); //swap ����
		}

		possibleMarkers.push_back(m); //ֻ���¿��ܵ������������Ķ�������ϳ����ģ�������
	}
	detectedMarkers.clear();
	for (size_t i = 0; i < possibleMarkers.size(); ++i)
	{
		detectedMarkers.push_back(possibleMarkers[i]);
	}
}

void QRCodeSolve::detectMarkers(const Mat &grayscale, vector<Marker> &detectedMarkers)
{
	Mat canonicalMarker;
	vector<Marker> goodMarkers;

	for (size_t i = 0; i < detectedMarkers.size(); ++i)
	{
		Marker marker = detectedMarkers[i];
		Mat M = getPerspectiveTransform(marker.points, m_markerCorners2d); //�任��ϵ����
		warpPerspective(grayscale, canonicalMarker, M, markerSize);		   //�����ı任 ��canonicalmarker��ͼ���任֮��canonical�䷶��size��markerSizeָ����

		int nRotations = 5;

		int id = 0;
		id = marker.getMarkerId(canonicalMarker, nRotations); //!!!�������������ȥ����Χ7*7�Ĳ������ʴ������������־��������idΪ-1

		if (id == -1) //�ж��Ƿ����Ԥ����ά����Ϣ
			continue;
		while (nRotations != 0) //������ʹ�ü�⵽��marker������ʵ��λ�ö�Ӧ������˳������PNP����
		{
			rotate(marker.points.begin(), marker.points.begin() + 1, marker.points.end());
			M = getPerspectiveTransform(marker.points, m_markerCorners2d);
			warpPerspective(grayscale, canonicalMarker, M, markerSize);
			id = marker.getMarkerId(canonicalMarker, nRotations);
			//imshow(to_string(0) + to_string(i), canonicalMarker);
			//waitKey(1);
		}
		marker.id = id;
		goodMarkers.push_back(marker);
		//destroyWindow(to_string(i));
		break;
	}

	if (goodMarkers.size() > 0)
	{
		vector<Point2f> preciseCorners(4 * goodMarkers.size());
		for (size_t i = 0; i < goodMarkers.size(); ++i)
		{
			Marker marker = goodMarkers[i];
			for (int c = 0; c < 4; ++c)
			{
				preciseCorners[i * 4 + c] = marker.points[c];
			}
		}

		cornerSubPix(grayscale, preciseCorners, cvSize(5, 5), cvSize(-1, -1), cvTermCriteria(CV_TERMCRIT_ITER, 30, 0.1));

		for (size_t i = 0; i < goodMarkers.size(); ++i) //��ϸ��λ�ø��Ƹ���ǽǵ�
		{
			for (int c = 0; c < 4; ++c)
			{
				goodMarkers[i].points[c] = preciseCorners[i * 4 + c];
			}
		}
	}
	detectedMarkers = goodMarkers;
}

bool QRCodeSolve::markerDetection(Mat img, double gpst)
{
	// Mat map1, map2, img2, img3, img4;
	// Size imgsize = Size(img_width, img_height);
	// initUndistortRectifyMap(camMatrix, distCoeff, Mat(), Mat(), imgsize, CV_16SC2, map1, map2);
	// remap(img, img2, map1, map2, INTER_LINEAR);
	// cv::imshow("undistort img2", img2);
	// cv::waitKey(1);

	// initUndistortRectifyMap(camMatrix2, distCoeff2, Mat(), Mat(), imgsize, CV_16SC2, map1, map2);
	// remap(img, img3, map1, map2, INTER_LINEAR);
	// cv::imshow("undistort img3", img3);
	// cv::waitKey(1);
	// undistort(img, img4, camMatrix, distCoeff);
	// cv::imshow("undistort img4", img4);
	// cv::waitKey(1);

	// Mat edge, edgeshow;
	// Canny(img, edge, 3, 9);
	// imshow("edge", edge);
	// waitKey(1);

	Mat imgdebug;
	img.copyTo(imgdebug);
	std::vector<cv::Point2f> cur_pts;
	std::vector<uchar> status;
	std::vector<float> err;
	if (!previmg.empty() && (pre_pts.size() > 0))
	{
		//cv::imshow("previous", previmg);
		//cv::waitKey(1);
		cv::calcOpticalFlowPyrLK(previmg, img, pre_pts, cur_pts, status, err, cv::Size(21, 21), 3);
		for (int i = 0; i < pre_pts.size(); i++)
		{
			if (status[i])
				circle(imgdebug, cur_pts[i], 3, Scalar(0, 0, 255), -1, 0);
		}
		//cv::imshow("LK", imgdebug);
		//waitKey(1);
	}
	Mat imgByMory;
	Mat imgByAdptThr;
	vector<Marker> detectedMarkers;
	vector<vector<Point>> contours;

	img.copyTo(originimg);
	TicToc findcode;
	findcode.tic();
	for (int k = 0; k < 9; k++) //�ݶ�ȫ����ֵ
	{
		cv::threshold(img, imgByAdptThr, imgthr[k], 255, THRESH_BINARY);
		morphologyEx(imgByAdptThr, imgByAdptThr, MORPH_OPEN, Mat());
		morphologyEx(imgByAdptThr, imgByAdptThr, MORPH_CLOSE, Mat());
		//imshow("morphologyEx", imgByAdptThr);
		//waitKey(1);

		imgByMory = imgByAdptThr.clone();

		vector<vector<Point2i>> allContours;
		vector<Vec4i> hierarchy;

		findContours(imgByAdptThr, allContours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE); //���������ֻ��Ϊ����

		contours.clear();
		for (size_t i = 0; i < allContours.size(); ++i)
		{
			if (allContours[i].size() >= 4)
			{
				contours.push_back(allContours[i]); //�˳�С������,���������С
			}
		}

		if (contours.size())
		{
			findMarkerCandidates(contours, detectedMarkers);
		}
		else
			continue;

		if (detectedMarkers.size())
		{

			detectMarkers(imgByMory, detectedMarkers); //��ֵ������̬ѧ����֮������Ϣ
			if (detectedMarkers.size())
			{
				time_info << findcode.toc();
#ifdef EVALUATE
				evaluatePOSE(detectedMarkers, gpst);
#else
				if (!estiMatePosition(detectedMarkers, gpst))
					continue;
#endif

				if (k != 0)
					rotate(imgthr, imgthr + k, imgthr + k + 1);
				return true;
			}
		}
		else
			continue;
	}
	//printf("%d\n", imgindex);
	//fprintf(errorimg, "%d\n", imgindex);
	LOG(INFO) << "errimg " << imgindex;
	// fprintf(results, "%f %f %f %f %f %f %f %f %f %f %f %f %f\n",
	// 		gpst, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	return false;
}

template <typename T>
double QRCodeSolve::getdist(T pointa, T pointb)
{
	return (double)sqrt((pointa.x - pointb.x) * (pointa.x - pointb.x) + (pointa.y - pointb.y) * (pointa.y - pointb.y));
}
/*
����͸��ͶӰ�任����M��������Ķ�ά���ڲ��ǵ�ͶӰ��ͼ����
*/
void QRCodeSolve::warpPerspectivePoints(std::vector<cv::Point2f> src, std::vector<cv::Point2f> &dst, cv::Mat M)
{
	double M11, M12, M13, M21, M22, M23, M31, M32, M33;
	Point2f temp;
	M11 = M.at<double>(0, 0);
	M12 = M.at<double>(0, 1);
	M13 = M.at<double>(0, 2);
	M21 = M.at<double>(1, 0);
	M22 = M.at<double>(1, 1);
	M23 = M.at<double>(1, 2);
	M31 = M.at<double>(2, 0);
	M32 = M.at<double>(2, 1);
	M33 = M.at<double>(2, 2);

	for (int i = 0; i < src.size(); i++)
	{
		temp.x = (M11 * src[i].x + M12 * src[i].y + M13) / (M31 * src[i].x + M32 * src[i].y + M33);
		temp.y = (M21 * src[i].x + M22 * src[i].y + M23) / (M31 * src[i].x + M32 * src[i].y + M33);
		dst.push_back(temp);
	}
}

/*
	��ȡͼ���ж�ά���ڲ��Ľǵ㣬���������趨marker�ǵ����Ĵ�С�����������ͬʱ�޸���Χ�ĵ�
*/
void QRCodeSolve::getintercorners(Mat img, Marker &m, vector<Point2f> &intercorners)
{

	vector<Point2f> corners, intercorners_seq, intercorners_seqtmp, m_markerinterCorners3d4caltmp;
	vector<Point2f> perspintercorners;
	vector<Point2f> outercorners;
	vector<int> distflag;
	Point2f temppoint;

	Mat imgdebug;
	cvtColor(img, imgdebug, CV_GRAY2BGR);

	// Mat kernel = (Mat_<double>(3, 3) << 0, -1, 0, -1, 5, -1, 0, -1, 0);
	// filter2D(img, imageEnhance, -1, kernel);
	// imshow("laplace", imageEnhance);
	// waitKey(1);

	double ppdist = 0.0;		 //�ǵ㵽��������
	double cornermatch_th = 8.0; //���ǵ���ģ��ͶӰ�ǵ�ƥ����������ֵ
	double tempdist = 0.0;
	int outnum = 0;

	//m_markerinterCorners3d4cal = m_markerinterCorners3d;
	Mat M = getPerspectiveTransform(m_markerCorners2d, m.points);
	warpPerspectivePoints(m_markerinterCorners2d, perspintercorners, M);
	m_Mask = cv::Mat(img_height, img_width, CV_8UC1, cv::Scalar(0));
#pragma omp parallel
	{
		for (int i = 0; i < m_Mask.rows; i++)
		{
			for (int j = 0; j < m_Mask.cols; j++)
			{
				if (pointPolygonTest(m.points, Point2f(j, i), false) == 1)
				{
					m_Mask.at<uchar>(i, j) = 255;
				}
			}
		}
	}
	//imshow("mask", m_Mask);
	//waitKey(1);
	goodFeaturesToTrack(img, corners, 22, 0.01, 5, m_Mask); //��ȡ����ͼ���е�SHi-Tomasi�ǵ�,Ӧ�����˽⡣����Harris

	for (int j = 0; j < corners.size(); j++)
	{
		ppdist = pointPolygonTest(m.points, corners[j], true);

		if (abs(ppdist) <= 5) //���ܻ�������㵱���ڵ�
		{
			circle(imgdebug, corners[j], 3, Scalar(0, 0, 255), -1, 0);
			outercorners.push_back(corners[j]);
			/*circle(imgdebug, corners[j], 3, Scalar(0, 0, 0), -1, 0);
			imshow("allcornersimg", imgdebug);
			waitKey(1);*/
		}
		else if (ppdist > 0) //��������ڵĽǵ�
		{
			circle(imgdebug, corners[j], 3, Scalar(0, 255, 0), -1, 0);
			intercorners.push_back(corners[j]);
		}
	}

	for (int k = 0; k < perspintercorners.size(); k++) //���ݶ�ά��ͶӰ������ƽ���ϵ��ڵ�
	{
		circle(imgdebug, perspintercorners[k], 1.5, Scalar(255, 0, 0), -1, 0);
		double mindist = 20;
		for (int n = 0; n < intercorners.size(); n++)
		{
			tempdist = getdist(perspintercorners[k], intercorners[n]);
			if (tempdist < mindist)
			{
				mindist = tempdist;
				temppoint = intercorners[n];
			}
		}
		//m_markerinterCorners3d4caltmp.push_back(m_markerinterCorners3d[k]);

		distflag.push_back(1);

		intercorners_seqtmp.push_back(temppoint);
		if (mindist < cornermatch_th) //��ͶӰ�ǵ������ͼ��ǵ�Ӧ����ֵ��
		{
			m_markerinterCorners3d4cal.push_back(m_markerinterCorners3d[k]); //ֻ�С��ж�Ӧͼ��ǵ㡱��3D��Ų������
			intercorners_seq.push_back(temppoint);							 //�Լ������ڲ��ǵ�����
		}
	}
	// int index = 0;//forward and back match
	// for (int i = 0; i < intercorners_seqtmp.size(); i++)
	// {
	// 	double mindist = 20;
	// 	for (int j = 0; j < perspintercorners.size(); j++)
	// 	{
	// 		tempdist = getdist(perspintercorners[j], intercorners_seqtmp[i]);
	// 		if (tempdist < mindist)
	// 		{
	// 			mindist = tempdist;
	// 			index = j;
	// 		}
	// 	}
	// 	if (i != index)
	// 		distflag[i] = 0;
	// }
	// for (int i = 0; i < distflag.size(); i++)
	// {
	// 	if (distflag[i])
	// 	{
	// 		m_markerinterCorners3d4cal.push_back(m_markerinterCorners3d[i]);
	// 		intercorners_seq.push_back(intercorners_seqtmp[i]);
	// 	}
	// }

	//show img for debug to get inner points
	// imshow("allcornersimg", imgdebug);
	// waitKey(1);

	if (m.points.size() <= outercorners.size())
	{
		for (int q = 0; q < m.points.size(); q++)
		{
			double mindist = 20;
			for (int n = 0; n < outercorners.size(); n++)
			{
				tempdist = getdist(m.points[q], outercorners[n]);
				if (tempdist < mindist)
				{
					mindist = tempdist;
					temppoint = outercorners[n];
				}
			}
			if (mindist < 5)
			{
				vector<Point2f> vectortmp;
				vectortmp.push_back(temppoint);
				//cornerSubPix(img, vectortmp, Size(5, 5), Size(-1, -1), cvTermCriteria(CV_TERMCRIT_ITER, 30, 0.1)); //��С���˷�ϸ���ڲ��ǵ㣬�����ؼ�

				m.points[q].x = (vectortmp[0].x + m.points[q].x) / 2.0; //ϸ�������ⲿ�ĸ��ǵ�
				m.points[q].y = (vectortmp[0].y + m.points[q].y) / 2.0;
			}
		}
	}

	cornerSubPix(img, intercorners_seq, Size(5, 5), Size(-1, -1), cvTermCriteria(CV_TERMCRIT_ITER, 30, 0.1)); //��С���˷�ϸ���ڲ��ǵ㣬�����ؼ�

	/*for (int m = 0; m < intercorners_seq.size(); m++)
	{
		circle(imgdebug, intercorners_seq[m], 4, Scalar(255, 0, 0), -1, 0);
		imshow("allcornersimg", imgdebug);
		waitKey(1);
	}*/
	intercorners = intercorners_seq;
}

void QRCodeSolve::optimize(vector<Point2f> allcorners, vector<Point3f> allworldcorners3d)
{
	ceres::Problem poseproblem;
	ceres::LocalParameterization *local_parameterization = new ceres::QuaternionParameterization();

	ceres::LossFunction *loss_function;
	loss_function = new ceres::CauchyLoss(1.0); //��ʧ���������ٴ�Ĳв��Ӱ�죬�������Ӿ��۲�

	poseproblem.AddParameterBlock(Q_wc, 4, local_parameterization);
	poseproblem.AddParameterBlock(T_wc, 3);

	for (int i = 0; i < allworldcorners3d.size(); i++)
	{
		Eigen::Vector3d pw(allworldcorners3d[i].x, allworldcorners3d[i].y, allworldcorners3d[i].z);
		double ux = (double)allcorners[i].x;
		double vy = (double)allcorners[i].y;
		ceres::CostFunction *cost_function = REPROJECT_COST::Create(ux, vy, pw);
		poseproblem.AddResidualBlock(cost_function, NULL, Q_wc, T_wc);
	}
	ceres::Solver::Options poseoptions;
	poseoptions.linear_solver_type = ceres::DENSE_SCHUR;
	poseoptions.minimizer_progress_to_stdout = false; // �����cout
	poseoptions.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;

	ceres::Solver::Summary posesummary;
	ceres::Solve(poseoptions, &poseproblem, &posesummary);

	cout << posesummary.BriefReport() << endl;
	LOG(INFO) << posesummary.BriefReport();
}
//rotation vector to quaternion,ref: Niu
bool Rvec2Q(double Rvec[], double Q[])
{
	double rvecnorm = 0.0;
	double n = 0.0;
	double nRvec[3] = {0.0};
	rvecnorm = sqrt(Rvec[0] * Rvec[0] + Rvec[1] * Rvec[1] + Rvec[2] * Rvec[2]);
	n = sin(0.5 * rvecnorm) / rvecnorm;

	Q[0] = cos(0.5 * rvecnorm);
	Q[1] = n * Rvec[0];
	Q[2] = n * Rvec[1];
	Q[3] = n * Rvec[2];
	return true;
}
// quaternion to rotation vector,ref: Shin 14
bool Q2Rvec(double Q[], double Rvec[])
{
	double f = 0;
	double halfPhiNorm = sqrt(Q[1] * Q[1] + Q[2] * Q[2] + Q[3] * Q[3]) / Q[0];
	f = sin(halfPhiNorm) / (2 * halfPhiNorm);
	Rvec[0] = Q[1] / f;
	Rvec[1] = Q[2] / f;
	Rvec[2] = Q[3] / f;
}
bool QRCodeSolve::cv2double(const Mat Rvec, const Mat Tvec)
{
	assert(Rvec.rows == 3 && Rvec.cols == 1 && Tvec.rows == 3 && Tvec.cols == 1);
	double R[3] = {0.0};
	//cout << "Rvec " << Rvec << endl;
	//cout << "Tvec " << Tvec << endl;

	Eigen::Vector3f RvecEg, TvecEg;
	cv2eigen(Rvec, RvecEg);
	cv2eigen(Tvec, TvecEg);

	R[0] = RvecEg(0);
	R[1] = RvecEg(1);
	R[2] = RvecEg(2);

	T_wc[0] = TvecEg(0);
	T_wc[1] = TvecEg(1);
	T_wc[2] = TvecEg(2);
	Rvec2Q(R, Q_wc);
	double Rcheck[3] = {0.0};
	Q2Rvec(Q_wc, Rcheck);
	return true;
}
void QRCodeSolve::distortion(const Eigen::Vector2d &p_u, Eigen::Vector2d &d_u)
{
	double k1 = distCoeff.at<double>(0, 0);
	double k2 = distCoeff.at<double>(1, 0);
	double p1 = distCoeff.at<double>(2, 0);
	double p2 = distCoeff.at<double>(3, 0);
	double k3 = distCoeff.at<double>(4, 0);

	double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;

	mx2_u = p_u(0) * p_u(0);
	my2_u = p_u(1) * p_u(1);
	mxy_u = p_u(0) * p_u(1);
	rho2_u = mx2_u + my2_u;
	rad_dist_u = k1 * rho2_u + k2 * rho2_u * rho2_u + k3 * rho2_u * rho2_u * rho2_u;
	d_u << p_u(0) * rad_dist_u + 2.0 * p1 * mxy_u + p2 * (rho2_u + 2.0 * mx2_u),
		p_u(1) * rad_dist_u + 2.0 * p2 * mxy_u + p1 * (rho2_u + 2.0 * my2_u); //ref:Gaoxiang P89,VINS
}

bool QRCodeSolve::m_undistortPoints(vector<Point2f> points, vector<Point2f> &points_un)
{
	double mx_d, my_d, mx_u, my_u;
	double fx = camMatrix.at<double>(0, 0); //double or uchar depends on the type of Matrix!
	double fy = camMatrix.at<double>(1, 1);
	double cx = camMatrix.at<double>(0, 2);
	double cy = camMatrix.at<double>(1, 2);
	int n = 8;
	Eigen::Vector2d d_u;
	for (int i = 0; i < points.size(); i++)
	{
		mx_d = (points[i].x - cx) / fx;
		my_d = (points[i].y - cy) / fy;

		distortion(Eigen::Vector2d(mx_d, my_d), d_u); //�������һ����������ʱ��������ת˲���ŵġ�const ���ÿ��Գ�ʼ��Ϊ��ͬ���͵Ķ�����߳�ʼ��Ϊ��ֵ
		// Approximate value
		mx_u = mx_d - d_u(0);
		my_u = my_d - d_u(1);

		for (int i = 1; i < n; ++i)
		{
			distortion(Eigen::Vector2d(mx_u, my_u), d_u);
			mx_u = mx_d - d_u(0);
			my_u = my_d - d_u(1);
		}

		points_un.push_back(cv::Point2f(fx * mx_u + cx, fy * my_u + cy));
	}
	return true;
}
bool QRCodeSolve::m_caljacobian(Eigen::Matrix3d R_wcEg, Eigen::Vector3d T_wcEg, vector<Point3f> allworldcorners3d, Eigen::MatrixXd &m_rjacobianEg, Eigen::MatrixXd &m_tjacobianEg)
{
	Eigen::Vector3d Pc, Pw, tmp;
	Eigen::Matrix3d camMatEg;
	cv2eigen(camMatrix, camMatEg);
	for (int i = 0; i < allworldcorners3d.size(); i++)
	{
		//cv2eigen(allworldcorners3d[i], Pw);
		Pw << allworldcorners3d[i].x, allworldcorners3d[i].y, allworldcorners3d[i].z;
		Pc = R_wcEg * Pw + T_wcEg;

		m_tjacobianEg.block(2 * i, 0, 2, 3) = (1.0 / (Pc.tail<1>())(0) * camMatEg).block<2, 3>(0, 0);
		m_rjacobianEg.block(2 * i, 0, 2, 3) = ((1.0 / (Pc.tail<1>())(0) * camMatEg).topRows<2>() * Utility::skewSymmetric(R_wcEg * Pw)).block<2, 3>(0, 0);
	}
	return true;
}
bool QRCodeSolve::estiMatePosition(std::vector<Marker> &detectedMarkers, double gpst)
{
	vector<Point2f> intercorners, allcorners, allcorners_undis, m_allcorners_undis,
		allcorners_norm, allcorners_repro, optm_allcorners_repro;
	vector<Point3f> allworldcorners3d;
	Marker m = detectedMarkers[0];
	Mat Rvec, Tvec, raux, taux;
	Mat jacobian, rjacobian, tjacobian;
	Eigen::MatrixXd rjacobianEg, tjacobianEg;
	double reproject_err = 0.0;
	double optm_reproject_err = 0.0;
	Eigen::Vector3d Euler;
	Eigen::Matrix3d R_wcEg, R_cw, R_bw, R_wn, R_bc; //rotate matrix from world to camera !ŷ���ǵĶ���
	Eigen::Vector3d P_w, T_wcEg, P_n;				//the Marker's center in UAVbody coor ,which is parelled with camera coor.transformation from world to camera,Eigen
	Mat_<float> rotMat(3, 3);
	Mat rgbimg;

	double fx = camMatrix.at<double>(0, 0); //double or uchar depends on the type of Matrix!
	double fy = camMatrix.at<double>(1, 1);
	double cx = camMatrix.at<double>(0, 2);
	double cy = camMatrix.at<double>(1, 2);

	TicToc getcorners;
	getcorners.tic();
	getintercorners(originimg, m, intercorners);
	time_info << " " << getcorners.toc();

	//allcorners.insert(allcorners.end(), m.points.begin(), m.points.end()); //matched points pair for calculating
	allcorners.insert(allcorners.end(), intercorners.begin(), intercorners.end());
	//allworldcorners3d.insert(allworldcorners3d.end(), m_markerCorners3d.begin(), m_markerCorners3d.end());
	allworldcorners3d.insert(allworldcorners3d.end(), m_markerinterCorners3d4cal.begin(), m_markerinterCorners3d4cal.end());

	m_undistortPoints(allcorners, m_allcorners_undis);
	undistortPoints(allcorners, allcorners_undis, camMatrix, distCoeff, cv::noArray(), camMatrix); //���÷�����ע��.����֮ǰ��ȥ����

	cvtColor(originimg, rgbimg, cv::COLOR_GRAY2BGR);
	vector<Point2f> deltapoint;
	for (int i = 0; i < allcorners.size(); i++)
	{
		allcorners_norm.push_back(cv::Point2f((allcorners_undis[i].x - cx) / fx, (allcorners_undis[i].y - cy) / fy)); //��һ���ǵ�
		circle(rgbimg, allcorners[i], 1.5, Scalar(0, 255, 0), -1, 0);
		deltapoint.push_back(allcorners_undis[i] - m_allcorners_undis[i]);
	}

	vector<int> inliers;
	solvePnP(allworldcorners3d, allcorners_undis, camMatrix, cv::noArray(), raux, taux, false, SOLVEPNP_ITERATIVE);
	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%solvePnPRansac (option)
	// {
	// 	TicToc solve;
	// 	solve.tic();
	// 	solvePnPRansac(allworldcorners3d, allcorners_undis, camMatrix, cv::noArray(), raux, taux, false, 100, 1.0, 0.99, inliers, SOLVEPNP_ITERATIVE);
	// 	time_info << " " << solve.toc() << endl;
	// 	if (inliers.size() < 4)
	// 	{
	// 		m_markerinterCorners3d4cal.clear();
	// 		return false;
	// 	}

	// 	for (int i = 0; i < inliers.size(); i++) //use inlier points to recize all points vectors
	// 	{
	// 		allcorners_undis[i] = allcorners_undis[inliers[i]];   //ȥ����Ľǵ�
	// 		allworldcorners3d[i] = allworldcorners3d[inliers[i]]; //ԭʼ�Ľǵ��3d����
	// 		allcorners_norm[i] = allcorners_norm[inliers[i]];	 //ȥ������һ���Ľǵ�
	// 		circle(rgbimg, allcorners[inliers[i]], 1.5, Scalar(0, 255, 0), -1, 0);
	// 	}
	// 	allcorners_undis.resize(inliers.size());
	// 	allworldcorners3d.resize(inliers.size());
	// 	allcorners_norm.resize(inliers.size());
	// 	LOG_IF(INFO, inliers.size() < 10) << "inliers " << inliers.size();
	// }
	m_markerinterCorners3d4cal.clear(); //must,wyb
	//Eigen::MatrixXd covariance = Eigen::MatrixXd::Identity(allworldcorners3d.size() * 2, allworldcorners3d.size() * 2); //RANSAC֮��Ľǵ���ȡ���Ϊһ������
	Eigen::MatrixXd weight = Eigen::MatrixXd::Identity(allworldcorners3d.size() * 2, allworldcorners3d.size() * 2);

	imshow("QRALLCorner", rgbimg);
	waitKey(1);
	imwrite(imgdir_str + "allcorner_noRANSAC/" + to_string(gpst) + ".png", rgbimg);
	projectPoints(allworldcorners3d, raux, taux, camMatrix, cv::noArray(), allcorners_repro, jacobian); //df/dr,df/dt,......(imagepoints)
	Eigen::VectorXd reproj_d(allworldcorners3d.size() * 2);
	for (int j = 0; j < allcorners_undis.size(); j++) //�����л���ĵ�,�Ż�֮ǰ����ͶӰ���
	{
		reproject_err += (sqrt((allcorners_undis[j].x - allcorners_repro[j].x) * (allcorners_undis[j].x - allcorners_repro[j].x) / fx / fx + (allcorners_undis[j].y - allcorners_repro[j].y) * (allcorners_undis[j].y - allcorners_repro[j].y) / fy / fy) / inliers.size());
		reproj_d.segment<2>(2 * j) = Eigen::Vector2d(allcorners_undis[j].x - allcorners_repro[j].x, allcorners_undis[j].y - allcorners_repro[j].y);
		//covariance.block<2, 2>(2 * (j - 1), 2 * (j - 1)) = reproj_d.asDiagonal();
	}

	raux.convertTo(Rvec, CV_32F); //��ת����
	taux.convertTo(Tvec, CV_32F); //ƽ������
	cv2double(Rvec, Tvec);
	cout << "Q_wc: " << Q_wc[0] << " " << Q_wc[1] << " " << Q_wc[2] << " " << Q_wc[3] << endl;
	cout << "T_wc: " << T_wc[0] << " " << T_wc[1] << " " << T_wc[2] << endl;

	/*//ceres�Ż� option
	optimize(allcorners_norm, allworldcorners3d); //nolinear optimization,Q_wc & T_wc���г�ֵ
	double optmrvec[3] = {0.0};
	Q2Rvec(Q_wc, optmrvec);
	Mat optmraux = (Mat_<double>(3, 1) << optmrvec[0], optmrvec[1], optmrvec[2]); //ceres�Ż�֮�����ת������ƽ��������cv��ʽ��
	Mat optmtaux = (Mat_<double>(3, 1) << T_wc[0], T_wc[1], T_wc[2]);
	projectPoints(allworldcorners3d, optmraux, optmtaux, camMatrix, cv::noArray(), optm_allcorners_repro, jacobian); //df/dr,df/dt,......
	for (int j = 0; j < inliers.size(); j++)																		 //ceres�Ż�֮�����ͶӰ���
	{
		optm_reproject_err += (sqrt((allcorners_undis[j].x - optm_allcorners_repro[j].x) * (allcorners_undis[j].x - optm_allcorners_repro[j].x) / fx / fx + (allcorners_undis[j].y - optm_allcorners_repro[j].y) * (allcorners_undis[j].y - optm_allcorners_repro[j].y) / fy / fy) / inliers.size());
	}
	LOG_IF(INFO, (optm_reproject_err * 234) > 1) << "img: " << imgindex << " reproject_err: " << reproject_err << " optm_reproject_err: " << optm_reproject_err << " inliers " << inliers.size();
����*/
	rjacobian = jacobian.colRange(0, 3);
	tjacobian = jacobian.colRange(3, 6);
	cv2eigen(rjacobian, rjacobianEg);
	cv2eigen(tjacobian, tjacobianEg);
	Eigen::MatrixXd fulljacobian(allcorners_undis.size() * 2, 6);
	Eigen::MatrixXd finalcov; //������
	fulljacobian.leftCols<3>() = rjacobianEg;
	fulljacobian.rightCols<3>() = tjacobianEg;

	//�Ƶ�����ʦ�֣���С����.�۲�ֵ�ķ���Ϊ��λ����������ȼ�
	//finalcov = (fulljacobian.transpose() * fulljacobian).inverse() * fulljacobian.transpose() * covariance * fulljacobian * (fulljacobian.transpose() * fulljacobian).inverse();
	finalcov = (fulljacobian.transpose() * weight * fulljacobian).inverse();
	Eigen::VectorXd deltax = (fulljacobian.transpose() * fulljacobian).inverse() * fulljacobian.transpose() * reproj_d;
	LOG(INFO) << "fulljacobian\n"
			  << fulljacobian;
	LOG(INFO) << "finalcov\n"
			  << finalcov;
	LOG(INFO) << "deltax:\n"
			  << deltax;
	LOG(INFO) << "reproj_d:\n"
			  << reproj_d;
	raux.convertTo(Rvec, CV_32F); //��ת��������������������ceres�Ż�ǰ��Ľ����
	taux.convertTo(Tvec, CV_32F); //ƽ������
	Rodrigues(Rvec, rotMat);
	cv2eigen(rotMat, R_wcEg);
	cv2eigen(Tvec, T_wcEg);
	{
		Eigen::MatrixXd m_rjacobian(allcorners_undis.size() * 2, 3);
		Eigen::MatrixXd m_tjacobian(allcorners_undis.size() * 2, 3);
		m_caljacobian(R_wcEg, T_wcEg, allworldcorners3d, m_rjacobian, m_tjacobian);
		Eigen::MatrixXd m_fulljacobian(allcorners_undis.size() * 2, 6);
		Eigen::MatrixXd m_finalcov; //������
		m_fulljacobian.leftCols<3>() = m_rjacobian;
		m_fulljacobian.rightCols<3>() = m_tjacobian;
		m_finalcov = (m_fulljacobian.transpose() * weight * m_fulljacobian).inverse();
		Eigen::VectorXd m_deltax = (m_fulljacobian.transpose() * m_fulljacobian).inverse() * m_fulljacobian.transpose() * reproj_d;
		LOG(INFO) << "m_fulljacobian\n"
				  << m_fulljacobian;
		LOG(INFO) << "m_finalcov\n"
				  << m_finalcov;
		LOG(INFO) << "m_deltax:\n"
				  << m_deltax;
	}

	R_cw = R_wcEg.transpose();
	//since the world coordinate is not orthometric with th NED,we must compensate the yaw angle
	Eigen::Vector3d Eul_R_wn(0.0, 0.0, 90 - 4.5083); //calibrate from data_09
	Eigen::Vector3d Eul_R_bc(-0.723, 0.546, -86.984);
	Utility::Eul2DCM(Eul_R_wn, R_wn);
	Utility::Eul2DCM(Eul_R_bc, R_bc);

	P_w = -R_cw * T_wcEg / 1000.0;
	Eigen::Matrix3d R_bn = R_wn * R_cw * R_bc; //cal euler must transform to Cbn
	Utility::DCM2Eul(R_bn, Euler);

	if (abs(P_w[2] / 1000.0) > 5)
	{
		printf("%dOutlier\n", imgindex);
	}
	Eigen::Vector3d calBLH;

	P_n = R_wn * P_w;
	Utility::NED2BLH(origBLH, P_n, calBLH);
	cout << setprecision(10) << "estimate:  " << gpst << "\n"
		 << "calBLH:\n"
		 << calBLH << "\nEuler\n"
		 << Euler << "\nR_cw\n"
		 << R_cw << endl;
	//����ĵ�λ����̬�ǻ��ȣ�λ����mm;��λ�����γ�ȵĵ�λ�ǻ��ȣ���̬�ĵ�λ�Ƕ�.��ά������ϵ�ͱ���������ϵ���˾�ʮ��

	results << setprecision(10) << fixed << gpst << " " << calBLH[0] * RAD2DEG << " " << calBLH[1] * RAD2DEG << " " << calBLH[2] << " " << abs(deltax[4]) * 1000 << " " << abs(deltax[3]) * 1000 << " " << abs(deltax[5]) * 1000 << " " << Euler[0] << " " << Euler[1] << " " << Euler[2] << endl;

	pre_pts.clear();
	pre_pts = allcorners; //������һ֡����׷��
	previmg.release();
	originimg.copyTo(previmg); //����ط�����matֱ����Ȼ������⣡������������û���е�������mat��ֵ����ֱ�������ǳ����������mat�Ṳ��һ���ռ䣩
	return true;
}

bool QRCodeSolve::evaluatePOSE(std::vector<Marker> &detectedMarkers, double gpst)
{
	const int length = 1024;
	double gpssow;
	double mean_err = 0.0;
	char camPOSE[length]; //export imu POSE to camPOSE by GINS
	Eigen::Vector3d camBLH, T_wc, T_cw, P_c, P_n, P_w, P_ctmp, Euler;
	Eigen::Matrix3d R_cn, R_cw, camMatrixEg, R_bn, R_wn, R_cb, R_bc;
	vector<Eigen::Vector3d> allworldcorners3d, allwordcorners3d_reproj, allcorners3d, deltapoints;
	Eigen::Vector3d Eul_R_wn(0.0, 0.0, 90 - 4.5083);
	Eigen::Vector3d Eul_R_bc(-0.723, 0.546, -86.984);
	Utility::Eul2DCM(Eul_R_wn, R_wn);
	Utility::Eul2DCM(Eul_R_bc, R_bc);
	R_cb = R_bc.inverse();
	Eigen::Matrix3d R_nw;
	R_nw = R_wn.inverse();

	while (evaluatefile.getline(camPOSE, length))
	{
		char *ptr;
		ptr = strtok(camPOSE, " ");
		double gpsweek = strtod(ptr, NULL);
		ptr = strtok(NULL, " ");
		gpssow = strtod(ptr, NULL);
		if (abs(gpssow - gpst) <= 0.01)
		{
			ptr = strtok(NULL, " ");
			camBLH[0] = strtod(ptr, NULL);
			ptr = strtok(NULL, " ");
			camBLH[1] = strtod(ptr, NULL);
			ptr = strtok(NULL, " ");
			camBLH[2] = strtod(ptr, NULL);
			ptr = strtok(NULL, " ");
			double Vn = strtod(ptr, NULL);
			ptr = strtok(NULL, " ");
			double Ve = strtod(ptr, NULL);
			ptr = strtok(NULL, " ");
			double Vd = strtod(ptr, NULL);
			ptr = strtok(NULL, " ");
			Euler[0] = strtod(ptr, NULL);
			ptr = strtok(NULL, " ");
			Euler[1] = strtod(ptr, NULL);
			ptr = strtok(NULL, " ");
			Euler[2] = strtod(ptr, NULL);

			Utility::Eul2DCM(Euler, R_bn);			//Cbn
			R_cw = R_nw * R_bn * R_cb;				//for pose from pure visual
			Utility::BLH2NED(origBLH, camBLH, P_n); //P_n(cam coors in ned)
			T_cw = R_nw * P_n;						//P_w(cam coors in world) = T_cw
			T_wc = -R_cw.inverse() * T_cw;

			Marker m = detectedMarkers[0];
			cv2eigen(camMatrix, camMatrixEg);

			for (int i = 0; i < m.points.size(); i++)
			{

				Eigen::Vector3d tmp_3(m.points[i].x, m.points[i].y, 1);
				Eigen::Vector3d tmp_2(m_markerCorners3d[i].x, m_markerCorners3d[i].y, m_markerCorners3d[i].z);

				allworldcorners3d.push_back(tmp_2);
				allcorners3d.push_back(tmp_3);
			}

			for (int i = 0; i < allworldcorners3d.size(); i++)
			{
				P_c = R_cw.inverse() * 0.001 * allworldcorners3d[i] + T_wc;
				P_ctmp = P_c[2] * camMatrixEg.inverse() * allcorners3d[i];
				P_w = R_cw * P_ctmp + T_cw;
				allwordcorners3d_reproj.push_back(P_w);
				deltapoints.push_back(0.001 * allworldcorners3d[i] - P_w);
				mean_err += (0.001 * allworldcorners3d[i] - P_w).norm();
			}
			evaluate_outfile << setprecision(6) << fixed << gpst << " " << mean_err / 4.0 << endl;
			cout << "mean_err " << mean_err / 4.0 << endl;
			cout << setprecision(10) << "evaluate:  " << gpssow << "\n"
				 << "camBLH:\n"
				 << camBLH << "\nEuler\n"
				 << Euler << "\nR_cw\n"
				 << R_cw << endl;
			break;
		}
		else if (gpssow > gpst)
			return false;
	}

	return true;
}