#include <time.h>
#include <ceres/ceres.h>

#include <opencv2/core/eigen.hpp>

#include "Marker.h"
#include "Const.h"
#include "Struct.h"
#include "QRCodeSolve.h"
#include "CostStruct.h"
#include "PNP/epnp.h"

using namespace std;
using namespace cv;

QRCodeSolve::QRCodeSolve(string config_path, char outputdir_[1000])
{
	readpara(config_path);

	char stime[32] = {'\0'};
	char filename[100] = {'\0'};
	char filepath[1000] = {'\0'};
	char errorimgname[100] = {'\0'};
	char errorimgpath[1000] = {'\0'};
	double blockside2d = 0.0;
	double blockside3d = 0.0;
	Point2f temppoint(0.0, 0.0);
	time_t tt = time(0);
	int shep = strftime(stime, sizeof(stime), "%Y%m%d%H%M%S", localtime(&tt)); //get current UTC time
	sprintf(filename, "%s%s%s", "POSEResults", stime, ".txt");
	sprintf(filepath, "%s%s", outputdir_, filename);
	sprintf(errorimgname, "%s%s%s", "errorimg", stime, ".txt");
	sprintf(errorimgpath, "%s%s", outputdir_, errorimgname);

	m_markerCorners3d.push_back(Point3f(-markerSizeinWorld.width / 2.0, -markerSizeinWorld.height / 2.0, 0));
	m_markerCorners3d.push_back(Point3f(markerSizeinWorld.width / 2.0, -markerSizeinWorld.height / 2.0, 0)); //（右下里）
	m_markerCorners3d.push_back(Point3f(markerSizeinWorld.width / 2.0, markerSizeinWorld.height / 2.0, 0));
	m_markerCorners3d.push_back(Point3f(-markerSizeinWorld.width / 2.0, markerSizeinWorld.height / 2.0, 0));

	m_markerCorners2d.push_back(Point2f(0, 0));
	m_markerCorners2d.push_back(Point2f(markerSize.width - 1, 0));
	m_markerCorners2d.push_back(Point2f(markerSize.width - 1, markerSize.height - 1)); //顺时针
	m_markerCorners2d.push_back(Point2f(0, markerSize.height - 1));

	blockside2d = (markerSize.width - 1) / 7.0;
	blockside3d = markerSizeinWorld.width / 7.0;

	if (markertype == OLDMARKER)
	{
		m_markerinterCorners2d.push_back(Point2f(blockside2d, blockside2d)); //顺时针
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
		m_markerinterCorners2d.push_back(Point2f(blockside2d, blockside2d)); //顺时针
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

	results = fopen(filepath, "w+");
	errorimg = fopen(errorimgpath, "w+");

	if (results == NULL || errorimg == NULL)
	{
		printf("create file failed!\n");
	}
	// if (cameratype == GUIDANCE)
	// {
	// 	//old_version
	// 	// camMatrix = (Mat_<double>(3, 3) << 240.2721, 1.5735, 163.4041, 0.000, 238.7327, 118.7457, 0.000, 0.000, 1.000); //相机内参
	// 	// distCoeff = (Mat_<double>(5, 1) << -0.0123, 0.2247, -0.0090, 0.0018, -0.4472);
	// 	//1206
	// 	camMatrix = (Mat_<double>(3, 3) << 233.6475, 0.000, 164.4622, 0.000, 235.9954, 120.2638, 0.000, 0.000, 1.000); //相机内参
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

	std::string imgdir_str;
	fsSettings["IMAGE_DIR"] >> imgdir_str;
	strcpy(imgdir, imgdir_str.c_str());
}

QRCodeSolve::~QRCodeSolve()
{
	fclose(results);
	fclose(errorimg);

	originimg.release();
	previmg.release();
	m_Mask.release();
}

float QRCodeSolve::perimeter(vector<Point2f> a)
{
	float sum = 0, dx, dy;

	for (size_t i = 0; i < a.size(); ++i)
	{
		size_t i2 = (i + 1) % a.size(); //取模很强，闭合的回路

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
		approxPolyDP(contours[i], approxCurve, double(contours[i].size()) * 0.15, true); //调参.approxCurve，拟合后的多边形轮廓，点集

		if (approxCurve.size() != 4)
			continue;

		if (!isContourConvex(approxCurve)) // 判断是不是凸多边形
			continue;

		float minDist = 1e10; //每次拟合的轮廓都应该与初值比较!(错过)
		for (int i = 0; i < 4; ++i)
		{
			vec = approxCurve[i] - approxCurve[(i + 1) % 4];
			squaredDistance = vec.dot(vec);
			minDist = min(minDist, squaredDistance); //取间距的最小值
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
		double o = (v1.x * v2.y) - (v1.y * v2.x); //几何，巧妙的判断,如果在右边应该交换第二个和第四个点便达到效果
		if (o < 0.0)
		{
			swap(m.points[1], m.points[3]); //swap 交换
		}

		possibleMarkers.push_back(m); //只留下可能的轮廓，轮廓的顶点是拟合出来的！！！！
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
		Mat M = getPerspectiveTransform(marker.points, m_markerCorners2d); //变换关系矩阵
		warpPerspective(grayscale, canonicalMarker, M, markerSize);		   //真正的变换 ，canonicalmarker是图（变换之后canonical典范，size由markerSize指定）

		int nRotations = 5;

		int id = 0;
		id = marker.getMarkerId(canonicalMarker, nRotations); //!!!这个函数里面有去除外围7*7的操作，故传入的是整个标志，若不是id为-1

		if (id == -1) //判断是否符合预定二维码信息
			continue;
		while (nRotations != 0) //迭代，使得检测到的marker点序与实际位置对应。起点和顺序！用于PNP解算
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

		for (size_t i = 0; i < goodMarkers.size(); ++i) //将细化位置复制给标记角点
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

	//
	//
	//
	//
	//
	//
	//

	Mat imgByMory;
	Mat imgByAdptThr;
	vector<Marker> detectedMarkers;
	vector<vector<Point>> contours;

	img.copyTo(originimg);

	for (int k = 0; k < 9; k++) //梯度全局阈值
	{
		cv::threshold(img, imgByAdptThr, imgthr[k], 255, THRESH_BINARY);
		morphologyEx(imgByAdptThr, imgByAdptThr, MORPH_OPEN, Mat());
		morphologyEx(imgByAdptThr, imgByAdptThr, MORPH_CLOSE, Mat());
		//imshow("morphologyEx", imgByAdptThr);
		//waitKey(1);

		imgByMory = imgByAdptThr.clone();

		vector<vector<Point2i>> allContours;
		vector<Vec4i> hierarchy;

		findContours(imgByAdptThr, allContours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE); //输出轮廓点只能为整型

		contours.clear();
		for (size_t i = 0; i < allContours.size(); ++i)
		{
			if (allContours[i].size() >= 4)
			{
				contours.push_back(allContours[i]); //滤除小的轮廓,面积过大或过小
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

			detectMarkers(imgByMory, detectedMarkers); //二值化和形态学处理之后找信息
			if (detectedMarkers.size())
			{
				if (!estiMatePosition(detectedMarkers, gpst))
					continue;
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
根据透视投影变换矩阵M，将规则的二维码内部角点投影到图像上
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
	提取图像中二维码内部的角点，并根据与设定marker角点距离的大小对其进行排序。同时修改外围四点
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

	double ppdist = 0.0;		 //角点到轮廓距离
	double cornermatch_th = 8.0; //检测角点与模板投影角点匹配误差最大阈值
	double tempdist = 0.0;
	int outnum = 0;

	//m_markerinterCorners3d4cal = m_markerinterCorners3d;
	Mat M = getPerspectiveTransform(m_markerCorners2d, m.points);
	warpPerspectivePoints(m_markerinterCorners2d, perspintercorners, M);
	m_Mask = cv::Mat(img_height, img_width, CV_8UC1, cv::Scalar(0));
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
	//imshow("mask", m_Mask);
	//waitKey(1);
	goodFeaturesToTrack(img, corners, 22, 0.01, 5, m_Mask); //提取整张图像中的SHi-Tomasi角点,应深入了解。优于Harris

	for (int j = 0; j < corners.size(); j++)
	{
		ppdist = pointPolygonTest(m.points, corners[j], true);

		if (abs(ppdist) <= 5) //可能会把轮廓点当成内点
		{
			circle(imgdebug, corners[j], 3, Scalar(0, 0, 255), -1, 0);
			outercorners.push_back(corners[j]);
			/*circle(imgdebug, corners[j], 3, Scalar(0, 0, 0), -1, 0);
			imshow("allcornersimg", imgdebug);
			waitKey(1);*/
		}
		else if (ppdist > 0) //提出轮廓内的角点
		{
			circle(imgdebug, corners[j], 3, Scalar(0, 255, 0), -1, 0);
			intercorners.push_back(corners[j]);
		}
	}

	for (int k = 0; k < perspintercorners.size(); k++) //根据二维码投影到像素平面上的内点
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
		if (mindist < cornermatch_th) //离投影角点最近的图像角点应在阈值内
		{
			m_markerinterCorners3d4cal.push_back(m_markerinterCorners3d[k]); //只有“有对应图像角点”的3D点才参与计算
			intercorners_seq.push_back(temppoint);							 //对检测出的内部角点排序
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
				//cornerSubPix(img, vectortmp, Size(5, 5), Size(-1, -1), cvTermCriteria(CV_TERMCRIT_ITER, 30, 0.1)); //最小二乘法细化内部角点，亚像素级

				m.points[q].x = (vectortmp[0].x + m.points[q].x) / 2.0; //细化轮廓外部四个角点
				m.points[q].y = (vectortmp[0].y + m.points[q].y) / 2.0;
			}
		}
	}

	cornerSubPix(img, intercorners_seq, Size(5, 5), Size(-1, -1), cvTermCriteria(CV_TERMCRIT_ITER, 30, 0.1)); //最小二乘法细化内部角点，亚像素级

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
	loss_function = new ceres::CauchyLoss(1.0); //损失函数，减少大的残差的影响，如错误的视觉观测

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
	poseoptions.minimizer_progress_to_stdout = false; // 输出到cout
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

		distortion(Eigen::Vector2d(mx_d, my_d), d_u); //在这里第一个参数是临时变量，是转瞬即逝的。const 引用可以初始化为不同类型的对象或者初始化为右值
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
	Eigen::Matrix3d R_wcEg, R_cw, R_bw, R_wn; //rotate matrix from world to camera !欧拉角的定义
	Eigen::Vector3d P_w, T_wcEg, P_n;		  //the Marker's center in UAVbody coor ,which is parelled with camera coor.transformation from world to camera,Eigen
	Mat_<float> rotMat(3, 3);
	Mat rgbimg;

	double fx = camMatrix.at<double>(0, 0); //double or uchar depends on the type of Matrix!
	double fy = camMatrix.at<double>(1, 1);
	double cx = camMatrix.at<double>(0, 2);
	double cy = camMatrix.at<double>(1, 2);

	getintercorners(originimg, m, intercorners);

	allcorners.insert(allcorners.end(), m.points.begin(), m.points.end()); //matched points pair for calculating
	allcorners.insert(allcorners.end(), intercorners.begin(), intercorners.end());
	allworldcorners3d.insert(allworldcorners3d.end(), m_markerCorners3d.begin(), m_markerCorners3d.end());
	allworldcorners3d.insert(allworldcorners3d.end(), m_markerinterCorners3d4cal.begin(), m_markerinterCorners3d4cal.end());

	m_undistortPoints(allcorners, m_allcorners_undis);
	undistortPoints(allcorners, allcorners_undis, camMatrix, distCoeff, cv::noArray(), camMatrix); //调用方法需注意.计算之前先去畸变

	cvtColor(originimg, rgbimg, cv::COLOR_GRAY2BGR);
	vector<Point2f> deltapoint;
	for (int i = 0; i < allcorners.size(); i++)
	{
		allcorners_norm.push_back(cv::Point2f((allcorners_undis[i].x - cx) / fx, (allcorners_undis[i].y - cy) / fy)); //归一化角点
		//circle(rgbimg, allcorners[i], 3, Scalar(0, 0, 255), -1, 0);
		deltapoint.push_back(allcorners_undis[i] - m_allcorners_undis[i]);
	}

	vector<int> inliers;
	//solvePnP(allworldcorners3d, allcorners_undis, camMatrix, cv::noArray(), raux, taux, false, SOLVEPNP_ITERATIVE);

	{
		solvePnPRansac(allworldcorners3d, allcorners_undis, camMatrix, cv::noArray(), raux, taux, false, 100, 1.0, 0.99, inliers, SOLVEPNP_ITERATIVE);
		if (inliers.size() < 4)
		{
			m_markerinterCorners3d4cal.clear();
			return false;
		}

		for (int i = 0; i < inliers.size(); i++) //use inlier points to recize all points vectors
		{
			allcorners_undis[i] = allcorners_undis[inliers[i]];   //去畸变的角点
			allworldcorners3d[i] = allworldcorners3d[inliers[i]]; //原始的角点的3d坐标
			allcorners_norm[i] = allcorners_norm[inliers[i]];	 //去畸变后归一化的角点
			circle(rgbimg, allcorners[inliers[i]], 1.5, Scalar(0, 255, 0), -1, 0);
		}
		allcorners_undis.resize(inliers.size());
		allworldcorners3d.resize(inliers.size());
		allcorners_norm.resize(inliers.size());
		LOG_IF(INFO, inliers.size() < 10) << "inliers " << inliers.size();
	}
	m_markerinterCorners3d4cal.clear();																						//must,wyb
	Eigen::MatrixXd covariance = Eigen::MatrixXd::Identity(allworldcorners3d.size() * 2, allworldcorners3d.size() * 2) * 2; //RANSAC之后的角点提取误差为一个像素

	imshow("QRALLCorner", rgbimg);
	waitKey(1);

	projectPoints(allworldcorners3d, raux, taux, camMatrix, cv::noArray(), allcorners_repro, jacobian); //df/dr,df/dt,......(imagepoints)
	for (int j = 0; j < allcorners_undis.size(); j++)													//都是有畸变的点,优化之前的重投影误差
	{
		reproject_err += (sqrt((allcorners_undis[j].x - allcorners_repro[j].x) * (allcorners_undis[j].x - allcorners_repro[j].x) / fx / fx + (allcorners_undis[j].y - allcorners_repro[j].y) * (allcorners_undis[j].y - allcorners_repro[j].y) / fy / fy) / inliers.size());
	}

	raux.convertTo(Rvec, CV_32F); //旋转向量
	taux.convertTo(Tvec, CV_32F); //平移向量
	cv2double(Rvec, Tvec);
	cout << "Q_wc: " << Q_wc[0] << " " << Q_wc[1] << " " << Q_wc[2] << " " << Q_wc[3] << endl;
	cout << "T_wc: " << T_wc[0] << " " << T_wc[1] << " " << T_wc[2] << endl;
	LOG(INFO) << "Q_wc: " << Q_wc[0] << " " << Q_wc[1] << " " << Q_wc[2] << " " << Q_wc[3];
	LOG(INFO) << "T_wc: " << T_wc[0] << " " << T_wc[1] << " " << T_wc[2];

	/*//ceres优化
	optimize(allcorners_norm, allworldcorners3d); //nolinear optimization,Q_wc & T_wc已有初值
	double optmrvec[3] = {0.0};
	Q2Rvec(Q_wc, optmrvec);
	Mat optmraux = (Mat_<double>(3, 1) << optmrvec[0], optmrvec[1], optmrvec[2]); //ceres优化之后的旋转向量和平移向量（cv形式）
	Mat optmtaux = (Mat_<double>(3, 1) << T_wc[0], T_wc[1], T_wc[2]);
	projectPoints(allworldcorners3d, optmraux, optmtaux, camMatrix, cv::noArray(), optm_allcorners_repro, jacobian); //df/dr,df/dt,......
	for (int j = 0; j < inliers.size(); j++)																		 //ceres优化之后的重投影误差
	{
		optm_reproject_err += (sqrt((allcorners_undis[j].x - optm_allcorners_repro[j].x) * (allcorners_undis[j].x - optm_allcorners_repro[j].x) / fx / fx + (allcorners_undis[j].y - optm_allcorners_repro[j].y) * (allcorners_undis[j].y - optm_allcorners_repro[j].y) / fy / fy) / inliers.size());
	}
	LOG_IF(INFO, (optm_reproject_err * 234) > 1) << "img: " << imgindex << " reproject_err: " << reproject_err << " optm_reproject_err: " << optm_reproject_err << " inliers " << inliers.size();
　　*/
	rjacobian = jacobian.colRange(0, 3);
	tjacobian = jacobian.colRange(3, 6);
	cv2eigen(rjacobian, rjacobianEg);
	cv2eigen(tjacobian, tjacobianEg);
	Eigen::MatrixXd fulljacobian(allcorners_undis.size() * 2, 6);
	Eigen::MatrixXd finalcov; //方差阵
	fulljacobian.leftCols<3>() = rjacobianEg;
	fulljacobian.rightCols<3>() = tjacobianEg;

	//推导，蒋师兄，最小二乘.观测值的方差为单位阵，所以这里等价
	//finalcov = (fulljacobian.transpose() * fulljacobian).inverse() * fulljacobian.transpose() * covariance * fulljacobian * (fulljacobian.transpose() * fulljacobian).inverse();
	finalcov = (fulljacobian.transpose() * covariance.inverse() * fulljacobian).inverse();
	LOG(INFO) << "fulljacobian\n"
			  << fulljacobian;
	LOG(INFO) << "finalcov\n"
			  << finalcov;

	raux.convertTo(Rvec, CV_32F); //旋转向量，最终输出（可输出ceres优化前后的结果）
	taux.convertTo(Tvec, CV_32F); //平移向量
	Rodrigues(Rvec, rotMat);
	cv2eigen(rotMat, R_wcEg);
	cv2eigen(Tvec, T_wcEg);

	R_cw = R_wcEg.transpose();

	P_w = -R_cw * T_wcEg / 1000.0;
	Euler[0] = atan2(R_cw(2, 1), R_cw(2, 2)) * RAD2DEG;												  //roll,ref:Niu's PPT
	Euler[1] = atan2(-R_cw(2, 0), sqrt(R_cw(2, 1) * R_cw(2, 1) + R_cw(2, 2) * R_cw(2, 2))) * RAD2DEG; //pitch
	Euler[2] = atan2(R_cw(1, 0), R_cw(0, 0)) * RAD2DEG;												  //yaw
	if (abs(P_w[2] / 1000.0) > 5)
	{
		printf("%dOutlier\n", imgindex);
	}
	Eigen::Vector3d origBLH(0.532810211116742, 1.995886165630296, 22.366768037761343); //origin point of code coordinate
	Eigen::Vector3d calBLH;
	R_wn << 0, -1, 0,
		1, 0, 0,
		0, 0, 1;
	P_n = R_wn * P_w;
	NED2BLH(origBLH, P_n, calBLH);

	//方差的单位，姿态是弧度，位置是mm;定位结果经纬度的单位是弧度，姿态的单位是度.二维码坐标系和北东地坐标系差了九十度
	// fprintf(results, "%f %.10f %.10f %f %f %f %f %lf %lf %lf %lf %lf %lf\n", gpst,
	// 		calBLH[0], calBLH[1], calBLH[2], Euler[0], Euler[1], Euler[2], finalcov.diagonal()[3], finalcov.diagonal()[4], finalcov.diagonal()[5], finalcov.diagonal()[0], finalcov.diagonal()[1], finalcov.diagonal()[2]);
	fprintf(results, "%f %.10f %.10f %.10f %.10f %.10f %.10f %d\n", gpst, calBLH[0] * RAD2DEG, calBLH[1] * RAD2DEG, calBLH[2], sqrt(finalcov.diagonal()[4]), sqrt(finalcov.diagonal()[3]), sqrt(finalcov.diagonal()[5]), inliers.size());

	pre_pts.clear();
	pre_pts = allcorners; //留作下一帧光流追踪
	previmg.release();
	originimg.copyTo(previmg); //这个地方两个mat直接相等会有问题！！！！！程序没运行到这里会给mat赋值，（直接相等是浅拷贝，两个mat会共用一处空间）
	return true;
}
void QRCodeSolve::NED2BLH(Eigen::Vector3d origBLH, Eigen::Vector3d NED, Eigen::Vector3d &calBLH)
{
	Eigen::Matrix3d Drinv = Eigen::Matrix3d::Zero();
	double Rm, Rn;
	Rm = a_WGS84 * (1 - e2) / pow(1 - e2 * sin(origBLH[0]) * sin(origBLH[0]), 1.5);
	Rn = a_WGS84 / sqrt(1 - e2 * sin(origBLH[0]) * sin(origBLH[0]));
	Drinv(0, 0) = 1 / (Rm + origBLH[2]);
	Drinv(1, 1) = 1 / (Rn + origBLH[2]) / cos(origBLH[0]);
	Drinv(2, 2) = -1;
	calBLH = Drinv * NED + origBLH;
}
