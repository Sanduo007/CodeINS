#include "Marker.h"
#include "Struct.h"
#include "QRCodeSolve.h"
#include "tic_toc.h"
#include "Utility.h"
#include <iomanip>
using namespace cv;

int main(int argc, char **argv)
{

	google::InitGoogleLogging(argv[0]);
	google::SetLogDestination(google::GLOG_INFO, "../output/wyb_log_");

	Mat img;
	FILE *fimgbin;

	std::vector<double> allimg_timestmp;
	double imgstamp;
	QRCodeSolve qrcodesolver("/home/wyb/wyb/CodeINS/config.yaml", "/home/wyb/wyb/CodeINS/output/");

	/*get timestamp file*/
	std::string timestamppath = qrcodesolver.imgdir_str + std::string("timestamp.bin");
	fimgbin = fopen(timestamppath.c_str(), "rb");
	if (fimgbin == NULL)
	{
		std::cout << "cannot open timestamp file!" << std::endl;
		return 1;
	}
	while (fread((void *)&imgstamp, sizeof(double), 1, fimgbin)) //don't use feof any more
	{
		allimg_timestmp.push_back(imgstamp);
	}
	fclose(fimgbin);

	/*get imgname list*/
	std::vector<std::string> imglist = Utility::getFiles(qrcodesolver.imgdir_str);
	if (imglist.size() != allimg_timestmp.size())
	{
		std::cout << "Fatal err: img nums != timestamp nums."
				  << "img_num: " << imglist.size() << " time_num " << allimg_timestmp.size() << std::endl;
		return 0;
	}
	TicToc tictoc;

	for (int i = 0, iend = imglist.size(); i < iend; ++i)
	{
		qrcodesolver.imgindex = i;
		std::string imgname = imglist[i];

		img = imread(qrcodesolver.imgdir_str + imgname, CV_LOAD_IMAGE_GRAYSCALE);
		if (img.empty())
		{
			std::cout << "No img: " << qrcodesolver.imgdir_str + imgname << std::endl;
			break;
		}
		imshow("img", img);
		waitKey(1);
		std::cout << "-------------img" << qrcodesolver.imgindex << "--------------" << std::endl;
		if (qrcodesolver.imgindex == 1540)
		{
			int kk = 0;
		}

		tictoc.tic();
		qrcodesolver.markerDetection(img, allimg_timestmp[i]);
		std::cout << tictoc.toc() << " ms used for all" << std::endl;
	}
	img.release();
	cvDestroyAllWindows();
	return 1;
}