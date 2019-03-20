/*
 * @Author: yibin wu 
 * @Date: 2019-03-01 15:02:58 
 * @Last Modified by: yibin wu
 * @Last Modified time: 2019-03-01 15:27:12
 */

#include <sys/types.h>
#include <dirent.h>
#include <algorithm>
#include <string.h>
#include "Utility.h"
#include "Const.h"
std::vector<std::string> Utility::getFiles(std::string cate_dir) //get all image files?
{
    std::vector<std::string> files; //save file names

    DIR *dir;
    struct dirent *ptr;
    char base[1000];

    if ((dir = opendir(cate_dir.c_str())) == NULL)
    {
        perror("Open dir error...");
        exit(1);
    }

    while ((ptr = readdir(dir)) != NULL)
    {
        std::string img_name(ptr->d_name);
        if (strcmp(ptr->d_name, ".") == 0 || strcmp(ptr->d_name, "..") == 0) ///current dir OR parent dir
            continue;
        else if (ptr->d_type == 8) ///file
        {
            if (img_name.substr(img_name.size() - 4) == ".jpg" || img_name.substr(img_name.size() - 4) == ".png")
                files.push_back(ptr->d_name);
        }
        else
            continue;
    }
    closedir(dir);

    sort(files.begin(), files.end());
    return files;
}

void Utility::NED2BLH(Eigen::Vector3d origBLH, Eigen::Vector3d NED, Eigen::Vector3d &calBLH)
{
    origBLH[0] = origBLH[0] * DEG2RAD;
    origBLH[1] = origBLH[1] * DEG2RAD;
    Eigen::Matrix3d Drinv = Eigen::Matrix3d::Zero();
    double Rm, Rn;
    Rm = a_WGS84 * (1 - e2) / pow(1 - e2 * sin(origBLH[0]) * sin(origBLH[0]), 1.5);
    Rn = a_WGS84 / sqrt(1 - e2 * sin(origBLH[0]) * sin(origBLH[0]));
    Drinv(0, 0) = 1 / (Rm + origBLH[2]);
    Drinv(1, 1) = 1 / (Rn + origBLH[2]) / cos(origBLH[0]);
    Drinv(2, 2) = -1;
    calBLH = Drinv * NED + origBLH;
}

void Utility::BLH2NED(Eigen::Vector3d origBLH, Eigen::Vector3d calBLH, Eigen::Vector3d &NED)
{
    origBLH[0] = origBLH[0] * DEG2RAD;
    origBLH[1] = origBLH[1] * DEG2RAD;
    calBLH[0] = calBLH[0] * DEG2RAD;
    calBLH[1] = calBLH[1] * DEG2RAD;
    Eigen::Matrix3d Dr = Eigen::Matrix3d::Zero();
    double Rm, Rn;
    Rm = a_WGS84 * (1 - e2) / pow(1 - e2 * sin(origBLH[0]) * sin(origBLH[0]), 1.5);
    Rn = a_WGS84 / sqrt(1 - e2 * sin(origBLH[0]) * sin(origBLH[0]));
    Dr(0, 0) = Rm + origBLH[2];
    Dr(1, 1) = (Rn + origBLH[2]) * cos(origBLH[0]);
    Dr(2, 2) = -1;

    NED = Dr * (calBLH - origBLH);
}
/*
**Transform Euler Angle to DCM.
**DCM = Rcw(means transform/rotate points in body coordinate to points in reference coordinate)
**Euler are the angles in which rotate w to obtain c.(in DEG)
*/
void Utility::Eul2DCM(Eigen::Vector3d Euler, Eigen::Matrix3d &DCM)
{
    double phi = 0.0;
    double theta = 0.0;
    double cosin = 0.0;
    phi = Euler[0] * DEG2RAD;
    theta = Euler[1] * DEG2RAD;
    cosin = Euler[2] * DEG2RAD;

    DCM(0, 0) = cos(theta) * cos(cosin);
    DCM(0, 1) = -cos(phi) * sin(cosin) + sin(phi) * sin(theta) * cos(cosin);
    DCM(0, 2) = sin(phi) * sin(cosin) + cos(phi) * sin(theta) * cos(cosin);
    DCM(1, 0) = cos(theta) * sin(cosin);
    DCM(1, 1) = cos(phi) * cos(cosin) + sin(phi) * sin(theta) * sin(cosin);
    DCM(1, 2) = -sin(phi) * cos(cosin) + cos(phi) * sin(theta) * sin(cosin);
    DCM(2, 0) = -sin(theta);
    DCM(2, 1) = sin(phi) * cos(theta);
    DCM(2, 2) = cos(phi) * cos(theta);
}
/*
**Transform DCM to Euler Angle.
**DCM = Rcw(means transform/rotate points in body coordinate to points in reference coordinate)
**Euler are the angles in which rotate w to obtain c.(in DEG)
*/
void Utility::DCM2Eul(Eigen::Matrix3d DCM, Eigen::Vector3d &Euler)
{
    Euler[0] = atan2(DCM(2, 1), DCM(2, 2)) * RAD2DEG;                                            //roll,ref:Niu's PPT
    Euler[1] = atan2(-DCM(2, 0), sqrt(DCM(2, 1) * DCM(2, 1) + DCM(2, 2) * DCM(2, 2))) * RAD2DEG; //pitch
    Euler[2] = atan2(DCM(1, 0), DCM(0, 0)) * RAD2DEG;                                            //yaw
}