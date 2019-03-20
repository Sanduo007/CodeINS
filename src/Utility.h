/*
 * @Author: yibin wu 
 * @Date: 2019-03-01 15:03:23 
 * @Last Modified by: yibin wu
 * @Last Modified time: 2019-03-01 15:14:41
 */
#include <vector>
#include <string>
#include <eigen3/Eigen/Core>
class Utility
{
public:
  static std::vector<std::string> getFiles(std::string cate_dir);
  static void NED2BLH(Eigen::Vector3d origBLH, Eigen::Vector3d NED, Eigen::Vector3d &calBLH);
  static void BLH2NED(Eigen::Vector3d origBLH, Eigen::Vector3d calBLH, Eigen::Vector3d &NED);
  static void Eul2DCM(Eigen::Vector3d Euler, Eigen::Matrix3d &DCM);
  static void DCM2Eul(Eigen::Matrix3d DCM, Eigen::Vector3d &Euler);

  template <typename Derived>
  static Eigen::Matrix<typename Derived::Scalar, 3, 3> skewSymmetric(const Eigen::MatrixBase<Derived> &q)
  {
    Eigen::Matrix<typename Derived::Scalar, 3, 3> ans;
    ans << typename Derived::Scalar(0), -q(2), q(1),
        q(2), typename Derived::Scalar(0), -q(0),
        -q(1), q(0), typename Derived::Scalar(0);
    return ans;
  }
};