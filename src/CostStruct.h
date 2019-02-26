#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <eigen3/Eigen/Core>

struct REPROJECT_COST
{
    REPROJECT_COST(double _observed_norm_u, double _observed_norm_v, Eigen::Vector3d _pw)
        : observed_norm_u(_observed_norm_u), observed_norm_v(_observed_norm_v), pw(_pw)
    {
    }

    template <typename T>
    bool operator()(const T *const Rwc, const T *const Twc, T *residuals) const //前两维是待估参数
    {
        T pc[3];
        T pwt[3] = {T(pw(0)), T(pw(1)), T(pw(2))};

        ceres::QuaternionRotatePoint(Rwc, pwt, pc); //按照此方法，应该传入归一化的uv
        pc[0] += Twc[0];
        pc[1] += Twc[1];
        pc[2] += Twc[2];

        T up = pc[0] / pc[2];
        T vp = pc[1] / pc[2];

        residuals[0] = up - T(observed_norm_u);
        residuals[1] = vp - T(observed_norm_v);
        //LOG(INFO) << "0: " << residuals[0] << " 1: " << residuals[1];
        return true;
    }

    static ceres::CostFunction *Create(const double _observed_norm_u, const double _observed_norm_v, const Eigen::Vector3d _pw)
    {
        return (new ceres::AutoDiffCostFunction<REPROJECT_COST, 2, 4, 3>(new REPROJECT_COST(_observed_norm_u, _observed_norm_v, _pw)));
    }

    double observed_norm_u;
    double observed_norm_v;
    Eigen::Vector3d pw;
};