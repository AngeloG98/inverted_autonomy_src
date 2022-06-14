#pragma once

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <bspline_opt/uniform_bspline.h>
#include <plan_manage/plan_helper.h>

using namespace bspline_opt;

namespace inverted_planner{

class PlanManager{
public:
    PlanManager();
    ~PlanManager();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void cubicBsplineFromControlPoint(const Eigen::MatrixXd &ctrl_pts, const double &ts);

    LocalTrajInfo local_traj_;
private:
    void updateTrajInfo(const UniformBspline &position_traj, const ros::Time time_now);
};

} // namespace inverted_planner
