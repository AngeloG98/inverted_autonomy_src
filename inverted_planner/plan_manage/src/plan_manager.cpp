#include <plan_manage/plan_manager.h>

namespace inverted_planner
{
PlanManager::PlanManager()
{
}

PlanManager::~PlanManager()
{
}

void PlanManager::cubicBsplineFromControlPoint(
        const Eigen::MatrixXd &ctrl_pts, const double &ts){
    updateTrajInfo(UniformBspline(ctrl_pts, 4, ts), ros::Time::now());
}

void PlanManager::updateTrajInfo(
        const UniformBspline &position_traj, const ros::Time time_now){
    local_traj_.start_time_ = time_now;
    local_traj_.position_traj_ = position_traj;
    local_traj_.velocity_traj_ = local_traj_.position_traj_.getDerivative();
    local_traj_.acceleration_traj_ = local_traj_.velocity_traj_.getDerivative();
    local_traj_.start_pos_ = local_traj_.position_traj_.evaluateDeBoorT(0.0);
    local_traj_.duration_ = local_traj_.position_traj_.getTimeSum();
    local_traj_.traj_id_ += 1;
}
} // namespace inverted_planner
