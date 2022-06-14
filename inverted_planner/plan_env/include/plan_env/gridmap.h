#pragma once

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace inverted_planner
{

struct MapHelper
{
    MapHelper() {}
    ~MapHelper() {}
    void setMapParams()
    {
        //
        resolution = 0.1;
        resolution_inv = 1 / resolution;
        map_size << 50.0, 50.0, 50.0;
        map_origin << -map_size(0)/ 2.0, -map_size(1)/ 2.0, 0.0;
        map_pos_min = map_origin;
        map_pos_max = map_origin + map_size;

        for (int i = 0; i < 3; i++){
            map_grid_num(i) = ceil(map_size(i) / resolution);
        }
        ROS_INFO("Map parameters ready!");
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // map parameters
    Eigen::Vector3d map_origin;
    Eigen::Vector3d map_size;
    Eigen::Vector3d map_pos_min, map_pos_max;
    Eigen::Vector3i map_grid_num;
    Eigen::Vector3d map_update_range;
    double resolution;
    double resolution_inv;

    // map data
    std::vector<char> occupancy_buffer;
};

class GridMap{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    GridMap(ros::NodeHandle &nh);
    ~GridMap(){}

    //
    void resetOccupancy();
    void setOccupancy(Eigen::Vector3d pos, double occ = 1);
    void setOccupancyObject(Eigen::MatrixXd objectlist, double occ = 1);
    int getOccupancy(Eigen::Vector3d pos);

    inline int toAddress(const Eigen::Vector3i& id) {
        return id(0) * map_data_->map_grid_num(1) * map_data_->map_grid_num(2) + id(1) * map_data_->map_grid_num(2) + id(2);
    }

    inline int toAddress(int& x, int& y, int& z) {
        return x * map_data_->map_grid_num(1) * map_data_->map_grid_num(2) + y * map_data_->map_grid_num(2) + z;
    }

    inline void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id) {
        for (int i = 0; i < 3; ++i) id(i) = floor((pos(i) - map_data_->map_origin(i)) * map_data_->resolution_inv);
    }

    inline void indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos) {
        for (int i = 0; i < 3; ++i) pos(i) = (id(i) + 0.5) * map_data_->resolution + map_data_->map_origin(i);
    }
    
    inline void getMapInfo(Eigen::Vector3d& origin, Eigen::Vector3d& size, double resolution){
        origin = map_data_->map_origin;
        size = map_data_->map_size;
        resolution = map_data_->resolution;
    }

    inline void boundIndex(Eigen::Vector3i& id) {
        Eigen::Vector3i id1;
        id1(0) = std::max(std::min(id(0), map_data_->map_grid_num(0) - 1), 0);
        id1(1) = std::max(std::min(id(1), map_data_->map_grid_num(1) - 1), 0);
        id1(2) = std::max(std::min(id(2), map_data_->map_grid_num(2) - 1), 0);
        id = id1;
    }

    inline bool isInMap(const Eigen::Vector3d& pos) {
        if (pos(0) < map_data_->map_pos_min(0) + 1e-4 || pos(1) < map_data_->map_pos_min(1) + 1e-4 ||
            pos(2) < map_data_->map_pos_min(2) + 1e-4) {
            return false;
        }
        if (pos(0) > map_data_->map_pos_max(0) - 1e-4 || pos(1) > map_data_->map_pos_max(1) - 1e-4 ||
            pos(2) > map_data_->map_pos_max(2) - 1e-4) {
            return false;
        }
        return true;
    }

    inline bool isInMap(const Eigen::Vector3i& idx) {
        if (idx(0) < 0 || idx(1) < 0 || idx(2) < 0) {
            return false;
        }
        if (idx(0) > map_data_->map_grid_num(0) - 1 || idx(1) > map_data_->map_grid_num(1) - 1 ||
            idx(2) > map_data_->map_grid_num(2) - 1) {
            return false;
        }
        return true;
    }

private:
    std::shared_ptr<inverted_planner::MapHelper> map_data_;
    ros::NodeHandle nh_;

    ros::Publisher map_pub_;
    // ros::Timer update_timer_;
    pcl::PointCloud<pcl::PointXYZ> cloud_;
    ros::Timer vis_timer_;

    // void updateMapCallback(const ros::TimerEvent &time);
    void visMapCallback(const ros::TimerEvent &time);
};

} // namespace inverted_planner
