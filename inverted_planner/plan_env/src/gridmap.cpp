#include <plan_env/gridmap.h>

namespace inverted_planner
{
GridMap::GridMap(ros::NodeHandle &nh)
{
    nh_ = nh;
    map_data_ = std::make_shared<inverted_planner::MapHelper>();
    map_data_->setMapParams();

    int buffer_size = map_data_->map_grid_num(0) * map_data_->map_grid_num(1) * map_data_->map_grid_num(2);
    map_data_->occupancy_buffer = std::vector<char>(buffer_size, 0);

    map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/plan_env/grid_map", 1);
    // update_timer_ = nh_.createTimer(ros::Duration(0.05), &LwqFlatness::updateMapCallback, this);
    vis_timer_ = nh_.createTimer(ros::Duration(0.1), &GridMap::visMapCallback, this);

    // cloud init
    cloud_.width = cloud_.points.size();
    cloud_.height = 1;
    cloud_.is_dense = true;
    cloud_.header.frame_id = "map";

    // objects
    Eigen::MatrixXd objectlist(3, 3);
    setOccupancyObject(objectlist);
}

void GridMap::resetOccupancy() // local range reset
{
    Eigen::Vector3d min_pos = map_data_->map_pos_min;
    Eigen::Vector3d max_pos = map_data_->map_pos_max;

    Eigen::Vector3i min_id, max_id;
    posToIndex(min_pos, min_id);
    posToIndex(max_pos, max_id);

    boundIndex(min_id);
    boundIndex(max_id);

    for (int x = min_id(0); x <= max_id(0); ++x)
        for (int y = min_id(1); y <= max_id(1); ++y)
            for (int z = min_id(2); z <= max_id(2); ++z)
            {
                map_data_->occupancy_buffer[toAddress(x, y, z)] = 0;
            }

    ROS_INFO("Map reset success!");
}

void GridMap::setOccupancy(Eigen::Vector3d pos, double occ){
    if (occ != 1 && occ != 0) {
        ROS_ERROR("Invalid occupancy, should be 0 or 1!");
        return;
    }

    if (!isInMap(pos)) return;

    Eigen::Vector3i id;
    posToIndex(pos, id);

    map_data_->occupancy_buffer[toAddress(id)] = occ;
}

void GridMap::setOccupancyObject(Eigen::MatrixXd objectlist, double occ){
    ROS_INFO("Setting objects occupancy...");
    // for (int i = 0; i < objectlist.cols(); i++){

    // }

    // box
    Eigen::Vector3d object_up(0.3, 10, 17);
    Eigen::Vector3d object_low(-0.3, -10, 12);

    // set a box-like range
    Eigen::Vector3d pos;
    pcl::PointXYZ pt;
    for (int x = object_low(0)*map_data_->resolution_inv; x <= object_up(0)*map_data_->resolution_inv; x++)
        for (int y = object_low(1)*map_data_->resolution_inv; y <= object_up(1)*map_data_->resolution_inv; y++)
            for (int z = object_low(2)*map_data_->resolution_inv; z <= object_up(2)*map_data_->resolution_inv; z++){
                // set occupancy_buffer
                pos << x*map_data_->resolution, y*map_data_->resolution, z*map_data_->resolution;
                setOccupancy(pos, occ);

                // set cloud for display
                if (occ == 1){
                    pt.x = pos(0);
                    pt.y = pos(1);
                    pt.z = pos(2);
                    cloud_.push_back(pt);
                }
            }
    ROS_INFO("Objects occupied success!");
}

int GridMap::getOccupancy(Eigen::Vector3d pos) {
  if (!isInMap(pos)) return -1;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  return int(map_data_->occupancy_buffer[toAddress(id)]);
}

void GridMap::visMapCallback(const ros::TimerEvent &time){
    // if (map_pub_.getNumSubscribers() <= 0)
    //     return;
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud_, cloud_msg);
    map_pub_.publish(cloud_msg);
}

} // namespace inverted_planner