#include <dynamic_search/rotation_astar.h>

namespace inverted_planner
{
RotationAstar::RotationAstar(){}

void RotationAstar::init(){
    // init map
    gridmap_->getMapInfo(map_origin_, map_size_, resolution_);
    inv_resolution_ = 1.0 / resolution_;
    ROS_INFO("Gridmap for rotation astar received: [map size x:%f, map size y:%f, map size z:%f]",
             map_size_(0), map_size_(1), map_size_(2));

    // init time
    time_resolution_ = 0.1;
    inv_time_resolution_ = 1.0 / time_resolution_;

    // searching parameters setup
    tau_ = 0.1;
    lambda_h_ = 1.0;
    memory_size_ = 100000;
    safety_check_ = 5;
    transition_ = Eigen::MatrixXd::Identity(6, 6);

    // init memory
    traj_nodes_mem_.resize(memory_size_);
    for (int i = 0; i < memory_size_; i++) {
        traj_nodes_mem_[i] = new TrajNode;
    }

    // init count 
    used_node_num_ = 0;
    step_num_ = 0;
}

void RotationAstar::reset(){
    // reset used memory
    for (int i = 0; i < used_node_num_; i++) {
        delete traj_nodes_mem_[i];
        traj_nodes_mem_[i] = new TrajNode;
    }
    expanded_nodes_.clear();
    full_traj_nodes_.clear();
    
    // clear open set
    std::priority_queue<TrajNodePtr, std::vector<TrajNodePtr>, NodeComparator> empty;
    open_set_.swap(empty);

    // reset count 
    used_node_num_ = 0;
    step_num_ = 0;
}

RotationAstar::~RotationAstar(){
    // clear memory
    for (int i = 0; i < memory_size_; i++) {
        delete traj_nodes_mem_[i];
    }
}

void RotationAstar::setGridMap(const std::shared_ptr<inverted_planner::GridMap> &gridmap){
    gridmap_ = gridmap;
}

Eigen::Vector3i RotationAstar::posToIndex(Eigen::Vector3d pos){
    Eigen::Vector3i index;
    for (int i = 0; i < 3; ++i)
        index(i) = floor((pos(i) - map_origin_(i)) * inv_resolution_);
    return index;
}

int RotationAstar::timeToIndex(double time){
    int index = floor((time - time_origin_) * inv_time_resolution_);
    return index;
}

void RotationAstar::stateTransition(Eigen::Matrix<double, 6, 1> &state_i, Eigen::Matrix<double, 6, 1> &state_f,
                     Eigen::Vector3d input, double tau){
    // state
    for (int i = 0; i < 3; i++) {
        transition_(i, i + 3) = tau;
    }

    // input
    Eigen::Matrix<double, 6, 1> input_integral;
    input_integral.head(3) = 0.5 * pow(tau, 2) * input; // pos
    input_integral.tail(3) = tau * input; // vel
    
    // transition
    state_f = transition_ * state_i + input_integral;
}


int RotationAstar::search(Eigen::Vector3d start_pos, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                     Eigen::Vector3d end_pos, Eigen::Vector3d end_vel, double start_time){
    // end info
    Eigen::Matrix<double, 6, 1> end_state;
    Eigen::Vector3i end_p_index;
    end_state.head(3) = end_pos;
    end_state.tail(3) = end_vel;
    end_p_index = posToIndex(end_pos);
    double full_duration;

    // start time
    time_origin_ = start_time;
    // start node
    TrajNodePtr curr_node = traj_nodes_mem_[0];
    curr_node->parent = NULL;
    curr_node->state.head(3) = start_pos;
    curr_node->state.tail(3) = start_vel;
    curr_node->last_input = start_acc;
    curr_node->timestamp = start_time;
    curr_node->p_index = posToIndex(start_pos);
    curr_node->t_index = timeToIndex(start_time);
    curr_node->g_score = 0.0;
    curr_node->h_score = funcH();
    curr_node->f_score = curr_node->g_score + lambda_h_ * curr_node->h_score;

    // start open set and expand
    curr_node->node_state = OPEN;
    open_set_.push(curr_node);
    expanded_nodes_.insertNode(curr_node->p_index, curr_node->t_index, curr_node);

    // start count
    used_node_num_ += 1;

    // loop
    while (!open_set_.empty()){
        curr_node = open_set_.top();

        // terminal condition

    }
}

void RotationAstar::getFullTraj(TrajNodePtr end_node){
    TrajNodePtr node = end_node;
    full_traj_nodes_.push_back(node);
    while(node->parent != NULL){
        node = node->parent;
        full_traj_nodes_.push_back(node);
    }
    reverse(full_traj_nodes_.begin(), full_traj_nodes_.end());
}

} // namespace inverted_planner
