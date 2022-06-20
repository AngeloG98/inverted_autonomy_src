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
    time_resolution_ = 0.05;
    inv_time_resolution_ = 1.0 / time_resolution_;

    // searching parameters setup
    tau_ = 0.1;
    max_vel_ = 100.0;
    max_acc_ = 2.0;
    rho_t_ = 1.0;
    horizon_ = 100.0;
    lambda_h_ = 100.0;
    memory_size_ = 100000;
    collision_check_ = 5;
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

void RotationAstar::stateTransition(Eigen::Matrix<double, 6, 1> state_i, Eigen::Matrix<double, 6, 1> &state_f,
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

double RotationAstar::getHeuristicCost(Eigen::Matrix<double, 6, 1> state_i, Eigen::Matrix<double, 6, 1> state_f){
    Eigen::Vector3d pos_i = state_i.head(3);
    Eigen::Vector3d pos_f = state_f.head(3);
    return (pos_i - pos_f).norm();
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
    curr_node->input = start_acc;
    curr_node->duration = 0.0;
    curr_node->timestamp = start_time;
    curr_node->p_index = posToIndex(start_pos);
    curr_node->t_index = timeToIndex(start_time);
    curr_node->g_cost = 0.0;
    curr_node->h_cost = getHeuristicCost(curr_node->state, end_state);
    curr_node->f_cost = curr_node->g_cost + lambda_h_ * curr_node->h_cost;

    // start open set and hashtable
    curr_node->node_state = OPEN;
    open_set_.push(curr_node);
    expanded_nodes_.insertNode(curr_node->p_index, curr_node->t_index, curr_node);
    // start count
    used_node_num_ += 1;

    TrajNodePtr terminal_node = NULL;
    /*  loop  */
    while (!open_set_.empty()){
        curr_node = open_set_.top();

        // terminal condition
        bool near_end = (curr_node->p_index - end_p_index).norm() <= 2*inv_resolution_;
        // bool near_end = (curr_node->state.head(3)[0] > end_pos[0]);
        bool reach_horizon = (curr_node->state.head(3) - end_state.head(3)).norm() >= horizon_;

        if (near_end || reach_horizon){
            terminal_node = curr_node; // close to end
            getFullTraj(terminal_node);
            return (near_end) ? 1 : 0;
        }

        // if not terminate, start expand
        step_num_ += 1;

        /*  expand  */
        open_set_.pop();
        curr_node->node_state = CLOSED;

        Eigen::Matrix<double, 6, 1> curr_state = curr_node->state;
        Eigen::Matrix<double, 6, 1> next_state;
        double next_time;
        std::vector<TrajNodePtr> neighboor_nodes;
        Eigen::Vector3d input;
        std::vector<Eigen::Vector3d> input_list;

        // normal input list
        // for (double ax = -max_acc_; ax <= max_acc_ + 1e-3; ax += max_acc_ )
        double ax = 0.0;
            for (double ay = -max_acc_; ay <= max_acc_ + 1e-3; ay += max_acc_)
            // double ay = 0.0;
                for (double az = -5*max_acc_; az <= 5*max_acc_ + 1e-3; az += max_acc_ ){
                    input << ax, ay, az;
                    input_list.push_back(input);
                }

        for (int i = 0; i < input_list.size(); ++i){
            input = input_list[i];
            stateTransition(curr_state, next_state, input, tau_);

            next_time = curr_node->timestamp + tau_;
            Eigen::Vector3d next_pos = next_state.head(3);
            Eigen::Vector3d next_vel = next_state.tail(3);
            int next_t_index = timeToIndex(next_time);
            Eigen::Vector3i next_p_index = posToIndex(next_pos);
            
            std::cout <<" x: "<< next_pos(0) <<" y: "<< next_pos(1)<<" z: "<< next_pos(2)<< std::endl;

            // check node if ever been expanded and if closed
            TrajNodePtr next_node = expanded_nodes_.findNode(next_p_index, next_t_index);
            if (next_node != NULL){
                ROS_WARN("Revisit same node!");
                // std::cout <<"Revisit" <<"x: "<< next_pos(0) <<"y: "<< next_pos(1)<<"z: "<< next_pos(2)<< std::endl;
                if (next_node->node_state == CLOSED)
                    continue;
            }

            // check if moved at least one grid
            if ((next_p_index - curr_node->p_index).norm() == 0){
                continue;
            }

            // check if collision
            Eigen::Matrix<double, 6, 1> midd_state;
            Eigen::Vector3d midd_pos;
            double d_tau;
            int collision = 0;
            for (int j = 0; j <= collision_check_; j++){
                d_tau = tau_ * double(j) / double(collision_check_);
                stateTransition(curr_state, midd_state, input, d_tau);
                midd_pos = midd_state.head(3);
                collision = gridmap_->getOccupancy(midd_pos);
                if (collision != 0){
                    ROS_WARN("collision!");
                    break;
                }
                   
            }
            if (collision != 0)
                continue;


            // get node cost
            double node_g_cost = (input.squaredNorm() + rho_t_) * tau_ + curr_node->g_cost;
            double node_h_cost = getHeuristicCost(next_state, end_state);
            double node_f_cost = node_g_cost + lambda_h_ * node_h_cost;

            // update 
            if (next_node == NULL){
                // allocate memory and update node info
                next_node = traj_nodes_mem_[used_node_num_];
                next_node->parent = curr_node;
                next_node->state = next_state;
                next_node->input = input;
                next_node->duration = tau_;
                next_node->timestamp = next_time;
                next_node->p_index = next_p_index;
                next_node->t_index = next_t_index;
                next_node->g_cost = node_g_cost;
                next_node->h_cost = node_h_cost;
                next_node->f_cost = node_f_cost;
                next_node->node_state = OPEN;

                // update open set and hashtable
                open_set_.push(next_node);
                expanded_nodes_.insertNode(next_node->p_index, next_node->t_index, next_node);

                // update mem count
                used_node_num_ += 1;
                if (used_node_num_ == memory_size_){
                    // ROS_ERROR("Run out of node memory!");
                    ROS_WARN("Run out of node memory!");
                    return -1;
                }
            } else if (next_node->node_state == OPEN){
                if (node_f_cost < next_node->f_cost){
                    ROS_WARN("Rearrange openset may needed!");
                    next_node->parent = curr_node;
                    next_node->state = next_state;
                    next_node->input = input;
                    next_node->timestamp = next_time;
                    next_node->g_cost = node_g_cost;
                    next_node->h_cost = node_h_cost;
                    next_node->f_cost = node_f_cost;
                }
            } else {
                ROS_WARN("Node state: CLOSED!");
            }
        }
    }
    // ROS_ERROR("Open set empty!");
    ROS_WARN("Open set empty!");
    return -2;
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

std::vector<Eigen::Vector3d> RotationAstar::getSampleTraj(double sample_rate){
    std::vector<Eigen::Vector3d> pos_sample_list;

    if (!full_traj_nodes_.empty()){
        TrajNodePtr node = full_traj_nodes_.back();
        Eigen::Matrix<double, 6, 1> state_i;
        Eigen::Matrix<double, 6, 1> state_f;
        Eigen::Vector3d input;
        double duration;

        while (node->parent != NULL){
            state_i = node->parent->state;
            input = node->input;
            duration = node->duration;

            for (double d_tau = duration; d_tau >= -1e-5; d_tau -= duration / sample_rate){
                stateTransition(state_i, state_f, input, d_tau);
                pos_sample_list.push_back(state_f.head(3));
            }

            node = node->parent;
        }
        reverse(pos_sample_list.begin(), pos_sample_list.end());
    }

    return pos_sample_list;
}

std::vector<Eigen::Vector3d> RotationAstar::getExpandedPoint(){
    std::vector<Eigen::Vector3d> expanded_pos_list;

    // used node 
    for (int i = 0; i < used_node_num_; i++) {
        expanded_pos_list.push_back(traj_nodes_mem_[i]->state.head(3));
    }

    return expanded_pos_list;
}

} // namespace inverted_planner
