#include <dynamic_search/rotation_astar.h>

namespace inverted_planner
{
RotationAstar::RotationAstar(){}

void RotationAstar::init(){
    // init map
    gridmap_->getMapInfo(map_origin_, map_size_, resolution_);
    resolution_ = 0.1;
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
    rho_t_ = 10000.0;
    horizon_ = 100.0;
    lambda_h_ = 1.0;
    memory_size_ = 100000;
    collision_check_ = 10;
    transition_ = Eigen::MatrixXd::Identity(6, 6);
    gravity_ = 9.80;

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

Eigen::Vector3i RotationAstar::rotToIndex(Eigen::Vector3d rot){
    Eigen::Vector3i index;
    double inv_resolution_ = 12 / M_PI;
    for (int i = 0; i < 3; ++i)
        index(i) = floor((rot(i) - M_PI) * inv_resolution_);
    return index;
}

void RotationAstar::getRotationInputPhiLarge(TrajNodePtr curr_node, std::vector<Eigen::Vector3d> &input_list,
                                             std::vector<Eigen::Vector3d> &rot_list, std::vector<double> &fz_list){
    // phi list
    std::vector<double> phi_list {curr_node->rotation(0) + M_PI / 6, curr_node->rotation(0), curr_node->rotation(0) - M_PI / 6};

    // fz list
    std::vector<double> fz_set {0.8*gravity_, gravity_, 1.5*gravity_};

    // input list
    // for (double theta = -M_PI / 6; theta <= M_PI / 6; theta += M_PI / 6)
    double theta = 0.0;
        for (int i = 0; i < phi_list.size(); i++){
            double tmp_phi = phi_list[i];
            for (int j = 0; j < fz_set.size(); j++){
                double tmp_fz = fz_set[j];
                Eigen::Vector3d rot(tmp_phi, theta, 0.0);
                Eigen::Vector3d input(-tmp_fz * sin(theta),
                                      tmp_fz * sin(tmp_phi) * cos(theta),
                                      -(gravity_ - tmp_fz * cos(tmp_phi) * cos(theta)));
                rot_list.push_back(rot);
                input_list.push_back(input);
                fz_list.push_back(tmp_fz);
            }
        }
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
    return (pos_i - pos_f).norm() / 10.0;
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
    curr_node->rotation << 0.0, 0.0, 0.0; // from flatness 
    curr_node->duration = 0.0;
    curr_node->timestamp = start_time;
    curr_node->p_index = posToIndex(start_pos);
    curr_node->t_index = timeToIndex(start_time);
    curr_node->r_index = rotToIndex(curr_node->rotation);
    curr_node->g_cost = 0.0;
    curr_node->h_cost = rho_t_ * getHeuristicCost(curr_node->state, end_state);
    curr_node->f_cost = curr_node->g_cost + lambda_h_ * curr_node->h_cost;

    // start open set and hashtable
    curr_node->node_state = OPEN;
    open_set_.push(curr_node);
    expanded_nodes_.insertNode(curr_node->p_index, curr_node->r_index, curr_node->t_index, curr_node);
    // start count
    used_node_num_ += 1;

    TrajNodePtr terminal_node = NULL;
    /*  loop  */
    while (!open_set_.empty()){
        curr_node = open_set_.top();

        // terminal condition
        // bool near_end = (curr_node->p_index - end_p_index).norm() <= 2*inv_resolution_;
        bool near_end = (curr_node->state.head(3)[0] >= 2.0);
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
        double fz;
        Eigen::Vector3d input;
        Eigen::Vector3d rotation;
        std::vector<double> fz_list;
        std::vector<Eigen::Vector3d> input_list;
        std::vector<Eigen::Vector3d> rot_list;

        // normal input list
        // // for (double ax = -max_acc_; ax <= max_acc_ + 1e-3; ax += max_acc_ )
        // double ax = 0.0;
        //     // for (double ay = -max_acc_; ay <= max_acc_ + 1e-3; ay += max_acc_)
        //     double ay = 0.0;
        //         for (double az = -5*max_acc_; az <= 5*max_acc_ + 1e-3; az += max_acc_ ){
        //             input << ax, ay, az;
        //             input_list.push_back(input);
        //         }
        
        // rotation aware input list
        getRotationInputPhiLarge(curr_node, input_list, rot_list, fz_list);

        for (int i = 0; i < input_list.size(); ++i){
            fz = fz_list[i];
            input = input_list[i];
            rotation = rot_list[i];
            Eigen::Vector3i next_r_index = rotToIndex(rotation);
            // std::cout << "unexpand============================" << std::endl;
            // std::cout << "fz" << fz << "input" << input << "rot" << rotation << std::endl;
            stateTransition(curr_state, next_state, input, tau_);

            next_time = curr_node->timestamp + tau_;
            Eigen::Vector3d next_pos = next_state.head(3);
            Eigen::Vector3d next_vel = next_state.tail(3);
            int next_t_index = timeToIndex(next_time);
            Eigen::Vector3i next_p_index = posToIndex(next_pos);
            // std::cout << "next_p_index" << next_p_index << std::endl;

            // check node if ever been expanded and if closed
            TrajNodePtr next_node = expanded_nodes_.findNode(next_p_index, next_r_index, next_t_index);
            if (next_node != NULL)
            {
                // ROS_WARN("Revisit same node!");
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
                    // ROS_WARN("collision!");
                    break;
                }
                   
            }
            if (collision != 0)
                continue;


            // get node cost
            double node_g_cost = (fz*fz+ rho_t_) * tau_ + curr_node->g_cost;
            double node_h_cost = rho_t_ * getHeuristicCost(next_state, end_state);
            double node_f_cost = node_g_cost + lambda_h_ * node_h_cost;

            // Compare nodes expanded from the same parent
            bool prune = false;
            for (int j = 0; j < neighboor_nodes.size(); ++j)
            {
                TrajNodePtr neighboor_node = neighboor_nodes[j];
                if ((next_p_index - neighboor_node->p_index).norm() == 0 && next_t_index == neighboor_node->t_index && (next_r_index - neighboor_node->r_index).norm() == 0)
                {
                    prune = true;
                    if (node_f_cost < neighboor_node->f_cost)
                    {
                        neighboor_node->g_cost = node_g_cost;
                        neighboor_node->h_cost = node_h_cost;
                        neighboor_node->f_cost = node_f_cost;
                        neighboor_node->state = next_state;
                        neighboor_node->input = input;
                        // neighboor_node->rotation = rotation;
                        neighboor_node->duration = tau_;
                        neighboor_node->timestamp = next_time;
                        break;
                    }
                }
            }

            // update 
            if (!prune)
            {
            if (next_node == NULL){
                // allocate memory and update node info
                next_node = traj_nodes_mem_[used_node_num_];
                next_node->parent = curr_node;
                next_node->state = next_state;
                next_node->input = input;
                next_node->rotation = rotation;
                next_node->duration = tau_;
                next_node->timestamp = next_time;
                next_node->p_index = next_p_index;
                next_node->t_index = next_t_index;
                next_node->r_index = next_r_index;
                next_node->g_cost = node_g_cost;
                next_node->h_cost = node_h_cost;
                next_node->f_cost = node_f_cost;
                next_node->node_state = OPEN;

                // update open set and hashtable
                // open_set_.push(next_node);
                // expanded_nodes_.insertNode(next_node->p_index, next_node->t_index, next_node);
                neighboor_nodes.push_back(next_node);

                // std::cout << "expand============================" << std::endl;
                // std::cout << " r : " << resolution_;
                // std::cout <<" x: "<< next_pos(0) <<" y: "<< next_pos(1)<<" z: "<< next_pos(2);
                // std::cout <<" p index: "<< next_node->p_index(0) <<" y: "<< next_node->p_index(1)<<" z: "<< next_node->p_index(2);
                // std::cout <<" t index: "<< next_node->t_index;
                // std::cout <<" input: "<< next_node->input;
                // std::cout <<"  last g:"<<curr_node->g_cost<<" node_g_cost: "<< node_g_cost <<" node_h_cost: "<< node_h_cost << std::endl;

                // update mem count
                used_node_num_ += 1;
                if (used_node_num_ == memory_size_){
                    // ROS_ERROR("Run out of node memory!");
                    ROS_WARN("Run out of node memory!");
                    return -1;
                }
            } else if (next_node->node_state == OPEN){
                if (node_f_cost < next_node->f_cost){
                    // ROS_WARN("Rearrange openset may needed!");
                    next_node->parent = curr_node;
                    next_node->state = next_state;
                    next_node->input = input;
                    next_node->rotation = rotation;
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
        for (int i = 0; i < neighboor_nodes.size(); i++)
        {
            open_set_.push(neighboor_nodes[i]);
            expanded_nodes_.insertNode(neighboor_nodes[i]->p_index,neighboor_nodes[i]->r_index, neighboor_nodes[i]->t_index, neighboor_nodes[i]);
            // std::cout << "expand============================" << std::endl;
            // std::cout << " r : " << resolution_;
            // std::cout <<" x: "<< next_pos(0) <<" y: "<< next_pos(1)<<" z: "<< next_pos(2);
            std::cout <<" p index: "<< neighboor_nodes[i]->p_index(0) <<" y: "<< neighboor_nodes[i]->p_index(1)<<" z: "<< neighboor_nodes[i]->p_index(2);
            std::cout <<" timestamp : "<< neighboor_nodes[i]->timestamp << std::endl;
            std::cout <<" rotation: "<< neighboor_nodes[i]->rotation(0) << std::endl;
            // std::cout <<"  last g:"<< curr_node->g_cost<<" node_g_cost: "<< node_g_cost <<" node_h_cost: "<< node_h_cost << std::endl;
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
    
    // display
    for (int i = 0; i < full_traj_nodes_.size();i++){
        std::cout << i << "  =====================" << std::endl;
        std::cout << " x: " << full_traj_nodes_[i]->state.head(3)(0) << " y: " << full_traj_nodes_[i]->state.head(3)(1) << " z: " << full_traj_nodes_[i]->state.head(3)(2) << std::endl;
        std::cout << " roll: "<< full_traj_nodes_[i]->rotation(0) <<" pitch: "<< full_traj_nodes_[i]->rotation(1) << std::endl;
        std::cout << " input x: "<< full_traj_nodes_[i]->input(0) <<" input y: "<< full_traj_nodes_[i]->input(1) <<" input z: "<< full_traj_nodes_[i]->input(2)<< std::endl;

    }
}

void RotationAstar::getSampleTraj(double sample_rate, std::vector<Eigen::Vector3d> &pos_sample_list,
                                  std::vector<Eigen::Vector3d> &force_sample_list)
{
    if (!full_traj_nodes_.empty()){
        TrajNodePtr node = full_traj_nodes_.back();
        Eigen::Matrix<double, 6, 1> state_i;
        Eigen::Matrix<double, 6, 1> state_f;
        Eigen::Vector3d input;
        Eigen::Vector3d g(0,0,gravity_);
        double duration;

        while (node->parent != NULL){
            state_i = node->parent->state;
            input = node->input;
            duration = node->duration;

            for (double d_tau = duration; d_tau >= -1e-5; d_tau -= duration / sample_rate){
                stateTransition(state_i, state_f, input, d_tau);
                pos_sample_list.push_back(state_f.head(3));
                // std::cout << "input " << input <<std::endl;
                // std::cout << " f" << input + g << std::endl;
                force_sample_list.push_back(input + g);
            }

            node = node->parent;
        }
        reverse(pos_sample_list.begin(), pos_sample_list.end());
        reverse(force_sample_list.begin(), force_sample_list.end());
    }
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
