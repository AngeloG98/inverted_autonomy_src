#pragma once

#include <ros/ros.h>
#include <ros/duration.h>
#include <Eigen/Eigen>
#include <map>
#include <unordered_map>
#include <queue>
#include <plan_env/gridmap.h>

namespace inverted_planner
{

#define UNEXPAND 'u'
#define OPEN 'o'
#define CLOSED 'c'

struct TrajNode
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Matrix<double, 6, 1> state; // current position and velocity
    Eigen::Vector3d input; // input from parent node, which is current acceleration(rotation).
    Eigen::Vector3d rotation;
    double duration;
    double timestamp;

    Eigen::Vector3i p_index;
    int t_index;
    double g_cost;
    double h_cost;
    // double a_cost;
    double f_cost;

    TrajNode *parent;
    char node_state;

    TrajNode(){
        parent = NULL;
        node_state = UNEXPAND;
    }
     ~TrajNode(){}
};
typedef TrajNode* TrajNodePtr;

class NodeComparator {
public:
    bool operator()(TrajNodePtr node1, TrajNodePtr node2) {
        return node1->f_cost > node2->f_cost;
    }
};

template <typename T>
struct matrix_hash : std::unary_function<T, size_t> {
    std::size_t operator()(T const& matrix) const {
        size_t seed = 0;
        for (size_t i = 0; i < matrix.size(); ++i) {
            auto elem = *(matrix.data() + i);
            seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};

class TrajNodeHashTable {
private:
    std::unordered_map<Eigen::Vector4i, TrajNodePtr, matrix_hash<Eigen::Vector4i>> trajnode_data_;

public:
    TrajNodeHashTable(){}
    ~TrajNodeHashTable(){}

    void clearMap(){
        trajnode_data_.clear();
    }

    void insertNode(Eigen::Vector3i p_index, int t_index, TrajNodePtr node){
        Eigen::Vector4i pt_index(p_index(0), p_index(1), p_index(2), t_index);
        trajnode_data_.insert(std::make_pair(pt_index, node));
    }

    TrajNodePtr findNode(Eigen::Vector3i p_index, int t_index){
        Eigen::Vector4i pt_index(p_index(0), p_index(1), p_index(2), t_index);
        auto iter = trajnode_data_.find(pt_index);
        return iter == trajnode_data_.end() ? NULL : iter->second;
    }

    void clear() { trajnode_data_.clear(); }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class RotationAstar{
private:
    // data 
    std::vector<TrajNodePtr> traj_nodes_mem_;
    std::vector<TrajNodePtr> full_traj_nodes_;
    std::priority_queue<TrajNodePtr, std::vector<TrajNodePtr>, NodeComparator> open_set_;
    TrajNodeHashTable expanded_nodes_;

    int used_node_num_;
    int step_num_;

    Eigen::Matrix<double, 6, 6> transition_;

    // searching parameters
    double tau_;
    double max_vel_, max_acc_;
    double rho_t_, horizon_, lambda_h_;
    int memory_size_, collision_check_;

    // map data
    std::shared_ptr<inverted_planner::GridMap> gridmap_;
    Eigen::Vector3d map_origin_, map_size_;
    double resolution_, inv_resolution_;
    double time_origin_;
    double time_resolution_, inv_time_resolution_;

    // get node index
    Eigen::Vector3i posToIndex(Eigen::Vector3d pos);
    int timeToIndex(double time);

    // get rotation aware input
    void getRotationInput(TrajNodePtr curr_node, std::vector<Eigen::Vector3d> &input_list, std::vector<Eigen::Vector3d> &rot_list);
    // get trajectory -> full_traj_nodes_
    void getFullTraj(TrajNodePtr end_node);

    // state transition
    void stateTransition(Eigen::Matrix<double, 6, 1> state_i,
                         Eigen::Matrix<double, 6, 1> &state_f,
                         Eigen::Vector3d input, double tau);

    // get heuristic
    double getHeuristicCost(Eigen::Matrix<double, 6, 1> state_i, Eigen::Matrix<double, 6, 1> state_f);

public:
    RotationAstar();
    ~RotationAstar();

    void init();
    void reset();
    int search(Eigen::Vector3d start_pos, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
               Eigen::Vector3d end_pos, Eigen::Vector3d end_vel, double start_time);

    void setGridMap(const std::shared_ptr<inverted_planner::GridMap> &gridmap);

    // get trajectory
    std::vector<Eigen::Vector3d> getSampleTraj(double sample_rate);
    std::vector<Eigen::Vector3d> getExpandedPoint();

    // enum
    // {
    //     REACH_HORIZON = 1,
    //     REACH_END = 2,
    //     NO_PATH = 3,
    //     NEAR_END = 4
    // };

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace inverted_planner