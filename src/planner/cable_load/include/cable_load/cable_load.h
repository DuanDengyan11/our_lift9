#include<iostream>
using namespace std;
#include <vector>
#include <Eigen/Eigen> 
#include <ros/ros.h>

class cable_load
{
private:

    double length_, height_, width_;
    double grav = 9.8;
    ros::NodeHandle node_;
    
public:
    cable_load(){};
    ~cable_load(){};
    void init(ros::NodeHandle &nh);

    std::vector<Eigen::Vector3d> cable_points;
    Eigen::Matrix<double, 12, 6> G_inv;
    Eigen::MatrixXd G_null_space;
    Eigen::Matrix3d Transformb2e;
    Eigen::VectorXd init_coef;  // alpha = 30 degree beta = 45 degree

    double cable_length_, load_mass_;

    Eigen::Matrix3d cal_skew_matrix(Eigen::Vector3d x);
    
    void compute_FM_each(Eigen::Vector3d acc_load, Eigen::VectorXd pos_coef, Eigen::VectorXd &FMeach);
    void compute_uav_pos(Eigen::VectorXd FMeach, Eigen::Vector3d load_position, std::vector<Eigen::Vector3d> &uav_positions);
    void compute_init_coef();

	typedef std::shared_ptr<cable_load> Ptr;

};
