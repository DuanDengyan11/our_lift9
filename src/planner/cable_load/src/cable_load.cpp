#include <cable_load/cable_load.h>

void cable_load::init(ros::NodeHandle &nh)
{  
    node_ = nh;
    node_.param("optimization/load_length", length_ , 1.0);  //吊挂物长宽高
    node_.param("optimization/load_width", width_ , 0.5);
    node_.param("optimization/load_height", height_ , 0.5);
    node_.param("optimization/load_mass", load_mass_, 2.0); // 吊挂物质量
    node_.param("cable/cable_length", cable_length_, 2.0); // cable length

    double half_length = length_/2.0, half_width = width_/2.0, half_height = height_/2.0;

    cable_points.push_back({-half_length, -half_width, -half_height});
    cable_points.push_back({-half_length, half_width, -half_height});
    cable_points.push_back({half_length, half_width, -half_height});
    cable_points.push_back({half_length, -half_width, -half_height});

    Eigen::Matrix<double, 6, 12> G;
    for (size_t i = 0; i < cable_points.size(); i++)
    {
        G.block<3,3>(0,3*i) << 1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0;
        G.block<3,3>(3,3*i) = cal_skew_matrix(cable_points[i]);
    }
    G_inv = G.transpose() * (G * G.transpose()).inverse();  //不变的量
    Eigen::FullPivLU<Eigen::MatrixXd> lu(G);
    G_null_space = lu.kernel(); //不变量 G的一组基
    
    Transformb2e << 1.0, 0.0, 0.0,
    0.0, -1.0, 0.0,
    0.0, 0.0, -1.0;

    compute_init_coef();
}

void cable_load::compute_init_coef()
{

    Eigen::VectorXd  FMeach(12);
    double alpha = M_PI/6.0, beta = M_PI/4.0, weight = load_mass_ * grav;
    double temp1 = weight/4.0 * tan(alpha)*cos(beta);
    double temp2 = weight/4.0 * tan(alpha)*sin(beta);
    double temp3 = weight/4.0;
    FMeach << -temp1, -temp2, -temp3, -temp1, temp2, -temp3, temp1, temp2, -temp3, temp1, -temp2, -temp3; 

    Eigen::VectorXd acc(3), FM_total(6), FMeach_space;
    acc << 0.0, 0.0, 0.0 - grav;
    FM_total << load_mass_ * acc, 0.0, 0.0, 0.0;
    FMeach_space = FMeach - G_inv * FM_total;

    init_coef = (G_null_space.transpose() * G_null_space).ldlt().solve(G_null_space.transpose() * FMeach_space);
    
    // cout << "FM_each" << G_inv *FM_total + G_null_space * init_coef << endl;
    // cout<< "init_coef" << init_coef.transpose() << endl;
    // Eigen::Vector3d load_position = {0.0, 0.0, 0.0};
    // std::vector<Eigen::Vector3d> uav_positions;
    // compute_uav_pos(FMeach, load_position, uav_positions);
    // cout << "uav_positions" << uav_positions[0].transpose() << endl;
    // cout << "uav_positions" << uav_positions[1].transpose() << endl;
    // cout << "uav_positions" << uav_positions[2].transpose() << endl;
    // cout << "uav_positions" << uav_positions[3].transpose() << endl;
}

void cable_load::compute_FM_each(Eigen::Vector3d acc_load, Eigen::VectorXd pos_coef, Eigen::VectorXd &FMeach)
{
    pos_coef.resize(6);
    FMeach.resize(12);

    Eigen::VectorXd acc(3), FM_total(6);
    acc << acc_load(0), -acc_load(1), -acc_load(2) - grav;
    FM_total << load_mass_ * acc, 0.0, 0.0, 0.0;
    FMeach = G_inv * FM_total + G_null_space * pos_coef;
}

void cable_load::compute_uav_pos(Eigen::VectorXd FMeach, Eigen::Vector3d load_position, std::vector<Eigen::Vector3d> &uav_positions)
{
    uav_positions.clear();
    FMeach.resize(12);
    for (size_t i = 0; i < 4; i++)
    {
      Eigen::Vector3d FMi = FMeach.block<3,1>(3*i,0);
      Eigen::Vector3d qi = FMi / FMi.norm();
      Eigen::Vector3d uav_position = load_position + Transformb2e *(cable_points[i] + cable_length_ * qi);
      uav_positions.push_back(uav_position);
    }
}

Eigen::Matrix3d cable_load::cal_skew_matrix(Eigen::Vector3d x)
{
    Eigen::Matrix3d x_hat;
    x_hat << 0, -x(2), x(1),
    x(2), 0, -x(0),
    -x(1), x(0), 0; 
    return x_hat;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ego_planner_node");
    ros::NodeHandle nh;
    cable_load cable_load_;
    cable_load_.init(nh);
}