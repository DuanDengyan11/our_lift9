#include "optimizer/poly_traj_optimizer.h"
// using namespace std;

namespace ego_planner
{
  /* main planning API */
  bool PolyTrajOptimizer::OptimizeTrajectory_lbfgs(
      const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
      const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT,
      Eigen::MatrixXd &optimal_points, double trajectory_start_time)
  {
    //need to remove
    // cps_num_prePiece_ = 5;
    // wei_time_ = 80;
    // max_vel_ = 5;
    // wei_feas_ = 10000;
    // wei_sqrvar_ = 10000;
    // max_acc_ = 10;

    if (initInnerPts.cols() != (initT.size() - 1))
    {
      ROS_ERROR("initInnerPts.cols() != (initT.size()-1)");
      return false;
    }

    t_now_ = trajectory_start_time;
    piece_num_ = initT.size();

    cout << "piece_num" << piece_num_ << endl;

    jerkOpt_.reset(iniState, finState, piece_num_);

    double final_cost;

    variable_num_ = dim * (piece_num_ - 1) + piece_num_; //内部点坐标+时间坐标
    ros::Time t1, t2;

    double q[variable_num_];
    memcpy(q, initInnerPts.data(), initInnerPts.size() * sizeof(q[0]));
    Eigen::Map<Eigen::VectorXd> Vt(q + initInnerPts.size(), initT.size());
    RealT2VirtualT(initT, Vt);

    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
   
    lbfgs_params.mem_size = mem_size_;
    lbfgs_params.g_epsilon = g_epsilon_;
    lbfgs_params.past = past_;
    lbfgs_params.delta = delta_;
    lbfgs_params.max_iterations = max_iterations_;
    lbfgs_params.max_linesearch = max_linesearch_;
    lbfgs_params.min_step = min_step_;
    lbfgs_params.max_step = max_step_;
    lbfgs_params.f_dec_coeff = f_dec_coeff_;
    lbfgs_params.s_curv_coeff = s_curv_coeff_;

    iter_num_ = 0;
    force_stop_type_ = DONT_STOP;

    results_step.clear();

    /* ---------- optimize ---------- */
    t1 = ros::Time::now();

    int result = lbfgs::lbfgs_optimize(
        variable_num_,
        q,
        &final_cost,
        PolyTrajOptimizer::costFunctionCallback_forLoad,
        NULL,
        PolyTrajOptimizer::earlyExitCallback,
        this,
        &lbfgs_params);

    results_total.push_back(results_step);

    optimal_points = cps_.points;

    // test collision
    bool occ = false;
    occ = checkCollision();

    t2 = ros::Time::now();
    double time_ms = (t2 - t1).toSec() * 1000;

    printf("\033[32miter=%d, time(ms)=%5.3f, \n\033[0m", iter_num_, time_ms);
    // ROS_WARN("The optimization result is : %s", lbfgs::lbfgs_strerror(result));

    if (occ)
      return false;
    else{
      for (size_t i = 0; i < results_step.size(); i++)
      {
        for (size_t j = 0; j < 11; j++)
        {
          result_file_ << results_step[i](j) << '\t';
        }
        
          result_file_ << time_ms << '\n';
      }
      return true;
    }


      // result_file_ << result(1) << '\t' << smoo_cost << '\t' << time_cost << '\t';
      // result_file_ << obs_swarm_feas_qvar_costs(0) << '\t' << obs_swarm_feas_qvar_costs(1) << '\t' << obs_swarm_feas_qvar_costs(2) << '\t' << obs_swarm_feas_qvar_costs(3) << '\t';
      // result_file_ << obs_swarm_feas_qvar_costs(4) << '\t' << obs_swarm_feas_qvar_costs(5) << '\t' << obs_swarm_feas_qvar_costs(6) << '\t';
      // result_file_ << smoo_cost + time_cost + obs_swarm_feas_qvar_costs.sum() << '\n';

  }

  bool PolyTrajOptimizer::checkCollision(void)
  {
    /* check the safety of trajectory */
    double T_end;
    poly_traj::Trajectory traj = jerkOpt_.getTraj();
    
    int N = traj.getPieceNum();
    int k = cps_num_prePiece_ * N + 1;
    int idx = k / 3 * 2; //取前面2/3
    int piece_of_idx = floor((idx - 1) / cps_num_prePiece_);
    Eigen::VectorXd durations = traj.getDurations();
    T_end = durations.head(piece_of_idx).sum() 
            + durations(piece_of_idx) * (idx - piece_of_idx * cps_num_prePiece_) / cps_num_prePiece_;

    bool occ_load = false;
    double dt = 0.01;
    int i_end = floor(T_end/dt);
    double t = 0.0;
    collision_check_time_end_ = T_end;

    for (int i=0; i<i_end; i++){
      Eigen::Matrix<double, dim, 1> pos = traj.getPos(t);
      Eigen::Matrix<double, dim, 1> vel = traj.getVel(t);
      Eigen::Matrix<double, dim, 1> acc = traj.getAcc(t);
      Eigen::Vector3d pos_load = pos.topRows(3);
      Eigen::Vector3d acc_load = acc.topRows(3);
      Eigen::Matrix<double, 6, 1> cable_coef = pos.bottomRows(6);

      Eigen::VectorXd pos_cable_coef(6), vel_cable_coef(6), FMeach(12);
      std::vector<Eigen::Vector3d> uav_positions;
      pos_cable_coef = pos.bottomRows(6);
      vel_cable_coef = vel.bottomRows(6);
      cable_load_->compute_FM_each(acc_load, pos_cable_coef, FMeach); 
      cable_load_->compute_uav_pos(FMeach, pos_load, uav_positions);

      // load obstacle

      if(grid_map_->getInflateOccupancy(pos_load) == 1 )
      { //检查load uav是否和地图中的障碍物碰撞
        occ_load = true;
        break;
      }

      if(grid_map_->getInflateOccupancy(uav_positions[0]) == 1 ||
      grid_map_->getInflateOccupancy(uav_positions[1]) == 1 ||
      grid_map_->getInflateOccupancy(uav_positions[2]) == 1 ||
      grid_map_->getInflateOccupancy(uav_positions[3]) == 1)
      { //检查load uav是否和地图中的障碍物碰撞
        occ_load = true;
        break;
      }

      int num = 5;
      double step = cable_length_ / num;
      for (size_t i = 0; i < 4; i++)
      {
        Eigen::Vector3d FMi = FMeach.block<3,1>(3*i,0);
        Eigen::Vector3d qi = FMi / FMi.norm();
        for (int j = 1; j < (num-1); j++)
        {
          double dist_j = step * j;
          Eigen::Vector3d cable_point = pos_load + Transformb2e * ( cable_load_->cable_points[i] + dist_j * qi);
          if(grid_map_->getInflateOccupancy(cable_point) == 1) // check cable obstacle
          {
            occ_load = true;
            break;
          }
        }
      }

      t += dt;
    }
    return occ_load;
  }


  bool PolyTrajOptimizer::addCollisionForCable(const int i_dp, Eigen::VectorXd FMeach, Eigen::VectorXd &grad0, double &cost, Eigen::VectorXd &grad1, Eigen::Vector3d pos_load, Eigen::VectorXd &grad2)
  {

    if (i_dp == 0 || i_dp >= cps_.cp_size * 2 / 3)
      return false;

    bool flag = false;
    cost = 0; 
    grad0.setZero();
    grad1.setZero();
    grad2.setZero();

    Eigen::MatrixXd space = cable_load_->G_null_space;
    Eigen::MatrixXd G_inv = cable_load_->G_inv;
    double load_mass = cable_load_->load_mass_;
    
    double dist, dist_err;
    for (size_t i = 0; i < 4; i++)
    {
      Eigen::Vector3d FMi = FMeach.block<3,1>(3*i,0);
      Eigen::Vector3d qi = FMi / FMi.norm();
      Eigen::MatrixXd Gi = space.block<3,6>(3*i,0);
      Eigen::MatrixXd G_invi = G_inv.block<3,3>(3*i,0);

      double step = cable_length_ / cable_piece_;
      double cost_temp;
      for (int j = 1; j < cable_piece_; j++)
      {
        double dist_j = step * j;
        Eigen::Vector3d cable_point = pos_load + Transformb2e * ( cable_load_->cable_points[i] + dist_j * qi);
        grid_map_->evaluateEDT(cable_point, dist);
        dist_err = cable_clearance_ - dist;
        if(dist_err > 0)
        {
          flag = true;
          cost_temp = weight_cable_collision_ * pow(dist_err, 3);
          Eigen::Vector3d dist_grad;
          grid_map_->evaluateFirstGrad(cable_point, dist_grad);

          grad0 += weight_cable_collision_ * 3 * pow(dist_err,2) * (-1) * Gi.transpose() * DN(FMi).transpose() * (dist_j*Transformb2e).transpose() * dist_grad;
          grad1 += weight_cable_collision_ * 3 * pow(dist_err,2) * (-1) * dist_grad;
          grad2 += weight_cable_collision_ * 3 * pow(dist_err,2) * (G_invi * Transformb2e.transpose() * load_mass).transpose() * DN(FMi).transpose() * (dist_j*Transformb2e).transpose() * (-1) * dist_grad;

          cost += cost_temp; 
        }
      }
    }
    return flag;
  }


  bool PolyTrajOptimizer::addSwarmForUAV(const int i_dp, Eigen::VectorXd FMeach, std::vector<Eigen::Vector3d> uav_positions, Eigen::VectorXd &grad0, double &cost, Eigen::VectorXd &grad1)
  {

    if (i_dp == 0 || i_dp >= cps_.cp_size * 2 / 3)
      return false;

    bool flag = false;
    cost = 0;
    grad0.setZero();
    grad1.setZero();
    
    Eigen::MatrixXd space = cable_load_->G_null_space;
    Eigen::MatrixXd G_inv = cable_load_->G_inv;
    double load_mass = cable_load_->load_mass_;

    for (size_t i = 0; i < 4; i++)
    {
      Eigen::Vector3d FMi = FMeach.block<3,1>(3*i,0);
      Eigen::Vector3d uav_position_i = uav_positions[i];
      Eigen::MatrixXd Gi = space.block<3,6>(3*i,0);
      Eigen::MatrixXd G_invi = G_inv.block<3,3>(3*i,0);
      
      for (size_t j = i+1; j < 4; j++)
      {
        Eigen::Vector3d FMj = FMeach.block<3,1>(3*j,0);
        Eigen::Vector3d uav_position_j = uav_positions[j];
        Eigen::MatrixXd Gj = space.block<3,6>(3*j,0);
        Eigen::MatrixXd G_invj = G_inv.block<3,3>(3*j,0);

        Eigen::Vector3d dist_vector = uav_position_j - uav_position_i;
        double dist = dist_vector.norm();
        double dist_err = pow(uav_swarm_clearance_,2) - pow(dist,2);
        if (dist_err > 0)
        {
          flag = true;

          cost += weight_uav_swarm_ * pow(dist_err,3);
          
          grad0 += weight_uav_swarm_ * 3 * pow(dist_err, 2) * ( Gj.transpose() * DN(FMj).transpose() - Gi.transpose() * DN(FMi).transpose() ) * (cable_length_*Transformb2e).transpose() * (-2) * dist_vector;

          grad1 += weight_uav_swarm_ * 3 * pow(dist_err, 2)  * (
            (G_invj * Transformb2e.transpose() * load_mass).transpose() * DN(FMj).transpose() 
            - (G_invi * Transformb2e.transpose() * load_mass).transpose() * DN(FMi).transpose()
            ) * (cable_length_*Transformb2e).transpose() * (-2) * dist_vector;
        }
      }
    }
    return flag;
  }

  bool PolyTrajOptimizer::addCollisionForUAV(const int i_dp, Eigen::VectorXd FMeach, std::vector<Eigen::Vector3d> uav_positions, Eigen::VectorXd &grad0, double &cost, Eigen::VectorXd &grad1, Eigen::VectorXd &grad2)
  {

    if (i_dp == 0 || i_dp >= cps_.cp_size * 2 / 3)
      return false;

    bool flag = false;
    cost = 0; 
    grad0.setZero();
    grad1.setZero();
    grad2.setZero();

    Eigen::MatrixXd space = cable_load_->G_null_space;
    Eigen::MatrixXd G_inv = cable_load_->G_inv;
    double load_mass = cable_load_->load_mass_;
    
    double dist, dist_err;
    for (size_t i = 0; i < 4; i++)
    {
      Eigen::Vector3d FMi = FMeach.block<3,1>(3*i,0);
      Eigen::Vector3d uav_position = uav_positions[i];
      grid_map_->evaluateEDT(uav_position, dist);
      dist_err = uav_obs_clearance_ - dist;
      if(dist_err>0)
      {
        flag = true;
        cost += weight_uav_obs_ * pow(dist_err,3);
  
        Eigen::Vector3d dist_grad;
        grid_map_->evaluateFirstGrad(uav_position, dist_grad);
        Eigen::MatrixXd Gi = space.block<3,6>(3*i,0);
        Eigen::MatrixXd G_invi = G_inv.block<3,3>(3*i,0);

        // grad0
        grad0 += weight_uav_obs_ * 3 * pow(dist_err,2) * Gi.transpose() * DN(FMi).transpose() * (cable_length_*Transformb2e).transpose() * (-1) * dist_grad;
        // grad1 
        grad1 += weight_uav_obs_ * 3 * pow(dist_err,2) * (-1) * dist_grad;
        // grad2
        grad2 += weight_uav_obs_ * 3 * pow(dist_err,2) * (G_invi * Transformb2e.transpose() * load_mass).transpose() * DN(FMi).transpose() *  (cable_length_*Transformb2e).transpose() * (-1) * dist_grad;
      }
    }
    return flag;
  }

  bool PolyTrajOptimizer::addFeasibilityForCable(Eigen::MatrixXd FMeach, Eigen::VectorXd &grad0, double &cost, Eigen::VectorXd &grad1)
  {
    bool flag = false;
    double cost_min = 0, cost_max = 0; 
    
    cost = 0;
    grad0.setZero();
    grad1.setZero();

    Eigen::VectorXd grad1_1, grad1_2, grad1_3;
    grad1_1.resize(1); grad1_1.setZero();
    grad1_2.resize(1); grad1_2.setZero();
    grad1_3.resize(1); grad1_3.setZero();

    Eigen::MatrixXd space = cable_load_->G_null_space;
    Eigen::MatrixXd G_inv = cable_load_->G_inv;
    double load_mass = cable_load_->load_mass_;
    
    Eigen::VectorXd Tmin, Tmax;
    Tmin.resize(12);
    Tmax.resize(12);
    Tmin << -1, -1,    -2, -1, 0,    -2,  0, 0,    -2,  0, -1,    -2; 
    Tmax <<  0,  0, -0.17,  0, 1, -0.17,  1, 1, -0.17,  1,  0, -0.17;
    Tmin = Tmin * load_mass * grav;
    Tmax = Tmax * load_mass * grav;

    for(int i = 0; i < 12; i++) 
    {
      int j = i%3;
      if(FMeach(i) < Tmin(i))
      {
        cost_min += weight_FM_feasibility_ * pow((Tmin(i) - FMeach(i)),2);
        grad0 += (space.block<1,6>(i,0)).transpose() * (-2) * weight_FM_feasibility_ * (Tmin(i) - FMeach(i));

        if(j==0)
        {
          grad1_1 += (G_inv.block<1,3>(i,0) * Transformb2e.transpose().block<3,1>(0,j) * load_mass).transpose() * (-2) * weight_FM_feasibility_ * (Tmin(i) - FMeach(i));
        }

        if(j==1)
        {
          grad1_2 += (G_inv.block<1,3>(i,0) * Transformb2e.transpose().block<3,1>(0,j) * load_mass).transpose() * (-2) * weight_FM_feasibility_ * (Tmin(i) - FMeach(i));
        }

        if(j==2)
        {
          grad1_3 += (G_inv.block<1,3>(i,0) * Transformb2e.transpose().block<3,1>(0,j) * load_mass).transpose() * (-2) * weight_FM_feasibility_ * (Tmin(i) - FMeach(i));
        }
        flag = true;
      }

      if(FMeach(i) > Tmax(i))
      {
        cost_max += weight_FM_feasibility_ * pow((FMeach(i) - Tmax(i)), 2);
        grad0 += (space.block<1,6>(i,0)).transpose() * 2 * weight_FM_feasibility_ * (FMeach(i) - Tmax(i));

        if(j==0)
        {
          grad1_1 += (G_inv.block<1,3>(i,0) * Transformb2e.transpose().block<3,1>(0,j) * load_mass).transpose() * 2 * weight_FM_feasibility_ * (FMeach(i) - Tmax(i));
        }

        if(j==1)
        {
          grad1_2 += (G_inv.block<1,3>(i,0) * Transformb2e.transpose().block<3,1>(0,j) * load_mass).transpose() * 2 * weight_FM_feasibility_ * (FMeach(i) - Tmax(i));
        }

        if(j==2)
        {
          grad1_3 += (G_inv.block<1,3>(i,0) * Transformb2e.transpose().block<3,1>(0,j) * load_mass).transpose() * 2 * weight_FM_feasibility_ * (FMeach(i) - Tmax(i));
        }
        flag = true;
      }
    }
    
    grad1 << grad1_1, grad1_2, grad1_3;
    cost = cost_min + cost_max;

    // double mg_first_six = - load_mass * grav / 6.0;
    // // FMeach(0) < 0, FMeach(1) < 0, FMeach(2) < 0
    // if(FMeach(0) >= 0)
    // {
    //   cost += pow(FMeach(0),2);
    //   grad += 2 * FMeach(0) * space.row(0).transpose();
    // }
    // if(FMeach(1) >= 0)
    // {
    //   cost += pow(FMeach(1),2);
    //   grad += 2 * FMeach(1) * space.row(1).transpose();
    // }
    // if((FMeach(2) - Tmin(2)) >= 0)
    // {
    //   cost += pow((FMeach(2) - mg_first_six),2);
    //   grad += 2 * (FMeach(2) - mg_first_six) * space.row(2).transpose();
    // }

    // // FMeach(3) < 0, FMeach(4) > 0, FMeach(5) < 0
    // if(FMeach(3) >= 0)
    // {
    //   cost += pow(FMeach(3),2);
    //   grad += 2 * FMeach(3) * space.row(3).transpose();
    // }
    // if(FMeach(4) <= 0)
    // {
    //   cost += pow(FMeach(4),2);
    //   grad += 2 * FMeach(4) * space.row(4).transpose();
    // }
    // if((FMeach(5) - mg_first_six) >= 0)
    // {
    //   cost += pow((FMeach(5)-mg_first_six),2);
    //   grad += 2 * (FMeach(5) - mg_first_six) * space.row(5).transpose();
    // }

    // // FMeach(6) > 0, FMeach(7) > 0, FMeach(8) < 0
    // if(FMeach(6) <= 0)
    // {
    //   cost += pow(FMeach(6),2);
    //   grad += 2 * FMeach(6) * space.row(6).transpose();
    // }
    // if(FMeach(7) <= 0)
    // {
    //   cost += pow(FMeach(7),2);
    //   grad += 2 * FMeach(7) * space.row(7).transpose();
    // }
    // if((FMeach(8) - mg_first_six) >= 0)
    // {
    //   cost += pow((FMeach(8) - mg_first_six),2);
    //   grad += 2 * (FMeach(8) - mg_first_six) * space.row(8).transpose();
    // }

    // // FMeach(9) > 0, FMeach(10) < 0, FMeach(11) < 0
    // if(FMeach(9) <= 0)
    // {
    //   cost += pow(FMeach(9),2);
    //   grad += 2 * FMeach(9) * space.row(9).transpose();
    // }
    // if(FMeach(10) >= 0)
    // {
    //   cost += pow(FMeach(10),2);
    //   grad += 2 * FMeach(10) * space.row(10).transpose();
    // }
    // if((FMeach(11) - mg_first_six) >= 0)
    // {
    //   cost += pow((FMeach(11)-mg_first_six),2);
    //   grad += 2 * (FMeach(11)-mg_first_six) * space.row(11).transpose();
    // }

    // cost = cost * weight_FM_feasibility_;  //乘上系数
    // grad = grad * weight_FM_feasibility_;

    // cost += pow(FMeach.norm(),2) * weight_FM_norm_ ;
    // grad += 2 * space.transpose() * FMeach * weight_FM_norm_;
    return flag;
  }

  // /* callbacks by the L-BFGS optimizer */
  double PolyTrajOptimizer::costFunctionCallback_forLoad(void *func_data, const double *x, double *grad, const int n)
   {
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);
    opt->min_ellip_dist2_ = std::numeric_limits<double>::max();

    Eigen::Map<const Eigen::MatrixXd> P(x, dim, opt->piece_num_ - 1);
    // Eigen::VectorXd T(Eigen::VectorXd::Constant(piece_nums, opt->t2T(x[n - 1]))); // same t
    Eigen::Map<const Eigen::VectorXd> t(x + (dim * (opt->piece_num_ - 1)), opt->piece_num_);
    Eigen::Map<Eigen::MatrixXd> gradP(grad, dim, opt->piece_num_ - 1);
    Eigen::Map<Eigen::VectorXd> gradt(grad + (dim * (opt->piece_num_ - 1)), opt->piece_num_);
    Eigen::VectorXd T(opt->piece_num_);

    opt->VirtualT2RealT(t, T);

    Eigen::VectorXd gradT(opt->piece_num_);
    double smoo_cost = 0, time_cost = 0;
    Eigen::VectorXd obs_swarm_feas_qvar_costs(7);

    opt->jerkOpt_.generate(P, T);

    opt->initAndGetSmoothnessGradCost2PT(gradT, smoo_cost); // Smoothness cost

    opt->addPVAGradCost2CT(gradT, obs_swarm_feas_qvar_costs, opt->cps_num_prePiece_); // Time int cost

    opt->jerkOpt_.getGrad2TP(gradT, gradP);

    opt->VirtualTGradCost(T, t, gradT, gradt, time_cost);

    opt->iter_num_ += 1;

    Eigen::VectorXd result;
    result.resize(11);
    result(0) = opt->iter_num_ ;
    result(1) = smoo_cost;
    result(2) = time_cost;
    result.block<7,1>(3,0) = obs_swarm_feas_qvar_costs;
    result(10) = smoo_cost + time_cost + obs_swarm_feas_qvar_costs.sum();
    opt->results_step.push_back(result);

    return smoo_cost + time_cost + obs_swarm_feas_qvar_costs.sum();
  return 1;
  }

  int PolyTrajOptimizer::earlyExitCallback(void *func_data, const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls)
  {
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);
    return (opt->force_stop_type_ == STOP_FOR_ERROR || opt->force_stop_type_ == STOP_FOR_REBOUND);
  }

  // /* mappings between real world time and unconstrained virtual time */
  template <typename EIGENVEC>
  void PolyTrajOptimizer::RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT)
  {
    for (int i = 0; i < RT.size(); ++i)
    {
      VT(i) = RT(i) > 1.0 ? (sqrt(2.0 * RT(i) - 1.0) - 1.0)
                          : (1.0 - sqrt(2.0 / RT(i) - 1.0));
    }
  }

  template <typename EIGENVEC>
  void PolyTrajOptimizer::VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT)
  {
    for (int i = 0; i < VT.size(); ++i)
    {
      RT(i) = VT(i) > 0.0 ? ((0.5 * VT(i) + 1.0) * VT(i) + 1.0)
                          : 1.0 / ((0.5 * VT(i) - 1.0) * VT(i) + 1.0);
    }
  }

  template <typename EIGENVEC, typename EIGENVECGD>
  void PolyTrajOptimizer::VirtualTGradCost(
      const Eigen::VectorXd &RT, const EIGENVEC &VT,
      const Eigen::VectorXd &gdRT, EIGENVECGD &gdVT,
      double &costT)
  {
    for (int i = 0; i < VT.size(); ++i)
    {
      double gdVT2Rt;
      if (VT(i) > 0)
      {
        gdVT2Rt = VT(i) + 1.0;
      }
      else
      {
        double denSqrt = (0.5 * VT(i) - 1.0) * VT(i) + 1.0;
        gdVT2Rt = (1.0 - VT(i)) / (denSqrt * denSqrt);
      }

      gdVT(i) = (gdRT(i) + wei_time_) * gdVT2Rt;
    }

    costT = RT.sum() * wei_time_;
  }

  // /* gradient and cost evaluation functions */
  template <typename EIGENVEC>
  void PolyTrajOptimizer::initAndGetSmoothnessGradCost2PT(EIGENVEC &gdT, double &cost)
  {
    jerkOpt_.initGradCost(gdT, cost, wei_energy_);
  }

  template <typename EIGENVEC>
  void PolyTrajOptimizer::addPVAGradCost2CT(EIGENVEC &gdT, Eigen::VectorXd &costs, const int &K)
  {
    int N = gdT.size();
    Eigen::Matrix<double, dim, 1> pos, vel, acc, jer;
    Eigen::Matrix<double, dim, 1> gradp, gradv, grada;
    double costp, costv, costa;
    Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3;
    double s1, s2, s3, s4, s5;
    double step, alpha;
    Eigen::Matrix<double, 6, dim> gradViolaPc, gradViolaVc, gradViolaAc;
    double gradViolaPt, gradViolaVt, gradViolaAt;
    double omg;
    int i_dp = 0;
    costs.setZero();
    double t = 0; //global time
    // Eigen::MatrixXd constrain_pts(3, N * K + 1);

    //load
    Eigen::Vector3d pos_load, vel_load, acc_load, jer_load;
    Eigen::Vector3d gradp_load, gradv_load, grada_load;
    double costp_load, costv_load, costa_load;

    //cable coef
    Eigen::VectorXd pos_cable_coef(6), vel_cable_coef(6), FMeach(12);
    std::vector<Eigen::Vector3d> uav_positions;
    Eigen::VectorXd gradp_uavSwarm_coef(6), gradp_uavObstacle_coef(6), gradp_cableObstacle_coef(6), gradp_cableObstacle_loadpos(3), gradp_uavObstacle_loadpos(3), gradp_cableFeasibility_coef(6), gradp_uavObstacle_loadacc(3), gradp_uavSwarm_loadacc(3), gradp_cableObstacle_loadacc(3), gradp_cableFeasibility_loadacc(3);
    double costp_uavSwarm_coef, costp_uavObstacle_coef, costp_cableFeasibility_coef,costp_cableObstacle_coef;

    // int innerLoop;
    for (int i = 0; i < N; ++i)
    {
      const Eigen::Matrix<double, 6, dim> &c = jerkOpt_.get_b().block<6, dim>(i * 6, 0);
      step = jerkOpt_.get_T1()(i) / K;
      s1 = 0.0; //local time
      // innerLoop = K;

      for (int j = 0; j <= K; ++j)
      {
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s4 * s1;
        beta0 << 1.0, s1, s2, s3, s4, s5;
        beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
        beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
        beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
        alpha = 1.0 / K * j;
        pos = c.transpose() * beta0;
        vel = c.transpose() * beta1;
        acc = c.transpose() * beta2;
        jer = c.transpose() * beta3;

        omg = (j == 0 || j == K) ? 0.5 : 1.0;
        cps_.points.col(i_dp) = pos;

        // for load
        pos_load = pos.topRows(3);
        vel_load = vel.topRows(3);
        acc_load = acc.topRows(3);
        jer_load = jer.topRows(3);
        // collision
        if (obstacleGradCostP(i_dp, pos_load, gradp_load, costp_load))
        {
          Eigen::Matrix<double, 6, 3> gradViolaPc_load = beta0 * gradp_load.transpose();
          double gradViolaPt_load = alpha * gradp_load.transpose() * vel_load;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaPc_load;
          gdT(i) += omg * (costp_load / K + step * gradViolaPt_load);
          costs(0) += omg * step * costp_load;
        }
        // feasibility
        if (feasibilityGradCostV(vel_load, gradv_load, costv_load))
        {
          Eigen::Matrix<double, 6, 3> gradViolaVc_load = beta1 * gradv_load.transpose();
          double gradViolaVt_load = alpha * gradv_load.transpose() * acc_load;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaVc_load;
          gdT(i) += omg * (costv_load / K + step * gradViolaVt_load);
          costs(1) += omg * step * costv_load;
        }
        if (feasibilityGradCostA(acc_load, grada_load, costa_load))
        {
          Eigen::Matrix<double, 6, 3> gradViolaAc_load = beta2 * grada_load.transpose();
          double gradViolaAt_load = alpha * grada_load.transpose() * jer_load;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaAc_load;
          gdT(i) += omg * (costa_load / K + step * gradViolaAt_load);
          costs(1) += omg * step * costa_load;
        }

        // //cable coef
        pos_cable_coef = pos.bottomRows(6);
        vel_cable_coef = vel.bottomRows(6);
        cable_load_->compute_FM_each(acc_load, pos_cable_coef, FMeach); 
        cable_load_->compute_uav_pos(FMeach, pos_load, uav_positions);
        // add uav abstacle
        if(addCollisionForUAV(i_dp, FMeach, uav_positions, gradp_uavObstacle_coef, costp_uavObstacle_coef, gradp_uavObstacle_loadpos, gradp_uavObstacle_loadacc))
        {
          Eigen::Matrix<double, 6, 6> gradViolaPc_cable_coef = beta0 * gradp_uavObstacle_coef.transpose();
          double gradViolaPt_cable_coef = alpha * gradp_uavObstacle_coef.transpose() * vel_cable_coef;
          jerkOpt_.get_gdC().block<6, 6>(i * 6, 3) += omg * step * gradViolaPc_cable_coef;

          Eigen::Matrix<double, 6, 3> gradViolaPc_load_pos = beta0 * gradp_uavObstacle_loadpos.transpose();
          double gradViolaPt_load_pos = alpha * gradp_uavObstacle_loadpos.transpose() * vel_load;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaPc_load_pos;

          Eigen::Matrix<double, 6, 3> gradViolaPc_load_acc = beta2 * gradp_uavObstacle_loadacc.transpose();
          double gradViolaPt_load_acc = alpha * gradp_uavObstacle_loadacc.transpose() * jer_load;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaPc_load_acc;
          gdT(i) += omg * (costp_uavObstacle_coef / K + step * gradViolaPt_cable_coef + step * gradViolaPt_load_pos + step * gradViolaPt_load_acc);
          costs(3) += omg * step * costp_uavObstacle_coef;
        }
        //uav swarm
        if(addSwarmForUAV(i_dp, FMeach, uav_positions, gradp_uavSwarm_coef, costp_uavSwarm_coef, gradp_uavSwarm_loadacc))
        {
          Eigen::Matrix<double, 6, 6> gradViolaPc_cable_coef = beta0 * gradp_uavSwarm_coef.transpose();
          double gradViolaPt_cable_coef = alpha * gradp_uavSwarm_coef.transpose() * vel_cable_coef;
          jerkOpt_.get_gdC().block<6, 6>(i * 6, 3) += omg * step * gradViolaPc_cable_coef;

          Eigen::Matrix<double, 6, 3> gradViolaPc_load_acc = beta2 * gradp_uavSwarm_loadacc.transpose();
          double gradViolaPt_load_acc = alpha * gradp_uavSwarm_loadacc.transpose() * jer_load;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaPc_load_acc;
          gdT(i) += omg * (costp_uavSwarm_coef / K + step * gradViolaPt_cable_coef + step * gradViolaPt_load_acc);
          costs(2) += omg * step * costp_uavSwarm_coef;
        }
        // add cable collision
        if(addCollisionForCable(i_dp, FMeach, gradp_cableObstacle_coef, costp_cableObstacle_coef, gradp_cableObstacle_loadpos, pos_load, gradp_cableObstacle_loadacc))
        {
          Eigen::Matrix<double, 6, 6> gradViolaPc_cable_coef = beta0 * gradp_cableObstacle_coef.transpose();
          double gradViolaPt_cable_coef = alpha * gradp_cableObstacle_coef.transpose() * vel_cable_coef;
          jerkOpt_.get_gdC().block<6, 6>(i * 6, 3) += omg * step * gradViolaPc_cable_coef;

          Eigen::Matrix<double, 6, 3> gradViolaPc_load_pos = beta0 * gradp_cableObstacle_loadpos.transpose();
          double gradViolaPt_load_pos = alpha * gradp_cableObstacle_loadpos.transpose() * vel_load;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaPc_load_pos;

          Eigen::Matrix<double, 6, 3> gradViolaPc_load_acc = beta2 * gradp_cableObstacle_loadacc.transpose();
          double gradViolaPt_load_acc = alpha * gradp_cableObstacle_loadacc.transpose() * jer_load;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaPc_load_acc;
          
          gdT(i) += omg * (costp_cableObstacle_coef / K + step * gradViolaPt_cable_coef + step * gradViolaPt_load_pos + step * gradViolaPt_load_acc);
          costs(4) += omg * step * costp_cableObstacle_coef;
        }
        // add cable force feasibility
        if(addFeasibilityForCable(FMeach, gradp_cableFeasibility_coef, costp_cableFeasibility_coef, gradp_cableFeasibility_loadacc))
        {
          Eigen::Matrix<double, 6, 6> gradViolaPc_cable_coef = beta0 * gradp_cableFeasibility_coef.transpose();
          double gradViolaPt_cable_coef = alpha * gradp_cableFeasibility_coef.transpose() * vel_cable_coef;
          jerkOpt_.get_gdC().block<6, 6>(i * 6, 3) += omg * step * gradViolaPc_cable_coef;

          Eigen::Matrix<double, 6, 3> gradViolaPc_load_acc = beta2 * gradp_cableFeasibility_loadacc.transpose();
          double gradViolaPt_load_acc = alpha * gradp_cableFeasibility_loadacc.transpose() * jer_load;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaPc_load_acc;

          gdT(i) += omg * (costp_cableFeasibility_coef / K + step * gradViolaPt_cable_coef + step * gradViolaPt_load_acc);
          costs(5) += omg * step * costp_cableFeasibility_coef;
        }

        s1 += step;
        if (j != K || (j == K && i == N - 1))
        {
          ++i_dp;
        }
      }
      t += jerkOpt_.get_T1()(i);
    }

    // quratic variance
    Eigen::MatrixXd gdp;
    double var;
    distanceSqrVarianceWithGradCost2p(cps_.points, gdp, var);

    i_dp = 0;
    for (int i = 0; i < N; ++i)
    {
      step = jerkOpt_.get_T1()(i) / K;
      s1 = 0.0;

      for (int j = 0; j <= K; ++j)
      {
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s4 * s1;
        beta0 << 1.0, s1, s2, s3, s4, s5;
        beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
        alpha = 1.0 / K * j;
        vel = jerkOpt_.get_b().block<6, dim>(i * 6, 0).transpose() * beta1;

        omg = (j == 0 || j == K) ? 0.5 : 1.0;

        gradViolaPc = beta0 * gdp.col(i_dp).transpose();
        gradViolaPt = alpha * gdp.col(i_dp).transpose() * vel;
        jerkOpt_.get_gdC().block<6, dim>(i * 6, 0) += omg * gradViolaPc;
        gdT(i) += omg * (gradViolaPt);

        s1 += step;
        if (j != K || (j == K && i == N - 1))
        {
          ++i_dp;
        }
      }
    }
    costs(6) += var;

  }

  bool PolyTrajOptimizer::obstacleGradCostP(const int i_dp,
                                            const Eigen::Vector3d &p,
                                            Eigen::Vector3d &gradp,
                                            double &costp)
  {
    if (i_dp == 0 || i_dp >= cps_.cp_size * 2 / 3)
      return false;

    bool ret = false;

    gradp.setZero();
    costp = 0;

    // use esdf
    double dist;
    grid_map_->evaluateEDT(p, dist);
    double dist_err = obs_clearance_ - dist;
    if (dist_err > 0)
    {
      ret = true;
      Eigen::Vector3d dist_grad;
      grid_map_->evaluateFirstGrad(p, dist_grad);

      costp = wei_obs_ * pow(dist_err, 3);
      gradp = -wei_obs_ * 3.0 * pow(dist_err, 2) * dist_grad;
    }

    return ret;
  }

  bool PolyTrajOptimizer::feasibilityGradCostV(const Eigen::Vector3d &v,
                                               Eigen::Vector3d &gradv,
                                               double &costv)
  {
    double vpen = v.squaredNorm() - max_vel_ * max_vel_;
    if (vpen > 0)
    {
      gradv = wei_feas_ * 6 * vpen * vpen * v;
      costv = wei_feas_ * vpen * vpen * vpen;
      return true;
    }
    return false;
  }

  bool PolyTrajOptimizer::feasibilityGradCostA(const Eigen::Vector3d &a,
                                               Eigen::Vector3d &grada,
                                               double &costa)
  {
    double apen = a.squaredNorm() - max_acc_ * max_acc_;
    if (apen > 0)
    {
      grada = wei_feas_ * 6 * apen * apen * a;
      costa = wei_feas_ * apen * apen * apen;
      return true;
    }
    return false;
  }

  void PolyTrajOptimizer::distanceSqrVarianceWithGradCost2p(const Eigen::MatrixXd &ps,
                                                            Eigen::MatrixXd &gdp,
                                                            double &var)
  {
    int N = ps.cols() - 1;
    Eigen::MatrixXd dps = ps.rightCols(N) - ps.leftCols(N);
    Eigen::VectorXd dsqrs = dps.colwise().squaredNorm().transpose();
    double dsqrsum = dsqrs.sum();
    double dquarsum = dsqrs.squaredNorm();
    double dsqrmean = dsqrsum / N;
    double dquarmean = dquarsum / N;
    var = wei_sqrvar_ * (dquarmean - dsqrmean * dsqrmean);
    gdp.resize(dim, N + 1);
    gdp.setZero();
    for (int i = 0; i <= N; i++)
    {
      if (i != 0)
      {
        gdp.col(i) += wei_sqrvar_ * (4.0 * (dsqrs(i - 1) - dsqrmean) / N * dps.col(i - 1));
      }
      if (i != N)
      {
        gdp.col(i) += wei_sqrvar_ * (-4.0 * (dsqrs(i) - dsqrmean) / N * dps.col(i));
      }
    }
    return;
  }

  void PolyTrajOptimizer::astarWithMinTraj(const Eigen::MatrixXd &iniState,
                                           const Eigen::MatrixXd &finState,
                                           vector<Eigen::Vector3d> &simple_path,
                                           Eigen::MatrixXd &ctl_points,
                                           poly_traj::MinJerkOpt &frontendMJ)
  {
    //frontendMJ is initMJO
    Eigen::Vector3d start_pos = iniState.col(0).topRows(3); //初始位置
    Eigen::Vector3d end_pos = finState.col(0).topRows(3); //结束位置

    /* astar search and get the simple path*/
    simple_path = a_star_->astarSearchAndGetSimplePath(grid_map_->getResolution(), start_pos, end_pos, load_dist_);

    /* generate minimum snap trajectory based on the simple_path waypoints*/
    int piece_num = simple_path.size() - 1;
    Eigen::MatrixXd innerPts;

    cout << "atar" << endl;
    if (piece_num > 1)
    {
      innerPts.resize(dim, piece_num - 1);
      for (int i = 0; i < piece_num - 1; i++)
        innerPts.col(i) << simple_path[i + 1], iniState.col(0).bottomRows(6);
    }
    else
    {
      // piece_num == 1
      piece_num = 2;
      innerPts.resize(dim, 1);
      innerPts.col(0) << (simple_path[0] + simple_path[1]) / 2,  iniState.col(0).bottomRows(6);
    }

    frontendMJ.reset(iniState, finState, piece_num);

    /* generate init traj*/
    double des_vel = max_vel_;
    Eigen::VectorXd time_vec(piece_num);
    int debug_num = 0;
    do
    {
      for (size_t i = 1; i <= piece_num; ++i)
      {
        time_vec(i - 1) = (i == 1) ? (simple_path[1] - start_pos).norm() / des_vel
                                   : (simple_path[i] - simple_path[i - 1]).norm() / des_vel;
      }
      frontendMJ.generate(innerPts, time_vec);
      debug_num++;
      des_vel /= 1.5;
    } while (frontendMJ.getTraj().getMaxVelRate() > max_vel_ && debug_num < 1);

    ctl_points = frontendMJ.getInitConstrainPoints(cps_num_prePiece_);
  }
  
  /* helper functions */
  void PolyTrajOptimizer::setParam(ros::NodeHandle &nh)
  {
    nh.param("optimization/constrain_points_perPiece", cps_num_prePiece_, -1);

    nh.param("optimization/weight_obstacle", wei_obs_, -1.0);
    nh.param("optimization/weight_feasibility", wei_feas_, -1.0);
    nh.param("optimization/weight_sqrvariance", wei_sqrvar_, -1.0);
    nh.param("optimization/weight_time", wei_time_, -1.0);
    nh.param("optimization/obstacle_clearance", obs_clearance_, -1.0); 
    nh.param("optimization/load_dist", load_dist_, 0.5);
    nh.param("optimization/max_vel", max_vel_, -1.0);
    nh.param("optimization/max_acc", max_acc_, -1.0);

    nh.param("cable/cable_length", cable_length_, 2.0);
    nh.param("cable/uav_obs_clearance", uav_obs_clearance_, 2.0 ); //无人机距离障碍物的最小距离
    nh.param("cable/uav_swarm_clearance", uav_swarm_clearance_, 2.0); //无人机间的最小距离
    nh.param("cable/cable_clearance", cable_clearance_, 2.0); //无人机间的最小距离
    nh.param("cable/weight_uav_obs", weight_uav_obs_, -1.0);
    nh.param("cable/weight_uav_swarm", weight_uav_swarm_, -1.0);
    nh.param("cable/weight_FM_feasibility", weight_FM_feasibility_, -1.0);
    nh.param("cable/weight_FM_norm", weight_FM_norm_, -1.0);
    nh.param("cable/weight_cable_collision", weight_cable_collision_, -1.0);
    nh.param("cable/cable_piece", cable_piece_, 5);
    nh.param("cable/wei_energy", wei_energy_, -1.0);

    nh.param("lbfgs_params/mem_size", mem_size_, 6);
    nh.param("lbfgs_params/g_epsilon", g_epsilon_, 1.0e-5);
    nh.param("lbfgs_params/past", past_, 0);
    nh.param("lbfgs_params/delta", delta_, 1.0e-5);
    nh.param("lbfgs_params/max_iterations", max_iterations_, 0);
    nh.param("lbfgs_params/max_linesearch", max_linesearch_, 40);
    nh.param("lbfgs_params/min_step", min_step_, 1.0e-20);
    nh.param("lbfgs_params/max_step", max_step_, 1.0e+20);
    nh.param("lbfgs_params/f_dec_coeff", f_dec_coeff_, 1.0e-4);
    nh.param("lbfgs_params/s_curv_coeff", s_curv_coeff_, 0.9);

    // set the formation type
  }

  void PolyTrajOptimizer::setEnvironment(const GridMap::Ptr &map, const cable_load::Ptr &CLoad)
  {
    grid_map_ = map;

    a_star_.reset(new AStar);
    a_star_->initGridMap(grid_map_, Eigen::Vector3i(800, 200, 40));

    cable_load_ = CLoad;
    Transformb2e = cable_load_->Transformb2e;
  }

  void PolyTrajOptimizer::setControlPoints(const Eigen::MatrixXd &points)
  {
    cps_.resize_cp(points.cols());
    cps_.points = points;
  }

}

// void plot(poly_traj::MinJerkOpt test_minco)
// {
//     // get data for plot 
//     double duration =  test_minco.getTraj().getTotalDuration();
//     size_t N = 1000;
//     double step = duration / N;
//     std::vector<double> times, pos_vector, vel_vector, acc_vector;
//     std::vector<std::vector<double>> pos_vectors, vel_vectors, acc_vectors;
//     for (size_t j = 0; j < dim; j++)
//     {
//         times.clear();
//         pos_vector.clear();
//         vel_vector.clear();
//         acc_vector.clear();
//         for (size_t i = 0; i < N; i++)
//         {
//             double time = i*step;
//             times.push_back(time); //times
//             pos_vector.push_back(test_minco.getTraj().getPos(time)(j));
//             vel_vector.push_back(test_minco.getTraj().getVel(time)(j));
//             acc_vector.push_back(test_minco.getTraj().getAcc(time)(j));
//         }    
//         pos_vectors.push_back(pos_vector);
//         vel_vectors.push_back(vel_vector);
//         acc_vectors.push_back(acc_vector);
//     }
//     // plot
//     plt::figure_size(1200,780);
//     for (size_t i = 0; i < dim; i++)
//     {
//        plt::subplot(3,3,i+1); 
//        plt::plot(times, pos_vectors[i]); 
//     }
//     plt::figure_size(1200,780);
//     for (size_t i = 0; i < dim; i++)
//     {
//        plt::subplot(3,3,i+1); 
//        plt::plot(times, vel_vectors[i]); 
//     }
//     plt::figure_size(1200,780);
//     for (size_t i = 0; i < dim; i++)
//     {
//        plt::subplot(3,3,i+1); 
//        plt::plot(times, acc_vectors[i]); 
//     }
// }

// void plot_results_step(std::map<int, std::string> ylabel_, std::vector<Eigen::VectorXd> results)
// {
//     plt::figure_size(1200,780);
    
//     size_t iter_num = results.size();
//     size_t variable_num = results[0].size(); 
//     long fig_num = variable_num - 1;

//     // columns = 3
//     // long fig_rows = ceil(fig_num / fig_columns);
//     // plt::figure_size(1800, 1200);

//     std::vector<std::vector<double>> datas;
//     std::vector<double> data;
    
//     for(size_t j = 0; j < variable_num; j++)
//     {
//         data.clear();
//         for (size_t i = 0; i < iter_num; i++)
//         {
//             data.push_back(results[i](j)); 
//         }
//         datas.push_back(data);
//     }

//     for (size_t i = 0; i < fig_num; i++)
//     {
//         plt::subplot(3, 4, i+1);
//         plt::plot( datas[0], datas[i+1]);
//         plt::ylabel(ylabel_[i]);
//     }
    
//     std::map<std::string, double> m = {{"hspace", 0.3}, {"wspace", 0.45}};
//     plt::subplots_adjust(m);
//     // plt::legend();
// }

// int main(int argc, char **argv)
// {
//   cout << "test opt" << endl;
//   ros::init(argc, argv, "ego_planner_node");
//   ros::NodeHandle nh("~");
  
//   int pieceNum = 5;
//   Eigen::Matrix<double, dim, 3> head_state = Eigen::Matrix<double,dim,3>::Zero();
//   Eigen::Matrix<double, dim, 3> tail_state = Eigen::Matrix<double,dim,3>::Zero();
//   Eigen::MatrixXd inPs; 
//   Eigen::VectorXd ts;
//   ts.resize(pieceNum); ts.setZero();
//   inPs.resize(dim, pieceNum-1); inPs.setZero();
    
//   tail_state(0,0) = 20;
//   tail_state(5,0) = 10;
//   inPs(0,0) = 10; inPs(2,1) = 5;
//   ts << 1, 2, 3, 4, 5;

//   ego_planner::PolyTrajOptimizer test; //init

//   test.OptimizeTrajectory_lbfgs(head_state, tail_state, inPs, ts, ros::Time::now().toSec());
//   plot(test.getMinJerkOptPtr0());
//   std::map<int, std::string> ylabel_ = {{0, "smooth_cost"}, 
//     {1, "time_cost"},
//     {2, "load_obstacle"},
//     {3, "load_feasibility}, 
//     {4, "uav_swarm"},
//     {5, "uav_obstacle"},
//     {6, "cable_collision"},
//     {7, "cable_feasibility"},
//     {8, "quadratic"},
//     {9, "total"}};
//   plot_results_step(ylabel_, test.results_step);
//   plt::show();
//   return 0;
// }