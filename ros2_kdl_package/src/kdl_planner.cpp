#include "kdl_planner.h"


KDLPlanner::KDLPlanner(){}

KDLPlanner::KDLPlanner(double _maxVel, double _maxAcc)
{
    velpref_ = new KDL::VelocityProfile_Trap(_maxVel,_maxAcc);
}

KDLPlanner::KDLPlanner(double _trajDuration, double _accDuration, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd, double _trajRadius)
{
    trajDuration_ = _trajDuration;
    accDuration_ = _accDuration;
    trajInit_ = _trajInit;
    trajEnd_ = _trajEnd;
    trajRadius_ = _trajRadius;
}

/*KDLPlanner::KDLPlanner(double _trajDuration, Eigen::Vector3d _trajInit, double _trajRadius)
{
  trajDuration_ = _trajDuration;
  trajInit_ = _trajInit;
  trajRadius_ = _trajRadius;
}*/

void KDLPlanner::CreateTrajectoryFromFrames(std::vector<KDL::Frame> &_frames,
                                            double _radius, double _eqRadius
                                            )
{
    path_ = new KDL::Path_RoundedComposite(_radius,_eqRadius,new KDL::RotationalInterpolation_SingleAxis());

    for (unsigned int i = 0; i < _frames.size(); i++)
    {
        path_->Add(_frames[i]);
    }
    path_->Finish();

    velpref_->SetProfile(0,path_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_, velpref_);
}

void KDLPlanner::createCircPath(KDL::Frame &_F_start,
                                KDL::Vector &_V_centre,
                                KDL::Vector& _V_base_p,
                                KDL::Rotation& _R_base_end,
                                double alpha,
                                double eqradius
                                )
{
    KDL::RotationalInterpolation_SingleAxis* otraj;
    otraj = new KDL::RotationalInterpolation_SingleAxis();
    otraj->SetStartEnd(_F_start.M,_R_base_end);
    path_circle_ = new KDL::Path_Circle(_F_start,
                                        _V_centre,
                                        _V_base_p,
                                        _R_base_end,
                                        alpha,
                                        otraj,
                                        eqradius);
    velpref_->SetProfile(0,path_circle_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_circle_, velpref_);
}

KDL::Trajectory* KDLPlanner::getTrajectory()
{
	return traject_;
}

void KDLPlanner::trapezoidal_vel(double t, double t_c, double& s, double& s_dot, double& s_ddot)
{
  //Final time calculation based on the acceleration time
  double t_f = trajDuration_;

  //Calculate the consatant acceleration needed
  double s_ddot_c = 1.0/(t_c * (t_f - t_c));  //corrected acceleration calcultion

  if (t <= t_c){
    //acceleration phase
    s = 0.5 * s_ddot_c * std::pow(t,2);
    s_dot = s_ddot_c * t;
    s_ddot = s_ddot_c;
  }
  else if(t <= (t_f - t_c)){
    //Constant velocity phase
    s = 0.5 * s_ddot_c * t_c * (2*t - t_c); //corrected formula
    s_dot = s_ddot_c * t_c; //constant velocity
    s_ddot = 0.0;
  }else if(t <= t_f){
    //Deceleration phase
    double tau = t_f - t;
    s = 1.0 - 0.5 * s_ddot_c * std::pow(tau, 2);
    s_dot = s_ddot_c * tau;
    s_ddot = -s_ddot_c;
  }else{
    //After final time
    s = 1.0;
    s_dot = 0.0;
    s_ddot = 0.0;
  }
}

void KDLPlanner::cubic_polinomial(double t, double& s, double& s_dot, double& s_ddot)
{
  double a_0 = trajInit_.y();
  double a_1 = 0.0 ;
  double a_2 = 3*(trajEnd_.y() - trajInit_.y())/std::pow(trajDuration_, 2);
  double a_3 = 2*(trajInit_.y() - trajEnd_.y())/std::pow(trajDuration_, 3);
  
  // Compute the curvilinear abscissa, velocity, and acceleration
  s = a_3 * std::pow(t, 3) + a_2 * std::pow(t, 2) + a_1 * t + a_0;
  s_dot = 3 * a_3 * std::pow(t, 2) + 2 * a_2 * t + a_1;
  s_ddot = 6 * a_3 * t + 2 * a_2;
}

trajectory_point KDLPlanner::compute_trajectory(double time, int VAL)
{
  /* trapezoidal velocity profile with accDuration_ acceleration time period and trajDuration_ total duration.
     time = current time
     trajDuration_  = final time
     accDuration_   = acceleration time
     trajInit_ = trajectory initial point
     trajEnd_  = trajectory final point */

  double s, s_dot, s_ddot;

  trajectory_point traj;

  if(VAL==0){
    std::cout << "VAL : " << VAL << std::endl;
    std::cout << "cubic " << std::endl;
  cubic_polinomial(time, s, s_dot, s_ddot);
  }else if (VAL==1){
    std::cout << "VAL : " << VAL << std::endl;
    std::cout << "trapezoidal " << std::endl;
  trapezoidal_vel(time, accDuration_, s, s_dot, s_ddot);
  }

  if(trajRadius_ > 0)
  { 
  //Circular Path Trajectory

    traj.pos.x() = trajInit_.x();
    traj.pos.y() = trajInit_.y() - trajRadius_*cos(2 * M_PI * s);
    traj.pos.z() = trajInit_.z() - trajRadius_*sin(2 * M_PI * s);

    traj.vel.x() = 0.0;
    traj.vel.y() = trajRadius_ * 2.0 *M_PI * sin(2* M_PI * s) * s_dot;
    traj.vel.z() = - trajRadius_ * 2.0 *M_PI * cos(2* M_PI * s) * s_dot;

    traj.acc.x() = 0.0;
    traj.acc.y() = trajRadius_ * 2.0 *M_PI * (2.0 * M_PI* cos(2* M_PI * s) * std::pow(s_dot,2) + sin(2* M_PI * s) * s_ddot);
    traj.acc.z() = trajRadius_ * 2.0 *M_PI * (2.0 * M_PI* sin(2* M_PI * s) * std::pow(s_dot,2) - cos(2* M_PI * s) * s_ddot);

    std::cout << "The circolar velocity is : " << traj.vel.y() << std::endl;
    std::cout << "The circolar acceleration is : " << traj.acc.y() << std::endl;

  }else{
  //Linear Path Trajectory

    traj.pos = trajInit_ + s * (trajEnd_ - trajInit_);
    traj.vel = s_dot * (trajEnd_ - trajInit_);
    traj.acc = s_ddot * (trajEnd_ - trajInit_);

    std::cout << "The velocity is : " << traj.vel.y() << std::endl;
    std::cout << "The acceleration is : " << traj.acc.y() << std::endl;
  }

  return traj;

}
