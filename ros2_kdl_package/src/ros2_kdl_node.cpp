// Copyright  (C)  2007  Francois Cauwe <francois at cauwe dot org>
 
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
 
#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"

#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"
 
using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;

class Iiwa_pub_sub : public rclcpp::Node
{
    public:
        Iiwa_pub_sub()
        : Node("ros2_kdl_node"), 
        node_handle_(std::shared_ptr<Iiwa_pub_sub>(this))
        {
            // declare cmd_interface parameter (position, velocity)
            declare_parameter("cmd_interface", "position"); // defaults to "position"
            get_parameter("cmd_interface", cmd_interface_);
            RCLCPP_INFO(get_logger(),"Current cmd interface is: '%s'", cmd_interface_.c_str());

            if (!(cmd_interface_ == "position" || cmd_interface_ == "velocity" || cmd_interface_ == "effort"))
            {
                RCLCPP_INFO(get_logger(),"Selected cmd interface is not valid!"); return;
            }

            iteration_ = 0;
            t_ = 0;
            joint_state_available_ = false; 

            // retrieve robot_description param
            auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node_handle_, "robot_state_publisher");
            while (!parameters_client->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    rclcpp::shutdown();
                }
                RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
            }
            auto parameter = parameters_client->get_parameters({"robot_description"});

            // create KDLrobot structure
            KDL::Tree robot_tree;
            if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree)){
                std::cout << "Failed to retrieve robot_description param!";
            }
            robot_ = std::make_shared<KDLRobot>(robot_tree);  
            
            // Create joint array
            unsigned int nj = robot_->getNrJnts();
            KDL::JntArray q_min(nj), q_max(nj);
            q_min.data << -2.96,-2.09,-2.96,-2.09,-2.96,-2.09,-2.96; //-2*M_PI,-2*M_PI; // TODO: read from urdf file
            q_max.data <<  2.96,2.09,2.96,2.09,2.96,2.09,2.96; //2*M_PI, 2*M_PI; // TODO: read from urdf file          
            robot_->setJntLimits(q_min,q_max);            
            joint_positions_.resize(nj); 
            joint_velocities_.resize(nj);
            ex_joint_velocities.resize(nj);
            joint_torques_.resize(nj); 
            joint_acceleration_.resize(nj);
            joint_positions_cmd_.resize(nj);
            joint_velocities_cmd_.resize(nj);
            joint_efforts_cmd_.resize(nj);
    
            // Subscriber to jnt states
            jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10, std::bind(&Iiwa_pub_sub::joint_state_subscriber, this, std::placeholders::_1));

            // Wait for the joint_state topic
            while(!joint_state_available_){
                RCLCPP_INFO(this->get_logger(), "No data received yet! ...");
                rclcpp::spin_some(node_handle_);
            }

            // Update KDLrobot object
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
            KDL::Frame f_T_ee = KDL::Frame::Identity();
            robot_->addEE(f_T_ee);
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

            // Compute EE frame
            init_cart_pose_ = robot_->getEEFrame();
            // std::cout << "The initial EE pose is: " << std::endl;  
            // std::cout << init_cart_pose_ <<std::endl;

            // Compute IK
            KDL::JntArray q(nj);
            robot_->getInverseKinematics(init_cart_pose_, q);
            // std::cout << "The inverse kinematics returned: " <<std::endl; 
            // std::cout << q.data <<std::endl;

            // Initialize controller
            KDLController controller_(*robot_);

            // EE's trajectory initial position (just an offset)
            Eigen::Vector3d init_position(Eigen::Vector3d(init_cart_pose_.p.data) - Eigen::Vector3d(0,0,0.1));

            // EE's trajectory end position (just opposite y)
            Eigen::Vector3d end_position; end_position << init_position[0], -init_position[1], init_position[2];

            // Plan trajectory
            double traj_duration = 1.5, acc_duration = 0.5, t = 0.0, radius = 0.0;
            planner_ = KDLPlanner(traj_duration, acc_duration, init_position, end_position, radius);

            // Parametro per selezionare il tipo di traiettoria
            this->declare_parameter<std::string>("trajectory_type", "trapezoidal"); //default to trapezoidal

            // Ottieni il parametro configurato
            std::string trajectory_type;
            this->get_parameter("trajectory_type", trajectory_type);
            if(cmd_interface_ == "effort"){
                std::cout<<"Press 0 for joint space inverse dynamics controller."<<std::endl;
                std::cout<<"Press 1 for inverse dynamics operational space controller."<<std::endl;
                std::cin>>choice; 
                if(choice != 0 && choice != 1){
                        
                        while(choice != 0 && choice != 1)     
                            {
                                
                                
                                std::cout<<"Input errato riprova."<<std::endl;
                                std::cin>>choice;
                                
                                }
                                
                }
            }                

            
            trajectory_point p;

            // Esegui il tipo di traiettoria scelto
            if (trajectory_type == "trapezoidal")
            {   
                VAL=1;
                // Crea una traiettoria lineare con il profilo trapezoidale
                p = planner_.compute_trajectory(t,VAL); // Funzione che implementa la traiettoria lineare
            }
            else if (trajectory_type == "cubic")
            {   
                VAL=0;
                // Crea una traiettoria lineare con il polinomio cubico
                p = planner_.compute_trajectory(t,VAL); // Funzione che implementa la traiettoria circolare
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Tipo di traiettoria non valido. Scegliere tra 'trapezoidal' oppure 'cubic' .");
                return ;
            }

            // compute errors
            Eigen::Vector3d error = computeLinearError(p.pos, Eigen::Vector3d(init_cart_pose_.p.data));
            //std::cout << "The initial error is : " << error << std::endl;
            
            if(cmd_interface_ == "position"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/iiwa_arm_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                            std::bind(&Iiwa_pub_sub::cmd_publisher, this));
            
                // Send joint position commands
                for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                    desired_commands_[i] = joint_positions_(i);
                }
            }
            else if(cmd_interface_ == "velocity"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                            std::bind(&Iiwa_pub_sub::cmd_publisher, this));
            
                // Send joint velocity commands
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = joint_velocities_(i);
                }
            }
            else if(cmd_interface_ == "effort")
            {
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/effort_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(10), 
                                            std::bind(&Iiwa_pub_sub::cmd_publisher, this));

                // Send joint torques commands
                for (long int i = 0; i < joint_torques_.size(); ++i) {
                    desired_commands_[i] = joint_torques_(i);
                }
            }

            // Create msg and publish
           
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);

            RCLCPP_INFO(this->get_logger(), "Starting trajectory execution ...");
        }

    private:

        void cmd_publisher(){

            iteration_ = iteration_ + 1;

            // define trajectory
            double total_time = 1.5; // 
            int trajectory_len = 150; // 
            int loop_rate = trajectory_len / total_time;
            double dt = 1.0 / loop_rate;
            t_+=dt;
            Kp = 210; // Guadagno proporzionale
            Kd = 20;  // Guadagno derivativo
            Kp_position = 210.0; // Guadagno proporzionale per la posizione
            Kd_position = 20.0;  // Guadagno per la velocità
            Kp_orientation = 210.0; // Guadagno proporzionale per l'orientamento
            Kd_orientation = 20.0;  // Guadagno per la velocità angolare
            // Determina il tipo di traiettoria
            if (trajectory_type == "cubic"){
                VAL = 0;  // Profilo trapezoidale
            } else if(trajectory_type == "trapezoidal"){
                VAL = 1;  // Profilo cubico
            }
            trajectory_point p;
            KDLController controller_(*robot_);
            
             
            
            if (t_ < total_time){

                // Set endpoint twist
                // double t = iteration_;
                // joint_velocities_.data[2] = 2 * 0.3 * cos(2 * M_PI * t / trajectory_len);
                // joint_velocities_.data[3] = -0.3 * sin(2 * M_PI * t / trajectory_len);

                // Integrate joint velocities
                // joint_positions_.data += joint_velocities_.data * dt;

                // Retrieve the trajectory point
                trajectory_point p = planner_.compute_trajectory(t_, VAL); 

                // Compute EE frame
                KDL::Frame cartpos = robot_->getEEFrame();           

                // Compute desired Frame
                KDL::Frame desFrame; desFrame.M = cartpos.M; desFrame.p = toKDL(p.pos); 

                // compute errors
                Eigen::Vector3d error = computeLinearError(p.pos, Eigen::Vector3d(cartpos.p.data));
                Eigen::Vector3d o_error = computeOrientationError(toEigen(init_cart_pose_.M), toEigen(cartpos.M));
                std::cout << "The error norm is : " << error.norm() << std::endl;

                if(cmd_interface_ == "position"){
                    // Next Frame
                    KDL::Frame nextFrame; nextFrame.M = cartpos.M; nextFrame.p = cartpos.p + (toKDL(p.vel) + toKDL(1*error))*dt; 

                    // Compute IK
                    robot_->getInverseKinematics(nextFrame, joint_positions_);
                }
                else if(cmd_interface_ == "velocity"){

                    // Compute differential IK
                    Vector6d cartvel; cartvel << p.vel + 5*error, o_error;
                    joint_velocities_.data = pseudoinverse(robot_->getEEJacobian().data)*cartvel;
                    joint_positions_.data = joint_positions_.data + joint_velocities_.data*dt;
                    
                }
                else if(cmd_interface_ == "effort") {
                     
                             
                    
                    // Posizione, velocità e accelerazione desiderate nello spazio cartesiano
                    KDL::Frame desired_frame;
                    desired_frame.p = toKDL(p.pos); // Posizione desiderata
                    desired_frame.M = cartpos.M; // Orientamento desiderato

                    KDL::Twist desired_vel;
                    desired_vel.vel = toKDL(p.vel); // Velocità lineare desiderata
                    

                    KDL::Twist desired_accel;
                    desired_accel.vel = toKDL(p.acc); // Accelerazione lineare desiderata (nessuna accelerazione per ora)
                    
                    robot_->getInverseKinematics(desired_frame, joint_positions_cmd_);
                    robot_->getInverseKinematicsVel(desired_vel,joint_velocities_cmd_);
                    Eigen::VectorXd q=pseudoinverse(robot_->getEEJacobian().data)*(toEigen(desired_accel)-robot_->getEEJaDot()*robot_->getJntVelocities());
                    for(unsigned int i=0;i<robot_->getNrJnts();i++)
                    {
                        joint_acceleration_(i)=q(i);
                    }
                    

                    if(choice==0){
                    joint_torques_=controller_.idCntr(joint_positions_cmd_,joint_velocities_cmd_,joint_acceleration_, Kp, Kd);
                    }
                    else if(choice==1){
                    joint_torques_=controller_.idCntr(desired_frame, desired_vel, desired_accel,Kp_position, Kp_orientation, Kd_position, Kd_orientation);
                    }
                    //std::cout<<"joint positions after \n\n"<<joint_positions_.data; 


                }
                /*else if(cmd_interface_ == "effort"){

                    

                }*/
                
                // Update KDLrobot structure
                robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

                if(cmd_interface_ == "position"){
                    // Send joint position commands
                    for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                        desired_commands_[i] = joint_positions_(i);
                    }
                }
                else if(cmd_interface_ == "velocity"){
                    // Send joint velocity commands
                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                        desired_commands_[i] = joint_velocities_(i);
                    }
                }
                else if(cmd_interface_ == "effort"){
                    // Send joint torques commands
                    for (long int i = 0; i < joint_torques_.size(); ++i) {
                        desired_commands_[i] = joint_torques_(i);
                    }
                }

                // Create msg and publish
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);

                // std::cout << "/////////////////////////////////////////////////" <<std::endl <<std::endl;
                // std::cout << "EE pose is: " << robot_->getEEFrame() <<std::endl;  
                // std::cout << "Jacobian: " << robot_->getEEJacobian().data <<std::endl;
                // std::cout << "joint_positions_: " << joint_positions_.data <<std::endl;
                // std::cout << "joint_velocities_: " << joint_velocities_.data <<std::endl;
                // std::cout << "iteration_: " << iteration_ <<std::endl <<std::endl;
                // std::cout << "/////////////////////////////////////////////////" <<std::endl <<std::endl; 
               

            }           
            else{
                RCLCPP_INFO_ONCE(this->get_logger(), "Trajectory executed successfully ...");
                // Send joint velocity commands
                
                if(cmd_interface_ == "position" || cmd_interface_ == "velocity"){               
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = 0.0;
                }
                } 
                if(cmd_interface_ == "effort"){
                    
                    joint_velocities_cmd_.data.setZero();
                    joint_acceleration_.data.setZero();
                    KDL::Frame frame_final = robot_->getEEFrame();
                    KDL::Twist velocities_final; velocities_final.vel=KDL::Vector::Zero(); velocities_final.rot=KDL::Vector::Zero();
                    KDL::Twist acceleration_final; acceleration_final.vel=KDL::Vector::Zero(); acceleration_final.rot=KDL::Vector::Zero();
                    
                   // if(choice==0){
                    joint_torques_=controller_.idCntr(joint_positions_cmd_,joint_velocities_cmd_,joint_acceleration_, Kp, Kd);
                   // }
                   // else if(choice==1){

                    //joint_torques_=controller_.idCntr(frame_final,velocities_final,acceleration_final,Kp_position, Kp_orientation, Kd_position, Kd_orientation);

                    
                    robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = joint_torques_(i);
                }
                   
                }
                // Create msg and publish
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);
            }
        }

        void joint_state_subscriber(const sensor_msgs::msg::JointState& sensor_msg){

            // for (size_t i = 0; i < sensor_msg.effort.size(); ++i) {
            //     RCLCPP_INFO(this->get_logger(), "Positions %zu: %f", i, sensor_msg.position[i]);                
            // }
            // std::cout<<"\n";
            // for (size_t i = 0; i < sensor_msg.effort.size(); ++i) {
            //     RCLCPP_INFO(this->get_logger(), "Velocities %zu: %f", i, sensor_msg.velocity[i]);
            // }
            // std::cout<<"\n";
            // for (size_t i = 0; i < sensor_msg.effort.size(); ++i) {
            //     RCLCPP_INFO(this->get_logger(), "Efforts %zu: %f", i, sensor_msg.effort[i]);
            // }

            joint_state_available_ = true;
            for (unsigned int i  = 0; i < sensor_msg.position.size(); i++){
                joint_positions_.data[i] = sensor_msg.position[i];
                joint_velocities_.data[i] = sensor_msg.velocity[i];
               // joint_torques_.i]=sensor_msg.effort[i];
            }
        }

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
        rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
        rclcpp::TimerBase::SharedPtr timer_; 
        rclcpp::TimerBase::SharedPtr subTimer_;
        rclcpp::Node::SharedPtr node_handle_;
        

        std::vector<double> desired_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        KDL::JntArray joint_positions_;
        KDL::JntArray joint_velocities_;
        Eigen::VectorXd joint_torques_;
        KDL::JntArray ex_joint_velocities;
        KDL::JntArray joint_acceleration_;
        std::shared_ptr<KDLRobot> robot_;
        KDLPlanner planner_;
        Eigen::Vector3d final_pos;
        KDL::JntArray joint_positions_cmd_;
        KDL::JntArray joint_velocities_cmd_;
        KDL::JntArray joint_efforts_cmd_;

        std::string trajectory_type;
        int iteration_;
        bool joint_state_available_;
        double t_;
        double Kd;
        double Kp;
        double Kp_position; // Guadagno proporzionale per la posizione
        double Kd_position;  // Guadagno per la velocità
        double Kp_orientation; // Guadagno proporzionale per l'orientamento
        double Kd_orientation;  // Guadagno per la velocità angolare
        int choice;
        int VAL; // variabile per scegliere il tipo di traiettoria
        std::string cmd_interface_;
        KDL::Frame init_cart_pose_;
};

 
int main( int argc, char** argv )
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Iiwa_pub_sub>());
    rclcpp::shutdown();
    return 1;
}