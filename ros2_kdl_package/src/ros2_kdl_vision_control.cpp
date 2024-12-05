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


#if __has_include("cv_bridge/cv_bridge.hpp")
#include "cv_bridge/cv_bridge.hpp"
#else
#include "cv_bridge/cv_bridge.h"
#endif
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "image_transport/image_transport.hpp"
#include "rcpputils/asserts.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker.hpp"
 
using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;

class Iiwa_pub_sub : public rclcpp::Node
{
    public:
        Iiwa_pub_sub()
        : Node("ros2_kdl_vision_control"),
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
            aruco_state_available_ = false; 

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
            init_p.resize(nj);
    
            // Subscriber to jnt states
            jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10, std::bind(&Iiwa_pub_sub::joint_state_subscriber, this, std::placeholders::_1));

            // Wait for the joint_state topic
            while(!joint_state_available_){
                RCLCPP_INFO(this->get_logger(), "No data received yet! ...");
                rclcpp::spin_some(node_handle_);
            }
            
            // Parametro per selezionare il tipo di tasks
            this->declare_parameter<std::string>("task", "positioning"); //default to positioning
            
            // Ottieni il parametro configurato
            std::string task;
            this->get_parameter("task", task);
            RCLCPP_INFO(get_logger(),"Current task is: '%s'", task.c_str());
            // Aruco Subscriber
            pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/aruco_single/pose", 10, std::bind(&Iiwa_pub_sub::marker_callback, this, std::placeholders::_1));

           
            while(!aruco_state_available_){
                RCLCPP_INFO(this->get_logger(), "No Marker detected yet! ...");
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
            init_p.data=robot_->getJntValues();
            // Initialize controller
            KDLController controller_(*robot_);
            
            // EE's trajectory initial position (just an offset)
            Eigen::Vector3d init_position(Eigen::Vector3d(init_cart_pose_.p.data));
            Eigen::Vector3d end_position;

            
            //KDL::Frame ee_to_camera = KDL::Frame(KDL::Rotation::Quaternion(-0.5,0.5,-0.5,0.5),KDL::Vector(0,0,0));
            
            KDL::Frame camera_base(aruco_frame.M,KDL::Vector(aruco_frame.p.data[0]+0.7,aruco_frame.p.data[1],aruco_frame.p.data[2]));
            KDL::Frame A_posa(aruco_frame.M,KDL::Vector(aruco_frame.p.data[0],aruco_frame.p.data[1],aruco_frame.p.data[2]));

            end_position=toEigen(camera_base.p);
            
           // aruco_frame.M = aruco_frame*roty;
            //aruco_frame_ =  aruco_frame * ee_to_camera;
            // Trasformazione desiderata EE rispetto alla base
           
            //end_orientation=base_to_ee.M;
            std::cout<<"\nx:"<<base_to_ee.p.data[0]<<std::endl;
            std::cout<<"\ny:"<<base_to_ee.p.data[1]<<std::endl;
            std::cout<<"\nz:"<<base_to_ee.p.data[2]<<std::endl;
            //base_to_ee = base_to_ee * aruco_frame * offset_to_ee;
    
            /*//Aruco
            KDL::Frame frame_offset = aruco_frame;
            frame_offset.p = aruco_frame.p - KDL::Vector(0,0,1);
            
            KDL::Frame base_offset = robot_->getEEFrame()*frame_offset;*/
           // if(task=="positioning")
            //{
             //   end_position=toEigen(base_to_ee.p);

            //}


            // EE's trajectory end position (just opposite y)
            //end_position << base_to_ee.p.data[0]+ offset_to_ee.p.data[0], base_to_ee.p.data[1], base_to_ee.p.data[2];
            //Eigen::Vector3d end_position(Eigen::Vector3d(base_to_ee.p.data));

            
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
            this->get_parameter("task",task);
            // define trajectory
            double total_time = 4.5; // 
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
           
                  // compute errors
                
            
            if(task=="positioning")
            {
                if(t_<total_time){
                // Retrieve the trajectory point
                trajectory_point p = planner_.compute_trajectory(t_, VAL); 

                
                
                    // Compute EE frame
                KDL::Frame cartpos = robot_->getEEFrame(); 

                KDL::Frame desired_frame; desired_frame.M = aruco_frame.M; desired_frame.p=toKDL(p.pos);
                Eigen::Vector3d error = computeLinearError(Eigen::Vector3d(p.pos), Eigen::Vector3d(cartpos.p.data));
                std::cout << "The error norm is : " << error.norm() << std::endl; 
                KDL::Rotation roty=KDL::Rotation::RotY(M_PI);
                Eigen::Matrix3d Aruc_r = toEigen(roty)*toEigen(aruco_frame.M);
                Eigen::Vector3d o_error = computeOrientationError(Aruc_r, toEigen(cartpos.M));

                // Compute desired Frame
               // Calculate differential IK
               if(cmd_interface_=="velocity"){
                Vector6d cart_vel; 
                cart_vel <<  p.vel + 5 * error, o_error;
                joint_velocities_.data = pseudoinverse(robot_->getEEJacobian().data) * cart_vel;
                joint_positions_.data += joint_velocities_.data * dt;
                }
                // Update robot state
                robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data)); 
                  
                // Assign joint velocity commands
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = joint_velocities_(i);
                }

                // Publish velocity commands
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);
                }else{
                // Stop movement by setting velocities to zero
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = 0.0;
                    std::cout << "Stopping joint velocities: " << joint_velocities_.data << std::endl;
                }

                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);
                }
            }           
            else if(task=="look-at-point"){

                    if(cmd_interface_=="velocity")
                    {   
                        
                        std::cout << "Eseguiamo il look_at_point \n";
 
                        // Posizione corrente del marker ArUco
                        Eigen::Vector3d cPo = toEigen(aruco_frame.p);
                        Eigen::MatrixXd Rc = toEigen(robot_->getEEFrame().M); // Matrice di rotazione della camera
                    
                        // S calcolato come lo skew del vettore normalizzato
                        Eigen::Vector3d s = cPo / cPo.norm();
                        Eigen::MatrixXd S = skew(s);
                        Eigen::MatrixXd J_c(6,7);
                        KDL::Jacobian J = robot_->getEEJacobian();
                        J_c=J.data;
                        // Matrice identità
                        Eigen::MatrixXd I = Eigen::Matrix3d::Identity();
                    
                        // (I - s * s^T)
                        Eigen::MatrixXd I_minus_ssT = I - s * s.transpose();
                    
                        Eigen::Matrix<double, 6, 6> R;
                        R.setZero();
                        R.block<3,3>(0, 0) = Rc.transpose();
                        R.block<3,3>(3, 3) = Rc.transpose();
                        KDL::Rotation rot = aruco_frame.M;
                        Eigen::Matrix<double, 3, 6> Ls;
                        Ls.setZero();
                        Ls.block<3,3>(0, 0) = (-(1.0 / cPo.norm()) * I_minus_ssT) * Rc.transpose();
                        Ls.block<3,3>(0, 3) = S * Rc.transpose();
                    
                        Eigen::MatrixXd N(7,7);
                        N = (Eigen::MatrixXd::Identity(7, 7) - (pseudoinverse(Ls * J_c) * (Ls * J_c)));
                    
                        // Calcolo della variazione del marker ArUco
                        static Eigen::Vector3d previous_cPo = cPo;
                        double marker_movement = (cPo - previous_cPo).norm();
                        previous_cPo = cPo;
                        Eigen::Vector3d sd;
                        sd<< 0 , 0 , 1 ;
                        double k = 0.5;
                        // Soglia di movimento per riattivare il robot
                        double movement_threshold = 0.01;
                        
                        if (t_ < total_time || marker_movement > movement_threshold) {
                            // Riprende il movimento solo se il marker si sposta
                        
                        Eigen::VectorXd q_dt;
                        
                        Eigen::VectorXd q_0_dot(7);
                    
                        q_0_dot = init_p.data - joint_positions_.data;
                        joint_velocities_.data = k*pseudoinverse(Ls*J_c)*sd + N*q_0_dot;
                    
                            
                        joint_positions_.data = joint_positions_.data + joint_velocities_.data * dt;
                    
                            std::cout << "Movimento verso il marker in corso...\n";
                        } else {
                            // Ferma il robot se il marker non si muove
                            joint_velocities_.data.setZero();
                            std::cout << "Il marker è fermo, il robot rimane in posizione.\n";
                            std::cout<<"s_Error: "<<(sd - s).norm()<<std::endl;
                        }
                        
                        // Aggiorna lo stato del robot
                        robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));

                    }


                    if(cmd_interface_ == "effort") {
                            
                            // Posizione, velocità e accelerazione desiderate nello spazio cartesiano
                            KDL::Frame desired_frame;
                            desired_frame.p = toKDL(p.pos); // Posizione desiderata
                        // desired_frame.M = cartpos.M; // Orientamento desiderato

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
                        // Update KDLrobot structure
                        robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

                        if(cmd_interface_ == "velocity"){
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

            }    
            else{
                RCLCPP_INFO_ONCE(this->get_logger(), "Trajectory executed successfully ...");
                              
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

        void marker_callback(const geometry_msgs::msg::PoseStamped & aruco_msg) 
        {
            /*RCLCPP_INFO(this->get_logger(), 
                "\nPosition -> x: %f \n y: %f \n z: %f",
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z);

            RCLCPP_INFO(this->get_logger(), 
                "\nOrientation (quaternion) -> x: %f \n y: %f \n z: %f \n w: %f",
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w);*/
                double x=aruco_msg.pose.position.x;
                double y=aruco_msg.pose.position.y;
                double z=aruco_msg.pose.position.z;

                double q_x=aruco_msg.pose.orientation.x;
                double q_y=aruco_msg.pose.orientation.y;
                double q_z=aruco_msg.pose.orientation.z;
                double q_w=aruco_msg.pose.orientation.w;
            
            
                aruco_frame.p = KDL::Vector(x,y,z);
                aruco_frame.M = KDL::Rotation::Quaternion(q_x,q_y,q_z,q_w);
                
                /*// Offset desiderato tra marker e EE
                KDL::Frame offset_to_ee;
                offset_to_ee.p = KDL::Vector(0.0, 0.0, 1.0); // 100 cm sopra il marker
                offset_to_ee.M = KDL::Rotation::Identity();  // Orientamento desiderato

                KDL::Frame ee_to_camera;
                ee_to_camera.p = KDL::Vector(0.0, 0.0, 0.304); // Traslazione
                ee_to_camera.M = KDL::Rotation::RPY(-M_PI, -M_PI/2, 0.0); // Rotazione

                // Trasformazione desiderata EE rispetto alla base
                KDL::Frame base_to_ee = robot_->getEEFrame()*ee_to_camera; // EE -> Base
                base_to_ee = base_to_ee * aruco_frame * offset_to_ee;*/
               
                          
            aruco_state_available_ = true;

        }
        
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub; 
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
        
        KDL::Frame aruco_frame; //aruco position
        bool aruco_state_available_;
        KDL::Frame base_to_ee;
        bool aruco_detect = false;

        KDL::Rotation end_orientation;

        std::string task;
        std::string trajectory_type;
        std::string cmd_interface_;
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
        int VAL;        // variabile per scegliere il tipo di traiettoria

        KDL::JntArray init_p;
        KDL::Frame init_cart_pose_;
       
};

int main( int argc, char** argv )
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Iiwa_pub_sub>());
    rclcpp::shutdown();
    return 1;
}


