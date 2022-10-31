// from ros-control meta packages
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include "ros/ros.h"

#include <urdf/model.h>

// from kdl packages
#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/frames.hpp>
#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chaindynparam.hpp>              // inverse dynamics
#include <kdl/chainjnttojacsolver.hpp>        // jacobian
#include <kdl/chainfksolverpos_recursive.hpp> // forward kinematics
#include <kdl/chainfksolvervel_recursive.hpp> // forward kinematics


#include <boost/scoped_ptr.hpp>
#include <boost/lexical_cast.hpp>
#include <math.h>
#include <Eigen/LU>
#include <utils/pseudo_inversion.h>
#include <utils/skew_symmetric.h>

#define SaveDataMax 49



namespace arm_controllers
{
class Velocity_Controller : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
  public:
  bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
  {
    // 1. get joint name from the parameter server
    std::string my_joint;
    if (!n.getParam("joints", joint_names_))
    {
      ROS_ERROR("Could not find joint name");
      return false;
    }
    n_joints_ = joint_names_.size();
    
    if (n_joints_ == 0)
        {
            ROS_ERROR("List of joint names is empty.");
            return false;
        }
        else
        {
            ROS_INFO("Found %d joint names", n_joints_);
            for (int i = 0; i < n_joints_; i++)
            {
                ROS_INFO("%s", joint_names_[i].c_str());
            }
        }
        
      // 1.2 Gain
      // 1.2.1 Joint Controller
        Kp_.resize(n_joints_);
        Kd_.resize(n_joints_);
        Ki_.resize(n_joints_);

        std::vector<double> Kp(n_joints_), Ki(n_joints_), Kd(n_joints_);
        for (size_t i = 0; i < n_joints_; i++)
        {
            std::string si = boost::lexical_cast<std::string>(i + 1);
            if (n.getParam("/elfin/velocity_controller/gains/elfin_joint" + si + "/pid/p", Kp[i]))
            {
                Kp_(i) = Kp[i];
                //Kp_(i) = 0;               
            }
            else
            {
                std::cout << "/elfin/velocity_controller/gains/elfin_joint" + si + "/pid/p" << std::endl;
                ROS_ERROR("Cannot find pid/p gain");
                return false;
            }

            if (n.getParam("/elfin/velocity_controller/gains/elfin_joint" + si + "/pid/i", Ki[i]))
            {
                Ki_(i) = Ki[i];
            }
            else
            {
                ROS_ERROR("Cannot find pid/i gain");
                return false;
            }

            if (n.getParam("/elfin/velocity_controller/gains/elfin_joint" + si + "/pid/d", Kd[i]))
            {
                Kd_(i) = Kd[i];
            }
            else
            {
                ROS_ERROR("Cannot find pid/d gain");
                return false;
            }
        }
        
    // 2. URDF Model
        urdf::Model urdf;
        if (!urdf.initParam("robot_description"))
        {
            ROS_ERROR("Failed to parse urdf file");
            return false;
        }
        else
        {
            ROS_INFO("Found robot_description");
        }
        
    // 3. ********* Get the joint object to use in the realtime loop [Joint Handle, URDF] *********
        for (int i = 0; i < n_joints_; i++)
        {
            try
            {
                joints_.push_back(hw->getHandle(joint_names_[i]));
            }
            catch (const hardware_interface::HardwareInterfaceException &e)
            {
                ROS_ERROR_STREAM("Exception thrown: " << e.what());
                return false;
            }

            urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_names_[i]);
            if (!joint_urdf)
            {
                ROS_ERROR("Could not find joint '%s' in urdf", joint_names_[i].c_str());
                return false;
            }
            joint_urdfs_.push_back(joint_urdf);
        }
    
    // 4. ********* KDL *********
    // 4.1 kdl parser
    if (!kdl_parser::treeFromUrdfModel(urdf, kdl_tree_))
        {
            ROS_ERROR("Failed to construct kdl tree");
            return false;
        }
        else
        {
            ROS_INFO("Constructed kdl tree");
        }

     // 4.2 kdl chain
        std::string root_name, tip_name;
        if (!n.getParam("root_link", root_name))
        {
            ROS_ERROR("Could not find root link name");
            return false;
        }
        if (!n.getParam("tip_link", tip_name))
        {
            ROS_ERROR("Could not find tip link name");
            return false;
        }
        if (!kdl_tree_.getChain(root_name, tip_name, kdl_chain_))
        {
            ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
            ROS_ERROR_STREAM("  " << root_name << " --> " << tip_name);
            ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfJoints() << " joints");
            ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfSegments() << " segments");
            ROS_ERROR_STREAM("  The segments are:");

            KDL::SegmentMap segment_map = kdl_tree_.getSegments();
            KDL::SegmentMap::iterator it;

            for (it = segment_map.begin(); it != segment_map.end(); it++)
                ROS_ERROR_STREAM("    " << (*it).first);

            return false;
        }
        else
        {
            ROS_INFO("Got kdl chain");
        }
        
      // 4.3 inverse dynamics solver
        gravity_ = KDL::Vector::Zero();
        gravity_(2) = -9.81;            // 0: x-axis 1: y-axis 2: z-axis

        id_solver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_));
        
      // 4.4 jacobian solver
        jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_)); 
        
      // 4.5 forward kinematics solver
        fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));


      // ********* 5. Data *********      
        
      // 5.1 KDL Vectors 
        tau_d_.data = Eigen::VectorXd::Zero(n_joints_);

        qd_.data = Eigen::VectorXd::Zero(n_joints_);
        qd_dot_.data = Eigen::VectorXd::Zero(n_joints_);
        qd_ddot_.data = Eigen::VectorXd::Zero(n_joints_);
        qd_old_.data = Eigen::VectorXd::Zero(n_joints_);
        
        q_dot_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
        qd_dot_cmd_.data = Eigen::VectorXd::Zero(n_joints_);

        q_.data = Eigen::VectorXd::Zero(n_joints_);
        qdot_.data = Eigen::VectorXd::Zero(n_joints_);

        e_.data = Eigen::VectorXd::Zero(n_joints_);
        e_dot_.data = Eigen::VectorXd::Zero(n_joints_);
        e_int_.data = Eigen::VectorXd::Zero(n_joints_);
        
        Vd_jnt_.data = Eigen::VectorXd::Zero(n_joints_);
        Vcmd_jnt_.data = Eigen::VectorXd::Zero(n_joints_);

	// 5.2 Matrix 
	J_.resize(kdl_chain_.getNrOfJoints());
	Jd_.resize(kdl_chain_.getNrOfJoints());
	J_inv_.resize(kdl_chain_.getNrOfJoints());
	
        M_.resize(kdl_chain_.getNrOfJoints());
        C_.resize(kdl_chain_.getNrOfJoints());
        G_.resize(kdl_chain_.getNrOfJoints());
        
      // ********* 6. ROS *********
      // 6.1 publisher
        pub_qd_ = n.advertise<std_msgs::Float64MultiArray>("qd", 1000);
        pub_q_ = n.advertise<std_msgs::Float64MultiArray>("q", 1000);
        pub_e_ = n.advertise<std_msgs::Float64MultiArray>("e", 1000);

        pub_SaveData_ = n.advertise<std_msgs::Float64MultiArray>("SaveData", 1000); // 뒤에 숫자는?

        // 6.2 subsriber
	sub_x_cmd_ = n.subscribe<std_msgs::Float64MultiArray>("commandCB", 1, &Velocity_Controller::command, this);
	safety_ = n.subscribe<std_msgs::Bool>("allclear", 1000, &Velocity_Controller::safety, this);
	safe = true;
	event = 0;
	
    	return true;
  }

  
  void safety(const std_msgs::Bool::ConstPtr& msg)
  //void safety()
  {


    safe = !safe;
  }

  void command(const std_msgs::Float64MultiArrayConstPtr &msg)
    {
        ROS_INFO("COMMAND RECEIVED");
        /*if (msg->data.size() != num_taskspace)
        {
            ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") does not match DOF of Task Space (" << 2 << ")! Not executing!");
            return;
        }

        for (int i = 0; i < num_taskspace; i++)
        {
            x_cmd_(i) = msg->data[i];
        }

        event = 1; // subscribe 받기 전: 0
                   // subscribe 받은 후: 1*/
    }

  void starting(const ros::Time& time) 
  {
        t = 0.0;
        ROS_INFO("Starting Velocity Controller with Closed-Loop Inverse Kinematics");
  }
  
  
  void update(const ros::Time& time, const ros::Duration& period)
  { 
  // ********* 0. Get states from gazebo *********
        // 0.1 sampling time
        double dt = period.toSec();
        t = t + 0.001;

        // 0.2 joint state
        for (int i = 0; i < n_joints_; i++)
        {
            q_(i) = joints_[i].getPosition();
            qdot_(i) = joints_[i].getVelocity();
        }
	
   	joint_space = false; // control in joint space or task space
   	error_total = true; // calculate error full frame or (R,p)
	//ROS_INFO("%f, ",t);
   	
   	if (safe == true){
		//ROS_INFO("Safety = true");
	// ********* 1. Desired Trajecoty in Joint Space *********
		for (size_t i = 0; i < n_joints_; i++)
		{
		    qd_ddot_(i) = -M_PI * M_PI / 4 * 45 * KDL::deg2rad * sin(M_PI / 2 * t); 
		    qd_dot_(i) = M_PI / 2 * 45 * KDL::deg2rad * cos(M_PI / 2 * t);   
		    qd_(i) = 45 * KDL::deg2rad * sin(M_PI / 2* t);  
		}	
	}
	
	else{
		ROS_INFO("Safety = false");
		for (size_t i = 0; i < n_joints_; i++)
		{
		    qd_ddot_(i) = 0; 
		    qd_dot_(i) = 0;   
		    qd_(i) = qd_(i);  
		}
	}
	
  // ********* 1. Desired Trajecoty in Joint Space *********      
        // ********* page 2 word document
   	    fk_pos_solver_ ->JntToCart(qd_,xd_); // Desired frame
   	    jnt_to_jac_solver_ ->JntToJac(qd_,Jd_); // Desired Jacobian
   	    Vd_jnt_.data = Jd_.data  * qd_dot_.data; // Desired Twist joint
	
	// Desired Twist     
   	    for (size_t i = 0; i < n_joints_; i++) 
  	    {
   	        Vd_[i] = Vd_jnt_.data[i];
   	    }         

  // ********* kinematic controll in Joint Space *********
   	if (joint_space == true) 
   	{
   	    e_.data = qd_.data - q_.data;
   	    q_dot_cmd_.data = qd_dot_.data + Kp_.data.cwiseProduct(e_.data); 
   	
   	    for (int i = 0; i < n_joints_; i++)
            {
               printf("%d, ", i);
               printf("q_dot_cmd:");
               printf("%f, ", q_dot_cmd_(i));
               printf("\n");
               printf("qdot_:");
               printf("%f, ", qd_dot_(i));
               printf("\n");

            }
   	}
 
 
  // ********* kinematic controll in Task Space *********
  // page 4 Word document
   	else
   	{
   	    fk_pos_solver_ ->JntToCart(q_,x_); // current frame


   	    // full frame error
   	    if (error_total == true)
   	    {
   		Xerr_ = diff(x_,xd_,1);
   	    }
   	    
   	    // frame error (R,p) separated
   	    else {
   		Xerr_.rot = diff(x_.M, xd_.M);
   		Xerr_.vel = diff(x_.p, xd_.p);
   	    }
   	    
   	    // Twist 
   	    Vcmd_ = Vd_ + 1.0*Xerr_;  // replace gain = 1.0 with a matrix?
   	    // convert Twist to Twist_joint
   	    
   	    for (size_t i = 0; i < n_joints_; i++) {
   		Vcmd_jnt_.data[i] = Vcmd_[i];
   	    }
   	    
  	    // Jacobian and Jacobian inverse
	    jnt_to_jac_solver_ ->JntToJac(q_,J_);
    	    
    	    J_inv_.data = J_.data.inverse();  	
   	    // 
   	    q_dot_cmd_.data = J_inv_.data * Vcmd_jnt_.data;
     
   	}
    
             
  // ********* 2. Motion Controller in Joint Space *********
  // 2.1 Error Definition in Joint Space
        e_.data = qd_.data - q_.data;
        //e_dot_.data = qd_dot_.data - qdot_.data; 
        e_dot_.data = q_dot_cmd_.data - qdot_.data; 
        
  // 2.2 Compute model(M,C,G)
        id_solver_->JntToMass(q_, M_);
        id_solver_->JntToCoriolis(q_, qdot_, C_);
        id_solver_->JntToGravity(q_, G_); 
        
  // 2.3 Apply Torque Command to Actuator
        aux_d_.data = M_.data * (qd_ddot_.data + Kp_.data.cwiseProduct(e_.data) + Kd_.data.cwiseProduct(e_dot_.data));
        comp_d_.data = C_.data + G_.data;
        tau_d_.data = aux_d_.data + comp_d_.data;
        
        for (int i = 0; i < n_joints_; i++)
        {
           joints_[i].setCommand(tau_d_(i));

        }
        
  }
  
  void stopping(const ros::Time& time) 
  {
  
  }

  private:
  double t;
  bool joint_space, error_total,safe;
  int n_joints_, event;
  std::vector<std::string> joint_names_;                // joint name ??
  std::vector<hardware_interface::JointHandle> joints_; // ??
  std::vector<urdf::JointConstSharedPtr> joint_urdfs_;  // ??
  
  // gains
  KDL::JntArray Kp_, Ki_, Kd_;

  // kdl
  KDL::Tree kdl_tree_;   // tree?
  KDL::Chain kdl_chain_; // chain?
  
  // kdl M,C,G
  KDL::JntSpaceInertiaMatrix M_; // intertia matrix
  KDL::JntArray C_;              // coriolis
  KDL::JntArray G_;              // gravity torque vector
  KDL::Vector gravity_;
  
  // kdl and Eigen Jacobian
  KDL::Jacobian J_;
  KDL::Jacobian Jd_;
  KDL::Jacobian J_inv_;
  //Eigen::MatrixXd J_inv_;


  // kdl solver
  boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_; // Solver to compute the forward kinematics (position)
  boost::scoped_ptr<KDL::ChainDynParam> id_solver_;                  // Solver To compute the inverse dynamics
  boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;    // Solver to compute the jacobian
  boost::scoped_ptr<KDL::ChainFkSolverVel_recursive> fk_vel_solver_; //Solver to compute the forward kinematics (velocity)
  
  // ros publisher
  ros::Publisher pub_qd_, pub_q_, pub_e_;
  ros::Publisher pub_SaveData_;
  
  // ros subscriber
  ros::Subscriber sub_x_cmd_;
  ros::Subscriber safety_;

  // ros message
  std_msgs::Float64MultiArray msg_qd_, msg_q_, msg_e_;
  std_msgs::Float64MultiArray msg_SaveData_;
  
  // Joint Space State
  KDL::JntArray qd_, qd_dot_, qd_ddot_;
  KDL::JntArray qd_old_;
  KDL::JntArray q_, qdot_;
  KDL::JntArray q_dot_cmd_, qd_dot_cmd_;
  KDL::JntArray e_, e_dot_, e_int_;
  KDL::JntArray Vd_jnt_, Vcmd_jnt_;
 
  
  // Task Space State
  KDL::Frame xd_; // x.p: frame position(3x1), x.m: frame orientation (3x3)
  KDL::Frame x_;
  KDL::Twist Xerr_;
  KDL::Twist Vd_;
  KDL::Twist Vcmd_;
 
  // Input
  KDL::JntArray aux_d_;
  KDL::JntArray comp_d_;
  KDL::JntArray tau_d_;

};
}//namespace
PLUGINLIB_EXPORT_CLASS(arm_controllers::Velocity_Controller, controller_interface::ControllerBase)

