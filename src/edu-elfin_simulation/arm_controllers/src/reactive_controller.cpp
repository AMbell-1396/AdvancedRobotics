// from ros-control meta packages
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

#include <urdf/model.h>

// from kdl packages
#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chaindynparam.hpp>              // inverse dynamics
#include <kdl/chainjnttojacsolver.hpp>        // jacobian
#include <kdl/chainfksolverpos_recursive.hpp> // forward kinematics

#include <boost/scoped_ptr.hpp>
#include <boost/lexical_cast.hpp>

#include <math.h>
#include <Eigen/LU>
#include <utils/pseudo_inversion.h>
#include <utils/skew_symmetric.h>

#include <string>
#include <fstream>
#include <iostream>
#include <dirent.h>

#define PI 3.141592
#define D2R PI / 180.0
#define R2D 180.0 / PI
#define SaveDataMax 55

namespace arm_controllers
{
class reactive_controller : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
  public:
    bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &n)
    {
        t_set = 1;
        joint_space = true;
        control_params = {0,0,0,0,0,0};
        

        // ********* 1. Get joint name / gain from the parameter server *********
        // 1.1 Joint Name
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
            if (n.getParam("/elfin/reactive_controller/gains/elfin_joint" + si + "/pid/p", Kp[i]))
            {
                Kp_(i) = Kp[i];
            }
            else
            {
                std::cout << "/elfin/reactive_controller/gains/elfin_joint" + si + "/pid/p" << std::endl;
                ROS_ERROR("Cannot find pid/p gain");
                return false;
            }

            if (n.getParam("/elfin/reactive_controller/gains/elfin_joint" + si + "/pid/i", Ki[i]))
            {
                Ki_(i) = Ki[i];
            }
            else
            {
                ROS_ERROR("Cannot find pid/i gain");
                return false;
            }

            if (n.getParam("/elfin/reactive_controller/gains/elfin_joint" + si + "/pid/d", Kd[i]))
            {
                Kd_(i) = Kd[i];
            }
            else
            {
                ROS_ERROR("Cannot find pid/d gain");
                return false;
            }
        }

        // 2. ********* urdf *********
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

        // 4.3 inverse dynamics solver 초기화
        gravity_ = KDL::Vector::Zero(); // ?
        gravity_(2) = -9.81;            // 0: x-axis 1: y-axis 2: z-axis

        id_solver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_));

        // 4.4 forward kinematics solver
        fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));

        // 4.5 jacobian solver 초기화
        jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));

        // ********* 5. 각종 변수 초기화 *********

        // 5.1 Vector 초기화 (사이즈 정의 및 값 0)
        tau_d_.data = Eigen::VectorXd::Zero(n_joints_);

        qd_.data = Eigen::VectorXd::Zero(n_joints_);
        qd_dot_.data = Eigen::VectorXd::Zero(n_joints_);
        qd_ddot_.data = Eigen::VectorXd::Zero(n_joints_);
        qd_old_.data = Eigen::VectorXd::Zero(n_joints_);
        qd_dot_old_.data = Eigen::VectorXd::Zero(n_joints_);

        q_.data = Eigen::VectorXd::Zero(n_joints_);
        qdot_.data = Eigen::VectorXd::Zero(n_joints_);

        e_.data = Eigen::VectorXd::Zero(n_joints_);
        e_dot_.data = Eigen::VectorXd::Zero(n_joints_);
        e_int_.data = Eigen::VectorXd::Zero(n_joints_);

        // 5.2 Matrix 초기화 (사이즈 정의 및 값 0)
        J_.resize(kdl_chain_.getNrOfJoints());
        Jd_.resize(kdl_chain_.getNrOfJoints());
        M_.resize(kdl_chain_.getNrOfJoints());
        C_.resize(kdl_chain_.getNrOfJoints());
        G_.resize(kdl_chain_.getNrOfJoints());

        // ********* 6. ROS 명령어 *********
        // 6.1 publisher
        pub_qd_ = n.advertise<std_msgs::Float64MultiArray>("qd", 1000);
        pub_q_ = n.advertise<std_msgs::Float64MultiArray>("q", 1000);
        pub_e_ = n.advertise<std_msgs::Float64MultiArray>("e", 1000);

        pub_SaveData_ = n.advertise<std_msgs::Float64MultiArray>("SaveData", 1000); // 뒤에 숫자는?

        pub_Frame_as_Twist_ = n.advertise<geometry_msgs::Twist>("FrameAsTwist", 1000);

        // 6.2 subsriber
        sub_x_cmd_ = n.subscribe<std_msgs::Float64MultiArray>("command", 1, &reactive_controller::commandCB, this);
        event = 0; // subscribe 받기 전: 0
                   // subscribe 받은 후: 1
        
        //alternative subscribe method that accepts member functions and the corresponding object 
        //sub_parse_command_ = n.subscribe<geometry_msgs::Twist>("cmdTwist", 10, boost::bind(&reactive_controller::cmdParser, this, _1)); //topic, queue length, callback handle, instance Object
        sub_parse_command_ = n.subscribe<std_msgs::String>("cmdString", 1, boost::bind(&reactive_controller::cmdStringParser, this, _1)); //topic, queue length, callback handle, instance Object

        //subscriber is not a member function & gets dropped after function (init) execution closes.


        n.getParam("/elfin/reactive_controller/reac_gain/K_UNIVERSAL", K_universal_);
 
        return true;
    }

    void cmdStringParser(const std_msgs::String::ConstPtr& msg)
    {
        const std::string s = msg->data.c_str();
        
        std::vector<std::string> parts;
        const char delim = ';';

        std::string::size_type beg = 0;
        for (auto end = 0; (end = s.find(delim, end)) != std::string::npos; ++end)
        {
            parts.push_back(s.substr(beg, end - beg));
            beg = end + 1;
        }
        
        parts.push_back(s.substr(beg));
        
        control_params[0] = std::stof(parts[0]);
        control_params[1] = std::stof(parts[1]);
        control_params[2] = std::stof(parts[2]);
        control_params[3] = std::stof(parts[3]);
        control_params[4] = std::stof(parts[4]);
        control_params[5] = std::stof(parts[5]);

        joint_space = false;
    }


    // the following func doesn't work because of implicit BOOST double->float conversion (?)
    /*
    void cmdParser(const geometry_msgs::Twist::ConstPtr& msg)
    {
        std::cout << "cmdCallback";

        //control_params (global) data: [xd.p(1), xd.p(2), xd.p(3), xd.M(1), xd.M(2), xd.M(3)]

        xd_.p(1) = msg->linear.x;
        xd_.p(2) = msg->linear.y;
        xd_.p(3) = msg->linear.z;
        KDL::Rotation(KDL::Rotation::RPY(msg->angular.x, msg->angular.x, msg->angular.z));
        
        //reset if all cmd = 0 (unknown cases)
        if (msg->linear.x == msg->linear.y == msg->linear.z == msg->angular.x == msg->angular.y == msg->angular.z == 0) {
            std::vector<float> control_params;
        }
        else {
            control_params[0] = msg->linear.x;
            control_params[1] = msg->linear.y;
            control_params[2] = msg->linear.z;
            control_params[3] = msg->angular.x;
            control_params[4] = msg->angular.y;
            control_params[5] = msg->angular.z;
        }
    }
    */

    //read text file content & return first line
    std::string textPasser(std::string path) 
    {
        std::string line;
        std::ifstream file(path);
        if (file.is_open()) {
            std::getline(file, line);
            

            file.close();
        } else {
            std::cout << "Couldn't open file: " << path << '\n';
        }
        return line;
    }

    void commandCB(const std_msgs::Float64MultiArrayConstPtr &msg)
    {
        if (msg->data.size() != 6)
        {
            ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") does not match DOF of Task Space (" << 2 << ")! Not executing!");
            return;
        }

        for (int i = 0; i < 6; i++)
        {
            x_cmd_(i) = msg->data[i];
        }

        event = 1; // subscribe 받기 전: 0
                   // subscribe 받은 후: 1
    }

    void starting(const ros::Time &time)
    {
        t = 0.0;
        ROS_INFO("Starting Velocity Controller");
    }
    void update(const ros::Time &time, const ros::Duration &period)
    {
        // ********* 0. Get states from gazebo *********
        // 0.1 sampling time
        //double dt = period.toSec();
        double dt = 0.001;  //sim period = 1000Hz
        t = t + dt;

        if (t < t_set) {
            xd_.p(0) = 0.0;
            xd_.p(1) = 0.0;
            xd_.p(2) = 0.0;
            xd_.M = KDL::Rotation(KDL::Rotation::RPY(0, 0, 0));
        }

        // 0.2 joint state
        for (int i = 0; i < n_joints_; i++)
        {
            q_(i) = joints_[i].getPosition();
            qdot_(i) = joints_[i].getVelocity();
        }
        // 0.3 end-effector state by Compute forward kinematics (x_,xdot_)
        fk_pos_solver_->JntToCart(q_, x_);

        xdot_ = J_.data * qdot_.data;

        // ********* 2. Motion Controller in Joint Space*********

        if (readf_timer % 1000 == 0) {
            //task_string = textPasser("/home/advrob/elfin_ws/src/edu-elfin_simulation/arm_controllers/src/burger_key.txt"); // via burger
            control_mode = textPasser("/home/advrob/elfin_ws/src/edu-elfin_simulation/arm_controllers/src/control_mode.txt"); // control mode
            readf_timer = 0;
        }
        ++readf_timer;

        if (!joint_space)
        {
            joint_space = false;

            //convert string control params to double
            xd_.p(0) = control_params[0];
            xd_.p(1) = control_params[1];
            xd_.p(2) = control_params[2];
            xd_.M = KDL::Rotation(KDL::Rotation::RPY(control_params[3], control_params[4], control_params[5]));
        }
        
        else 
        {
            joint_space = true;
            for (size_t i = 0; i < n_joints_; i++)
            {
                // sin oscillation
                qd_ddot_(i) = -M_PI * M_PI / 4 * 45 * KDL::deg2rad * sin(M_PI / 2); 
                qd_dot_(i) = M_PI / 2 * 45 * KDL::deg2rad * cos(M_PI / 2);          
                qd_(i) = 45 * KDL::deg2rad * sin(M_PI / 2);
            }
        }
        /*
        {
            joint_space = true;
            // ********* Desired Trajectory in Joint Space *********
            for (size_t i = 0; i < n_joints_; i++)
            {
                // sin oscillation
                qd_ddot_(i) = -M_PI * M_PI / 4 * 45 * KDL::deg2rad * sin(M_PI / 2 * t); 
                qd_dot_(i) = M_PI / 2 * 45 * KDL::deg2rad * cos(M_PI / 2 * t);          
                qd_(i) = 45 * KDL::deg2rad * sin(M_PI / 2* t);
            }
        }
        */

        // CHOOSE CALC FOR WEIGHT TRIM
        if (!joint_space) 
        {
            ex_temp_ = diff(x_, xd_, dt); // error in task-space (full)

            jnt_to_jac_solver_->JntToJac(q_, J_);   // Jacobian from joint space
            //J_transpose_ = J_.data.transpose();     // unused
            J_inv_ = J_.data.inverse();             // Jinv
            /*
            int sum = 0;
            for (int i=0; i < 6; ++i)
            {
                sum += J_inv_(i,i);
            }
            if (sum < 0.1)
            {
                pseudo_inverse(J_.data, J_inv_, false);
            }
            */

            //Task space control based on xd
            //ex_temp_.rot = diff(x_.M, xd_.M);
            //ex_temp_.vel = diff(x_.p, xd_.p)

            for (size_t i = 0; i < n_joints_; i++)
            {
                ex_(i) = ex_temp_(i);
                xd_dot_(i) = 1 * ex_temp_(i);
                xd_ddot_(i) = 1 * ex_temp_(i);
            }
            //closed loop inv kinematics with xd_dot
            //qd_.data = qd_old_.data + J_inv_ * (xd_dot_ + K_universal_ * ex_) * dt;
            
            qd_.data = (qd_old_.data * 0)+ J_inv_ * K_universal_ * ex_ * dt;
            qd_dot_.data = J_inv_ * (xd_dot_ + K_universal_ * ex_);
            qd_ddot_.data = (qd_dot_.data - qd_dot_old_.data) / dt; // qdd = qd d/dt
            /*
            for (size_t i = 0; i < n_joints_; i++)
            {
                qd_dot_(i) = wrapToPi(qd_dot_(i));
                qd_ddot_(i) = wrapToPi(qd_ddot_(i));
            }
            */

            qd_old_.data = qd_.data;
            qd_dot_old_.data = qd_dot_.data;     // unused
        }
        //Joint space
        // Error def in joint space
        e_.data = qd_.data - q_.data;
        e_dot_.data = qd_dot_.data - qdot_.data;
        e_int_.data = qd_.data - q_.data;           // (To do: e_int 업데이트 필요요)

        // *** 2.3 Compute model(M,C,G) ***
        id_solver_->JntToMass(q_, M_);
        id_solver_->JntToCoriolis(q_, qdot_, C_);
        id_solver_->JntToGravity(q_, G_); 
    

        // *** 2.4 Apply Torque Command to Actuator ***

        // USE VEL OR FULL CONTROLLER
        if (control_mode == "velocity")
        {
            // use velocity control
            aux_d_.data = M_.data * (qd_ddot_.data + Kd_.data.cwiseProduct(e_dot_.data));
        }
        else 
        {
            // default behaviour
            aux_d_.data = M_.data * (qd_ddot_.data + Kp_.data.cwiseProduct(e_.data) + Kd_.data.cwiseProduct(e_dot_.data));
        }
        comp_d_.data = C_.data + G_.data;
        tau_d_.data = aux_d_.data + comp_d_.data;

        for (int i = 0; i < n_joints_; i++)
        {
            joints_[i].setCommand(tau_d_(i));
        }

        // ********* 3. data 저장 *********
        save_data();

        // ********* 4. state 출력 *********
        //print_state();
    }

    double wrapToPi(double x) 
    {
        int x_mult = 1;
        if (x < 0) {
            x = -x;
            x_mult = -1;
        }
        x = fmod(x,2*PI);
        if (x<0)
            x += 2*PI;
        return x_mult * (x);
    }

    void stopping(const ros::Time &time)
    {
    }

    void save_data()
    {
        // 1
        // Simulation time (unit: sec)
        SaveData_[0] = t;

        // Desired position in joint space (unit: rad)
        SaveData_[1] = qd_(0);
        SaveData_[2] = qd_(1);
        SaveData_[3] = qd_(2);
        SaveData_[4] = qd_(3);
        SaveData_[5] = qd_(4);
        SaveData_[6] = qd_(5);

        // Desired velocity in joint space (unit: rad/s)
        SaveData_[7] = qd_dot_(0);
        SaveData_[8] = qd_dot_(1);
        SaveData_[9] = qd_dot_(2);
        SaveData_[10] = qd_dot_(3);
        SaveData_[11] = qd_dot_(4);
        SaveData_[12] = qd_dot_(5);

        // Desired acceleration in joint space (unit: rad/s^2)
        SaveData_[13] = qd_ddot_(0);
        SaveData_[14] = qd_ddot_(1);
        SaveData_[15] = qd_ddot_(2);
        SaveData_[16] = qd_ddot_(3);
        SaveData_[17] = qd_ddot_(4);
        SaveData_[18] = qd_ddot_(5);

        // Actual position in joint space (unit: rad)
        SaveData_[19] = q_(0);
        SaveData_[20] = q_(1);
        SaveData_[21] = q_(2);
        SaveData_[22] = q_(3);
        SaveData_[23] = q_(4);
        SaveData_[24] = q_(5);

        // Actual velocity in joint space (unit: rad/s)
        SaveData_[25] = qdot_(0);
        SaveData_[26] = qdot_(1);
        SaveData_[27] = qdot_(2);
        SaveData_[28] = qdot_(3);
        SaveData_[29] = qdot_(4);
        SaveData_[30] = qdot_(5);

        // Error position in joint space (unit: rad)
        SaveData_[31] = e_(0);
        SaveData_[32] = e_(1);
        SaveData_[33] = e_(2);
        SaveData_[34] = e_(3);
        SaveData_[35] = e_(4);
        SaveData_[36] = e_(5);

        // Error velocity in joint space (unit: rad/s)
        SaveData_[37] = e_dot_(0);
        SaveData_[38] = e_dot_(1);
        SaveData_[39] = e_dot_(2);
        SaveData_[40] = e_dot_(3);
        SaveData_[41] = e_dot_(4);
        SaveData_[42] = e_dot_(5);

        // Error intergal value in joint space (unit: rad*sec)
        SaveData_[43] = e_int_(0);
        SaveData_[44] = e_int_(1);
        SaveData_[45] = e_int_(2);
        SaveData_[46] = e_int_(3);
        SaveData_[47] = e_int_(4);
        SaveData_[48] = e_int_(5);

        // Current Position
        SaveData_[49] = x_.p(0);
        SaveData_[50] = x_.p(1);
        SaveData_[51] = x_.p(2);

        // Desired Position
        SaveData_[52] = xd_.p(0);
        SaveData_[53] = xd_.p(1);
        SaveData_[54] = xd_.p(2);

        // 2
        msg_qd_.data.clear();
        msg_q_.data.clear();
        msg_e_.data.clear();

        msg_SaveData_.data.clear();

        // 3
        for (int i = 0; i < n_joints_; i++)
        {
            msg_qd_.data.push_back(qd_(i));
            msg_q_.data.push_back(q_(i));
            msg_e_.data.push_back(e_(i));
        }

        for (int i = 0; i < SaveDataMax; i++)
        {
            msg_SaveData_.data.push_back(SaveData_[i]);
        }

        // 4
        pub_qd_.publish(msg_qd_);
        pub_q_.publish(msg_q_);
        pub_e_.publish(msg_e_);

        pub_SaveData_.publish(msg_SaveData_);
    }

    void print_state()
    {
        static int count = 0;
        if (count > 99)
        {
            printf("*********************************************************\n\n");
            printf("*** Simulation Time (unit: sec)  ***\n");
            printf("t = %f\n", t);
            printf("\n");

            printf("*** Desired State in Joint Space (unit: rad) ***\n");
            /*
            printf("qd_(0): %f, ", qd_(0)*R2D);
            printf("qd_(1): %f, ", qd_(1)*R2D);
            printf("qd_(2): %f, ", qd_(2)*R2D);
            printf("qd_(3): %f, ", qd_(3)*R2D);
            printf("qd_(4): %f, ", qd_(4)*R2D);
            printf("qd_(5): %f\n", qd_(5)*R2D);
            */
            printf("qd_(0): %f, ", qd_(0));
            printf("qd_(1): %f, ", qd_(1));
            printf("qd_(2): %f, ", qd_(2));
            printf("qd_(3): %f, ", qd_(3));
            printf("qd_(4): %f, ", qd_(4));
            printf("qd_(5): %f\n", qd_(5));
            printf("\n");

            printf("*** Actual State in Joint Space (unit: rad) ***\n");
            /*
            printf("q_(0): %f, ", q_(0) * R2D);
            printf("q_(1): %f, ", q_(1) * R2D);
            printf("q_(2): %f, ", q_(2) * R2D);
            printf("q_(3): %f, ", q_(3) * R2D);
            printf("q_(4): %f, ", q_(4) * R2D);
            printf("q_(5): %f\n", q_(5) * R2D);
            */
            printf("q_(0): %f, ", q_(0));
            printf("q_(1): %f, ", q_(1));
            printf("q_(2): %f, ", q_(2));
            printf("q_(3): %f, ", q_(3));
            printf("q_(4): %f, ", q_(4));
            printf("q_(5): %f\n", q_(5));
            printf("\n");


            printf("*** Joint Space Error (unit: rad)  ***\n");
            /*
            printf("%f, ", R2D * e_(0));
            printf("%f, ", R2D * e_(1));
            printf("%f, ", R2D * e_(2));
            printf("%f, ", R2D * e_(3));
            printf("%f, ", R2D * e_(4));
            printf("%f\n", R2D * e_(5));
            */
            printf("%f, ", e_(0));
            printf("%f, ", e_(1));
            printf("%f, ", e_(2));
            printf("%f, ", e_(3));
            printf("%f, ", e_(4));
            printf("%f\n", e_(5));
            printf("\n");

            if (!joint_space) 
            {
                printf("*** Current Pose in Task Space\n");
                printf("%f, ", x_.p(0));
                printf("%f, ", x_.p(1));
                printf("%f, ", x_.p(2));

                printf("*** Desired Pos in Task Space: \n");
                printf("%f, ", xd_.p(0));
                printf("%f, ", xd_.p(1));
                printf("%f, ", xd_.p(2));
            }

            count = 0;
        }
        count++;
    }

  private:
    // others
    double t;
    int event;
    int readf_timer;
    int t_set;
    char text;
    std::string task_string;
    std::string control_mode;
    std::vector<float> control_params;
    bool joint_space;

    //Joint handles
    unsigned int n_joints_;                               // joint 숫자
    std::vector<std::string> joint_names_;                // joint name ??
    std::vector<hardware_interface::JointHandle> joints_; // ??
    std::vector<urdf::JointConstSharedPtr> joint_urdfs_;  // ??

    // kdl
    KDL::Tree kdl_tree_;   // tree?
    KDL::Chain kdl_chain_; // chain?
    KDL::Jacobian J_, Jd_;
    Eigen::MatrixXd J_inv_;
    Eigen::Matrix<double, 6, 6> J_transpose_;
    //KDL::Jacobian J_inv_, J_transpose_;
    //Eigen::Matrix<double, 6, 6> J_transpose_;

    // kdl M,C,G
    KDL::JntSpaceInertiaMatrix M_; // intertia matrix
    KDL::JntArray C_;              // coriolis
    KDL::JntArray G_;              // gravity torque vector
    KDL::Vector gravity_;

    // kdl solver
    boost::scoped_ptr<KDL::ChainDynParam> id_solver_;                   // Solver To compute the inverse dynamics
    boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;     // Solver to compute the jacobian
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;  // Solver to compute the forward kinematics (position)

    // Joint Space State
    KDL::JntArray qd_, qd_dot_, qd_ddot_;
    KDL::JntArray qd_old_, qd_dot_old_;
    KDL::JntArray q_, qdot_;
    KDL::JntArray e_, e_dot_, e_int_;

    KDL::Twist Vd_;
    KDL::JntArray Vd_jnt_;

    // Task Space State
    KDL::Frame xd_; // x.p: frame position(3x1), x.m: frame orientation (3x3)
    KDL::Frame x_;
    KDL::Twist ex_temp_;

    Eigen::Matrix<double, 6, 1> ex_;
    Eigen::Matrix<double, 6, 1> xd_dot_, xd_ddot_;
    Eigen::Matrix<double, 6, 1> xdot_;
    Eigen::Matrix<double, 6, 1> ex_dot_, ex_int_;

    double K_universal_;


    // Input
    KDL::JntArray aux_d_;
    KDL::JntArray comp_d_;
    KDL::JntArray tau_d_;
    KDL::JntArray x_cmd_;

    // gains
    KDL::JntArray Kp_, Ki_, Kd_;

    // save the data
    double SaveData_[SaveDataMax];

    // ros publisher
    ros::Publisher pub_qd_, pub_q_, pub_e_;
    ros::Publisher pub_SaveData_;
    ros::Publisher pub_Frame_as_Twist_;

    // ros message
    std_msgs::Float64MultiArray msg_qd_, msg_q_, msg_e_;
    std_msgs::Float64MultiArray msg_SaveData_;

    // ros cmd subscriber
    ros::Subscriber sub_x_cmd_, sub_parse_command_;
};
}; // namespace arm_controllers
PLUGINLIB_EXPORT_CLASS(arm_controllers::reactive_controller, controller_interface::ControllerBase)