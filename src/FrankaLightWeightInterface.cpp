#include "franka_lightweight_interface/FrankaLightWeightInterface.hpp"

namespace frankalwi
{
    FrankaLightWeightInterface::FrankaLightWeightInterface(const std::string & robot_ip):
    robot_ip_(robot_ip), connected_(false), shutdown_(false)
    {}

    void FrankaLightWeightInterface::init()
    {
        // create connection to the robot
        this->franka_robot_ = std::make_unique<franka::Robot>(this->robot_ip_);
        this->franka_model_ = std::make_unique<franka::Model>(this->franka_robot_->loadModel());
        this->connected_ = true;

        this->current_cartesian_twist_.setZero();
        this->current_cartesian_wrench_.setZero();
        this->current_joint_positions_.setZero();
        this->current_joint_velocities_.setZero();
        this->current_joint_torques_.setZero();
        this->command_joint_torques_.setZero();
    }

    void FrankaLightWeightInterface::run_controller()
    {
        if(this->is_connected())
        {
            // restart the controller unless the node is shutdown
            while(!this->is_shutdown())
            {
                std::cout << "Starting controller..." << std::endl;
                try
                {
                    this->run_joint_torques_controller();
                }
                catch (const franka::CommandException& e)
                {
                    std::cerr << e.what() << std::endl;
                }
                std::cerr << "Controller stopped but the node is still active, restarting..." << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }
        else
        {
            std::cerr << "Robot not connected firs call the init function" << std::endl;
        }
    }

    void FrankaLightWeightInterface::read_robot_state(const franka::RobotState& robot_state)
    {
        // extract cartesian info
        Eigen::Affine3d eef_transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
        this->current_cartesian_position_ = eef_transform.translation();
        this->current_cartesian_orientation_ = Eigen::Quaterniond(eef_transform.linear());
        this->current_cartesian_wrench_ = Eigen::VectorXd::Map(robot_state.O_F_ext_hat_K.data(), robot_state.O_F_ext_hat_K.size());
        // extract joint info
        this->current_joint_positions_ = Eigen::VectorXd::Map(robot_state.q.data(), robot_state.q.size());
        this->current_joint_velocities_ = Eigen::VectorXd::Map(robot_state.dq.data(), robot_state.q.size());
        this->current_joint_torques_ = Eigen::VectorXd::Map(robot_state.tau_J.data(), robot_state.q.size());
        // extract jacobian
        std::array<double, 42> jacobian_array = this->franka_model_->zeroJacobian(franka::Frame::kEndEffector, robot_state);
        this->current_jacobian_ = Eigen::Map<const Eigen::Matrix<double, 6, 7> >(jacobian_array.data());
        // get the twist from jacobian and current joint velocities
        this->current_cartesian_twist_ = this->current_jacobian_ * this->current_joint_velocities_;

        this->print_state();
    }

    void FrankaLightWeightInterface::run_joint_torques_controller()
    {
        // Set additional parameters always before the control loop, NEVER in the control loop!
        // Set collision behavior.
        this->franka_robot_->setCollisionBehavior(
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

        // Damping
        Eigen::ArrayXd d_gains = Eigen::ArrayXd(7);
        d_gains << 25.0, 25.0, 25.0, 25.0, 15.0, 15.0, 5.0;

        try
        {
            this->franka_robot_->control([this, d_gains](const franka::RobotState& robot_state, franka::Duration) -> franka::Torques
            {
                // lock mutex
                std::lock_guard<std::mutex> lock(this->get_mutex());
                // extract current state
                this->read_robot_state(robot_state);

                // get the coriolis array
                std::array<double, 7> coriolis_array = this->franka_model_->coriolis(robot_state);
                Eigen::Map<const Eigen::Matrix<double, 7, 1> > coriolis(coriolis_array.data());

                std::array<double, 7> torques{};
                Eigen::VectorXd::Map(&torques[0], 7) = this->command_joint_torques_.array()
                                                       - d_gains * current_joint_velocities_.array()
                                                       + coriolis.array();
                //return torques;
                return torques;
            });
        }
        catch (const franka::Exception& e)
        {
            std::cerr << e.what() << std::endl;
        }
    }
}