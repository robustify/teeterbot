#pragma once

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs.hh>

#include <teeterbot_gazebo/DcMotorSim.hpp>
#include <teeterbot_gazebo/MotorController.hpp>

using namespace gz;
using namespace sim;

namespace teeterbot_gazebo {

    class TeeterbotInterfacePlugin
    : public System,
        public ISystemConfigure,
        public ISystemPreUpdate,
        public ISystemPostUpdate
    {
        public:
            TeeterbotInterfacePlugin();
            ~TeeterbotInterfacePlugin() override;

            void Configure(const Entity & _entity, const std::shared_ptr<const sdf::Element> & _sdf, EntityComponentManager & _ecm, EventManager & _eventMgr) override;

            void PreUpdate( const UpdateInfo & _info, EntityComponentManager & _ecm) override;

            void PostUpdate(const UpdateInfo & _info, const EntityComponentManager & _ecm) override;

        private:
            Model model_;
            Entity left_joint_;
            Entity right_joint_;
            transport::Node node_;
            transport::Node::Publisher pub_left_speed_;
            transport::Node::Publisher pub_right_speed_;
            transport::Node::Publisher pub_fallen_over_;

            double left_speed_cmd_ = 0.0;
            double right_speed_cmd_ = 0.0;
            double left_torque_cmd_ = 0.0;
            double right_torque_cmd_ = 0.0;
            bool torque_mode_ = false;
            uint64_t speed_stamp_ = 0;
            uint64_t torque_stamp_ = 0;
            uint64_t encoder_pub_stamp_ = 0;
            uint64_t current_time_ = 0;
            bool fallen_over_ = false;
            static constexpr double FALLEN_OVER_THRES = (M_PI / 3.0);
            static constexpr double ENCODER_SAMPLE_TIME = 0.01;

            // DC motor simulation instances
            teeterbot_gazebo::DcMotorSim::SharedPtr left_motor_;
            teeterbot_gazebo::DcMotorSim::SharedPtr right_motor_;

            // Controller instances
            teeterbot_gazebo::MotorController::SharedPtr left_speed_control_;
            teeterbot_gazebo::MotorController::SharedPtr right_speed_control_;
            teeterbot_gazebo::MotorController::SharedPtr left_torque_control_;
            teeterbot_gazebo::MotorController::SharedPtr right_torque_control_;

            void recvLeftSpeedCmd(const gz::msgs::Double& msg);
            void recvRightSpeedCmd(const gz::msgs::Double& msg);
            void recvLeftTorqueCmd(const gz::msgs::Double& msg);
            void recvRightTorqueCmd(const gz::msgs::Double& msg);

    };

}
