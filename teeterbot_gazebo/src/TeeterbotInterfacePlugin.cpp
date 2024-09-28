#include <teeterbot_gazebo/TeeterbotInterfacePlugin.hpp>

#include <gz/plugin/Register.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/ParentEntity.hh>

using namespace gz;
using namespace gz::sim;
using namespace systems;

namespace teeterbot_gazebo {
    math::Pose3d worldPose(const Entity &_entity, const EntityComponentManager &_ecm) {
        auto poseComp = _ecm.Component<components::Pose>(_entity);
        if (nullptr == poseComp) {
            gzwarn << "Trying to get world pose from entity [" << _entity
                    << "], which doesn't have a pose component" << std::endl;
            return math::Pose3d();
        }

        // work out pose in world frame
        math::Pose3d pose = poseComp->Data();
        auto p = _ecm.Component<components::ParentEntity>(_entity);
        while (p) {
            // get pose of parent entity
            auto parentPose = _ecm.Component<components::Pose>(p->Data());
            if (!parentPose)
                break;
            // transform pose
            pose = parentPose->Data() * pose;
            // keep going up the tree
            p = _ecm.Component<components::ParentEntity>(p->Data());
        }
        return pose;
    }

    TeeterbotInterfacePlugin::TeeterbotInterfacePlugin() {}
    TeeterbotInterfacePlugin::~TeeterbotInterfacePlugin() {}

    void TeeterbotInterfacePlugin::Configure(const Entity &_entity,
                          const std::shared_ptr<const sdf::Element> &_sdf,
                          EntityComponentManager &_ecm,
                          EventManager &/*_eventMgr*/)
    {
        this->model_ = Model(_entity);
        if (!this->model_.Valid(_ecm)) {
            gzerr << "Teeterbot system plugin should be attached to a model"
                  << " entity. Failed to initialize." << std::endl;
            return;
        }

        this->left_joint_ = this->model_.JointByName(_ecm, "left_wheel_joint");
        this->right_joint_ = this->model_.JointByName(_ecm, "right_wheel_joint");
        this->left_motor_ = std::make_shared<teeterbot_gazebo::DcMotorSim>();
        this->right_motor_ = std::make_shared<teeterbot_gazebo::DcMotorSim>();
        this->left_speed_control_ = std::make_shared<teeterbot_gazebo::MotorController>(teeterbot_gazebo::SPEED_PARAMS);
        this->right_speed_control_ = std::make_shared<teeterbot_gazebo::MotorController>(teeterbot_gazebo::SPEED_PARAMS);
        this->left_torque_control_ = std::make_shared<teeterbot_gazebo::MotorController>(teeterbot_gazebo::TORQUE_PARAMS);
        this->right_torque_control_ = std::make_shared<teeterbot_gazebo::MotorController>(teeterbot_gazebo::TORQUE_PARAMS);

        this->node_.Subscribe("/model/teeterbot/left_speed_cmd", &TeeterbotInterfacePlugin::recvLeftSpeedCmd, this);
        this->node_.Subscribe("/model/teeterbot/right_speed_cmd", &TeeterbotInterfacePlugin::recvRightSpeedCmd, this);
        this->node_.Subscribe("/model/teeterbot/left_torque_cmd", &TeeterbotInterfacePlugin::recvLeftTorqueCmd, this);
        this->node_.Subscribe("/model/teeterbot/right_torque_cmd", &TeeterbotInterfacePlugin::recvRightTorqueCmd, this);
        this->pub_left_speed_ = this->node_.Advertise<msgs::Double>("/model/teeterbot/left_speed");
        this->pub_right_speed_ = this->node_.Advertise<msgs::Double>("/model/teeterbot/right_speed");
        this->pub_fallen_over_ = this->node_.Advertise<msgs::Boolean>("/model/teeterbot/fallen_over");
    }

    void TeeterbotInterfacePlugin::recvLeftSpeedCmd(const gz::msgs::Double& msg) {
        this->left_speed_cmd_ = msg.data();
        this->speed_stamp_ = this->current_time_;
    }

    void TeeterbotInterfacePlugin::recvRightSpeedCmd(const gz::msgs::Double& msg) {
        this->right_speed_cmd_ = msg.data();
        this->speed_stamp_ = this->current_time_;
    }

    void TeeterbotInterfacePlugin::recvLeftTorqueCmd(const gz::msgs::Double& msg) {
        this->left_torque_cmd_ = msg.data();
        this->torque_stamp_ = this->current_time_;
    }

    void TeeterbotInterfacePlugin::recvRightTorqueCmd(const gz::msgs::Double& msg) {
        this->right_torque_cmd_ = msg.data();
        this->torque_stamp_ = this->current_time_;
    }

    void TeeterbotInterfacePlugin::PreUpdate(const UpdateInfo & _info, EntityComponentManager & _ecm) {
        this->current_time_ = _info.realTime.count();
        bool speed_valid = true;

        auto left_joint_vel = _ecm.Component<components::JointVelocity>(this->left_joint_);
        if (!left_joint_vel) {
            _ecm.CreateComponent(this->left_joint_, components::JointVelocity());
            speed_valid = false;
        }
        auto right_joint_vel = _ecm.Component<components::JointVelocity>(this->right_joint_);
        if (!right_joint_vel) {
            _ecm.CreateComponent(this->right_joint_, components::JointVelocity());
            speed_valid = false;
        }

        if (_info.dt.count() == 0 || !speed_valid) {
            return;
        }
        double dt = 1e-9 * _info.dt.count();

        double left_feedback;
        double right_feedback;
        double left_voltage;
        double right_voltage;

        if (this->fallen_over_) {
            this->left_torque_control_->reset();
            this->right_torque_control_->reset();
            this->left_speed_control_->reset();
            this->right_speed_control_->reset();
            _ecm.SetComponentData<components::JointForceCmd>(this->left_joint_, {0.0});
            _ecm.SetComponentData<components::JointForceCmd>(this->right_joint_, {0.0});
        } else {
            if (this->torque_stamp_ >= this->speed_stamp_) {
                // Torque mode
                left_feedback = this->left_motor_->get_current() * this->left_motor_->get_torque_constant();
                right_feedback = this->right_motor_->get_current() * this->right_motor_->get_torque_constant();
                left_voltage = this->left_torque_control_->update(dt, left_torque_cmd_, left_feedback);
                right_voltage = this->right_torque_control_->update(dt, right_torque_cmd_, right_feedback);
            } else {
                // Speed mode
                left_feedback = left_joint_vel->Data()[0];
                right_feedback = right_joint_vel->Data()[0];
                left_voltage = this->left_speed_control_->update(dt, left_speed_cmd_, left_feedback);
                right_voltage = this->right_speed_control_->update(dt, right_speed_cmd_, right_feedback);
            }
            double left_torque_actual = this->left_motor_->step(dt, left_voltage, left_joint_vel->Data()[0]);
            double right_torque_actual = this->right_motor_->step(dt, right_voltage, right_joint_vel->Data()[0]);

            _ecm.SetComponentData<components::JointForceCmd>(this->left_joint_, {left_torque_actual});
            _ecm.SetComponentData<components::JointForceCmd>(this->right_joint_, {right_torque_actual});
        }
    }

    void TeeterbotInterfacePlugin::PostUpdate(const UpdateInfo& _info, const EntityComponentManager &_ecm) {
        auto left_joint_vel = _ecm.Component<components::JointVelocity>(this->left_joint_);
        auto right_joint_vel = _ecm.Component<components::JointVelocity>(this->right_joint_);
        if (!left_joint_vel || !right_joint_vel) {
            return;
        }

        const math::Pose3d vehicle_pose = worldPose(this->model_.Entity(), _ecm);
        this->fallen_over_ = (std::abs(vehicle_pose.Pitch()) > FALLEN_OVER_THRES);
        if ((1e-9 * (this->current_time_ - this->encoder_pub_stamp_)) > ENCODER_SAMPLE_TIME) {
            this->encoder_pub_stamp_ = this->current_time_;
            msgs::Double left_speed_msg;
            left_speed_msg.set_data(left_joint_vel->Data()[0]);
            this->pub_left_speed_.Publish(left_speed_msg);
            msgs::Double right_speed_msg;
            right_speed_msg.set_data(right_joint_vel->Data()[0]);
            this->pub_right_speed_.Publish(right_speed_msg);

            msgs::Boolean fallen_over_msg;
            fallen_over_msg.set_data(this->fallen_over_);
            this->pub_fallen_over_.Publish(fallen_over_msg);
        }
    }
}

// Register plugin
GZ_ADD_PLUGIN(teeterbot_gazebo::TeeterbotInterfacePlugin,
              System,
              teeterbot_gazebo::TeeterbotInterfacePlugin::ISystemConfigure,
              teeterbot_gazebo::TeeterbotInterfacePlugin::ISystemPreUpdate,
              teeterbot_gazebo::TeeterbotInterfacePlugin::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(teeterbot_gazebo::TeeterbotInterfacePlugin, "gz::sim::systems::TeeterbotInterfacePlugin")
