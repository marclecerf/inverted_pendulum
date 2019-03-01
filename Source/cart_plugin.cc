#ifndef _CART_PLUGIN_HH_
#define _CART_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo {

const char* kPendulumJoint = "InvertedPendulumCart::ChassisPendulumRevolute";

const std::vector<const char*> kWheelJoints = {
 //"InvertedPendulumCart::WheelRightBackRevolute",
 //"InvertedPendulumCart::WheelLeftBackRevolute",
 "InvertedPendulumCart::WheelRightFrontRevolute",
 "InvertedPendulumCart::WheelLeftFrontRevolute",
};

const char* kRightFrontJoint = "InvertedPendulumCart::WheelRightFrontRevolute";
const char* kLeftFrontJoint = "InvertedPendulumCart::WheelLeftFrontRevolute";

class CartPlugin : public ModelPlugin {
 public:
  CartPlugin() {}

  void SetPendulumPositionPid(double kp, double ki, double kd) {
    auto joint = this->model->GetJoint(kPendulumJoint);
    if (!joint) {
      std::cerr << "Could not find joint '" << kPendulumJoint << "'\n";
      return;
    }
    common::PID pid(kp, ki, kd);
    this->model->GetJointController()->SetPositionPID(
      joint->GetScopedName(), pid);
  }

  void SetWheelVelocityPid(double kp, double ki, double kd) {
    common::PID pid(kp, ki, kd);
    for (const auto& name: kWheelJoints) {
      auto joint = this->model->GetJoint(name);
      if (!joint) {
        std::cerr << "Could not find joint '" << kPendulumJoint << "'\n";
        continue;
      }
      this->model->GetJointController()->SetVelocityPID(
        joint->GetScopedName(), pid);
    }
  }

  void SetFrontWheelVelocityTarget(double left, double right) {
    auto left_joint = this->model->GetJoint(kLeftFrontJoint);
    if (!left_joint) {
      std::cerr << "Could not find joint '" << kLeftFrontJoint << "'\n";
      return;
    }
    auto right_joint = this->model->GetJoint(kRightFrontJoint);
    if (!right_joint) {
      std::cerr << "Could not find joint '" << kRightFrontJoint << "'\n";
      return;
    }
    this->model->GetJointController()->SetVelocityTarget(
            left_joint->GetScopedName(), left);
    this->model->GetJointController()->SetVelocityTarget(
            right_joint->GetScopedName(), right);
  }

  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    if (_model->GetJointCount() == 0) {
      std::cerr << "Invalid joint count, cart plugin not loaded\n";
      return;
    }

    for (const auto& j: _model->GetJoints()) {
      std::cerr << "Joint name: '" << j->GetName() << "'\n";
    }

    this->model = _model;
    auto joint = _model->GetJoint(kPendulumJoint);
    if (!joint) {
      std::cerr << "Could not find joint '" << kPendulumJoint << "'\n";
      return;
    }
    this->model->GetJointController()->SetPositionTarget(
            joint->GetScopedName(), 0.0);
    this->SetPendulumPositionPid(0, 0, 0);

    this->node = transport::NodePtr(new transport::Node());
    this->node->Init(this->model->GetWorld()->Name());
    this->sub_p_pid = this->node->Subscribe(
            "~/" + this->model->GetName() + "/pendulum/pid",
            &CartPlugin::OnPendulumPidMsg, this);
    this->sub_w_pid = this->node->Subscribe(
            "~/" + this->model->GetName() + "/wheel/pid",
            &CartPlugin::OnWheelPidMsg, this);
    this->sub_w_tgt = this->node->Subscribe(
            "~/" + this->model->GetName() + "/wheel/tgt",
            &CartPlugin::OnWheelTgtMsg, this);
  }

 private:
  void OnPendulumPidMsg(ConstVector3dPtr& _msg) {
    this->SetPendulumPositionPid(_msg->x(), _msg->y(), _msg->z());
  }
  void OnWheelPidMsg(ConstVector3dPtr& _msg) {
    this->SetWheelVelocityPid(_msg->x(), _msg->y(), _msg->z());
  }
  void OnWheelTgtMsg(ConstVector3dPtr& _msg) {
    this->SetFrontWheelVelocityTarget(_msg->x(), _msg->y());
  }
 
 private:
  physics::ModelPtr model;
  transport::NodePtr node;
  transport::SubscriberPtr sub_p_pid;
  transport::SubscriberPtr sub_w_pid;
  transport::SubscriberPtr sub_w_tgt;
};

GZ_REGISTER_MODEL_PLUGIN(CartPlugin)
}  // namespace gazebo

#endif  // _CART_PLUGIN_HH_
