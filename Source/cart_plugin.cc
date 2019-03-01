#ifndef _CART_PLUGIN_HH_
#define _CART_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo {
class CartPlugin : public ModelPlugin {
 public:
  CartPlugin() {}

  void SetPositionPid(ConstVector3dPtr& pid) {
    this->pid = common::PID(pid->x(), pid->y(), pid->z());
    this->model->GetJointController()->SetPositionPID(
      this->joint->GetScopedName(), this->pid);
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
    this->joint = _model->GetJoint("InvertedPendulumCart::ChassisPendulumRevolute");
    if (!this->joint) {
      std::cerr << "Could not find pendulum joint!\n";
      return;
    }
    this->pid = common::PID(0.5, 0, 0);
    this->model->GetJointController()->SetPositionPID(
      this->joint->GetScopedName(), this->pid);

    this->model->GetJointController()->SetPositionTarget(
      this->joint->GetScopedName(), 0.0);

    this->node = transport::NodePtr(new transport::Node());
    this->node->Init(this->model->GetWorld()->Name());
    std::string topicName = "~/" + this->model->GetName() + "/kp";
    this->sub = this->node->Subscribe(topicName,
            &CartPlugin::OnMsg, this);
  }

 private:
  void OnMsg(ConstVector3dPtr& _msg) {
    this->SetPositionPid(_msg);
  }

 private:
  physics::ModelPtr model;
  physics::JointPtr joint;
  common::PID pid;
  transport::NodePtr node;
  transport::SubscriberPtr sub;
};

GZ_REGISTER_MODEL_PLUGIN(CartPlugin)
}  // namespace gazebo

#endif  // _CART_PLUGIN_HH_
