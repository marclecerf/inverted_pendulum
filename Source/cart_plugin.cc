#ifndef _CART_PLUGIN_HH_
#define _CART_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo {
  class CartPlugin : public ModelPlugin {
    public: CartPlugin() {}
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
      std::cerr << "\nCart plugin is attach to model[" <<
        _model->GetName() << "]\n";
    }
  };

  GZ_REGISTER_MODEL_PLUGIN(CartPlugin)
}  // namespace gazebo

#endif  // _CART_PLUGIN_HH_
