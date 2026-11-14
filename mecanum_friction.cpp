// Classic equivalent of Husarion's fdir1 expressed_in="base_link".
// This plugin only transforms contact friction directions. Joint motors supply motion.
#include <array>
#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo {
class MecanumFriction : public ModelPlugin {
 public:
  void Load(physics::ModelPtr model, sdf::ElementPtr) override {
    model_ = model;
    const std::array<std::string, 4> names{
        "fl_wheel_link", "fr_wheel_link", "rl_wheel_link", "rr_wheel_link"};
    for (std::size_t i = 0; i < names.size(); ++i) {
      collisions_[i] = model_->GetLink(names[i])->GetCollisions().front();
      auto friction = collisions_[i]->GetSurface()->FrictionPyramid();
      friction->SetMuPrimary(1.0);
      friction->SetMuSecondary(0.0);
    }
    connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&MecanumFriction::Update, this));
  }

 private:
  void Update() {
    const std::array<double, 4> signs{-1.0, 1.0, 1.0, -1.0};
    const auto rotation = model_->GetLink("base_footprint")->WorldPose().Rot();
    for (std::size_t i = 0; i < collisions_.size(); ++i) {
      const auto world_direction = rotation.RotateVector(
          ignition::math::Vector3d(1.0, signs[i], 0.0).Normalized());
      collisions_[i]->GetSurface()->FrictionPyramid()->direction1 =
          collisions_[i]->WorldPose().Rot().RotateVectorReverse(world_direction);
    }
  }
  physics::ModelPtr model_;
  std::array<physics::CollisionPtr, 4> collisions_;
  event::ConnectionPtr connection_;
};
GZ_REGISTER_MODEL_PLUGIN(MecanumFriction)
}
