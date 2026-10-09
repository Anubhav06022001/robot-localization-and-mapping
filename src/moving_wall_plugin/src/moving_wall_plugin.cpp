#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Pose3.hh>

#include <iostream>

namespace gazebo
{

class MovingWallPlugin : public ModelPlugin
{
public:
  MovingWallPlugin() = default;

  void Load(physics::ModelPtr model, sdf::ElementPtr /*sdf*/) override
  {
    model_ = model;
    last_shift_time_ = 0.0;
    shift_count_ = 0;

    update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&MovingWallPlugin::OnUpdate, this));

    std::cout << "[MovingWallPlugin] Loaded for model: "
              << model_->GetName() << std::endl;
    std::cout << "[MovingWallPlugin] Wall will shift +1.5 m in Y every 15 s."
              << std::endl;
  }

private:
  void OnUpdate()
{
  const double sim_time =
    model_->GetWorld()->SimTime().Double();

  if (sim_time >= 15.0 && sim_time - last_shift_time_ >= 15.0)
  {
    auto pose = model_->WorldPose();

    shift_count_++;

    // Alternate between -1.5 m and +1.5 m.
    if (shift_count_ % 2 == 1)
    {
      pose.Pos().Y() = 1.5;
    }
    else
    {
      pose.Pos().Y() = -1.5;
    }

    model_->SetWorldPose(pose);

    last_shift_time_ = sim_time;

    std::cout << "[MovingWallPlugin] Shift "
              << shift_count_
              << " at T = " << sim_time
              << " s, Y = " << pose.Pos().Y()
              << " m" << std::endl;
  }
}

  physics::ModelPtr model_;
  event::ConnectionPtr update_connection_;

  double last_shift_time_;
  int shift_count_;
};

GZ_REGISTER_MODEL_PLUGIN(MovingWallPlugin)

}  // namespace gazebo
