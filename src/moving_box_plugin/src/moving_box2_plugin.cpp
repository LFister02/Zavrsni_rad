#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Pose3.hh>

namespace gazebo
{
  class BoxMoveOnce : public ModelPlugin
  {
  private:
    physics::ModelPtr model;
    event::ConnectionPtr updateConnection;
    double distanceTravelled = 0.0;
    double totalDistance = 7.5;
    double movementSpeed = 0.25;
    ignition::math::Pose3d startPose;
    bool movementStop = false;      

  public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/) override
    {
      model = _model;
      startPose = model->WorldPose();
      updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&BoxMoveOnce::OnUpdate, this));
      gzdbg << "BoxMoveOnce plugin učitan za model: " << model->GetName() << "\n";
    }

    void OnUpdate()
    {
      if (movementStop) return;   

      double dt = model->GetWorld()->Physics()->GetMaxStepSize();

      auto pose = model->WorldPose();
      double dtStep = movementSpeed * dt;

      pose.Pos().X() -= dtStep;
      distanceTravelled += dtStep;

      if (distanceTravelled >= totalDistance)
      {

          pose.Pos().X() = startPose.Pos().X() - totalDistance;
          movementStop = true;
          gzdbg << "Kutija 2 je pomaknuta " << totalDistance << " m i zaustavljena.\n";
      }

      model->SetWorldPose(pose);
    }
  };

  GZ_REGISTER_MODEL_PLUGIN(BoxMoveOnce)
}