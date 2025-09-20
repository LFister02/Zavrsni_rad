#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Pose3.hh>

namespace gazebo
{
  class BoxMove : public ModelPlugin
  {
  private:
    physics::ModelPtr model;
    event::ConnectionPtr updateConnection;
    double direction = 1.0;       
    double distanceTravelled = 0.0;
    double totalDistance = 8.5;     
    double movementSpeed = 0.5;           
    ignition::math::Pose3d startPose;

  public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/) override
    {
      model = _model;
      startPose = model->WorldPose();  
      updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&BoxMove::OnUpdate, this));
      gzdbg << "BoxMove plugin učitan za model: " << model->GetName() << "\n";
    }

    void OnUpdate()
    {

      double dt = model->GetWorld()->Physics()->GetMaxStepSize();

      auto pose = model->WorldPose();
      double dtStep = direction * movementSpeed * dt;

      pose.Pos().X() += dtStep;
      distanceTravelled += fabs(dtStep);

      if (distanceTravelled >= totalDistance)
      {
          direction = -direction;      
          distanceTravelled = 0.0;     
      }

      model->SetWorldPose(pose);
    }
  };

  GZ_REGISTER_MODEL_PLUGIN(BoxMove)
}