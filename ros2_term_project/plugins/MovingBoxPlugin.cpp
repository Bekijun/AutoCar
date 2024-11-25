#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
namespace gazebo {
  class MovingBoxPlugin : public ModelPlugin {
  public:
    void Load(physics::ModelPtr model, sdf::ElementPtr /*_sdf*/) override {
      this->model = model;
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&MovingBoxPlugin::OnUpdate, this));
    }
    void OnUpdate() {
      // 현재 시간 가져오기
      double time = this->model->GetWorld()->SimTime().Double();
      // 왕복 운동을 위한 파라미터
      double speed = 2.0; // 속도 2m/s
      double min_y = -64; // 최소 Y값
      double max_y = -77; // 최대 Y값
      double range = max_y - min_y; // 왕복 범위

      double y_position = min_y + (range / 2) * (1 + std::sin(speed * time));
      // 박스 위치 업데이트
      ignition::math::Pose3d pose = this->model->WorldPose();
      pose.Pos().Y(y_position); // Y값을 새로 설정
      this->model->SetWorldPose(pose); // 새로운 위치 반영
    }
  private:
    physics::ModelPtr model;
    event::ConnectionPtr updateConnection;
  };
  GZ_REGISTER_MODEL_PLUGIN(MovingBoxPlugin)
}
