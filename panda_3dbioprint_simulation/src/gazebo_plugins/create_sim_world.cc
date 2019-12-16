#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

namespace gazebo
{
  class Factory : public WorldPlugin
  {
    public: void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      _parent->InsertModelFile("model://patient_room");
      _parent->InsertModelFile("model://hospital_bed");
      _parent->InsertModelFile("model://doctor_1");
      _parent->InsertModelFile("model://doctor_2");
      _parent->InsertModelFile("model://patient");
      _parent->InsertModelFile("model://service_cart");
      _parent->InsertModelFile("model://emergency_console");
      _parent->InsertModelFile("model://tv");
      _parent->InsertModelFile("model://serum_support");
      _parent->InsertModelFile("model://support_table");
    }
  };
  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(Factory)
}