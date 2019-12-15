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
      _parent->InsertModelFile("model://patient_room_small");
      _parent->InsertModelFile("model://hospital_bed");
    }
  };
  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(Factory)
}