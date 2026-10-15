// Copyright 2026 Robin Müller
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <Eigen/Core>

#include "auto_apms_behavior_tree_core/node.hpp"
#include "px4_ros2/utils/geodesic.hpp"

#define INPUT_KEY_LAT "lat"
#define INPUT_KEY_LON "lon"
#define INPUT_KEY_ALT "alt"
#define INPUT_KEY_N "north"
#define INPUT_KEY_E "east"
#define INPUT_KEY_D "down"
#define OUTPUT_KEY_LAT "target_lat"
#define OUTPUT_KEY_LON "target_lon"
#define OUTPUT_KEY_ALT "target_alt"
#define OUTPUT_KEY_VEC "target_vector"

namespace auto_apms_px4
{

/**
 * @ingroup auto_apms_px4
 * @brief Behavior tree node that offsets a global WGS84 position by a local NED vector given in meters.
 *
 * Pure computation (no ROS 2 communication): Adds the vector (north, east, down) to the global position (lat, lon, alt)
 * and writes the resulting global position to the output ports. Use it together with %GetGlobalPosition to compute
 * targets relative to the vehicle's position for nodes that require absolute global coordinates, e.g.
 * %SendCmdDoReposition.
 */
class OffsetGlobalPosition : public BT::SyncActionNode
{
public:
  using SyncActionNode::SyncActionNode;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>(INPUT_KEY_LAT, "{lat}", "Latitude [°] of the position to offset"),
      BT::InputPort<double>(INPUT_KEY_LON, "{lon}", "Longitude [°] of the position to offset"),
      BT::InputPort<double>(INPUT_KEY_ALT, "{alt}", "Altitude [m] (AMSL) of the position to offset"),
      BT::InputPort<double>(INPUT_KEY_N, 0.0, "Offset towards north [m]"),
      BT::InputPort<double>(INPUT_KEY_E, 0.0, "Offset towards east [m]"),
      BT::InputPort<double>(INPUT_KEY_D, 0.0, "Offset downwards [m] (negative to go up)"),
      BT::OutputPort<double>(OUTPUT_KEY_LAT, "{target_lat}", "Resulting latitude [°]"),
      BT::OutputPort<double>(OUTPUT_KEY_LON, "{target_lon}", "Resulting longitude [°]"),
      BT::OutputPort<double>(OUTPUT_KEY_ALT, "{target_alt}", "Resulting altitude [m] (AMSL)"),
      BT::OutputPort<Eigen::MatrixXd>(
        OUTPUT_KEY_VEC, "{target_vec}",
        "Resulting global position vector (latitude [°], longitude [°], altitude AMSL [m])")};
  }

  BT::NodeStatus tick() override final
  {
    auto get_input = [this](const char * key) {
      const BT::Expected<double> expected = getInput<double>(key);
      if (!expected) {
        throw BT::RuntimeError(name() + " (" + registrationName() + ") - " + expected.error());
      }
      return expected.value();
    };
    const Eigen::Vector3d global_position(get_input(INPUT_KEY_LAT), get_input(INPUT_KEY_LON), get_input(INPUT_KEY_ALT));
    const Eigen::Vector3f vector_ned =
      Eigen::Vector3d(get_input(INPUT_KEY_N), get_input(INPUT_KEY_E), get_input(INPUT_KEY_D)).cast<float>();

    const Eigen::Vector3d target = px4_ros2::addVectorToGlobalPosition(global_position, vector_ned);
    Eigen::MatrixXd mat(1, 3);
    mat << target.x(), target.y(), target.z();
    setOutput(OUTPUT_KEY_LAT, target.x());
    setOutput(OUTPUT_KEY_LON, target.y());
    setOutput(OUTPUT_KEY_ALT, target.z());
    setOutput(OUTPUT_KEY_VEC, mat);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace auto_apms_px4

AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_px4::OffsetGlobalPosition)
