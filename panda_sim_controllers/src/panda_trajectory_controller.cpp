
#include <pluginlib/class_list_macros.h>
#include <controller_interface/controller.h>

// #include <panda_sim_controllers/panda_joint_effort_controller.h>
#include <panda_hardware_interface/shared_joint_interface.h>
#include <trajectory_interface/quintic_spline_segment.h>
#include <joint_trajectory_controller/joint_trajectory_controller.h>
#include <hardware_interface/joint_command_interface.h>
// #include <effort_controllers/joint_trajectory_controller.h>

namespace panda_sim_controllers {
    /**
   * \brief Joint trajectory controller that represents trajectory segments as <b>quintic splines</b> and sends
   * commands to an \b effort interface.
   */
  // namespace effort_controllers
  // {
      // typedef effort_controllers::JointTrajectoryController PandaJointTrajectoryController;
  // }
      typedef joint_trajectory_controller::JointTrajectoryController<trajectory_interface::QuinticSplineSegment<double>,
                                                                 panda_hardware_interface::SharedJointInterface>
          PandaJointTrajectoryController;
  
}

PLUGINLIB_EXPORT_CLASS(panda_sim_controllers::PandaJointTrajectoryController, controller_interface::ControllerBase)
