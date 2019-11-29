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
      // typedef joint_trajectory_controller::JointTrajectoryController<trajectory_interface::QuinticSplineSegment<double>,
      //                                                            panda_hardware_interface::SharedJointInterface>
      //     PandaJointTrajectoryController;
    class PandaJointTrajectoryController : public joint_trajectory_controller::JointTrajectoryController<trajectory_interface::QuinticSplineSegment<double>, panda_hardware_interface::SharedJointInterface> {

      public:
        bool init(panda_hardware_interface::SharedJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);

        void starting(const ros::Time& time);

        /** \brief Cancels the active action goal, if any. */
        void stopping(const ros::Time& /*time*/);

        void update(const ros::Time& time, const ros::Duration& period);

    };
  
}