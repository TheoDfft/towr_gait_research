/******************************************************************************
Copyright (c) 2018, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <towr/initialization/gait_generator.h>
#include <towr_ros/towr_ros_interface.h>
#include <towr/terrain/examples/height_map_examples.h>
#include <ros/ros.h>

namespace towr {

/**
 * @brief A hopper example application using TOWR with ROS.
 *
 * This implements a simple monoped (hopper) robot trajectory optimization
 * that makes the robot jump in place to reach 1.5 times its initial height.
 */
class HopperRosApp : public TowrRosInterface {
public:
  /**
   * @brief Constructor that configures for a monoped robot
   */
  HopperRosApp() {
    // Configure to use monoped robot by default
    formulation_.model_ = RobotModel(RobotModel::Monoped);
    
    // Set up node handle for getting parameters
    ros::NodeHandle nh;
    
    // Read jump height multiplier if provided (default 1.5)
    height_multiplier_ = 1.5;
    if (nh.hasParam("height_multiplier")) {
      nh.getParam("height_multiplier", height_multiplier_);
      ROS_INFO("Using custom height multiplier: %.2f", height_multiplier_);
    }
    
    // Set the terrain to flat ground
    formulation_.terrain_ = std::make_shared<FlatGround>(0.0);
  }

  /**
   * @brief Sets the initial state for the hopper.
   * 
   * Base floats above ground, foot at the origin.
   */
  void SetTowrInitialState() override
  {
    // Set initial foot position (at origin on the ground)
    formulation_.initial_ee_W_.clear();
    formulation_.initial_ee_W_.push_back(Eigen::Vector3d::Zero());

    // Set base position above ground
    formulation_.initial_base_.lin.at(kPos).z() = initial_height_;
  }

  /**
   * @brief Override to set the goal state for the jump
   * 
   * We want to stay in place (x,y same) but reach the target height
   */
  BaseState GetGoalState(const TowrCommandMsg& msg) const override
  {
    BaseState goal;
    
    // Calculate target height for the jump
    double target_height = height_multiplier_ * initial_height_;
    ROS_INFO("Setting jump target height to %.2f meters", target_height);
    
    // Stay in same x,y position but jump to target z height
    goal.lin.at(kPos).x() = 0.0;
    goal.lin.at(kPos).y() = 0.0;
    goal.lin.at(kPos).z() = target_height;
    
    // No velocity at the goal position (at apex of jump)
    goal.lin.at(kVel).setZero();
    
    // No angular motion for the jump
    goal.ang.at(kPos).setZero();
    goal.ang.at(kVel).setZero();
    
    return goal;
  }

  /**
   * @brief Sets the parameters required to formulate the TOWR problem.
   */
  Parameters GetTowrParameters(int n_ee, const TowrCommandMsg& msg) const override
  {
    Parameters params;

    // Configure phase durations for a jump trajectory
    // A jump needs: initial stance, push-off, flight, landing, final stance
    params.ee_phase_durations_.push_back({0.3, 0.2, 0.4, 0.2, 0.3});
    params.ee_in_contact_at_start_.push_back(true);

    // Add constraints to ensure a physically valid motion
    params.constraints_.push_back(Parameters::BaseRom);
    params.constraints_.push_back(Parameters::Dynamic);
    params.constraints_.push_back(Parameters::Force);

    // Optional: allow the optimizer to adjust phase durations
    if (msg.optimize_phase_durations)
      params.OptimizePhaseDurations();

    return params;
  }

  /**
   * @brief Sets the parameters for IPOPT.
   */
  void SetIpoptParameters(const TowrCommandMsg& msg) override
  {
    solver_->SetOption("linear_solver", "mumps");
    solver_->SetOption("jacobian_approximation", "exact");
    solver_->SetOption("max_cpu_time", 20.0);
    solver_->SetOption("print_level", 5);
    
    // Set a higher maximum iteration count for complex jumps
    if (msg.play_initialization)
      solver_->SetOption("max_iter", 0);
    else
      solver_->SetOption("max_iter", 5000);
  }

private:
  double height_multiplier_ = 1.5; // How much higher to jump compared to initial height
  const double initial_height_ = 0.5; // Initial base height
};

} // namespace towr


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "towr_hopper_app");
  towr::HopperRosApp hopper_app;
  ros::spin();

  return 1;
} 