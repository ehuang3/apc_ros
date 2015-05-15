/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Georgia Tech Research Corporation
 *  All rights reserved.
 *
 *  Author(s): Eric Huang <ehuang@gatech.edu>
 *  Georgia Tech Socially Intelligent Machines Lab
 *  Under Direction of Prof. Andrea Thomaz <athomaz@cc.gatech.edu>
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#pragma once
#include <string>
#include <Eigen/Dense>
#include <sns.h>
#include <apc_path/Path.h>
#include <apc_path/Trajectory.h>


namespace apc_control
{
    class MotorGroup;

    struct MotorGroupError
    {
        enum ErrorCode
        {
            STALE_FRAMES = 1,
            SUCCESSFUL = 0,
            INVALID_GOAL = -1,
            INVALID_JOINTS = -2,
            OLD_HEADER_TIMESTAMP = -3,
            PATH_TOLERANCE_VIOLATED = -4,
            GOAL_TOLERANCE_VIOLATED = -5,
            FAILED_CLOCK_GETTIME = -6,
            PREEMPTED = -7,
            TRAJECTORY_DURATION_TOO_LONG = -8,
            FORCE_TORQUE_DETECTED_CONTACT = -9,
        };

        ErrorCode   error_code;       // Value describing the error.
        std::string error_string;     // String describing the error.

        // True only if we errored out.
        operator bool() const
        {
            return error_code;
        }

        // Concatenate error messages.
        MotorGroupError& operator +=(const MotorGroupError& err)
        {
            // No error from the other.
            if (!err.error_code)
                return *this;

            // Has error.
            if (this->error_code)
                this->error_string += std::string("\n");

            // Append error.
            this->error_string += err.error_string;

            // HACK No way to concatenate error codes atm.
            this->error_code = err.error_code;

            return *this;
        }

        // Initialize to no errors.
        MotorGroupError()
            : error_code(SUCCESSFUL),
              error_string("")
        {}
    };

    class MotorGroupUnion
    {
    public:
        // Vector of joint names.
        typedef std::vector<std::string> JointNames;

        // Internal state.
        struct State
        {
            std::vector<MotorGroup*> groups;     // List of motor groups.
            std::vector<int> active;             // Index into active motor groups.
            size_t num_active_dofs;              // Number of active dofs.
        };

        // Create and initialize all motor groups.
        bool initGroups();

        // Open ACH channels for all motor groups.
        bool openChannels();

        // Reset all motor group states.
        void reset();

        // Set the active groups.
        bool setActive(const JointNames& joint_names);

        // Set the trajectory for all motor groups.
        void setTrajectory(Trajectory& trajectory);

        // Get the number of active motor groups (for a given trajectory).
        int getActiveCount();

        // Get the ith active motor group.
        MotorGroup* getActiveGroup(int i);

        // Get the current state from all active motor groups.
        void getState(Eigen::VectorXd& state);

        // Get the maximum velocity vector.
        Eigen::VectorXd getMaxVel();

        // Get the maximum acceleration vector.
        Eigen::VectorXd getMaxAccel();

        // Are all the motor groups arrived at their goals.
        bool isAtGoal();

        // Are any of the motor groups still moving.
        bool isMoving();

        // Check start state for all groups.
        MotorGroupError checkStartState(const Eigen::VectorXd& start,
                                        const JointNames&      joints);

        // Apply preconditions before execution.
        MotorGroupError applyPreConditions();

        // Build motor reference commands for all active motor groups.
        MotorGroupError buildCommands(double t);

        // Send motor reference commands for all active motor groups.
        MotorGroupError sendCommands(double t);

        // Apply postconditions after execution.
        MotorGroupError applyPostConditions();

    protected:
        State _state;
    };

    class MotorGroup
    {
    public:
        // Mapping from joint names to indicies.
        typedef std::map<std::string, int> JointMap;

        // Vector of joint names.
        typedef std::vector<std::string> JointNames;

        // Motor group parameters.
        struct GroupParams
        {
            JointMap map;                        // Mapping from joint names to motor reference command indices.
            JointNames joint_names;              // Names of all joints in this motor group.
            std::string name_group;              // Name of this group.
            std::string name_state;              // Name of read motor states channel.
            std::string name_ref;                // Name of send reference commands channel.
            ach_channel_t chan_state;            // Channel to read motor states from.
            ach_channel_t chan_ref;              // Channel to send reference commands to.
            enum sns_motor_mode mode;            // Operating mode of motor reference commands.
            std::string mode_name;               // Human readable name of the motor mode.
            double max_vel;                      // Maximum joint velocity along the path.
            double max_accel;                    // Maximum joint acceleration along the path.
            struct timespec timeout;             // Timeout to wait when reading motor states.
            double dt;                           // Time between control loop iterations.
            bool allow_execution;                // If true, execute commands on real hardware. vBoth must be true.
            bool enabled;                        // If true, execute commands on real hardware. ^Both must be true.

            double k_p;                          // Position error gain.

            bool log;                            // If true, output logging messages to the logging channels.
            std::string name_track;              // Name of the debug command channel.
            ach_channel_t chan_track;            // Channel to send full commands to for debugging.
            std::string name_feedback_state;     // Name of the debug feedback state channel.
            ach_channel_t chan_feedback_state;   // Channel to send feedback states for debugging.
        };

        // Motor group state.
        struct GroupState
        {
            Trajectory* T;                       // Trajectory for the motor group to follow.
            JointMap map;                        // Mapping from joint names to trajectory indices.
            struct sns_msg_motor_state* state;   // Latest motor state.
            struct sns_msg_motor_ref* ref;       // Last motor reference command.
            bool dirty_state;                    // True if motor state has not been read since last command.
            bool fresh_ref;                      // True if a new motor command is ready to be sent.
            int64_t num_sent;                    // Number of messages sent.

            struct sns_msg_motor_state* track;   // Latest command position and velocity to track.
        };

        MotorGroup();
        ~MotorGroup();

        // Get motor group parameters.
        GroupParams* getParameters();

        // Get motor group state.
        GroupState* getState();

        // Get number of joints in this motor group.
        int getNumDofs();

        // Read in parameters from the ROS parameter server.
        bool readParameters(std::string group_name);

        // Open ACH channel.
        bool openChannel(std::string    chan_name,
                         ach_channel_t* chan);

        // Open ACH channels for reading motor states and sending reference commands.
        bool openChannels();

        // Reset motor group internal state.
        void resetState();

        // Set the active motors.
        MotorGroupError setActive(const JointNames& joint_names);

        // Set the trajectory to follow for this motor group.
        bool setTrajectory(Trajectory& trajectory);

        // Get the clock time.
        MotorGroupError getTime(struct timespec* time);

        // Read in motor with timeout.
        MotorGroupError readState(int64_t ms);

        // Read in motor state.
        MotorGroupError readState(struct sns_msg_motor_state* msg,
                                  struct timespec* time_out);

        // Get the motor state.
        void getState(struct sns_msg_motor_state* state);

        // Fill 'state' based on trajectory index mapping.
        void getPartialState(Eigen::VectorXd& state);

        // Get the maximum velocity vector.
        void getPartialMaxVel(Eigen::VectorXd& max_vel);

        // Get the maximum acceleration vector.
        void getPartialMaxAccel(Eigen::VectorXd& max_accel);

        // True if the motors are actuated in this trajectory.
        bool isActive();

        // True when motors have reached the goal state.
        bool isAtGoal();

        // True when motors are moving with non-zero velocity.
        bool isMoving();

        // Check desired start state against current motor encoder readings.
        MotorGroupError checkStartState(const Eigen::VectorXd& start,
                                        const JointNames&      joints);

        // Apply general pre-conditions at the beginning of a trajectory.
        MotorGroupError applyGeneralPreConditions();

        // Apply custom pre-conditions at the beginning of a trajectory.
        virtual MotorGroupError applyCustomPreConditions() { return MotorGroupError(); }

        // Build reference command at timestep 't' along the trajectory.
        virtual MotorGroupError buildCommand(double t);

        // Build reference command from eigen vector.
        MotorGroupError buildCommand(const Eigen::VectorXd& cmd);

        // Send reference command at timestep 't' along the trajectory.
        virtual MotorGroupError sendCommand(double t);

        // Send reference command to motors.
        MotorGroupError sendCommand(struct sns_msg_motor_ref* cmd);

        // Apply custom post-conditions at the end of a trajectory.
        virtual MotorGroupError applyCustomPostConditions() { return MotorGroupError(); }

        // Apply general pre-conditions at the end of a trajectory.
        MotorGroupError applyGeneralPostConditions();

    protected:
        GroupParams _params;
        GroupState  _state;
    };

    class SDHGroup : public MotorGroup
    {
        // Only send commands when the joints have reached the last
        // command position. No interpolation between distinct
        // positions.
        virtual MotorGroupError sendCommand(double t);
    };

    class LWA4Group : public MotorGroup
    {
    public:
        // Set joint angle velocities to zero.
        MotorGroupError setVelocityToZero();

        // LWA4 velocities will be set to zero.
        virtual MotorGroupError applyCustomPreConditions();

        // LWA4 velocities will be set to zero.
        virtual MotorGroupError applyCustomPostConditions();
    };
}
