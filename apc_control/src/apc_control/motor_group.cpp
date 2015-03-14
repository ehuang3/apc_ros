#include <apc_control/motor_group.h>
#include <ros/ros.h>

namespace apc_control
{
    bool MotorGroupUnion::initGroups()
    {
        // Create ROS node handle.
        ros::NodeHandle node("~");

        // Read in all motor group names from the parameter server
        std::vector<std::string> lwa4_motor_groups;
        if (!node.getParam("lwa4_motor_groups", lwa4_motor_groups))
        {
            ROS_ERROR("Failed to get lwa4_motor_groups from parameter server!");
            return false;
        }
        std::vector<std::string> sdh_motor_groups;
        if (!node.getParam("sdh_motor_groups", sdh_motor_groups))
        {
            ROS_ERROR("Failed to get motor_groups from parameter server!");
            return false;
        }

        // Clear all groups.
        _state.groups.clear();

        // Make sure that all groups are initialized correctly.
        bool success = true;

        // Create LWA4 motor groups.
        for (int i = 0; i < lwa4_motor_groups.size(); i++)
        {
            LWA4Group* group = new LWA4Group();
            success &= group->readParameters(lwa4_motor_groups[i]);
            _state.groups.push_back(group);
        }

        // Create SDH motor groups.
        for (int i = 0; i < sdh_motor_groups.size(); i++)
        {
            SDHGroup* group = new SDHGroup();
            success &= group->readParameters(sdh_motor_groups[i]);
            _state.groups.push_back(group);
        }

        // True on success.
        return success;
    }

    bool MotorGroupUnion::openChannels()
    {
        // Open channels for each group.
        for (int i = 0; i < _state.groups.size(); i++)
            if (!_state.groups[i]->openChannels())
                return false;

        // True on success.
        return true;
    }

    void MotorGroupUnion::reset()
    {
        for (int i = 0; i < _state.groups.size(); i++)
            _state.groups[i]->resetState();
    }

    void MotorGroupUnion::setActive(const JointNames& joint_names)
    {
        _state.active.clear();
        _state.num_active_dofs = 0;
        for (int i = 0; i < _state.groups.size(); i++)
        {
            _state.groups[i]->setActive(joint_names);
            if (_state.groups[i]->isActive())
            {
                _state.active.push_back(i);
                _state.num_active_dofs += _state.groups[i]->getNumDofs();
            }
        }
    }

    void MotorGroupUnion::setTrajectory(Trajectory& trajectory)
    {
        // Set the trajectory into each group.
        for (int i = 0; i < _state.groups.size(); i++)
            _state.groups[i]->setTrajectory(trajectory);
    }

    int MotorGroupUnion::getActiveCount()
    {
        return _state.active.size();
    }

    MotorGroup* MotorGroupUnion::getActiveGroup(int i)
    {
        return _state.groups[_state.active[i]];
    }

    void MotorGroupUnion::getState(Eigen::VectorXd& state)
    {
        for (int i = 0; i < _state.groups.size(); i++)
            if (_state.groups[i]->isActive())
                _state.groups[i]->getPartialState(state);
    }

    Eigen::VectorXd MotorGroupUnion::getMaxVel()
    {
        Eigen::VectorXd max_vel(_state.num_active_dofs);
        for (int i = 0; i < _state.groups.size(); i++)
            if (_state.groups[i]->isActive())
                _state.groups[i]->getPartialMaxVel(max_vel);
        return max_vel;
    }

    Eigen::VectorXd MotorGroupUnion::getMaxAccel()
    {
        Eigen::VectorXd max_accel(_state.num_active_dofs);
        for (int i = 0; i < _state.groups.size(); i++)
            if (_state.groups[i]->isActive())
                _state.groups[i]->getPartialMaxAccel(max_accel);
        return max_accel;
    }

    bool MotorGroupUnion::isAtGoal()
    {
        bool atGoal = true;
        for (int i = 0; i < _state.groups.size(); i++)
            if (_state.groups[i]->isActive())
                atGoal &= _state.groups[i]->isAtGoal();
        return atGoal;
    }

    bool MotorGroupUnion::isMoving()
    {
        bool moving = true;
        for (int i = 0; i < _state.groups.size(); i++)
            if (_state.groups[i]->isActive())
                moving |= _state.groups[i]->isMoving();
        return moving;
    }

    MotorGroupError MotorGroupUnion::checkStartState(const Eigen::VectorXd& start,
                                                     const JointNames&      joints)
    {
        MotorGroupError ret, err;
        for (int i = 0; i < _state.groups.size(); i++)
            if (err = _state.groups[i]->checkStartState(start, joints))
                ret += err;
        return ret;
    }

    MotorGroupError MotorGroupUnion::applyPreConditions()
    {
        MotorGroupError ret, err;
        for (int i = 0; i < _state.active.size(); i++)
        {
            // Apply general preconditions first.
            if (err = _state.groups[_state.active[i]]->applyGeneralPreConditions())
                ret += err;

            // Apply custom preconditions second.
            if (err = _state.groups[_state.active[i]]->applyCustomPreConditions())
                ret += err;
        }
        return ret;
    }

    MotorGroupError MotorGroupUnion::buildCommands(double t)
    {
        MotorGroupError ret, err;
        for (int i = 0; i < _state.groups.size(); i++)
            if (_state.groups[i]->isActive())
            {
                err = _state.groups[i]->buildCommand(t);
                ret += err;
            }
        return ret;
    }

    MotorGroupError MotorGroupUnion::sendCommands(double t)
    {
        MotorGroupError ret;
        for (int i = 0; i < _state.groups.size(); i++)
            if (_state.groups[i]->isActive())
                if (ret = _state.groups[i]->sendCommand(t))
                    return ret;
        return ret;
    }

    MotorGroupError MotorGroupUnion::applyPostConditions()
    {
        MotorGroupError ret, err;
        for (int i = 0; i < _state.active.size(); i++)
        {
            // Apply custom postconditions first.
            if (err = _state.groups[_state.active[i]]->applyCustomPostConditions())
                ret += err;

            // Apply general postconditions second.
            if (err = _state.groups[_state.active[i]]->applyGeneralPostConditions())
                ret += err;
        }
        return ret;
    }

    MotorGroup::MotorGroup()
    {
    }

    MotorGroup::~MotorGroup()
    {
    }

    MotorGroup::GroupParams* MotorGroup::getParameters()
    {
        return &_params;
    }

    MotorGroup::GroupState* MotorGroup::getState()
    {
        return &_state;
    }

    int MotorGroup::getNumDofs()
    {
        // Return the number of active joints!
        return _state.map.size();
    }

    void MotorGroup::getPartialState(Eigen::VectorXd& state)
    {
        if (!isActive())
        {
            ROS_ERROR("Requesting partial state from an inactive motor group: %s",
                      _params.name_group.c_str());
            return;
        }

        if (!_params.allow_execution)
            return;

        if (_state.dirty_state)
            if (!readState((int64_t) 100))
            {
                ROS_ERROR("Failed to read state within timeout!");
                return;
            }

        for (JointMap::iterator iter = _state.map.begin(); iter != _state.map.end(); ++iter)
            state[iter->second] = _state.state->X[_params.map[iter->first]].pos;
    }

    // Get the maximum velocity vector.
    void MotorGroup::getPartialMaxVel(Eigen::VectorXd& max_vel)
    {
        if (!isActive())
        {
            ROS_ERROR("Requesting partial max vel from an inactive motor group: %s",
                      _params.name_group.c_str());
            return;
        }

        for (JointMap::iterator iter = _state.map.begin(); iter != _state.map.end(); ++iter)
            max_vel[iter->second] = _params.max_vel;
    }

    // Get the maximum acceleration vector.
    void MotorGroup::getPartialMaxAccel(Eigen::VectorXd& max_accel)
    {
        if (!isActive())
        {
            ROS_ERROR("Requesting partial max accel from an inactive motor group: %s",
                      _params.name_group.c_str());
            return;
        }

        for (JointMap::iterator iter = _state.map.begin(); iter != _state.map.end(); ++iter)
            max_accel[iter->second] = _params.max_accel;
    }

    bool MotorGroup::readParameters(std::string group_name)
    {
        // Create node handle.
        ros::NodeHandle node("~");

        // Set the group name.
        _params.name_group = group_name;

#define READ_PARAM(key, member)                                     \
        if (!node.getParam(_params.name_group + std::string(#key),  \
                           _params.member))                         \
        {                                                           \
            ROS_ERROR("Failed to get %s for %s",                    \
                      #member, _params.name_group.c_str());         \
            return false;                                           \
        }                                                           \

        // Read most motor group parameters.
        READ_PARAM(_state,     name_state);
        READ_PARAM(_ref,       name_ref);
        READ_PARAM(_max_vel,   max_vel);
        READ_PARAM(_max_accel, max_accel);
        READ_PARAM(_map,       map);
        READ_PARAM(_dt,        dt);

#undef READ_PARAM

        // Create joint names from joint map.
        _params.joint_names.clear();
        for (JointMap::iterator iter = _params.map.begin(); iter != _params.map.end(); ++iter)
            _params.joint_names.push_back(iter->first);

        // Create timeout from dt.
        double to;
        if (!node.getParam(_params.name_group + std::string("_timeout"),
                           to))
        {
            ROS_ERROR("Failed to get %s for %s",
                      "timeout", _params.name_group.c_str());
            return false;
        }
        _params.timeout.tv_sec = (int) to;
        _params.timeout.tv_nsec = (to - (int) to) * 1e9;

        // Read in motor mode.
        if (!node.getParam(_params.name_group + std::string("_mode"),
                           _params.mode_name))
        {
            ROS_ERROR("Failed to get %s for %s",
                      "mode", _params.name_group.c_str());
            return false;
        }
        if (_params.mode_name == "SNS_MOTOR_MODE_VEL")
            _params.mode = SNS_MOTOR_MODE_VEL;
        else if (_params.mode_name == "SNS_MOTOR_MODE_POS")
            _params.mode = SNS_MOTOR_MODE_POS;
        else
        {
            std::cerr << "Unsupported SNS MOTOR MODE: " << _params.mode_name << std::endl;
            return false;
        }

        // Read in allow execution.
        if (!node.getParam("allow_execution",
                           _params.allow_execution))
        {
            ROS_ERROR("Failed to get %s for %s",
                      "allow_execution", _params.name_group.c_str());
            return false;
        }

        // Allocate space for state and ref messages.
        if (_params.map.size() == 0)
        {
            ROS_ERROR("Map is empty!");
            return false;
        }
        _state.state = sns_msg_motor_state_heap_alloc(_params.map.size());
        _state.ref   = sns_msg_motor_ref_heap_alloc(_params.map.size());

        return true;
    }

    bool MotorGroup::openChannel(std::string chan_name, ach_channel_t* chan)
    {
        sns_chan_open( chan, chan_name.c_str(), NULL );
        {
            ach_channel_t *chans[] = {chan, NULL};
            sns_sigcancel( chans, sns_sig_term_default );
        }
        return true;
    }

    bool MotorGroup::openChannels()
    {
        bool success = true;
        success &= openChannel(_params.name_state, &_params.chan_state);
        success &= openChannel(_params.name_ref,   &_params.chan_ref);
        return success;
    }

    void MotorGroup::resetState()
    {
        _state.T = NULL;
        _state.map.clear();
        memset(_state.state, 0, sizeof (struct sns_msg_motor_state));
        memset(_state.ref,   0, sizeof (struct sns_msg_motor_ref));
        _state.dirty_state = true;
        _state.fresh_ref = false;
        _state.num_sent = 0;
    }

    MotorGroupError MotorGroup::setActive(const JointNames& joint_order)
    {
        MotorGroupError ret;

        // Clear current map from joint names to trajectory indicies.
        _state.map.clear();

        // Generate new mapping from trajectory joint order.
        for (int i = 0; i < joint_order.size(); i++)
            // For each joint in our motor group...
            if (_params.map.count(joint_order[i]) > 0)
                // Map its joint name to trajectory index.
                _state.map[joint_order[i]] = i;

        if (_state.map.size() == 0)
        {
            ROS_INFO("Disabling %s", _params.name_group.c_str());
        }
        else if (_state.map.size() != _params.map.size())
        {
            ROS_ERROR("Incomplete joint trajectory for %s", _params.name_group.c_str());
            ret.error_code = MotorGroupError::INVALID_JOINTS;
            ret.error_string = "Incomplete joint trajectory for " + _params.name_group;
        }
        else
        {

        }

        return ret;
    }

    bool MotorGroup::setTrajectory(Trajectory& trajectory)
    {
        // Pointer to trajectory.
        _state.T = &trajectory;
        return true;
    }

    MotorGroupError MotorGroup::getTime(struct timespec* time)
    {
        MotorGroupError ret;
        // Get the current time.
        if( clock_gettime( ACH_DEFAULT_CLOCK, time ) )
        {
            SNS_LOG(LOG_ERR, "clock_gettime failed: %s\n", strerror(errno));
            ret.error_code = MotorGroupError::FAILED_CLOCK_GETTIME;
            ret.error_string = "Failed clock_gettime";
        }
        return ret;
    }

    MotorGroupError MotorGroup::readState(int64_t ms)
    {
        if (ms == 0)
            return readState(_state.state, NULL);

        struct timespec timeout;

        timeout.tv_sec = 0;
        timeout.tv_nsec = ms * 1e6;

        return readState(_state.state, &timeout);
    }

    MotorGroupError MotorGroup::readState(struct sns_msg_motor_state* msg,
                                          struct timespec* time_out)
    {
        MotorGroupError ret;

        // The frame size of the buffer.
        size_t frame_size;

        // Pointer to buffer that stores the message.
        void* buf;

        // We will get the latest message with timeout.
        int opt = ACH_O_LAST | (time_out ? ACH_O_WAIT : 0);

        // Get the motor state message.
        ach_status_t s = sns_msg_local_get( &_params.chan_state, &buf, &frame_size, time_out, opt );

        // Handle the return status. ach ok and missed frame are both
        // acceptable as we only want the latest motor state.
        switch (s)
        {
        case ACH_OK:
        case ACH_MISSED_FRAME:
        {
            // Sanity check the size of the message.
            if (frame_size != sns_msg_motor_state_size_n(_params.map.size()))
            {
                ret.error_code = MotorGroupError::INVALID_JOINTS;
                ret.error_string = "Failed to get the correct motor states";
                break;
            }

            // Convert raw buffer to motor state message.
            memcpy(msg, buf, frame_size);

            // Mark state as not dirty.
            _state.dirty_state = false;

            break;
        }
        case ACH_STALE_FRAMES:
        {
            // If we are polling quickly..it's okay for stale frames
            if (!time_out)
                break;

            ret.error_code = MotorGroupError::STALE_FRAMES;
            ret.error_string = "No new data has been published to channel: " + _params.name_state;

            break;
        }
        default:
            // Log error on failure.
            SNS_LOG(LOG_ERR, "Failed ach_get: %s\n", ach_result_to_string(s));

            ret.error_code = MotorGroupError::INVALID_JOINTS;
            ret.error_string = std::string("Failed ach_get: ") + ach_result_to_string(s);

            break;
        }

        return ret;
    }

    void MotorGroup::getState(struct sns_msg_motor_state* state)
    {
        if (_state.dirty_state)
            readState(false);

        *state = *_state.state;
    }

    bool MotorGroup::isActive()
    {
        return _state.map.size() > 0;
    }

    bool MotorGroup::isAtGoal()
    {
        // If we are not executing, always return false as we do not know if we
        // are at goal.
        if (!_params.allow_execution)
            return false;

        // If motors are not actively controlled, vacuously return true.
        if (!isActive())
            return true;

        // If no trajectory is set, return false.
        if (!_state.T)
            return false;

        // If the state is unread, read in new state.
        if (_state.dirty_state)
            // If state read fails, return false.
            if (readState(0))
                return false;

        // Get the total trajectory duration.
        const double duration = _state.T->getDuration();

        // Get goal position.
        const Eigen::VectorXd goal = _state.T->getPosition(duration);

        // Check goal position against current state. TODO Smarter tolerances.
        for (int i = 0; i < _state.state->header.n; i++)
            if (std::abs(goal[i] - _state.state->X[i].pos) > 0.017453)   // 1 degree in radians
                return false;
        return true;
    }

    bool MotorGroup::isMoving()
    {
        if (!_params.allow_execution)
            return false;
        if (_state.dirty_state)
            if (readState(0))
                return false;
        for (int i = 0; i < _state.state->header.n; i++)
            if (std::abs(_state.state->X[i].vel) > 0.000001)
                return true;
        return false;
    }

    MotorGroupError MotorGroup::checkStartState(const Eigen::VectorXd& start,
                                                const JointNames&      joints)
    {
        MotorGroupError ret;

        // Skip if not executing.
        if (!_params.allow_execution)
            return ret;

        // Read in state.
        if (ret = readState((int64_t) 100)) // 100 ms timeout.
            return ret;

        // Check start against real joint angles.
        for (int i = 0; i < joints.size(); i++)
            if (_params.map.count(joints[i]) > 0)
            {
                std::string joint_name = joints[i];
                double delta = _state.state->X[_params.map[joint_name]].pos - start[i];
                if (std::abs(delta) > 0.017453) // 1 degree in radians.
                {
                    MotorGroupError e;
                    e.error_code = MotorGroupError::PATH_TOLERANCE_VIOLATED;
                    e.error_string = "Joint differs by more than 1 degrees: " + joint_name;
                    ret += e;
                }
            }

        // On error, print out all desired and actual joint states.
        if (ret.error_code)
            for (int i = 0; i < _params.joint_names.size(); i++)
            {
                int j = _params.map[_params.joint_names[i]];
                ROS_ERROR("%s desired[%d]: %+.6f actual[%d]: %+.6f",
                          _params.joint_names[i].c_str(),
                          i,
                          start[_state.map[_params.joint_names[i]]],
                          j,
                          _state.state->X[j].pos);
            }

        return ret;
    }

    MotorGroupError MotorGroup::applyGeneralPreConditions()
    {
        MotorGroupError ret;

        // Assert we are an active motor group.
        if (!isActive())
        {
            ret.error_code = MotorGroupError::INVALID_JOINTS;
            ret.error_string = std::string("Failed to satisfy active group pre-condition for ") + _params.name_group;
            return ret;
        }

        // If we are not really exectuting, ignore hardware side pre-conditions.
        if (!_params.allow_execution)
            return ret;

        // Read in state with timeout.
        if (ret = readState(true))
            return ret;

        // Assert we are not moving.
        if (isMoving())
        {
            ret.error_code = MotorGroupError::INVALID_JOINTS;
            ret.error_string = std::string("Failed to satisfy non-moving pre-condition for ") + _params.name_group;
            return ret;
        }

        // Get the start configuration.
        Eigen::VectorXd start = _state.T->getPosition(0);

        // Check start state.
        for (JointMap::iterator iter = _state.map.begin(); iter != _state.map.end(); ++iter)
        {
            double error = _state.state->X[_params.map[iter->first]].pos - start[iter->second];
            if (std::abs(error) > 0.017453) // 1 degree in radians.
            {
                MotorGroupError e;
                e.error_code = MotorGroupError::PATH_TOLERANCE_VIOLATED;
                e.error_string = "Joint differs by more than 1 degrees: " + iter->first;
                ret += e;
            }
        }

        // On error, print out all desired and actual joint states.
        if (ret.error_code)
            for (int i = 0; i < _params.joint_names.size(); i++)
            {
                int j = _params.map[_params.joint_names[i]];
                ROS_ERROR("%s desired[%d]: %+.6f actual[%d]: %+.6f",
                          _params.joint_names[i].c_str(),
                          i,
                          start[_state.map[_params.joint_names[i]]],
                          j,
                          _state.state->X[j].pos);
            }

        return ret;
    }

    MotorGroupError MotorGroup::buildCommand(double t)
    {
        MotorGroupError ret;

        // Error out if inactive.
        if (!isActive())
        {
            ret.error_code = MotorGroupError::INVALID_GOAL;
            ret.error_string = std::string("Failed to build command for inactive motor group ") +
                _params.name_group;
            return ret;
        }

        // Create command vector based on motor control mode.
        Eigen::VectorXd cmd;
        switch (_params.mode)
        {
        case SNS_MOTOR_MODE_POS:
            cmd = _state.T->getPosition(t);
            break;
        case SNS_MOTOR_MODE_VEL:
            cmd = _state.T->getVelocity(t);
            break;
        default:
            ret.error_code = MotorGroupError::INVALID_GOAL;
            std::stringstream err;
            err << "Unsupported motor mode: " << _params.mode_name;
            ret.error_string = err.str();
            ROS_ERROR_STREAM(err.str());
            return ret;
        }

        // Convert the trajectory command into the sequence we expect to send over ACH.
        Eigen::VectorXd x(_params.map.size());
        for (JointMap::iterator iter = _state.map.begin(); iter != _state.map.end(); ++iter)
        {
            x[_params.map[iter->first]] = cmd[iter->second];
        }

        // Build SNS command to send over ACH.
        ret += buildCommand(x);

        return ret;
    }

    MotorGroupError MotorGroup::buildCommand(const Eigen::VectorXd& cmd)
    {
        MotorGroupError r;

        // Clear reference command.
        memset( _state.ref, 0, sizeof (struct sns_msg_motor_ref));

        // Initialize the message.
        sns_msg_motor_ref_init( _state.ref, _state.map.size() );

        // Set motor control mode.
        _state.ref->mode = _params.mode;

        // Copy the velocity command to the motor reference message.
        AA_MEM_CPY(_state.ref->u, cmd.data(), _state.ref->header.n);

        // Set a timeout 1 second in the future.
        struct timespec now;
        clock_gettime( ACH_DEFAULT_CLOCK, &now );
        sns_msg_set_time( &_state.ref->header, &now, 1e9 ); /* 1 second duration */

        // We now have a command waiting to be sent.
        _state.fresh_ref = true;

        return r;
    }

    MotorGroupError MotorGroup::sendCommand(double t)
    {
        MotorGroupError ret;

        // Build a new command for this time.
        if (ret = buildCommand(t))
            return ret;

        // Send reference command.
        ret = sendCommand(_state.ref);

        return ret;
    }

    MotorGroupError MotorGroup::sendCommand(struct sns_msg_motor_ref* cmd)
    {
        MotorGroupError r;

        // The ACH return status.
        ach_status_t status = ACH_OK;

        // Send reference command message, if enabled.
        if (_params.allow_execution)
            status = ach_put( &_params.chan_ref, cmd, sns_msg_motor_ref_size(cmd) );
        else
        {
            std::stringstream ss;
            for (int i=0; i < cmd->header.n; i++)
                ss << cmd->u[i] << " ";
            ROS_INFO_STREAM("sim " << _params.name_ref << ": " << ss.str());
        }

        // Handle ach errors.
        if(status != ACH_OK)
        {
            SNS_LOG(LOG_ERR, "ach_put failed: %s\n", ach_result_to_string(status));
            r.error_code = MotorGroupError::INVALID_JOINTS;
            r.error_string = std::string("Failed ach_put: ") + ach_result_to_string(status);
        }

        // Increment the send counter if no errors.
        if (!r)
            _state.num_sent++;

        return r;
    }

    MotorGroupError MotorGroup::applyGeneralPostConditions()
    {
        return MotorGroupError();
    }

    MotorGroupError SDHGroup::sendCommand(double t)
    {
        MotorGroupError ret;

        // HACK We only send the goal state to the SDH hands. Incremental
        // position commands causes the SDH to twitch violently.
        // ROS_INFO("%d %ld", !isMoving(), _state.num_sent);
        if (!isMoving() && _state.num_sent == 0)
        {
            // Get the duration of the trajectory.
            double t_end = _state.T->getDuration();

            // Get the final position in the trajectory.
            const Eigen::VectorXd& cmd = _state.T->getPosition(t_end);

            // Convert the trajectory command into the sequence we expect to send over ACH.
            Eigen::VectorXd x(_params.map.size());
            for (JointMap::iterator iter = _state.map.begin(); iter != _state.map.end(); ++iter)
            {
                x[_params.map[iter->first]] = cmd[iter->second];
            }

            // Build motor command for final position.
            if (ret = buildCommand(x))
                return ret;

            // Send motor command for final position.
            if (ret = MotorGroup::sendCommand(_state.ref))
                return ret;

        }

        return ret;
    }

    MotorGroupError LWA4Group::setVelocityToZero()
    {
        MotorGroupError ret;

        ROS_INFO("Setting velocity to 0");

        // Set to zero.
        Eigen::VectorXd p(_params.map.size());
        p.setZero();

        // Build reference command message.
        if (ret = buildCommand(p))
            return ret;

        // Send motor reference command.
        if (ret = sendCommand(_state.ref))
            return ret;

        return ret;
    }

    MotorGroupError LWA4Group::applyCustomPreConditions()
    {
        // BUG There is a bug in SCHUNK firmware that requires this.
        // BUG We do this before reading the motor state because the
        // BUG joints tend to move slightly even after setting velocity
        // BUG to zero.
        return setVelocityToZero();
    }

    MotorGroupError LWA4Group::applyCustomPostConditions()
    {
        // Make sure nothing is moving anymore.
        return setVelocityToZero();
    }

}
