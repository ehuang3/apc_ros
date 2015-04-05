#include <sstream>
#include <sns.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <argp.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdint.h>
#include <unistd.h>
#include <assert.h>
#include <time.h>
#include <syslog.h>
#include <dlfcn.h>
#include <unistd.h>


ach_channel_t chan_state;

ros::Publisher state_publisher;

int main(int argc, char** argv)
{
    // ros init
    ros::init(argc, argv, "apc_state_publisher");
    ros::NodeHandle n("~");

    // Topic for JointStates
    std::string topic = "/crichton/left_arm/joint_states";
    n.getParam("topic", topic);

    // Channel for motor state messages.
    std::string chan_name = "state-left";
    n.getParam("channel", chan_name);

    // Convineince typedef.
    typedef std::map<std::string, int> JointMap;

    // Mapping from joint names to motor state indicies.
    JointMap map;
    n.getParam("map", map);

    // ROS_INFO_STREAM(chan_name);

    // Create state publisher.
    state_publisher = n.advertise<sensor_msgs::JointState>(topic, 1);

    // Publish motor state message as JointState.
    sensor_msgs::JointState joint_state_msg;

    // Get the number of dofs.
    int n_dofs = map.size();

    // Resize joint states to correct size.
    joint_state_msg.name.resize(n_dofs);
    joint_state_msg.position.resize(n_dofs);
    joint_state_msg.velocity.resize(n_dofs);
    joint_state_msg.effort.resize(n_dofs);

    // Fill out joint state names.
    for (JointMap::iterator iter = map.begin(); iter != map.end(); ++iter)
    {
        // Add joint name to message.
        joint_state_msg.name[iter->second] = iter->first;

        // Set everything to zero.
        joint_state_msg.position[iter->second] = 0;
        joint_state_msg.velocity[iter->second] = 0;
        joint_state_msg.effort[iter->second] = 0;
    }

    // Initialize sns
    sns_init();
    sns_chan_open( &chan_state,  chan_name.c_str(),  NULL );
    {
        ach_channel_t *chans[] = {&chan_state, NULL};
        sns_sigcancel( chans, sns_sig_term_default );
    }
    sns_start();

    // TODO Find a better polling method.
    ros::Rate rate(30);
    rate.sleep();
    while (ros::ok())
    {
        // The frame size of the buffer.
        size_t frame_size;

        // Pointer to buffer that stores the message.
        void* buf;

        // Amount of time to wait for a new message.
        struct timespec* abstime = NULL;

        // We will get the latest message with optional timeout if
        // provided.
        int opt = ACH_O_LAST | (abstime ? ACH_O_WAIT : 0);

        // Get the motor state message.
        ach_status_t r = sns_msg_local_get( &chan_state, &buf, &frame_size, abstime, opt );

        // Pointer to motor state message.
        struct sns_msg_motor_state* msg;

        // Handle the return status. ach ok and missed frame are both
        // acceptable as we only want the latest motor state.
        switch (r)
        {
        case ACH_OK:
        case ACH_MISSED_FRAME:
            // Convert raw buffer to motor state message.
            msg = (struct sns_msg_motor_state*) buf;

            // Sanity check the size of the message.
            // assert(frame_size == sns_motor_state_size_n( (uint32_t) (*msg)->header.n )); // FIXME Why won't this compile?

            // Publish motor state message as JointState.
            joint_state_msg.header.stamp = ros::Time::now();

            // Fill out joint state positions, velocities, and efforts.
            for (int i = 0; i < msg->header.n; i++)
            {
                joint_state_msg.position[i] = msg->X[i].pos;
                joint_state_msg.velocity[i] = msg->X[i].vel;
                joint_state_msg.effort[i]   = 0;
                // std::cout << msg->X[i].pos << ", ";
            }
            // std::cout << std::endl;

            // Send out joint state.
            state_publisher.publish(joint_state_msg);

            break;
        case ACH_STALE_FRAMES:
            // No new messages.
            break;
        default:
            // Log error on failure.
            SNS_LOG(LOG_ERR, "Failed ach_get: %s\n", ach_result_to_string(r));

            exit(1);

            break;
        }

        // Free local memory.
        aa_mem_region_local_release();

        // usleep( (useconds_t)(1e6 / 30) );

        ros::spinOnce();
        rate.sleep();
    }
}
