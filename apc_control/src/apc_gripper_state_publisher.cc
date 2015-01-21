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
	ros::init(argc, argv, "gripper_state_publisher");
	ros::NodeHandle n("~");

	// Left or right.
	std::string side = "left";
	n.getParam("side", side);

	// Topic for JointStates
	std::string topic = "/crichton/left_hand/joint_states";
	n.getParam("topic", topic);

	// Channel for motor state messages.
	std::string chan_name = "sdhstate-left";
	n.getParam("channel", chan_name);

	// Create state publisher.
	state_publisher = n.advertise<sensor_msgs::JointState>(topic, 1);

	// Publish motor state message as JointState.
	sensor_msgs::JointState joint_state_msg;

	// Resize joint states to correct size.
	joint_state_msg.position.resize(7);
	joint_state_msg.velocity.resize(7);
	joint_state_msg.effort.resize(7);

	// Joint names.
	std::vector<std::string> joint_names;
    joint_names.push_back("knuckle_joint");
	joint_names.push_back("finger_22_joint");
    joint_names.push_back("finger_23_joint");
    joint_names.push_back("thumb_2_joint");
    joint_names.push_back("thumb_3_joint");
    joint_names.push_back("finger_12_joint");
    joint_names.push_back("finger_13_joint");

	// Fill out joint state names.
	for (int i = 1; i <= 7; i++)
	{
		std::stringstream ss;
		ss << "crichton_" << side << "_" << joint_names[i-1];

		joint_state_msg.name.push_back(ss.str());

		// Set joint state to zero.
		joint_state_msg.position[i-1] = 0;
		joint_state_msg.velocity[i-1] = 0;
		joint_state_msg.effort[i-1]   = 0;
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
			}

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

		ros::spinOnce();
		rate.sleep();
	}
}
