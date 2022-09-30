/**
 ***********************************************************************************************************************
 *
 * @author ZhangRan
 * @date   2021/08/07
 *
 * <h2><center>&copy; COPYRIGHT 2021 Dobot CORPORATION</center></h2>
 *
 ***********************************************************************************************************************
 */

#include <ros/ros.h>
#include <ros/param.h>
#include <bringup/robot.h>
#include <sensor_msgs/JointState.h>
#include <bringup/ToolVectorActual.h>

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "M1ProRobot");

    try
    {
        ros::NodeHandle private_node("~");

        ros::AsyncSpinner async_spinner(1);
        async_spinner.start();

        sensor_msgs::JointState joint_state_msg;
        ros::Publisher joint_state_pub = private_node.advertise<sensor_msgs::JointState>("/joint_states", 100);
        bringup::RobotStatus robot_status_msg;
        ros::Publisher robot_status_pub = private_node.advertise<bringup::RobotStatus>("msg/RobotStatus", 100);

        bringup::ToolVectorActual tool_vector_actual_msg;
        ros::Publisher tool_vector_pub =
            private_node.advertise<bringup::ToolVectorActual>("msg/ToolVectorActual", 100);

        M1ProRobot robot(private_node, "joint_state_controller/JointStateController");

        double rate_vale = private_node.param("JointStatePublishRate", 10);

        joint_state_msg.name.push_back("joint1");
        joint_state_msg.name.push_back("joint2");
        joint_state_msg.name.push_back("joint3");
        joint_state_msg.name.push_back("joint4");

        joint_state_msg.position.resize(4);

        robot.init();
        ros::Rate rate(rate_vale);
        double position[6] = {};
        while (ros::ok())
        {
            //
            // publish joint state
            //
            robot.getJointState(position);
            joint_state_msg.header.stamp = ros::Time::now();
            joint_state_msg.header.frame_id = "base_link";
            
            joint_state_msg.position[0] = position[0];
            joint_state_msg.position[1] = position[1];
            joint_state_msg.position[2] = position[2];
            joint_state_msg.position[3] = position[3];

            joint_state_pub.publish(joint_state_msg);

            //
            // publish coordinate state
            //
            double val[6] = {};
            robot.getToolVectorActual(val);
            tool_vector_actual_msg.x = val[0];
            tool_vector_actual_msg.y = val[1];
            tool_vector_actual_msg.z = val[2];
            tool_vector_actual_msg.rx = val[3];
            tool_vector_actual_msg.ry = val[4];
            tool_vector_actual_msg.rz = val[5];
            tool_vector_pub.publish(tool_vector_actual_msg);

            //
            // publish robot status
            //
            robot_status_msg.is_enable = robot.isEnable();
            robot_status_msg.is_connected = robot.isConnected();
            robot_status_pub.publish(robot_status_msg);

            rate.sleep();
        }
    }
    catch (const std::exception& err)
    {
        ROS_ERROR("%s", err.what());
        return -1;
    }

    return 0;
}
