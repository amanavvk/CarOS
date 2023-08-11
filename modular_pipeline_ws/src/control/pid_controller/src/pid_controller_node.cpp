#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

class PIDControllerNode
{
public:
    PIDControllerNode()
    {
        // Initialize ROS node handle
        nh_ = ros::NodeHandle("~");

        // Subscribe to the desired setpoint topic
        setpoint_sub_ = nh_.subscribe("setpoint", 10, &PIDControllerNode::setpointCallback, this);

        // Subscribe to the feedback topic
        feedback_sub_ = nh_.subscribe("feedback", 10, &PIDControllerNode::feedbackCallback, this);

        // Publish the control command
        control_pub_ = nh_.advertise<geometry_msgs::Twist>("control_command", 10);

        // Load PID parameters from ROS parameter server
        nh_.param("kp", kp_, 0.1);
        nh_.param("ki", ki_, 0.01);
        nh_.param("kd", kd_, 0.001);

        // Initialize PID error terms
        prev_error_ = 0.0;
        integral_ = 0.0;

        // Initialize the PID loop rate
        nh_.param("pid_rate", pid_rate_, 10.0);
        pid_timer_ = nh_.createTimer(ros::Duration(1.0 / pid_rate_), &PIDControllerNode::pidLoop, this);
    }

    void setpointCallback(const std_msgs::Float64::ConstPtr& msg)
    {
        setpoint_ = msg->data;
    }

    void feedbackCallback(const std_msgs::Float64::ConstPtr& msg)
    {
        feedback_ = msg->data;
    }

    void pidLoop(const ros::TimerEvent& event)
    {
        double error = setpoint_ - feedback_;
        integral_ += error;

        // Calculate control command using PID formula
        double control_command = kp_ * error + ki_ * integral_ + kd_ * (error - prev_error_);

        // Create and publish control message
        geometry_msgs::Twist control_msg;
        control_msg.linear.x = control_command;

        control_pub_.publish(control_msg);

        // Update previous error
        prev_error_ = error;
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber setpoint_sub_;
    ros::Subscriber feedback_sub_;
    ros::Publisher control_pub_;
    ros::Timer pid_timer_;

    double setpoint_;
    double feedback_;

    double kp_;
    double ki_;
    double kd_;

    double prev_error_;
    double integral_;

    double pid_rate_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pid_controller_node");
    PIDControllerNode pid_controller_node;

    ros::spin();

    return 0;
}
