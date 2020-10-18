#include <ros/ros.h>
#include<geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <cmath>

class swerve_control{
public:
    swerve_control(){
        vel_sub_ = nh.subscribe("/cmd_vel" , 1 , &swerve_control::robot_control , this);

        wheel_fr_pub_ = nh.advertise<std_msgs::Float64>("/swerve_drive_robot/wheel/wheel_controller_fr/command" ,1);
        wheel_fl_pub_ = nh.advertise<std_msgs::Float64>("/swerve_drive_robot/wheel/wheel_controller_fl/command" ,1);
        wheel_rr_pub_ = nh.advertise<std_msgs::Float64>("/swerve_drive_robot/wheel/wheel_controller_rr/command" ,1);
        wheel_rl_pub_ = nh.advertise<std_msgs::Float64>("/swerve_drive_robot/wheel/wheel_controller_rl/command" ,1);

        steering_fr_pub_ = nh.advertise<std_msgs::Float64>("/swerve_drive_robot/steering/steering_controller_fr/command" ,1);
        steering_fl_pub_ = nh.advertise<std_msgs::Float64>("/swerve_drive_robot/steering/steering_controller_fl/command" ,1);
        steering_rr_pub_ = nh.advertise<std_msgs::Float64>("/swerve_drive_robot/steering/steering_controller_rr/command" ,1);
        steering_rl_pub_ = nh.advertise<std_msgs::Float64>("/swerve_drive_robot/steering/steering_controller_rl/command" ,1);
    }

    void robot_control(const geometry_msgs::Twist& cmd_vel){
        rep_a = cmd_vel.linear.x -  cmd_vel.angular.z * (robot_length/2) ;
        rep_b = cmd_vel.linear.x +  cmd_vel.angular.z * (robot_width/2) ;
        rep_c = cmd_vel.linear.y -  cmd_vel.angular.z * (robot_length/2) ;
        rep_d = cmd_vel.linear.y +  cmd_vel.angular.z * (robot_width/2) ;

        float wheel_data[4];
        wheel_data[0] = std::sqrt(rep_a * rep_a + rep_c *rep_c);
        wheel_data[1] = std::sqrt(rep_b * rep_b + rep_c *rep_c);
        wheel_data[2] = std::sqrt(rep_a * rep_a + rep_d *rep_d);
        wheel_data[3] = std::sqrt(rep_b * rep_b + rep_d *rep_d);



        float max = wheel_data[0];
        for (size_t i = 0 ; i < 3; i ++){
            if(max < wheel_data[i]){
                max = wheel_data[i];
            }
        }

        if (max > max_speed){
            for (size_t i = 0 ; i < 3; i ++){
                wheel_data[i] = wheel_data[i] / max * max_speed ;
            }
        }


        float steering_data[4];
        steering_data[0] = std::atan2(rep_c ,rep_a) ; 
        steering_data[1] = std::atan2(rep_c ,rep_b) ;
        steering_data[2] = std::atan2(rep_d ,rep_a) ;
        steering_data[3] = std::atan2(rep_d ,rep_b) ;

        std_msgs::Float64 wheel_fr;
        std_msgs::Float64 wheel_fl;
        std_msgs::Float64 wheel_rr;
        std_msgs::Float64 wheel_rl;

        wheel_fr.data = wheel_data[0];
        wheel_fl.data = wheel_data[1];
        wheel_rr.data = wheel_data[2];
        wheel_rl.data = wheel_data[3];


        std_msgs::Float64 steering_fr;
        std_msgs::Float64 steering_fl;
        std_msgs::Float64 steering_rr;
        std_msgs::Float64 steering_rl;

        steering_fr.data = steering_data[0];
        steering_fl.data = steering_data[1];
        steering_rr.data = steering_data[2];
        steering_rl.data = steering_data[3];

        wheel_fr_pub_.publish(wheel_fr);
        wheel_fl_pub_.publish(wheel_fl);
        wheel_rr_pub_.publish(wheel_rr);
        wheel_rl_pub_.publish(wheel_rl);
        steering_fr_pub_.publish(steering_fr);
        steering_fl_pub_.publish(steering_fl);
        steering_rr_pub_.publish(steering_rr);
        steering_rl_pub_.publish(steering_rl);

        ROS_INFO("%f ,  %f ,  %f ,  %f m/s" ,wheel_fr , wheel_fl,wheel_rr ,wheel_rl);

        ROS_INFO("%f ,  %f ,  %f ,  %f rad" ,steering_fr , steering_fl,steering_rr ,steering_rl);

        
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber vel_sub_;

    ros::Publisher wheel_fr_pub_;
    ros::Publisher wheel_fl_pub_;
    ros::Publisher wheel_rr_pub_;
    ros::Publisher wheel_rl_pub_;
    ros::Publisher steering_fr_pub_;
    ros::Publisher steering_fl_pub_;
    ros::Publisher steering_rr_pub_;
    ros::Publisher steering_rl_pub_;

    const float robot_length = 0.06;//[m]
    const float robot_width = 0.06;//[m]
    const int max_speed = 10.0;//[m]
    const float PI = 3.1415;
    float rep_a;
    float rep_b;
    float rep_c;
    float rep_d;
};

int main (int argc, char** argv)
{
    ros::init (argc, argv, "swerve_control");
    swerve_control s;
    ros::spin();
    return 0;
}