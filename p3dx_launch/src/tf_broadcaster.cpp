#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){

	ros::init(argc, argv, "p3dx_tf_publisher");
	ros::NodeHandle n;

	ros::Rate r(100);

	tf::TransformBroadcaster bc;

	while(n.ok()){
		bc.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0.2,0.0,0.17)),
			ros::Time::now(), "base_link", "laser"));

		//bc.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0.2,0.0,0.17)),
		//	ros::Time::now(), "level_mux_map", "laser"));
		
		bc.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0.15,0.0,0.06)),
			ros::Time::now(), "base_link", "sonar")); //Change from sonar_frame to sonar
		r.sleep();
	}

}
