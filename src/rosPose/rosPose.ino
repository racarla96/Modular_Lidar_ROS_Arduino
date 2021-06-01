#include <ros.h>
ros::NodeHandle  nh;

#include <geometry_msgs/PoseStamped.h>
uint32_t msgPoseLastId;
geometry_msgs::PoseStamped msgPose;

void msgPoseStampCb(const geometry_msgs::PoseStamped & msgPoseStamp){
  msgPose = msgPoseStamp;
}

ros::Subscriber <geometry_msgs::PoseStamped> poseSub("slam_out_pose", &msgPoseStampCb);

void setup()
{
  SerialUSB.begin(115200);
  nh.initNode();
  nh.subscribe(poseSub);
}

void loop()
{
  nh.spinOnce();
  if(msgPose.header.seq != msgPoseLastId) {
    SerialUSB.print(msgPose.pose.position.x);
    SerialUSB.print(",");
    SerialUSB.print(msgPose.pose.position.y);
    SerialUSB.println();
    msgPoseLastId = msgPose.header.seq;
  }
  delay(10);
}
