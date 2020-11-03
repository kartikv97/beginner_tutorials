/**
 * @copyright (c) 2020, Kartik Venkat
 *
 * @file talker.cpp
 * @authors
 * Kartik Venkat kartik.venkat86@gmail.com \n
 * @version 1.0
 *
 * @section LICENSE
 *
 * MIT License

 * @section DESCRIPTION:
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
#include <beginner_tutorials/UpdateString.h>
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"

std::string updatedWord = "This is the default string:";


/**
 * @brief Callback service to update string
 * @param request
 * @param response
 * @return
 */
bool updateString(beginner_tutorials::UpdateString::Request& request,
                  beginner_tutorials::UpdateString::Response& response) {
  updatedWord = request.newString;
  ROS_WARN_STREAM("The message being published has been updated");
  response.updatedString = updatedWord;
  return true;
}


int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::ServiceServer server = n.advertiseService("UpdateString", updateString);

  double publish_freq = atof(argv[1]);
  if (publish_freq < 0) {
    ROS_ERROR_STREAM("The values of publish frequency cannot be negative");
    publish_freq = 5;   // Minimum publish frequency
  }

  ros::Rate loop_rate(publish_freq);
  ROS_DEBUG_STREAM("current publish frequency:" << publish_freq);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok()) {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
//    ss << " This is the first ROS Assignment. " << count;
    ROS_DEBUG_STREAM("current count:" << count);
    ss << updatedWord << count;
    msg.data = ss.str();

    ROS_INFO_STREAM("%s" << msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  ROS_FATAL_STREAM("ROS service Terminated");
  return 0;
}
