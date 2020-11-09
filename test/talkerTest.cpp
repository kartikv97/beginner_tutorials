/**
 * @copyright (c) 2020, Kartik Venkat
 *
 * @file talkerTest.cpp
 * @authors
 * Kartik Venkat kartik.venkat86@gmail.com \n
 * @version 1.0
 *
 * @section LICENSE
 *
 * MIT License

 * @section DESCRIPTION:
 * This is a unit test for the talker node.
 */

#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>
#include "beginner_tutorials/UpdateString.h"
#include "std_msgs/String.h"

TEST(UnitTest, RosServiceTest) {
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<beginner_tutorials::UpdateString>(
      "UpdateString");
  /**
   * @brief Verify if the service exists.
   */
  bool exists(client.waitForExistence(ros::Duration(5)));
  EXPECT_TRUE(exists);

  beginner_tutorials::UpdateString srv;
  srv.request.newString = "TEST STRING";
  client.call(srv.request, srv.response);
  /**
   * @brief Verify if the published string gets updated
   */
  EXPECT_EQ("TEST STRING", srv.response.updatedString);
}
/**
 * @brief Run all the Unit Tests
 * @param argc
 * @param argv
 * @return function call to Run all tests.
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "talkerTest");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
