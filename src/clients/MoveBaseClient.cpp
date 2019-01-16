/*
 * Copyright © 2018 Ahmed Faisal Abdelrahman, Sushant Vijay Chavan All rights reserved.

 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright notice, this
 *       list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright notice, this
 *       list of conditions and the following disclaimer in the documentation and/or
 *       other materials provided with the distribution.
 *     * Neither the name of “Hochschule Bonn-Rhein-Sieg” nor the names of its contributors
 *       may be used to endorse or promote products derived from this software without specific
 *       prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS” AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/**
  File: MoveBaseClient.cpp
  Purpose: Class for implementing a MoveBase action client

  @author Ahmed Faisal Abdelrahman
  @version 1.0 28/12/18
*/

#include "MoveBaseClient.h"
#include <actionlib/client/terminal_state.h>

#include <mir_yb_action_msgs/MoveBaseSafeAction.h>

using namespace action_client_plugin;

MoveBaseClient::MoveBaseClient(YAML::Node node, float timeOut, QLabel* labelWidget)
: ClientBase(timeOut, labelWidget)
{
  actionName_ = "MoveBase";
  serverName_ = "move_base_safe_server";
  ac_ = new actionlib::SimpleActionClient<mir_yb_action_msgs::MoveBaseSafeAction>(serverName_.c_str(), true);
  createGoal(node);
}

MoveBaseClient::~MoveBaseClient()
{
  if (ac_ != NULL) {
    delete ac_;
  }
}

void MoveBaseClient::sendGoal()
{
  ROS_INFO("Waiting for action server to start.");

  if (ac_->waitForServer(ros::Duration(actionTimeOut_)))
  {
    ac_->sendGoal(goal_,
          boost::bind(&MoveBaseClient::doneCallback, this, _1, _2),
          NULL,
          boost::bind(&MoveBaseClient::feedbackCallback, this, _1));
    ROS_INFO("Action server found, sent goal.");

    printGoal();
  }
  else
  {
    ROS_INFO("The action server is not up yet. Try again.");
    resultTextBox_->setText((std::string("Server ") + serverName_ + std::string(" not found!")).c_str());
  }
}
void MoveBaseClient::printGoal()
{
  ROS_INFO("%s Goal Details:", actionName_.c_str());
  ROS_INFO("Source Location : %s", goal_.source_location.c_str());
  ROS_INFO("Destination Location : %s", goal_.destination_location.c_str());
  ROS_INFO("Destination Orientation : %s", goal_.destination_orientation.c_str());
  ROS_INFO("Arm Safe Position : %s", goal_.arm_safe_position.c_str());
  ROS_INFO("Don't Be Safe? : %s", goal_.dont_be_safe ? "True" : "False");
}

void MoveBaseClient::createGoal(YAML::Node node)
{
  goal_.arm_safe_position = node["arm_safe_position"].as<std::string>();
  goal_.source_location = node["source_location"].as<std::string>();
  goal_.destination_location = node["destination_location"].as<std::string>();
  goal_.destination_orientation = node["destination_orientation"].as<std::string>();
  goal_.dont_be_safe = node["dont_be_safe"].as<bool>();
}

void MoveBaseClient::feedbackCallback(const mir_yb_action_msgs::MoveBaseSafeFeedbackConstPtr& feedback)
{
  ROS_INFO("%s Feedback: Current State: %s. Text: %s.", actionName_.c_str(), feedback->current_state.c_str(), feedback->text.c_str());
}
void MoveBaseClient::doneCallback(const actionlib::SimpleClientGoalState& state,
    const mir_yb_action_msgs::MoveBaseSafeResultConstPtr& result)
{
  ROS_INFO("%s Result: %s", actionName_.c_str(), result->success ? "True": "False");
  printActionStatus(state.toString());
}

