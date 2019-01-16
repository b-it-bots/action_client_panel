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
  File: PickObjectClient.cpp
  Purpose: Class for implementing a PickObject action client

  @author Ahmed Faisal Abdelrahman
  @version 1.0 28/12/18
*/

#include "PickObjectClient.h"

#include <actionlib/client/terminal_state.h>

using namespace action_client_plugin;

PickObjectClient::PickObjectClient(YAML::Node node, float timeOut, QLabel* labelWidget) :
    ObjectManipulationClientBase(timeOut, labelWidget)
{
  actionName_ = "PickObject";
  serverName_ = "wbc_pick_object_server";
  ac_ = new actionlib::SimpleActionClient<
      mir_yb_action_msgs::PickObjectWBCAction>(serverName_, true);
  createGoal(node);
}

PickObjectClient::~PickObjectClient()
{
  if (ac_ != NULL)
  {
    delete ac_;
  }
}

void PickObjectClient::sendGoal()
{
  ROS_INFO("Waiting for action server to start.");

  if (ac_->waitForServer(ros::Duration(actionTimeOut_)))
  {
    ac_->sendGoal(goal_,
          boost::bind(&PickObjectClient::doneCallback, this, _1, _2), NULL,
          boost::bind(&PickObjectClient::feedbackCallback, this, _1));
    ROS_INFO("Action server found, sent goal.");

    printGoal();
  }
  else
  {
    ROS_INFO("The action server is not up yet. Try again.");
    resultTextBox_->setText((std::string("Server ") + serverName_ + std::string(" not found!")).c_str());
  }
}
void PickObjectClient::printGoal()
{
  ROS_INFO("%s Goal Details:", actionName_.c_str());
  ROS_INFO("Location : %s", goal_.location.c_str());
  ROS_INFO("Object : %s", goal_.object.c_str());
  std::cout << std::endl;
}

void PickObjectClient::createGoal(YAML::Node node)
{
  goal_.location = node["location"].as<std::string>();
  goal_.object = node["object"].as<std::string>();
}

void PickObjectClient::feedbackCallback(
    const mir_yb_action_msgs::PickObjectWBCFeedbackConstPtr& feedback)
{
  ROS_INFO("%s Feedback: Current State: %s. Text: %s.", actionName_.c_str(),
      feedback->current_state.c_str(), feedback->text.c_str());
}
void PickObjectClient::doneCallback(
    const actionlib::SimpleClientGoalState& state,
    const mir_yb_action_msgs::PickObjectWBCResultConstPtr& result)
{
  ROS_INFO("%s Result: %s", actionName_.c_str(),
      result->success ? "True" : "False");
  printActionStatus(state.toString());
}

