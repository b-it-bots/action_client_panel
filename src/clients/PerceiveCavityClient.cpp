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
  File: PerceiveCavityClient.cpp
  Purpose: Class for implementing a PerceiveCavity action client

  @author Sushant Vijay Chavan
  @version 1.0 28/12/18
*/

#include "PerceiveCavityClient.h"

using namespace action_client_plugin;

PerceiveCavityClient::PerceiveCavityClient(YAML::Node node, float timeOut, QLabel* labelWidget)
: PerceiveClientBase(timeOut, labelWidget)
{
  serverName_ = "perceive_cavity_server";
  ac_ = new actionlib::SimpleActionClient<mir_yb_action_msgs::PerceiveLocationAction>(serverName_ , true);
  actionName_ = "PerceiveCavityClient";
  createGoal(node);
}

PerceiveCavityClient::~PerceiveCavityClient()
{
  if (NULL != ac_)
  {
    delete ac_;
    ac_ = NULL;
  }
}

void PerceiveCavityClient::createGoal(YAML::Node node)
{
  goal_.location = node["location"].as<std::string>();
}

void PerceiveCavityClient::sendGoal()
{
  ROS_INFO("Waiting for action server to start.");

  if (ac_->waitForServer(ros::Duration(actionTimeOut_)))
  {
    ac_->sendGoal(goal_, boost::bind(&PerceiveCavityClient::doneCallback, this, _1, _2),
                    NULL, boost::bind(&PerceiveCavityClient::feedbackCallback, this, _1));

    ROS_INFO("Action server found, sent goal.");
    printGoal();
  }
  else
  {
    ROS_INFO("The action server is not up yet. Try again.");
    resultTextBox_->setText((std::string("Server ") + serverName_ + std::string(" not found!")).c_str());
  }
}

void PerceiveCavityClient::feedbackCallback(const mir_yb_action_msgs::PerceiveLocationFeedbackConstPtr& feedback)
{
  ROS_INFO("FEEDBACK CALLED --> State: %s, Test: %s", feedback->current_state.c_str(),feedback->text.c_str());
}

void PerceiveCavityClient::doneCallback(const actionlib::SimpleClientGoalState& state,
                            const mir_yb_action_msgs::PerceiveLocationResultConstPtr& result)
{
  printActionStatus(state.toString());
}

void PerceiveCavityClient::printGoal()
{
  ROS_INFO("%s Goal Details:", actionName_.c_str());
  ROS_INFO("Location : %s", goal_.location.c_str());
}
