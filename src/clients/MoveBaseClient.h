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
  File: MoveBaseClient.h
  Purpose: Class for implementing a MoveBase action client

  @author Ahmed Faisal Abdelrahman
  @version 1.0 28/12/18
*/

#ifndef ACTION_CLIENT_PLUGIN_MOVE_BASE_CLIENT_H_
#define ACTION_CLIENT_PLUGIN_MOVE_BASE_CLIENT_H_

#include <ros/ros.h>
#include "yaml-cpp/yaml.h"

#include "clientBase/ClientBase.h"
#include <actionlib/client/simple_action_client.h>
#include <mir_yb_action_msgs/MoveBaseSafeAction.h>

namespace action_client_plugin
{

class MoveBaseClient: public ClientBase
{
public:
  MoveBaseClient(YAML::Node node, float timeOut, QLabel* labelWidget);
  virtual ~MoveBaseClient();

  virtual void sendGoal();
  virtual void printGoal();

protected:
  virtual void createGoal(YAML::Node node);

  void feedbackCallback(const mir_yb_action_msgs::MoveBaseSafeFeedbackConstPtr& feedback);
  void doneCallback(const actionlib::SimpleClientGoalState& state,
      const mir_yb_action_msgs::MoveBaseSafeResultConstPtr& result);

  std::string armSafePosition_;
  std::string sourceLocation_;
  std::string destinationLocation_;
  std::string destinationOrientation_;
  bool dontBeSafe_;
  actionlib::SimpleActionClient<mir_yb_action_msgs::MoveBaseSafeAction>* ac_;
  mir_yb_action_msgs::MoveBaseSafeGoal goal_;
};

}
#endif /* MOVEBASECLIENT_H_ */
