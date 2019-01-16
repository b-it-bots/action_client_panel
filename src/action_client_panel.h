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
  File: action_client_panel.h
  Purpose: RViz plugin for GUI-based dynamic action handling

  @author Ahmed Faisal Abdelrahman
  @author Sushant Vijay Chavan
  @version 1.0 28/12/18
*/

#ifndef ACTION_CLIENT_PANEL_H
#define ACTION_CLIENT_PANEL_H

#include <ros/ros.h>
#include <rviz/panel.h>
#include <yaml-cpp/yaml.h>

namespace rviz
{
class Property;
class EnumProperty;
class EditableEnumProperty;
class PropertyTreeWidget;
}

namespace YAML
{
  class Node;
}

class QPushButton;
class QLabel;

namespace action_client_plugin
{

class ClientBase;

class ActionClientPanel: public rviz::Panel
{
Q_OBJECT
public:
  ActionClientPanel( QWidget* parent = 0 );
  virtual ~ActionClientPanel();

  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

  virtual void onInitialize();

public Q_SLOTS:
  void handleButton();

protected:
  enum ActionParameterTypes
  {
    INVALID = 0,
    INT,
    FLOAT,
    STRING,
    BOOL,
    ENUM
  };

  virtual void createLayoutAndAddWidgets();
  virtual void addEnumProperty();
  virtual std::vector<std::string> getSortedKeysFromYAMLNode(const YAML::Node& node);

  virtual std::string getActionFilePath();
  virtual std::string getNavGoalsFilePath();

  virtual void executeInsertObjectAction();
  virtual void executeMoveBaseAction();
  virtual void executePerceiveCavityAction();
  virtual void executePerceiveLocationAction();
  virtual void executePickObjectAction();
  virtual void executePlaceObjectAction();
  virtual void executeStageObjectAction();
  virtual void executeUnstageObjectAction();
  virtual void executeInsertObjectInCavityAction();

  virtual YAML::Node getParametersFromProperties();
  virtual int getIntFromProperty(rviz::Property* property);
  virtual float getFloatFromProperty(rviz::Property* property);
  virtual std::string getStringFromProperty(rviz::Property* property);
  virtual bool getBoolFromProperty(rviz::Property* property);

  virtual std::string getPropertyName(rviz::Property* property);
  virtual ActionParameterTypes getActionParamType(const std::string& actionName, const std::string& paramName);

  virtual void addEntryToActionEnum(ActionParameterTypes paramType,
                                    const std::string& entryName,
                                    const std::string& entryDescription,
                                    const std::string& actionName);
  virtual void addStringsToEnum(rviz::EditableEnumProperty* property,
                                const std::vector<std::string>& strings);
  virtual std::vector<std::string> getEnumStrings(const std::string& actionName,
                                                  const std::string& parameterName);
  virtual std::vector<std::string> getEnumStringsFromActionsYAML(const std::string& actionName,
                                                                 const std::string& parameterName);
  virtual std::vector<std::string> getEnumStringsFromNavGoalsYAML();

private Q_SLOTS:
  void enumSelected();
  void updateActionParams();

private:
  rviz::Property* propertyContainer_;
  rviz::PropertyTreeWidget* tree_widget_;
  QPushButton* set_goal_button_;
  QLabel* resultTextBox_;
  ros::Publisher pub_;

  ros::Subscriber sub_;
  rviz::EnumProperty* enumProp_;
  std::map<int, std::string> actionsIndexMap_;
  std::map<std::string, int> propertyTypeMap_;

  YAML::Node actionsClientNode_;
  YAML::Node navigationGoalsNode_;

  std::vector<std::string> perceivedItems_;

  ClientBase* actionClient_;
};

} // end namespace action_client_plugin

#endif // ACTION_CLIENT_PANEL_H
