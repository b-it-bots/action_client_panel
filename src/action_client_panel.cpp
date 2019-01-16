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
  File: action_client_panel.cpp
  Purpose: RViz plugin for GUI-based dynamic action handling

  @author Ahmed Faisal Abdelrahman
  @author Sushant Vijay Chavan
  @version 1.0 28/12/18
*/

#include <rviz/properties/property_tree_widget.h>
#include <rviz/properties/property_tree_model.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/vector_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/editable_enum_property.h>
#include <rviz/properties/bool_property.h>

#include <rviz/tool_properties_panel.h>
#include <rviz/tool_manager.h>
#include <rviz/visualization_manager.h>

#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>

#include "clients/PerceiveLocationClient.h"
#include "clients/PerceiveCavityClient.h"
#include "clients/StageObjectClient.h"
#include "clients/UnStageObjectClient.h"
#include "clients/MoveBaseClient.h"
#include "clients/PickObjectClient.h"
#include "clients/PlaceObjectClient.h"
#include "clients/InsertObjectClient.h"
#include "clients/InsertObjectInCavityClient.h"

#include "action_client_panel.h"

using namespace action_client_plugin;

ActionClientPanel::ActionClientPanel(QWidget* parent) :
    rviz::Panel(parent)
    , actionClient_(NULL)
{
}

ActionClientPanel::~ActionClientPanel()
{
  if (actionClient_ != NULL)
  {
    delete actionClient_;
    actionClient_ = NULL;
  }
}

void ActionClientPanel::createLayoutAndAddWidgets()
{
  set_goal_button_ = new QPushButton("Send Goal", this);
  set_goal_button_->setGeometry(QRect(QPoint(100, 100), QSize(200, 50)));

  connect(set_goal_button_, SIGNAL(released()), this, SLOT(handleButton()));

  resultTextBox_ = new QLabel("Result Widget", this);
  resultTextBox_->setStyleSheet("QLabel { background-color : white; color : black; }");
  resultTextBox_->setAlignment(Qt::AlignCenter);

  propertyContainer_ = new rviz::Property(
      QString::fromStdString("ActionClientRootProperty"));
  rviz::PropertyTreeModel* model = new rviz::PropertyTreeModel(
      propertyContainer_, this);
  tree_widget_ = new rviz::PropertyTreeWidget();
  tree_widget_->setModel(model);

  QVBoxLayout* layout = new QVBoxLayout();
  layout->addWidget(tree_widget_);
  layout->addWidget(set_goal_button_);
  layout->addWidget(resultTextBox_);
  setLayout(layout);
}

void ActionClientPanel::onInitialize()
{
  actionsClientNode_ = YAML::LoadFile(getActionFilePath());
  navigationGoalsNode_ = YAML::LoadFile(getNavGoalsFilePath());

  createLayoutAndAddWidgets();
  addEnumProperty();

  propertyTypeMap_.insert(std::pair<std::string, int>("int", ActionParameterTypes::INT));
  propertyTypeMap_.insert(std::pair<std::string, int>("float", ActionParameterTypes::FLOAT));
  propertyTypeMap_.insert(std::pair<std::string, int>("string", ActionParameterTypes::STRING));
  propertyTypeMap_.insert(std::pair<std::string, int>("bool", ActionParameterTypes::BOOL));
  propertyTypeMap_.insert(std::pair<std::string, int>("enum", ActionParameterTypes::ENUM));
}

std::string ActionClientPanel::getActionFilePath()
{
  std::stringstream ss;
  ss << ros::package::getPath("action_client_plugin") << "/";
  ss << "ActionClients.yaml";

  return ss.str();
}

std::string ActionClientPanel::getNavGoalsFilePath()
{
  std::stringstream ss;
  ss << ros::package::getPath("mcr_default_env_config") << "/";
  ss << std::getenv("ROBOT_ENV") << "/";
  ss << "navigation_goals.yaml";

  return ss.str();
}

void ActionClientPanel::save( rviz::Config config ) const
{
  rviz::Panel::save(config);
  tree_widget_->save(config);
}

void ActionClientPanel::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
  tree_widget_->load(config);
}

void ActionClientPanel::handleButton()
{
  std::string actionName = actionsIndexMap_[enumProp_->getOptionInt()];

  if (actionClient_ != NULL)
  {
    delete actionClient_;
    actionClient_ = NULL;
  }

  if (actionName == "insert_object")
    executeInsertObjectAction();
  else if (actionName == "insert_object_in_cavity")
    executeInsertObjectInCavityAction();
  else if (actionName == "move_base")
    executeMoveBaseAction();
  else if (actionName == "perceive_cavity")
    executePerceiveCavityAction();
  else if (actionName == "perceive_location")
    executePerceiveLocationAction();
  else if (actionName == "pick_object")
    executePickObjectAction();
  else if (actionName == "place_object")
    executePlaceObjectAction();
  else if (actionName == "stage_object")
    executeStageObjectAction();
  else if (actionName == "unstage_object")
    executeUnstageObjectAction();
  else
    ROS_WARN("No client implemented for the chosen action!!");
}

void ActionClientPanel::addEnumProperty()
{
  enumProp_ = new rviz::EnumProperty(
      QString::fromStdString(std::string("Actions")), QString(),
      "Enum property to list all possible actions", propertyContainer_,
      SLOT(enumSelected()), this);

  std::vector<std::string> actions = getSortedKeysFromYAMLNode(actionsClientNode_);
  for (unsigned int i = 0; i < actions.size(); i++)
  {
    int actionIdx = i+1;
    std::string currentAction = actions[i];

    enumProp_->addOptionStd(currentAction, actionIdx);
    actionsIndexMap_.insert(
        std::pair<int, std::string>(actionIdx, currentAction));
  }
}

std::vector<std::string> ActionClientPanel::getSortedKeysFromYAMLNode(const YAML::Node& node)
{
  std::vector<std::string> keys;

  for (const auto& n : node)
  {
    keys.push_back(n.first.as<std::string>());
  }

  std::sort(keys.begin(), keys.end());

  return keys;
}

ActionClientPanel::ActionParameterTypes ActionClientPanel::getActionParamType(const std::string& actionName,
                                                           const std::string& paramName)
{
  ActionParameterTypes retval = ActionParameterTypes::INVALID;

  YAML::Node parameterNode = actionsClientNode_[actionName];

    if (parameterNode[paramName].IsSequence())
    {
      retval = ActionParameterTypes::ENUM;
    }
    else
    {
      std::string type = parameterNode[paramName].as<std::string>();
      std::map<std::string, int>::iterator it = propertyTypeMap_.find(type);

      if (it == propertyTypeMap_.end())
      {
        ROS_ERROR("INVALID type \"%s\" specified for parameter \"%s\" in action \"%s\"", type.c_str(), paramName.c_str(), actionName.c_str());
      }
      else
      {
        retval = static_cast<ActionParameterTypes>(it->second);
      }
    }

    return retval;
}

void ActionClientPanel::enumSelected()
{
  resultTextBox_->clear();
  std::string actionName = actionsIndexMap_[enumProp_->getOptionInt()];
  enumProp_->removeChildren(0, enumProp_->numChildren());

  std::vector<std::string> parameters = getSortedKeysFromYAMLNode(actionsClientNode_[actionName]);

  for (unsigned int i = 0; i < parameters.size(); i++)
  {
    std::string paramName = parameters[i];

    std::stringstream ss;
    ss << actionName;
    ss << "-";
    ss << paramName;

    ActionParameterTypes paramType = getActionParamType(actionName, paramName);

    if (paramType == ActionParameterTypes::INVALID)
      continue;

    addEntryToActionEnum(paramType, paramName, ss.str(), actionName);
  }
}

void ActionClientPanel::addEntryToActionEnum(ActionParameterTypes paramType,
                                             const std::string& entryName,
                                             const std::string& entryDescription,
                                             const std::string& actionName)
{
  switch (paramType)
  {
    case ActionParameterTypes::INT:
    {
      rviz::IntProperty* prop = new rviz::IntProperty(
        QString::fromStdString(entryName), 0,
        entryDescription.c_str(), enumProp_, /*SLOT( updateActionParams() )*/ NULL, this);
      break;
    }
    case ActionParameterTypes::FLOAT:
    {
      rviz::FloatProperty* prop = new rviz::FloatProperty(
        QString::fromStdString(entryName), 0.0f,
        entryDescription.c_str(), enumProp_, /*SLOT( updateActionParams() )*/ NULL, this);
      break;
    }
    case ActionParameterTypes::STRING:
    {
      rviz::StringProperty* prop = new rviz::StringProperty(
        QString::fromStdString(entryName), QString(),
        entryDescription.c_str(), enumProp_, /*SLOT( updateActionParams() )*/ NULL, this);
      break;
    }
    case ActionParameterTypes::BOOL:
    {
      rviz::BoolProperty* prop = new rviz::BoolProperty(
        QString::fromStdString(entryName), false,
        entryDescription.c_str(), enumProp_, /*SLOT( updateActionParams() )*/ NULL, this);
      break;
    }
    case ActionParameterTypes::ENUM:
    {
      rviz::EditableEnumProperty* prop = new rviz::EditableEnumProperty (QString::fromStdString(entryName),
       QString(), entryDescription.c_str(), enumProp_, /*SLOT( updateActionParams() )*/ NULL, this);

      addStringsToEnum(prop, getEnumStrings(actionName, entryName));
      break;
    }
      default:
      break;
  }
}

void ActionClientPanel::addStringsToEnum(rviz::EditableEnumProperty* property, const std::vector<std::string>& strings)
{
  for(unsigned int i = 0; i < strings.size(); i++)
  {
    property->addOptionStd(strings[i]);
  }
}

std::vector<std::string> ActionClientPanel::getEnumStrings(const std::string& actionName, const std::string& parameterName)
{
  std::vector<std::string> enumStrings;
  if ((actionName == "move_base" && parameterName == "destination_orientation") ||
      (actionName == "move_base" && parameterName == "arm_safe_position") ||
      (actionName == "insert_object" && parameterName == "robot_platform") ||
      (actionName == "insert_object_in_cavity" && parameterName == "robot_platform") ||
      (actionName == "stage_object" && parameterName == "robot_platform") ||
      (actionName == "unstage_object" && parameterName == "robot_platform"))
  {
    enumStrings = getEnumStringsFromActionsYAML(actionName, parameterName);
  }
  else if ((actionName == "move_base" && parameterName == "source_location") ||
           (actionName == "move_base" && parameterName == "destination_location") ||
           (actionName == "perceive_cavity" && parameterName == "location") ||
           (actionName == "perceive_location" && parameterName == "location") ||
           (actionName == "pick_object" && parameterName == "location") ||
           (actionName == "place_object" && parameterName == "location"))
  {
    enumStrings = getEnumStringsFromNavGoalsYAML();
  }
  else if ((actionName == "pick_object" && parameterName == "object"))
  {
    enumStrings = perceivedItems_;
  }
  else
  {
    ROS_WARN("Unable to load enum values for parameter \"%s\" in action \"%s\"", parameterName.c_str(), actionName.c_str());
  }

  std::sort(enumStrings.begin(), enumStrings.end());

  return enumStrings;
}

std::vector<std::string> ActionClientPanel::getEnumStringsFromActionsYAML(const std::string& actionName, const std::string& parameterName)
{
  std::vector<std::string> enumStrings;
  YAML::Node node = actionsClientNode_[actionName][parameterName];
  for (unsigned int i = 0; i < node.size(); i++)
  {
    enumStrings.push_back(node[i].as<std::string>());
  }

  return enumStrings;
}

std::vector<std::string> ActionClientPanel::getEnumStringsFromNavGoalsYAML()
{
  std::vector < std::string > enumStrings;

  for (const auto& g : navigationGoalsNode_)
  {
    enumStrings.push_back(g.first.as<std::string>());
  }
  return enumStrings;
}

void ActionClientPanel::updateActionParams()
{
}

std::string ActionClientPanel::getPropertyName(rviz::Property* property)
{
  return std::string(property->getName().toUtf8().constData());
}

void ActionClientPanel::executeInsertObjectAction()
{
  ROS_INFO("Received request for executing InsertObject...");

  YAML::Node node = getParametersFromProperties();

  actionClient_ = new InsertObjectClient(node, 10.0, resultTextBox_);
  actionClient_->sendGoal();
}
void ActionClientPanel::executeInsertObjectInCavityAction()
{
  ROS_INFO("Received request for executing InsertObjectInCavity...");

  YAML::Node node = getParametersFromProperties();

  actionClient_ = new InsertObjectInCavityClient(node, 10.0, resultTextBox_);
  actionClient_->sendGoal();
}
void ActionClientPanel::executeMoveBaseAction()
{
  ROS_INFO("Received request for executing MoveBase...");

  YAML::Node node = getParametersFromProperties();

  actionClient_ = new MoveBaseClient(node, 10.0, resultTextBox_);
  actionClient_->sendGoal();
}
void ActionClientPanel::executePerceiveCavityAction()
{
  ROS_INFO("Received request for executing PerceiveCavity...");

  YAML::Node node = getParametersFromProperties();

  actionClient_ = new PerceiveCavityClient(node, 10.0, resultTextBox_);
  actionClient_->sendGoal();
}

void ActionClientPanel::executePerceiveLocationAction()
{
  ROS_INFO("Received request for executing PerceiveLocation...");

  YAML::Node node = getParametersFromProperties();

  actionClient_ = new PerceiveLocationClient(node, 10.0, resultTextBox_, perceivedItems_);
  actionClient_->sendGoal();
}

void ActionClientPanel::executePickObjectAction()
{
  ROS_INFO("Received request for executing PickObject...");

  YAML::Node node = getParametersFromProperties();

  actionClient_ = new PickObjectClient(node, 10.0, resultTextBox_);
  actionClient_->sendGoal();
}
void ActionClientPanel::executePlaceObjectAction()
{
  ROS_INFO("Received request for executing PlaceObject...");

  YAML::Node node = getParametersFromProperties();

  actionClient_ = new PlaceObjectClient(node, 10.0, resultTextBox_);
  actionClient_->sendGoal();
}

void ActionClientPanel::executeStageObjectAction()
{
  ROS_INFO("Received request for executing StageObject...");

  YAML::Node node = getParametersFromProperties();

  actionClient_ = new StageObjectClient(node, 10.0, resultTextBox_);
  actionClient_->sendGoal();
}

void ActionClientPanel::executeUnstageObjectAction()
{
  ROS_INFO("Received request for executing UnstageObject...");

  YAML::Node node = getParametersFromProperties();

  actionClient_ = new UnStageObjectClient(node, 10.0, resultTextBox_);
  actionClient_->sendGoal();
}

YAML::Node ActionClientPanel::getParametersFromProperties()
{
  std::string actionName = actionsIndexMap_[enumProp_->getOptionInt()];
  YAML::Node parameterNode;

  for (unsigned int i = 0; i < enumProp_->numChildren(); i++)
  {
    rviz::Property* prop = enumProp_->childAt(i);
    std::string paramName = getPropertyName(prop);

    ActionParameterTypes paramType = getActionParamType(actionName, paramName);

    if (paramType == ActionParameterTypes::INVALID)
      continue;

    switch (paramType)
    {
      case ActionParameterTypes::INT:
      {
        parameterNode[paramName] = getIntFromProperty(prop);
        break;
      }
      case ActionParameterTypes::FLOAT:
      {
        parameterNode[paramName] = getFloatFromProperty(prop);
        break;
      }
      case ActionParameterTypes::STRING:
      case ActionParameterTypes::ENUM:
      {
        parameterNode[paramName] = getStringFromProperty(prop);
        break;
      }
      case ActionParameterTypes::BOOL:
      {
        parameterNode[paramName] = getBoolFromProperty(prop);
        break;
      }
        default:
        break;
    }
  }

  return parameterNode;
}

int ActionClientPanel::getIntFromProperty(rviz::Property* property)
{
  int retval = 0;
  rviz::IntProperty* iprop = dynamic_cast<rviz::IntProperty*>(property);
  if (iprop)
  {
    retval = iprop->getInt();
  } else
  {
    ROS_WARN("Unable to cast property %s to IntProperty!",
        getPropertyName(property).c_str());
  }

  return retval;
}

float ActionClientPanel::getFloatFromProperty(rviz::Property* property)
{
  float retval = 0.0f;
  rviz::FloatProperty* fprop = dynamic_cast<rviz::FloatProperty*>(property);
  if (fprop)
  {
    retval = fprop->getFloat();
  } else
  {
    ROS_WARN("Unable to cast property %s to FloatProperty!",
        getPropertyName(property).c_str());
  }

  return retval;
}

std::string ActionClientPanel::getStringFromProperty(rviz::Property* property)
{
  std::string retval = "";
  rviz::StringProperty* sprop = dynamic_cast<rviz::StringProperty*>(property);
  if (sprop)
  {
    retval = sprop->getString().toUtf8().constData();
  } else
  {
    ROS_WARN("Unable to cast property %s to StringProperty!",
        getPropertyName(property).c_str());
  }

  return retval;
}

bool ActionClientPanel::getBoolFromProperty(rviz::Property* property)
{
  bool retval = false;
  rviz::BoolProperty* bprop = dynamic_cast<rviz::BoolProperty*>(property);
  if (bprop)
  {
    retval = bprop->getBool();
  } else
  {
    ROS_WARN("Unable to cast property %s to BoolProperty!",
        getPropertyName(property).c_str());
  }

  return retval;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(action_client_plugin::ActionClientPanel,rviz::Panel )
