//  Copyright 2023 The Autoware Contributors
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#ifndef RESULT_HPP_
#define RESULT_HPP_

#include <QLabel>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <autoware_practice_msgs/msg/judge_status.hpp>

namespace autoware_practice_visualization
{

class ResultPanel : public rviz_common::Panel
{
  Q_OBJECT
  using JudgeStatus = autoware_practice_msgs::msg::JudgeStatus;

public:
  explicit ResultPanel(QWidget * parent = nullptr);
  void onInitialize() override;

private:
  QLabel * label_result_;
  rclcpp::Subscription<JudgeStatus>::SharedPtr sub_result_;
  void onResult(const JudgeStatus & msg);
};

}  // namespace autoware_practice_visualization

#endif  // RESULT_HPP_
