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

#include "result.hpp"

#include <QGridLayout>
#include <rviz_common/display_context.hpp>

namespace autoware_practice_rviz_plugins
{

ResultPanel::ResultPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  label_result_ = new QLabel();
  label_result_->setAlignment(Qt::AlignCenter);

  const auto layout = new QGridLayout();
  layout->addWidget(label_result_);
  setLayout(layout);
}

void ResultPanel::onInitialize()
{
  auto lock = getDisplayContext()->getRosNodeAbstraction().lock();
  auto node = lock->get_raw_node();

  sub_result_ =
    node->create_subscription<JudgeStatus>("/evaluator/result", rclcpp::QoS(1), std::bind(&ResultPanel::onResult, this, std::placeholders::_1));
}

void ResultPanel::onResult(const JudgeStatus & msg)
{
  switch (msg.result) {
    case JudgeStatus::FAILURE:
      label_result_->setText("FAILURE");
      label_result_->setStyleSheet("background-color: red");
      break;
    case JudgeStatus::JUDGING:
      label_result_->setText("JUDGING");
      label_result_->setStyleSheet("background-color: yellow");
      break;
    case JudgeStatus::SUCCESS:
      label_result_->setText("SUCCESS");
      label_result_->setStyleSheet("background-color: lime");
      break;
    default:
      label_result_->setText("UNKNOWN");
      label_result_->setStyleSheet("background-color: gray");
      break;
  }
}

}  // namespace autoware_practice_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autoware_practice_rviz_plugins::ResultPanel, rviz_common::Panel)
