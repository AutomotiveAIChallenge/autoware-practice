// Copyright 2024 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under #include <memory>the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "condition.hpp"

#include <boost/geometry/algorithms/intersects.hpp>

#include <algorithm>
#include <string>

namespace autoware_practice_evaluator
{

Point yaml_point2(YAML::Node yaml)
{
  double x = yaml[0].as<double>();
  double y = yaml[1].as<double>();
  return Point2(x, y);
}

Polygon yaml_polygon(YAML::Node yaml)
{
  Polygon polygon;
  for (const auto & node : yaml) {
    polygon.push_back(yaml_point2(node));
  }
  return polygon;
}

TriState normalize(TriState state)
{
  if (state == TriState::Minimum) return TriState::Judging;
  if (state == TriState::Maximum) return TriState::Judging;
  return state;
}

std::unique_ptr<Condition> Condition::make_unique(YAML::Node yaml)
{
  const auto type = yaml["type"].as<std::string>();
  if (type == "LatchResult") return std::make_unique<LatchResult>(yaml);
  if (type == "JudgeResult") return std::make_unique<JudgeResult>(yaml);
  if (type == "SuccessArea") return std::make_unique<SuccessArea>(yaml);
  if (type == "FailureArea") return std::make_unique<FailureArea>(yaml);
  if (type == "LogicalAnd") return std::make_unique<LogicalAnd>(yaml);
  throw std::runtime_error("unknown condition type: " + type);
}

std::vector<Condition *> Condition::descendants(Condition * node)
{
  std::vector<Condition *> result = {node};
  for (const auto & child : node->children()) {
    const auto list = descendants(child);
    result.insert(result.end(), list.begin(), list.end());
  }
  return result;
}

LatchResult::LatchResult(YAML::Node yaml)
{
  latch_ = TriState::Judging;
  condition_ = make_unique(yaml["node"]);
}

std::vector<Condition *> LatchResult::children()
{
  return {condition_.get()};
}

TriState LatchResult::update(const JudgeInput & data)
{
  if (latch_ == TriState::Judging) latch_ = condition_->update(data);
  return latch_;
}

JudgeResult::JudgeResult(YAML::Node yaml)
{
  for (const auto & node : yaml["list"]) {
    conditions_.push_back(make_unique(node));
  }
}

std::vector<Condition *> JudgeResult::children()
{
  std::vector<Condition *> result;
  for (const auto & condition : conditions_) {
    result.push_back(condition.get());
  }
  return result;
}

TriState JudgeResult::update(const JudgeInput & data)
{
  TriState result = TriState::Maximum;
  for (const auto & condition : conditions_) {
    const auto state = condition->update(data);
    if (state != TriState::Judging) {
      result = std::min(result, state);
    }
  }
  return normalize(result);
}

LogicalAnd::LogicalAnd(YAML::Node yaml)
{
  for (const auto & node : yaml["list"]) {
    conditions_.push_back(make_unique(node));
  }
}

std::vector<Condition *> LogicalAnd::children()
{
  std::vector<Condition *> result;
  for (const auto & condition : conditions_) {
    result.push_back(condition.get());
  }
  return result;
}

TriState LogicalAnd::update(const JudgeInput & data)
{
  TriState result = TriState::Success;
  for (const auto & condition : conditions_) {
    result = std::min(result, condition->update(data));
  }
  return result;
}

SuccessArea::SuccessArea(YAML::Node yaml)
{
  area_ = yaml_polygon(yaml["area"]);
}

TriState SuccessArea::update(const JudgeInput & data)
{
  return boost::geometry::intersects(data.path, area_) ? TriState::Success : TriState::Judging;
}

std::vector<RvizMarker> SuccessArea::visualize()
{
  RvizMarker marker;
  marker.color(0.0, 1.0, 0.0);
  marker.polygon(area_);
  return {marker};
}

FailureArea::FailureArea(YAML::Node yaml)
{
  area_ = yaml_polygon(yaml["area"]);
}

TriState FailureArea::update(const JudgeInput & data)
{
  return boost::geometry::intersects(data.path, area_) ? TriState::Failure : TriState::Judging;
}

std::vector<RvizMarker> FailureArea::visualize()
{
  RvizMarker marker;
  marker.color(1.0, 0.0, 0.0);
  marker.polygon(area_);
  return {marker};
}

}  // namespace autoware_practice_evaluator
