// Copyright 2024 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CONDITION_HPP_
#define CONDITION_HPP_

#include "geometry.hpp"
#include "visualization.hpp"

#include <yaml-cpp/yaml.h>

#include <memory>
#include <vector>

namespace autoware_practice_evaluator
{

enum class TriState : int
{
  Minimum = 0,  // Dummy value
  Failure = 1,  // False
  Judging = 2,  // Undefined
  Success = 3,  // True
  Maximum = 4   // Dummy value
};

struct JudgeInput
{
  Point point;
  LineString path;
};

class Condition
{
public:
  using UniquePtr = std::unique_ptr<Condition>;
  static Condition::UniquePtr make_unique(YAML::Node yaml);
  static std::vector<Condition *> descendants(Condition * node);

  virtual TriState update(const JudgeInput & data) = 0;
  virtual std::vector<RvizMarker> visualize() { return {}; }
  virtual std::vector<Condition *> children() { return {}; }
};

class LatchResult : public Condition
{
public:
  explicit LatchResult(YAML::Node yaml);
  TriState update(const JudgeInput & data) override;
  std::vector<Condition *> children() override;

private:
  TriState latch_;
  Condition::UniquePtr condition_;
};

class JudgeResult : public Condition
{
public:
  explicit JudgeResult(YAML::Node yaml);
  TriState update(const JudgeInput & data) override;
  std::vector<Condition *> children() override;

private:
  std::vector<Condition::UniquePtr> conditions_;
};

class LogicalAnd : public Condition
{
public:
  explicit LogicalAnd(YAML::Node yaml);
  TriState update(const JudgeInput & data) override;
  std::vector<Condition *> children() override;

private:
  std::vector<Condition::UniquePtr> conditions_;
};

class SuccessArea : public Condition
{
public:
  explicit SuccessArea(YAML::Node yaml);
  TriState update(const JudgeInput & data) override;
  std::vector<RvizMarker> visualize() override;

private:
  Polygon area_;
};

class FailureArea : public Condition
{
public:
  explicit FailureArea(YAML::Node yaml);
  TriState update(const JudgeInput & data) override;
  std::vector<RvizMarker> visualize() override;

private:
  Polygon area_;
};

class Constant : public Condition
{
public:
  explicit Constant(YAML::Node yaml);
  TriState update(const JudgeInput & data) override;

private:
  TriState value_;
};

}  // namespace autoware_practice_evaluator

#endif  // CONDITION_HPP_
