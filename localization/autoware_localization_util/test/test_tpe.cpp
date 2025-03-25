// Copyright 2023 The Autoware Contributors
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

#include "autoware/localization_util/tree_structured_parzen_estimator.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

using TreeStructuredParzenEstimator = autoware::localization_util::TreeStructuredParzenEstimator;

TEST(TreeStructuredParzenEstimatorTest, TPE_is_better_than_random_search_on_sphere_function)
{
  auto sphere_function = [](const TreeStructuredParzenEstimator::Input & input) {
    double value = 0.0;
    const auto n = static_cast<int64_t>(input.size());
    for (int64_t i = 0; i < n; i++) {
      const double v = input[i] * 10;
      value += v * v;
    }
    return value;
  };

  constexpr int64_t k_outer_trials_num = 20;
  constexpr int64_t k_inner_trials_num = 200;
  std::cout << std::fixed;
  std::vector<double> mean_scores;
  std::vector<double> sample_mean(5, 0.0);
  std::vector<double> sample_stddev{1.0, 1.0, 0.1, 0.1, 0.1};

  for (const int64_t n_startup_trials : {k_inner_trials_num, k_inner_trials_num / 2}) {
    const std::string method = ((n_startup_trials == k_inner_trials_num) ? "Random" : "TPE");

    std::vector<double> scores;
    for (int64_t i = 0; i < k_outer_trials_num; i++) {
      double best_score = std::numeric_limits<double>::lowest();
      TreeStructuredParzenEstimator estimator(
        TreeStructuredParzenEstimator::Direction::MAXIMIZE, n_startup_trials, sample_mean,
        sample_stddev);
      for (int64_t trial = 0; trial < k_inner_trials_num; trial++) {
        const TreeStructuredParzenEstimator::Input input = estimator.get_next_input();
        const double score = -sphere_function(input);
        estimator.add_trial({input, score});
        best_score = std::max(best_score, score);
      }
      scores.push_back(best_score);
    }

    const double sum = std::accumulate(scores.begin(), scores.end(), 0.0);
    const double mean = sum / static_cast<double>(scores.size());
    mean_scores.push_back(mean);
    double sq_sum = std::accumulate(
      scores.begin(), scores.end(), 0.0,
      [mean](double total, double score) { return total + (score - mean) * (score - mean); });
    const double stddev = std::sqrt(sq_sum / static_cast<double>(scores.size()));

    std::cout << method << ", mean = " << mean << ", stddev = " << stddev << std::endl;
  }
  ASSERT_LT(mean_scores[0], mean_scores[1]);
}


TEST(TreeStructuredParzenEstimatorTest, MinimizationTest)
{
  auto quadratic_function = [](const TreeStructuredParzenEstimator::Input & input) {
    return input[0] * input[0] + input[1] * input[1];
  };

  constexpr int64_t k_trials_num = 100;
  std::vector<double> sample_mean{0.0, 0.0};
  std::vector<double> sample_stddev{1.0, 1.0};

  double best_score = std::numeric_limits<double>::max();
  TreeStructuredParzenEstimator estimator(
    TreeStructuredParzenEstimator::Direction::MINIMIZE, k_trials_num / 2, sample_mean,
    sample_stddev);
  
  for (int64_t trial = 0; trial < k_trials_num; trial++) {
    const TreeStructuredParzenEstimator::Input input = estimator.get_next_input();
    const double score = quadratic_function(input);
    estimator.add_trial({input, score});
    best_score = std::min(best_score, score);
  }

  EXPECT_LT(best_score, 0.1);  // Should find a solution close to 0
}

TEST(TreeStructuredParzenEstimatorTest, SingleDimensionTest)
{
  auto linear_function = [](const TreeStructuredParzenEstimator::Input & input) {
    return input[0];
  };

  constexpr int64_t k_trials_num = 50;
  std::vector<double> sample_mean{0.0};
  std::vector<double> sample_stddev{1.0};

  double best_score = std::numeric_limits<double>::lowest();
  TreeStructuredParzenEstimator estimator(
    TreeStructuredParzenEstimator::Direction::MAXIMIZE, k_trials_num / 2, sample_mean,
    sample_stddev);
  
  for (int64_t trial = 0; trial < k_trials_num; trial++) {
    const TreeStructuredParzenEstimator::Input input = estimator.get_next_input();
    const double score = linear_function(input);
    estimator.add_trial({input, score});
    best_score = std::max(best_score, score);
  }

  EXPECT_GT(best_score, 2.0);  // Should find a value in the right tail
}

TEST(TreeStructuredParzenEstimatorTest, EmptyTrials)
{
  std::vector<double> sample_mean{0.0, 0.0};
  std::vector<double> sample_stddev{1.0, 1.0};

  TreeStructuredParzenEstimator estimator(
    TreeStructuredParzenEstimator::Direction::MAXIMIZE, 10, sample_mean, sample_stddev);
  
  // Should not crash when getting input without any trials
  const TreeStructuredParzenEstimator::Input input = estimator.get_next_input();
  EXPECT_EQ(input.size(), 2u);
}

TEST(TreeStructuredParzenEstimatorTest, SingleTrial)
{
  std::vector<double> sample_mean{0.0, 0.0};
  std::vector<double> sample_stddev{1.0, 1.0};

  TreeStructuredParzenEstimator estimator(
    TreeStructuredParzenEstimator::Direction::MAXIMIZE, 0, sample_mean, sample_stddev);
  
  // Add one trial
  const TreeStructuredParzenEstimator::Input first_input{1.0, 1.0};
  estimator.add_trial({first_input, 1.0});
  
  // Should be able to get next input
  const TreeStructuredParzenEstimator::Input next_input = estimator.get_next_input();
  EXPECT_EQ(next_input.size(), 2u);
}

TEST(TreeStructuredParzenEstimatorTest, DifferentSampleStddev)
{
  std::vector<double> sample_mean{0.0, 0.0, 0.0};
  std::vector<double> sample_stddev{1.0, 0.1, 10.0};  // Different scales for different dimensions

  TreeStructuredParzenEstimator estimator(
    TreeStructuredParzenEstimator::Direction::MAXIMIZE, 10, sample_mean, sample_stddev);
  
  // Add some trials
  for (int i = 0; i < 5; ++i) {
    TreeStructuredParzenEstimator::Input input{0.1 * i, 0.01 * i, 1.0 * i};
    estimator.add_trial({input, static_cast<double>(i)});
  }
  
  // Get next input and verify it's within reasonable bounds
  const TreeStructuredParzenEstimator::Input next_input = estimator.get_next_input();
  EXPECT_NEAR(next_input[0], 0.0, 2.0);  // stddev = 1.0
  EXPECT_NEAR(next_input[1], 0.0, 0.2);  // stddev = 0.1
  EXPECT_NEAR(next_input[2], 0.0, 20.0); // stddev = 10.0
}

TEST(TreeStructuredParzenEstimatorTest, FixedIndexTest)
{
  // Test that the enum values are as expected
  EXPECT_EQ(TreeStructuredParzenEstimator::TRANS_X, 0);
  EXPECT_EQ(TreeStructuredParzenEstimator::TRANS_Y, 1);
  EXPECT_EQ(TreeStructuredParzenEstimator::TRANS_Z, 2);
  EXPECT_EQ(TreeStructuredParzenEstimator::ANGLE_X, 3);
  EXPECT_EQ(TreeStructuredParzenEstimator::ANGLE_Y, 4);
  EXPECT_EQ(TreeStructuredParzenEstimator::ANGLE_Z, 5);
  EXPECT_EQ(TreeStructuredParzenEstimator::INDEX_NUM, 6);
}

TEST(TreeStructuredParzenEstimatorTest, StartupTrialsBehavior)
{
  std::vector<double> sample_mean{0.0, 0.0};
  std::vector<double> sample_stddev{1.0, 1.0};

  // With n_startup_trials = 5, first 5 trials should be random
  const int64_t n_startup_trials = 5;
  TreeStructuredParzenEstimator estimator(
    TreeStructuredParzenEstimator::Direction::MAXIMIZE, n_startup_trials, sample_mean,
    sample_stddev);
  
  // Add startup trials
  for (int i = 0; i < n_startup_trials; ++i) {
    const auto input = estimator.get_next_input();
    estimator.add_trial({input, static_cast<double>(i)});
  }
  
  // After startup trials, the algorithm should start using TPE
  const auto tpe_input = estimator.get_next_input();
  EXPECT_EQ(tpe_input.size(), 2u);
}

TEST(TreeStructuredParzenEstimatorTest, InputDimensionMatching)
{
  // Test that constructor throws if sample_mean and sample_stddev sizes don't match
  std::vector<double> sample_mean{0.0, 0.0};
  std::vector<double> sample_stddev{1.0};  // Mismatched size
  
  EXPECT_THROW(
    {
      TreeStructuredParzenEstimator estimator(
        TreeStructuredParzenEstimator::Direction::MAXIMIZE, 10, sample_mean, sample_stddev);
    },
    std::exception);
}

TEST(TreeStructuredParzenEstimatorTest, ScoreOrderingAffectsResults)
{
  std::vector<double> sample_mean{0.0, 0.0};
  std::vector<double> sample_stddev{1.0, 1.0};

  // First run with increasing scores
  TreeStructuredParzenEstimator estimator1(
    TreeStructuredParzenEstimator::Direction::MAXIMIZE, 5, sample_mean, sample_stddev);
  for (int i = 0; i < 10; ++i) {
    TreeStructuredParzenEstimator::Input input{static_cast<double>(i), static_cast<double>(i)};
    estimator1.add_trial({input, static_cast<double>(i)});
  }
  const auto input1 = estimator1.get_next_input();

  // Second run with decreasing scores
  TreeStructuredParzenEstimator estimator2(
    TreeStructuredParzenEstimator::Direction::MAXIMIZE, 5, sample_mean, sample_stddev);
  for (int i = 0; i < 10; ++i) {
    TreeStructuredParzenEstimator::Input input{static_cast<double>(i), static_cast<double>(i)};
    estimator2.add_trial({input, static_cast<double>(10 - i)});
  }
  const auto input2 = estimator2.get_next_input();

  // The suggested inputs should be different because of different score ordering
  EXPECT_NE(input1[0], input2[0]);
  EXPECT_NE(input1[1], input2[1]);
}