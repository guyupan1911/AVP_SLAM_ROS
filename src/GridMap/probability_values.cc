#include "probability_values.h"

namespace AVP
{
    
namespace mapping
{
    
namespace {

constexpr int kValueCount = 32768;

// 0 is unknown, [1, 32767] maps to [lower_bound, upper_bound].
// 将[0, 1~32767] 映射成 [0.9, 0.1~0.9]
float SlowValueToBoundedFloat(const uint16 value, const uint16 unknown_value,
                              const float unknown_result,
                              const float lower_bound,
                              const float upper_bound) {

  if (value == unknown_value) return unknown_result;
  const float kScale = (upper_bound - lower_bound) / (kValueCount - 2.f);
  return value * kScale + (lower_bound - kScale);
}

// 新建转换表
std::unique_ptr<std::vector<float>> PrecomputeValueToBoundedFloat(
    const uint16 unknown_value, const float unknown_result,
    const float lower_bound, const float upper_bound) {
  auto result = std::unique_ptr<std::vector<float>>(new std::vector<float>);
  // Repeat two times, so that both values with and without the update marker
  // can be converted to a probability.
  // 重复2遍
  constexpr int kRepetitionCount = 2;
  result->reserve(kRepetitionCount * kValueCount);
  for (int repeat = 0; repeat != kRepetitionCount; ++repeat) {
    for (int value = 0; value != kValueCount; ++value) {
      result->push_back(SlowValueToBoundedFloat(
          value, unknown_value, unknown_result, lower_bound, upper_bound));
    }
  }
  return result;
}

// 返回ValueToProbability转换表的指针
std::unique_ptr<std::vector<float>> PrecomputeValueToProbability() {
  return PrecomputeValueToBoundedFloat(kUnknownProbabilityValue,          // 0
                                       kMinProbability, kMinProbability,  // 0.1, 0.1
                                       kMaxProbability);                  // 0.9
}

// 返回ValueToCorrespondenceCost转换表的指针
std::unique_ptr<std::vector<float>> PrecomputeValueToCorrespondenceCost() {
  return PrecomputeValueToBoundedFloat(
      kUnknownCorrespondenceValue, kMaxCorrespondenceCost,  // 0,   0.9
      kMinCorrespondenceCost, kMaxCorrespondenceCost);      // 0.1, 0.9
}

}  // namespace

// ValueToProbability转换表
const std::vector<float>* const kValueToProbability =
    PrecomputeValueToProbability().release();

// [0, 1~32767] 映射成 [0.9, 0.1~0.9]转换表
const std::vector<float>* const kValueToCorrespondenceCost =
    PrecomputeValueToCorrespondenceCost().release();

// 将栅格是未知状态与odds状态下, 将更新时的所有可能结果预先计算出来
std::vector<uint16> ComputeLookupTableToApplyOdds(const float odds) {
  std::vector<uint16> result;
  result.reserve(kValueCount);
  // 当前cell是unknown情况下直接把 odd转成概率值付给cell
  result.push_back(ProbabilityToValue(ProbabilityFromOdds(odds)) +
                   kUpdateMarker); // 加上kUpdateMarker作为一个标志, 代表这个栅格已经被更新了
  // 计算更新时 从1到32768的所有可能的 更新后的结果 
  for (int cell = 1; cell != kValueCount; ++cell) {
    result.push_back(ProbabilityToValue(ProbabilityFromOdds(
                         odds * Odds((*kValueToProbability)[cell]))) +
                     kUpdateMarker);
  }
  return result;
}

// 将栅格是未知状态与odds状态下, 将更新时的所有可能结果预先计算出来
std::vector<uint16> ComputeLookupTableToApplyCorrespondenceCostOdds(
    float odds) {
  std::vector<uint16> result;
  result.reserve(kValueCount); // 32768

  // 当前cell是unknown情况下直接把odds（更新系数）转成value存进来
  result.push_back(CorrespondenceCostToValue(ProbabilityToCorrespondenceCost( // 转换为miss的概率 转换为0.1~0.9对应的整数值 
                       ProbabilityFromOdds(odds))) + // odds转换为hit的概率
                   kUpdateMarker); // 加上kUpdateMarker作为一个标志, 代表这个栅格已经被更新了
  // 计算更新时 从1到32768的所有可能的 更新后的结果 
  for (int cell = 1; cell != kValueCount; ++cell) {
    result.push_back(
        CorrespondenceCostToValue( // 转换为对应的int
            ProbabilityToCorrespondenceCost(ProbabilityFromOdds( // odds转换为hit 转换为free
                odds * Odds(CorrespondenceCostToProbability( // 转换成hit的概率， 计算odds 更新
                           (*kValueToCorrespondenceCost)[cell]))))) + //free的概率
        kUpdateMarker);
  }
  return result;
}

} // namespace mapping
} // namespace AVP
