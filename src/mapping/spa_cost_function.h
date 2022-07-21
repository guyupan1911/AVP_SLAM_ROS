#ifndef SPA_COST_FUNCTION_H
#define SPA_COST_FUNCTION

#include "constraint.h"
#include <ceres/ceres.h>

namespace AVP
{
namespace mapping
{
    
template <typename T>
static std::array<T, 3> ComputeUnscaledError(
    const Eigen::Vector3d& relative_pose, const T* const start,
    const T* const end);

template <typename T>
std::array<T, 3> ScaleError(const std::array<T, 3>& error,
                            double translation_weight, double rotation_weight);    

class SpaCostFunction2D {
 public:
  explicit SpaCostFunction2D(
      const Constraint& observed_relative_pose)
      : observed_relative_pose_(observed_relative_pose) {}

  template <typename T>
  bool operator()(const T* const start_pose, const T* const end_pose,
                  T* e) const {
    const std::array<T, 3> error =
        ScaleError(ComputeUnscaledError(observed_relative_pose_.relative_pose , start_pose, end_pose),
                   observed_relative_pose_.translation_weight,
                   observed_relative_pose_.rotation_weight);
    std::copy(std::begin(error), std::end(error), e);
    return true;
  }

 private:
  
  const Constraint observed_relative_pose_;
};

ceres::CostFunction* CreateAutoDiffSpaCostFunction(
    const Constraint& observed_relative_pose) ;

} // namespace mapping
} // namespace AVP


#endif