#ifndef VALUE_CONVERSION_TABLE_H
#define VALUE_CONVERSION_TABLE_H

#include <map>
#include <vector>
#include <memory>
#include <limits>
#include "port.h"

namespace AVP
{
    
namespace mapping
{


class ValueConversionTables {
 public:
  const std::vector<float>* GetConversionTable(float unknown_result,
                                               float lower_bound,
                                               float upper_bound);

 private:
  std::map<const std::tuple<float /* unknown_result */, float /* lower_bound */,
                            float /* upper_bound */>,
           std::shared_ptr<const std::vector<float>>>
      bounds_to_lookup_table_;
};

} // namespace mapping


} // namespace AVP


#endif