#ifndef UDACITY__UWS_HPP
#define UDACITY__UWS_HPP

#include <math.h>

#include <string>
#include <vector>

namespace udacity
{

namespace uws
{

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in std::string format will be returned,
//   else the empty std::string "" will be returned.
std::string has_data(std::string s)
{
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != std::string::npos) {
    return "";
  } else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

}  // namespace uws

}  // namespace udacity

#endif  // UDACITY__UWS_HPP
