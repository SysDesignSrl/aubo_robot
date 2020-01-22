#ifndef AUBO_ERROR_CODES_H
#define AUBO_ERROR_CODES_H
// STL
#include <string>
#include <map>


namespace aubo {

std::map<int, std::string> error_codes {
  {     0, "Success" },
  { 10001, "General Fail" },
  { 10002, "Parameter error" },
  { 10003, "Socket connection fail" },
  { 10004, "Socket disconnect" },
  { 10005, "Fail to create request" },
  { 10006, "Request related internal variable error" },
  { 10007, "Request timeout" },
  { 10008, "Fail to send request information" },
  { 10009, "Response information is null" },
  { 10010, "Fail to resolve response" },
  { 10011, "Forward kinematics failed" },
  { 10012, "Inverse kinematics failed" },
  { 10013, "Tool calibration parameters error" },
  { 10014, "Tool calibration parameters error" },
  { 10015, "Fail to calibrate parameter system" },
  { 10016, "Fail to convert base coordinate system to user coordinate system" },
  { 10017, "Fail to convert user coordinate system to base coordinate system" },
  { 10018, "Motion-related internal variable error" },
  { 10019, "Motion request fail" },
  { 10020, "Fail to create motion request" },
  { 10021, "Motion is interrupted by event" },
  { 10022, "Motion-related waypoint container size is illegal" },
  { 10023, "Server response return error" },
  { 10024, "The real robot is not existing because some interfaces can be call only if the real robot is existent." },
};

} // namespace
#endif
