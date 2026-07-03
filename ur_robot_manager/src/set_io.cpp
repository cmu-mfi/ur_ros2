#include "ur_robot_manager/ur_robot_manager.hpp"

namespace ur_robot_manager
{
  void UrRobotManager::set_io_service_callback(
      const std::shared_ptr<SetIo::Request> request,
      std::shared_ptr<SetIo::Response> response) 
  {
    // Extract io parameters from the request
    int pin = request->pin;
    int state = request->state;

    // Pin needs to be in range 0-7
    if (pin < 0 || pin > 7) {
      RCLCPP_ERROR(this->get_logger(), "[Set Io Service] Invalid Pin: %d. Needs to be in between 0 and 7.", pin);
      response->success = false;
      response->message = "Io update failed: Invalid pin.";
      return;
    }
    if (state != 0 && state != 1) {
      RCLCPP_ERROR(this->get_logger(), "[Set Io Service] Invalid State: %d. Needs to be 0 or 1", state);
      response->success = false;
      response->message = "Io update failed: Invalid state.";
      return;
    }
    RCLCPP_INFO(this->get_logger(), "[Set Io Service] Attempting to set io - Pin: %d, state: %s", 
                pin, state ? "true" : "false");

    auto ur_request = std::make_shared<UrSetIo::Request>();
    ur_request->fun = ur_request->FUN_SET_DIGITAL_OUT;
    ur_request->pin = pin;
    ur_request->state = state;

    auto future_result = ur_set_io_client_->async_send_request(ur_request);
    std::future_status status = future_result.wait_for(std::chrono::seconds(3));
    if (status == std::future_status::ready) {
      auto ur_response = future_result.get();
      if (ur_response->success) {
        RCLCPP_INFO(this->get_logger(), "[Set Io Service] Successfully set io.");
        response->success = true;
        response->message = "Io successfully updated.";
      } else {
        RCLCPP_ERROR(this->get_logger(), "[Set Io Service] Driver rejected the io parameters.");
        response->success = false;
        response->message = "Driver rejected the io configuration.";
      }
    } 
    else if (status == std::future_status::timeout) {
      RCLCPP_ERROR(this->get_logger(), "[Set Io Service] Timed out waiting for io_and_status_controller service.");
      response->success = false;
      response->message = "Service call timed out. Check if io_and_status_controller is active.";
    }
  }
}  // namespace ur_robot_managr
