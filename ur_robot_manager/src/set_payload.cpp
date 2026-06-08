#include "ur_robot_manager/ur_robot_manager.hpp"

namespace ur_robot_manager
{
  void UrRobotManager::set_payload_service_callback(
      const std::shared_ptr<SetPayload::Request> request,
      std::shared_ptr<SetPayload::Response> response) 
  {
    // Extract payload parameters from the request
    double mass = request->mass;
    double cog_x = request->cog.x;
    double cog_y = request->cog.y;
    double cog_z = request->cog.z;
    // Basic validation: Mass should not be negative
    if (mass < 0.0) {
      RCLCPP_ERROR(this->get_logger(), "[Set Payload Service] Invalid mass: %.2f kg. Mass cannot be negative.", mass);
      response->success = false;
      response->message = "Payload update failed: Mass cannot be negative.";
      return;
    }

    RCLCPP_INFO(this->get_logger(), "[Set Payload Service] Attempting to set payload - Mass: %.2f kg, COG: [%.3f, %.3f, %.3f]", 
                mass, cog_x, cog_y, cog_z);

    auto ur_request = std::make_shared<UrSetPayload::Request>();
    ur_request->mass = mass;
    ur_request->set__center_of_gravity(request->cog);

    auto future_result = ur_set_payload_client_->async_send_request(ur_request);
    std::future_status status = future_result.wait_for(std::chrono::seconds(3));
    if (status == std::future_status::ready) {
      auto ur_response = future_result.get();
      if (ur_response->success) {
        RCLCPP_INFO(this->get_logger(), "[Set Payload Service] Successfully set payload.");
        response->success = true;
        response->message = "Payload successfully updated.";
      } else {
        RCLCPP_ERROR(this->get_logger(), "[Set Payload Service] Driver rejected the payload parameters.");
        response->success = false;
        response->message = "Driver rejected the payload configuration.";
      }
    } 
    else if (status == std::future_status::timeout) {
      RCLCPP_ERROR(this->get_logger(), "[Set Payload Service] Timed out waiting for io_and_status_controller service.");
      response->success = false;
      response->message = "Service call timed out. Check if io_and_status_controller is active.";
    }
  }
}  // namespace ur_robot_managr
