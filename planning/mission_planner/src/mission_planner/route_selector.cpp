// Copyright 2024 The Autoware Contributors
// SPDX-License-Identifier: Apache-2.0

#include "route_selector.hpp"

#include "service_utils.hpp"

#include <array>
#include <random>

namespace mission_planner::uuid
{

std::array<uint8_t, 16> generate_random_id()
{
  static std::independent_bits_engine<std::mt19937, 8, uint8_t> engine(std::random_device{}());
  std::array<uint8_t, 16> id;
  std::generate(id.begin(), id.end(), std::ref(engine));
  return id;
}

UUID generate_if_empty(const UUID & uuid)
{
  constexpr std::array<uint8_t, 16> zero_uuid = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  UUID result;
  result.uuid = (uuid.uuid == zero_uuid) ? generate_random_id() : uuid.uuid;
  return result;
}

}  // namespace mission_planner::uuid

namespace mission_planner
{

RouteSelector::RouteSelector(const rclcpp::NodeOptions & options) : Node("route_selector", options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  const auto service_qos = rmw_qos_profile_services_default;
  const auto durable_qos = rclcpp::QoS(1).transient_local();

  // Init main route interface.
  main_.srv_clear_route = create_service<ClearRoute>(
    "~/main/clear_route",
    service_utils::handle_exception(&RouteSelector::on_clear_route_main, this));
  main_.srv_set_waypoint_route = create_service<SetWaypointRoute>(
    "~/main/set_waypoint_route",
    service_utils::handle_exception(&RouteSelector::on_set_waypoint_route_main, this));
  main_.srv_set_lanelet_route = create_service<SetLaneletRoute>(
    "~/main/set_lanelet_route",
    service_utils::handle_exception(&RouteSelector::on_set_lanelet_route_main, this));
  main_.pub_state_ = create_publisher<RouteState>("~/main/state", durable_qos);
  main_.pub_route_ = create_publisher<LaneletRoute>("~/main/route", durable_qos);

  // Init mrm route interface.
  mrm_.srv_clear_route = create_service<ClearRoute>(
    "~/mrm/clear_route", service_utils::handle_exception(&RouteSelector::on_clear_route_mrm, this));
  mrm_.srv_set_waypoint_route = create_service<SetWaypointRoute>(
    "~/mrm/set_waypoint_route",
    service_utils::handle_exception(&RouteSelector::on_set_waypoint_route_mrm, this));
  mrm_.srv_set_lanelet_route = create_service<SetLaneletRoute>(
    "~/mrm/set_lanelet_route",
    service_utils::handle_exception(&RouteSelector::on_set_lanelet_route_mrm, this));
  mrm_.pub_state_ = create_publisher<RouteState>("~/mrm/state", durable_qos);
  mrm_.pub_route_ = create_publisher<LaneletRoute>("~/mrm/route", durable_qos);

  // Init mission planner interface.
  group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cli_clear_route_ = create_client<ClearRoute>("~/planner/clear_route", service_qos, group_);
  cli_set_lanelet_route_ =
    create_client<SetLaneletRoute>("~/planner/set_lanelet_route", service_qos, group_);
  cli_set_waypoint_route_ =
    create_client<SetWaypointRoute>("~/planner/set_waypoint_route", service_qos, group_);
  sub_state_ = create_subscription<RouteState>(
    "~/planner/state", durable_qos, std::bind(&RouteSelector::on_state, this, _1));
  sub_route_ = create_subscription<LaneletRoute>(
    "~/planner/route", durable_qos, std::bind(&RouteSelector::on_route, this, _1));

  // Set initial state.
  const auto stamp = now();
  main_.change_state(stamp, RouteState::INITIALIZING);
  mrm_.change_state(stamp, RouteState::INITIALIZING);
  initialized_ = false;
  mrm_operating_ = false;
  main_request_ = std::monostate{};
}

void RouteSelector::on_state(const RouteState::ConstSharedPtr msg)
{
  if (msg->state == RouteState::UNSET && !initialized_) {
    const auto stamp = now();
    main_.change_state(stamp, RouteState::UNSET);
    mrm_.change_state(stamp, RouteState::UNSET);
    initialized_ = true;
  }

  (mrm_operating_ ? mrm_ : main_).update_state(*msg);
}

void RouteSelector::on_route(const LaneletRoute::ConstSharedPtr msg)
{
  (mrm_operating_ ? mrm_ : main_).update_route(*msg);
}

void RouteSelector::on_clear_route_main(
  ClearRoute::Request::SharedPtr req, ClearRoute::Response::SharedPtr res)
{
  // Save the request to resume from MRM.
  main_request_ = std::monostate{};

  // During MRM, only change the state.
  if (mrm_operating_) {
    main_.change_state(now(), RouteState::UNSET);
    res->status.success = true;
    return;
  }

  // Forward the request if not in MRM.
  res->status = service_utils::sync_call(cli_clear_route_, req);
}

void RouteSelector::on_set_waypoint_route_main(
  SetWaypointRoute::Request::SharedPtr req, SetWaypointRoute::Response::SharedPtr res)
{
  // Save the request to resume from MRM.
  req->uuid = uuid::generate_if_empty(req->uuid);
  main_request_ = req;

  // During MRM, only change the state.
  if (mrm_operating_) {
    main_.change_state(now(), RouteState::SET);
    res->status.success = true;
    return;
  }

  // Forward the request if not in MRM.
  res->status = service_utils::sync_call(cli_set_waypoint_route_, req);
}

void RouteSelector::on_set_lanelet_route_main(
  SetLaneletRoute::Request::SharedPtr req, SetLaneletRoute::Response::SharedPtr res)
{
  // Save the request to resume from MRM.
  req->uuid = uuid::generate_if_empty(req->uuid);
  main_request_ = req;

  // During MRM, only change the state.
  if (mrm_operating_) {
    main_.change_state(now(), RouteState::SET);
    res->status.success = true;
    return;
  }

  // Forward the request if not in MRM.
  res->status = service_utils::sync_call(cli_set_lanelet_route_, req);
}

void RouteSelector::on_clear_route_mrm(
  ClearRoute::Request::SharedPtr req, ClearRoute::Response::SharedPtr res)
{
  mrm_operating_ = false;
  mrm_.change_state(now(), RouteState::UNSET);

  // Resume main route using the saved request.
  if (std::holds_alternative<std::monostate>(main_request_)) {
    res->status = service_utils::sync_call(cli_clear_route_, req);
    return;
  }
  if (auto request = std::get_if<RoutePointRequest>(&main_request_)) {
    // NOTE: Clear the waypoint to avoid returning. Remove this once resuming is supported.
    (**request).waypoints.clear();
    res->status = service_utils::sync_call(cli_set_waypoint_route_, *request);
    return;
  }
  if (auto request = std::get_if<RouteRequest>(&main_request_)) {
    res->status = service_utils::sync_call(cli_set_lanelet_route_, *request);
    return;
  }
  RCLCPP_ERROR_STREAM(get_logger(), "unknown main route request");
}

void RouteSelector::on_set_waypoint_route_mrm(
  SetWaypointRoute::Request::SharedPtr req, SetWaypointRoute::Response::SharedPtr res)
{
  mrm_operating_ = true;
  res->status = service_utils::sync_call(cli_set_waypoint_route_, req);
}

void RouteSelector::on_set_lanelet_route_mrm(
  SetLaneletRoute::Request::SharedPtr req, SetLaneletRoute::Response::SharedPtr res)
{
  mrm_operating_ = true;
  res->status = service_utils::sync_call(cli_set_lanelet_route_, req);
}

void RouteInterface::change_state(const rclcpp::Time & stamp, RouteState::_state_type state)
{
  state_.stamp = stamp;
  state_.state = state;
  pub_state_->publish(state_);
}

void RouteInterface::update_state(const RouteState & state)
{
  state_ = state;
  pub_state_->publish(state_);
}

void RouteInterface::update_route(const LaneletRoute & route)
{
  pub_route_->publish(route);
}

}  // namespace mission_planner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mission_planner::RouteSelector)
