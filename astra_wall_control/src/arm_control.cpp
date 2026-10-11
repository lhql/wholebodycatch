// Gazebo Classic / ROS 2 model interface, not an MPC implementation.
// Only SetForce is used: actuator reactions are propagated by the physics solver.
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace gazebo {
class MyArmControl : public ModelPlugin {
  struct Axis {
    physics::JointPtr joint;
    std::string name;
    double kp, kd, lower, upper, effort, speed;
    double goal = 0, reference = 0, feedforward = 0, applied = 0;
  };
  physics::ModelPtr model_;
  event::ConnectionPtr update_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr positionSub_, effortSub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr statePub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr modePub_;
  std::vector<Axis> axes_;
  bool effortMode_ = false, commandReceived_ = false;
  double now_ = 0, lastUpdate_ = 0, lastCommand_ = 0, lastState_ = -1;
  double timeout_ = .25, statePeriod_ = .005;
  static double clamp(double x, double a, double b) {
    return std::max(a, std::min(b, x));
  }
  void mode(const std::string &value) {
    if (!node_) return;
    std_msgs::msg::String msg; msg.data = value; modePub_->publish(msg);
  }
  // All callbacks execute from the physics update thread; no racing SetForce,
  // partially updated command vectors, or asynchronous joint reads.
  void command(const sensor_msgs::msg::JointState::SharedPtr &msg, bool effort) {
    const auto &values = effort ? msg->effort : msg->position;
    const size_t n = axes_.size();
    if (msg->name.size() != n || values.size() != n ||
        !msg->velocity.empty() || (effort && !msg->position.empty()) ||
        (!effort && !msg->effort.empty() && msg->effort.size() != n)) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "my arm: command must contain all 7 names and ONLY position or effort");
      return;
    }
    if (rclcpp::Time(msg->header.stamp).nanoseconds() != 0) {
      const double age = now_ - rclcpp::Time(msg->header.stamp).seconds();
      if (age > timeout_ || age < -.02) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "my arm: stale/future command rejected (use simulation time)");
        return;
      }
    }
    std::vector<double> ordered(n);
    std::vector<bool> seen(n, false);
    for (size_t k=0; k<n; ++k) {
      auto it = std::find_if(axes_.begin(), axes_.end(), [&](const Axis &a) { return a.name == msg->name[k]; });
      if (it == axes_.end() || !std::isfinite(values[k]) ||
          (!effort && !msg->effort.empty() && !std::isfinite(msg->effort[k]))) return;
      const size_t i = std::distance(axes_.begin(), it);
      if (seen[i]) return;
      seen[i] = true;
      if (!effort && (values[k] < it->lower || values[k] > it->upper)) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "my arm: out-of-range position rejected");
        return;
      }
      ordered[i] = values[k];
    }
    for (size_t i=0; i<n; ++i) {
      if (effort) axes_[i].feedforward = clamp(ordered[i], -axes_[i].effort, axes_[i].effort);
      else {
        if (effortMode_) axes_[i].reference = clamp(axes_[i].joint->Position(0), axes_[i].lower, axes_[i].upper);
        axes_[i].goal = ordered[i];
        auto k = std::find(msg->name.begin(), msg->name.end(), axes_[i].name) - msg->name.begin();
        axes_[i].feedforward = msg->effort.empty() ? 0. : clamp(msg->effort[k], -axes_[i].effort, axes_[i].effort);
      }
    }
    effortMode_ = effort; commandReceived_ = true; lastCommand_ = now_;
    mode(effort ? "effort" : "position");
  }
  void hold() {
    for (auto &a : axes_) {
      a.goal = a.reference = clamp(a.joint->Position(0), a.lower, a.upper);
      a.feedforward = 0;
    }
    effortMode_ = false; commandReceived_ = false;
    mode("hold");
  }
  void update(const common::UpdateInfo &info) {
    now_ = info.simTime.Double();
    if (now_ < lastUpdate_) { hold(); lastState_ = -1; }
    const double dt = clamp(now_ - lastUpdate_, 0, .02);
    lastUpdate_ = now_;
    executor_.spin_some(std::chrono::nanoseconds(0));
    if (commandReceived_ && now_ - lastCommand_ > timeout_) hold();
    for (auto &a : axes_) {
      const double q = a.joint->Position(0), v = a.joint->GetVelocity(0);
      double u = 0;
      if (std::isfinite(q) && std::isfinite(v)) {
        if (effortMode_) u = a.feedforward;  // No hidden position PID in MPC mode.
        else {
          a.reference += clamp(a.goal-a.reference, -a.speed*dt, a.speed*dt);
          u = a.kp*(a.reference-q) - a.kd*v + a.feedforward;
        }
        // Do not drive farther through limits, nor accelerate an overspeed axis.
        if ((q <= a.lower && u < 0) || (q >= a.upper && u > 0) ||
            (v >= a.speed && u > 0) || (v <= -a.speed && u < 0)) u = 0;
      }
      a.applied = clamp(u, -a.effort, a.effort);
      a.joint->SetForce(0, a.applied);
    }
    if (node_ && now_ - lastState_ >= statePeriod_) {
      sensor_msgs::msg::JointState msg;
      msg.header.stamp = rclcpp::Time(static_cast<int64_t>(now_*1e9));
      for (const auto &a : axes_) {
        msg.name.push_back(a.name); msg.position.push_back(a.joint->Position(0));
        msg.velocity.push_back(a.joint->GetVelocity(0)); msg.effort.push_back(a.applied);
      }
      statePub_->publish(msg); lastState_ = now_;
    }
  }
 public:
  ~MyArmControl() override {
    update_.reset();
    if (node_) executor_.remove_node(node_);
  }
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
    model_ = model;
    timeout_ = sdf->Get<double>("commandTimeout");
    statePeriod_ = 1.0/sdf->Get<double>("stateRate");
    for (auto el = sdf->GetElement("joint"); el; el = el->GetNextElement("joint")) {
      Axis a;
      a.name = el->Get<std::string>("name"); a.joint = model_->GetJoint(a.name);
      if (!a.joint) { gzerr << "my arm: missing joint " << a.name << "\n"; return; }
      a.kp = el->Get<double>("kp"); a.kd = el->Get<double>("kd");
      a.lower = a.joint->LowerLimit(0); a.upper = a.joint->UpperLimit(0);
      a.effort = a.joint->GetEffortLimit(0); a.speed = a.joint->GetVelocityLimit(0);
      axes_.push_back(a);
    }
    if (axes_.size() != 7) { gzerr << "my arm: expected seven actuators\n"; return; }
    if (!rclcpp::ok()) { gzerr << "Load gazebo_ros_init before arm plugin\n"; return; }
    node_ = std::make_shared<rclcpp::Node>("arm_actuation", "/my/arm");
    executor_.add_node(node_);
    positionSub_ = node_->create_subscription<sensor_msgs::msg::JointState>("position_command", 1,
      [this](sensor_msgs::msg::JointState::SharedPtr m) { command(m, false); });
    effortSub_ = node_->create_subscription<sensor_msgs::msg::JointState>("effort_command", 1,
      [this](sensor_msgs::msg::JointState::SharedPtr m) { command(m, true); });
    statePub_ = node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 5);
    modePub_ = node_->create_publisher<std_msgs::msg::String>("control_mode", rclcpp::QoS(1).transient_local());
    mode("position");
    lastUpdate_ = model_->GetWorld()->SimTime().Double();
    update_ = event::Events::ConnectWorldUpdateBegin(std::bind(&MyArmControl::update, this, std::placeholders::_1));
  }
  void Reset() override {
    effortMode_ = commandReceived_ = false;
    for (auto &a : axes_) a.goal = a.reference = a.feedforward = 0;
    lastUpdate_ = model_->GetWorld()->SimTime().Double(); lastState_ = -1;
    mode("position");
  }
};
GZ_REGISTER_MODEL_PLUGIN(MyArmControl)
}  // namespace gazebo
