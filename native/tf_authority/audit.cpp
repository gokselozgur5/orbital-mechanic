// Read-only attribution of each observed TF edge to its actual DDS publisher.
// Names/GIDs identify trusted local ROS endpoints; this is not authentication.
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>

using Json = nlohmann::json;
using Message = tf2_msgs::msg::TFMessage;
struct Rule {std::string parent, owner, topic;};
const std::map<std::string, Rule> rules{
  {"odom", {"map", "amcl", "/tf"}},
  {"base_footprint", {"odom", "ros_gz_bridge", "/tf"}},
  {"base_link", {"base_footprint", "robot_state_publisher", "/tf_static"}},
  {"lidar_link", {"base_link", "robot_state_publisher", "/tf_static"}},
  {"camera_link", {"base_link", "robot_state_publisher", "/tf_static"}},
  {"camera_optical_frame", {"camera_link", "robot_state_publisher", "/tf_static"}},
  {"left_wheel", {"base_link", "robot_state_publisher", "/tf"}},
  {"right_wheel", {"base_link", "robot_state_publisher", "/tf"}}
};

template<typename Range> std::string gid_text(const Range & bytes) {
  std::ostringstream out;
  for (const auto byte : bytes) out << std::hex << std::setw(2) << std::setfill('0') << unsigned(byte);
  return out.str();
}

struct Observation {
  std::string topic, parent, child, gid;
  uint64_t count{0};
  int64_t first_stamp{0}, last_stamp{0};
};

class Auditor : public rclcpp::Node {
public:
  Auditor() : Node("orbital_tf_authority_audit") {
    dynamic_ = create_subscription<Message>("/tf", rclcpp::QoS(100).best_effort(),
      [this](Message::ConstSharedPtr msg, const rclcpp::MessageInfo & info) {receive("/tf", *msg, info);});
    fixed_ = create_subscription<Message>("/tf_static", rclcpp::QoS(100).reliable().transient_local(),
      [this](Message::ConstSharedPtr msg, const rclcpp::MessageInfo & info) {receive("/tf_static", *msg, info);});
    timer_ = create_wall_timer(std::chrono::milliseconds(250), [this]() {refresh();});
  }

  Json result() {
    refresh();
    std::map<std::string, std::set<std::string>> edge_gids;
    Json observed = Json::array();
    for (const auto & [key, item] : observations_) {
      (void)key;
      edge_gids[item.child].insert(item.gid);
      const auto endpoint = endpoints_.find(item.topic+":"+item.gid);
      std::string owner;
      if (endpoint == endpoints_.end()) {
        errors_.insert("Unresolved message publisher: "+item.child);
      } else {
        owner = endpoint->second.at("node").get<std::string>();
        const auto rule = rules.find(item.child);
        if (rule != rules.end() && (owner != rule->second.owner || endpoint->second.at("namespace") != "/"))
          errors_.insert("Wrong message publisher: "+item.child+" from "+owner);
      }
      if (item.topic == "/tf" && (item.count < 2 || item.last_stamp <= item.first_stamp))
        errors_.insert("Nonadvancing dynamic edge: "+item.child);
      observed.push_back({{"topic",item.topic},{"parent",item.parent},{"child",item.child},
        {"publisher_gid",item.gid},{"owner",owner},{"samples",item.count},
        {"first_stamp_ns",item.first_stamp},{"last_stamp_ns",item.last_stamp}});
    }
    for (const auto & [child, rule] : rules) {
      (void)rule;
      if (edge_gids[child].size() != 1) errors_.insert("Missing or multiple message owners: "+child);
    }
    // Final graph must also retain exactly the intended endpoint inventory.
    for (const std::string topic : {"/tf", "/tf_static"}) {
      std::multiset<std::string> actual;
      for (const auto & e : graph_[topic]) {
        if (e.at("namespace") != "/" || e.at("type") != "tf2_msgs/msg/TFMessage")
          errors_.insert("Unexpected TF endpoint namespace/type");
        actual.insert(e.at("node").get<std::string>());
      }
      const std::multiset<std::string> expected = topic == "/tf" ?
        std::multiset<std::string>{"amcl","ros_gz_bridge","robot_state_publisher"} :
        std::multiset<std::string>{"robot_state_publisher"};
      if (actual != expected) errors_.insert("Endpoint inventory mismatch: "+topic);
    }
    return {{"passed",errors_.empty()},{"per_message_tf_authority_proven",errors_.empty()},
      {"messages",messages_},{"observations",observed},{"graph",graph_},{"errors",errors_},
      {"incomplete_discovery_observations",incomplete_discovery_},
      {"scope","sampled DDS GID/edge ownership, parent and finite geometry; not authentication, localization accuracy or continuous enforcement"}};
  }

private:
  void refresh() {
    for (const std::string topic : {"/tf", "/tf_static"}) {
      graph_[topic] = Json::array();
      for (const auto & endpoint : get_publishers_info_by_topic(topic)) {
        const auto gid = gid_text(endpoint.endpoint_gid());
        Json value{{"node",endpoint.node_name()},{"namespace",endpoint.node_namespace()},
          {"type",endpoint.topic_type()},{"gid",gid}};
        graph_[topic].push_back(value);
        // DDS endpoint discovery precedes ROS node-name discovery. Retain the
        // raw final graph, but do not mistake unresolved metadata for identity.
        // Messages remain unresolved/rejected unless a complete mapping arrives.
        if (endpoint.node_name() == "_NODE_NAME_UNKNOWN_" ||
            endpoint.node_namespace() == "_NODE_NAMESPACE_UNKNOWN_" ||
            endpoint.node_name().empty() || endpoint.node_namespace().empty()) {
          ++incomplete_discovery_;
          continue;
        }
        const auto key = topic+":"+gid;
        if (endpoints_.count(key) && endpoints_.at(key) != value)
          errors_.insert("Publisher GID changed identity during observation: "+endpoints_.at(key).dump()+" -> "+value.dump());
        if (endpoints_.size() >= 128 && !endpoints_.count(key)) {
          errors_.insert("Endpoint observation bound exceeded");
          continue;
        }
        endpoints_[key] = value;
      }
    }
  }

  void receive(const std::string & topic, const Message & message, const rclcpp::MessageInfo & info) {
    ++messages_;
    const auto & meta = info.get_rmw_message_info();
    const auto gid = gid_text(meta.publisher_gid.data);
    if (std::all_of(std::begin(meta.publisher_gid.data), std::end(meta.publisher_gid.data), [](uint8_t b) {return b==0;}))
      errors_.insert("Message publisher GID is unavailable");
    if (message.transforms.size() > 64) {
      errors_.insert("Transform count bound exceeded");
      return;
    }
    for (const auto & transform : message.transforms) {
      const auto & parent = transform.header.frame_id;
      const auto & child = transform.child_frame_id;
      if (parent.size() > 128 || child.size() > 128) {
        errors_.insert("Frame identifier bound exceeded");
        continue;
      }
      const auto rule = rules.find(child);
      if (rule == rules.end() || rule->second.parent != parent || rule->second.topic != topic)
        errors_.insert("Unexpected edge parent/topic: "+parent+" -> "+child);
      const auto & p = transform.transform.translation;
      const auto & q = transform.transform.rotation;
      const double norm = q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || !std::isfinite(norm) || std::abs(norm-1.) > .001)
        errors_.insert("Invalid transform geometry: "+child);
      const int64_t stamp = int64_t(transform.header.stamp.sec)*1000000000LL+transform.header.stamp.nanosec;
      if (stamp < 0 || transform.header.stamp.nanosec >= 1000000000U)
        errors_.insert("Invalid transform stamp: "+child);
      const auto key = std::make_tuple(topic,parent,child,gid);
      if (observations_.size() >= 128 && !observations_.count(key)) {
        errors_.insert("Transform observation bound exceeded");
        continue;
      }
      auto & item = observations_[key];
      if (item.count == 0) {
        item = {topic,parent,child,gid,0,stamp,stamp};
      } else if (topic == "/tf" && stamp < item.last_stamp) {
        errors_.insert("Regressing transform stamp: "+child);
      }
      ++item.count;
      item.last_stamp = stamp;
    }
  }

  rclcpp::Subscription<Message>::SharedPtr dynamic_, fixed_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::map<std::tuple<std::string,std::string,std::string,std::string>,Observation> observations_;
  std::map<std::string,Json> endpoints_;
  Json graph_ = Json::object();
  std::set<std::string> errors_;
  uint64_t messages_{0};
  uint64_t incomplete_discovery_{0};
};

int main(int argc, char ** argv) {
  try {
    const std::string domain = std::getenv("ROS_DOMAIN_ID") ? std::getenv("ROS_DOMAIN_ID") : "";
    const std::string discovery = std::getenv("ROS_AUTOMATIC_DISCOVERY_RANGE") ? std::getenv("ROS_AUTOMATIC_DISCOVERY_RANGE") : "";
    if ((domain != "42" && domain != "84") || discovery != "LOCALHOST")
      throw std::runtime_error("Requires Orbital localhost domain42 or isolated domain84");
    const auto args = rclcpp::remove_ros_arguments(argc, argv);
    double seconds = 6.;
    if (args.size() == 3 && args[1] == "--seconds") seconds = std::stod(args[2]);
    else if (args.size() != 1) throw std::runtime_error("Usage: orbital_tf_authority [--seconds 2..20]");
    if (!std::isfinite(seconds) || seconds < 2. || seconds > 20.) throw std::runtime_error("Invalid observation window");
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Auditor>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    const auto until = std::chrono::steady_clock::now()+std::chrono::duration<double>(seconds);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < until) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (!rclcpp::ok()) throw std::runtime_error("Observation interrupted");
    executor.spin_some();
    const auto report = node->result();
    std::cout << report.dump(2) << std::endl;
    executor.remove_node(node); node.reset(); rclcpp::shutdown();
    return report.at("passed").get<bool>() ? 0 : 1;
  } catch (const std::exception & error) {
    std::cerr << error.what() << std::endl;
    if (rclcpp::ok()) rclcpp::shutdown();
    return 2;
  }
}
