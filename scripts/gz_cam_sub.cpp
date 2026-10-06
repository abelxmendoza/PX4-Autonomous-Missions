// C++ gz-transport Image subscriber that wakes Gazebo camera sensors.
//
// gz-sim camera sensors only render while a subscriber is registered with
// their publisher; GPU LiDAR publishes regardless, which is why LiDAR worked
// while every camera stayed dark.
//
// Registration only happens when the subscriber and the Gazebo server are on
// the same transport interface. PX4's startup runs the server with
// GZ_IP=127.0.0.1, so this process (and camera_bridge) must be started with
// GZ_IP=127.0.0.1 too -- full_stack.launch.py does that. Without it
// subscribe() still returns true and `gz topic -e` shows nothing, so the
// failure is silent. Verified live: same binary, same moment, with GZ_IP
// -> ~10 frames/s per camera; without it -> 0.
//
// It also waits for each topic's publisher to exist and re-subscribes any
// topic that stays silent, so start-up order cannot strand it.
#include <atomic>
#include <chrono>
#include <csignal>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gz/msgs/image.pb.h>
#include <gz/transport/Node.hh>

namespace {
std::atomic<bool> g_running{true};
void handle_signal(int) { g_running.store(false, std::memory_order_relaxed); }

using Clock = std::chrono::steady_clock;
constexpr auto kSilentBeforeResubscribe = std::chrono::seconds(5);
constexpr auto kPoll = std::chrono::milliseconds(500);

struct Watch {
  std::string topic;
  std::shared_ptr<std::atomic<uint64_t>> frames =
      std::make_shared<std::atomic<uint64_t>>(0);
  bool subscribed = false;
  uint64_t frames_at_subscribe = 0;
  Clock::time_point subscribed_at;
  uint64_t last_report = 0;
};

bool publisher_exists(gz::transport::Node &node, const std::string &topic) {
  std::vector<gz::transport::MessagePublisher> pubs, subs;
  return node.TopicInfo(topic, pubs, subs) && !pubs.empty();
}

bool subscribe(gz::transport::Node &node, Watch &w) {
  auto counter = w.frames;
  std::function<void(const gz::msgs::Image &)> cb =
      [counter](const gz::msgs::Image &) {
        counter->fetch_add(1, std::memory_order_relaxed);
      };
  w.subscribed = node.Subscribe(w.topic, cb);
  w.frames_at_subscribe = w.frames->load();
  w.subscribed_at = Clock::now();
  return w.subscribed;
}
}  // namespace

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cerr << "usage: gz_cam_sub <image_topic> [image_topic...]\n";
    return 2;
  }
  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);

  gz::transport::Node node;
  std::vector<Watch> watches;
  for (int i = 1; i < argc; ++i) {
    watches.push_back(Watch{argv[i]});
  }

  auto last_report_t = Clock::now();
  while (g_running.load(std::memory_order_relaxed)) {
    for (auto &w : watches) {
      if (!w.subscribed) {
        if (publisher_exists(node, w.topic)) {
          const bool ok = subscribe(node, w);
          std::cerr << "gz_cam_sub subscribed=" << std::boolalpha << ok
                    << " topic=" << w.topic << "\n";
        }
        continue;
      }
      const bool silent =
          w.frames->load() == w.frames_at_subscribe &&
          Clock::now() - w.subscribed_at > kSilentBeforeResubscribe;
      if (silent) {
        std::cerr << "gz_cam_sub resubscribing silent topic=" << w.topic << "\n";
        node.Unsubscribe(w.topic);
        subscribe(node, w);
      }
    }
    if (Clock::now() - last_report_t >= std::chrono::seconds(2)) {
      last_report_t = Clock::now();
      for (auto &w : watches) {
        const uint64_t now = w.frames->load();
        std::cerr << "gz_cam_sub frames=" << now
                  << " delta=" << (now - w.last_report) << " topic=..."
                  << w.topic.substr(w.topic.find("link/") + 5, 22) << "\n";
        w.last_report = now;
      }
    }
    std::this_thread::sleep_for(kPoll);
  }
  return 0;
}
