// Dummy C++ gz-transport Image subscriber.
//
// gz-sim camera sensors only render when Publisher::HasConnections() is
// true. The Python gz.transport13 bindings advertise a subscribe that
// returns True but never increment that count, so cameras stay dark even
// though camera_bridge thinks it is attached. LiDAR still works because
// PX4's C++ gz_bridge is a real subscriber.
//
// This process is that missing C++ subscriber. Once it is connected,
// Gazebo starts emitting frames and camera_bridge (Python) receives them.
#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <gz/msgs/image.pb.h>
#include <gz/transport/Node.hh>

namespace {
std::atomic<bool> g_running{true};
std::atomic<uint64_t> g_frames{0};

void on_image(const gz::msgs::Image &)
{
  g_frames.fetch_add(1, std::memory_order_relaxed);
}

void handle_signal(int)
{
  g_running.store(false, std::memory_order_relaxed);
}
}  // namespace

int main(int argc, char **argv)
{
  if (argc < 2) {
    std::cerr << "usage: gz_cam_sub <image_topic> [image_topic...]\n";
    return 2;
  }

  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);

  gz::transport::Node node;
  std::vector<std::string> topics;
  for (int i = 1; i < argc; ++i) {
    const std::string topic = argv[i];
    const bool ok = node.Subscribe(topic, on_image);
    std::cerr << "gz_cam_sub subscribed=" << std::boolalpha << ok
              << " topic=" << topic << "\n";
    if (!ok) {
      return 1;
    }
    topics.push_back(topic);
  }

  uint64_t last = 0;
  while (g_running.load(std::memory_order_relaxed)) {
    std::this_thread::sleep_for(std::chrono::seconds(2));
    const uint64_t now = g_frames.load(std::memory_order_relaxed);
    std::cerr << "gz_cam_sub frames=" << now << " delta=" << (now - last)
              << "\n";
    last = now;
  }
  return 0;
}
