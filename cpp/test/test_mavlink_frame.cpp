// Golden-vector conformance test for the C++ MAVLink codec. No test framework:
// plain checks, a non-zero exit code on any failure (so ctest/JUnit can see it).
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "px4v/mavlink_frame.hpp"

using namespace px4v;

static int failures = 0;
static int checks = 0;

#define CHECK(cond, msg)                                              \
  do {                                                                \
    ++checks;                                                         \
    if (!(cond)) {                                                    \
      ++failures;                                                     \
      std::cerr << "FAIL: " << msg << "  [" #cond "]\n";              \
    }                                                                 \
  } while (0)

static std::vector<uint8_t> from_hex(const std::string& s) {
  std::vector<uint8_t> out;
  for (size_t i = 0; i + 1 < s.size(); i += 2) out.push_back(static_cast<uint8_t>(std::stoi(s.substr(i, 2), nullptr, 16)));
  return out;
}

static std::vector<std::string> split(const std::string& s, char sep) {
  std::vector<std::string> parts;
  std::string cur;
  std::istringstream in(s);
  while (std::getline(in, cur, sep)) parts.push_back(cur);
  if (!s.empty() && s.back() == sep) parts.emplace_back();
  return parts;
}

static std::string describe(const ParseEvent& e) {
  switch (e.kind) {
    case ParseKind::Frame: return "frame:" + std::to_string(e.frame.seq);
    case ParseKind::BadCrc: return "bad_crc";
    case ParseKind::UnknownMsg: return "unknown_msg";
    case ParseKind::Unsupported: return "unsupported";
  }
  return "?";
}

static void check_frame_line(const std::vector<std::string>& f) {
  const std::string& name = f[1];
  const uint32_t msgid = static_cast<uint32_t>(std::stoul(f[2]));
  const uint8_t seq = static_cast<uint8_t>(std::stoul(f[3]));
  const uint8_t sysid = static_cast<uint8_t>(std::stoul(f[4]));
  const uint8_t compid = static_cast<uint8_t>(std::stoul(f[5]));
  const auto payload = from_hex(f[6]);
  const auto wire = from_hex(f[7]);

  CHECK(encode_frame(msgid, payload, seq, sysid, compid) == wire, name << ": encoded bytes differ from golden");

  FrameParser parser;
  const auto events = parser.feed(wire.data(), wire.size());
  CHECK(events.size() == 1 && events[0].kind == ParseKind::Frame, name << ": golden frame did not parse");
  if (events.size() == 1 && events[0].kind == ParseKind::Frame) {
    const Frame& fr = events[0].frame;
    CHECK(fr.msgid == msgid && fr.seq == seq && fr.sysid == sysid && fr.compid == compid, name << ": header mismatch");
    std::vector<uint8_t> expected = payload;
    while (expected.size() > 1 && expected.back() == 0) expected.pop_back();
    CHECK(fr.payload == expected, name << ": payload mismatch");
  }

  // Byte-at-a-time delivery must give the same single frame.
  FrameParser slow;
  size_t frames = 0;
  for (uint8_t b : wire)
    for (const auto& e : slow.feed(&b, 1)) frames += e.kind == ParseKind::Frame;
  CHECK(frames == 1, name << ": byte-by-byte delivery");
}

static void check_stream_line(const std::vector<std::string>& f) {
  const std::string& name = f[1];
  const auto data = from_hex(f[2]);
  const auto expected = split(f.size() > 3 ? f[3] : "", ',');
  FrameParser parser;
  std::vector<std::string> got;
  for (const auto& e : parser.feed(data.data(), data.size())) got.push_back(describe(e));
  std::string g, x;
  for (const auto& s : got) g += s + ",";
  for (const auto& s : expected) x += s + ",";
  CHECK(g == x, name << ": events [" << g << "] expected [" << x << "]");
}

static void check_basics() {
  const uint8_t check[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  CHECK(crc_x25(check, sizeof check) == 0x6F91, "crc_x25 check value");
  CHECK(encode_frame(0xFFFF, {}, 0).empty(), "unknown msgid is not encodable");
  CHECK(encode_frame(0, std::vector<uint8_t>(256, 1), 0).empty(), "oversize payload is not encodable");

  // Random-looking noise must never fabricate a frame (fixed LCG, deterministic).
  uint32_t state = 12345;
  FrameParser noise;
  size_t fabricated = 0;
  for (int i = 0; i < 200; ++i) {
    std::vector<uint8_t> chunk(1 + (state % 63));
    for (auto& b : chunk) {
      state = state * 1664525u + 1013904223u;
      b = static_cast<uint8_t>(state >> 24);
    }
    for (const auto& e : noise.feed(chunk.data(), chunk.size())) fabricated += e.kind == ParseKind::Frame;
  }
  CHECK(fabricated == 0, "noise produced " << fabricated << " frame(s)");
}

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "usage: test_mavlink_frame golden_frames.txt\n";
    return 2;
  }
  std::ifstream in(argv[1]);
  if (!in) {
    std::cerr << "cannot open " << argv[1] << "\n";
    return 2;
  }
  check_basics();
  std::string line;
  size_t vectors = 0;
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    const auto f = split(line, '|');
    if (f[0] == "frame" && f.size() == 8) check_frame_line(f);
    else if (f[0] == "stream" && f.size() >= 3) check_stream_line(f);
    else { ++failures; std::cerr << "bad golden line: " << line << "\n"; }
    ++vectors;
  }
  CHECK(vectors >= 10, "golden file has " << vectors << " vectors");
  std::cout << checks << " checks, " << failures << " failure(s), " << vectors << " golden vectors\n";
  return failures == 0 ? 0 : 1;
}
