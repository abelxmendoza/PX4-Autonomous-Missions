// MAVLink v2 frame codec: C++ counterpart of px4_offboard/comms/mavlink_frame.py.
//
// The two implementations are checked against the same golden vectors
// (cpp/test/golden_frames.txt), which the Python side verifies byte-for-byte
// against pymavlink. This is a small, dependency-free library on purpose: it is
// the shape of code that would sit on a companion computer or flight-controller
// side of a UART link.
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace px4v {

constexpr uint8_t kStxV2 = 0xFD;
constexpr size_t kHeaderLen = 10;
constexpr size_t kCrcLen = 2;

uint16_t crc_x25(const uint8_t* data, size_t len, uint16_t crc = 0xFFFF);

// crc_extra for the message ids this project uses; nullopt for anything else.
std::optional<uint8_t> crc_extra(uint32_t msgid);

// Returns an empty vector for an unknown msgid or a payload over 255 bytes.
std::vector<uint8_t> encode_frame(uint32_t msgid, const std::vector<uint8_t>& payload,
                                  uint8_t seq, uint8_t sysid = 1, uint8_t compid = 1);

struct Frame {
  uint32_t msgid = 0;
  uint8_t seq = 0, sysid = 0, compid = 0;
  std::vector<uint8_t> payload;
};

enum class ParseKind { Frame, BadCrc, UnknownMsg, Unsupported };

struct ParseEvent {
  ParseKind kind;
  Frame frame;  // valid only when kind == Frame
};

struct ParseStats {
  size_t frames = 0, bad_crc = 0, unknown_msg = 0, unsupported = 0, garbage_bytes = 0;
};

// Incremental parser: feed any chunking of the byte stream.
class FrameParser {
 public:
  std::vector<ParseEvent> feed(const uint8_t* data, size_t len);
  size_t pending_bytes() const { return buf_.size(); }
  void reset() { buf_.clear(); }
  const ParseStats& stats() const { return stats_; }

 private:
  std::vector<uint8_t> buf_;
  ParseStats stats_;
};

}  // namespace px4v
