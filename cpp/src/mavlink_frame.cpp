#include "px4v/mavlink_frame.hpp"

#include <algorithm>

namespace px4v {

uint16_t crc_x25(const uint8_t* data, size_t len, uint16_t crc) {
  for (size_t i = 0; i < len; ++i) {
    uint8_t tmp = static_cast<uint8_t>(data[i] ^ (crc & 0xFF));
    tmp = static_cast<uint8_t>(tmp ^ (tmp << 4));
    crc = static_cast<uint16_t>((crc >> 8) ^ (static_cast<uint16_t>(tmp) << 8) ^
                                (static_cast<uint16_t>(tmp) << 3) ^ (tmp >> 4));
  }
  return crc;
}

std::optional<uint8_t> crc_extra(uint32_t msgid) {
  switch (msgid) {
    case 0: return 50;     // HEARTBEAT
    case 1: return 124;    // SYS_STATUS
    case 30: return 39;    // ATTITUDE
    case 32: return 185;   // LOCAL_POSITION_NED
    case 33: return 104;   // GLOBAL_POSITION_INT
    case 76: return 152;   // COMMAND_LONG
    case 77: return 143;   // COMMAND_ACK
    case 84: return 143;   // SET_POSITION_TARGET_LOCAL_NED
    case 253: return 83;   // STATUSTEXT
    default: return std::nullopt;
  }
}

std::vector<uint8_t> encode_frame(uint32_t msgid, const std::vector<uint8_t>& payload, uint8_t seq,
                                  uint8_t sysid, uint8_t compid) {
  const auto extra = crc_extra(msgid);
  if (!extra || payload.size() > 255) return {};
  // MAVLink v2 truncates trailing zero bytes but always sends at least one.
  size_t len = payload.size();
  while (len > 1 && payload[len - 1] == 0) --len;
  if (len == 0) len = 1;
  std::vector<uint8_t> wire;
  wire.reserve(kHeaderLen + len + kCrcLen);
  wire.push_back(kStxV2);
  wire.push_back(static_cast<uint8_t>(len));
  wire.push_back(0);  // incompat flags
  wire.push_back(0);  // compat flags
  wire.push_back(seq);
  wire.push_back(sysid);
  wire.push_back(compid);
  wire.push_back(static_cast<uint8_t>(msgid & 0xFF));
  wire.push_back(static_cast<uint8_t>((msgid >> 8) & 0xFF));
  wire.push_back(static_cast<uint8_t>((msgid >> 16) & 0xFF));
  for (size_t i = 0; i < len; ++i) wire.push_back(i < payload.size() ? payload[i] : 0);
  std::vector<uint8_t> covered(wire.begin() + 1, wire.end());
  covered.push_back(*extra);
  const uint16_t crc = crc_x25(covered.data(), covered.size());
  wire.push_back(static_cast<uint8_t>(crc & 0xFF));
  wire.push_back(static_cast<uint8_t>(crc >> 8));
  return wire;
}

std::vector<ParseEvent> FrameParser::feed(const uint8_t* data, size_t len) {
  buf_.insert(buf_.end(), data, data + len);
  std::vector<ParseEvent> events;
  for (;;) {
    auto stx = std::find(buf_.begin(), buf_.end(), kStxV2);
    if (stx == buf_.end()) {
      stats_.garbage_bytes += buf_.size();
      buf_.clear();
      break;
    }
    if (stx != buf_.begin()) {
      stats_.garbage_bytes += static_cast<size_t>(stx - buf_.begin());
      buf_.erase(buf_.begin(), stx);
    }
    if (buf_.size() < kHeaderLen) break;
    const size_t length = buf_[1];
    if (buf_[2] != 0) {  // signed or unknown incompat flags: do not guess a length
      ++stats_.unsupported;
      events.push_back({ParseKind::Unsupported, {}});
      buf_.erase(buf_.begin());
      continue;
    }
    const size_t total = kHeaderLen + length + kCrcLen;
    if (buf_.size() < total) break;
    const uint32_t msgid = static_cast<uint32_t>(buf_[7]) | (static_cast<uint32_t>(buf_[8]) << 8) |
                           (static_cast<uint32_t>(buf_[9]) << 16);
    const auto extra = crc_extra(msgid);
    if (!extra) {
      ++stats_.unknown_msg;
      events.push_back({ParseKind::UnknownMsg, {}});
      buf_.erase(buf_.begin(), buf_.begin() + static_cast<long>(total));
      continue;
    }
    std::vector<uint8_t> covered(buf_.begin() + 1, buf_.begin() + static_cast<long>(total - kCrcLen));
    covered.push_back(*extra);
    const uint16_t expected = crc_x25(covered.data(), covered.size());
    const uint16_t received = static_cast<uint16_t>(buf_[total - 2] | (buf_[total - 1] << 8));
    if (expected != received) {
      ++stats_.bad_crc;
      events.push_back({ParseKind::BadCrc, {}});
      buf_.erase(buf_.begin());  // resync: a real frame may start inside this one
      continue;
    }
    Frame frame;
    frame.msgid = msgid;
    frame.seq = buf_[4];
    frame.sysid = buf_[5];
    frame.compid = buf_[6];
    frame.payload.assign(buf_.begin() + kHeaderLen, buf_.begin() + static_cast<long>(kHeaderLen + length));
    buf_.erase(buf_.begin(), buf_.begin() + static_cast<long>(total));
    ++stats_.frames;
    events.push_back({ParseKind::Frame, std::move(frame)});
  }
  return events;
}

}  // namespace px4v
