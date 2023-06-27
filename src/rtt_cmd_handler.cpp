
#include "rtt_cmd_handler.h"

#include "ditto/crc32c.h"
#include "logger.h"
#include "postform/rtt/transport.h"

namespace {

struct [[gnu::packed]] Header {
  RttCmdHandlerListener::Cmd cmd{};
  uint32_t size{};
};

struct [[gnu::packed]] Footer {
  uint32_t crc32{};
};

struct Packet {
  RttCmdHandlerListener::Cmd cmd{};
  uint32_t size{};
  size_t data_offset{};
};

template <typename T>
[[nodiscard]] T read_data(RttCmdHandler::Channel& channel, const size_t offset) noexcept {
  DITTO_VERIFY(offset < channel.size);

  T data;
  std::array<volatile uint8_t, sizeof(T)>& dest = *reinterpret_cast<std::array<volatile uint8_t, sizeof(T)>*>(&data);
  for (size_t i = 0; i < dest.size(); i++) {
    dest[i] = channel.buffer[(offset + i) % channel.size];
  }

  return data;
}

[[nodiscard]] std::optional<Packet> read_down_channel(RttCmdHandler::Channel& channel) noexcept {
  const auto calc_available_size = [&channel](const uint32_t read_ptr, const uint32_t write_ptr) noexcept -> uint32_t {
    if (write_ptr >= read_ptr) {
      return write_ptr - read_ptr;
    }
    return write_ptr + channel.size - read_ptr;
  };

  const uint32_t read_ptr = channel.read.load(std::memory_order_relaxed);
  const uint32_t write_ptr = channel.write.load(std::memory_order_acquire);

  const uint32_t available_size = calc_available_size(read_ptr, write_ptr);
  if (available_size == 0) {
    return std::optional<Packet>{};
  }

  if (available_size < sizeof(Header) + sizeof(Footer)) {
    channel.read.store(write_ptr, std::memory_order_release);
    LOG_WARNING(&logger, "Received packet is too small");
    return std::optional<Packet>{};
  }

  const auto hdr = read_data<Header>(channel, read_ptr);
  if (hdr.size < sizeof(Header) + sizeof(Footer)) {
    channel.read.store(write_ptr, std::memory_order_release);
    LOG_WARNING(&logger, "Received packet has a reported size that doesn't cover header and footer");
    return std::optional<Packet>{};
  }

  if (hdr.size > available_size) {
    channel.read.store(write_ptr, std::memory_order_release);
    LOG_WARNING(&logger, "Received incomplete packet! Discarding it! Packet says %u. But ring buffer only contains %u",
                hdr.size, available_size);
    return std::optional<Packet>{};
  }

  const auto footer = read_data<Footer>(channel, (read_ptr + hdr.size - sizeof(Footer)) % channel.size);

  Crc32cCalculator crc32;
  for (size_t i = 0; i < hdr.size - sizeof(Footer); i++) {
    const uint8_t* ptr = &channel.buffer[(read_ptr + i) % channel.size];
    crc32.hash(Ditto::span<const uint8_t>{ptr, 1});
  }
  const uint32_t calculated_crc32 = crc32.finish();
  if (calculated_crc32 != footer.crc32) {
    channel.read.store(write_ptr, std::memory_order_release);
    LOG_WARNING(&logger, "Skipping packet with invalid Crc32. Calculated %x but got %x", calculated_crc32,
                footer.crc32);
    return std::optional<Packet>{};
  }

  return std::optional{
      Packet{
          .cmd = hdr.cmd,
          .size = hdr.size - sizeof(hdr) - sizeof(footer),
          .data_offset = (read_ptr + sizeof(hdr)) % channel.size,
      },
  };
}

void advance_down_channel(RttCmdHandler::Channel& channel, const Packet& packet) noexcept {
  const uint32_t read_ptr = channel.read.load(std::memory_order_relaxed);
  const uint32_t new_read_ptr = (read_ptr + packet.size + sizeof(Header) + sizeof(Footer)) % channel.size;
  channel.read.store(new_read_ptr, std::memory_order_release);
}

[[nodiscard]] std::span<const uint8_t> copyDataToWorkspace(const Postform::Rtt::Channel& channel, const Packet& packet,
                                                           std::span<uint8_t> workspace) noexcept {
  DITTO_VERIFY(packet.size <= workspace.size());
  uint32_t offset = packet.data_offset;

  for (uint8_t& byte : std::span<uint8_t>{&workspace[0], packet.size}) {
    byte = channel.buffer[offset];
    if (++offset >= channel.size) {
      offset = 0;
    }
  }

  return std::span<uint8_t>{&workspace[0], packet.size};
}

void write_response(Postform::Rtt::Channel& channel, const RttCmdHandlerListener::CmdHandleStatus status) noexcept {
  Postform::Rtt::Transport transport{&channel};
  transport.write(static_cast<uint8_t>(status));
  transport.commit();
}

}  // namespace

RttCmdHandler::RttCmdHandler(Postform::Rtt::Channel& up_channel, Postform::Rtt::Channel& down_channel) noexcept
    : m_up_channel{up_channel}, m_down_channel{down_channel} {
  m_up_channel.flags.store(Postform::Rtt::Flags::BLOCK_IF_FULL, std::memory_order_relaxed);
}

void RttCmdHandler::register_listener(RttCmdHandlerListener& listener) noexcept { m_listener = &listener; }
void RttCmdHandler::unregister_listener() noexcept { m_listener = nullptr; }

void RttCmdHandler::run() noexcept {
  const std::optional<Packet> packet = read_down_channel(m_down_channel);
  if (!packet.has_value()) {
    return;
  }

  // LOG_DEBUG(&logger, "Packet received with cmd = %hhu and size = %u", static_cast<uint8_t>(packet.value().cmd),
  //          packet.value().size);

  RttCmdHandlerListener::CmdHandleStatus status = RttCmdHandlerListener::CmdHandleStatus::OUT_OF_RANGE;
  if (packet->size <= MAX_CMD_DATA_SIZE) {
    const std::span workspace{copyDataToWorkspace(m_down_channel, packet.value(), m_workspace)};
    advance_down_channel(m_down_channel, packet.value());
    status = m_listener->handle_cmd(packet.value().cmd, workspace);
  } else {
    advance_down_channel(m_down_channel, packet.value());
  }

  write_response(m_up_channel, status);
}
