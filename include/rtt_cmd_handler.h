#pragma once

#include "postform/rtt/rtt.h"

class RttCmdHandlerListener {
 public:
  enum class CmdHandleStatus : uint8_t {
    OUT_OF_RANGE = 0,
    BUSY = 1,
    INVALID_ARGS = 2,
    FLASH_ERROR = 3,
    HANDLED = 0xA5,
  };

  enum class Cmd : uint8_t {
    ERASE_SUBSECTOR = 0,
    WRITE_PAGE = 1,
    JUMP_TO_ENTRYPOINT = 3,
  };

  [[nodiscard]] virtual CmdHandleStatus handle_cmd(Cmd, std::span<const uint8_t> data) noexcept = 0;

  virtual ~RttCmdHandlerListener() noexcept = default;
};

class RttCmdHandler final {
 public:
  using Channel = Postform::Rtt::Channel;

  RttCmdHandler(Channel& up_channel, Channel& down_channel) noexcept;

  void run() noexcept;

  void register_listener(RttCmdHandlerListener& listener) noexcept;
  void unregister_listener() noexcept;

  [[nodiscard]] bool did_receive_cmd() const noexcept;

 private:
  Postform::Rtt::Channel& m_up_channel;
  Postform::Rtt::Channel& m_down_channel;
  RttCmdHandlerListener* m_listener = nullptr;
  bool m_did_receive_cmd = false;

  constexpr static size_t MAX_CMD_DATA_SIZE = 4096;
  std::array<uint8_t, MAX_CMD_DATA_SIZE> m_workspace{};
};
