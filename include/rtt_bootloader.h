#pragma once

#include "hw/qspi_flash.h"
#include "postform/rtt/rtt.h"
#include "rtt_cmd_handler.h"

class RttBootloader final : public RttCmdHandlerListener {
 public:
  using Channel = Postform::Rtt::Channel;

  RttBootloader(Channel& up_channel, Channel& down_channel, Hw::QspiFlash& flash) noexcept;

  void run() noexcept;

 private:
  RttCmdHandler m_cmd_handler;
  Hw::QspiFlash& m_flash;

  [[nodiscard]] CmdHandleStatus handle_cmd(Cmd, std::span<const uint8_t> data) noexcept final;

  RttCmdHandlerListener::CmdHandleStatus erase_subsector(std::span<const uint8_t> data) noexcept;
  RttCmdHandlerListener::CmdHandleStatus write_page(std::span<const uint8_t> data) noexcept;
  RttCmdHandlerListener::CmdHandleStatus jump_to_entrypoint(std::span<const uint8_t> data) noexcept;
};
