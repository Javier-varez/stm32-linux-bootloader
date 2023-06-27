#include "rtt_bootloader.h"

#include "cortex_m_hal/systick.h"
#include "logger.h"

namespace {
using CmdHandler = RttCmdHandlerListener::CmdHandleStatus (RttBootloader::*)(std::span<const uint8_t>) noexcept;

struct CommandTableEntry {
  RttBootloader::Cmd cmd;
  CmdHandler handler;
};
}  // namespace

RttCmdHandlerListener::CmdHandleStatus RttBootloader::erase_subsector(std::span<const uint8_t> data) noexcept {
  if (data.size() != sizeof(uint32_t)) {
    return RttCmdHandlerListener::CmdHandleStatus::INVALID_ARGS;
  }
  uint32_t addr{};
  memcpy(&addr, &*data.begin(), data.size());
  LOG_DEBUG(&logger, "Erase subsector invoked, addr %x", addr);
  Ditto::Result result = m_flash.erase_subsector(addr);
  if (!result.is_ok()) {
    LOG_DEBUG(&logger, "Erase subsector failed %d", static_cast<int>(result.error_value()));
    return RttCmdHandlerListener::CmdHandleStatus::FLASH_ERROR;
  }
  return RttCmdHandlerListener::CmdHandleStatus::HANDLED;
}

RttCmdHandlerListener::CmdHandleStatus RttBootloader::write_page(std::span<const uint8_t> data) noexcept {
  if (data.size() < sizeof(uint32_t)) {
    return RttCmdHandlerListener::CmdHandleStatus::INVALID_ARGS;
  }
  uint32_t addr{};
  memcpy(&addr, &*data.begin(), sizeof(addr));
  LOG_DEBUG(&logger, "write page invoked, addr %x", addr);
  Ditto::Result result = m_flash.write_page(addr, data.subspan(sizeof(uint32_t), data.size() - sizeof(uint32_t)));
  if (!result.is_ok()) {
    LOG_DEBUG(&logger, "write page failed %d", static_cast<int>(result.error_value()));
    return RttCmdHandlerListener::CmdHandleStatus::FLASH_ERROR;
  }
  return RttCmdHandlerListener::CmdHandleStatus::HANDLED;
}

using EntryPoint = void (*)(uint32_t, uint32_t, void *);

RttCmdHandlerListener::CmdHandleStatus RttBootloader::jump_to_entrypoint(std::span<const uint8_t>) noexcept {
  LOG_DEBUG(&logger, "Jump to entrypoint invoked");
  m_flash.enable_memory_mapped_mode();
  LOG_DEBUG(&logger, "memory mapped flash");

  SysTick &systick = SysTick::getInstance();
  systick.disable();
  systick.maskIrq();

  LOG_DEBUG(&logger, "Disabled systick");

  constexpr static uint32_t BASE = 0x9000'0000;
  void *const dtb_ptr = std::bit_cast<void *>(BASE + 0x400000);
  const EntryPoint entry = std::bit_cast<EntryPoint>(BASE + 1);
  entry(0, 0xffffffff, dtb_ptr);

  return RttCmdHandlerListener::CmdHandleStatus::HANDLED;
}

RttBootloader::RttBootloader(Channel &up_channel, Channel &down_channel, Hw::QspiFlash &flash) noexcept
    : m_cmd_handler{up_channel, down_channel}, m_flash{flash} {
  m_cmd_handler.register_listener(*this);
}

void RttBootloader::run() noexcept { m_cmd_handler.run(); }
void RttBootloader::boot() noexcept { jump_to_entrypoint(std::span<const uint8_t>{}); }

bool RttBootloader::did_receive_cmd() const noexcept { return m_cmd_handler.did_receive_cmd(); }

RttCmdHandlerListener::CmdHandleStatus RttBootloader::handle_cmd(Cmd cmd, std::span<const uint8_t> data) noexcept {
  constexpr static std::array COMMAND_TABLE{
      CommandTableEntry{
          .cmd = RttBootloader::Cmd::ERASE_SUBSECTOR,
          .handler = &RttBootloader::erase_subsector,
      },
      CommandTableEntry{
          .cmd = RttBootloader::Cmd::WRITE_PAGE,
          .handler = &RttBootloader::write_page,
      },
      CommandTableEntry{
          .cmd = RttBootloader::Cmd::JUMP_TO_ENTRYPOINT,
          .handler = &RttBootloader::jump_to_entrypoint,
      },
  };
  for (const CommandTableEntry &entry : COMMAND_TABLE) {
    if (cmd == entry.cmd) {
      return (this->*entry.handler)(data);
    }
  }
  return RttCmdHandlerListener::CmdHandleStatus::BUSY;
}
