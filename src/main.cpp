#include <array>

#include "cortex_m_hal/systick.h"
#include "hw/flash.h"
#include "hw/gpio.h"
#include "hw/pwr.h"
#include "hw/qspi_flash.h"
#include "hw/rcc.h"
#include "hw/sdram.h"
#include "postform/logger.h"
#include "postform/rtt/rtt.h"
#include "postform/rtt/transport.h"
#include "postform/serial_logger.h"
#include "systick_config.h"

std::array<std::uint8_t, 1024u> postform_channel_buffer;

extern "C" Postform::Rtt::ControlBlock<1u, 0u> _SEGGER_RTT{
    std::array{
        Postform::Rtt::ChannelDescriptor{
            .name = "postform",
            .buffer = postform_channel_buffer,
        },
    },
    {},
};

Postform::Rtt::Transport transport{&_SEGGER_RTT.up_channels[0]};
Postform::SerialLogger<Postform::Rtt::Transport> logger{&transport};

namespace {
void configure_voltage_regulator(Hw::Rcc::RegBank& rcc_regs, Hw::Pwr::RegBank& power_regs) noexcept {
  // Enable pwr clock
  rcc_regs.get_register<Hw::Rcc::Apb1Enr>().read_modify_write(
      [](auto reg) { reg.template write<Hw::Rcc::PwrEn>(true); });

  power_regs.get_register<Hw::Pwr::ControlReg1>().read_modify_write(
      [=](auto reg) { reg.template write<Hw::Pwr::VoltageScalingOutputField>(Hw::Pwr::VoltageScaling::MODE_3); });

  while (!power_regs.get_register<Hw::Pwr::ControlStatusReg1>().read().read<Hw::Pwr::VoltageScalingRdyField>())
    ;

  power_regs.get_register<Hw::Pwr::ControlReg1>().read_modify_write(
      [=](auto reg) { reg.template write<Hw::Pwr::OverDriveEnField>(true); });

  while (!power_regs.get_register<Hw::Pwr::ControlStatusReg1>().read().read<Hw::Pwr::OverDriveRdyField>())
    ;

  power_regs.get_register<Hw::Pwr::ControlReg1>().read_modify_write(
      [=](auto reg) { reg.template write<Hw::Pwr::OverDriveSwEnField>(true); });

  while (!power_regs.get_register<Hw::Pwr::ControlStatusReg1>().read().read<Hw::Pwr::OverDriveSwRdyField>())
    ;
}

void configure_system_clock(Hw::Rcc::RegBank& rcc_regs, Hw::Flash::RegBank& flash_regs,
                            Hw::Pwr::RegBank& power_regs) noexcept {
  constexpr static uint32_t HSE_FREQ_MHZ = 25;
  constexpr static uint32_t PLL_OUT_FREQ_MHZ = 216;

  auto clock_config_reg = rcc_regs.get_register<Hw::Rcc::ClockConfigReg>();
  auto control_reg = rcc_regs.get_register<Hw::Rcc::ControlReg>();
  auto pll_reg = rcc_regs.get_register<Hw::Rcc::PllReg>();

  control_reg.read_modify_write([](auto reg) {
    reg.template write<Hw::Rcc::HseBypField>(true);
    reg.template write<Hw::Rcc::HseOnField>(true);
  });

  while (!control_reg.read().read<Hw::Rcc::HseRdyField>())
    ;

  pll_reg.read_modify_write([=](auto reg) {
    reg.template write<Hw::Rcc::PllMField>(HSE_FREQ_MHZ);
    reg.template write<Hw::Rcc::PllNField>(PLL_OUT_FREQ_MHZ * 2);
    reg.template write<Hw::Rcc::PllPField>(Hw::Rcc::PllPSetting::DIV_2);
    reg.template write<Hw::Rcc::PllQField>(Hw::Rcc::PllQSetting::DIV_4);
    reg.template write<Hw::Rcc::PllSrcField>(Hw::Rcc::PllSource::HSE);
  });

  control_reg.read_modify_write([](auto reg) { reg.template write<Hw::Rcc::PllOnField>(true); });

  configure_voltage_regulator(rcc_regs, power_regs);

  constexpr static uint8_t FLASH_WAIT_STATES = 7;
  flash_regs.get_register<Hw::Flash::AccessCtrlReg>().read_modify_write([=](auto reg) {
    reg.template write<Hw::Flash::LatencyField>(FLASH_WAIT_STATES);
    reg.template write<Hw::Flash::ArtEnField>(true);
    reg.template write<Hw::Flash::PreFetchEnField>(true);
  });

  while (!control_reg.read().read<Hw::Rcc::PllRdyField>())
    ;

  clock_config_reg.read_modify_write([](auto reg) {
    reg.template write<Hw::Rcc::AhbPreField>(Hw::Rcc::AhbPreSetting::DIV_1);
    reg.template write<Hw::Rcc::Apb1PreField>(Hw::Rcc::ApbPreSetting::DIV_4);
    reg.template write<Hw::Rcc::Apb2PreField>(Hw::Rcc::ApbPreSetting::DIV_2);
    reg.template write<Hw::Rcc::SwitchField>(Hw::Rcc::SystemClockSwitch::PLL);
  });

  while (clock_config_reg.read().read<Hw::Rcc::SwitchStatusField>() != Hw::Rcc::SystemClockSwitch::PLL)
    ;
}

extern "C" uint8_t qspi_data_bin[];
extern "C" int qspi_data_bin_len;

bool validate_qspi_data() noexcept {
  volatile uint8_t* const qspi_base = std::bit_cast<uint8_t*>(0x9000'0000);
  for (int i = 0; i < qspi_data_bin_len; i++) {
    if (qspi_data_bin[i] != qspi_base[i]) {
      LOG_ERROR(&logger, "Unexpected data at byte offset %d: [0x%hhx != 0x%hhx]", i, qspi_data_bin[i], qspi_base[i]);
      return false;
    }
  }
  LOG_INFO(&logger, "QSPI validation successful");
  return true;
}

}  // namespace

int main() {
  Hw::Rcc::RegBank rcc_regs{Hw::MmappedRegs{0x4002'3800}};
  Hw::Flash::RegBank flash_regs{Hw::MmappedRegs{0x4002'3C00}};
  Hw::Pwr::RegBank pwr_regs{Hw::MmappedRegs{0x4000'7000}};
  Hw::Fmc::RegBank fmc_regs{Hw::MmappedRegs{0xA000'0000}};
  Hw::QuadSpi::RegBank quadspi_regs{Hw::MmappedRegs{0xA000'1000}};
  configure_system_clock(rcc_regs, flash_regs, pwr_regs);

  Hw::Sdram sdram{rcc_regs, fmc_regs};
  Hw::QspiFlash qspi_flash{rcc_regs, quadspi_regs};

  SysTick& systick = SysTick::getInstance();
  systick.init(App::SYSTICK_CLK_HZ);
  sdram.init();
  qspi_flash.init();

  LOG_INFO(&logger, "STM32 Linux Bootloader");

  validate_qspi_data();

  volatile int* const sdram_base = std::bit_cast<int*>(0xC000'0000);
  int value = 0;
  while (true) {
    systick.delay(1'000);

    *sdram_base = ++value;
    LOG_DEBUG(&logger, "Sdram value is %d", *sdram_base);
  }
}

void operator delete(void*) { DITTO_UNIMPLEMENTED(); }
