#include <array>

#include "cortex_m_hal/systick.h"
#include "hw/flash.h"
#include "hw/gpio.h"
#include "hw/pwr.h"
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

Hw::Rcc::RegBank rcc_regs{Hw::MmappedRegs{0x4002'3800}};
Hw::Flash::RegBank flash_regs{Hw::MmappedRegs{0x4002'3C00}};
Hw::Pwr::RegBank pwr_regs{Hw::MmappedRegs{0x4000'7000}};
Hw::Fmc::RegBank fmc_regs{Hw::MmappedRegs{0xA000'0000}};

namespace {
void enable_gpio_i_bank_clk(Hw::Rcc::RegBank& rcc_regs) noexcept {
  rcc_regs.get_register<Hw::Rcc::Ahb1Enr>().read_modify_write(
      [](auto reg) { reg.template write<Hw::Rcc::GpioIClkEn>(true); });
}

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
  flash_regs.get_register<Hw::Flash::AccessCtrlReg>().read_modify_write(
      [=](auto reg) { reg.template write<Hw::Flash::LatencyField>(FLASH_WAIT_STATES); });

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

}  // namespace

int main() {
  Hw::Sdram sdram;

  configure_system_clock(rcc_regs, flash_regs, pwr_regs);

  SysTick& systick = SysTick::getInstance();
  systick.init(App::SYSTICK_CLK_HZ);

  LOG_DEBUG(&logger, "Hello world!");
  LOG_INFO(&logger, "Hello world! (Again!)");
  LOG_WARNING(&logger, "Hello world! (And again!)");

  Hw::GpioBank& gpioBankI = Hw::get_gpio_bank(Hw::GpioBankId::I);

  enable_gpio_i_bank_clk(rcc_regs);
  Hw::OutputGpioPin pinI1 = gpioBankI.try_get_as_output(Hw::GpioPinNumber{1}, Hw::GpioState::High);
  DITTO_VERIFY(pinI1);

  sdram.init(rcc_regs, fmc_regs);

  int value = 0;
  while (true) {
    systick.delay(1'000);
    LOG_ERROR(&logger, "Hello world! (Yet again!)");
    volatile int* sdram_base = (int*)0xC000'0000;
    LOG_DEBUG(&logger, "Sdram value is %d", *sdram_base);
    *sdram_base = value++;
    pinI1.toggle_state();
  }
}
