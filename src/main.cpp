#include <array>

#include "cortex_m_hal/systick.h"
#include "hw/flash.h"
#include "hw/gpio.h"
#include "hw/pwr.h"
#include "hw/qspi_flash.h"
#include "hw/rcc.h"
#include "hw/sdram.h"
#include "hw/uart.h"
#include "postform/logger.h"
#include "postform/rtt/rtt.h"
#include "postform/rtt/transport.h"
#include "postform/serial_logger.h"
#include "rtt_bootloader.h"
#include "systick_config.h"

namespace {

std::array<std::uint8_t, 1024u> postform_channel_buffer;
std::array<std::uint8_t, 1024u> bootloader_up_buffer;
std::array<std::uint8_t, 8192u> bootloader_down_buffer;

std::array up_descriptors{
    Postform::Rtt::ChannelDescriptor{
        .name = "postform",
        .buffer = postform_channel_buffer,
    },
    Postform::Rtt::ChannelDescriptor{
        .name = "bootloader_up",
        .buffer = bootloader_up_buffer,
    },
};

std::array down_descriptors{
    Postform::Rtt::ChannelDescriptor{
        .name = "bootloader_down",
        .buffer = bootloader_down_buffer,
    },
};

}  // namespace

extern "C" Postform::Rtt::ControlBlock<2u, 1u> _SEGGER_RTT{std::span{up_descriptors}, std::span{down_descriptors}};

Postform::Rtt::Transport transport{&_SEGGER_RTT.up_channels[0]};
Postform::SerialLogger<Postform::Rtt::Transport> logger{&transport};

namespace {
void configure_voltage_regulator(Hw::Rcc::RegBank& rcc_regs, Hw::Pwr::RegBank& power_regs) noexcept {
  // Enable pwr clock
  rcc_regs.get_register<Hw::Rcc::Apb1Enr>().read_modify_write(
      [](auto reg) { reg.template write<Hw::Rcc::PwrEn>(true); });

  power_regs.get_register<Hw::Pwr::ControlReg1>().read_modify_write(
      [](auto reg) { reg.template write<Hw::Pwr::VoltageScalingOutputField>(Hw::Pwr::VoltageScaling::MODE_3); });

  while (!power_regs.get_register<Hw::Pwr::ControlStatusReg1>().read().read<Hw::Pwr::VoltageScalingRdyField>())
    ;

  power_regs.get_register<Hw::Pwr::ControlReg1>().read_modify_write(
      [](auto reg) { reg.template write<Hw::Pwr::OverDriveEnField>(true); });

  while (!power_regs.get_register<Hw::Pwr::ControlStatusReg1>().read().read<Hw::Pwr::OverDriveRdyField>())
    ;

  power_regs.get_register<Hw::Pwr::ControlReg1>().read_modify_write(
      [](auto reg) { reg.template write<Hw::Pwr::OverDriveSwEnField>(true); });

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
    reg.template write<Hw::Rcc::PllQField>(Hw::Rcc::PllQSetting::DIV_9);  // PLLQ is now 48 MHz
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

  // Set the USB clock source to the main PLL
  auto dedicated_clock_config_reg = rcc_regs.get_register<Hw::Rcc::DedicatedClocksConfigReg>();
  dedicated_clock_config_reg.read_modify_write(
      [](auto reg) { reg.template write<Hw::Rcc::Clock48MSelField>(Hw::Rcc::Clock48MSrc::PLL); });
}

Hw::Rcc::RegBank rcc_regs{Hw::MmappedRegs{0x4002'3800}};
Hw::Flash::RegBank flash_regs{Hw::MmappedRegs{0x4002'3C00}};
Hw::Pwr::RegBank pwr_regs{Hw::MmappedRegs{0x4000'7000}};
Hw::Fmc::RegBank fmc_regs{Hw::MmappedRegs{0xA000'0000}};
Hw::QuadSpi::RegBank quadspi_regs{Hw::MmappedRegs{0xA000'1000}};
Hw::Uart::RegBank uart_regs{Hw::MmappedRegs{0x4001'1000}};

Hw::Sdram sdram{rcc_regs, fmc_regs};
Hw::QspiFlash qspi_flash{rcc_regs, quadspi_regs};

RttBootloader bootloader{_SEGGER_RTT.up_channels[1], _SEGGER_RTT.down_channels[0], qspi_flash};

void init_uart() noexcept {
  rcc_regs.get_register<Hw::Rcc::Apb2Enr>().read_modify_write(
      [](auto reg) { reg.template write<Hw::Rcc::Uart1En>(true); });

  uart_regs.get_register<Hw::Uart::ControlReg1>().read_modify_write([](auto reg) {
    reg.template write<Hw::Uart::TxEnField>(false);
    reg.template write<Hw::Uart::RxEnField>(false);
    reg.template write<Hw::Uart::UartEnField>(false);
  });

  uart_regs.get_register<Hw::Uart::BaudrateReg>().write(
      [](auto reg) { reg.template write<Hw::Uart::BaudrateField>(937); });

  uart_regs.get_register<Hw::Uart::ControlReg1>().read_modify_write([](auto reg) {
    reg.template write<Hw::Uart::TxEnField>(true);
    reg.template write<Hw::Uart::RxEnField>(true);
    reg.template write<Hw::Uart::UartEnField>(true);
  });
}

void send_char(const char c) noexcept {
  while (!uart_regs.get_register<Hw::Uart::InterruptAndStatusReg>().read().read<Hw::Uart::TxEmptyField>())
    ;

  uart_regs.get_register<Hw::Uart::TransmitDataReg>().write(
      [=](auto reg) { reg.template write<Hw::Uart::TransmitDataField>(c); });
}

void send_string(const char* str) noexcept {
  while (*str != '\0') {
    send_char(*str);
    str++;
  }
}

}  // namespace

extern "C" void _exit() noexcept {}

int main() {
  configure_system_clock(rcc_regs, flash_regs, pwr_regs);

  SysTick& systick = SysTick::getInstance();
  systick.init(App::SYSTICK_CLK_HZ);

  sdram.init();
  qspi_flash.init();

  LOG_INFO(&logger, "STM32 Linux Bootloader");

  rcc_regs.get_register<Hw::Rcc::Ahb1Enr>().read_modify_write([](auto reg) {
    reg.template write<Hw::Rcc::GpioAClkEn>(true);
    reg.template write<Hw::Rcc::GpioBClkEn>(true);
  });

  auto uart_tx_pin =
      Hw::get_gpio_bank(Hw::GpioBankId::A)
          .try_get_as_alternate_function(Hw::GpioPinNumber{9}, Hw::GpioAltFunc::AF7_SPI2_3_USART1_2_3_UART5_SPDIFRX);
  DITTO_VERIFY(uart_tx_pin.is_valid());
  auto uart_rx_pin =
      Hw::get_gpio_bank(Hw::GpioBankId::B)
          .try_get_as_alternate_function(Hw::GpioPinNumber{7}, Hw::GpioAltFunc::AF7_SPI2_3_USART1_2_3_UART5_SPDIFRX);
  DITTO_VERIFY(uart_rx_pin.is_valid());

  init_uart();
  send_string("Hi from bootloader!");

  LOG_INFO(&logger, "Running bootloader");

  auto start = systick.getCoarseTickCount();

  while (true) {
    bootloader.run();

    if ((systick.getCoarseTickCount() - start) > 5000) {
      if (!bootloader.did_receive_cmd()) {
        LOG_INFO(&logger, "Time out on cmd handling. Booting linux");
        bootloader.boot();
      }
    }
  }
}

void operator delete(void*) { DITTO_UNIMPLEMENTED(); }
