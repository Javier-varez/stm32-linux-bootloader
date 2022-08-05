#include "hw/sdram.h"

#include <string.h>

#include "cortex_m_hal/systick.h"

namespace Hw {

void Sdram::init(Hw::Rcc::RegBank& rcc_regs, Fmc::RegBank& fmc_regs) noexcept {
  configure_pins(rcc_regs);
  configure_fmc(rcc_regs, fmc_regs);
}

void Sdram::configure_fmc(Rcc::RegBank& rcc_regs, Fmc::RegBank& fmc_regs) noexcept {
  rcc_regs.get_register<Rcc::Ahb3Enr>().read_modify_write([](auto reg) { reg.template write<Rcc::FmcEn>(true); });

  fmc_regs.get_register<Fmc::SdramCr1>().write([](auto reg) {
    reg.template write<Fmc::sdcr::Nc>(Fmc::NumberOfColumnAddrBits::Bits8);
    reg.template write<Fmc::sdcr::Nr>(Fmc::NumberOfRowAddrBits::Bits12);
    reg.template write<Fmc::sdcr::Nb>(Fmc::NumInternalBanks::Four);
    reg.template write<Fmc::sdcr::Wp>(false);
    reg.template write<Fmc::sdcr::Cas>(Fmc::CasLatency::Cycles2);
    reg.template write<Fmc::sdcr::Mwid>(Fmc::MemBusDataWidth::W16);
    reg.template write<Fmc::sdcr::Sdclk>(Fmc::SdclkConfig::HclkDiv2);
    reg.template write<Fmc::sdcr::Rburst>(true);
    reg.template write<Fmc::sdcr::Rpipe>(Fmc::ReadPipe::NoHclkDelay);
  });

  fmc_regs.get_register<Fmc::SdramTr1>().write([](auto reg) {
    reg.template write<Fmc::sdtr::Tmrd>(1);
    reg.template write<Fmc::sdtr::Tras>(4);
    reg.template write<Fmc::sdtr::Trc>(6);
    reg.template write<Fmc::sdtr::Trcd>(1);
    reg.template write<Fmc::sdtr::Trp>(1);
    reg.template write<Fmc::sdtr::Twr>(1);
    reg.template write<Fmc::sdtr::Txsr>(7);
  });

  send_command(fmc_regs, Fmc::CommandMode::ClockConfigEnable, 0, 0);
  SysTick::getInstance().delay(1);
  send_command(fmc_regs, Fmc::CommandMode::PrechargeAll, 0, 0);
  send_command(fmc_regs, Fmc::CommandMode::AutoRefresh, 7, 0);
  send_command(fmc_regs, Fmc::CommandMode::LoadModeRegister, 0, 0x220);

  fmc_regs.get_register<Fmc::SdramRtr>().read_modify_write(
      [](auto reg) { reg.template write<Fmc::sdrtr::Count>(0x67e); });
}

void Sdram::send_command(Fmc::RegBank& fmc_regs, Fmc::CommandMode command, uint8_t autorefresh_num,
                         uint16_t mode_reg) noexcept {
  fmc_regs.get_register<Fmc::SdramCmr>().write([=](auto reg) {
    reg.template write<Fmc::sdcmr::Mode>(command);
    reg.template write<Fmc::sdcmr::Ctb1>(true);
    reg.template write<Fmc::sdcmr::Ctb2>(false);
    reg.template write<Fmc::sdcmr::Nrfs>(autorefresh_num);
    reg.template write<Fmc::sdcmr::Mrd>(mode_reg);
  });

  constexpr static uint32_t TIMEOUT_MS = 100;
  uint32_t ticks = 0;
  while (fmc_regs.get_register<Fmc::SdramSr>().read().read<Fmc::sdsr::Busy>()) {
    SysTick::getInstance().delay(1);
    if (++ticks >= TIMEOUT_MS) {
      break;
    }
  }

  DITTO_VERIFY(ticks < TIMEOUT_MS);
  DITTO_VERIFY(!fmc_regs.get_register<Fmc::SdramSr>().read().read<Fmc::sdsr::Re>());
}

void Sdram::configure_pins(Rcc::RegBank& rcc_regs) noexcept {
  // Configure pins
  rcc_regs.get_register<Rcc::Ahb1Enr>().read_modify_write([](auto reg) {
    // Enable gpio banks required for SDRAM
    reg.template write<Rcc::GpioCClkEn>(true);
    reg.template write<Rcc::GpioDClkEn>(true);
    reg.template write<Rcc::GpioEClkEn>(true);
    reg.template write<Rcc::GpioFClkEn>(true);
    reg.template write<Rcc::GpioGClkEn>(true);
    reg.template write<Rcc::GpioHClkEn>(true);
  });

  save_pin(Pin::D0, GpioBankId::D, GpioPinNumber{14});
  save_pin(Pin::D1, GpioBankId::D, GpioPinNumber{15});
  save_pin(Pin::D2, GpioBankId::D, GpioPinNumber{0});
  save_pin(Pin::D3, GpioBankId::D, GpioPinNumber{1});
  save_pin(Pin::D4, GpioBankId::E, GpioPinNumber{7});
  save_pin(Pin::D5, GpioBankId::E, GpioPinNumber{8});
  save_pin(Pin::D6, GpioBankId::E, GpioPinNumber{9});
  save_pin(Pin::D7, GpioBankId::E, GpioPinNumber{10});
  save_pin(Pin::D8, GpioBankId::E, GpioPinNumber{11});
  save_pin(Pin::D9, GpioBankId::E, GpioPinNumber{12});
  save_pin(Pin::D10, GpioBankId::E, GpioPinNumber{13});
  save_pin(Pin::D11, GpioBankId::E, GpioPinNumber{14});
  save_pin(Pin::D12, GpioBankId::E, GpioPinNumber{15});
  save_pin(Pin::D13, GpioBankId::D, GpioPinNumber{8});
  save_pin(Pin::D14, GpioBankId::D, GpioPinNumber{9});
  save_pin(Pin::D15, GpioBankId::D, GpioPinNumber{10});
  save_pin(Pin::A0, GpioBankId::F, GpioPinNumber{0});
  save_pin(Pin::A1, GpioBankId::F, GpioPinNumber{1});
  save_pin(Pin::A2, GpioBankId::F, GpioPinNumber{2});
  save_pin(Pin::A3, GpioBankId::F, GpioPinNumber{3});
  save_pin(Pin::A4, GpioBankId::F, GpioPinNumber{4});
  save_pin(Pin::A5, GpioBankId::F, GpioPinNumber{5});
  save_pin(Pin::A6, GpioBankId::F, GpioPinNumber{12});
  save_pin(Pin::A7, GpioBankId::F, GpioPinNumber{13});
  save_pin(Pin::A8, GpioBankId::F, GpioPinNumber{14});
  save_pin(Pin::A9, GpioBankId::F, GpioPinNumber{15});
  save_pin(Pin::A10, GpioBankId::G, GpioPinNumber{0});
  save_pin(Pin::A11, GpioBankId::G, GpioPinNumber{1});
  save_pin(Pin::BA0, GpioBankId::G, GpioPinNumber{4});
  save_pin(Pin::BA0, GpioBankId::G, GpioPinNumber{5});
  save_pin(Pin::SDNWE, GpioBankId::H, GpioPinNumber{5});
  save_pin(Pin::SDNCAS, GpioBankId::G, GpioPinNumber{15});
  save_pin(Pin::SDNRAS, GpioBankId::F, GpioPinNumber{11});
  save_pin(Pin::SDNE0, GpioBankId::H, GpioPinNumber{3});
  save_pin(Pin::SDCKE0, GpioBankId::C, GpioPinNumber{3});
  save_pin(Pin::SDCLK, GpioBankId::G, GpioPinNumber{8});
  save_pin(Pin::NBL0, GpioBankId::E, GpioPinNumber{0});
  save_pin(Pin::NBL1, GpioBankId::E, GpioPinNumber{1});
}

void Sdram::save_pin(Pin pin, GpioBankId bank, GpioPinNumber pin_number) noexcept {
  auto af_pin = get_gpio_bank(bank).try_get_as_alternate_function(pin_number, GpioAltFunc::AF12_FMC_SDMMC1_OTG2FS);
  DITTO_VERIFY(af_pin);
  af_pin.set_output_speed(GpioOutputSpeed::VeryHigh);
  af_pin.set_output_type(GpioOutputType::PushPull);
  m_gpio_pins[static_cast<size_t>(pin)] = std::move(af_pin);
}

}  // namespace Hw
