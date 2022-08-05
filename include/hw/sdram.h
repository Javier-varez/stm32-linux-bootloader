#pragma once

#include <array>

#include "hw/fmc.h"
#include "hw/gpio.h"
#include "hw/rcc.h"

namespace Hw {

class Sdram {
 public:
  Sdram() noexcept = default;

  void init(Hw::Rcc::RegBank&, Fmc::RegBank&) noexcept;

 private:
  enum class Pin {
    A0 = 0,
    A1,
    A2,
    A3,
    A4,
    A5,
    A6,
    A7,
    A8,
    A9,
    A10,
    A11,
    D0,
    D1,
    D2,
    D3,
    D4,
    D5,
    D6,
    D7,
    D8,
    D9,
    D10,
    D11,
    D12,
    D13,
    D14,
    D15,
    BA0,
    BA1,
    SDNWE,
    SDNCAS,
    SDNRAS,
    SDNE0,
    SDCKE0,
    SDCLK,
    NBL0,
    NBL1,
    MAX_PINS
  };

  std::array<AltFuncGpioPin, static_cast<size_t>(Pin::MAX_PINS)> m_gpio_pins;

  void configure_pins(Rcc::RegBank& rcc_regs) noexcept;
  void configure_fmc(Rcc::RegBank& rcc_regs, Fmc::RegBank& fmc_regs) noexcept;
  void save_pin(Pin pin, GpioBankId bank, GpioPinNumber pin_number) noexcept;
  void send_command(Fmc::RegBank& fmc_regs, Fmc::CommandMode command, uint8_t autorefresh_num,
                    uint16_t mode_reg) noexcept;
};

}  // namespace Hw
