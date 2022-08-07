#pragma once

#include <array>

#include "hw/gpio.h"
#include "hw/quadspi.h"
#include "hw/rcc.h"

namespace Hw {

class QspiFlash {
 public:
  QspiFlash() noexcept = default;

  void init(Rcc::RegBank&, QuadSpi::RegBank&) noexcept;

 private:
  AltFuncGpioPin m_ncs;
  AltFuncGpioPin m_clk;
  std::array<AltFuncGpioPin, 4> m_data{};

  void configure_pins(Rcc::RegBank&) noexcept;
  void configure_quadspi(Rcc::RegBank&, QuadSpi::RegBank&) noexcept;
};

}  // namespace Hw
