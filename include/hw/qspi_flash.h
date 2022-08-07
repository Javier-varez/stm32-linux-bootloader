#pragma once

#include <array>

#include "hw/gpio.h"
#include "hw/quadspi.h"
#include "hw/rcc.h"

namespace Hw {

class QspiFlash {
 public:
  QspiFlash(Rcc::RegBank& rcc_regs, QuadSpi::RegBank& quadspi_regs) noexcept
      : m_rcc_regs(rcc_regs), m_quadspi_regs(quadspi_regs) {}

  void init() noexcept;

 private:
  Rcc::RegBank& m_rcc_regs;
  QuadSpi::RegBank& m_quadspi_regs;
  AltFuncGpioPin m_ncs;
  AltFuncGpioPin m_clk;
  std::array<AltFuncGpioPin, 4> m_data{};

  void configure_pins() noexcept;
  void configure_quadspi() noexcept;
};

}  // namespace Hw
