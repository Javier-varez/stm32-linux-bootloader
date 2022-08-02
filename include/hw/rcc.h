#pragma once

#include "hw/register.h"

namespace Hw {

class Rcc {
 public:
  explicit Rcc(MmappedRegs mmapped_regs) noexcept;

  Rcc(const Rcc&) = delete;
  Rcc& operator=(const Rcc&) = delete;
  Rcc(Rcc&&) = delete;
  Rcc& operator=(Rcc&&) = delete;

  void enable_gpio_bank_I() noexcept;

 private:
  MmappedRegs m_regs;
};

}  // namespace Hw
