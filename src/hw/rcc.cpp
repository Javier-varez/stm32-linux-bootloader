#include "hw/rcc.h"

namespace Hw {

namespace rcc_regs {

using GpioIClkEn = FieldDesc<std::uint32_t, bool, RegOffset{8}, RegNumBits{1}>;
using Ahb1Enr = Register<std::uint32_t, BlockOffset{0x30u}, GpioIClkEn>;

using RccRegBank = RegisterBank<Ahb1Enr>;

}  // namespace rcc_regs
//
[[nodiscard]] rcc_regs::RccRegBank get_reg_bank(MmappedRegs regs) { return rcc_regs::RccRegBank{regs}; }

Rcc::Rcc(MmappedRegs mmapped_regs) noexcept : m_regs(mmapped_regs) {}

void Rcc::enable_gpio_bank_I() noexcept {
  auto regs = get_reg_bank(m_regs);
  regs.get_register<rcc_regs::Ahb1Enr>().read_modify_write(
      [](auto reg) { reg.template write<rcc_regs::GpioIClkEn>(true); });
}

}  // namespace Hw
