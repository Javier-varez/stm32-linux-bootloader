#pragma once

#include "hw/register.h"

namespace Hw::Rcc {

enum class PllSource { HSI = 0, HSE = 1 };
enum class PllQSetting {
  DIV_2 = 2,
  DIV_3 = 3,
  DIV_4 = 4,
  DIV_5 = 5,
  DIV_6 = 6,
  DIV_7 = 7,
  DIV_8 = 8,
  DIV_9 = 9,
  DIV_10 = 10,
  DIV_11 = 11,
  DIV_12 = 12,
  DIV_13 = 13,
  DIV_14 = 14,
  DIV_15 = 15,
};

enum class PllPSetting {
  DIV_2 = 0,
  DIV_4 = 1,
  DIV_6 = 2,
  DIV_8 = 3,
};

enum class ApbPreSetting {
  DIV_2 = 4,
  DIV_4 = 5,
  DIV_8 = 6,
  DIV_16 = 7,
};

enum class AhbPreSetting {
  DIV_1 = 0,
  DIV_2 = 8,
  DIV_4 = 9,
  DIV_8 = 10,
  DIV_16 = 11,
  DIV_64 = 12,
  DIV_128 = 13,
  DIV_256 = 14,
  DIV_512 = 15,
};

enum class SystemClockSwitch {
  HSI = 0,
  HSE = 1,
  PLL = 2,
};

using PllRdyField = FieldDesc<std::uint32_t, bool, RegOffset{25}, RegNumBits{1}>;
using PllOnField = FieldDesc<std::uint32_t, bool, RegOffset{24}, RegNumBits{1}>;
using HseBypField = FieldDesc<std::uint32_t, bool, RegOffset{18}, RegNumBits{1}>;
using HseRdyField = FieldDesc<std::uint32_t, bool, RegOffset{17}, RegNumBits{1}>;
using HseOnField = FieldDesc<std::uint32_t, bool, RegOffset{16}, RegNumBits{1}>;
using ControlReg =
    Register<std::uint32_t, BlockOffset{0x00u}, PllRdyField, PllOnField, HseBypField, HseRdyField, HseOnField>;

using PllQField = FieldDesc<std::uint32_t, PllQSetting, RegOffset{24}, RegNumBits{4}>;
using PllSrcField = FieldDesc<std::uint32_t, PllSource, RegOffset{22}, RegNumBits{1}>;
using PllPField = FieldDesc<std::uint32_t, PllPSetting, RegOffset{16}, RegNumBits{2}>;
using PllNField = FieldDesc<std::uint32_t, uint16_t, RegOffset{6}, RegNumBits{9}>;
using PllMField = FieldDesc<std::uint32_t, uint16_t, RegOffset{0}, RegNumBits{6}>;
using PllReg = Register<std::uint32_t, BlockOffset{0x04u}, PllQField, PllSrcField, PllPField, PllNField, PllMField>;

using Apb2PreField = FieldDesc<std::uint32_t, ApbPreSetting, RegOffset{13}, RegNumBits{3}>;
using Apb1PreField = FieldDesc<std::uint32_t, ApbPreSetting, RegOffset{10}, RegNumBits{3}>;
using AhbPreField = FieldDesc<std::uint32_t, AhbPreSetting, RegOffset{4}, RegNumBits{4}>;
using SwitchStatusField = FieldDesc<std::uint32_t, SystemClockSwitch, RegOffset{2}, RegNumBits{2}>;
using SwitchField = FieldDesc<std::uint32_t, SystemClockSwitch, RegOffset{0}, RegNumBits{2}>;
using ClockConfigReg = Register<std::uint32_t, BlockOffset{0x08u}, Apb2PreField, Apb1PreField, AhbPreField,
                                SwitchStatusField, SwitchField>;

using GpioIClkEn = FieldDesc<std::uint32_t, bool, RegOffset{8}, RegNumBits{1}>;
using Ahb1Enr = Register<std::uint32_t, BlockOffset{0x30u}, GpioIClkEn>;

using PwrEn = FieldDesc<std::uint32_t, bool, RegOffset{28}, RegNumBits{1}>;
using Apb1Enr = Register<std::uint32_t, BlockOffset{0x40u}, PwrEn>;

using RegBank = RegisterBank<ControlReg, PllReg, ClockConfigReg, Ahb1Enr, Apb1Enr>;

}  // namespace Hw::Rcc
