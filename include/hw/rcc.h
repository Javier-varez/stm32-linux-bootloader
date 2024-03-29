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

enum class Clock48MSrc {
  PLL = 0,
  PLLSAI = 1,
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

using GpioAClkEn = FieldDesc<std::uint32_t, bool, RegOffset{0}, RegNumBits{1}>;
using GpioBClkEn = FieldDesc<std::uint32_t, bool, RegOffset{1}, RegNumBits{1}>;
using GpioCClkEn = FieldDesc<std::uint32_t, bool, RegOffset{2}, RegNumBits{1}>;
using GpioDClkEn = FieldDesc<std::uint32_t, bool, RegOffset{3}, RegNumBits{1}>;
using GpioEClkEn = FieldDesc<std::uint32_t, bool, RegOffset{4}, RegNumBits{1}>;
using GpioFClkEn = FieldDesc<std::uint32_t, bool, RegOffset{5}, RegNumBits{1}>;
using GpioGClkEn = FieldDesc<std::uint32_t, bool, RegOffset{6}, RegNumBits{1}>;
using GpioHClkEn = FieldDesc<std::uint32_t, bool, RegOffset{7}, RegNumBits{1}>;
using GpioIClkEn = FieldDesc<std::uint32_t, bool, RegOffset{8}, RegNumBits{1}>;
using GpioJClkEn = FieldDesc<std::uint32_t, bool, RegOffset{9}, RegNumBits{1}>;
using GpioKClkEn = FieldDesc<std::uint32_t, bool, RegOffset{10}, RegNumBits{1}>;
using Ahb1Enr = Register<std::uint32_t, BlockOffset{0x30u}, GpioAClkEn, GpioBClkEn, GpioCClkEn, GpioDClkEn, GpioEClkEn,
                         GpioFClkEn, GpioGClkEn, GpioHClkEn, GpioIClkEn, GpioJClkEn, GpioKClkEn>;

using QspiRst = FieldDesc<std::uint32_t, bool, RegOffset{1}, RegNumBits{1}>;
using FmcRst = FieldDesc<std::uint32_t, bool, RegOffset{0}, RegNumBits{1}>;
using Ahb3Rst = Register<std::uint32_t, BlockOffset{0x18u}, QspiRst, FmcRst>;

using QspiEn = FieldDesc<std::uint32_t, bool, RegOffset{1}, RegNumBits{1}>;
using FmcEn = FieldDesc<std::uint32_t, bool, RegOffset{0}, RegNumBits{1}>;
using Ahb3Enr = Register<std::uint32_t, BlockOffset{0x38u}, QspiEn, FmcEn>;

using PwrEn = FieldDesc<std::uint32_t, bool, RegOffset{28}, RegNumBits{1}>;
using Apb1Enr = Register<std::uint32_t, BlockOffset{0x40u}, PwrEn>;

using Uart1En = FieldDesc<std::uint32_t, bool, RegOffset{4}, RegNumBits{1}>;
using Apb2Enr = Register<std::uint32_t, BlockOffset{0x44u}, Uart1En>;

using Clock48MSelField = FieldDesc<std::uint32_t, Clock48MSrc, RegOffset{27}, RegNumBits{1}>;
using DedicatedClocksConfigReg = Register<std::uint32_t, BlockOffset{0x90u}, Clock48MSelField>;

using RegBank = RegisterBank<ControlReg, PllReg, ClockConfigReg, Ahb3Rst, Ahb1Enr, Ahb3Enr, Apb1Enr, Apb2Enr,
                             DedicatedClocksConfigReg>;

}  // namespace Hw::Rcc
