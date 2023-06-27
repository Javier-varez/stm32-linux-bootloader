#pragma once

#include "hw/register.h"

namespace Hw::Uart {
using TxEnField = FieldDesc<std::uint32_t, bool, RegOffset{4}, RegNumBits{1}>;
using RxEnField = FieldDesc<std::uint32_t, bool, RegOffset{3}, RegNumBits{1}>;
using UartEnField = FieldDesc<std::uint32_t, bool, RegOffset{0}, RegNumBits{1}>;
using ControlReg1 = Register<std::uint32_t, BlockOffset{0x00u}, UartEnField, TxEnField, RxEnField>;

using BaudrateField = FieldDesc<std::uint32_t, uint32_t, RegOffset{0}, RegNumBits{16}>;
using BaudrateReg = Register<std::uint32_t, BlockOffset{0x0cu}, BaudrateField>;

using TxEmptyField = FieldDesc<std::uint32_t, bool, RegOffset{7}, RegNumBits{1}>;
using InterruptAndStatusReg = Register<std::uint32_t, BlockOffset{0x1cu}, TxEmptyField>;

using TransmitDataField = FieldDesc<std::uint32_t, uint8_t, RegOffset{0}, RegNumBits{8}>;
using TransmitDataReg = Register<std::uint32_t, BlockOffset{0x28u}, TransmitDataField>;

using RegBank = RegisterBank<ControlReg1, BaudrateReg, InterruptAndStatusReg, TransmitDataReg>;
}  // namespace Hw::Uart
