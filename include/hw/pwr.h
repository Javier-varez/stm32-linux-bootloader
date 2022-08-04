#pragma once

#include "hw/register.h"

namespace Hw::Pwr {

enum class VoltageScaling {
  MODE_1 = 3,
  MODE_2 = 2,
  MODE_3 = 1,
};

using OverDriveSwEnField = FieldDesc<std::uint32_t, bool, RegOffset{17}, RegNumBits{1}>;
using OverDriveEnField = FieldDesc<std::uint32_t, bool, RegOffset{16}, RegNumBits{1}>;
using VoltageScalingOutputField = FieldDesc<std::uint32_t, VoltageScaling, RegOffset{14}, RegNumBits{2}>;
using ControlReg1 =
    Register<std::uint32_t, BlockOffset{0x00u}, OverDriveSwEnField, OverDriveEnField, VoltageScalingOutputField>;

using OverDriveSwRdyField = FieldDesc<std::uint32_t, bool, RegOffset{17}, RegNumBits{1}>;
using OverDriveRdyField = FieldDesc<std::uint32_t, bool, RegOffset{16}, RegNumBits{1}>;
using VoltageScalingRdyField = FieldDesc<std::uint32_t, bool, RegOffset{14}, RegNumBits{1}>;
using ControlStatusReg1 =
    Register<std::uint32_t, BlockOffset{0x04u}, OverDriveSwRdyField, OverDriveRdyField, VoltageScalingRdyField>;

using RegBank = RegisterBank<ControlReg1, ControlStatusReg1>;

}  // namespace Hw::Pwr
