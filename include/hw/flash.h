#pragma once

#include "hw/register.h"

namespace Hw::Flash {

using LatencyField = FieldDesc<std::uint32_t, uint8_t, RegOffset{0}, RegNumBits{4}>;
using ArtEnField = FieldDesc<std::uint32_t, bool, RegOffset{9}, RegNumBits{1}>;
using PreFetchEnField = FieldDesc<std::uint32_t, bool, RegOffset{8}, RegNumBits{1}>;
using AccessCtrlReg = Register<std::uint32_t, BlockOffset{0x00u}, ArtEnField, PreFetchEnField, LatencyField>;

using RegBank = RegisterBank<AccessCtrlReg>;

}  // namespace Hw::Flash
