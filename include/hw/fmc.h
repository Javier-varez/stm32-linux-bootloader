#pragma once

#include "hw/register.h"

namespace Hw::Fmc {

enum class NumberOfColumnAddrBits {
  Bits8 = 0,
  Bits9 = 1,
  Bits10 = 2,
  Bits11 = 3,
};
enum class NumberOfRowAddrBits {
  Bits11 = 0,
  Bits12 = 1,
  Bits13 = 2,
};

enum class MemBusDataWidth {
  W8 = 0,
  W16 = 1,
  W32 = 2,
};

enum class NumInternalBanks {
  Two = 0,
  Four = 1,
};

enum class CasLatency {
  Cycles1 = 1,
  Cycles2 = 2,
  Cycles3 = 3,
};

enum class SdclkConfig { Disable = 0, HclkDiv2 = 2, HclkDiv3 = 3 };

enum class ReadPipe {
  NoHclkDelay = 0,
  HclkDelay1Cycle = 1,
  HclkDelay2Cycles = 2,
};

enum class CommandMode {
  Normal = 0,
  ClockConfigEnable = 1,
  PrechargeAll = 2,
  AutoRefresh = 3,
  LoadModeRegister = 4,
  SelfRefresh = 5,
  PowerDown = 6,
};

enum class StatusMode {
  Normal = 0,
  SelfRefresh = 1,
  PowerDown = 2,
};

namespace sdcr {
using Rpipe = FieldDesc<std::uint32_t, ReadPipe, RegOffset{13}, RegNumBits{2}>;
using Rburst = FieldDesc<std::uint32_t, bool, RegOffset{12}, RegNumBits{1}>;
using Sdclk = FieldDesc<std::uint32_t, SdclkConfig, RegOffset{10}, RegNumBits{2}>;
using Wp = FieldDesc<std::uint32_t, bool, RegOffset{9}, RegNumBits{1}>;
using Cas = FieldDesc<std::uint32_t, CasLatency, RegOffset{7}, RegNumBits{2}>;
using Nb = FieldDesc<std::uint32_t, NumInternalBanks, RegOffset{6}, RegNumBits{1}>;
using Mwid = FieldDesc<std::uint32_t, MemBusDataWidth, RegOffset{4}, RegNumBits{2}>;
using Nr = FieldDesc<std::uint32_t, NumberOfRowAddrBits, RegOffset{2}, RegNumBits{2}>;
using Nc = FieldDesc<std::uint32_t, NumberOfColumnAddrBits, RegOffset{0}, RegNumBits{2}>;
}  // namespace sdcr
using SdramCr1 = Register<std::uint32_t, BlockOffset{0x140}, sdcr::Rpipe, sdcr::Rburst, sdcr::Sdclk, sdcr::Wp,
                          sdcr::Cas, sdcr::Nb, sdcr::Mwid, sdcr::Nc, sdcr::Nr>;
using SdramCr2 = Register<std::uint32_t, BlockOffset{0x144}, sdcr::Rpipe, sdcr::Rburst, sdcr::Sdclk, sdcr::Wp,
                          sdcr::Cas, sdcr::Nb, sdcr::Mwid, sdcr::Nc, sdcr::Nr>;

namespace sdtr {
using Trcd = FieldDesc<std::uint32_t, std::uint8_t, RegOffset{24}, RegNumBits{4}>;
using Trp = FieldDesc<std::uint32_t, std::uint8_t, RegOffset{20}, RegNumBits{4}>;
using Twr = FieldDesc<std::uint32_t, std::uint8_t, RegOffset{16}, RegNumBits{4}>;
using Trc = FieldDesc<std::uint32_t, std::uint8_t, RegOffset{12}, RegNumBits{4}>;
using Tras = FieldDesc<std::uint32_t, std::uint8_t, RegOffset{8}, RegNumBits{4}>;
using Txsr = FieldDesc<std::uint32_t, std::uint8_t, RegOffset{4}, RegNumBits{4}>;
using Tmrd = FieldDesc<std::uint32_t, std::uint8_t, RegOffset{0}, RegNumBits{4}>;
}  // namespace sdtr
using SdramTr1 = Register<std::uint32_t, BlockOffset{0x148}, sdtr::Trcd, sdtr::Trp, sdtr::Twr, sdtr::Trc, sdtr::Tras,
                          sdtr::Txsr, sdtr::Tmrd>;
using SdramTr2 = Register<std::uint32_t, BlockOffset{0x14C}, sdtr::Trcd, sdtr::Trp, sdtr::Twr, sdtr::Trc, sdtr::Tras,
                          sdtr::Txsr, sdtr::Tmrd>;

namespace sdcmr {
using Mode = FieldDesc<std::uint32_t, CommandMode, RegOffset{0}, RegNumBits{3}>;
using Ctb2 = FieldDesc<std::uint32_t, bool, RegOffset{3}, RegNumBits{1}>;
using Ctb1 = FieldDesc<std::uint32_t, bool, RegOffset{4}, RegNumBits{1}>;
using Nrfs = FieldDesc<std::uint32_t, std::uint8_t, RegOffset{5}, RegNumBits{4}>;
using Mrd = FieldDesc<std::uint32_t, std::uint8_t, RegOffset{9}, RegNumBits{13}>;
}  // namespace sdcmr

using SdramCmr =
    Register<std::uint32_t, BlockOffset{0x150}, sdcmr::Mode, sdcmr::Nrfs, sdcmr::Ctb1, sdcmr::Ctb2, sdcmr::Mrd>;

namespace sdrtr {
using Cre = FieldDesc<std::uint32_t, bool, RegOffset{0}, RegNumBits{1}>;
using Count = FieldDesc<std::uint32_t, std::uint16_t, RegOffset{1}, RegNumBits{13}>;
using Reie = FieldDesc<std::uint32_t, bool, RegOffset{14}, RegNumBits{1}>;
}  // namespace sdrtr

using SdramRtr = Register<std::uint32_t, BlockOffset{0x154}, sdrtr::Cre, sdrtr::Count, sdrtr::Reie>;

namespace sdsr {
using Re = FieldDesc<std::uint32_t, bool, RegOffset{0}, RegNumBits{1}>;
using Modes1 = FieldDesc<std::uint32_t, StatusMode, RegOffset{1}, RegNumBits{2}>;
using Modes2 = FieldDesc<std::uint32_t, StatusMode, RegOffset{3}, RegNumBits{2}>;
using Busy = FieldDesc<std::uint32_t, bool, RegOffset{5}, RegNumBits{1}>;
}  // namespace sdsr
using SdramSr = Register<std::uint32_t, BlockOffset{0x158}, sdsr::Busy, sdsr::Modes1, sdsr::Modes2, sdsr::Re>;

using RegBank = RegisterBank<SdramSr, SdramRtr, SdramCmr, SdramTr1, SdramTr2, SdramCr1, SdramCr2>;

}  // namespace Hw::Fmc
