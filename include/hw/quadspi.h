#pragma once

#include "hw/register.h"

namespace Hw::QuadSpi {

enum class SelectedFlash { Mem1 = 0, Mem2 };

enum class MatchMode { And = 0, Or = 1 };

enum class DeviceSpiClockMode {
  Mode0 = 0,
  Mode3 = 1,
};

enum class Instruction : std::uint8_t {};

enum class InstructionMode {
  None = 0,
  SingleLine,
  DualLine,
  QuadLine,
};

enum class AddressMode {
  None = 0,
  SingleLine,
  DualLine,
  QuadLine,
};

enum class AddressSize {
  Bits8 = 0,
  Bits16,
  Bits24,
  Bits32,
};

enum class AlternateBytesMode {
  None = 0,
  SingleLine,
  DualLine,
  QuadLine,
};

enum class AlternateBytesSize {
  Bits8 = 0,
  Bits16,
  Bits24,
  Bits32,
};

enum class DataMode {
  None = 0,
  SingleLine,
  DualLine,
  QuadLine,
};

enum class FunctionalMode {
  IndirectWrite = 0,
  IndirectRead,
  AutoPolling,
  MemMapped,
};

namespace cr {
using Enable = FieldDesc<std::uint32_t, bool, RegOffset{0}, RegNumBits{1}>;
using Abort = FieldDesc<std::uint32_t, bool, RegOffset{1}, RegNumBits{1}>;
using DmaEn = FieldDesc<std::uint32_t, bool, RegOffset{2}, RegNumBits{1}>;
using TimeoutCounterEn = FieldDesc<std::uint32_t, bool, RegOffset{3}, RegNumBits{1}>;
using SampleShift = FieldDesc<std::uint32_t, bool, RegOffset{4}, RegNumBits{1}>;
using DualFlashMode = FieldDesc<std::uint32_t, bool, RegOffset{6}, RegNumBits{1}>;
using FlashSelection = FieldDesc<std::uint32_t, SelectedFlash, RegOffset{7}, RegNumBits{1}>;
using FifoThLvl = FieldDesc<std::uint32_t, uint8_t, RegOffset{8}, RegNumBits{5}>;
using TxErrIntEn = FieldDesc<std::uint32_t, bool, RegOffset{16}, RegNumBits{1}>;
using TxCompleteIntEn = FieldDesc<std::uint32_t, bool, RegOffset{17}, RegNumBits{1}>;
using FifoThresholdIntEn = FieldDesc<std::uint32_t, bool, RegOffset{18}, RegNumBits{1}>;
using StatusMatchIntEn = FieldDesc<std::uint32_t, bool, RegOffset{19}, RegNumBits{1}>;
using TimeoutIntEn = FieldDesc<std::uint32_t, bool, RegOffset{20}, RegNumBits{1}>;
using AutomaticPollModeStop = FieldDesc<std::uint32_t, bool, RegOffset{22}, RegNumBits{1}>;
using PollingMatchMode = FieldDesc<std::uint32_t, MatchMode, RegOffset{23}, RegNumBits{1}>;
using Prescaler = FieldDesc<std::uint32_t, uint8_t, RegOffset{24}, RegNumBits{8}>;
}  // namespace cr

using ControlReg = Register<std::uint32_t, BlockOffset{0x00}, cr::Enable, cr::Abort, cr::DmaEn, cr::TimeoutCounterEn,
                            cr::SampleShift, cr::DualFlashMode, cr::FlashSelection, cr::FifoThLvl, cr::TxErrIntEn,
                            cr::TxCompleteIntEn, cr::FifoThresholdIntEn, cr::StatusMatchIntEn, cr::TimeoutIntEn,
                            cr::AutomaticPollModeStop, cr::PollingMatchMode, cr::Prescaler>;

namespace dcr {
using ClockMode = FieldDesc<std::uint32_t, DeviceSpiClockMode, RegOffset{0}, RegNumBits{1}>;
using ChipSelectHighTime = FieldDesc<std::uint32_t, std::uint8_t, RegOffset{8}, RegNumBits{3}>;
using FlashSize = FieldDesc<std::uint32_t, std::uint8_t, RegOffset{16}, RegNumBits{5}>;
}  // namespace dcr

using DeviceConfigReg =
    Register<std::uint32_t, BlockOffset{0x04}, dcr::ClockMode, dcr::ChipSelectHighTime, dcr::FlashSize>;

namespace sr {
using TxError = FieldDesc<std::uint32_t, bool, RegOffset{0}, RegNumBits{1}>;
using TxComplete = FieldDesc<std::uint32_t, bool, RegOffset{1}, RegNumBits{1}>;
using FifoThreshold = FieldDesc<std::uint32_t, bool, RegOffset{2}, RegNumBits{1}>;
using StatusMatch = FieldDesc<std::uint32_t, bool, RegOffset{3}, RegNumBits{1}>;
using Timeout = FieldDesc<std::uint32_t, bool, RegOffset{4}, RegNumBits{1}>;
using Busy = FieldDesc<std::uint32_t, bool, RegOffset{5}, RegNumBits{1}>;
using FifoLevel = FieldDesc<std::uint32_t, uint8_t, RegOffset{8}, RegNumBits{6}>;
}  // namespace sr

using StatusReg = Register<std::uint32_t, BlockOffset{0x08}, sr::TxError, sr::TxComplete, sr::FifoThreshold,
                           sr::StatusMatch, sr::Timeout, sr::Busy, sr::FifoLevel>;

namespace fcr {
using TxError = FieldDesc<std::uint32_t, bool, RegOffset{0}, RegNumBits{1}>;
using TxComplete = FieldDesc<std::uint32_t, bool, RegOffset{1}, RegNumBits{1}>;
using StatusMatch = FieldDesc<std::uint32_t, bool, RegOffset{3}, RegNumBits{1}>;
using Timeout = FieldDesc<std::uint32_t, bool, RegOffset{4}, RegNumBits{1}>;
}  // namespace fcr

using FlagClearReg =
    Register<std::uint32_t, BlockOffset{0x0C}, fcr::TxError, fcr::TxComplete, fcr::StatusMatch, fcr::Timeout>;

namespace dlr {
using DataLength = FieldDesc<std::uint32_t, std::uint32_t, RegOffset{0}, RegNumBits{32}>;
}

using DataLengthReg = Register<std::uint32_t, BlockOffset{0x10}, dlr::DataLength>;

namespace ccr {
using Instruction = FieldDesc<std::uint32_t, Instruction, RegOffset{0}, RegNumBits{8}>;
using InstructionMode = FieldDesc<std::uint32_t, InstructionMode, RegOffset{8}, RegNumBits{2}>;
using AddressMode = FieldDesc<std::uint32_t, AddressMode, RegOffset{10}, RegNumBits{2}>;
using AddressSize = FieldDesc<std::uint32_t, AddressSize, RegOffset{12}, RegNumBits{2}>;
using AlternateBytesMode = FieldDesc<std::uint32_t, AlternateBytesMode, RegOffset{14}, RegNumBits{2}>;
using AlternateBytesSize = FieldDesc<std::uint32_t, AlternateBytesSize, RegOffset{16}, RegNumBits{2}>;
using NumDummyCycles = FieldDesc<std::uint32_t, std::uint8_t, RegOffset{18}, RegNumBits{5}>;
using DataMode = FieldDesc<std::uint32_t, DataMode, RegOffset{24}, RegNumBits{2}>;
using FunctionalMode = FieldDesc<std::uint32_t, FunctionalMode, RegOffset{26}, RegNumBits{2}>;
using SendInstructionOnce = FieldDesc<std::uint32_t, bool, RegOffset{28}, RegNumBits{1}>;
using DdrHold = FieldDesc<std::uint32_t, bool, RegOffset{30}, RegNumBits{1}>;
using DdrMode = FieldDesc<std::uint32_t, bool, RegOffset{31}, RegNumBits{1}>;
}  // namespace ccr

using CommConfigReg =
    Register<std::uint32_t, BlockOffset{0x14}, ccr::Instruction, ccr::InstructionMode, ccr::AddressMode,
             ccr::AddressSize, ccr::AlternateBytesMode, ccr::AlternateBytesSize, ccr::NumDummyCycles, ccr::DataMode,
             ccr::FunctionalMode, ccr::SendInstructionOnce, ccr::DdrHold, ccr::DdrMode>;

namespace ar {
using Address = FieldDesc<std::uint32_t, std::uint32_t, RegOffset{0}, RegNumBits{32}>;
}
using AddressReg = Register<std::uint32_t, BlockOffset{0x18}, ar::Address>;

namespace abr {
using AlternateBytes = FieldDesc<std::uint32_t, std::uint32_t, RegOffset{0}, RegNumBits{32}>;
}
using AlternateBytesReg = Register<std::uint32_t, BlockOffset{0x1C}, abr::AlternateBytes>;

namespace dr {
using Data32 = FieldDesc<std::uint32_t, std::uint32_t, RegOffset{0}, RegNumBits{32}>;
using Data16 = FieldDesc<std::uint16_t, std::uint16_t, RegOffset{0}, RegNumBits{16}>;
using Data8 = FieldDesc<std::uint8_t, std::uint8_t, RegOffset{0}, RegNumBits{8}>;
}  // namespace dr

using Data32Reg = Register<std::uint32_t, BlockOffset{0x20}, dr::Data32>;
using Data16Reg = Register<std::uint16_t, BlockOffset{0x20}, dr::Data16>;
using Data8Reg = Register<std::uint8_t, BlockOffset{0x20}, dr::Data8>;

namespace psmr {
using Mask = FieldDesc<std::uint32_t, std::uint32_t, RegOffset{0}, RegNumBits{32}>;
}
using PollingStatusMaskReg = Register<std::uint32_t, BlockOffset{0x24}, psmr::Mask>;

namespace psmar {
using Match = FieldDesc<std::uint32_t, std::uint32_t, RegOffset{0}, RegNumBits{32}>;
}
using PollingStatusMatchReg = Register<std::uint32_t, BlockOffset{0x28}, psmar::Match>;

namespace pir {
using Interval = FieldDesc<std::uint32_t, std::uint16_t, RegOffset{0}, RegNumBits{16}>;
}
using PollingIntervalReg = Register<std::uint32_t, BlockOffset{0x2C}, pir::Interval>;

namespace lptr {
using Timeout = FieldDesc<std::uint32_t, std::uint16_t, RegOffset{0}, RegNumBits{16}>;
}
using LowPowerTimeoutReg = Register<std::uint32_t, BlockOffset{0x30}, lptr::Timeout>;

using RegBank = RegisterBank<ControlReg, DeviceConfigReg, StatusReg, FlagClearReg, DataLengthReg, CommConfigReg,
                             AddressReg, AlternateBytesReg, Data8Reg, Data16Reg, Data32Reg, PollingStatusMaskReg,
                             PollingStatusMatchReg, PollingIntervalReg, LowPowerTimeoutReg>;

}  // namespace Hw::QuadSpi
