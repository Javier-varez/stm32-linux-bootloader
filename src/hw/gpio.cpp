#include "hw/gpio.h"

namespace Hw {

namespace gpio_regs {

enum class Mode {
  Input = 0,
  Output = 1,
  AltFunc = 2,
  Analog = 3,
};

enum class OutputType { PushPull = 0, OpenDrain = 1 };
enum class OutputSpeed { Low = 0, Medium = 1, High = 2, VeryHigh = 3 };
enum class PullResistorConfig { Disabled = 0, PullUp = 1, PullDown = 2 };

// Mode register
using ModeField = IndexedField<std::uint32_t, Mode, RegOffset{0}, RegNumBits{2}, NumFields{16}>;
using ModeReg = Register<std::uint32_t, BlockOffset{0x00u}, ModeField>;

// Output type register
using OutputTypeField = IndexedField<std::uint32_t, OutputType, RegOffset{0}, RegNumBits{1}, NumFields{16}>;
using OutputTypeReg = Register<std::uint32_t, BlockOffset{0x04}, OutputTypeField>;

// Output speed register
using OutputSpeedField = IndexedField<std::uint32_t, OutputSpeed, RegOffset{0}, RegNumBits{2}, NumFields{16}>;
using OutputSpeedReg = Register<std::uint32_t, BlockOffset{0x08}, OutputSpeedField>;

// Pull resistor register
using PuPdField = IndexedField<std::uint32_t, PullResistorConfig, RegOffset{0}, RegNumBits{2}, NumFields{16}>;
using PuPdReg = Register<std::uint32_t, BlockOffset{0x0C}, PuPdField>;

// Input data register
using InputField = IndexedField<std::uint32_t, GpioState, RegOffset{0}, RegNumBits{1}, NumFields{16}>;
using InputReg = Register<std::uint32_t, BlockOffset{0x10}, InputField>;

// Output data register
using OutputField = IndexedField<std::uint32_t, GpioState, RegOffset{0}, RegNumBits{1}, NumFields{16}>;
using OutputReg = Register<std::uint32_t, BlockOffset{0x14}, OutputField>;

// Bit Set/Reset register
using BitSetField = IndexedField<std::uint32_t, bool, RegOffset{0}, RegNumBits{1}, NumFields{16}>;
using BitResetField = IndexedField<std::uint32_t, bool, RegOffset{16}, RegNumBits{1}, NumFields{16}>;
using BitSetResetReg = Register<std::uint32_t, BlockOffset{0x18}, BitSetField, BitResetField>;

// Gpio lock register
using LockField = IndexedField<std::uint32_t, bool, RegOffset{0}, RegNumBits{1}, NumFields{16}>;
using LockKeyField = FieldDesc<std::uint32_t, bool, RegOffset{16}, RegNumBits{1}>;
using LockReg = Register<std::uint32_t, BlockOffset{0x1c}, LockField, LockKeyField>;

// Gpio alternate function low register
using AfLowField = IndexedField<std::uint32_t, GpioAltFunc, RegOffset{0}, RegNumBits{4}, NumFields{8}>;
using AfLowReg = Register<std::uint32_t, BlockOffset{0x20}, AfLowField>;
using AfHighField = IndexedField<std::uint32_t, GpioAltFunc, RegOffset{0}, RegNumBits{4}, NumFields{8}>;
using AfHighReg = Register<std::uint32_t, BlockOffset{0x24}, AfHighField>;

using GpioRegBank = RegisterBank<ModeReg, OutputTypeReg, OutputSpeedReg, PuPdReg, InputReg, OutputReg, BitSetResetReg,
                                 LockReg, AfLowReg, AfHighReg>;

[[nodiscard]] GpioRegBank get_reg_bank(MmappedRegs regs) { return GpioRegBank{regs}; }

}  // namespace gpio_regs

GpioBank::GpioBank(MmappedRegs mmapped_regs, std::uint32_t num_pins) noexcept
    : m_regs(mmapped_regs), m_num_pins{num_pins} {}

InputGpioPin GpioBank::try_get_as_input(const GpioPinNumber number) noexcept {
  if (static_cast<std::size_t>(number) >= m_num_pins) {
    return {};
  }

  const uint32_t mask = 1u << static_cast<std::size_t>(number);
  const bool taken = (m_pin_taken.fetch_or(mask) & mask) != 0;
  if (taken) {
    return {};
  }
  auto reg_bank = gpio_regs::get_reg_bank(m_regs);
  reg_bank.get_register<gpio_regs::ModeReg>().read_modify_write([number](auto reg) noexcept -> void {
    reg.template write<gpio_regs::ModeField>(RegIndex{static_cast<std::size_t>(number)}, gpio_regs::Mode::Input);
  });
  return InputGpioPin{this, number};
}

OutputGpioPin GpioBank::try_get_as_output(GpioPinNumber number, GpioState initial_state) noexcept {
  if (static_cast<std::size_t>(number) >= m_num_pins) {
    return {};
  }

  const uint32_t mask = 1u << static_cast<std::size_t>(number);
  const bool taken = (m_pin_taken.fetch_or(mask) & mask) != 0;
  if (taken) {
    return OutputGpioPin{};
  }
  auto reg_bank = gpio_regs::get_reg_bank(m_regs);

  reg_bank.get_register<gpio_regs::OutputReg>().read_modify_write([number, initial_state](auto reg) noexcept -> void {
    reg.template write<gpio_regs::OutputField>(RegIndex{static_cast<std::size_t>(number)}, initial_state);
  });

  reg_bank.get_register<gpio_regs::ModeReg>().read_modify_write([number](auto reg) noexcept -> void {
    reg.template write<gpio_regs::ModeField>(RegIndex{static_cast<std::size_t>(number)}, gpio_regs::Mode::Output);
  });

  return OutputGpioPin{this, number};
}

AltFuncGpioPin GpioBank::try_get_as_alternate_function(GpioPinNumber number, GpioAltFunc alt_func) noexcept {
  if (static_cast<std::size_t>(number) >= m_num_pins) {
    return {};
  }

  const uint32_t mask = 1u << static_cast<std::size_t>(number);
  const bool taken = (m_pin_taken.fetch_or(mask) & mask) != 0;
  if (taken) {
    return AltFuncGpioPin{};
  }

  auto reg_bank = gpio_regs::get_reg_bank(m_regs);

  constexpr static std::size_t NUM_PINS_PER_REG = 8;
  if (static_cast<std::size_t>(number) < NUM_PINS_PER_REG) {
    reg_bank.get_register<gpio_regs::AfLowReg>().read_modify_write([number, alt_func](auto reg) noexcept -> void {
      reg.template write<gpio_regs::AfLowField>(RegIndex{static_cast<std::size_t>(number)}, alt_func);
    });
  } else {
    reg_bank.get_register<gpio_regs::AfHighReg>().read_modify_write([number, alt_func](auto reg) noexcept -> void {
      reg.template write<gpio_regs::AfHighField>(RegIndex{static_cast<std::size_t>(number) - NUM_PINS_PER_REG},
                                                 alt_func);
    });
  }

  reg_bank.get_register<gpio_regs::ModeReg>().read_modify_write([number](auto reg) noexcept -> void {
    reg.template write<gpio_regs::ModeField>(RegIndex{static_cast<std::size_t>(number)}, gpio_regs::Mode::AltFunc);
  });

  return AltFuncGpioPin{this, number};
}

GpioState GpioBank::get_pin_state(GpioPinNumber number) const noexcept {
  auto reg_bank = gpio_regs::get_reg_bank(m_regs);
  auto reg = reg_bank.get_register<gpio_regs::InputReg>().read();
  return reg.read<gpio_regs::InputField>(RegIndex{static_cast<std::size_t>(number)});
}

void GpioBank::set_pin_state(GpioPinNumber number, GpioState state) noexcept {
  auto reg_bank = gpio_regs::get_reg_bank(m_regs);
  reg_bank.get_register<gpio_regs::BitSetResetReg>().write([state, number](auto reg) {
    if (state == GpioState::Low) {
      reg.template write<gpio_regs::BitResetField>(RegIndex{static_cast<std::size_t>(number)}, true);
    } else {
      reg.template write<gpio_regs::BitSetField>(RegIndex{static_cast<std::size_t>(number)}, true);
    }
  });
}

void GpioBank::release(GpioPinNumber number) noexcept {
  // Set pin as input
  auto bank = gpio_regs::get_reg_bank(m_regs);
  bank.get_register<gpio_regs::ModeReg>().read_modify_write([number](auto reg) {
    reg.template write<gpio_regs::ModeField>(RegIndex{static_cast<std::size_t>(number)}, gpio_regs::Mode::Input);
  });

  m_pin_taken.fetch_or(1u << static_cast<std::size_t>(number));
}

}  // namespace Hw
