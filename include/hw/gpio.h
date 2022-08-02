#pragma once

#include <atomic>
#include <cstdint>

#include "ditto/result.h"
#include "hw/register.h"

namespace Hw {

enum class GpioPinNumber : std::uint32_t {};

// TODO(javier-varez): Populate alternate functions
enum class GpioAltFunc : std::uint32_t {};

enum class GpioState {
  Low = 0,
  High,
};

class GpioBank {
 public:
  explicit GpioBank(MmappedRegs mmapped_regs, std::uint32_t num_pins) noexcept;

  GpioBank(const GpioBank&) = delete;
  GpioBank& operator=(const GpioBank&) = delete;
  GpioBank(GpioBank&&) = delete;
  GpioBank& operator=(GpioBank&&) = delete;

  [[nodiscard]] class InputGpioPin try_get_as_input(GpioPinNumber number) noexcept;
  [[nodiscard]] class OutputGpioPin try_get_as_output(GpioPinNumber number, GpioState initial_state) noexcept;
  [[nodiscard]] class AltFuncGpioPin try_get_as_alternate_function(GpioPinNumber number, GpioAltFunc alt_func) noexcept;

 private:
  MmappedRegs m_regs;
  std::uint32_t m_num_pins{0u};
  std::atomic_uint32_t m_pin_taken{};

  [[nodiscard]] GpioState get_pin_state(GpioPinNumber pin_number) const noexcept;
  void set_pin_state(GpioPinNumber pin_number, GpioState state) noexcept;
  void release(GpioPinNumber pin_number) noexcept;

  friend class GpioPin;
  friend class InputGpioPin;
  friend class OutputGpioPin;
  friend class AltFuncGpioPin;
};

class GpioPin {
 public:
  GpioPin() noexcept = default;
  GpioPin(GpioBank* bank, GpioPinNumber number) noexcept : m_bank{bank}, m_pin_number{number} {}

  GpioPin(const GpioPin&) = delete;
  GpioPin& operator=(const GpioPin&) = delete;

  GpioPin(GpioPin&& other) noexcept : m_bank{other.m_bank}, m_pin_number{other.m_pin_number} {}

  GpioPin& operator=(GpioPin&& other) noexcept {
    if (this != &other) {
      release();
      m_bank = other.m_bank;
      m_pin_number = other.m_pin_number;
      other.release();
    }
    return *this;
  }

  virtual ~GpioPin() noexcept { release(); }

  void release() noexcept {
    if (is_valid()) {
      m_bank->release(m_pin_number);
      m_bank = nullptr;
    }
  }

  [[nodiscard]] explicit operator bool() const noexcept { return m_bank != nullptr; }

  [[nodiscard]] bool is_valid() const noexcept { return m_bank != nullptr; }

 protected:
  GpioBank* m_bank{nullptr};
  GpioPinNumber m_pin_number{0u};
};

class InputGpioPin : public GpioPin {
 public:
  InputGpioPin() noexcept = default;
  InputGpioPin(GpioBank* bank, GpioPinNumber number) noexcept : GpioPin(bank, number) {}

  InputGpioPin(const InputGpioPin&) = delete;
  InputGpioPin& operator=(const InputGpioPin&) = delete;

  InputGpioPin(InputGpioPin&&) noexcept = default;
  InputGpioPin& operator=(InputGpioPin&&) noexcept = default;

  [[nodiscard]] GpioState get_state() const noexcept {
    DITTO_VERIFY(is_valid());
    return m_bank->get_pin_state(m_pin_number);
  }
};

class OutputGpioPin final : public GpioPin {
 public:
  OutputGpioPin() noexcept = default;
  OutputGpioPin(GpioBank* bank, GpioPinNumber number) noexcept : GpioPin(bank, number) {}

  OutputGpioPin(const OutputGpioPin&) = delete;
  OutputGpioPin& operator=(const OutputGpioPin&) = delete;

  OutputGpioPin(OutputGpioPin&&) noexcept = default;
  OutputGpioPin& operator=(OutputGpioPin&&) noexcept = default;

  [[nodiscard]] GpioState get_state() const noexcept {
    DITTO_VERIFY(is_valid());
    return m_bank->get_pin_state(m_pin_number);
  }

  void set_state(GpioState state) noexcept {
    DITTO_VERIFY(is_valid());
    return m_bank->set_pin_state(m_pin_number, state);
  }
};

class AltFuncGpioPin final : public GpioPin {
 public:
  AltFuncGpioPin() noexcept = default;
  AltFuncGpioPin(GpioBank* bank, GpioPinNumber number) noexcept : GpioPin(bank, number) {}

  AltFuncGpioPin(const AltFuncGpioPin&) = delete;
  AltFuncGpioPin& operator=(const AltFuncGpioPin&) = delete;

  AltFuncGpioPin(AltFuncGpioPin&&) noexcept = default;
  AltFuncGpioPin& operator=(AltFuncGpioPin&&) noexcept = default;

  [[nodiscard]] GpioState get_state() const noexcept {
    DITTO_VERIFY(is_valid());
    return m_bank->get_pin_state(m_pin_number);
  }
};

}  // namespace Hw
