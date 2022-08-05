#pragma once

#include <atomic>
#include <cstdint>

#include "ditto/result.h"
#include "hw/register.h"

namespace Hw {

enum class GpioPinNumber : std::uint32_t {};

enum class GpioAltFunc : std::uint32_t {
  AF0_SYS,
  AF1_TIM1_2,
  AF2_TIM3_4_5,
  AF3_TIM8_9_10_11_LPTIM1_CEC,
  AF4_I2C1_2_3_4_CEC,
  AF5_SPI1_2_3_4_5_6,
  AF6_SPI3_SAI1,
  AF7_SPI2_3_USART1_2_3_UART5_SPDIFRX,
  AF8_SAI2_USART6_UART4_5_7_8_SPDIFRX,
  AF9_CAN1_2_TIM12_13_14_QUADSPI_LCD,
  AF10_SAI2_QUADSPI_OTG2HS_OTG1FS,
  AF11_ETH_OTG1FS,
  AF12_FMC_SDMMC1_OTG2FS,
  AF13_DCMI,
  AF14_LCD,
  AF15_SYS,
};

enum class GpioState {
  Low = 0,
  High,
};

enum class GpioOutputType { PushPull = 0, OpenDrain = 1 };
enum class GpioOutputSpeed { Low = 0, Medium = 1, High = 2, VeryHigh = 3 };
enum class GpioPullResistorConfig { Disabled = 0, PullUp = 1, PullDown = 2 };

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
  void set_pin_output_type(GpioPinNumber pin_number, GpioOutputType type) noexcept;
  void set_pin_output_speed(GpioPinNumber pin_number, GpioOutputSpeed speed) noexcept;
  void set_pin_pull_resistor_config(GpioPinNumber pin_number, GpioPullResistorConfig config) noexcept;

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
      other.m_bank = nullptr;
    }
    return *this;
  }

  virtual ~GpioPin() noexcept { release(); }

  inline void release() noexcept {
    if (is_valid()) {
      m_bank->release(m_pin_number);
      m_bank = nullptr;
    }
  }

  inline void set_pull_resistor_config(GpioPullResistorConfig config) noexcept {
    DITTO_VERIFY(is_valid());
    return m_bank->set_pin_pull_resistor_config(m_pin_number, config);
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

  [[nodiscard]] inline GpioState get_state() const noexcept {
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

  inline void set_state(GpioState state) noexcept {
    DITTO_VERIFY(is_valid());
    return m_bank->set_pin_state(m_pin_number, state);
  }

  inline void toggle_state() noexcept {
    if (get_state() == GpioState::High) {
      set_state(GpioState::Low);
    } else {
      set_state(GpioState::High);
    }
  }

  inline void set_output_type(GpioOutputType type) noexcept {
    DITTO_VERIFY(is_valid());
    return m_bank->set_pin_output_type(m_pin_number, type);
  }

  inline void set_output_speed(GpioOutputSpeed speed) noexcept {
    DITTO_VERIFY(is_valid());
    return m_bank->set_pin_output_speed(m_pin_number, speed);
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

  [[nodiscard]] inline GpioState get_state() const noexcept {
    DITTO_VERIFY(is_valid());
    return m_bank->get_pin_state(m_pin_number);
  }

  inline void set_output_type(GpioOutputType type) noexcept {
    DITTO_VERIFY(is_valid());
    return m_bank->set_pin_output_type(m_pin_number, type);
  }

  inline void set_output_speed(GpioOutputSpeed speed) noexcept {
    DITTO_VERIFY(is_valid());
    return m_bank->set_pin_output_speed(m_pin_number, speed);
  }
};

enum class GpioBankId { A, B, C, D, E, F, G, H, I, J, K };

GpioBank& get_gpio_bank(GpioBankId id);

}  // namespace Hw
