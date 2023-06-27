#pragma once

#include <array>
#include <span>

#include "ditto/result.h"
#include "hw/gpio.h"
#include "hw/quadspi.h"
#include "hw/rcc.h"

namespace Hw {

class QspiFlash {
 public:
  enum class Error {
    TOO_MUCH_DATA = 0,
    UNALIGNED_ADDRESS = 1,
    PROTECTED_RANGE = 2,
    OPERATION_FAILED = 3,
    DEVICE_BUSY = 4,
  };

  QspiFlash(Rcc::RegBank& rcc_regs, QuadSpi::RegBank& quadspi_regs) noexcept
      : m_rcc_regs(rcc_regs), m_quadspi_regs(quadspi_regs) {}

  void init() noexcept;

  void enable_memory_mapped_mode() noexcept;

  Ditto::Result<void, Error> erase_subsector(const uint32_t address) noexcept;
  Ditto::Result<void, Error> write_page(const uint32_t address, std::span<const uint8_t> data) noexcept;
  void read_data(const uint32_t address, std::span<uint8_t> data) noexcept;

 private:
  Rcc::RegBank& m_rcc_regs;
  QuadSpi::RegBank& m_quadspi_regs;
  AltFuncGpioPin m_ncs;
  AltFuncGpioPin m_clk;
  std::array<AltFuncGpioPin, 4> m_data{};

  void configure_pins() noexcept;
  void configure_quadspi() noexcept;
  Ditto::Result<void, QspiFlash::Error> read_and_translate_flags_status_reg() noexcept;

  constexpr static size_t PAGE_SIZE = 256;
  constexpr static size_t SUBSECTOR_SIZE = 4 * 1024;
  constexpr static size_t SECTOR_SIZE = 64 * 1024;
  constexpr static size_t MEMORY_SIZE = 1024u * 1024u * 16u;

  static_assert(MEMORY_SIZE / PAGE_SIZE == 65536);
  static_assert(MEMORY_SIZE / SUBSECTOR_SIZE == 4096);
  static_assert(MEMORY_SIZE / SECTOR_SIZE == 256);
};

}  // namespace Hw
