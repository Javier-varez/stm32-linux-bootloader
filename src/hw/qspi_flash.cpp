#include "hw/qspi_flash.h"

#include "cortex_m_hal/systick.h"
#include "logger.h"

namespace Hw {

namespace {
constexpr static uint8_t NUM_DUMMY_CYCLES = 10;

enum class Command {
  ReadStatusReg = 0x05,
  WriteEnable = 0x06,
  ResetEnable = 0x66,
  ReadVolatileConfigReg = 0x85,
  WriteVolatileConfigReg = 0x81,
  ReadEnhancedVolatileConfigReg = 0x65,
  WriteEnhancedVolatileConfigReg = 0x61,
  ResetMemory = 0x99,
  ReadId = 0x9e,
  MultipleIoReadId = 0xaf,
  QuadIoFastRead = 0xeb,
};

void clear_flags(QuadSpi::RegBank& regs) noexcept {
  regs.get_register<QuadSpi::FlagClearReg>().write([=](auto reg) {
    reg.template write<QuadSpi::fcr::StatusMatch>(true);
    reg.template write<QuadSpi::fcr::Timeout>(true);
    reg.template write<QuadSpi::fcr::TxComplete>(true);
    reg.template write<QuadSpi::fcr::TxError>(true);
  });
}

void send_simple_command(QuadSpi::RegBank& regs, Command command) noexcept {
  clear_flags(regs);

  regs.get_register<QuadSpi::CommConfigReg>().write([=](auto reg) {
    reg.template write<QuadSpi::ccr::AddressMode>(QuadSpi::AddressMode::None);
    reg.template write<QuadSpi::ccr::AlternateBytesMode>(QuadSpi::AlternateBytesMode::None);
    reg.template write<QuadSpi::ccr::DataMode>(QuadSpi::DataMode::None);
    reg.template write<QuadSpi::ccr::Instruction>(static_cast<QuadSpi::Instruction>(command));
    reg.template write<QuadSpi::ccr::InstructionMode>(QuadSpi::InstructionMode::SingleLine);
    reg.template write<QuadSpi::ccr::DdrMode>(false);
    reg.template write<QuadSpi::ccr::DdrHold>(false);
    reg.template write<QuadSpi::ccr::NumDummyCycles>(0);
    reg.template write<QuadSpi::ccr::FunctionalMode>(QuadSpi::FunctionalMode::IndirectWrite);
  });

  // Wait until done
  while (regs.get_register<QuadSpi::StatusReg>().read().read<QuadSpi::sr::Busy>()) {
  }
}

uint32_t read_id(QuadSpi::RegBank& regs) noexcept {
  clear_flags(regs);

  regs.get_register<QuadSpi::DataLengthReg>().write([=](auto reg) { reg.template write<QuadSpi::dlr::DataLength>(2); });

  regs.get_register<QuadSpi::CommConfigReg>().write([=](auto reg) {
    reg.template write<QuadSpi::ccr::AddressMode>(QuadSpi::AddressMode::None);
    reg.template write<QuadSpi::ccr::AlternateBytesMode>(QuadSpi::AlternateBytesMode::None);
    reg.template write<QuadSpi::ccr::DataMode>(QuadSpi::DataMode::QuadLine);
    reg.template write<QuadSpi::ccr::Instruction>(static_cast<QuadSpi::Instruction>(Command::MultipleIoReadId));
    reg.template write<QuadSpi::ccr::InstructionMode>(QuadSpi::InstructionMode::QuadLine);
    reg.template write<QuadSpi::ccr::DdrMode>(false);
    reg.template write<QuadSpi::ccr::DdrHold>(false);
    reg.template write<QuadSpi::ccr::NumDummyCycles>(0);
    reg.template write<QuadSpi::ccr::FunctionalMode>(QuadSpi::FunctionalMode::IndirectRead);
  });

  // Wait until done
  while (!regs.get_register<QuadSpi::StatusReg>().read().read<QuadSpi::sr::TxComplete>())
    ;

  const uint32_t data{regs.get_register<QuadSpi::DataReg>().get()};

  while (regs.get_register<QuadSpi::StatusReg>().read().read<QuadSpi::sr::Busy>())
    ;

  return data;
}

std::uint8_t read_config_reg(QuadSpi::RegBank& regs) noexcept {
  clear_flags(regs);
  regs.get_register<QuadSpi::DataLengthReg>().write([=](auto reg) { reg.template write<QuadSpi::dlr::DataLength>(0); });
  regs.get_register<QuadSpi::CommConfigReg>().write([=](auto reg) {
    reg.template write<QuadSpi::ccr::AddressMode>(QuadSpi::AddressMode::None);
    reg.template write<QuadSpi::ccr::AlternateBytesMode>(QuadSpi::AlternateBytesMode::None);
    reg.template write<QuadSpi::ccr::DataMode>(QuadSpi::DataMode::SingleLine);
    reg.template write<QuadSpi::ccr::Instruction>(static_cast<QuadSpi::Instruction>(Command::ReadVolatileConfigReg));
    reg.template write<QuadSpi::ccr::InstructionMode>(QuadSpi::InstructionMode::SingleLine);
    reg.template write<QuadSpi::ccr::DdrMode>(false);
    reg.template write<QuadSpi::ccr::DdrHold>(false);
    reg.template write<QuadSpi::ccr::NumDummyCycles>(0);
    reg.template write<QuadSpi::ccr::FunctionalMode>(QuadSpi::FunctionalMode::IndirectRead);
  });

  // Wait until done
  while (!regs.get_register<QuadSpi::StatusReg>().read().read<QuadSpi::sr::TxComplete>())
    ;
  const uint32_t data{regs.get_register<QuadSpi::DataReg>().get()};
  while (regs.get_register<QuadSpi::StatusReg>().read().read<QuadSpi::sr::Busy>())
    ;
  return data;
}

void write_config_reg(QuadSpi::RegBank& regs, std::uint8_t value) noexcept {
  send_simple_command(regs, Command::WriteEnable);

  clear_flags(regs);
  regs.get_register<QuadSpi::DataLengthReg>().write([=](auto reg) { reg.template write<QuadSpi::dlr::DataLength>(0); });
  regs.get_register<QuadSpi::CommConfigReg>().write([=](auto reg) {
    reg.template write<QuadSpi::ccr::AddressMode>(QuadSpi::AddressMode::None);
    reg.template write<QuadSpi::ccr::AlternateBytesMode>(QuadSpi::AlternateBytesMode::None);
    reg.template write<QuadSpi::ccr::DataMode>(QuadSpi::DataMode::SingleLine);
    reg.template write<QuadSpi::ccr::Instruction>(static_cast<QuadSpi::Instruction>(Command::WriteVolatileConfigReg));
    reg.template write<QuadSpi::ccr::InstructionMode>(QuadSpi::InstructionMode::SingleLine);
    reg.template write<QuadSpi::ccr::DdrMode>(false);
    reg.template write<QuadSpi::ccr::DdrHold>(false);
    reg.template write<QuadSpi::ccr::NumDummyCycles>(0);
    reg.template write<QuadSpi::ccr::FunctionalMode>(QuadSpi::FunctionalMode::IndirectWrite);
  });

  regs.get_register<QuadSpi::DataReg>().set(value);

  // Wait until done
  while (!regs.get_register<QuadSpi::StatusReg>().read().read<QuadSpi::sr::TxComplete>())
    ;
  while (regs.get_register<QuadSpi::StatusReg>().read().read<QuadSpi::sr::Busy>())
    ;
}

std::uint8_t read_enhanced_config_reg(QuadSpi::RegBank& regs) noexcept {
  clear_flags(regs);
  regs.get_register<QuadSpi::DataLengthReg>().write([=](auto reg) { reg.template write<QuadSpi::dlr::DataLength>(0); });
  regs.get_register<QuadSpi::CommConfigReg>().write([=](auto reg) {
    reg.template write<QuadSpi::ccr::AddressMode>(QuadSpi::AddressMode::None);
    reg.template write<QuadSpi::ccr::AlternateBytesMode>(QuadSpi::AlternateBytesMode::None);
    reg.template write<QuadSpi::ccr::DataMode>(QuadSpi::DataMode::SingleLine);
    reg.template write<QuadSpi::ccr::Instruction>(
        static_cast<QuadSpi::Instruction>(Command::ReadEnhancedVolatileConfigReg));
    reg.template write<QuadSpi::ccr::InstructionMode>(QuadSpi::InstructionMode::SingleLine);
    reg.template write<QuadSpi::ccr::DdrMode>(false);
    reg.template write<QuadSpi::ccr::DdrHold>(false);
    reg.template write<QuadSpi::ccr::NumDummyCycles>(0);
    reg.template write<QuadSpi::ccr::FunctionalMode>(QuadSpi::FunctionalMode::IndirectRead);
  });

  // Wait until done
  while (!regs.get_register<QuadSpi::StatusReg>().read().read<QuadSpi::sr::TxComplete>())
    ;
  const uint32_t data{regs.get_register<QuadSpi::DataReg>().get()};
  while (regs.get_register<QuadSpi::StatusReg>().read().read<QuadSpi::sr::Busy>())
    ;
  return data;
}

void write_enhanced_config_reg(QuadSpi::RegBank& regs, uint8_t value) noexcept {
  send_simple_command(regs, Command::WriteEnable);

  clear_flags(regs);
  regs.get_register<QuadSpi::DataLengthReg>().write([=](auto reg) { reg.template write<QuadSpi::dlr::DataLength>(0); });
  regs.get_register<QuadSpi::CommConfigReg>().write([=](auto reg) {
    reg.template write<QuadSpi::ccr::AddressMode>(QuadSpi::AddressMode::None);
    reg.template write<QuadSpi::ccr::AlternateBytesMode>(QuadSpi::AlternateBytesMode::None);
    reg.template write<QuadSpi::ccr::DataMode>(QuadSpi::DataMode::SingleLine);
    reg.template write<QuadSpi::ccr::Instruction>(
        static_cast<QuadSpi::Instruction>(Command::WriteEnhancedVolatileConfigReg));
    reg.template write<QuadSpi::ccr::InstructionMode>(QuadSpi::InstructionMode::SingleLine);
    reg.template write<QuadSpi::ccr::DdrMode>(false);
    reg.template write<QuadSpi::ccr::DdrHold>(false);
    reg.template write<QuadSpi::ccr::NumDummyCycles>(0);
    reg.template write<QuadSpi::ccr::FunctionalMode>(QuadSpi::FunctionalMode::IndirectWrite);
  });

  regs.get_register<QuadSpi::DataReg>().set(value);

  // Wait until done
  while (!regs.get_register<QuadSpi::StatusReg>().read().read<QuadSpi::sr::TxComplete>())
    ;
  while (regs.get_register<QuadSpi::StatusReg>().read().read<QuadSpi::sr::Busy>())
    ;
}

void configure_quadspi_operation(QuadSpi::RegBank& regs) noexcept {
  constexpr static uint8_t QUADSPI_IO_PROTOCOL_MASK = 0x80;
  constexpr static uint8_t XIP_MASK = 1 << 3;
  constexpr static uint8_t DUMMY_CYCLES_OFFSET = 4;
  constexpr static uint8_t DUMMY_CYCLES_MASK = 0xF0;

  uint8_t cfg_reg = read_config_reg(regs);
  cfg_reg &= ~XIP_MASK & ~DUMMY_CYCLES_MASK;
  cfg_reg |= (NUM_DUMMY_CYCLES << DUMMY_CYCLES_OFFSET) & DUMMY_CYCLES_MASK;
  write_config_reg(regs, cfg_reg);

  uint8_t enhanced_cfg_reg = read_enhanced_config_reg(regs);
  enhanced_cfg_reg &= ~QUADSPI_IO_PROTOCOL_MASK;
  write_enhanced_config_reg(regs, enhanced_cfg_reg);

  return;
}

void enable_memory_mapped_operation(QuadSpi::RegBank& regs) noexcept {
  regs.get_register<QuadSpi::CommConfigReg>().write([=](auto reg) {
    reg.template write<QuadSpi::ccr::AddressMode>(QuadSpi::AddressMode::QuadLine);
    reg.template write<QuadSpi::ccr::AddressSize>(QuadSpi::AddressSize::Bits24);
    reg.template write<QuadSpi::ccr::AlternateBytesMode>(QuadSpi::AlternateBytesMode::None);
    reg.template write<QuadSpi::ccr::DataMode>(QuadSpi::DataMode::QuadLine);
    reg.template write<QuadSpi::ccr::Instruction>(static_cast<QuadSpi::Instruction>(Command::QuadIoFastRead));
    reg.template write<QuadSpi::ccr::InstructionMode>(QuadSpi::InstructionMode::QuadLine);
    reg.template write<QuadSpi::ccr::DdrMode>(false);
    reg.template write<QuadSpi::ccr::DdrHold>(false);
    reg.template write<QuadSpi::ccr::NumDummyCycles>(NUM_DUMMY_CYCLES);
    reg.template write<QuadSpi::ccr::FunctionalMode>(QuadSpi::FunctionalMode::MemMapped);
  });
}

}  // namespace

void QspiFlash::configure_pins() noexcept {
  m_rcc_regs.get_register<Rcc::Ahb1Enr>().read_modify_write([](auto reg) {
    reg.template write<Rcc::GpioBClkEn>(true);
    reg.template write<Rcc::GpioDClkEn>(true);
    reg.template write<Rcc::GpioEClkEn>(true);
  });

  m_ncs = get_gpio_bank(GpioBankId::B)
              .try_get_as_alternate_function(GpioPinNumber{6}, GpioAltFunc::AF10_SAI2_QUADSPI_OTG2HS_OTG1FS);
  m_clk = get_gpio_bank(GpioBankId::B)
              .try_get_as_alternate_function(GpioPinNumber{2}, GpioAltFunc::AF9_CAN1_2_TIM12_13_14_QUADSPI_LCD);
  m_data[0] = get_gpio_bank(GpioBankId::D)
                  .try_get_as_alternate_function(GpioPinNumber{11}, GpioAltFunc::AF9_CAN1_2_TIM12_13_14_QUADSPI_LCD);
  m_data[1] = get_gpio_bank(GpioBankId::D)
                  .try_get_as_alternate_function(GpioPinNumber{12}, GpioAltFunc::AF9_CAN1_2_TIM12_13_14_QUADSPI_LCD);
  m_data[2] = get_gpio_bank(GpioBankId::E)
                  .try_get_as_alternate_function(GpioPinNumber{2}, GpioAltFunc::AF9_CAN1_2_TIM12_13_14_QUADSPI_LCD);
  m_data[3] = get_gpio_bank(GpioBankId::D)
                  .try_get_as_alternate_function(GpioPinNumber{13}, GpioAltFunc::AF9_CAN1_2_TIM12_13_14_QUADSPI_LCD);

  DITTO_VERIFY(m_ncs);
  DITTO_VERIFY(m_clk);
  for (auto& pin : m_data) {
    DITTO_VERIFY(pin);
  }

  m_ncs.set_pull_resistor_config(GpioPullResistorConfig::PullUp);
  m_ncs.set_output_speed(GpioOutputSpeed::VeryHigh);
  m_ncs.set_output_type(GpioOutputType::PushPull);
  m_clk.set_output_speed(GpioOutputSpeed::VeryHigh);
  m_clk.set_output_type(GpioOutputType::PushPull);
  for (auto& pin : m_data) {
    pin.set_output_speed(GpioOutputSpeed::VeryHigh);
    pin.set_output_type(GpioOutputType::PushPull);
  }
}

void QspiFlash::configure_quadspi() noexcept {
  m_rcc_regs.get_register<Rcc::Ahb3Enr>().read_modify_write([](auto reg) { reg.template write<Rcc::QspiEn>(true); });
  m_rcc_regs.get_register<Rcc::Ahb3Rst>().read_modify_write([](auto reg) { reg.template write<Rcc::QspiRst>(true); });
  m_rcc_regs.get_register<Rcc::Ahb3Rst>().read_modify_write([](auto reg) { reg.template write<Rcc::QspiRst>(false); });

  m_quadspi_regs.get_register<QuadSpi::ControlReg>().read_modify_write([](auto reg) {
    reg.template write<QuadSpi::cr::Enable>(false);
    reg.template write<QuadSpi::cr::FlashSelection>(QuadSpi::SelectedFlash::Mem1);
    reg.template write<QuadSpi::cr::Prescaler>(1);  // TODO(javier-varez): Make this fast after bringup
    reg.template write<QuadSpi::cr::FifoThLvl>(6);
    reg.template write<QuadSpi::cr::SampleShift>(true);
    reg.template write<QuadSpi::cr::DualFlashMode>(false);
  });

  m_quadspi_regs.get_register<QuadSpi::DeviceConfigReg>().read_modify_write([](auto reg) {
    reg.template write<QuadSpi::dcr::ChipSelectHighTime>(6);
    reg.template write<QuadSpi::dcr::ClockMode>(QuadSpi::DeviceSpiClockMode::Mode0);
    reg.template write<QuadSpi::dcr::FlashSize>(23);  // (2 ^ 24) = 16 MB flash
  });

  m_quadspi_regs.get_register<QuadSpi::ControlReg>().read_modify_write(
      [](auto reg) { reg.template write<QuadSpi::cr::Enable>(true); });

  send_simple_command(m_quadspi_regs, Command::ResetEnable);
  send_simple_command(m_quadspi_regs, Command::ResetMemory);

  configure_quadspi_operation(m_quadspi_regs);

  constexpr static uint32_t MEMORY_ID = 0x18BA20;
  uint32_t id = read_id(m_quadspi_regs);
  LOG_DEBUG(&logger, "QSPI ID: %x", id);
  DITTO_VERIFY(id == MEMORY_ID);

  enable_memory_mapped_operation(m_quadspi_regs);
}

void QspiFlash::init() noexcept {
  configure_pins();
  configure_quadspi();
}

}  // namespace Hw
