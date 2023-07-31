#pragma once

#include <bit>
#include <climits>
#include <concepts>
#include <cstddef>
#include <cstdint>

#include "ditto/type_traits.h"

namespace Hw {

enum class RegOffset : std::size_t {};
enum class RegNumBits : std::size_t {};
enum class NumFields : std::size_t {};
enum class FieldStride : std::size_t {};
enum class BlockOffset : std::size_t {};
enum class RegIndex : std::size_t {};

namespace detail {
template <std::integral T>
consteval inline T generate_mask(RegOffset offset, RegNumBits num_bits) noexcept {
  const size_t first_bit = static_cast<std::size_t>(offset);
  const size_t last_bit = static_cast<std::size_t>(offset) + static_cast<std::size_t>(num_bits);

  T mask{0};
  for (std::size_t i = first_bit; i < last_bit; i++) {
    mask |= T{1} << i;
  }

  return mask;
}

template <typename T>
concept field = requires(T t) {
                  requires std::integral<std::remove_cvref_t<decltype(T::mask)>>;
                  requires std::same_as<std::remove_cvref_t<decltype(T::offset)>, RegOffset>;
                  { T::get_full_mask() } -> std::same_as<typename T::ParentType>;
                };

template <typename T>
concept indexed_field = requires(T t) {
                          requires field<T>;
                          requires std::same_as<std::remove_cvref_t<decltype(T::stride)>, FieldStride>;
                          requires std::same_as<std::remove_cvref_t<decltype(T::num_fields)>, NumFields>;
                        };

}  // namespace detail

template <std::integral T, typename F, RegOffset Offset, RegNumBits NumBits>
struct FieldDesc final {
  using InnerType = F;
  using ParentType = T;

  constexpr static RegOffset offset = Offset;
  constexpr static ParentType mask = detail::generate_mask<ParentType>(Offset, NumBits);

  [[nodiscard]] static consteval ParentType get_full_mask() noexcept { return mask; }

  static_assert(static_cast<std::size_t>(NumBits) > 0u);
  static_assert(static_cast<std::size_t>(Offset) + static_cast<std::size_t>(NumBits) <= sizeof(T) * CHAR_BIT);
};

template <std::integral T, typename F, RegOffset Offset, RegNumBits NumBits, NumFields Numfields,
          FieldStride Stride = FieldStride{static_cast<std::size_t>(NumBits)}>
struct IndexedField final {
  using InnerType = F;
  using ParentType = T;

  constexpr static RegOffset offset = Offset;
  constexpr static ParentType mask = detail::generate_mask<ParentType>(Offset, NumBits);
  constexpr static FieldStride stride = Stride;
  constexpr static NumFields num_fields = Numfields;

  [[nodiscard]] static consteval ParentType offset_for_index(std::size_t index) noexcept {
    return static_cast<std::size_t>(Stride) * index;
  }

  [[nodiscard]] static consteval ParentType mask_for_index(std::size_t index) noexcept {
    return mask << offset_for_index(index);
  }

  [[nodiscard]] static consteval ParentType get_full_mask() noexcept {
    ParentType mask{0};
    for (std::size_t i = 0; i < static_cast<std::size_t>(Numfields); i++) {
      mask |= mask_for_index(i);
    }
    return mask;
  }

  static_assert(static_cast<std::size_t>(NumBits) > 0u);
  static_assert(static_cast<std::size_t>(Offset) + static_cast<std::size_t>(NumBits) <= sizeof(T) * CHAR_BIT);
  static_assert(static_cast<std::size_t>(Stride) >= static_cast<std::size_t>(NumBits));
};

template <std::integral T, BlockOffset Offset, detail::field... Fields>
  requires((std::same_as<T, typename Fields::ParentType> && ...))
class Register final {
 private:
  template <detail::field Field>
  [[nodiscard]] static consteval bool validate_single_field() noexcept {
    size_t count = (((Field::get_full_mask() & Fields::get_full_mask()) != 0) + ...);
    return count == 1;
  }

  [[nodiscard]] static consteval bool validate_all_fields() noexcept {
    return (validate_single_field<Fields>() && ...);
  }

  static_assert(validate_all_fields(), "Detected overlap between the provided fields");

  template <Ditto::one_of<Fields...> Field, Ditto::one_of<Fields...>... OtherFields>
  static T write_fields(T value, const typename Field::InnerType first_field,
                        const typename OtherFields::InnerType... other_fields) {
    value &= ~Field::mask;
    value |= Field::mask & (static_cast<T>(first_field) << static_cast<std::size_t>(Field::offset));

    if constexpr (sizeof...(other_fields) > 0) {
      value = write_fields<OtherFields...>(value, other_fields...);
    }

    return value;
  }

  class WriteRegProxy {
   public:
    explicit WriteRegProxy(T& value) noexcept : m_value(value) {}
    WriteRegProxy(WriteRegProxy&&) = delete;
    WriteRegProxy(const WriteRegProxy&) = delete;
    WriteRegProxy& operator=(WriteRegProxy&&) = delete;
    WriteRegProxy& operator=(const WriteRegProxy&) = delete;

    template <typename Field>
      requires(Ditto::one_of<Field, Fields...> && !detail::indexed_field<Field>)
    void write(typename Field::InnerType first_field) {
      m_value &= ~Field::mask;
      m_value |= Field::mask & (static_cast<T>(first_field) << static_cast<std::size_t>(Field::offset));
    }

    template <typename Field>
      requires(Ditto::one_of<Field, Fields...> && detail::indexed_field<Field>)
    void write(RegIndex index, typename Field::InnerType field) {
      T mask = Field::mask << (static_cast<std::size_t>(index) * static_cast<std::size_t>(Field::stride));
      std::size_t offset = static_cast<size_t>(Field::offset) +
                           (static_cast<std::size_t>(index) * static_cast<std::size_t>(Field::stride));
      m_value &= ~mask;
      m_value |= mask & (static_cast<T>(field) << static_cast<std::size_t>(offset));
    }

    template <Ditto::one_of<Fields...> Field>
    [[nodiscard]] typename Field::InnerType read() const {
      return static_cast<typename Field::InnerType>((m_value & Field::mask) >> static_cast<std::size_t>(Field::offset));
    }

   private:
    T& m_value;
  };

  class ReadRegProxy {
   public:
    explicit ReadRegProxy(T value) noexcept : m_value(value) {}
    ReadRegProxy(ReadRegProxy&&) = delete;
    ReadRegProxy(const ReadRegProxy&) = delete;
    ReadRegProxy& operator=(ReadRegProxy&&) = delete;
    ReadRegProxy& operator=(const ReadRegProxy&) = delete;

    template <typename Field>
      requires(Ditto::one_of<Field, Fields...> && !detail::indexed_field<Field>)
    [[nodiscard]] typename Field::InnerType read() const {
      return static_cast<typename Field::InnerType>((m_value & Field::mask) >> static_cast<std::size_t>(Field::offset));
    }

    template <typename Field>
      requires(Ditto::one_of<Field, Fields...> && detail::indexed_field<Field>)
    [[nodiscard]] typename Field::InnerType read(RegIndex index) const {
      T mask = Field::mask << (static_cast<std::size_t>(index) * static_cast<std::size_t>(Field::stride));
      std::size_t offset = static_cast<size_t>(Field::offset) +
                           (static_cast<std::size_t>(index) * static_cast<std::size_t>(Field::stride));
      return static_cast<typename Field::InnerType>((m_value & mask) >> offset);
    }

   private:
    T m_value;
  };

 public:
  constexpr static BlockOffset OFFSET = Offset;

  explicit Register(volatile void* addr) noexcept : m_register_addr(static_cast<volatile T*>(addr)) {}

  template <typename C>
  void read_modify_write(C callable) {
    T value = *m_register_addr;
    callable(WriteRegProxy{value});
    *m_register_addr = value;
  }

  template <typename C>
  void write(C callable) {
    T value{};
    callable(WriteRegProxy{value});
    *m_register_addr = value;
  }

  ReadRegProxy read() { return ReadRegProxy{*m_register_addr}; }

  // Not const because the register might be clear-on-read or have other
  // side-effects
  [[nodiscard]] T get() { return *m_register_addr; }

  void set(const T value) { *m_register_addr = value; }

 private:
  volatile T* const m_register_addr;
};

struct MmappedRegs {
  volatile void* base_address = nullptr;

  MmappedRegs(std::uintptr_t ptr) : base_address{std::bit_cast<volatile void*>(ptr)} {}

  volatile void* offset(BlockOffset offset) const {
    return static_cast<volatile std::uint8_t*>(base_address) + static_cast<std::size_t>(offset);
  }
};

template <typename... Registers>
class RegisterBank {
 public:
  explicit RegisterBank(MmappedRegs mmapped_regs) noexcept : m_mmapped_regs{mmapped_regs} {}

  template <Ditto::one_of<Registers...> T>
  [[nodiscard]] T get_register() noexcept {
    return T{m_mmapped_regs.offset(T::OFFSET)};
  }

 private:
  MmappedRegs m_mmapped_regs;
};

}  // namespace Hw
