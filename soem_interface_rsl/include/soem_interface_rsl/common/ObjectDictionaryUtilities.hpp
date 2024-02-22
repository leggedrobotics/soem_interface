/*
** Copyright (2019-2020) Robotics Systems Lab - ETH Zurich:
** Markus Staeuble, Jonas Junger, Johannes Pankert, Philipp Leemann,
** Tom Lankhorst, Samuel Bachmann, Gabriel Hottiger, Lennert Nachtigall,
** Mario Mauerer, Remo Diethelm
**
** This file is part of the soem_interface_rsl.
**
** The soem_interface_rsl is free software: you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published by
** the Free Software Foundation, either version 3 of the License, or
** (at your option) any later version.
**
** The seom_interface is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with the soem_interface_rsl.  If not, see <https://www.gnu.org/licenses/>.
*/
#pragma once
#include <array>
#include <cstdint>
#include <exception>
#include <optional>
#include <string_view>
#include <type_traits>
#include "EthercatTypes.hpp"

namespace soem_interface_rsl {

// todo make this reasonable:
//  the definition of a Object Dictionary should be simpel, the best is a macro which generates nested structs / array's.
//  each Object Entry (depending on the type, VARiable, ARRAY, RECORD) can also provide some special member functions
//  then also PDO's (dynamic) can be hardcoded with a simpel Macro.
//  addtionally, auto generation of PDO and ObjectDict Headers from Slave Info is possible.

template <ETHERCAT_TYPE typeEnum>
struct TypeFromECType {};

template <>
struct TypeFromECType<ETHERCAT_TYPE::ECT_BOOLEAN> {
  using Type = bool;
  static constexpr size_t size = sizeof(Type);
};

template <>
struct TypeFromECType<ETHERCAT_TYPE::ECT_INTEGER8> {
  using Type = int8_t;
  static constexpr size_t size = sizeof(Type);
};

template <>
struct TypeFromECType<ETHERCAT_TYPE::ECT_INTEGER16> {
  using Type = int16_t;
  static constexpr size_t size = sizeof(Type);
};

template <>
struct TypeFromECType<ETHERCAT_TYPE::ECT_INTEGER32> {
  using Type = int32_t;
  static constexpr size_t size = sizeof(Type);
};

template <>
struct TypeFromECType<ETHERCAT_TYPE::ECT_INTEGER64> {
  using Type = int64_t;
  static constexpr size_t size = sizeof(Type);
};

template <>
struct TypeFromECType<ETHERCAT_TYPE::ECT_UNSIGNED8> {
  using Type = uint8_t;
  static constexpr size_t size = sizeof(Type);
};

template <>
struct TypeFromECType<ETHERCAT_TYPE::ECT_UNSIGNED16> {
  using Type = uint16_t;
  static constexpr size_t size = sizeof(Type);
};

template <>
struct TypeFromECType<ETHERCAT_TYPE::ECT_UNSIGNED24> {
  // whats this.?
};

template <>
struct TypeFromECType<ETHERCAT_TYPE::ECT_UNSIGNED32> {
  using Type = uint32_t;
  static constexpr size_t size = sizeof(Type);
};

template <>
struct TypeFromECType<ETHERCAT_TYPE::ECT_UNSIGNED64> {
  using Type = uint64_t;
  static constexpr size_t size = sizeof(Type);
};

template <>
struct TypeFromECType<ETHERCAT_TYPE::ECT_REAL64> {
  using Type = double;
  static constexpr size_t size = sizeof(Type);
};

template <>
struct TypeFromECType<ETHERCAT_TYPE::ECT_REAL32> {
  using Type = float;
  static constexpr size_t size = sizeof(Type);
};

struct SubEntry {
  constexpr SubEntry(uint8_t subIdx_, uint16_t simplifiedAccess_, std::string_view name_, ETHERCAT_TYPE datatype_)
      : subIdx(subIdx_), simplifiedAccess(simplifiedAccess_), name(name_), datatype(datatype_){};
  constexpr SubEntry(uint8_t subIdx_, uint16_t simplifiedAccess_, std::string_view name_, ETHERCAT_TYPE datatype_,
                     std::string_view unitStr_, double toSIFactor_, std::string_view siUnitStr_)
      : subIdx(subIdx_),
        simplifiedAccess(simplifiedAccess_),
        name(name_),
        datatype(datatype_),
        unitStr(unitStr_),
        toSIFactor(toSIFactor_),
        siUnitStr(siUnitStr_){};

  constexpr explicit SubEntry(uint8_t subIdx_, uint16_t simplifiedAccess_ = EcAccess::NOT_IMPL)
      : subIdx(subIdx_), simplifiedAccess(simplifiedAccess_), name("not_impl"), datatype(ETHERCAT_TYPE::ECT_BOOLEAN){};

  const uint8_t subIdx;
  uint16_t simplifiedAccess{EcAccess::NOT_IMPL};
  const std::string_view name;
  const ETHERCAT_TYPE datatype;
  const std::optional<std::string_view> unitStr{};
  const std::optional<double> toSIFactor{};
  const std::optional<std::string_view> siUnitStr{};

  constexpr uint8_t getSubidx() { return subIdx; };
};

template <class EntryType, class EntryEnum>
struct Entries {
  using EntryCollection = std::array<EntryType, static_cast<size_t>(EntryEnum::SIZE) - 1>;
  EntryCollection entries;

  using iterator = typename EntryCollection::iterator;
  using const_iterator = typename EntryCollection::const_iterator;
  [[nodiscard]] constexpr const_iterator begin() const { return entries.begin(); }
  [[nodiscard]] constexpr const_iterator end() const { return entries.end(); }
  [[nodiscard]] constexpr const_iterator cbegin() const { return entries.cbegin(); }
  [[nodiscard]] constexpr const_iterator cend() const { return entries.cend(); }

  template <typename IdxType>
  constexpr const EntryType& operator[](const IdxType idx) const {
    if constexpr (std::is_same<IdxType, EntryEnum>::value) {
      return entries[static_cast<size_t>(idx)];
    } else {
      return entries[idx];
    }
  }

  [[nodiscard]] constexpr ETHERCAT_TYPE ecDatatypeByEnum(const EntryEnum entryEnum) const {
    for (const auto& entry : entries) {
      if (entry.subIdx == static_cast<size_t>(entryEnum)) {
        return entry.datatype;
      }
    }
    throw std::runtime_error("entry not found, should be throw at ct");
  }

  [[nodiscard]] constexpr bool check() const {
    unsigned int i = 0;
    for (const auto& entry : entries) {
      ++i;
      if (entry.subIdx != i) {
        return true;
      }
    }
    return true;
  }
};

template <uint16_t id_, uint8_t nSubEntries_>
struct ObjectBase {
  static constexpr uint16_t idx{id_};
  static constexpr uint8_t nSubEntries{nSubEntries_};
  using ETHERCAT_TYPE = soem_interface_rsl::ETHERCAT_TYPE;
};

}  // namespace soem_interface_rsl