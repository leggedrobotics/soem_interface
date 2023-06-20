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

namespace detail {

template <typename DescriptionTuple, typename DataTuple, typename Function, typename... FunctionArgs, size_t... Indices>
constexpr void callFunctionOnDoubleTupleHelper(const DescriptionTuple& descriptionTuple, DataTuple& dataTuple, Function&& function,
                                               std::index_sequence<Indices...>, FunctionArgs&&... functionArgs) {
  (std::forward<Function>(function)(Indices, std::get<Indices>(descriptionTuple), std::get<Indices>(dataTuple),
                                    std::forward<FunctionArgs>(functionArgs)...),
   ...);
}

template <typename DescriptionTuple, typename DataTuple, typename Function, typename... FunctionArgs>
constexpr void callFunctionDoubleTuple(const DescriptionTuple& descriptionTuple, DataTuple& dataTuple, Function&& function,
                                       FunctionArgs&&... functionArgs) {
  constexpr size_t descriptionSize = std::tuple_size_v<DescriptionTuple>;
  constexpr size_t dataSize = std::tuple_size_v<DataTuple>;
  static_assert(descriptionSize == dataSize, "Description and Data Tuple require the same size");
  callFunctionOnDoubleTupleHelper(descriptionTuple, dataTuple, std::forward<Function>(function),
                                  std::make_index_sequence<descriptionSize>(), std::forward<FunctionArgs>(functionArgs)...);
}

template <typename TupleLike_, typename Function, typename... FunctionArgs, size_t... Indices>
constexpr void callFunctionOnTuple(const TupleLike_& tuple, Function&& function, std::index_sequence<Indices...>,
                                   FunctionArgs&&... functionArgs) {
  (function(Indices, std::get<Indices>(tuple), std::forward<FunctionArgs>(functionArgs)...), ...);
}

template <typename TupleLike_, typename Function, typename... FunctionArgs>
constexpr void callFunctionOnTuple(const TupleLike_& tuple, Function&& function, FunctionArgs&&... functionArgs) {
  constexpr size_t tupleSize = std::tuple_size_v<TupleLike_>;
  return callFunctionOnTuple(tuple, std::forward<Function>(function), std::make_index_sequence<tupleSize>(),
                             std::forward<FunctionArgs>(functionArgs)...);
}

template <class Callable>
struct SubEntryIterator {
 private:
  Callable callable_;

 public:
  SubEntryIterator() = delete;
  explicit SubEntryIterator(Callable&& callable) : callable_(callable){};

  template <typename DescriptionContainerElement, typename DataContainerElement>
  constexpr void operator()(size_t entryIndex, DescriptionContainerElement descriptionContainerElement,
                            DataContainerElement& dataContainerElement) {
    // we have a nested tuple, the 0th entry is for both the same description.
    constexpr auto descriptionEntry = std::get<0>(descriptionContainerElement);
    constexpr auto descriptionSubentryTuple = std::get<1>(descriptionContainerElement);
    auto& dataSubentryTuple = std::get<1>(dataContainerElement);
    std::cout << "Entry: " << descriptionEntry.OD_Index << " tupleIndex: " << entryIndex << " Name: " << descriptionEntry.Name << std::endl;
    callFunctionDoubleTuple(descriptionSubentryTuple, dataSubentryTuple, callable_, entryIndex, descriptionEntry);
  };
};

struct EntryAndSubEntryPrinter {
  template <typename DescriptionContainerElement, typename DataContainerElement>
  constexpr void operator()(size_t entryIndex, DescriptionContainerElement descriptionContainerElement,
                            DataContainerElement& dataContainerElement) {
    // we have a nested tuple, the 0th entry is for both the same description.
    constexpr auto descriptionEntry = std::get<0>(descriptionContainerElement);
    constexpr auto descriptionSubentryTuple = std::get<1>(descriptionContainerElement);
    auto& dataSubentryTuple = std::get<1>(dataContainerElement);
    std::cout << "Entry: " << descriptionEntry.OD_Index << " tupleIndex: " << entryIndex << " Name: " << descriptionEntry.Name << std::endl;

    auto funOnSubentry = [](size_t subEntryIndex, auto subEntryDescription, auto& subEntryData) {
      // we can use descriptionContainerElement in constexpr fashion
      constexpr soem_interface_rsl::ETHERCAT_TYPE ecatType = subEntryDescription.EthercatType;
      std::cout << "       Subindex: " << static_cast<uint16_t>(subEntryDescription.SubIndex) << " subEntryTupleIndex: " << subEntryIndex
                << " Name: " << subEntryDescription.Name << " Value: " << subEntryData << std::endl;
    };

    callFunctionDoubleTuple(descriptionSubentryTuple, dataSubentryTuple, funOnSubentry);
  };
};

}  // namespace detail

// Non generated - general code, relies on the generated code/containers, to have the same structure.
template <typename OD_Description_, typename OD_DescriptionContainer_>
struct ObjectDictionaryDescriptionConcrete {
 private:
  static constexpr OD_DescriptionContainer_ OD_DescriptionContainer{};
  using OD_Description = OD_Description_;

  template <size_t InternalIndex, size_t InternalSubIndex>
  struct GetDescriptionSubEntryInternalHelper {
    struct internal {
      using ODEntryTuple_Type_found = typename std::tuple_element<InternalIndex, OD_DescriptionContainer_>::type;
      using ODEntryListTuple_Type_found = typename std::tuple_element<1, ODEntryTuple_Type_found>::type;
    };
    using type = typename std::tuple_element<InternalSubIndex, typename internal::ODEntryListTuple_Type_found>::type;
  };

  template <size_t InternalIndex>
  using GetDescriptionEntryInternal = typename std::tuple_element<InternalIndex, OD_DescriptionContainer_>::type;

 public:
  template <uint16_t Index>
  using GetDescriptionEntry = typename std::tuple_element<
      0, GetDescriptionEntryInternal<OD_Description::Internal::IndexInternalMap::template InternalIndexAt<Index>>>::type;

 private:
  template <uint16_t Index, uint8_t SubIndex>
  struct GetDescriptionSubEntry_ {
    using DescriptionEntry = GetDescriptionEntry<Index>;
    using type = typename GetDescriptionSubEntryInternalHelper<
        OD_Description::Internal::IndexInternalMap::template InternalIndexAt<Index>,
        DescriptionEntry::Internal::SubIndexInternalMap::template InternalIndexAt<SubIndex>>::type;
  };

 public:
  template <uint16_t Index, uint16_t SubIndex>
  using GetDescriptionSubEntry = typename GetDescriptionSubEntry_<Index, SubIndex>::type;

  template <uint16_t Index, uint16_t SubIndex>
  using GetDescriptionSubEntryType = typename GetDescriptionSubEntry_<Index, SubIndex>::type::Type;

  template <typename OD_DataContainer_>
  static void iterateOverSubEntryStructure(OD_DataContainer_& odDataContainer) {
    auto funOnSubentry = [](size_t subEntryIndex, auto subEntryDescription, auto& subEntryData, size_t entryIndex, auto descriptionEntry) {
      // we can use descriptionContainerElement in constexpr fashion
      constexpr soem_interface_rsl::ETHERCAT_TYPE ecatType = subEntryDescription.EthercatType;
      std::cout << "       Subindex: " << static_cast<uint16_t>(subEntryDescription.SubIndex) << " subEntryTupleIndex: " << subEntryIndex
                << " Name: " << subEntryDescription.Name << std::endl;

      switch (ecatType) {
        case soem_interface_rsl::ETHERCAT_TYPE::UNSIGNED16:
          subEntryData = static_cast<typename decltype(subEntryDescription)::Type>(10);
          break;
        case soem_interface_rsl::ETHERCAT_TYPE::INTEGER32:
          subEntryData = static_cast<typename decltype(subEntryDescription)::Type>(-10);
          break;
      }
      // to something type specific here.
    };

    detail::callFunctionDoubleTuple(OD_DescriptionContainer, odDataContainer, detail::SubEntryIterator(std::move(funOnSubentry)));
  }

  template <typename OD_DataContainer_>
  static void printODwithValues(const OD_DataContainer_& odDataContainer) {
    detail::callFunctionDoubleTuple(OD_DescriptionContainer, odDataContainer, detail::EntryAndSubEntryPrinter{});
  }
};

template <typename OD_Description_, typename OD_DataContainer_>
struct ObjectDiectionaryDataConcrete {
  using OD_Description = OD_Description_;
  OD_DataContainer_ data{};

 private:
  template <size_t InternalIndex, size_t InternalSubIndex>
  auto getSubEntryInternal() {
    return std::get<InternalSubIndex>(std::get<1>(std::get<InternalIndex>(data)));
  }

  template <size_t InternalIndex>
  using GetDescriptionEntryInternal = typename std::tuple_element<InternalIndex, OD_DataContainer_>::type;

  template <uint16_t Index>
  using GetDescriptionEntry = typename std::tuple_element<
      0, GetDescriptionEntryInternal<OD_Description::Internal::IndexInternalMap::template InternalIndexAt<Index>>>::type;

 public:
  template <uint16_t Index, uint8_t SubIndex>
  auto getValueByIndex() {
    using DescriptionEntry = GetDescriptionEntry<Index>;
    return getSubEntryInternal<OD_Description::Internal::IndexInternalMap::template InternalIndexAt<Index>,
                               DescriptionEntry::Internal::SubIndexInternalMap::template InternalIndexAt<SubIndex>>();
  };
};

// only possible on single entry level. or on packed structures.
template <class T>
T fromRaw(void* data, size_t size, T& readEntryOut) {
  if (size > sizeof(T)) {
    // todo care about endianess of platform.
    auto returnMemcpy = std::memcpy(&readEntryOut, data, sizeof(T));
    if (returnMemcpy) {
      return true;
    } else {
      // nullpointer returned by memcpy.
      return false;
    }
  } else {
    throw std::runtime_error("EntryFromRaw provided buffer should contain Fullobject but is to small to fully hold it.");
  }
}

}  // namespace soem_interface_rsl