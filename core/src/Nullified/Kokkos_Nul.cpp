//@HEADER
// ************************************************************************
//
//                        Kokkos v. 4.0
//       Copyright (2022) National Technology & Engineering
//               Solutions of Sandia, LLC (NTESS).
//
// Under the terms of Contract DE-NA0003525 with NTESS,
// the U.S. Government retains certain rights in this software.
//
// Part of Kokkos, under the Apache License v2.0 with LLVM Exceptions.
// See https://kokkos.org/LICENSE for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//@HEADER

#ifndef KOKKOS_IMPL_PUBLIC_INCLUDE
#define KOKKOS_IMPL_PUBLIC_INCLUDE
#endif

#include <Kokkos_Core.hpp>

#include <Nullified/Kokkos_Nul.hpp>
#include <impl/Kokkos_Traits.hpp>
#include <impl/Kokkos_Error.hpp>
#include <impl/Kokkos_ExecSpaceManager.hpp>
#include <impl/Kokkos_SharedAlloc.hpp>

#include <cstdlib>
#include <iostream>
#include <sstream>

/*--------------------------------------------------------------------------*/

namespace Kokkos {
namespace Impl {

bool NulInternal::is_initialized() { return m_is_initialized; }

void NulInternal::initialize() {
  if (is_initialized()) return;

  // Impl::SharedAllocationRecord<void, void>::tracking_enable();

  m_is_initialized = true;
}

void NulInternal::finalize() {

  m_is_initialized = false;
}

NulInternal& NulInternal::singleton() {
  static NulInternal* self = nullptr;
  if (!self) {
    self = new NulInternal();
  }
  return *self;
}

// Resize thread team data scratch memory
void NulInternal::resize_thread_team_data(size_t pool_reduce_bytes,
                                             size_t team_reduce_bytes,
                                             size_t team_shared_bytes, 
					     size_t thread_local_bytes)
					    
  {

  }

} // namespace Impl

Nul::Nul()
    : m_space_instance(&Impl::NulInternal::singleton(),
                       [](Impl::NulInternal*) {}) {}

Nul::Nul(NewInstance)
    : m_space_instance(new Impl::NulInternal, [](Impl::NulInternal* ptr) {
        ptr->finalize();
        delete ptr;
      }) {}

void Nul::print_configuration(std::ostream& os, bool /*verbose*/) const {
  os << "Host Nul Execution Space:\n";
  os << "  KOKKOS_ENABLE_NULLIFIED: yes\n";

#ifdef KOKKOS_ENABLE_ATOMICS_BYPASS
  os << "Kokkos atomics disabled\n";
#endif

  os << "\nNUL Runtime Configuration:\n";
}

bool Nul::impl_is_initialized() {
  return Impl::NulInternal::singleton().is_initialized();
}

void Nul::impl_initialize(InitializationSettings const&) {
  Impl::NulInternal::singleton().initialize();
}

void Nul::impl_finalize() { Impl::NulInternal::singleton().finalize(); }

const char* Nul::name() { return "Nul"; }

namespace Impl {

int g_nul_space_factory_initialized =
    initialize_space_factory<Nul>("100_Nul");

}  // namespace Impl

}  // namespace Kokkos
