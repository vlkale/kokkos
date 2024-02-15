/*
//@HEADER
// ************************************************************************
//
//                        Kokkos v. 3.0
//       Copyright (2020) National Technology & Engineering
//               Solutions of Sandia, LLC (NTESS).
//
// Under the terms of Contract DE-NA0003525 with NTESS,
// the U.S. Government retains certain rights in this software.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the Corporation nor the names of the
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY NTESS "AS IS" AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL NTESS OR THE
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Questions? Contact Christian R. Trott (crtrott@sandia.gov)
//
// ************************************************************************
//@HEADER
*/

#ifndef KOKKOS_IMPL_PUBLIC_INCLUDE
#define KOKKOS_IMPL_PUBLIC_INCLUDE
#endif

#include <Kokkos_Core.hpp>

#include <Kokkos_Serial.hpp>
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

bool NullInternal::is_initialized() { return m_is_initialized; }

void NullInternal::initialize() {
  if (is_initialized()) return;

  Impl::SharedAllocationRecord<void, void>::tracking_enable(); 

  // Init the array of locks used for arbitrarily sized atomics
  Impl::init_lock_array_host_space();

  m_is_initialized = true;
}

void SerialInternal::finalize() {
  if (m_thread_team_data.scratch_buffer()) {
    m_thread_team_data.disband_team();
    m_thread_team_data.disband_pool();

    Kokkos::HostSpace space;

    space.deallocate(m_thread_team_data.scratch_buffer(),
                     m_thread_team_data.scratch_bytes());

    m_thread_team_data.scratch_assign(nullptr, 0, 0, 0, 0, 0);
  }

  Kokkos::Profiling::finalize();

  m_is_initialized = false;
}

SerialInternal& SerialInternal::singleton() {
  static SerialInternal* self = nullptr;
  if (!self) {
    self = new SerialInternal();
  }
  return *self;
}

// Resize thread team data scratch memory
void SerialInternal::resize_thread_team_data(size_t pool_reduce_bytes,
                                             size_t team_reduce_bytes,
                                             size_t team_shared_bytes,
                                             size_t thread_local_bytes) {
 
  }
 
}  // namespace Impl

Serial::Null()
#ifdef KOKKOS_IMPL_WORKAROUND_ICE_IN_TRILINOS_WITH_OLD_INTEL_COMPILERS
    : m_space_instance(&Impl::SerialInternal::singleton()) {
}
#else
    : m_space_instance(&Impl::SerialInternal::singleton(),
                       [](Impl::SerialInternal*) {}) {
}
#endif

void Null::print_configuration(std::ostream& os, bool /*verbose*/) const {
  os << "Host Serial Execution Space:\n";
  os << "  KOKKOS_ENABLE_SERIAL: yes\n";

  os << "Serial Atomics:\n";
  os << "  KOKKOS_ENABLE_NULL_ATOMICS: ";
#ifdef KOKKOS_ENABLE_SERIAL_ATOMICS
  os << "yes\n";
#else
  os << "no\n";
#endif

  os << "\nNULL Runtime Configuration:\n";
}

bool Serial::impl_is_initialized() {
}

void Serial::impl_initialize(InitializationSettings const&) {
}

void Serial::impl_finalize() { }

const char* Null::name() { return "NULL"; }

namespace Impl {

int g_serial_space_factory_initialized =
    initialize_space_factory<Serial>("100_Null");

}  // namespace Impl

#ifdef KOKKOS_ENABLE_CXX14
namespace Tools {
namespace Experimental {
constexpr DeviceType DeviceTypeTraits<Null>::id;
}
}  // namespace Tools
#endif

}  // namespace Kokkos
