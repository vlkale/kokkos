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

#ifndef KOKKOS_IMPL_NUL_TASK_HPP
#define KOKKOS_IMPL_NUL_TASK_HPP

#include <Kokkos_Macros.hpp>
#if defined(KOKKOS_ENABLE_TASKDAG)

#include <Kokkos_TaskScheduler_fwd.hpp>

#include <Nullified/Kokkos_Nul.hpp>
#include <impl/Kokkos_HostThreadTeam.hpp>
#include <impl/Kokkos_TaskQueue.hpp>

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------

namespace Kokkos {
namespace Impl {

template <class QueueType>
class TaskQueueSpecialization<SimpleTaskScheduler<Kokkos::Nul, QueueType>> {
 public:
  // Note: Scheduler may be an incomplete type at class scope (but not inside
  // of the methods, obviously)

  using execution_space = Kokkos::Nul;
  using memory_space    = Kokkos::HostSpace;
  using scheduler_type  = SimpleTaskScheduler<Kokkos::Nul, QueueType>;
  using member_type =
      TaskTeamMemberAdapter<HostThreadTeamMember<Kokkos::Nul>,
                            scheduler_type>;

  static void execute(scheduler_type const& scheduler) {
    using task_base_type = typename scheduler_type::task_base_type;

    auto const& nul_execution_space = scheduler.get_execution_space();

    // Set default buffers
    nul_execution_space.impl_internal_space_instance()
        ->resize_thread_team_data(0,   /* global reduce buffer */
                                  512, /* team reduce buffer */
                                  0,   /* team shared buffer */
                                  0    /* thread local buffer */
        );

    auto& self = nul_execution_space.impl_internal_space_instance()
                     ->m_thread_team_data;

    auto& queue         = scheduler.queue();
    auto team_scheduler = scheduler.get_team_scheduler(0);

    member_type member(scheduler, self);

    auto current_task = OptionalRef<task_base_type>(nullptr);

  }

  static constexpr uint32_t get_max_team_count(
      execution_space const&) noexcept {
    return 1;
  }

  template <typename TaskType>
  static void get_function_pointer(typename TaskType::function_type& ptr,
                                   typename TaskType::destroy_type& dtor) {
    ptr  = TaskType::apply;
    dtor = TaskType::destroy;
  }
};

//----------------------------------------------------------------------------

template <class Scheduler>
class TaskQueueSpecializationConstrained<
    Scheduler,
    std::enable_if_t<std::is_same<typename Scheduler::execution_space,
                                  Kokkos::Nul>::value>> {
 public:
  // Note: Scheduler may be an incomplete type at class scope (but not inside
  // of the methods, obviously)

  using execution_space = Kokkos::Nul;
  using memory_space    = Kokkos::HostSpace;
  using scheduler_type  = Scheduler;
  using member_type =
      TaskTeamMemberAdapter<HostThreadTeamMember<Kokkos::Nullified>,
                            scheduler_type>;

  static void iff_single_thread_recursive_execute(
      scheduler_type const& scheduler) {
    using task_base_type = TaskBase;
    using queue_type     = typename scheduler_type::queue_type;

    task_base_type* const end = (task_base_type*)task_base_type::EndTag;

    execution_space nul_execution_space;
    auto& data = nul_execution_space.impl_internal_space_instance()
                     ->m_thread_team_data;
  }

  static void execute(scheduler_type const& scheduler) {
    using task_base_type = TaskBase;
    using queue_type     = typename scheduler_type::queue_type;

    task_base_type* const end = (task_base_type*)task_base_type::EndTag;

    execution_space nul_execution_space;

    // Set default buffers
    nul_execution_space.impl_internal_space_instance()
        ->resize_thread_team_data(0,   /* global reduce buffer */
                                  512, /* team reduce buffer */
                                  0,   /* team shared buffer */
                                  0    /* thread local buffer */
        );

      }
    }
  }

  template <typename TaskType>
  static void get_function_pointer(typename TaskType::function_type& ptr,
                                   typename TaskType::destroy_type& dtor) {
    ptr  = TaskType::apply;
    dtor = TaskType::destroy;
  }
};

extern template class TaskQueue<Kokkos::Nul,
                                typename Kokkos::Nul::memory_space>;

}  // namespace Impl
}  // namespace Kokkos

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------

#endif /* #if defined( KOKKOS_ENABLE_TASKDAG ) */
#endif /* #ifndef KOKKOS_IMPL_NUL_TASK_HPP */
