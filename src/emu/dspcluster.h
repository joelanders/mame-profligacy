// license:BSD-3-Clause
// Optional controller/DSP execution cluster.
//
// A driver may register three consecutive DSP execution devices. When the
// process-wide preference is enabled, one persistent worker owns those DSPs,
// every callback that touches them and the sound graph. The scheduler keeps
// shadow clocks, queues cycle budgets and captured control events in original
// order, and drains the queue before any synchronous observation. Output is
// identical to serial execution; only the executing thread changes.
#ifndef MAME_EMU_DSPCLUSTER_H
#define MAME_EMU_DSPCLUSTER_H

#pragma once

#include <array>
#include <cstdint>

class device_scheduler;
class device_execute_interface;
class device_t;

using cluster_callback = void (*)(void *, std::uint64_t, std::uint64_t);

// Hooks for code that touches cluster-owned state. cluster_post() returns false
// when no cluster is active on the calling thread, or when it is already the
// worker; the caller then performs the operation directly, as before.
bool cluster_post(device_scheduler &scheduler, void *object, cluster_callback callback,
	std::uint64_t first = 0, std::uint64_t second = 0);
bool cluster_consumer();
void cluster_fence();
void cluster_audio_boundary(const device_t &device);
void cluster_shutdown();
void cluster_refresh_policy();

// Called by the driver on the scheduler's owner thread during machine start.
// The devices must satisfy the same contract as configure_parallel_execution().
void configure_dsp_cluster(device_scheduler &scheduler, std::array<device_execute_interface *, 3> devices);

// Process-wide preference, applied by each registered scheduler between
// timeslices. Changing it at any time is safe and never alters emulated output.
enum class dsp_cluster_status : int
{
	off = 0,         // serial execution, as requested or not registered
	active = 1,      // worker thread running
	unavailable = 2  // requested, but the worker could not be started
};
void set_dsp_cluster_enabled(bool enabled);
bool dsp_cluster_enabled();
dsp_cluster_status get_dsp_cluster_status();

#endif // MAME_EMU_DSPCLUSTER_H
