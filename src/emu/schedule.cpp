// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/***************************************************************************

    schedule.cpp

    Core device execution and scheduling engine.

***************************************************************************/

#include "emu.h"
#include "debugger.h"
#include "dspcluster.h"

#include <atomic>
#include <cerrno>
#include <cstdlib>
#include <exception>
#include <functional>
#include <system_error>
#include <thread>

#if defined(__APPLE__)
#include <pthread.h>
#include <mach/mach.h>
#include <mach/thread_policy.h>
#include <os/os_sync_wait_on_address.h>
#endif

//**************************************************************************
//  CONTROLLER/DSP EXECUTION CLUSTER
//**************************************************************************

// Scheduled clocks belong to the coordinator; actual DSP clocks and state are
// touched only by the worker, or after an explicit drain while it is idle.
namespace {
std::atomic<bool> g_cluster_requested{false};
std::atomic<u64> g_cluster_generation{0};
std::atomic<int> g_cluster_status{int(dsp_cluster_status::off)};

// Progress is published every four guest audio frames, or earlier whenever a
// synchronous observation needs it.
constexpr unsigned cluster_batch_frames = 4;

// The sleep bit and generation share one atomic modification order. A publisher
// either sees the waiter asleep and wakes it, or changes the value before wait()
// can park; unlike a separate flag, this cannot lose a wakeup.
struct alignas(128) cluster_signal
{
	static constexpr u64 sleeping = u64(1) << 63;
	std::atomic<u64> value{0};
	u64 load() const { return value.load(std::memory_order_acquire) & ~sleeping; }
	void publish(u64 generation)
	{
		if (value.exchange(generation, std::memory_order_release) & sleeping)
		{
#if defined(__APPLE__)
			const int r = os_sync_wake_by_address_any(&value, sizeof(u64), OS_SYNC_WAKE_BY_ADDRESS_NONE);
			if (r < 0 && errno != ENOENT) std::abort();
#else
			value.notify_one();
#endif
		}
	}
	void wait(u64 old)
	{
		const u64 parked = old | sleeping;
		if (value.compare_exchange_strong(old, parked, std::memory_order_acq_rel) || old == parked)
		{
#if defined(__APPLE__)
			// Park directly in the kernel; no user-space spin budget.
			const int r = os_sync_wait_on_address(&value, parked, sizeof(u64), OS_SYNC_WAIT_ON_ADDRESS_NONE);
			if (r < 0 && errno != EINTR && errno != ENOMEM && errno != EFAULT) std::abort();
#else
			value.wait(parked, std::memory_order_acquire);
#endif
		}
	}
};

struct cluster_entry
{
	cluster_callback callback = nullptr;
	void *object = nullptr;
	u64 first = 0, second = 0;
	attotime time;
	std::array<u32, 3> cycles{};
	unsigned execute = 0;
};

struct alignas(128) cluster_chip
{
	device_execute_interface *device = nullptr;
	attotime scheduled_time;
	u64 scheduled_cycles = 0;
	u32 pending = 0;
	bool executing = true;
	std::function<void(u32, bool)> run;
	std::function<void()> validate;
};

struct cluster_group;
thread_local cluster_group *cluster_context = nullptr;
thread_local bool cluster_in_consumer = false;
thread_local attotime cluster_event_time;
thread_local device_execute_interface *cluster_device = nullptr;

struct cluster_group
{
	static constexpr unsigned capacity = 4096;
	device_scheduler &scheduler;
	std::array<cluster_chip, 3> chips;
	std::array<cluster_entry, capacity> queue;
	cluster_signal requested, completed;
	std::atomic<bool> stopping{false}, failed{false};
	std::exception_ptr error;
	std::thread thread;
	unsigned staged_frames = 0;
	u64 written = 0, published = 0, consumed = 0;
#if defined(__APPLE__)
	thread_time_constraint_policy_data_t policy{};
	bool use_policy = false;
	qos_class_t qos = QOS_CLASS_DEFAULT;
#endif

	explicit cluster_group(device_scheduler &s) : scheduler(s) { capture_policy(); }
	~cluster_group()
	{
		stopping.store(true, std::memory_order_release);
		requested.publish(written + 1);
		if (thread.joinable()) thread.join();
	}
	// The worker inherits the coordinator's real-time scheduling policy, so a
	// real-time producer never waits on an ordinary-priority DSP thread.
	void capture_policy()
	{
#if defined(__APPLE__)
		pthread_get_qos_class_np(pthread_self(), &qos, nullptr);
		mach_msg_type_number_t count = THREAD_TIME_CONSTRAINT_POLICY_COUNT;
		boolean_t is_default = false;
		use_policy = thread_policy_get(pthread_mach_thread_np(pthread_self()),
			THREAD_TIME_CONSTRAINT_POLICY, reinterpret_cast<thread_policy_t>(&policy),
			&count, &is_default) == KERN_SUCCESS && !is_default && policy.period;
#endif
	}
	void apply_policy()
	{
#if defined(__APPLE__)
		pthread_set_qos_class_self_np(qos, 0);
		if (use_policy)
			thread_policy_set(pthread_mach_thread_np(pthread_self()), THREAD_TIME_CONSTRAINT_POLICY,
				reinterpret_cast<thread_policy_t>(&policy), THREAD_TIME_CONSTRAINT_POLICY_COUNT);
#endif
	}
	bool start()
	{
		try { thread = std::thread([this] { loop(); }); }
		catch (const std::system_error &e)
		{
			std::fprintf(stderr, "DSP cluster worker creation failed; using serial execution: %s\n", e.what());
			return false;
		}
		return true;
	}
	void check_error()
	{
		if (failed.load(std::memory_order_acquire)) std::rethrow_exception(error);
	}
	void execute(const cluster_entry &entry)
	{
		struct restore
		{
			device_execute_interface *device = cluster_device;
			bool consumer = cluster_in_consumer;
			attotime time = cluster_event_time;
			~restore() { cluster_device = device; cluster_in_consumer = consumer; cluster_event_time = time; }
		} guard;
		cluster_in_consumer = true;
		cluster_event_time = entry.time;
		cluster_device = nullptr;
		for (unsigned i = 0; i < 3; ++i)
			if (entry.cycles[i])
			{
				cluster_device = chips[i].device;
				chips[i].run(entry.cycles[i], entry.execute & (1U << i));
			}
		cluster_device = nullptr;
		if (entry.callback) entry.callback(entry.object, entry.first, entry.second);
	}
	void loop()
	{
#if defined(__APPLE__)
		pthread_setname_np("Profligacy DSP cluster");
#endif
		cluster_context = this;
		apply_policy();
		for (;;)
		{
			u64 end;
			while ((end = requested.load()) == consumed && !stopping.load(std::memory_order_acquire))
				requested.wait(consumed);
			if (stopping.load(std::memory_order_acquire)) break;
			try
			{
				while (consumed != end)
				{
					execute(queue[consumed % capacity]);
					++consumed;
				}
			}
			catch (...)
			{
				error = std::current_exception();
				failed.store(true, std::memory_order_release);
				// A failure before consuming the first entry must still change
				// the wait word; publishing the unchanged cursor can lose a
				// wake between the coordinator's flag check and kernel park.
				completed.publish(cluster_signal::sleeping - 1);
				return;
			}
			completed.publish(consumed);
		}
	}
	void wait_for(u64 target)
	{
		for (u64 done; (done = completed.load()) < target; )
		{
			check_error();
			completed.wait(done);
		}
		check_error();
	}
	void publish()
	{
		staged_frames = 0;
		if (written == published) return;
		published = written;
		requested.publish(published);
	}
	void append(const cluster_entry &entry)
	{
		check_error();
		// A full queue applies backpressure by draining published work.
		if (written - completed.load() == capacity)
		{
			publish();
			wait_for(published);
		}
		queue[written++ % capacity] = entry;
	}
	void flush_work()
	{
		cluster_entry entry;
		bool any = false;
		for (unsigned i = 0; i < 3; ++i)
		{
			entry.cycles[i] = chips[i].pending;
			if (chips[i].executing) entry.execute |= 1U << i;
			any |= chips[i].pending != 0;
			chips[i].pending = 0;
		}
		if (any) append(entry);
	}
	void fence()
	{
		flush_work();
		publish();
		wait_for(published);
		for (auto &chip : chips) chip.validate();
	}
};

thread_local std::unique_ptr<cluster_group> cluster_owner;
thread_local u64 cluster_failed_generation = ~u64(0);

// Checked every timeslice; write only on change, so polling it stays cheap.
void cluster_set_status(dsp_cluster_status status)
{
	if (g_cluster_status.load(std::memory_order_relaxed) != int(status))
		g_cluster_status.store(int(status), std::memory_order_release);
}

} // anonymous namespace

bool cluster_consumer() { return cluster_in_consumer; }

bool cluster_post(device_scheduler &scheduler, void *object, cluster_callback callback, u64 first, u64 second)
{
	if (!cluster_context || cluster_in_consumer) return false;
	auto &group = *cluster_context;
	if (&scheduler != &group.scheduler) throw emu_fatalerror("DSP cluster scheduler mismatch");
	group.flush_work();
	cluster_entry entry;
	entry.callback = callback;
	entry.object = object;
	entry.first = first;
	entry.second = second;
	entry.time = scheduler.time();
	group.append(entry);
	return true;
}

void cluster_fence()
{
	if (cluster_context && !cluster_in_consumer) cluster_context->fence();
}

void cluster_audio_boundary(const device_t &device)
{
	if (!cluster_context || cluster_in_consumer) return;
	auto &group = *cluster_context;
	if (&device != &group.chips[2].device->device()) return;
	if (++group.staged_frames >= cluster_batch_frames)
	{
		group.flush_work();
		group.publish();
	}
}

void cluster_shutdown()
{
	if (!cluster_owner) return;
	// Drain every queued interval so the actual clocks reach their scheduled
	// endpoints before serial execution resumes; then join the worker.
	try { cluster_owner->fence(); }
	catch (...) { cluster_owner.reset(); cluster_context = nullptr; cluster_set_status(dsp_cluster_status::off); throw; }
	cluster_owner.reset();
	cluster_context = nullptr;
	cluster_set_status(dsp_cluster_status::off);
}

void cluster_refresh_policy()
{
	if (!cluster_context || cluster_in_consumer) return;
	auto &group = *cluster_context;
	group.fence();
	group.capture_policy();
	cluster_post(group.scheduler, &group, [](void *p, u64, u64) { static_cast<cluster_group *>(p)->apply_policy(); });
	group.fence();
}

void configure_dsp_cluster(device_scheduler &scheduler, std::array<device_execute_interface *, 3> devices)
{
	if (!devices[0] || !devices[1] || !devices[2] || devices[0] == devices[1] ||
		devices[1] == devices[2] || devices[0] == devices[2])
		throw emu_fatalerror("Invalid DSP cluster");
	scheduler.m_cluster_devices = devices;
}

void set_dsp_cluster_enabled(bool enabled)
{
	if (g_cluster_requested.exchange(enabled, std::memory_order_acq_rel) != enabled)
		g_cluster_generation.fetch_add(1, std::memory_order_acq_rel);
}

bool dsp_cluster_enabled()
{
	return g_cluster_requested.load(std::memory_order_acquire);
}

dsp_cluster_status get_dsp_cluster_status()
{
	return dsp_cluster_status(g_cluster_status.load(std::memory_order_acquire));
}



//**************************************************************************
//  DEBUGGING
//**************************************************************************

#define VERBOSE 0

#define LOG(...)  do { if (VERBOSE) machine().logerror(__VA_ARGS__); } while (0)
#define PRECISION



//**************************************************************************
//  CONSTANTS
//**************************************************************************

// internal trigger IDs
enum
{
	TRIGGER_INT         = -2000,
	TRIGGER_YIELDTIME   = -3000,
	TRIGGER_SUSPENDTIME = -4000
};



//**************************************************************************
//  EMU TIMER
//**************************************************************************

//-------------------------------------------------
//  emu_timer - constructor
//-------------------------------------------------

inline emu_timer::emu_timer() noexcept :
	m_scheduler(nullptr),
	m_next(nullptr),
	m_prev(nullptr),
	m_param(0),
	m_enabled(false),
	m_temporary(false),
	m_period(attotime::zero),
	m_start(attotime::zero),
	m_expire(attotime::never)
{
}


//-------------------------------------------------
//  ~emu_timer - destructor
//-------------------------------------------------

emu_timer::~emu_timer()
{
}


//-------------------------------------------------
//  init - completely initialize the state when
//  re-allocated as a non-device timer
//-------------------------------------------------

inline emu_timer &emu_timer::init(
		running_machine &machine,
		timer_expired_delegate &&callback,
		attotime start_delay,
		s32 param,
		bool temporary)
{
	// ensure the entire timer state is clean
	m_scheduler = &machine.scheduler();
	m_next = nullptr;
	m_prev = nullptr;
	m_callback = std::move(callback);
	m_param = param;
	m_temporary = temporary;
	m_period = attotime::never;

	m_start = m_scheduler->time();
	m_expire = m_start + start_delay;
	m_enabled = !m_expire.is_never();

	// if we're not temporary, register ourselves with the save state system
	if (!m_temporary)
		register_save(machine.save());

	// insert into the list
	m_scheduler->timer_list_insert(*this);
	if (this == m_scheduler->first_timer())
		m_scheduler->abort_timeslice();

	return *this;
}


//-------------------------------------------------
//  enable - enable/disable a timer
//-------------------------------------------------

bool emu_timer::enable(bool enable) noexcept
{
	assert(m_scheduler);

	// reschedule only if the state has changed
	const bool old = m_enabled;
	if (old != enable)
	{
		// set the enable flag
		m_enabled = enable;

		// remove the timer and insert back into the list
		m_scheduler->timer_list_remove(*this);
		m_scheduler->timer_list_insert(*this);
	}
	return old;
}


//-------------------------------------------------
//  adjust - adjust the time when this timer will
//  fire and specify a period for subsequent
//  firings
//-------------------------------------------------

void emu_timer::adjust(attotime start_delay, s32 param, const attotime &period) noexcept
{
	if (cluster_consumer()) fatalerror("DSP cluster worker attempted to mutate a scheduler timer");
	assert(m_scheduler);

	// if this is the callback timer, mark it modified
	if (m_scheduler->m_callback_timer == this)
		m_scheduler->m_callback_timer_modified = true;

	// compute the time of the next firing and insert into the list
	m_param = param;
	m_enabled = true;

	// clamp negative times to 0
	if (start_delay.seconds() < 0)
		start_delay = attotime::zero;

	// set the start and expire times
	m_start = m_scheduler->time();
	m_expire = m_start + start_delay;
	m_period = period;

	// remove and re-insert the timer in its new order
	m_scheduler->timer_list_remove(*this);
	m_scheduler->timer_list_insert(*this);

	// if this was inserted as the head, abort the current timeslice and resync
	if (this == m_scheduler->first_timer())
		m_scheduler->abort_timeslice();
}


//-------------------------------------------------
//  elapsed - return the amount of time since the
//  timer was started
//-------------------------------------------------

attotime emu_timer::elapsed() const noexcept
{
	assert(m_scheduler);

	return m_scheduler->time() - m_start;
}


//-------------------------------------------------
//  remaining - return the amount of time
//  remaining until the timer expires
//-------------------------------------------------

attotime emu_timer::remaining() const noexcept
{
	assert(m_scheduler);

	const attotime curtime = m_scheduler->time();
	if (curtime >= m_expire)
		return attotime::zero;
	return m_expire - curtime;
}


//-------------------------------------------------
//  register_save - register ourself with the save
//  state system
//-------------------------------------------------

void emu_timer::register_save(save_manager &manager)
{
	// determine our instance number - timers are indexed based on the callback function name
	int index = 0;
	std::string name = m_callback.name() ? m_callback.name() : "unnamed";
	for (const emu_timer *curtimer = m_scheduler->first_timer(); curtimer; curtimer = curtimer->m_next)
	{
		if (!curtimer->m_temporary)
		{
			if (curtimer->m_callback.name() && m_callback.name() && !strcmp(curtimer->m_callback.name(), m_callback.name()))
				index++;
			else if (!curtimer->m_callback.name() && !m_callback.name())
				index++;
		}
	}
	for (const emu_timer *curtimer = m_scheduler->m_inactive_timers; curtimer; curtimer = curtimer->m_next)
	{
		assert(!curtimer->m_temporary);

		if (curtimer->m_callback.name() && m_callback.name() && !strcmp(curtimer->m_callback.name(), m_callback.name()))
			index++;
		else if (!curtimer->m_callback.name() && !m_callback.name())
			index++;
	}

	// save the bits
	manager.save_item(nullptr, "timer", name.c_str(), index, NAME(m_param));
	manager.save_item(nullptr, "timer", name.c_str(), index, NAME(m_enabled));
	manager.save_item(nullptr, "timer", name.c_str(), index, NAME(m_period));
	manager.save_item(nullptr, "timer", name.c_str(), index, NAME(m_start));
	manager.save_item(nullptr, "timer", name.c_str(), index, NAME(m_expire));
	manager.save_item(nullptr, "timer", name.c_str(), index, NAME(m_index));
}


//-------------------------------------------------
//  schedule_next_period - schedule the next
//  period
//-------------------------------------------------

inline void emu_timer::schedule_next_period() noexcept
{
	assert(m_scheduler);

	// advance by one period
	m_start = m_expire;
	m_expire += m_period;

	// remove and re-insert us
	m_scheduler->timer_list_remove(*this);
	m_scheduler->timer_list_insert(*this);
}


//-------------------------------------------------
//  dump - dump internal state to a single output
//  line in the error log
//-------------------------------------------------

void emu_timer::dump() const
{
	assert(m_scheduler);

	m_scheduler->machine().logerror("%p: en=%d temp=%d exp=%15s start=%15s per=%15s param=%d", this, m_enabled, m_temporary, m_expire.as_string(PRECISION), m_start.as_string(PRECISION), m_period.as_string(PRECISION), m_param);
	if (!m_callback.name())
		m_scheduler->machine().logerror(" cb=NULL\n");
	else
		m_scheduler->machine().logerror(" cb=%s\n", m_callback.name());
}



//**************************************************************************
//  DEVICE SCHEDULER
//**************************************************************************

//-------------------------------------------------
//  device_scheduler - constructor
//-------------------------------------------------

device_scheduler::device_scheduler(running_machine &machine) :
	m_machine(machine),
	m_executing_device(nullptr),
	m_execute_list(nullptr),
	m_basetime(attotime::zero),
	m_timer_list(nullptr),
	m_inactive_timers(nullptr),
	m_callback_timer(nullptr),
	m_callback_timer_modified(false),
	m_callback_timer_expire_time(attotime::zero),
	m_suspend_changes_pending(true),
	m_quantum_minimum(ATTOSECONDS_IN_NSEC(1) / 1000)
{
	// append a single never-expiring timer so there is always one in the list
	// need to subvert it because it would naturally be inserted in the inactive list
	m_timer_list = &timer_list_remove(m_timer_allocator.alloc()->init(machine, timer_expired_delegate(), attotime::never, 0, true));

	assert(m_timer_list);
	assert(!m_timer_list->m_prev);
	assert(!m_timer_list->m_next);
	assert(!m_inactive_timers);

	// register global states
	machine.save().save_item(NAME(m_basetime));
	machine.save().register_presave(save_prepost_delegate(FUNC(device_scheduler::presave), this));
	machine.save().register_postload(save_prepost_delegate(FUNC(device_scheduler::postload), this));
}


//-------------------------------------------------
//  device_scheduler - destructor
//-------------------------------------------------

device_scheduler::~device_scheduler()
{
	try { cluster_shutdown(); }
	catch (...) { }
	// remove all timers
	while (m_inactive_timers)
		m_timer_allocator.reclaim(timer_list_remove(*m_inactive_timers));
	while (m_timer_list)
		m_timer_allocator.reclaim(timer_list_remove(*m_timer_list));
}


//-------------------------------------------------
//  time - return the current time
//-------------------------------------------------

attotime device_scheduler::time() const noexcept
{
	// Worker callbacks see the captured event time, or their own chip's clock.
	if (cluster_in_consumer)
		return cluster_device ? cluster_device->local_time() : cluster_event_time;

	// if we're currently in a callback, use the timer's expiration time as a base
	if (m_callback_timer != nullptr)
		return m_callback_timer_expire_time;

	// if we're executing as a particular CPU, use its local time as a base
	// otherwise, return the global base time
	return (m_executing_device != nullptr) ? m_executing_device->local_time() : m_basetime;
}

device_execute_interface *device_scheduler::currently_executing() const noexcept
{
	return cluster_in_consumer ? cluster_device : m_executing_device;
}


//-------------------------------------------------
//  can_save - return true if it's safe to save
//  (i.e., no temporary timers outstanding)
//-------------------------------------------------

bool device_scheduler::can_save() const
{
	// if any live temporary timers exit, fail
	for (emu_timer *timer = m_timer_list; timer; timer = timer->m_next)
	{
		if (timer->m_temporary && !timer->expire().is_never())
		{
			machine().logerror("Failed save state attempt due to anonymous timers:\n");
			dump_timers();
			return false;
		}
	}

	// otherwise, we're good
	return true;
}


//-------------------------------------------------
//  apply_suspend_changes - applies suspend/resume
//  changes to all device_execute_interfaces
//-------------------------------------------------

inline void device_scheduler::apply_suspend_changes()
{
	u32 suspendchanged = 0;
	for (device_execute_interface *exec = m_execute_list; exec != nullptr; exec = exec->m_nextexec)
	{
		suspendchanged |= exec->m_suspend ^ exec->m_nextsuspend;
		exec->m_suspend = exec->m_nextsuspend;
		exec->m_nextsuspend &= ~SUSPEND_REASON_TIMESLICE;
		exec->m_eatcycles = exec->m_nexteatcycles;
	}

	// recompute the execute list if any CPUs changed their suspension state
	if (suspendchanged != 0)
		rebuild_execute_list();
	else
		m_suspend_changes_pending = false;
}


//-------------------------------------------------
//  timeslice - execute all devices for a single
//  timeslice
//-------------------------------------------------

void device_scheduler::timeslice()
{
	bool call_debugger = ((machine().debug_flags & DEBUG_FLAG_ENABLED) != 0);
	update_dsp_cluster(call_debugger || g_profiler.enabled());

	// build the execution list if we don't have one yet
	if (UNEXPECTED(m_execute_list == nullptr))
		rebuild_execute_list();

	// if the current quantum has expired, find a new one
	while (m_basetime >= m_quantum_list.first()->m_expire)
		m_quantum_allocator.reclaim(m_quantum_list.detach_head());

	// loop until we hit the next timer
	while (m_basetime < m_timer_list->m_expire)
	{
		// by default, assume our target is the end of the next quantum
		attotime target(m_basetime + attotime(0, m_quantum_list.first()->m_actual));

		// however, if the next timer is going to fire before then, override
		if (m_timer_list->m_expire < target)
			target = m_timer_list->m_expire;

		LOG("------------------\n");
		LOG("cpu_timeslice: target = %s\n", target.as_string(PRECISION));

		// do we have pending suspension changes?
		if (m_suspend_changes_pending)
			apply_suspend_changes();

		// loop over all CPUs
		for (device_execute_interface *exec = m_execute_list; exec != nullptr; exec = exec->m_nextexec)
		{
			if (cluster_context && advance_dsp_cluster(exec, target)) continue;
			// only process if this CPU is executing or truly halted (not yielding)
			// and if our target is later than the CPU's current time (coarse check)
			if (EXPECTED((exec->m_suspend == 0 || exec->m_eatcycles) && target.seconds() >= exec->m_localtime.seconds()))
			{
				// compute how many attoseconds to execute this CPU
				attoseconds_t delta = target.attoseconds() - exec->m_localtime.attoseconds();
				if (delta < 0 && target.seconds() > exec->m_localtime.seconds())
					delta += ATTOSECONDS_PER_SECOND;
				assert(delta == (target - exec->m_localtime).as_attoseconds());

				if (exec->m_attoseconds_per_cycle == 0)
				{
					exec->m_localtime = target;
				}
				// if we have enough for at least 1 cycle, do the math
				else if (delta >= exec->m_attoseconds_per_cycle)
				{
					// compute how many cycles we want to execute
					int ran = exec->m_cycles_running = divu_64x32(u64(delta) >> exec->m_divshift, exec->m_divisor);
					LOG("  cpu '%s': %d (%d cycles)\n", exec->device().tag(), delta, exec->m_cycles_running);

					// if we're not suspended, actually execute
					if (exec->m_suspend == 0)
					{
						auto profile = g_profiler.start(exec->m_profiler);

						// note that this global variable cycles_stolen can be modified
						// via the call to cpu_execute
						exec->m_cycles_stolen = 0;
						m_executing_device = exec;
						*exec->m_icountptr = exec->m_cycles_running;
						if (!call_debugger)
							exec->run();
						else
						{
							exec->debugger_start_cpu_hook(target);
							exec->run();
							exec->debugger_stop_cpu_hook();
						}

						// adjust for any cycles we took back
						assert(ran >= *exec->m_icountptr);
						ran -= *exec->m_icountptr;
						assert(ran >= exec->m_cycles_stolen);
						ran -= exec->m_cycles_stolen;
					}

					// account for these cycles
					exec->m_totalcycles += ran;

					// update the local time for this CPU
					attotime deltatime;
					if (EXPECTED(ran < exec->m_cycles_per_second))
						deltatime = attotime(0, exec->m_attoseconds_per_cycle * ran);
					else
					{
						u32 remainder;
						s32 secs = divu_64x32_rem(ran, exec->m_cycles_per_second, remainder);
						deltatime = attotime(secs, u64(remainder) * exec->m_attoseconds_per_cycle);
					}
					assert(deltatime >= attotime::zero);
					exec->m_localtime += deltatime;
					LOG("         %d ran, %d total, time = %s\n", ran, s32(exec->m_totalcycles), exec->m_localtime.as_string(PRECISION));

					// if the new local CPU time is less than our target, move the target up, but not before the base
					if (exec->m_localtime < target)
					{
						target = std::max(exec->m_localtime, m_basetime);
						LOG("         (new target)\n");
					}
				}
			}
		}
		m_executing_device = nullptr;

		// update the base time
		m_basetime = target;
	}

	// execute timers
	execute_timers();
}


//-------------------------------------------------
//  update_dsp_cluster - apply the multi-core DSP
//  preference between timeslices, where no
//  device is executing
//-------------------------------------------------

void device_scheduler::update_dsp_cluster(bool blocked)
{
	if (!m_cluster_devices[0]) return;
	const bool requested = g_cluster_requested.load(std::memory_order_acquire);
	const u64 generation = g_cluster_generation.load(std::memory_order_acquire);
	if (blocked || !requested || cluster_failed_generation == generation)
	{
		if (cluster_owner) cluster_shutdown();
		cluster_set_status(requested && !blocked ? dsp_cluster_status::unavailable : dsp_cluster_status::off);
		return;
	}
	if (cluster_owner) return;

	// Shadow clocks start from the actual clocks; nothing is queued yet.
	auto group = std::make_unique<cluster_group>(*this);
	for (unsigned i = 0; i < 3; ++i)
	{
		device_execute_interface *exec = m_cluster_devices[i];
		auto &chip = group->chips[i];
		chip.device = exec;
		chip.scheduled_time = exec->m_localtime;
		chip.scheduled_cycles = exec->m_totalcycles;
		chip.run = [exec](u32 cycles, bool execute) {
			if (cycles >= u32(exec->m_cycles_per_second))
				throw emu_fatalerror("DSP cluster interval exceeds the bounded execution contract");
			exec->m_cycles_running = cycles;
			exec->m_cycles_stolen = 0;
			*exec->m_icountptr = execute ? cycles : 0;
			if (execute) exec->run();
			if (*exec->m_icountptr || exec->m_cycles_stolen)
				throw emu_fatalerror("DSP cluster device failed to consume its full cycle budget");
			exec->m_totalcycles += cycles;
			exec->m_localtime += attotime(0, exec->m_attoseconds_per_cycle * cycles);
		};
		chip.validate = [exec, &chip] {
			if (exec->m_localtime != chip.scheduled_time || exec->m_totalcycles != chip.scheduled_cycles)
				throw emu_fatalerror("DSP cluster device %s changed its scheduled endpoint", exec->device().tag());
		};
	}
	if (!group->start())
	{
		cluster_failed_generation = generation;
		cluster_set_status(dsp_cluster_status::unavailable);
		return;
	}
	cluster_owner = std::move(group);
	cluster_context = cluster_owner.get();
	cluster_set_status(dsp_cluster_status::active);
}


//-------------------------------------------------
//  advance_dsp_cluster - queue a cluster DSP's
//  interval instead of executing it here
//-------------------------------------------------

bool device_scheduler::advance_dsp_cluster(device_execute_interface *exec, attotime &target)
{
	auto &group = *cluster_context;
	const int index = exec == group.chips[0].device ? 0 : exec == group.chips[1].device ? 1 :
		exec == group.chips[2].device ? 2 : -1;
	if (index < 0) return false;

	// Same cycle computation as the serial loop, applied to the shadow clock.
	auto &chip = group.chips[index];
	const auto local = chip.scheduled_time;
	if ((exec->m_suspend == 0 || exec->m_eatcycles) && target.seconds() >= local.seconds())
	{
		attoseconds_t delta = target.attoseconds() - local.attoseconds();
		if (delta < 0 && target.seconds() > local.seconds()) delta += ATTOSECONDS_PER_SECOND;
		if (!exec->m_attoseconds_per_cycle)
			throw emu_fatalerror("DSP cluster device clock stopped");
		if (delta >= exec->m_attoseconds_per_cycle)
		{
			const int ran = divu_64x32(u64(delta) >> exec->m_divshift, exec->m_divisor);
			const bool execute = exec->m_suspend == 0;
			if ((chip.pending && chip.executing != execute) || u32(ran) > u32(INT_MAX) - chip.pending)
				group.flush_work();
			chip.executing = execute;
			chip.pending += ran;
			chip.scheduled_cycles += ran;
			chip.scheduled_time += attotime(0, exec->m_attoseconds_per_cycle * ran);
			if (chip.scheduled_time < target) target = std::max(chip.scheduled_time, m_basetime);
		}
	}
	return true;
}


//-------------------------------------------------
//  abort_timeslice - abort execution for the
//  current timeslice
//-------------------------------------------------

void device_scheduler::abort_timeslice() noexcept
{
	if (m_executing_device != nullptr)
		m_executing_device->abort_timeslice();
}


//-------------------------------------------------
//  trigger - generate a global trigger
//-------------------------------------------------

void device_scheduler::trigger(int trigid, const attotime &after)
{
	// ensure we have a list of executing devices
	if (m_execute_list == nullptr)
		rebuild_execute_list();

	if (after != attotime::zero)
	{
		// if we have a non-zero time, schedule a timer
		timer_set(after, timer_expired_delegate(FUNC(device_scheduler::timed_trigger), this), trigid);
	}
	else
	{
		// send the trigger to everyone who cares
		for (device_execute_interface *exec = m_execute_list; exec; exec = exec->m_nextexec)
			exec->trigger(trigid);
	}
}


//-------------------------------------------------
//  add_quantum - add a scheduling quantum;
//  the smallest active one is the one that is in use
//-------------------------------------------------

void device_scheduler::add_quantum(const attotime &quantum, const attotime &duration)
{
	assert(quantum.seconds() == 0);

	attotime curtime = time();
	attotime expire = curtime + duration;
	const attoseconds_t quantum_attos = quantum.attoseconds();

	// figure out where to insert ourselves, expiring any quanta that are out-of-date
	quantum_slot *insert_after = nullptr;
	quantum_slot *next;
	for (quantum_slot *quant = m_quantum_list.first(); quant != nullptr; quant = next)
	{
		// if this quantum is expired, nuke it
		next = quant->next();
		if (curtime >= quant->m_expire)
			m_quantum_allocator.reclaim(m_quantum_list.detach(*quant));

		// if this quantum is shorter than us, we need to be inserted afterwards
		else if (quant->m_requested <= quantum_attos)
			insert_after = quant;
	}

	// if we found an exact match, just take the maximum expiry time
	if (insert_after != nullptr && insert_after->m_requested == quantum_attos)
		insert_after->m_expire = std::max(insert_after->m_expire, expire);

	// otherwise, allocate a new quantum and insert it after the one we picked
	else
	{
		quantum_slot &quant = *m_quantum_allocator.alloc();
		quant.m_requested = quantum_attos;
		quant.m_actual = std::max(quantum_attos, m_quantum_minimum);
		quant.m_expire = expire;
		m_quantum_list.insert_after(quant, insert_after);
	}
}


//-------------------------------------------------
//  perfect_quantum - add a (temporary) minimum
//  scheduling quantum to boost the interleave
//-------------------------------------------------

void device_scheduler::perfect_quantum(const attotime &duration)
{
	add_quantum(attotime::zero, duration);
}


//-------------------------------------------------
//  timer_alloc - allocate a global non-device
//  timer and return a pointer
//-------------------------------------------------

emu_timer *device_scheduler::timer_alloc(timer_expired_delegate callback)
{
	return &m_timer_allocator.alloc()->init(machine(), std::move(callback), attotime::never, 0, false);
}


//-------------------------------------------------
//  timer_set - allocate an anonymous non-device
//  timer and set it to go off after the given
//  amount of time
//-------------------------------------------------

void device_scheduler::timer_set(const attotime &duration, timer_expired_delegate callback, s32 param)
{
	[[maybe_unused]] emu_timer &timer = m_timer_allocator.alloc()->init(
			machine(),
			std::move(callback),
			duration,
			param,
			true);
	assert(!timer.m_expire.is_never()); // this is not handled
}


//-------------------------------------------------
//  synchronize - allocate an anonymous non-device
//  timer and set it to go off as soon as possible
//-------------------------------------------------

void device_scheduler::synchronize(timer_expired_delegate callback, s32 param)
{
	m_timer_allocator.alloc()->init(
			machine(),
			std::move(callback),
			attotime::zero,
			param,
			true);
}


//-------------------------------------------------
//  eat_all_cycles - eat a ton of cycles on all
//  CPUs to force a quick exit
//-------------------------------------------------

void device_scheduler::eat_all_cycles()
{
	for (device_execute_interface *exec = m_execute_list; exec != nullptr; exec = exec->m_nextexec)
		exec->eat_cycles(1000000000);
}


//-------------------------------------------------
//  timed_trigger - generate a trigger after a
//  given amount of time
//-------------------------------------------------

void device_scheduler::timed_trigger(s32 param)
{
	trigger(param);
}


//-------------------------------------------------
//  presave - before creating a save state
//-------------------------------------------------

void device_scheduler::presave()
{
	cluster_shutdown();
	// report the timer state after a log
	LOG("Prior to saving state:\n");
	u32 index = 0;
	for (emu_timer *timer = m_timer_list; timer; timer = timer->m_next)
		timer->m_index = index++;
#if VERBOSE
	dump_timers();
#endif
}


//-------------------------------------------------
//  postload - after loading a save state
//-------------------------------------------------

void device_scheduler::postload()
{
	// Loading replaces device state behind the worker; the product never loads
	// MAME states, and a cluster must not be active if something else does.
	if (cluster_context) throw emu_fatalerror("DSP cluster must be disabled before loading state");
	// remove all timers and make a private list of permanent ones
	emu_timer *private_list = nullptr;
	while (m_inactive_timers)
	{
		emu_timer &timer = *m_inactive_timers;
		assert(!timer.m_temporary);

		timer_list_remove(timer).m_next = private_list;
		private_list = &timer;
	}
	while (m_timer_list->m_next)
	{
		emu_timer &timer = *m_timer_list;

		if (timer.m_temporary)
		{
			assert(!timer.expire().is_never());

			// temporary timers go away entirely (except our special never-expiring one)
			timer.m_callback.reset();
			m_timer_allocator.reclaim(timer_list_remove(timer));
		}
		else
		{
			// permanent ones get added to our private list
			timer_list_remove(timer).m_next = private_list;
			private_list = &timer;
		}
	}

	// special dummy timer
	assert(!m_timer_list->m_enabled);
	assert(m_timer_list->m_temporary);
	assert(m_timer_list->m_expire.is_never());

	// now re-insert them; this effectively re-sorts them by time
	while (private_list)
	{
		emu_timer &timer = *private_list;
		private_list = timer.m_next;
		timer_list_insert<true>(timer);
	}

	m_suspend_changes_pending = true;
	rebuild_execute_list();

	// report the timer state after a log
	LOG("After resetting/reordering timers:\n");
#if VERBOSE
	dump_timers();
#endif
}


//-------------------------------------------------
//  compute_perfect_interleave - compute the
//  "perfect" interleave interval
//-------------------------------------------------

void device_scheduler::compute_perfect_interleave()
{
	// ensure we have a list of executing devices
	if (m_execute_list == nullptr)
		rebuild_execute_list();

	// start with the first one
	device_execute_interface *first = m_execute_list;
	if (first != nullptr)
	{
		// start with a huge time factor and find the 2nd smallest cycle time
		attoseconds_t smallest = first->minimum_quantum();
		attoseconds_t perfect = ATTOSECONDS_PER_SECOND - 1;
		for (device_execute_interface *exec = first->m_nextexec; exec != nullptr; exec = exec->m_nextexec)
		{
			// find the 2nd smallest cycle interval
			attoseconds_t curquantum = exec->minimum_quantum();
			if (curquantum < smallest)
			{
				perfect = smallest;
				smallest = curquantum;
			}
			else if (curquantum < perfect)
				perfect = curquantum;
		}

		// if this is a new minimum quantum, apply it
		if (m_quantum_minimum != perfect)
		{
			// adjust all the actuals; this doesn't affect the current
			m_quantum_minimum = perfect;
			for (quantum_slot &quant : m_quantum_list)
				quant.m_actual = std::max(quant.m_requested, m_quantum_minimum);
		}
	}
}


//-------------------------------------------------
//  rebuild_execute_list - rebuild the list of
//  executing CPUs, moving suspended CPUs to the
//  end
//-------------------------------------------------

void device_scheduler::rebuild_execute_list()
{
	// if we haven't yet set a scheduling quantum, do it now
	if (m_quantum_list.empty())
	{
		// set the core scheduling quantum, ensuring it's no longer than 60Hz
		attotime min_quantum = machine().config().maximum_quantum(attotime::from_hz(60));

		// if the configuration specifies a device to make perfect, pick that as the minimum
		device_execute_interface *const exec(machine().config().perfect_quantum_device());
		if (exec)
			min_quantum = (std::min)(attotime(0, exec->minimum_quantum()), min_quantum);

		// inform the timer system of our decision
		add_quantum(min_quantum, attotime::never);
	}

	// start with an empty list
	device_execute_interface **active_tailptr = &m_execute_list;
	*active_tailptr = nullptr;

	// also make an empty list of suspended devices
	device_execute_interface *suspend_list = nullptr;
	device_execute_interface **suspend_tailptr = &suspend_list;

	// iterate over all devices
	for (device_execute_interface &exec : execute_interface_enumerator(machine().root_device()))
	{
		// append to the appropriate list
		exec.m_nextexec = nullptr;
		if (exec.m_suspend == 0)
		{
			*active_tailptr = &exec;
			active_tailptr = &exec.m_nextexec;
		}
		else
		{
			*suspend_tailptr = &exec;
			suspend_tailptr = &exec.m_nextexec;
		}
	}

	// append the suspend list to the end of the active list
	*active_tailptr = suspend_list;
}


//-------------------------------------------------
//  timer_list_insert - insert a new timer into
//  the list at the appropriate location
//-------------------------------------------------

template <bool CheckIndex>
inline emu_timer &device_scheduler::timer_list_insert(emu_timer &timer)
{
	// disabled timers never expire
	if (!timer.m_expire.is_never() && timer.m_enabled)
	{
		// loop over the timer list
		emu_timer *prevtimer = nullptr;
		for (emu_timer *curtimer = m_timer_list; curtimer; prevtimer = curtimer, curtimer = curtimer->m_next)
		{
			// if the current list entry expires after us, we should be inserted before it
			bool const here =
					(curtimer->m_expire > timer.m_expire) ||
					(CheckIndex && !(curtimer->m_expire < timer.m_expire) && (curtimer->m_index > timer.m_index));
			if (here)
			{
				// link the new guy in before the current list entry
				timer.m_prev = prevtimer;
				timer.m_next = curtimer;

				if (prevtimer)
					prevtimer->m_next = &timer;
				else
					m_timer_list = &timer;

				curtimer->m_prev = &timer;
				return timer;
			}
		}

		// need to insert after the last one
		if (prevtimer)
			prevtimer->m_next = &timer;
		else
			m_timer_list = &timer;

		timer.m_prev = prevtimer;
		timer.m_next = nullptr;
	}
	else
	{
		// keep inactive timers in a separate list
		if (m_inactive_timers)
			m_inactive_timers->m_prev = &timer;

		timer.m_next = m_inactive_timers;
		timer.m_prev = nullptr;

		m_inactive_timers = &timer;
	}
	return timer;
}


//-------------------------------------------------
//  timer_list_remove - remove a timer from the
//  linked list
//-------------------------------------------------

inline emu_timer &device_scheduler::timer_list_remove(emu_timer &timer)
{
	// remove it from the list
	if (timer.m_prev)
	{
		timer.m_prev->m_next = timer.m_next;
	}
	else if (&timer == m_timer_list)
	{
		m_timer_list = timer.m_next;
	}
	else
	{
		assert(&timer == m_inactive_timers);
		m_inactive_timers = timer.m_next;
	}

	if (timer.m_next)
		timer.m_next->m_prev = timer.m_prev;

	return timer;
}


//-------------------------------------------------
//  execute_timers - execute timers that are due
//-------------------------------------------------

inline void device_scheduler::execute_timers()
{
	LOG("execute_timers: new=%s head->expire=%s\n", m_basetime.as_string(PRECISION), m_timer_list->m_expire.as_string(PRECISION));

	// now process any timers that are overdue
	while (m_timer_list->m_expire <= m_basetime)
	{
		// if this is a one-shot timer, disable it now
		emu_timer &timer = *m_timer_list;
		bool was_enabled = timer.m_enabled;
		if (timer.m_period.is_zero() || timer.m_period.is_never())
			timer.m_enabled = false;

		// set the global state of which callback we're in
		m_callback_timer_modified = false;
		m_callback_timer = &timer;
		m_callback_timer_expire_time = timer.m_expire;

		// call the callback
		if (was_enabled)
		{
			auto profile = g_profiler.start(PROFILER_TIMER_CALLBACK);

			if (!timer.m_callback.isnull())
			{
				LOG("execute_timers: timer callback %s\n", timer.m_callback.name());
				timer.m_callback(timer.m_param);
			}
		}

		// reset or remove the timer, but only if it wasn't modified during the callback
		if (!m_callback_timer_modified)
		{
			if (!timer.m_temporary)
			{
				// if the timer is not temporary, reschedule it
				timer.schedule_next_period();
			}
			else
			{
				// otherwise, remove it now
				timer.m_callback.reset();
				m_timer_allocator.reclaim(timer_list_remove(timer));
			}
		}
	}

	// clear the callback timer global
	m_callback_timer = nullptr;
}


//-------------------------------------------------
//  dump_timers - dump the current timer state
//-------------------------------------------------

void device_scheduler::dump_timers() const
{
	machine().logerror("=============================================\n");
	machine().logerror("Timer Dump: Time = %15s\n", time().as_string(PRECISION));
	for (emu_timer *timer = m_timer_list; timer; timer = timer->m_next)
		timer->dump();
	for (emu_timer *timer = m_inactive_timers; timer; timer = timer->m_next)
		timer->dump();
	machine().logerror("=============================================\n");
}
