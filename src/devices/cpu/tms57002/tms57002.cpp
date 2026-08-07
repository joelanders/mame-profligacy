// license:BSD-3-Clause
// copyright-holders:Olivier Galibert
/***************************************************************************

    tms57002.cpp

    TMS57002 "DASP" emulator.

***************************************************************************/

#include "emu.h"
#include "tms57002.h"
#include "57002dsm.h"
#include "tms57002_jit.h"   // DSP dynarec (KPROP_DSP_PERFRAME=4 live seam)

#include <filesystem>
#include <fstream>
#include <memory>
#include <limits>
#include <cmath>
#include <string>

DEFINE_DEVICE_TYPE(TMS57002, tms57002_device, "tms57002", "Texas Instruments TMS57002 \"DASP\"")

void tms57002_device::internal_pgm(address_map &map)
{
	map(0x00, 0xff).ram();
}

tms57002_device::tms57002_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: cpu_device(mconfig, TMS57002, tag, owner, clock)
	, device_sound_interface(mconfig, *this)
	, macc(0), macc_read(0), macc_write(0)
	, m_serial_input_latch{0, 0, 0, 0}
	, m_serial_input_active{0, 0, 0, 0}
	, m_serial_input_frame{0, 0, 0, 0}
	, m_serial_input_prev_frame{0, 0, 0, 0}
	, m_serial_input_pending{0, 0, 0, 0}
	, m_serial_output_frame{0, 0, 0, 0}
	, m_serial_output_build{0, 0, 0, 0}
	, m_serial_input_valid(0)
	, m_serial_input_active_valid(0)
	, m_serial_input_prev_valid(0)
	, m_serial_input_pending_valid(0)
	, m_serial_output_pending_valid(0)
	, m_pending_pre_transfer(pending_pre_transfer_type::none)
	, m_pending_pre_transfer_addr(0)
	, m_pending_pre_transfer_value(0)
	, m_pending_pre_transfer_pc(0)
	, m_serial_input_timing_mode(serial_input_timing_mode::same_sample)
	, m_serial_frame_mode(serial_frame_mode::snapshot)
	, m_serial_input_immediate_side_mode(serial_input_immediate_side_mode::auto_select)
	, m_serial_frame_clocks(0)
	, m_serial_exec_halfcycles(0)
	, m_serial_input_pending_halfcycle(0)
	, m_serial_output_pending_halfcycle(0)
	, m_serial_output_pending_halfcycle_left(0)
	, m_serial_input_pending_halfcycle_override(-1)
	, m_serial_output_pending_halfcycle_override(-1)
	, m_serial_output_pending_halfcycle_left_override(-1)
		, m_serial_output_write_handoff(false)
		, m_sync_polarity_rising(false)
		, m_serial_output_muted(false)
	, m_serial_frame_flip_input(false)
	, m_serial_frame_flip_output(false)
	, m_stream_output_raw(false)
	, m_stream_sync_enabled(true)
	, m_smhc_post_slot(false)
	, m_ca_inc_delayed(false)
	, m_abs_saturate(false)
	, m_sound_updates(0)
	, m_dready_line(-1)
	, m_pc0_line(-1)
	, m_empty_line(-1)
	, m_update_timing_mode(update_timing_mode::normal), m_update_timing_addr_any(false)
	, m_update_active(false)
	, m_update_address_run(0)
	, m_debug_cmem_force(false)
	, m_debug_ca_mpy_extra_shift(0)
	, m_debug_dc_mpy_extra_shift(0)
	, m_debug_smhc_trunc_bits(0)
	, m_debug_lmhd_srbd_forward(false)
	, m_debug_mpy_smhd_forward(false)
	, m_debug_mpy_smhd_forward_pc(-1)
	, m_debug_smhd_raw_path(false)
	, m_debug_smld_raw_path(false)
	, st0(0), st1(0), sti(0), txrd(0)
	, m_dready_callback(*this)
	, m_pc0_callback(*this)
	, m_empty_callback(*this)
	, program_config("program", ENDIANNESS_LITTLE, 32, 8, -2, address_map_constructor(FUNC(tms57002_device::internal_pgm), this))
	, data_config("data", ENDIANNESS_LITTLE, 8, 20)
{
}

std::unique_ptr<util::disasm_interface> tms57002_device::create_disassembler()
{
	return std::make_unique<tms57002_disassembler>();
}

void tms57002_device::pload_w(int state)
{
	u8 olds = sti;
	if (state)
		sti &= ~IN_PLOAD;
	else
		sti |= IN_PLOAD;
	if (olds ^ sti)
	{
		m_program_version++;   // M4: PLOAD (re)loads microcode + flushes the decode cache -> invalidate any JIT frame keyed on the old program
		if (sti & IN_PLOAD)
		{
			hidx = 0;
			pc = 0;
			ca = 0;
			// Fresh program uploads can replace only the leading words, so any
			// cached decode chain from the previous image becomes invalid here.
			cache_flush();
			sti &= ~(SU_MASK);
		}
	}
}

void tms57002_device::cload_w(int state)
{
	u8 olds = sti;
	if (state)
		sti &= ~IN_CLOAD;
	else
		sti |= IN_CLOAD;
	if (olds ^ sti)
	{
		if (sti & IN_CLOAD)
		{
			hidx = 0;
			sti &= ~SU_CVAL;
			//ca = 0; // Seems extremely dubious
		}
		else
		{
			hidx = 0;
			sti &= ~SU_CVAL;
		}
	}
}

// Out-of-line: m_jit's unique_ptr destructor needs the complete tms57002::Jit type, which only
// this TU has (tms57002_jit.h is included above).
tms57002_device::~tms57002_device() = default;

void tms57002_device::device_post_load()
{
	// A restored savestate carries the device's architectural state but NOT the Jit's cross-frame
	// state (pending-partial flag, active cache entry, baked decode-cache priming). Drop the Jit
	// wholesale — it lazily recreates and recompiles at the next hook, which is always safe.
	m_jit.reset();
}

void tms57002_device::device_reset()
{
	sti = (sti & ~(SU_MASK|S_READ|S_WRITE|S_BRANCH|S_HOST|S_UPDATE)) | (SU_ST0|S_IDLE);
	pc = 0;
	ca = 0;
	hidx = 0;
	id = 0;
	ba0 = 0;
	ba1 = 0;
	sa = 0;
	rptc = 0;
	rptc_next = 0;
	update_counter_tail = 0;
	update_counter_head = 0;
	std::fill(std::begin(update_sa), std::end(update_sa), 0U);
	m_update_active = false;
	m_update_address_run = 0;
	std::fill(std::begin(m_update_address_run_for_entry), std::end(m_update_address_run_for_entry), 0U);
	m_sound_updates = 0;
	m_dready_line = -1;
	m_pc0_line = -1;
	m_empty_line = -1;
	m_update_enqueue_su.fill(0);
	m_update_read_delay_seen.fill(0);
	std::fill(std::begin(si), std::end(si), 0U);
	std::fill(std::begin(so), std::end(so), 0U);
	m_serial_input_latch.fill(0U);
	m_serial_input_active.fill(0U);
	m_serial_input_frame.fill(0U);
	m_serial_input_prev_frame.fill(0U);
	m_serial_input_pending.fill(0U);
	m_serial_output_frame.fill(0U);
	m_serial_output_build.fill(0U);
	m_serial_input_valid = 0;
	m_serial_input_active_valid = 0;
	m_serial_input_prev_valid = 0;
	m_serial_input_pending_valid = 0;
	m_serial_output_pending_valid = 0;
	m_pending_pre_transfer = pending_pre_transfer_type::none;
	m_pending_pre_transfer_addr = 0;
	m_pending_pre_transfer_value = 0;
	m_serial_exec_halfcycles = 0;
	m_serial_input_pending_halfcycle = 0;
	m_serial_output_pending_halfcycle = 0;
	m_serial_output_pending_halfcycle_left = 0;
	st0 &= ~(ST0_INCS | ST0_DIRI | ST0_FI | ST0_SIM | ST0_PLRI |
			ST0_PBCI | ST0_DIRO | ST0_FO | ST0_SOM | ST0_PLRO |
			ST0_PBCO | ST0_CNS);
	st1 &= ~(ST1_AOV | ST1_SFAI | ST1_SFAO | ST1_AOVM | ST1_MOVM | ST1_MOV |
			ST1_SFMA | ST1_SFMO | ST1_RND | ST1_CRM | ST1_DBP);
	update_dready();
	update_pc0();
	update_empty();

	xba = 0;
	xoa = 0;
	xm_adr = 0;
	xm_cycles = 0;
	xm_fetches = 0;
	cache_flush();
}

void tms57002_device::data_w(u8 data)
{
	switch (sti & (IN_PLOAD|IN_CLOAD))
	{
	case 0:
		hidx = 0;
		sti &= ~SU_CVAL;
		break;
	case IN_PLOAD:
		host[hidx++] = data;
		if (hidx >= 3)
		{
			u32 val = (host[0]<<16) | (host[1]<<8) | host[2];
			hidx = 0;

			switch (sti & SU_MASK)
			{
			case SU_ST0:
				st0 = val;
				sti = (sti & ~SU_MASK) | SU_ST1;
				break;
			case SU_ST1:
				st1 = val;
				sti = (sti & ~SU_MASK) | SU_PRG;
				break;
			case SU_PRG:
				program->write_dword(pc++, val);
				update_pc0();
				break;
			}
		}
		break;
	case IN_CLOAD:
		if (sti & SU_CVAL)
		{
			host[hidx++] = data;
			if (hidx >= 4)
			{
				u32 val = (host[0]<<24) | (host[1]<<16) | (host[2]<<8) | host[3];
				if (m_update_timing_mode == update_timing_mode::eager && update_timing_selected(sa))
				{
					write_cmem_direct(sa, val, "HOST_EAGER");
				}
				else
				{
					update_sa[update_counter_head] = sa;
					m_update_address_run_for_entry[update_counter_head] = m_update_address_run;
					update[update_counter_head] = val;
					m_update_enqueue_su[update_counter_head] = m_sound_updates;
					m_update_read_delay_seen[update_counter_head] = 0;
					update_counter_head = (update_counter_head + 1) & 0x0f;
					update_empty();
				}
				hidx = 0;
			}
		}
		else
		{
			sa = data;
			hidx = 0;
			m_update_address_run++;
			m_update_active = false;
			sti |= SU_CVAL;
		}

		break;
	case IN_PLOAD|IN_CLOAD:
		host[hidx++] = data;
			if (hidx >= 4)
			{
				u32 val = (host[0]<<24) | (host[1]<<16) | (host[2]<<8) | host[3];
				const u8 addr = ca;
				hidx = 0;
				write_cmem_direct(addr, val, "PLOAD_INIT");
				ca++;
		}
		break;
	};
}

u8 tms57002_device::data_r()
{
	u8 res;
	if (!(sti & S_HOST))
		return 0xff;

	res = host[hidx];
	hidx++;
	if (hidx == 4)
	{
		hidx = 0;
		sti &= ~S_HOST;
		update_dready();
	}

	return res;
}

int tms57002_device::dready_r()
{
	return sti & S_HOST ? 0 : 1;
}

void tms57002_device::update_dready()
{
	const int state = (sti & S_HOST) ? 0 : 1;
	if (state != m_dready_line)
	{
		m_dready_line = state;
		m_dready_callback(state);
	}
}

int tms57002_device::pc0_r()
{
	return pc == 0 ? 0 : 1;
}

void tms57002_device::update_pc0()
{
	const int state = (pc == 0) ? 0 : 1;
	if (state != m_pc0_line)
	{
		m_pc0_line = state;
		m_pc0_callback(state);
	}
}

int tms57002_device::empty_r()
{
	return (update_counter_head == update_counter_tail);
}

void tms57002_device::update_empty()
{
	const int state = (update_counter_head == update_counter_tail) ? 1 : 0;
	if (state != m_empty_line)
	{
		m_empty_line = state;
		m_empty_callback(state);
	}
}

bool tms57002_device::serial_cycle_model_enabled() const
{
	return m_serial_frame_mode == serial_frame_mode::lrck_subframe && m_serial_frame_clocks > 0;
}

void tms57002_device::prepare_serial_inputs(const std::array<u32, 4> &frame)
{
	m_serial_input_pending_valid = 0;
	if (m_serial_frame_mode == serial_frame_mode::snapshot)
	{
		for (u8 i = 0; i < 4; i++)
			si[i] = frame[i];
		m_serial_input_prev_frame = frame;
		m_serial_input_prev_valid = 0x0f;
		return;
	}

	// The serial-port TRM states that the SI left/right registers are updated
	// on LRCK transitions and become valid a half-BCK later. With SYPL tied high
	// on the Prophecy board, program sync starts on the LRCK rising edge, so one
	// sub-frame is already stable at sync start while the other becomes valid a
	// little later. Approximate that by exposing one side immediately and
	// publishing the opposite side lazily when the program first reads it.
	const bool left_low = !(st0 & ST0_PLRI);
	const bool pending_right = (left_low && m_sync_polarity_rising) || (!left_low && !m_sync_polarity_rising);
	const u8 pending_mask = pending_right ? 0x0a : 0x05;

	for (u8 i = 0; i < 4; i++)
	{
		if (BIT(pending_mask, i))
		{
			si[i] = BIT(m_serial_input_prev_valid, i) ? m_serial_input_prev_frame[i] : frame[i];
			m_serial_input_pending[i] = frame[i];
			m_serial_input_pending_valid |= u8(1U << i);
		}
		else
		{
			si[i] = frame[i];
			m_serial_input_pending[i] = frame[i];
		}
	}

	m_serial_input_prev_frame = frame;
	m_serial_input_prev_valid = 0x0f;
}

void tms57002_device::schedule_pre_dis_write(bool dmem1_bank, u32 addr, u32 value)
{
	m_pending_pre_transfer = dmem1_bank ? pending_pre_transfer_type::dis_dmem1 : pending_pre_transfer_type::dis_dmem0;
	m_pending_pre_transfer_addr = addr;
	m_pending_pre_transfer_value = value & 0x00ffffffU;
	m_pending_pre_transfer_pc = pc;
}

void tms57002_device::apply_pending_pre_transfer()
{
	if (m_pending_pre_transfer == pending_pre_transfer_type::none)
		return;   // [fast-path] common per-op case: nothing queued -> skip the switch + field resets (no-ops when none)
	switch (m_pending_pre_transfer)
	{
	case pending_pre_transfer_type::dis_dmem0:
			dmem0[m_pending_pre_transfer_addr & 0xff] = m_pending_pre_transfer_value;
		break;

	case pending_pre_transfer_type::dis_dmem1:
			dmem1[m_pending_pre_transfer_addr & 0x1f] = m_pending_pre_transfer_value;
		break;

	case pending_pre_transfer_type::none:
	default:
		break;
	}

	m_pending_pre_transfer = pending_pre_transfer_type::none;
	m_pending_pre_transfer_addr = 0;
	m_pending_pre_transfer_value = 0;
	m_pending_pre_transfer_pc = 0;
}

void tms57002_device::write_dmem(bool dmem1_bank, u32 addr, u32 value, const char *source, u8 pc_value)
{
	const u32 new_value = value & 0x00ffffffU;
	if (dmem1_bank)
	{
		const u8 phys = addr & 0x1f;
		dmem1[phys] = new_value;
	}
	else
	{
		const u8 phys = addr & 0xff;
		dmem0[phys] = new_value;
	}
}


void tms57002_device::write_cmem_direct(u8 addr, u32 value, const char *source, int queue_pos)
{
	cmem[addr] = value;
}

void tms57002_device::write_cmem_from_dsp(u8 addr, u32 value)
{
	write_cmem_direct(addr, value, "DSP_WC");
}


void tms57002_device::finalize_serial_output_build()
{
	if (!serial_cycle_model_enabled())
		return;

	m_serial_output_frame = m_serial_output_build;
	if (m_serial_output_observer)
		m_serial_output_observer("FRAME_FINALIZE", -1, 0, m_serial_exec_halfcycles, m_serial_output_pending_valid);
}

void tms57002_device::start_serial_frame(const std::array<u32, 4> &frame)
{
	if (!serial_cycle_model_enabled())
	{
		prepare_serial_inputs(frame);
		return;
	}

	const auto side_mask = [](bool left) -> u8 { return left ? 0x05 : 0x0a; };
	const bool left_low_in = !(st0 & ST0_PLRI);
	const bool prev_lrck_low = m_sync_polarity_rising;
	bool immediate_input_left = (left_low_in == prev_lrck_low);
	if (m_serial_frame_flip_input)
		immediate_input_left = !immediate_input_left;
	if (m_serial_input_immediate_side_mode == serial_input_immediate_side_mode::left)
		immediate_input_left = true;
	else if (m_serial_input_immediate_side_mode == serial_input_immediate_side_mode::right)
		immediate_input_left = false;
	const u8 immediate_input_mask = side_mask(immediate_input_left);

	m_serial_input_pending_valid = 0;
	for (u8 i = 0; i < 4; i++)
	{
		if (BIT(immediate_input_mask, i))
		{
			si[i] = frame[i];
		}
		else
		{
			si[i] = BIT(m_serial_input_prev_valid, i) ? m_serial_input_prev_frame[i] : frame[i];
			m_serial_input_pending[i] = frame[i];
			m_serial_input_pending_valid |= u8(1U << i);
		}
	}
	m_serial_input_prev_frame = frame;
	m_serial_input_prev_valid = 0x0f;

	const bool left_low_out = !(st0 & ST0_PLRO);
	const bool next_lrck_low = !m_sync_polarity_rising;
	bool immediate_output_left = (left_low_out == next_lrck_low);
	if (m_serial_frame_flip_output)
		immediate_output_left = !immediate_output_left;
	const u8 immediate_output_mask = side_mask(immediate_output_left);
	m_serial_output_pending_valid = immediate_output_mask ^ 0x0f;
	for (u8 i = 0; i < 4; i++)
	{
		if (BIT(immediate_output_mask, i))
			m_serial_output_build[i] = so[i];
	}

	const int bck_clocks = std::max(1, m_serial_frame_clocks / 64);
	m_serial_exec_halfcycles = 3; // SYNC-to-program-start delay is 1.5 clocks.
	m_serial_input_pending_halfcycle = m_serial_frame_clocks + (bck_clocks * 2);
	if (m_serial_input_pending_halfcycle_override >= 0)
		m_serial_input_pending_halfcycle = m_serial_input_pending_halfcycle_override;
	// The pending serial-register transfer is committed 2.5 clocks before the
	// nominal frame boundary. This is the latest-write point demonstrated by
	// the manual's PMEM-FA DOS deadline and the phase-locked silicon oracle.
	m_serial_output_pending_halfcycle = std::max(0, m_serial_frame_clocks - 5);
	if (m_serial_output_pending_halfcycle_override >= 0)
		m_serial_output_pending_halfcycle = m_serial_output_pending_halfcycle_override;
	// Left subframe (SO0/SO2) transmits in the second half of the frame, so its
	// DOS deadline can be configured independently from the right subframe.
	m_serial_output_pending_halfcycle_left = m_serial_output_pending_halfcycle;
	if (m_serial_output_pending_halfcycle_left_override >= 0)
		m_serial_output_pending_halfcycle_left = m_serial_output_pending_halfcycle_left_override;
	if (m_serial_output_observer)
		m_serial_output_observer("FRAME_START", -1, 0, m_serial_exec_halfcycles, immediate_output_mask);
}

void tms57002_device::advance_serial_phase(int halfcycles)
{
	if (!serial_cycle_model_enabled())
		return;

	if (!(m_serial_input_pending_valid | m_serial_output_pending_valid))
	{
		if (halfcycles > m_serial_exec_halfcycles)
			m_serial_exec_halfcycles = halfcycles;
		return;
	}

	if (m_serial_input_pending_valid && halfcycles >= m_serial_input_pending_halfcycle)
	{
		for (u8 i = 0; i < 4; i++)
		{
			if (BIT(m_serial_input_pending_valid, i))
				si[i] = m_serial_input_pending[i];
		}
		m_serial_input_pending_valid = 0;
	}

	if (m_serial_output_pending_valid)
	{
		u8 handoff_mask = 0;
		for (u8 i = 0; i < 4; i++)
		{
			const int lane_deadline = (i & 1)
					? m_serial_output_pending_halfcycle
					: m_serial_output_pending_halfcycle_left;
			if (BIT(m_serial_output_pending_valid, i) && halfcycles >= lane_deadline)
			{
				m_serial_output_build[i] = so[i];
				handoff_mask |= u8(1U << i);
			}
		}
		m_serial_output_pending_valid &= ~handoff_mask;
		if (handoff_mask && m_serial_output_observer)
			m_serial_output_observer("OUTPUT_HANDOFF", -1, 0, halfcycles, handoff_mask);
	}

	if (halfcycles > m_serial_exec_halfcycles)
		m_serial_exec_halfcycles = halfcycles;
}

u32 tms57002_device::serial_input_read(int index)
{
	if (index < 0 || index >= 4)
		return 0;

	if (serial_cycle_model_enabled())
		advance_serial_phase(m_serial_exec_halfcycles + 1);

	if (!serial_cycle_model_enabled() && BIT(m_serial_input_pending_valid, index))
	{
		for (u8 i = 0; i < 4; i++)
		{
			if (BIT(m_serial_input_pending_valid, i))
				si[i] = m_serial_input_pending[i];
		}
		m_serial_input_pending_valid = 0;
	}

	return si[index];
}

void tms57002_device::serial_output_write(int index, u32 value)
{
	if (index >= 0 && index < 4)
	{
		so[index] = value & 0x00ffffffU;
		// Preserve the latest DOS value written before this lane's handoff;
		// writes after the deadline belong to the following serial frame.
		if (serial_cycle_model_enabled() && m_serial_output_write_handoff
				&& (m_serial_exec_halfcycles + 1) <= ((index & 1)
						? m_serial_output_pending_halfcycle
						: m_serial_output_pending_halfcycle_left))
		{
			m_serial_output_build[index] = so[index];
			if (m_serial_output_observer)
				m_serial_output_observer("OUTPUT_WRITE_HANDOFF", index, so[index], m_serial_exec_halfcycles + 1, u8(1U << index));
		}
		if (m_serial_output_observer)
			m_serial_output_observer("SO_WRITE", index, so[index], m_serial_exec_halfcycles + 1, u8(1U << index));
		if (serial_cycle_model_enabled())
			advance_serial_phase(m_serial_exec_halfcycles + 1);
	}
}

u32 tms57002_device::serial_output_unmuted(int index) const
{
	if (index < 0 || index >= 4)
		return 0;
	if (serial_cycle_model_enabled())
		return m_serial_output_frame[index];
	return so[index];
}

u32 tms57002_device::serial_output_pin(int index) const
{
	if (index < 0 || index >= 4)
		return 0;
	if (m_serial_output_muted)
		return 0;
	if (serial_cycle_model_enabled())
		return m_serial_output_frame[index];
	return so[index];
}


void tms57002_device::sync_w(int state)
{
	if (sti & IN_PLOAD)
		return;

	if (serial_cycle_model_enabled())
		m_serial_exec_halfcycles = 3;

	allow_update = 1;
	pc = 0;
	ca = 0;
	id = 0;
	if (!(st0 & ST0_INCS))
	{
		ba0--;
		ba1++;
	}
	xba = (xba-1) & 0x7ffff;
	st1 &= ~(ST1_AOV | ST1_MOV);
	sti &= ~S_IDLE;
}


void tms57002_device::xm_init()
{
	u32 adr = xoa + xba;
	u32 mask = 0;

	switch (st0 & ST0_M)
	{
	case ST0_M_64K:  mask = 0x0ffff; break;
	case ST0_M_256K: mask = 0x3ffff; break;
	case ST0_M_1M:   mask = 0xfffff; break;
	}
	if (st0 & ST0_WORD)
		adr <<= 2;
	else
		adr <<= 1;

	if (!(st0 & ST0_SEL))
		adr <<= 1;

	xm_adr = adr & mask;
	txrd = 0;
	xm_fetches = 0;
	xm_cycles = 0;
}

int tms57002_device::xm_required_fetches() const
{
	if (st0 & ST0_WORD)
		// WORD+SEL: a 24-bit word occupies a 4-byte window but only 3 bytes
		// carry data (merge offsets 16/8/0). A 4th fetch would compute a
		// negative shift and corrupt txrd / write a garbage byte at adr+3.
		return (st0 & ST0_SEL) ? 3 : 6;
	else
		return (st0 & ST0_SEL) ? 2 : 4;
}

int tms57002_device::xm_required_cycles() const
{
	int cycles = 0;
	if (st0 & ST0_SRAM)
	{
		if (st0 & ST0_SEL)
			cycles = (st0 & ST0_WORD) ? 15 : 11;
		else
			cycles = xm_required_fetches();
	}
	else if (st0 & ST0_SEL)
		cycles = (st0 & ST0_WORD) ? 8 : 6;
	else
		cycles = (st0 & ST0_WORD) ? 14 : 10;

	return std::max(cycles, xm_required_fetches());
}

inline void tms57002_device::xm_step_read()
{
	xm_cycles++;

	const int required_fetches = xm_required_fetches();
	const int required_cycles = xm_required_cycles();
	const int first_fetch_cycle = std::max(1, required_cycles - required_fetches + 1);
	if (xm_cycles < first_fetch_cycle)
		return;

	u32 adr = xm_adr;
	u8 v = data->read_byte(adr);
	int done;
	if (st0 & ST0_WORD)
	{
		if (st0 & ST0_SEL)
		{
			int off = 16 - ((adr & 3) << 3);
			txrd = (txrd & ~(0xff << off)) | (v << off);
			done = ++xm_fetches >= required_fetches;
		}
		else
		{
			int off = 20 - ((adr & 7) << 2);
			txrd = (txrd & ~(0xf << off)) | ((v & 0xf) << off);
			done = ++xm_fetches >= required_fetches;
		}
	}
	else
	{
		if (st0 & ST0_SEL)
		{
			int off = 16 - ((adr & 1) << 3);
			txrd = (txrd & ~(0xff << off)) | (v << off);
			done = ++xm_fetches >= required_fetches;
			if (done && xm_cycles >= required_cycles)
				txrd &= 0xffff00;
		}
		else
		{
			int off = 20 - ((adr & 3) << 2);
			txrd = (txrd & ~(0xf << off)) | ((v & 0xf) << off);
			done = ++xm_fetches >= required_fetches;
			if (done && xm_cycles >= required_cycles)
				txrd &= 0xffff00;
		}
	}
	if (done && xm_cycles >= required_cycles)
	{
		xrd = txrd;
		sti &= ~S_READ;
		xm_adr = 0;
		xm_cycles = 0;
		xm_fetches = 0;
	}
	else
		xm_adr = adr+1;
}

inline void tms57002_device::xm_step_write()
{
	xm_cycles++;

	const int required_fetches = xm_required_fetches();
	const int required_cycles = xm_required_cycles();
	const int first_fetch_cycle = std::max(1, required_cycles - required_fetches + 1);
	if (xm_cycles < first_fetch_cycle)
		return;

	u32 adr = xm_adr;
	u8 v;
	int done;
	if (st0 & ST0_WORD)
	{
		if (st0 & ST0_SEL)
		{
			int off = 16 - ((adr & 3) << 3);
			v = xwr >> off;
			done = ++xm_fetches >= required_fetches;
		}
		else
		{
			int off = 20 - ((adr & 7) << 2);
			v = (xwr >> off) & 0xf;
			done = ++xm_fetches >= required_fetches;
		}
	}
	else
	{
		if (st0 & ST0_SEL)
		{
			int off = 16 - ((adr & 1) << 3);
			v = xwr >> off;
			done = ++xm_fetches >= required_fetches;
		}
		else
		{
			int off = 20 - ((adr & 3) << 2);
			v = (xwr >> off) & 0xf;
			done = ++xm_fetches >= required_fetches;
		}
	}
	data->write_byte(adr, v);
	if (done && xm_cycles >= required_cycles)
	{
		sti &= ~S_WRITE;
		xm_adr = 0;
		xm_cycles = 0;
		xm_fetches = 0;
	}
	else
		xm_adr = adr+1;
}

s64 tms57002_device::macc_to_output_0(s64 rounding, u64 rmask)
{
	s64 m = macc_read;
	u64 m1;
	int over = false;

	// Overflow detection and shifting
	m1 = m & 0xf800000000000ULL;
	if (m1 && m1 != 0xf800000000000ULL)
		over = true;

	m = (m + rounding) & rmask;

	// Second overflow detection
	m1 = m & 0xf800000000000ULL;
	if (m1 && m1 != 0xf800000000000ULL)
		over = true;

	// Overflow handling
	if (over)
	{
		st1 |= ST1_MOV;
	}
	return m;
}

s64 tms57002_device::macc_to_output_1(s64 rounding, u64 rmask)
{
	s64 m = macc_read;
	u64 m1;
	int over = false;

	// Overflow detection and shifting
	m1 = m & 0xfe00000000000ULL;
	if (m1 && m1 != 0xfe00000000000ULL)
		over = true;
	m <<= 2;

	m = (m + rounding) & rmask;

	// Second overflow detection
	m1 = m & 0xf800000000000ULL;
	if (m1 && m1 != 0xf800000000000ULL)
		over = true;

	// Overflow handling
	if (over)
	{
		st1 |= ST1_MOV;
	}
	return m;
}

s64 tms57002_device::macc_to_output_2(s64 rounding, u64 rmask)
{
	s64 m = macc_read;
	u64 m1;
	int over = false;

	// Overflow detection and shifting
	m1 = m & 0xff80000000000ULL;
	if (m1 && m1 != 0xff80000000000ULL)
		over = true;
	m <<= 4;

	m = (m + rounding) & rmask;

	// Second overflow detection
	m1 = m & 0xf800000000000ULL;
	if (m1 && m1 != 0xf800000000000ULL)
		over = true;

	// Overflow handling
	if (over)
	{
		st1 |= ST1_MOV;
	}
	return m;
}

s64 tms57002_device::macc_to_output_3(s64 rounding, u64 rmask)
{
	s64 m = macc_read;
	u64 m1;
	int over = false;

	// Overflow detection and shifting
	m >>= 8;

	m = (m + rounding) & rmask;

	// Second overflow detection
	m1 = m & 0xf800000000000ULL;
	if (m1 && m1 != 0xf800000000000ULL)
		over = true;

	// Overflow handling
	if (over)
	{
		st1 |= ST1_MOV;
	}
	return m;
}

namespace {

// Silicon clips the positive MOVM rail to the 48-bit limit masked by the
// active RND width.  The negative limit already has every discarded bit clear.
// Proven by 436/436 standalone DOMH oracle points.
constexpr s64 k_round_clipped_positive[5] = {
	0x00007fffffffffffLL,
	0x00007fffffff0000LL,
	0x00007fffff000000LL,
	0x00007ffff0000000LL,
	0x00007fff00000000LL
};

constexpr s64 k_round_clipped_negative[5] = {
	s64(0xffff800000000000ULL),
	s64(0xffff800000000000ULL),
	s64(0xffff800000000000ULL),
	s64(0xffff800000000000ULL),
	s64(0xffff800000000000ULL)
};

inline s64 rounded_clipped_macc_output(bool negative, int rnd_mode)
{
	rnd_mode = std::clamp(rnd_mode, 0, 4);
	return negative ? k_round_clipped_negative[rnd_mode] : k_round_clipped_positive[rnd_mode];
}

} // anonymous namespace

s64 tms57002_device::macc_to_output_0s(s64 rounding, u64 rmask, int rnd_mode)
{
	s64 m = macc_read;
	u64 m1;
	int over = false;

	// Overflow detection and shifting
	m1 = m & 0xf800000000000ULL;
	if (m1 && m1 != 0xf800000000000ULL)
		over = true;

	m = (m + rounding) & rmask;

	// Second overflow detection
	m1 = m & 0xf800000000000ULL;
	if (m1 && m1 != 0xf800000000000ULL)
		over = true;

	// Overflow handling
	if (over)
	{
		st1 |= ST1_MOV;
		m = rounded_clipped_macc_output(bool(macc_read & 0x8000000000000ULL), rnd_mode);
	}
	return m;
}

s64 tms57002_device::macc_to_output_1s(s64 rounding, u64 rmask, int rnd_mode)
{
	s64 m = macc_read;
	u64 m1;
	int over = false;

	// Overflow detection and shifting
	m1 = m & 0xfe00000000000ULL;
	if (m1 && m1 != 0xfe00000000000ULL)
		over = true;
	m <<= 2;

	m = (m + rounding) & rmask;

	// Second overflow detection
	m1 = m & 0xf800000000000ULL;
	if (m1 && m1 != 0xf800000000000ULL)
		over = true;

	// Overflow handling
	if (over)
	{
		st1 |= ST1_MOV;
		m = rounded_clipped_macc_output(bool(macc_read & 0x8000000000000ULL), rnd_mode);
	}
	return m;
}

s64 tms57002_device::macc_to_output_2s(s64 rounding, u64 rmask, int rnd_mode)
{
	s64 m = macc_read;
	u64 m1;
	int over = false;

	// Overflow detection and shifting
	m1 = m & 0xff80000000000ULL;
	if (m1 && m1 != 0xff80000000000ULL)
		over = true;
	m <<= 4;

	m = (m + rounding) & rmask;

	// Second overflow detection
	m1 = m & 0xf800000000000ULL;
	if (m1 && m1 != 0xf800000000000ULL)
		over = true;

	// Overflow handling
	if (over)
	{
		st1 |= ST1_MOV;
		m = rounded_clipped_macc_output(bool(macc_read & 0x8000000000000ULL), rnd_mode);
	}
	return m;
}

s64 tms57002_device::macc_to_output_3s(s64 rounding, u64 rmask, int rnd_mode)
{
	s64 m = macc_read;
	u64 m1;
	int over = false;

	// Overflow detection and shifting
	m >>= 8;

	m = (m + rounding) & rmask;

	// Second overflow detection
	m1 = m & 0xf800000000000ULL;
	if (m1 && m1 != 0xf800000000000ULL)
		over = true;

	// Overflow handling
	if (over)
	{
		st1 |= ST1_MOV;
		m = rounded_clipped_macc_output(bool(macc_read & 0x8000000000000ULL), rnd_mode);
	}
	return m;
}

s64 tms57002_device::macc_to_output_0n(s64 rounding, u64 rmask)
{
	s64 m = macc_read;
	u64 m1;
	int over = false;

	m1 = m & 0xf800000000000ULL;
	if (m1 && m1 != 0xf800000000000ULL)
		over = true;

	m = (m + rounding) & rmask;

	m1 = m & 0xf800000000000ULL;
	if (m1 && m1 != 0xf800000000000ULL)
		over = true;

	if (over)
	{
		st1 |= ST1_MOV;
		if (macc_read & 0x8000000000000ULL)
			m = s64(0xffff800000000000ULL);
		else
			m = 0x00007fffffffffffLL;
	}
	return m;
}

s64 tms57002_device::macc_to_output_1n(s64 rounding, u64 rmask)
{
	s64 m = macc_read;
	u64 m1;
	int over = false;

	m1 = m & 0xfe00000000000ULL;
	if (m1 && m1 != 0xfe00000000000ULL)
		over = true;
	m <<= 2;

	m = (m + rounding) & rmask;

	m1 = m & 0xf800000000000ULL;
	if (m1 && m1 != 0xf800000000000ULL)
		over = true;

	if (over)
	{
		st1 |= ST1_MOV;
		if (macc_read & 0x8000000000000ULL)
			m = s64(0xffff800000000000ULL);
		else
			m = 0x00007fffffffffffLL;
	}
	return m;
}

s64 tms57002_device::macc_to_output_2n(s64 rounding, u64 rmask)
{
	s64 m = macc_read;
	u64 m1;
	int over = false;

	m1 = m & 0xff80000000000ULL;
	if (m1 && m1 != 0xff80000000000ULL)
		over = true;
	m <<= 4;

	m = (m + rounding) & rmask;

	m1 = m & 0xf800000000000ULL;
	if (m1 && m1 != 0xf800000000000ULL)
		over = true;

	if (over)
	{
		st1 |= ST1_MOV;
		if (macc_read & 0x8000000000000ULL)
			m = s64(0xffff800000000000ULL);
		else
			m = 0x00007fffffffffffLL;
	}
	return m;
}

s64 tms57002_device::macc_to_output_3n(s64 rounding, u64 rmask)
{
	s64 m = macc_read;
	u64 m1;
	int over = false;

	m >>= 8;

	m = (m + rounding) & rmask;

	m1 = m & 0xf800000000000ULL;
	if (m1 && m1 != 0xf800000000000ULL)
		over = true;

	if (over)
	{
		st1 |= ST1_MOV;
		if (macc_read & 0x8000000000000ULL)
			m = s64(0xffff800000000000ULL);
		else
			m = 0x00007fffffffffffLL;
	}
	return m;
}

s64 tms57002_device::check_macc_overflow_0()
{
	s64 m = macc_read;
	u64 m1;

	// Overflow detection
	m1 = m & 0xf800000000000ULL;
	if (m1 && m1 != 0xf800000000000ULL)
	{
		st1 |= ST1_MOV;
	}
	return m;
}

s64 tms57002_device::check_macc_overflow_1()
{
	s64 m = macc_read;
	u64 m1;

	// Overflow detection
	m1 = m & 0xfe00000000000ULL;
	if (m1 && m1 != 0xfe00000000000ULL)
	{
		st1 |= ST1_MOV;
	}
	return m;
}

s64 tms57002_device::check_macc_overflow_2()
{
	s64 m = macc_read;
	u64 m1;

	// Overflow detection
	m1 = m & 0xff80000000000ULL;
	if (m1 && m1 != 0xff80000000000ULL)
	{
		st1 |= ST1_MOV;
	}
	return m;
}

s64 tms57002_device::check_macc_overflow_3()
{
	return macc_read;
}

s64 tms57002_device::check_macc_overflow_0s()
{
	s64 m = macc_read;
	u64 m1;

	// Overflow detection
	m1 = m & 0xf800000000000ULL;
	if (m1 && m1 != 0xf800000000000ULL)
	{
		st1 |= ST1_MOV;
		if (macc_read & 0x8000000000000ULL)
			m = s64(0xffff800000000000ULL);
		else
			m = 0x00007fffffffffffLL;
	}
	return m;
}

s64 tms57002_device::check_macc_overflow_1s()
{
	s64 m = macc_read;
	u64 m1;

	// Overflow detection
	m1 = m & 0xfe00000000000ULL;
	if (m1 && m1 != 0xfe00000000000ULL)
	{
		st1 |= ST1_MOV;
		if (macc_read & 0x8000000000000ULL)
			m = s64(0xffff800000000000ULL);
		else
			m = 0x00007fffffffffffLL;
	}
	return m;
}

s64 tms57002_device::check_macc_overflow_2s()
{
	s64 m = macc_read;
	u64 m1;

	// Overflow detection
	m1 = m & 0xff80000000000ULL;
	if (m1 && m1 != 0xff80000000000ULL)
	{
		st1 |= ST1_MOV;
		if (macc_read & 0x8000000000000ULL)
			m = s64(0xffff800000000000ULL);
		else
			m = 0x00007fffffffffffLL;
	}
	return m;
}

s64 tms57002_device::check_macc_overflow_3s()
{
	return macc_read;
}

s64 tms57002_device::ca_mpy_product_to_macc(s64 product, int base_shift) const
{
	return product >> base_shift;
}

s64 tms57002_device::dc_mpy_product_to_macc(s64 product) const
{
	return product >> 7;
}

// SMHC ("store MACC high to CMEM"): MAME writes the full macc>>16 (32-bit).
// Bench evidence (Korg Prophecy coefficient-ramp equilibrium, 2026-07-04) shows
// real silicon stores only the high 24 bits, i.e. the low m_debug_smhc_trunc_bits
// bits of the written coefficient are forced to zero. Default 0 = legacy behavior.
s64 tms57002_device::smhc_store_shift(s64 macc_out) const
{
	return (macc_out >> 16) & ~((s64(1) << m_debug_smhc_trunc_bits) - 1);
}

u32 tms57002_device::lmhd_d_operand(u32 current, u8 param) const
{
	if (!m_debug_lmhd_srbd_forward)
		return current;

	const u32 opcode = program->read_dword(pc);
	const bool same_direct_lmhd_srbd =
		(opcode >> 18) == 0x31 &&
		((opcode >> 11) & 0x7f) == 0x0f &&
		(opcode & 0xff) == param;
	if (!same_direct_lmhd_srbd)
		return current;

	return (xrd & 0x00ffffffU) << 8;
}

u32 tms57002_device::mpy_d_operand(u32 current, u8 param)
{
	if (!m_debug_mpy_smhd_forward)
		return current;
	if (m_debug_mpy_smhd_forward_pc >= 0 && pc != u8(m_debug_mpy_smhd_forward_pc))
		return current;

	const u32 opcode = program->read_dword(pc);
	const bool same_direct_mpy_smhd =
		(opcode >> 18) == 0x21 &&
		((opcode >> 11) & 0x7f) == 0x03 &&
		(opcode & 0xff) == param;
	if (!same_direct_mpy_smhd)
		return current;

	const debug_macc_eval_result eval = debug_eval_macc_output(
		u64(macc_read),
		(st1 & ST1_SFMO) >> ST1_SFMO_SHIFT,
		(st1 & ST1_RND) >> ST1_RND_SHIFT,
		(st1 & ST1_MOVM) ? debug_macc_clip_mode::rounded : debug_macc_clip_mode::none);
	return u32(eval.value >> 24) & 0x00ffffffU;
}

tms57002_device::debug_macc_eval_result tms57002_device::debug_eval_macc_output(u64 macc_value, int sfmo_mode, int rnd_mode, debug_macc_clip_mode clip_mode)
{
	static constexpr s64 rounding_lut[5] = {
		0x0000000000000000ULL,
		0x0000000000008000ULL,
		0x0000000000800000ULL,
		0x0000000008000000ULL,
		0x0000000080000000ULL
	};
	static constexpr u64 rmask_lut[5] = {
		0xffffffffffffffffULL,
		0xffffffffffff0000ULL,
		0xffffffffff000000ULL,
		0xfffffffff0000000ULL,
		0xffffffff00000000ULL
	};

	constexpr u64 mask52 = (u64(1) << 52) - 1;
	u64 raw = macc_value & mask52;
	s64 signed_macc = BIT(raw, 51) ? s64(raw | ~mask52) : s64(raw);
	sfmo_mode &= 3;
	rnd_mode = std::clamp(rnd_mode, 0, 4);

	const u32 saved_st1 = st1;
	const s64 saved_macc = macc;
	const s64 saved_macc_read = macc_read;
	const s64 saved_macc_write = macc_write;

	st1 &= ~(ST1_SFMO | ST1_RND | ST1_MOVM | ST1_MOV);
	st1 |= u32(sfmo_mode << ST1_SFMO_SHIFT);
	st1 |= u32(rnd_mode << ST1_RND_SHIFT);
	if (clip_mode != debug_macc_clip_mode::none)
		st1 |= ST1_MOVM;

	macc = signed_macc;
	macc_read = signed_macc;
	macc_write = signed_macc;

	s64 result = 0;
	switch (sfmo_mode)
	{
	case 0:
		if (clip_mode == debug_macc_clip_mode::rounded)
			result = macc_to_output_0s(rounding_lut[rnd_mode], rmask_lut[rnd_mode], rnd_mode);
		else if (clip_mode == debug_macc_clip_mode::unrounded)
			result = macc_to_output_0n(rounding_lut[rnd_mode], rmask_lut[rnd_mode]);
		else
			result = macc_to_output_0(rounding_lut[rnd_mode], rmask_lut[rnd_mode]);
		break;
	case 1:
		if (clip_mode == debug_macc_clip_mode::rounded)
			result = macc_to_output_1s(rounding_lut[rnd_mode], rmask_lut[rnd_mode], rnd_mode);
		else if (clip_mode == debug_macc_clip_mode::unrounded)
			result = macc_to_output_1n(rounding_lut[rnd_mode], rmask_lut[rnd_mode]);
		else
			result = macc_to_output_1(rounding_lut[rnd_mode], rmask_lut[rnd_mode]);
		break;
	case 2:
		if (clip_mode == debug_macc_clip_mode::rounded)
			result = macc_to_output_2s(rounding_lut[rnd_mode], rmask_lut[rnd_mode], rnd_mode);
		else if (clip_mode == debug_macc_clip_mode::unrounded)
			result = macc_to_output_2n(rounding_lut[rnd_mode], rmask_lut[rnd_mode]);
		else
			result = macc_to_output_2(rounding_lut[rnd_mode], rmask_lut[rnd_mode]);
		break;
	default:
		if (clip_mode == debug_macc_clip_mode::rounded)
			result = macc_to_output_3s(rounding_lut[rnd_mode], rmask_lut[rnd_mode], rnd_mode);
		else if (clip_mode == debug_macc_clip_mode::unrounded)
			result = macc_to_output_3n(rounding_lut[rnd_mode], rmask_lut[rnd_mode]);
		else
			result = macc_to_output_3(rounding_lut[rnd_mode], rmask_lut[rnd_mode]);
		break;
	}

	const debug_macc_eval_result eval{ result, bool(st1 & ST1_MOV) };

	st1 = saved_st1;
	macc = saved_macc;
	macc_read = saved_macc_read;
	macc_write = saved_macc_write;

	return eval;
}

void tms57002_device::debug_load_program(const u32 *words, u32 count, u32 st0_value, u32 st1_value)
{
	device_reset();

	for (u32 addr = 0; addr < 0x100; addr++)
		program->write_dword(addr, 0);

	for (u32 addr = 0; addr < count && addr < 0x100; addr++)
		program->write_dword(addr, words[addr] & 0x00ffffffU);

	cache_flush();
	pc = 0;
	ca = 0;
	id = 0;
	ba0 = 0;
	ba1 = 0;
	rptc = 0;
	rptc_next = 0;
	st0 = st0_value;
	st1 = st1_value;
	sti = S_IDLE;
	update_pc0();
	update_dready();
	update_empty();
}

void tms57002_device::debug_reset_live_state(bool zero_accumulators)
{
	device_reset();
	if (zero_accumulators)
		debug_set_accumulators(0, 0, 0, 0, 0);
}

void tms57002_device::debug_run_cycles(int max_cycles)
{
	icount = std::max(max_cycles, 1);
	execute_run();
}

std::array<u32, 4> tms57002_device::debug_run_sample_frame(const std::array<u32, 4> &frame, int max_cycles)
{
	m_sound_updates++;

	if (serial_cycle_model_enabled())
	{
		finalize_serial_output_build();
		m_serial_input_frame = frame;
		start_serial_frame(frame);
	}
	else
	{
		prepare_serial_inputs(frame);
	}

	m_serial_input_active = m_serial_input_latch;
	m_serial_input_active_valid = m_serial_input_valid;
	m_serial_input_valid = 0;

	sync_w(1);
	icount = std::max(max_cycles, 1);
	execute_run();

	return { serial_output_pin(0), serial_output_pin(1), serial_output_pin(2), serial_output_pin(3) };
}

u64 tms57002_device::jit_program_hash()
{
	u64 h = 1469598103934665603ull;   // FNV-1a 64-bit offset basis
	for (u32 a = 0; a < 0x100; a++) {
		const u32 w = program->read_dword(a) & 0x00ffffffU;
		h = (h ^ (w & 0xff)) * 1099511628211ull;
		h = (h ^ ((w >> 8) & 0xff)) * 1099511628211ull;
		h = (h ^ ((w >> 16) & 0xff)) * 1099511628211ull;
	}
	return h;
}

void tms57002_device::jit_begin_frame(const std::array<u32, 4> &frame)
{
	// The per-sample setup of debug_run_sample_frame() (everything before its execute
	// loop), factored out so the JIT driver shares an identical frame boundary with the
	// interpreter — only the execution in between differs. Kept in lockstep with
	// debug_run_sample_frame; tms57002_jit_test asserts they stay equivalent.
	m_sound_updates++;

	if (serial_cycle_model_enabled())
	{
		finalize_serial_output_build();
		m_serial_input_frame = frame;
		start_serial_frame(frame);
	}
	else
	{
		prepare_serial_inputs(frame);
	}

	m_serial_input_active = m_serial_input_latch;
	m_serial_input_active_valid = m_serial_input_valid;
	m_serial_input_valid = 0;

	sync_w(1);
}

// Generated op-index -> mnemonic table (same numbering as the interpreter switch).
namespace {
#define CINTRPNAME
#include "cpu/tms57002/tms57002.hxx"
#undef CINTRPNAME
}  // namespace

std::vector<std::string> tms57002_device::jit_used_mnemonics() const
{
	std::set<std::string> seen;
	const int n = cache.iused;
	const int table_size = int(sizeof(tms57002_op_name) / sizeof(tms57002_op_name[0]));
	for (int k = 0; k < n; k++) {
		const int op = cache.inst[k].op;
		if (op >= 4 && op < table_size) {
			const char *nm = tms57002_op_name[op];
			if (nm && nm[0]) seen.insert(nm);
		}
	}
	return std::vector<std::string>(seen.begin(), seen.end());
}

const char *tms57002_device::jit_op_mnemonic(unsigned op) const
{
	const unsigned table_size = unsigned(sizeof(tms57002_op_name) / sizeof(tms57002_op_name[0]));
	return (op < table_size) ? tms57002_op_name[op] : nullptr;
}

// Variant index of an op within its mnemonic group: ops are generated as consecutive runs of
// _variants sharing one mnemonic (e.g. "lacc" -> cmode=0 then cmode=1; "smhd" -> dmode/dbp/sfmo/...),
// so (op - first-op-with-this-mnemonic) is the variant index. Variant 0 is the simplest form
// (all addressing/mode flags 0) — the one the JIT op-inliners assume; the JIT deopts other variants.
unsigned tms57002_device::jit_op_variant(unsigned op) const
{
	const unsigned table_size = unsigned(sizeof(tms57002_op_name) / sizeof(tms57002_op_name[0]));
	if (op >= table_size || !tms57002_op_name[op] || !tms57002_op_name[op][0])
		return 0;
	const char *nm = tms57002_op_name[op];
	unsigned base = op;
	while (base > 4 && tms57002_op_name[base - 1] && tms57002_op_name[base - 1][0]
		&& strcmp(tms57002_op_name[base - 1], nm) == 0)
		base--;
	return op - base;
}

// --- JIT execution seam --------------------------------------------------------
// jit_run_chain() executes exactly ONE program instruction (one PC's full micro-op
// chain + the surrounding machinery) given the chain-head ipc, returning the next
// ipc. It is a TRACE-FREE replica of execute_run()'s per-PC body — the unit the JIT
// drives in compile-time-unrolled order (so the runtime while-loop and per-PC decode
// are eliminated). Kept bit-exact vs execute_run() by tms57002_jit_test; the trace
// code in execute_run() only reads state, so omitting it cannot change execution.
// Decomposed into the three pieces the JIT emits per PC: pre (xm_step + MACC
// pipeline), per-micro-op exec (the ex_N dispatch), and post (transfer/serial/pc/
// branch). The JIT unrolls the chain at compile time and emits these, removing the
// runtime chain-walk + dispatch. jit_run_chain composes them; the bit-exact gate is
// tms57002_jit_test (interp == per-PC seam).
void tms57002_device::jit_pc_pre()
{
	if (sti & (S_READ | S_WRITE))
	{
		if (sti & S_READ)
			xm_step_read();
		else
			xm_step_write();
	}
	macc_read = macc_write;
	macc_write = macc;
}

void tms57002_device::jit_op_exec(unsigned op, const void *icd_ptr)
{
	const icd *i = static_cast<const icd *>(icd_ptr);
	switch (op)
	{
#define CINTRPSWITCH
#include "cpu/tms57002/tms57002.hxx"
#undef CINTRPSWITCH
	default:
		fatalerror("Unhandled opcode in tms57002 jit_op_exec\n");
	}
}

int tms57002_device::jit_pc_post(int iipc, int ipc)
{
	apply_pending_pre_transfer();

	if (serial_cycle_model_enabled())
	{
		const int serial_next_halfcycles = m_serial_exec_halfcycles + 2;
		if (m_serial_input_pending_valid || m_serial_output_pending_valid)
			advance_serial_phase(serial_next_halfcycles);
		else
			m_serial_exec_halfcycles = serial_next_halfcycles;
	}

	icount--;

	if (rptc)
	{
		rptc--;
		ipc = iipc;
	}
	else if (sti & S_BRANCH)
	{
		sti &= ~S_BRANCH;
		ipc = -1;
	}
	else
	{
		pc++;
		update_pc0();
		// A sequential wrap completes the 256-word sample program.  Silicon
		// waits at that boundary for the next SYNC rather than starting a
		// second pass from the remaining scheduler cycle budget.
		if (!pc)
			sti |= S_IDLE;
	}

	if (rptc_next)
	{
		rptc = rptc_next;
		rptc_next = 0;
	}
	return ipc;
}

int tms57002_device::jit_run_chain(int ipc)
{
	const int iipc = ipc;
	jit_pc_pre();
	for (;;)
	{
		const icd *i = cache.inst + ipc;
		ipc = i->next;
		const unsigned op = i->op;
		if (op < 4)   // chain terminator (cases 0..3: nop / ++ca / ++id / ++ca,++id)
		{
			if (op == 1) ++ca;
			else if (op == 2) ++id;
			else if (op == 3) { ++ca; ++id; }
			break;
		}
		jit_op_exec(op, i);
		if (sti & S_IDLE)
			break;
	}
	return jit_pc_post(iipc, ipc);
}

// Run one sample frame driven by the jit_run_chain seam (instead of execute_run).
// Mirrors debug_run_sample_frame; bit-exact with it once jit_run_chain is correct.
std::array<u32, 4> tms57002_device::jit_run_sample_frame(const std::array<u32, 4> &frame, int max_cycles)
{
	jit_begin_frame(frame);
	icount = std::max(max_cycles, 1);
	int ipc = -1;
	while (icount > 0 && !(sti & (S_IDLE | IN_PLOAD)))
	{
		if (ipc == -1)
			ipc = decode_get_pc();
		ipc = jit_run_chain(ipc);
	}
	if (serial_cycle_model_enabled() && (sti & S_IDLE))
		finalize_serial_output_build();
	if (icount > 0)
		icount = 0;
	return jit_end_frame();
}

tms57002_device::debug_snapshot tms57002_device::debug_capture_snapshot() const
{
	debug_snapshot snapshot;
	snapshot.sound_updates = m_sound_updates;
	snapshot.pc = pc;
	snapshot.hpc = hpc;
	snapshot.ca = ca;
	snapshot.id = id;
	snapshot.ba0 = ba0;
	snapshot.ba1 = ba1;
	snapshot.rptc = rptc;
	snapshot.rptc_next = rptc_next;
	snapshot.sa = sa;
	snapshot.hidx = hidx;
	snapshot.allow_update = allow_update;
	snapshot.st0 = st0;
	snapshot.st1 = st1;
	snapshot.sti = sti;
	snapshot.aacc = aacc;
	snapshot.macc = u64(macc);
	snapshot.macc_read = u64(macc_read);
	snapshot.macc_write = u64(macc_write);
	snapshot.creg = creg;
	snapshot.xoa = xoa;
	snapshot.xba = xba;
	snapshot.xwr = xwr;
	snapshot.xrd = xrd;
	snapshot.txrd = txrd;
	snapshot.xm_adr = xm_adr;
	snapshot.xm_cycles = xm_cycles;
	snapshot.xm_fetches = xm_fetches;
	std::copy(std::begin(si), std::end(si), snapshot.si.begin());
	std::copy(std::begin(so), std::end(so), snapshot.so.begin());
	snapshot.serial_input_latch = m_serial_input_latch;
	snapshot.serial_input_active = m_serial_input_active;
	snapshot.serial_input_frame = m_serial_input_frame;
	snapshot.serial_input_prev_frame = m_serial_input_prev_frame;
	snapshot.serial_input_pending = m_serial_input_pending;
	snapshot.serial_input_valid = m_serial_input_valid;
	snapshot.serial_input_active_valid = m_serial_input_active_valid;
	snapshot.serial_input_prev_valid = m_serial_input_prev_valid;
	snapshot.serial_input_pending_valid = m_serial_input_pending_valid;
	snapshot.serial_output_pending_valid = m_serial_output_pending_valid;
	snapshot.pending_pre_transfer = u8(m_pending_pre_transfer);
	snapshot.pending_pre_transfer_addr = m_pending_pre_transfer_addr;
	snapshot.pending_pre_transfer_value = m_pending_pre_transfer_value;
	snapshot.serial_input_timing_mode = u8(m_serial_input_timing_mode);
	snapshot.serial_frame_mode = u8(m_serial_frame_mode);
	snapshot.serial_frame_clocks = m_serial_frame_clocks;
	snapshot.serial_exec_halfcycles = m_serial_exec_halfcycles;
	snapshot.serial_input_pending_halfcycle = m_serial_input_pending_halfcycle;
	snapshot.serial_output_pending_halfcycle = m_serial_output_pending_halfcycle;
	snapshot.sync_polarity_rising = u8(m_sync_polarity_rising);
	snapshot.serial_output_muted = u8(m_serial_output_muted);
	snapshot.serial_frame_flip_input = u8(m_serial_frame_flip_input);
	snapshot.serial_frame_flip_output = u8(m_serial_frame_flip_output);
	snapshot.update_counter_head = update_counter_head;
	snapshot.update_counter_tail = update_counter_tail;
	snapshot.update_active = u8(m_update_active);
	snapshot.update_address_run = m_update_address_run;
	std::copy(std::begin(cmem), std::end(cmem), snapshot.cmem.begin());
	std::copy(std::begin(dmem0), std::end(dmem0), snapshot.dmem0.begin());
	std::copy(std::begin(dmem1), std::end(dmem1), snapshot.dmem1.begin());
	std::copy(std::begin(update), std::end(update), snapshot.update.begin());
	std::copy(std::begin(update_sa), std::end(update_sa), snapshot.update_sa.begin());
	std::copy(std::begin(m_update_address_run_for_entry), std::end(m_update_address_run_for_entry), snapshot.update_address_run_for_entry.begin());
	std::copy(std::begin(m_update_enqueue_su), std::end(m_update_enqueue_su), snapshot.update_enqueue_su.begin());
	std::copy(std::begin(m_update_read_delay_seen), std::end(m_update_read_delay_seen), snapshot.update_read_delay_seen.begin());
	return snapshot;
}

void tms57002_device::debug_restore_snapshot(const debug_snapshot &snapshot)
{
	m_sound_updates = snapshot.sound_updates;
	pc = snapshot.pc;
	hpc = snapshot.hpc;
	ca = snapshot.ca;
	id = snapshot.id;
	ba0 = snapshot.ba0;
	ba1 = snapshot.ba1;
	rptc = snapshot.rptc;
	rptc_next = snapshot.rptc_next;
	sa = snapshot.sa;
	hidx = snapshot.hidx;
	allow_update = snapshot.allow_update;
	st0 = snapshot.st0;
	st1 = snapshot.st1;
	sti = snapshot.sti;
	aacc = snapshot.aacc;
	macc = s64(snapshot.macc);
	macc_read = s64(snapshot.macc_read);
	macc_write = s64(snapshot.macc_write);
	creg = snapshot.creg;
	xoa = snapshot.xoa;
	xba = snapshot.xba;
	xwr = snapshot.xwr;
	xrd = snapshot.xrd;
	txrd = snapshot.txrd;
	xm_adr = snapshot.xm_adr;
	xm_cycles = snapshot.xm_cycles;
	xm_fetches = snapshot.xm_fetches;
	std::copy(snapshot.si.begin(), snapshot.si.end(), std::begin(si));
	std::copy(snapshot.so.begin(), snapshot.so.end(), std::begin(so));
	m_serial_input_latch = snapshot.serial_input_latch;
	m_serial_input_active = snapshot.serial_input_active;
	m_serial_input_frame = snapshot.serial_input_frame;
	m_serial_input_prev_frame = snapshot.serial_input_prev_frame;
	m_serial_input_pending = snapshot.serial_input_pending;
	m_serial_input_valid = snapshot.serial_input_valid;
	m_serial_input_active_valid = snapshot.serial_input_active_valid;
	m_serial_input_prev_valid = snapshot.serial_input_prev_valid;
	m_serial_input_pending_valid = snapshot.serial_input_pending_valid;
	m_serial_output_pending_valid = snapshot.serial_output_pending_valid;
	m_pending_pre_transfer = pending_pre_transfer_type(snapshot.pending_pre_transfer);
	m_pending_pre_transfer_addr = snapshot.pending_pre_transfer_addr;
	m_pending_pre_transfer_value = snapshot.pending_pre_transfer_value;
	m_serial_input_timing_mode = serial_input_timing_mode(snapshot.serial_input_timing_mode);
	m_serial_frame_mode = serial_frame_mode(snapshot.serial_frame_mode);
	m_serial_frame_clocks = snapshot.serial_frame_clocks;
	m_serial_exec_halfcycles = snapshot.serial_exec_halfcycles;
	m_serial_input_pending_halfcycle = snapshot.serial_input_pending_halfcycle;
	m_serial_output_pending_halfcycle = snapshot.serial_output_pending_halfcycle;
	m_sync_polarity_rising = bool(snapshot.sync_polarity_rising);
	m_serial_output_muted = bool(snapshot.serial_output_muted);
	m_serial_frame_flip_input = bool(snapshot.serial_frame_flip_input);
	m_serial_frame_flip_output = bool(snapshot.serial_frame_flip_output);
	update_counter_head = snapshot.update_counter_head;
	update_counter_tail = snapshot.update_counter_tail;
	m_update_active = bool(snapshot.update_active);
	m_update_address_run = snapshot.update_address_run;
	std::copy(snapshot.cmem.begin(), snapshot.cmem.end(), std::begin(cmem));
	std::copy(snapshot.dmem0.begin(), snapshot.dmem0.end(), std::begin(dmem0));
	std::copy(snapshot.dmem1.begin(), snapshot.dmem1.end(), std::begin(dmem1));
	std::copy(snapshot.update.begin(), snapshot.update.end(), std::begin(update));
	std::copy(snapshot.update_sa.begin(), snapshot.update_sa.end(), std::begin(update_sa));
	std::copy(snapshot.update_address_run_for_entry.begin(), snapshot.update_address_run_for_entry.end(), std::begin(m_update_address_run_for_entry));
	std::copy(snapshot.update_enqueue_su.begin(), snapshot.update_enqueue_su.end(), std::begin(m_update_enqueue_su));
	std::copy(snapshot.update_read_delay_seen.begin(), snapshot.update_read_delay_seen.end(), std::begin(m_update_read_delay_seen));
	update_pc0();
	update_dready();
	update_empty();
}

void tms57002_device::maybe_state_zap()
{
}

void tms57002_device::debug_begin_sample_frame(const std::array<u32, 4> &frame)
{
	maybe_state_zap();
	m_sound_updates++;
	if (serial_cycle_model_enabled())
	{
		finalize_serial_output_build();
		m_serial_input_frame = frame;
		start_serial_frame(frame);
	}
	else
	{
		prepare_serial_inputs(frame);
	}

	m_serial_input_active = m_serial_input_latch;
	m_serial_input_active_valid = m_serial_input_valid;
	m_serial_input_valid = 0;
	sync_w(1);
}

u32 tms57002_device::get_cmem(u8 addr)
{
	const bool update_pending = update_counter_head != update_counter_tail;
	if (update_pending && (sti & IN_CLOAD))
	{
		sti &= ~S_UPDATE;
		return current_cmem_view(addr);
	}
	const bool update_matches = update_pending && (m_update_active || update_sa[update_counter_tail] == addr);
	if (update_matches)
	{
		if (update_timing_selected(addr))
		{
			switch (m_update_timing_mode)
			{
			case update_timing_mode::sample_delay_1:
				if (m_sound_updates <= m_update_enqueue_su[update_counter_tail])
				{
					sti &= ~S_UPDATE;
					return current_cmem_view(addr);
				}
				break;

			case update_timing_mode::read_delay_1:
				if (!m_update_read_delay_seen[update_counter_tail])
				{
					m_update_read_delay_seen[update_counter_tail] = 1;
					sti &= ~S_UPDATE;
					return current_cmem_view(addr);
				}
				break;

			case update_timing_mode::normal:
			case update_timing_mode::eager:
			default:
				break;
			}
		}

		sti |= S_UPDATE;
		m_update_active = true;
		const u8 consumed_update_run = m_update_address_run_for_entry[update_counter_tail];
		write_cmem_direct(addr, update[update_counter_tail], "HOST_APPLY");
		update_counter_tail = (update_counter_tail + 1) & 0x0f;
		update_empty();

		if (update_counter_head == update_counter_tail)
		{
			m_update_active = false;
			sti &= ~S_UPDATE;
		}
		else
		{
			m_update_active = (m_update_address_run_for_entry[update_counter_tail] == consumed_update_run);
		}

		return cmem[addr]; // The value of crm is ignored during an update.
	}
	else
	{
		sti &= ~S_UPDATE;
		return current_cmem_view(addr);
	}
}

void tms57002_device::cache_flush()
{
	int i;
	cache.hused = cache.iused = 0;
	for (i = 0; i != 256; i++)
		cache.hashbase[i] = -1;
	for (i = 0; i != HBS; i++)
	{
		cache.hashnode[i].st1 = 0;
		cache.hashnode[i].ipc = -1;
		cache.hashnode[i].next = -1;
	}
	for (i = 0; i != IBS; i++)
	{
		cache.inst[i].op = 0;
		cache.inst[i].next = -1;
		cache.inst[i].param = 0;
	}
}

void tms57002_device::add_one(cstate *cs, u16 op, u8 param)
{
	s16 ipc = cache.iused++;
	cache.inst[ipc].op = op;
	cache.inst[ipc].param = param;
	cache.inst[ipc].next = -1;
	if (cs->ipc != -1)
		cache.inst[cs->ipc].next = ipc;
	cs->ipc = ipc;
	if (cs->hnode != -1)
	{
		cache.hashnode[cs->hnode].ipc = ipc;
		cs->hnode = -1;
	}
}

void tms57002_device::decode_one(u32 opcode, cstate *cs, void (tms57002_device::*dec)(u32 opcode, u16 *op, cstate *cs))
{
	u16 op = 0;
	(this->*dec)(opcode, &op, cs);
	if (!op)
		return;
	add_one(cs, op, opcode & 0xff);
}

s16 tms57002_device::get_hash(u8 adr, u32 st1, s16 *pnode)
{
	s16 hnode;
	st1 &= ST1_CACHE;
	*pnode = -1;
	hnode = cache.hashbase[adr];
	while(hnode != -1)
	{
		if (cache.hashnode[hnode].st1 == st1)
			return cache.hashnode[hnode].ipc;
		*pnode = hnode;
		hnode = cache.hashnode[hnode].next;
	}
	return -1;
}

s16 tms57002_device::get_hashnode(u8 adr, u32 st1, s16 pnode)
{
	s16 hnode = cache.hused++;
	cache.hashnode[hnode].st1 = st1 & ST1_CACHE;
	cache.hashnode[hnode].ipc = -1;
	cache.hashnode[hnode].next = -1;
	if (pnode == -1)
		cache.hashbase[adr] = hnode;
	else
		cache.hashnode[pnode].next = hnode;
	return hnode;
}

int tms57002_device::decode_get_pc()
{
	s16 pnode, res;
	cstate cs;
	u8 adr = pc;

	res = get_hash(adr, st1, &pnode);
	if (res != -1)
		return res;

	if (HBS - cache.hused < 256 || IBS - cache.iused < 256*3)
	{
		cache_flush();
		pnode = -1;
	}

	cs.hnode = res = get_hashnode(adr, st1, pnode);
	cs.ipc = -1;
	cs.branch = 0;

	// m_ca_inc_delayed: a c*+ CA increment commits only after the FOLLOWING
	// instruction's operand reads, so its INC_CA bit is emitted on the next
	// instruction's boundary op instead of its own. At chain seams (branches,
	// joins, chaining disabled) the pending bit is merged into the current
	// boundary op: commit is one slot early across seams only. ID increments
	// are never carried.
	u16 carried_ca_inc = 0;
	for (;;)
	{
		s16 ipc;
		u32 opcode = program->read_dword(adr);

		cs.inc = 0;

		if ((opcode & 0xfc0000) == 0xfc0000)
			decode_one(opcode, &cs, &tms57002_device::decode_cat3);
		else {
			// CMEM-store cat2 ops (smhc 0x05, sacc 0x01): historically these
			// commit BEFORE the cat1 operand read (a same-instruction
			// smhc c(x);lacc c(x) pair reads the just-stored value). The
			// A12 voice program's decay loop requires the store to commit in
			// the post slot (cat1 reads the OLD value) or a stable loop turns
			// into a gain-5 amplifier and the idle voice drones forever
			// (drone investigation 2026-07-02). m_smhc_post_slot selects the
			// post-slot (read-old) behavior for both ops.
			const u8 c2op = u8((opcode >> 11) & 0x7f);
			const bool c2_store = (c2op == 0x05) || (c2op >= 0x01 && c2op <= 0x04)
				|| (c2op == 0x06) || (c2op == 0x07);
			const bool smhc_slot_override = c2_store && !m_smhc_post_slot;
			if (smhc_slot_override)
				decode_one(opcode, &cs, &tms57002_device::decode_cat2_post);
			else
				decode_one(opcode, &cs, &tms57002_device::decode_cat2_pre);
			decode_one(opcode, &cs, &tms57002_device::decode_cat1);
			if (!smhc_slot_override)
				decode_one(opcode, &cs, &tms57002_device::decode_cat2_post);
		}
		if (m_ca_inc_delayed)
		{
			// Dual-memory (d,c) instructions read their coefficient operand a
			// pipeline slot early, before the preceding instruction's CA
			// post-increment has landed. Model: carry a pending INC_CA past
			// exactly those instructions (commit on their boundary op instead).
			// Single-memory and direct-addressed reads are unaffected.
			u16 boundary = u16((cs.inc & ~u16(INC_CA)) | carried_ca_inc);
			carried_ca_inc = u16(cs.inc & INC_CA);
			bool carry = false;
			if (!m_disable_decode_chaining && !cs.branch && carried_ca_inc)
			{
				const u32 next_opcode = program->read_dword(u8(adr + 1));
				if ((next_opcode & 0xfc0000) != 0xfc0000)
				{
					switch (u8(next_opcode >> 18))
					{
					case 0x07: case 0x0d: case 0x16: case 0x19: case 0x1c:
					case 0x21: case 0x24: case 0x28: case 0x29:
					case 0x2c: case 0x2d: case 0x38:
						carry = true;
						break;
					default:
						break;
					}
				}
			}
			if (!carry)
			{
				boundary |= carried_ca_inc;
				carried_ca_inc = 0;
			}
			add_one(&cs, boundary, 0);
		}
		else
			add_one(&cs, cs.inc, 0);

		if (m_disable_decode_chaining)
			break;

		if (cs.branch)
			break;

		adr++;
		ipc = get_hash(adr, st1, &pnode);
		if (ipc != -1)
		{
			if (carried_ca_inc)
				cache.inst[cs.ipc].op |= carried_ca_inc;
			cache.inst[cs.ipc].next = ipc;
			break;
		}
		cs.hnode = get_hashnode(adr, st1, pnode);
	}

	return cache.hashnode[res].ipc;
}

void tms57002_device::execute_run()
{
	int ipc = -1;
	const bool serial_cycle_model = serial_cycle_model_enabled();

	while(icount > 0 && !(sti & (S_IDLE | IN_PLOAD /*| IN_CLOAD*/)))
	{
		int iipc;
		const u8 curpc = pc;

		// Pooled DSP dynarec: at pc0 with a clean state, run the whole program pass as one compiled
		// native frame (asmjit; bit-exact rendition of this loop), or RESUME a partial frame the
		// previous timeslice left mid-way. run_begun_frame_pooled compiles on first sight (returns
		// false -> the interpreter runs THIS frame, byte-identical), then drives natively from the
		// next frame; it applies the identical idle/icount contract as this function's tail
		// (jit_finalize_if_idle == `if(icount>0) icount=0`).
		// Selection: the machine-config default (set_dynarec_default — ON for the Korg Prophecy,
		// gated by the 2026-07-07 F6 corpus sweep, byte-identical across 36 programs; OFF for
		// generic TMS57002 users), overridden by KPROP_DSP_PERFRAME: 4 = force on, any other
		// value = force interpreter (the kill switch). Non-x86-64 hosts fall back to the
		// interpreter automatically (compile returns nullptr -> programs run interpreted).
		{
			static const int pf_env = [] {
				const char *e = std::getenv("KPROP_DSP_PERFRAME");
				return e ? std::atoi(e) : -1;   // -1 = unset -> use the machine-config default
			}();
			const bool pf4 = (pf_env >= 0) ? (pf_env == 4) : m_dynarec_default;
			if (pf4 && !serial_cycle_model)
			{
				if (!m_jit)
					m_jit = std::make_unique<tms57002::Jit>();
				// Fire at a FRESH frame boundary (pc0, clean state) OR to RESUME a partial frame the previous
				// timeslice left mid-way (curpc != 0). run_begun_frame_pooled runs whole or partial (up to
				// the timeslice); a partial run is resumed natively next timeslice via the fn's entry dispatch.
				const bool resume = m_jit->pooled_resume_pending();
				if (resume || (curpc == 0 && !rptc && !rptc_next && !(sti & S_BRANCH)))
				{
					if (m_jit->run_begun_frame_pooled(*this))
						continue;
				}
			}
		}

		if (ipc == -1)
			ipc = decode_get_pc();

		iipc = ipc;

		if (sti & (S_READ|S_WRITE))
		{
			if (sti & S_READ)
				xm_step_read();
			else
				xm_step_write();
		}

		macc_read = macc_write;
		macc_write = macc;

		for (;;)
		{
			const icd *i = cache.inst + ipc;



			ipc = i->next;
			switch (i->op)
			{
			case 0:
				goto inst;

			case 1:
				++ca;
				goto inst;

			case 2:
				++id;
				goto inst;

			case 3:
				++ca, ++id;
				goto inst;

#define CINTRPSWITCH
#include "cpu/tms57002/tms57002.hxx"
#undef CINTRPSWITCH

			default:
				fatalerror("Unhandled opcode in tms57002_execute\n");
			}


			// `idle` ends the current sample program immediately; do not keep
			// walking the cached tail after the flag is raised.
			if (sti & S_IDLE)
				goto inst;
		}
inst:
		if (m_pending_pre_transfer != pending_pre_transfer_type::none)
			apply_pending_pre_transfer();   // [fast-path] skip the per-op call when nothing is queued

		if (serial_cycle_model)
		{
			const int serial_next_halfcycles = m_serial_exec_halfcycles + 2;
			if (m_serial_input_pending_valid || m_serial_output_pending_valid)
				advance_serial_phase(serial_next_halfcycles);
			else
				m_serial_exec_halfcycles = serial_next_halfcycles;
		}


		icount--;

		if (rptc)
		{
			rptc--;
			ipc = iipc;
		}
		else if (sti & S_BRANCH)
		{
			sti &= ~S_BRANCH;
			ipc = -1;
		}
		else
		{
			pc++; // Wraps if it reaches 256, next wraps too
			update_pc0();
			// Sequential 0xff -> 0x00 is the end-of-sample boundary.  Branches
			// to zero take the S_BRANCH arm above and remain unaffected.
			if (!pc)
				sti |= S_IDLE;
		}

		if (rptc_next)
		{
			rptc = rptc_next;
			rptc_next = 0;
		}
	}

	if (serial_cycle_model && (sti & S_IDLE))
		finalize_serial_output_build();

	if (icount > 0)
		icount = 0;
}


void tms57002_device::sound_stream_update(sound_stream &stream)
{
	assert(stream.samples() == 1);
	maybe_state_zap();
	m_sound_updates++;

	if (serial_cycle_model_enabled())
		finalize_serial_output_build();

	sound_stream::sample_t in_scale = 32768.0 * ((st0 & ST0_SIM) ? 1.0 : 256.0);
	const double sine_sample = m_input_sine
		? std::sin((2.0 * 3.14159265358979323846 * m_input_sine_freq * double(m_sound_updates)) / double(stream.sample_rate())) * m_input_sine_amp
		: 0.0;
	std::array<u32, 4> frame{};
	for (u8 i = 0; i < 4; i++)
	{
		const u32 stream_value = m_input_sine
			? (s32((i < 2 ? sine_sample : 0.0) * in_scale) & 0x00ffffffU)
			: (s32(stream.get(i, 0) * in_scale) & 0x00ffffffU);
		switch (m_serial_input_timing_mode)
		{
		case serial_input_timing_mode::sample_delay_1:
			frame[i] = BIT(m_serial_input_active_valid, i) ? m_serial_input_active[i] : stream_value;
			break;

		case serial_input_timing_mode::same_sample:
		default:
			frame[i] = BIT(m_serial_input_valid, i) ? m_serial_input_latch[i] : stream_value;
			break;
		}
	}
	if (serial_cycle_model_enabled())
	{
		m_serial_input_frame = frame;
		start_serial_frame(frame);
	}
	else
	{
		prepare_serial_inputs(frame);
	}
	m_serial_input_active = m_serial_input_latch;
	m_serial_input_active_valid = m_serial_input_valid;
	m_serial_input_valid = 0;

	const auto stream_output = [this](int index) { return m_stream_output_raw ? so[index] : serial_output_pin(index); };
	stream.put(0, 0, s32(stream_output(0) << 8) /  2147483648.0);
	stream.put(1, 0, s32(stream_output(1) << 8) /  2147483648.0);
	stream.put(2, 0, s32(stream_output(2) << 8) /  2147483648.0);
	stream.put(3, 0, s32(stream_output(3) << 8) /  2147483648.0);

	if (m_stream_sync_enabled)
		sync_w(1);
}

void tms57002_device::device_start()
{
	// Test/fallback controls for the pooled CMEM-read guard.  Both default off.
	m_pf4_force_cmem_unsafe = (std::getenv("KPROP_PF4_FORCE_CMEM_UNSAFE") != nullptr);
	m_pf4_cmem_deopt = (std::getenv("KPROP_PF4_CMEM_DEOPT") != nullptr);
	m_debug_cmem_force = false;
	m_serial_input_timing_mode = serial_input_timing_mode::same_sample;
	m_serial_frame_mode = serial_frame_mode::snapshot;
	m_serial_input_immediate_side_mode = serial_input_immediate_side_mode::auto_select;
	m_sync_polarity_rising = false;
	m_serial_output_muted = false;
	m_serial_input_pending_halfcycle_override = -1;
	m_serial_output_pending_halfcycle_override = -1;
	m_serial_output_pending_halfcycle_left_override = -1;
	m_serial_output_write_handoff = false;
	m_stream_sync_enabled = true;
	if (const char *hc_env = std::getenv("KPROP_DSP_SERIAL_OUTPUT_PENDING_HALFCYCLE"))
	{
		const int value = std::atoi(hc_env);
		if (value >= 0)
			m_serial_output_pending_halfcycle_override = value;
	}
	if (const char *hc_env = std::getenv("KPROP_DSP_SERIAL_OUTPUT_PENDING_HALFCYCLE_LEFT"))
	{
		const int value = std::atoi(hc_env);
		if (value >= 0)
			m_serial_output_pending_halfcycle_left_override = value;
	}
	if (const char *handoff_env = std::getenv("KPROP_DSP_SERIAL_OUTPUT_WRITE_HANDOFF"))
		m_serial_output_write_handoff = std::strtol(handoff_env, nullptr, 0) != 0;
	// KPSHIP-ENGINE: 440 Hz test-tone injector wired into a shared core. Dev harness; delete
	// (or move to the standalone test harness). Not accuracy.
	if (const char *env = std::getenv("KPROP_DSP_INPUT_SINE_TAG"))
	{
		if (env[0] && std::strstr(tag(), env) != nullptr)
		{
			m_input_sine = true;
			m_input_sine_amp = 0.10;
			m_input_sine_freq = 440.0;
			if (const char *amp_env = std::getenv("KPROP_DSP_INPUT_SINE_AMP"))
				m_input_sine_amp = std::strtod(amp_env, nullptr);
			if (const char *freq_env = std::getenv("KPROP_DSP_INPUT_SINE_FREQ"))
				m_input_sine_freq = std::strtod(freq_env, nullptr);
			if (m_input_sine_amp < 0.0)
				m_input_sine_amp = 0.0;
			if (m_input_sine_amp > 1.0)
				m_input_sine_amp = 1.0;
			if (m_input_sine_freq < 0.0)
				m_input_sine_freq = 0.0;
		}
	}
	sti = S_IDLE;
	program = &space(AS_PROGRAM);
	data    = &space(AS_DATA);

	state_add(STATE_GENPC,    "GENPC",  pc).noshow();
	state_add(STATE_GENPCBASE,"CURPC",  pc).noshow();
	state_add(TMS57002_PC,    "PC",     pc);
	state_add(TMS57002_ST0,   "ST0",    st0);
	state_add(TMS57002_ST1,   "ST1",    st1);
	state_add(TMS57002_RPTC,  "RPTC",   rptc);
	state_add(TMS57002_AACC,  "AACC",   aacc);
	state_add(TMS57002_MACC,  "MACC",   macc).mask(0xfffffffffffffU);
	state_add(TMS57002_BA0,   "BA0",    ba0);
	state_add(TMS57002_BA1,   "BA1",    ba1);
	state_add(TMS57002_CREG,  "CREG",   creg);
	state_add(TMS57002_CA,    "CA",     ca);
	state_add(TMS57002_ID,    "ID",     id);
	state_add(TMS57002_XBA,   "XBA",    xba);
	state_add(TMS57002_XOA,   "XOA",    xoa);
	state_add(TMS57002_XRD,   "XRD",    xrd);
	state_add(TMS57002_XWR,   "XWR",    xwr);
	state_add(TMS57002_HIDX,  "HIDX",   hidx);
	state_add(TMS57002_HOST0, "HOST0",  host[0]);
	state_add(TMS57002_HOST1, "HOST1",  host[1]);
	state_add(TMS57002_HOST2, "HOST2",  host[2]);
	state_add(TMS57002_HOST3, "HOST3",  host[3]);

	set_icountptr(icount);
	// This DSP is a sample-synchronous processor: it can legitimately follow
	// either its upstream source or its downstream sink. Using full adaptive
	// mode lets fixed-rate DAC sinks solve the rate of an otherwise all-adaptive
	// processing chain.
	stream_alloc(4, 4, SAMPLE_RATE_ADAPTIVE, STREAM_SYNCHRONOUS);

	save_item(NAME(macc));
	save_item(NAME(macc_read));
	save_item(NAME(macc_write));

	save_item(NAME(cmem));
	save_item(NAME(dmem0));
	save_item(NAME(dmem1));
	save_item(NAME(update));
	save_item(NAME(update_sa));
	save_item(NAME(m_update_active));
	save_item(NAME(m_update_address_run));
	save_item(NAME(m_update_address_run_for_entry));

	save_item(NAME(si));
	save_item(NAME(so));
	save_item(NAME(m_serial_input_latch));
	save_item(NAME(m_serial_input_active));
	save_item(NAME(m_serial_input_frame));
	save_item(NAME(m_serial_input_prev_frame));
	save_item(NAME(m_serial_input_pending));
	save_item(NAME(m_serial_output_frame));
	save_item(NAME(m_serial_output_build));
	save_item(NAME(m_serial_input_valid));
	save_item(NAME(m_serial_input_active_valid));
	save_item(NAME(m_serial_input_prev_valid));
	save_item(NAME(m_serial_input_pending_valid));
	save_item(NAME(m_serial_output_pending_valid));
	save_item(NAME(m_stream_sync_enabled));
	save_item(NAME(m_pending_pre_transfer_addr));
	save_item(NAME(m_pending_pre_transfer_value));
	save_item(NAME(m_serial_frame_clocks));
	save_item(NAME(m_serial_exec_halfcycles));
	save_item(NAME(m_serial_input_pending_halfcycle));
	save_item(NAME(m_serial_output_pending_halfcycle));
	save_item(NAME(m_serial_output_pending_halfcycle_left));
	save_item(NAME(m_sound_updates));
	save_item(NAME(m_dready_line));
	save_item(NAME(m_pc0_line));
	save_item(NAME(m_empty_line));
	save_item(NAME(m_update_enqueue_su));
	save_item(NAME(m_update_read_delay_seen));

	save_item(NAME(st0));
	save_item(NAME(st1));
	save_item(NAME(sti));
	save_item(NAME(aacc));
	save_item(NAME(xoa));
	save_item(NAME(xba));
	save_item(NAME(xwr));
	save_item(NAME(xrd));
	save_item(NAME(txrd));
	save_item(NAME(creg));

	save_item(NAME(pc));
	save_item(NAME(ca));
	save_item(NAME(id));
	save_item(NAME(ba0));
	save_item(NAME(ba1));
	save_item(NAME(rptc));
	save_item(NAME(rptc_next));
	save_item(NAME(sa));

	save_item(NAME(xm_adr));
	save_item(NAME(xm_cycles));
	save_item(NAME(xm_fetches));

	save_item(NAME(host));
	save_item(NAME(hidx));

	save_item(NAME(update_counter_head));
	save_item(NAME(update_counter_tail));
	save_item(NAME(allow_update));
}

bool tms57002_device::update_timing_selected(u8 addr) const
{
	return m_update_timing_addr_any && m_update_timing_addr[addr];
}

u32 tms57002_device::current_cmem_view(u8 addr) const
{
	int crm = (st1 & ST1_CRM) >> ST1_CRM_SHIFT;
	u32 cvar = cmem[addr];
	if (crm == 1)
		return (cvar & 0xffff0000);
	else if (crm == 2)
		return (cvar << 16);
	return cvar;
}

u32 tms57002_device::execute_min_cycles() const noexcept
{
	return 1;
}

u32 tms57002_device::execute_max_cycles() const noexcept
{
	return 3;
}

device_memory_interface::space_config_vector tms57002_device::memory_space_config() const
{
	return space_config_vector {
		std::make_pair(AS_PROGRAM, &program_config),
		std::make_pair(AS_DATA, &data_config)
	};
}
