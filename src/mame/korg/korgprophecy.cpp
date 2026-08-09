// license:BSD-3-Clause
// copyright-holders:AJR
/***************************************************************************

    Korg Prophecy (1995) — monophonic modeling solo synthesizer.

    Main CPU: Hitachi H8/3003. Sub-CPU: NEC V55 (uPD70433), modeled on top of
    the V5x core. Voice engine: three Texas Instruments TMS57002 DSPs. Front
    panel driven over the H8/V55 bus with an HD44780 character LCD.

***************************************************************************/

// ---------------------------------------------------------------------------
// KPSHIP tags — future-revisit markers. NONE block release (generated audio
// matches the hardware); they mark where the model still leans on a knob or a
// bring-up compromise, for a later accuracy/cleanup pass. Grep `KPSHIP-`.
//   KPSHIP-BAKE     accuracy A/B env knob whose default = the hw-validated value.
//                   Inline the default + delete the getenv (byte-identical;
//                   gate: 00-overnight-gate/census_wide.sh).
//   KPSHIP-ENGINE   execution-engine / dev-signal knob (JIT select, test tone,
//                   idle-skip). Pick a policy, then delete. Stays byte-identical.
//   KPSHIP-ACCURACY hardcoded, NON-flag compromise ("bring-up / hypothesis / not
//                   a proven hw fact"). The real accuracy backlog. Gate any fix
//                   vs the HARDWARE corpus (korgprophecy_serial_scoreboard.py),
//                   NOT the self-golden (which already bakes these in). Do not
//                   delete blind — load-bearing for "it plays at all".
//   KPSHIP-TRACE    logging/trace knob. Strip freely (a getenv in a shared core
//                   or the OSD is also an upstream blocker).
//   KPSHIP-FORK     deliberate fork feature (HLE oracle, INJECT harness, GUI).
//                   Keep for the fork; strip only for an upstream target.
//   KPSHIP-KEEP     looks like a hack but is intentional & correct — leave it.
// The public control inventory records the disposition of every KPROP_* token.
// ---------------------------------------------------------------------------

#include "emu.h"
#include "cpu/h8/h83003.h"
#include "cpu/nec/v5x.h"
#include "cpu/tms57002/tms57002.h"
#include "bus/midi/midi.h"
#include "machine/nvram.h"
#include "video/hd44780.h"
#include "emupal.h"
#include "screen.h"
#include "speaker.h"
#include <algorithm>
#include <array>
#include <cerrno>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#if defined(__unix__) || defined(__APPLE__)
#include <unistd.h>
#endif

namespace
{
enum class dsp_board_dsp2_input_mode : u8
{
	normal = 0,
	so1_to_si0,
	so1_r_to_si0_r,
	so1_l_to_si0_r,
	so1_l_to_si0_l,
	so1_r_to_si0_l,
	// E3 topology probe retained as a deterministic routing-test seam:
	// feed DSP2 from the dsp3 device (the voice chip under KPROP_DSP_HOST_MAP=3,2,1).
	dsp3_so1_to_si0,
	dsp3_so0_to_si0,
	// as dsp3_so1_to_si0, but SI1 carries the dsp1 device's SO0 (the
	// housekeeping/master-gain chip under the swapped host map).
	dsp3_so1_dsp1_so0
};

dsp_board_dsp2_input_mode parse_dsp2_input_route_env(const char *env)
{
	if (!env || !env[0] || !std::strcmp(env, "0") || !std::strcmp(env, "normal") || !std::strcmp(env, "so0_to_si0"))
		return dsp_board_dsp2_input_mode::normal;
	if (!std::strcmp(env, "so1_to_si0") || !std::strcmp(env, "dsp1_so1") || !std::strcmp(env, "pair1") || !std::strcmp(env, "swap_pairs"))
		return dsp_board_dsp2_input_mode::so1_to_si0;
	if (!std::strcmp(env, "so1_r_to_si0_r") || !std::strcmp(env, "right_from_so1_r"))
		return dsp_board_dsp2_input_mode::so1_r_to_si0_r;
	if (!std::strcmp(env, "so1_l_to_si0_r") || !std::strcmp(env, "right_from_so1_l"))
		return dsp_board_dsp2_input_mode::so1_l_to_si0_r;
	if (!std::strcmp(env, "so1_l_to_si0_l") || !std::strcmp(env, "left_from_so1_l"))
		return dsp_board_dsp2_input_mode::so1_l_to_si0_l;
	if (!std::strcmp(env, "so1_r_to_si0_l") || !std::strcmp(env, "left_from_so1_r"))
		return dsp_board_dsp2_input_mode::so1_r_to_si0_l;
	if (!std::strcmp(env, "dsp3_so1_to_si0") || !std::strcmp(env, "voice_tail"))
		return dsp_board_dsp2_input_mode::dsp3_so1_to_si0;
	if (!std::strcmp(env, "dsp3_so0_to_si0") || !std::strcmp(env, "voice_tail_so0"))
		return dsp_board_dsp2_input_mode::dsp3_so0_to_si0;
	if (!std::strcmp(env, "dsp3_so1_dsp1_so0") || !std::strcmp(env, "voice_tail_aux"))
		return dsp_board_dsp2_input_mode::dsp3_so1_dsp1_so0;
	return dsp_board_dsp2_input_mode::normal;
}

enum class dsp_board_feedback_mode : u8
{
	off = 0,
	delay1
};

}

class korgprophecy_u2_device : public device_t, public device_sound_interface
{
public:
	korgprophecy_u2_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

protected:
	virtual void device_start() override ATTR_COLD
	{
		// KPSHIP-ACCURACY: U2 is a pass-through DAC placeholder; exact IC24/IC25
		// WS/LRCK edge timing is modeled as a fixed 48 kHz stereo copy, not the real part.
		// The traced board's DAC side runs at a fixed 48 kHz FS rate. In the
		// TMS57002 core, SO0 is already exposed as a left/right word pair
		// (`so0_l` / `so0_r`), so this placeholder U2 device runs at that fixed
		// DAC rate and presents the pair as stereo output while leaving the exact
		// IC24/IC25 WS/LRCK edge timing as future work.
		m_stream = stream_alloc(2, 2, 48'000, STREAM_SYNCHRONOUS);
	}

	virtual void sound_stream_update(sound_stream &stream) override
	{
		for (int sample = 0; sample < stream.samples(); sample++)
		{
			const sound_stream::sample_t left = stream.get(0, sample);
			const sound_stream::sample_t right = stream.get(1, sample);
			stream.put(0, sample, left);
			stream.put(1, sample, right);
		}
	}

private:
	sound_stream *m_stream;
};

DEFINE_DEVICE_TYPE(KORGPROPHECY_U2, korgprophecy_u2_device, "korgprophecy_u2", "Korg Prophecy U2 DAC placeholder")

korgprophecy_u2_device::korgprophecy_u2_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, KORGPROPHECY_U2, tag, owner, clock)
	, device_sound_interface(mconfig, *this)
	, m_stream(nullptr)
{
}


class korgprophecy_state : public driver_device
{
public:
	korgprophecy_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_subcpu(*this, "subcpu")
		, m_dsp1(*this, "dsp1")
		, m_dsp2(*this, "dsp2")
		, m_dsp3(*this, "dsp3")
		, m_u2(*this, "u2")
		, m_lcdc(*this, "lcdc")
		, m_sysram(*this, "sysram")
		, m_panel(*this, "PANEL%u", 0U)
		, m_kbd(*this, "KBD%u", 0U)
		, m_cfg(*this, "CFG")
		, m_leds(*this, "led%u", 0U)
	{
	}

	void prophecy(machine_config &config);

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;

private:
	static constexpr u32 MMIO_INPUT_STATUS = 0x0fff00;
	static constexpr u32 MMIO_INPUT_DATA = 0x0fff02;
	static constexpr u32 MMIO_LCD_CTRL   = 0x0fff04;
	static constexpr u32 MMIO_LCD_DATA   = 0x0fff06;
	static constexpr u32 MMIO_SCAN_SEL   = 0x0fff16;
	static constexpr u32 MB_PHYS_A702    = 0x00622;
	static constexpr u32 MB_PHYS_A70A    = 0x0062a;
	static constexpr u32 MB_PHYS_A70C    = 0x0062c;
	static constexpr u32 MB_PHYS_A70E    = 0x0062e;
	static constexpr u32 MB_PHYS_A710    = 0x00630;
	static constexpr u32 MB_PHYS_A712    = 0x00632;
	static constexpr u32 MB_PHYS_A714    = 0x00634;
	static constexpr u32 MB_PHYS_A717    = 0x00637;
	static constexpr u32 MB_PHYS_A719    = 0x00639;
	static constexpr u32 MB_PHYS_A71A    = 0x0063a;
	static constexpr u32 MB_PHYS_A71B    = 0x0063b;
	static constexpr u32 MBQ_PHYS_DATA_BASE = 0x008c6;
	static constexpr u32 MBQ_PHYS_CTRL_BASE = 0x00cc6;
	static constexpr u32 EVTQ_PHYS_WPTR  = 0x00882;
	static constexpr u32 EVTQ_PHYS_RPTR  = 0x00884;
	static constexpr u32 EVTQ_PHYS_CNT   = 0x00886;
	static constexpr u32 TXQ_PHYS_WPTR   = 0x00888;
	static constexpr u32 TXQ_PHYS_RPTR   = 0x0088a;
	static constexpr u32 TXQ_PHYS_CNT    = 0x0088c;
	static constexpr u32 EVTQ_PHYS_SEG_PTR = 0x000c4;
	static constexpr u32 INPUTQ_PHYS_SEG = 0x00090;
	static constexpr u32 INPUTQ_PHYS_CNT = 0x00742;
	static constexpr u32 INPUTQ_PHYS_WPTR = 0x00744;
	static constexpr u32 INPUTQ_PHYS_RPTR = 0x00746;
	static constexpr u32 SCANQ_PHYS_CNT = 0x00816;
	static constexpr u32 SCANQ_PHYS_WPTR = 0x00818;
	static constexpr u32 SCANQ_PHYS_RPTR = 0x0081a;
	static constexpr u32 SCANQ_PHYS_ENABLE = 0x00822;
	static constexpr u32 SCANQ_PHYS_BUF = 0x0082a;
	static constexpr u32 TASK_PHYS_A892 = 0x00892;
	static constexpr u32 MB_PHYS_A716    = 0x00636;
	static constexpr u32 MB_PHYS_A721    = 0x00641;
	static constexpr u32 V55_PENDING_PARAM_BASE = 0x00916;
	static constexpr u32 V55_PENDING_PARAM_WPTR = 0x00936;
	static constexpr u32 V55_PENDING_PARAM_RPTR = 0x00938;
	static constexpr u32 MB_PHYS_A89A    = 0x0079a;
	static constexpr u32 MB_PHYS_A8A8    = 0x007c8;
	static constexpr u32 MB_PHYS_A8B6    = 0x007d6;
	static constexpr u32 MB_PHYS_A8C4    = 0x007e4;
	static constexpr u32 MB_PHYS_A8D2    = 0x007f2;
	static constexpr u32 MB_PHYS_A8C3    = 0x007e3;
	static constexpr u32 MB_PHYS_A8D1    = 0x007f1;
	static constexpr u32 MB_PHYS_A8E0    = 0x00800;
	static constexpr u32 MB_PHYS_A8E1    = 0x00801;
	static constexpr u32 MB_PHYS_A8E2    = 0x00802;
	static constexpr u32 MB_PHYS_A8E4    = 0x00804;
	static constexpr u32 MB_PHYS_A8F0    = 0x00810;
	static constexpr u32 MB_PHYS_A8F1    = 0x00811;
	static constexpr u32 MB_PHYS_A8F2    = 0x00812;
	static constexpr u32 MB_PHYS_A8F3    = 0x00813;
	static constexpr u32 MB_PHYS_A8F4    = 0x00814;
	static constexpr u32 MB_PHYS_A8FC    = 0x0081c;
	static constexpr u32 MB_PHYS_A94A    = 0x0086a;
	static constexpr u32 MB_PHYS_A956    = 0x00876;
	static constexpr u32 MB_PHYS_A958    = 0x00878;
	static constexpr u32 MB_PHYS_A96E    = 0x0088e;
	static constexpr u32 MB_PHYS_A970    = 0x00890;
	static constexpr u32 MB_PHYS_A980    = 0x008a0;
	static constexpr u32 MB_PHYS_AB84    = 0x000aa4;
	static constexpr u32 MB_PHYS_AB86    = 0x000aa6;
	static constexpr u32 MB_PHYS_AB8A    = 0x000aaa;
	static constexpr u32 MB_PHYS_AB8C    = 0x000aac;
	static constexpr u32 MB_PHYS_AB8E    = 0x000aae;
	static constexpr u32 MB_PHYS_AB90    = 0x000ab0;
	static constexpr u32 MB_PHYS_AB99    = 0x000ab9;
	static constexpr u32 MB_PHYS_AB9A    = 0x000aba;
	static constexpr u32 MB_PHYS_ABC4    = 0x000ae4;
	static constexpr u32 MB_PHYS_AA7B    = 0x00099b;
	static constexpr u32 MB_PHYS_D814    = 0x0d814;
	static constexpr u32 MB_PHYS_D892    = 0x0d892;
	static constexpr bool NATIVE_V55_H8_TRANSPORT = true;
	// Large enough for the test-mode harness to replay a known prefix and still
	// sweep the full 0..47 logical scanq space in one unattended run.
	static constexpr u8 MAX_SCRIPT_PULSES = 64;
	static constexpr u8 MAX_HOST_PANEL_PULSES = 32;

	struct host_panel_pulse
	{
		s16 row = -1;
		s16 bit = -1;
		double end = 0.0;
	};

	HD44780_PIXEL_UPDATE(lcd_pixel_update);
	TIMER_CALLBACK_MEMBER(mailbox_lie_tick);
	TIMER_CALLBACK_MEMBER(v55_h8_service_tick);
	TIMER_CALLBACK_MEMBER(v55_h8_tx_bit_tick);
	TIMER_CALLBACK_MEMBER(h8_txd_sample_tick);
	TIMER_CALLBACK_MEMBER(v55_txd1_sample_tick);
	TIMER_CALLBACK_MEMBER(gui_midi_rx_bit_tick);
	TIMER_CALLBACK_MEMBER(host_service_tick);
	TIMER_CALLBACK_MEMBER(inject_note_tick);   // KPROP_INJECT_NOTE: deterministic tick-exact note on/off
	TIMER_CALLBACK_MEMBER(inject_sysex_tick);  // KPROP_INJECT_SYSEX: enqueue a .syx blob at a tick-exact time
	TIMER_CALLBACK_MEMBER(dsp_empty_ready_tick);
	TIMER_CALLBACK_MEMBER(dsp_board_sync_tick);
	void lcd_trace_control(u8 value);
	void lcd_trace_data(u8 value);
	void lcd_trace_dump();
	void inject_v55_input_code(address_space &space, u16 code, u8 pulse_idx, bool release);
	void inject_v55_scanq_code(address_space &space, u8 code, u8 pulse_idx, bool release);
	void inject_v55_evtq_record(address_space &space, u16 ax, u16 dx, const char *tag, u8 seq, s16 forced_seg = -1);
	void enqueue_gui_midi_bytes(std::vector<u8> &&bytes, const std::string &tag);
	void drain_host_midi();
	void drain_host_panel();
	void drain_host_adin();
	void push_host_lcd();
	void render_lcd_snapshot(char *line1, char *line2) const;
	void poll_host_scanq_inputs(address_space &space);
	u8 read_matrix_row(u8 row);
	u8 read_scanned_inputs();
	u8 read_special_inputs();
	u8 read_v55_p0_status();
	u8 active_scan_row() const;
	void update_led_bank(u8 value);
	void set_led_output(u8 index, bool state);
	void log_led_output_bank(u8 bank);
	void log_led_output_bank_if_changed(u8 bank);
	void update_panel_led_driver(u8 driver, u16 value);
	void update_panel_led_serial(u8 old_data, u8 data);
	void apply_auto_stimulus(u8 row, u8 &panel);
	bool host_panel_stimulus_active(u8 row) const;
	void apply_auto_kbd_stimulus(u8 row, u8 &kbd);
	bool auto_kbd_stimulus_active(u8 row) const;
	u8 v55_shadow_byte(u8 reg) const;
	void set_v55_shadow_byte(u8 reg, u8 data);
	u8 v55_sfr_byte(u8 reg) const;
	void v55_sfr_write_byte(u8 reg, u8 data);
	u8 v55_p0_r();
	void v55_p0_w(u8 data);
	u8 v55_p1_r();
	u8 v55_p2_r();
	void v55_p2_w(u8 data);
	void v55_p3_w(u8 data);
	u8 v55_p4_r();
	u8 v55_p8_r();
	void v55_p8_w(u8 data);
	void v55_adc_fint_w(int state);
	u8 v55_p7_r();
	void v55_p5_w(u8 data);
	void v55_p7_w(u8 data);
	u16 v55_scratch_r(offs_t offset, u16 mem_mask = ~0);
	void v55_scratch_w(offs_t offset, u16 data, u16 mem_mask = ~0);
	u16 sysram_r(offs_t offset, u16 mem_mask = ~0);
	void sysram_w(offs_t offset, u16 data, u16 mem_mask = ~0);
	template <unsigned Channel> u8 v55_adc_r();
	template <unsigned Channel> u16 h8_adc_r();
	u8 h8_porta_r();
	u8 h8_portb_r();
	u8 h8_portc_r();
	void h8_porta_w(u8 data);
	void h8_portc_w(u8 data);
	void h8_port8_w(u8 data);
	void update_dsp_mute();
	attotime h8_sci0_bit_period() const;
	void start_v55_h8_tx(u8 data);
	void poll_h8_tx_ring();
	void v55_txd_w(int state);
	void v55_txd1_w(int state);
	void handle_midi_tx_byte(u8 data);
	void midi_rxd1_w(int state);
	void h8_txd0_w(int state);
	void pulse_h8_irq1(const char *reason);
	void update_h8_cts_from_port8();
	u8 h8_sram_r8(offs_t offset);
	void h8_sram_w8(offs_t offset, u8 data);
	u8 h8_nocycle_r8(u32 addr);
	u16 h8_nocycle_r16(u32 addr);
	u32 h8_nocycle_r32(u32 addr);
	u8 h8_dsp_r8(offs_t offset);
	void h8_dsp_w8(offs_t offset, u8 data);
	u16 h8_busctrl_r(offs_t offset, u16 mem_mask = ~0);
	void h8_busctrl_w(offs_t offset, u16 data, u16 mem_mask = ~0);

	u16 io_r(offs_t offset, u16 mem_mask = ~0);
	void io_w(offs_t offset, u16 data, u16 mem_mask = ~0);
	u16 mmio_r(offs_t offset, u16 mem_mask = ~0);
	void mmio_w(offs_t offset, u16 data, u16 mem_mask = ~0);

	void prog_map(address_map &map) ATTR_COLD;
	void h8_map(address_map &map) ATTR_COLD;
	void dsp_ram_map(address_map &map) ATTR_COLD;
	void dsp_noram_map(address_map &map) ATTR_COLD;
	void io_map(address_map &map) ATTR_COLD;
	void palette_init(palette_device &palette);
	void dsp1_empty_w(int state);
	void dsp2_empty_w(int state);
	void dsp3_empty_w(int state);
	void update_dsp_host_ctrl();

	required_device<v55_device> m_maincpu;
	required_device<h83003_device> m_subcpu;
	required_device<tms57002_device> m_dsp1;
	required_device<tms57002_device> m_dsp2;
	required_device<tms57002_device> m_dsp3;
	required_device<korgprophecy_u2_device> m_u2;
	required_device<hd44780_a00_reconstructed_device> m_lcdc;
	required_shared_ptr<u16> m_sysram;
	optional_ioport_array<8> m_panel;
	optional_ioport_array<8> m_kbd;
	optional_ioport m_cfg;
	output_finder<96> m_leds;

	u16 m_mmio_shadow[0x80]{};
	std::array<u16, 0x80> m_v55_scratch{};
	bool m_lcd_data_mode = false;
	bool m_lcd_swap = false;
	std::array<u8, 0x80> m_lcd_ddram{};
	std::array<u8, 0x40> m_lcd_cgram{};
	u8 m_lcd_addr = 0x00;
	bool m_lcd_entry_inc = true;
	bool m_lcd_entry_shift = false;
	bool m_lcd_cgram_mode = false;
	int m_lcd_disp_shift = 0;
	u64 m_lcd_dump_count = 0;
	u8 m_scan_sel = 0;
	u8 m_last_scan_sel = 0;
	u8 m_scan_pattern = 0xff;
	u8 m_input_ctrl = 0x05;
	std::array<u8, 12> m_led_output_bank{};
	std::array<u8, 12> m_led_output_logged_bank{};
	std::array<bool, 12> m_led_output_logged_valid{};
	std::array<u16, 2> m_m66311_shift{};
	std::array<u16, 2> m_m66311_latch{};
	std::array<u8, 2> m_m66311_bits{};
	bool m_m66311_seen_p2 = false;
	std::array<u8, 0x20000> m_h8_sram{};
	std::array<u8, 8> m_last_input_by_row{};
	emu_timer *m_mb_lie_timer = nullptr;
	u32 m_dsp_sync_hz = 48'000;
	u8 m_dsp_feedback_mode = u8(dsp_board_feedback_mode::delay1);
	// The schematic feed is DSP1 (oscillator) -> DSP2 (filter). Alternate
	// historical routes remain available for explicit comparison.
	u8 m_dsp2_input_route_mode = u8(dsp_board_dsp2_input_mode::normal);
	// A/B knob: when false, PC7 mute gates only the DAC/stream tap, not the
	// DSP->DSP serial routing (hardware mute wiring unverified; audit Â§3.2).
	bool m_dsp_mute_inter_dsp = true;
	u32 dsp_route_so(tms57002_device &dsp, int index) const
	{ return m_dsp_mute_inter_dsp ? dsp.serial_output(index) : dsp.serial_output_unmuted(index); }
	// A/B knob: route PLOAD/CLOAD strobes only to the most recently
	// data-addressed DSP instead of broadcasting to all three (board-level
	// strobe gating is unverified; see code audit findings Â§2.1/Â§5).
	bool m_dsp_strobe_addressed_only = false;
	int m_dsp_strobe_last_target = -1;
	bool m_boot_combo_requested = false;

	bool m_pulse_queue_inject = false;
	bool m_pulse_scanq_inject = false;
	bool m_pulse_virtual_only = false;
	bool m_evtq_note_inject = false;
	emu_timer *m_v55_h8_service_timer = nullptr;
	emu_timer *m_v55_h8_tx_timer = nullptr;
	emu_timer *m_h8_txd_sample_timer = nullptr;
	emu_timer *m_v55_txd1_sample_timer = nullptr;
	emu_timer *m_gui_midi_rx_timer = nullptr;
	emu_timer *m_host_service_timer = nullptr;

	// KPROP_INJECT_NOTE=<on_secs>[:<off_secs>[:<note>[:<vel>]]] — deterministic tick-exact MIDI note.
	// Enqueues note-on/off bytes into the GUI MIDI RX path at exact EMULATED times (bypasses
	// -midiin wall-clock delivery); makes sustain-vs-release measurement reliable.
	emu_timer *m_inject_note_timer = nullptr;
	double m_inject_note_on = -1.0;
	double m_inject_note_off = -1.0;
	u8 m_inject_note = 60;
	u8 m_inject_note_vel = 64;
	int m_inject_note_phase = 0;
	// KPROP_INJECT_SYSEX=<file>[:<at_secs>] — enqueue a raw .syx blob (e.g. a full 619-byte program-data
	// dump) via the reliable GUI-midi RX path. -midiin drops sysex, so this is how custom patches load.
	emu_timer *m_inject_sysex_timer = nullptr;
	std::vector<u8> m_inject_sysex_bytes;

	emu_timer *m_dsp_sync_timer = nullptr;
	double m_evtq_note_start = 2.20;
	double m_evtq_note_len = 0.35;
	u64 m_dsp_sync_ticks = 0;
	u32 m_dsp_sdn_input = 0x000000;
	u32 m_dsp_optionin_input = 0x00ffffff;
	// Host windows follow the schematic/service-manual chain: DSP1 oscillator,
	// DSP2 filter, DSP3 effects/DAC feeder. The former {2,1,0} default inverted
	// those roles and prevented effects output from reaching the DAC.
	std::array<u8, 3> m_dsp_host_map{{0, 1, 2}};
	// KPSHIP-KEEP: hardware-serial verification tap state (installed in machine_start). Diagnostic, default-inactive.
	double m_dspserout_start = 0.0;
	double m_dspserout_end = 0.0;
	u32 m_dspserout_max = 0;
	std::array<u32, 3> m_dspserout_count{};
	std::array<u32, 2> m_dsp2_so1_prev{};
	std::array<u32, 4> m_dsp3_so_prev{};
	u16 m_txsm_latched_mbseg = 0xffff;
	bool m_last_scanq_host_valid = false;
	std::array<u8, 6> m_last_scanq_host{};
	std::array<u8, 8> m_adc_level{};
	std::array<u8, 4> m_adc_level_override{};
	std::array<u8, 16> m_adin_level{};
	bool m_adc_mux_enable = true;
	bool m_adc_mux_from_p2 = true;
	bool m_adc_mux_from_phase_shadow = false;
	bool m_adc_mux_autoscan = false;
	bool m_adc_mux_phase_on_fint = false; // diagnostic hook only; FINT must not drive the board ADC mux
	u8 m_adc_sel = 0;
	bool m_adc_sel_locked = false;
	u8 m_adc_autoscan_sel = 0;
	u8 m_last_adc_value = 0xff;
	std::array<u16, 8> m_h8_adc_level{};
	s16 m_last_stim_step = -1;
	s16 m_evtq_note = -1;
	s16 m_evtq_velocity = 0x64;
	s16 m_evtq_channel = -1;
	s32 m_evtq_force_seg = -1;
	s32 m_gui_evtq_seg = 0x03fd;
	s16 m_boot_testmode_sel = -1;
	u8 m_evtq_note_state = 0;
	u8 m_boot_mode = 0;
	double m_boot_hold_secs = 2.0;
	s16 m_pulse_row = -1;
	s16 m_pulse_bit = -1;
	double m_pulse_start = 8.0;
	double m_pulse_len = 0.4;
	s16 m_kbd_pulse_row = -1;
	s16 m_kbd_pulse_bit = -1;
	double m_kbd_pulse_start = 8.0;
	double m_kbd_pulse_len = 0.4;
	u8 m_kbd_script_pulse_count = 0;
	std::array<s16, MAX_SCRIPT_PULSES> m_kbd_script_pulse_row{};
	std::array<s16, MAX_SCRIPT_PULSES> m_kbd_script_pulse_bit{};
	std::array<double, MAX_SCRIPT_PULSES> m_kbd_script_pulse_start{};
	std::array<double, MAX_SCRIPT_PULSES> m_kbd_script_pulse_len{};
	u8 m_script_pulse_count = 0;
	std::array<s16, MAX_SCRIPT_PULSES> m_script_pulse_row{};
	std::array<s16, MAX_SCRIPT_PULSES> m_script_pulse_bit{};
	std::array<double, MAX_SCRIPT_PULSES> m_script_pulse_start{};
	std::array<double, MAX_SCRIPT_PULSES> m_script_pulse_len{};
	std::array<u8, MAX_SCRIPT_PULSES> m_script_pulse_queue_state{};
	std::array<host_panel_pulse, MAX_HOST_PANEL_PULSES> m_host_panel_pulses{};
	u8 m_host_panel_pulse_next = 0;
	double m_host_lcd_last_push = 0.0;
	std::string m_host_lcd_last_l1;
	std::string m_host_lcd_last_l2;
	std::array<u8, 40> m_host_lcd_last_raw1{};
	std::array<u8, 40> m_host_lcd_last_raw2{};
	std::array<u8, 0x40> m_host_lcd_last_cgram{};
	u8 m_h8_porta = 0xff;
	u8 m_h8_portc = 0xff;
	u8 m_h8_port8 = 0xff;
	u8 m_h8_cts_state = 0;
	u8 m_v55_txd_state = 1;
	attotime m_v55_txd_byte_end = attotime::zero;
	u8 m_v55_txd1_state = 1;
	u8 m_midi_rxd1_state = 1;
	u8 m_h8_txd_state = 1;
	u32 m_v55_txd_edges = 0;
	u32 m_v55_txd1_edges = 0;
	u32 m_midi_rxd1_edges = 0;
	u32 m_h8_txd_edges = 0;
	std::deque<u8> m_gui_midi_rx_queue;
	bool m_gui_midi_rx_active = false;
	u16 m_gui_midi_rx_frame = 0x03ff;
	u8 m_gui_midi_rx_bit = 0;
	u64 m_gui_midi_rx_enqueued = 0;
	u64 m_gui_midi_rx_sent = 0;
	u64 m_gui_midi_rx_last_progress_sent = 0;
	double m_gui_midi_rx_last_progress_time = 0.0;
	bool m_v55_h8_tx_active = false;
	u32 m_last_h8_tx_wptr = 0xffffffffU;
	bool m_h8_txd_decode_active = false;
	u8 m_h8_txd_decode_byte = 0x00;
	u8 m_h8_txd_decode_bit = 0x00;
	bool m_v55_txd1_decode_active = false;
	u8 m_v55_txd1_decode_byte = 0x00;
	u8 m_v55_txd1_decode_bit = 0x00;
	std::vector<u8> m_midi_tx_sysex;
	std::array<u8, 4> m_h8_busctrl{ 0xff, 0xff, 0xff, 0xff };
	u8 m_dsp_pload_level = 1;
		u8 m_dsp_empty = 0x07;
	std::array<emu_timer *, 3> m_dsp_empty_timer{};
	u32 m_dsp_empty_hold_us = 2;
	bool m_card_present = false;
	bool m_card_write_protect = false;
	u8 m_last_card_p0_status = 0xff;
	memory_passthrough_handler m_v55_status_tap;
	memory_passthrough_handler m_h8_cmdq_tap;
	memory_passthrough_handler m_h8_rxfsm_read_tap;
	memory_passthrough_handler m_h8_rxfsm_write_tap;
	memory_passthrough_handler m_h8_pload_selector_live_tap;
	memory_passthrough_handler m_h8_pload_selector_table_tap;
	memory_passthrough_handler m_h8_dsp_state_tap;
	memory_passthrough_handler m_h8_dsp_filter_source_read_tap;
	memory_passthrough_handler m_h8_dsp_filter_source_write_tap;
	memory_passthrough_handler m_h8_source_state_write_tap;
};

// Minimal statically-linked host ABI used by prophecy-plugin.  Standalone MAME
// leaves these callbacks null, making every host service a no-op.
extern "C" {
typedef bool (*kprop_host_midi_pop_fn)(uint8_t *out, size_t cap, size_t *n, double emu_seconds);
static kprop_host_midi_pop_fn g_kprop_host_midi_pop = nullptr;
void kprop_set_host_midi_pop(kprop_host_midi_pop_fn fn) { g_kprop_host_midi_pop = fn; }

typedef void (*kprop_host_midi_tx_fn)(const uint8_t *bytes, size_t n);
static kprop_host_midi_tx_fn g_kprop_host_midi_tx = nullptr;
void kprop_set_host_midi_tx(kprop_host_midi_tx_fn fn) { g_kprop_host_midi_tx = fn; }

typedef void (*kprop_host_midi_tx_byte_fn)(uint8_t data, double emu_seconds);
static kprop_host_midi_tx_byte_fn g_kprop_host_midi_tx_byte = nullptr;
void kprop_set_host_midi_tx_byte(kprop_host_midi_tx_byte_fn fn) { g_kprop_host_midi_tx_byte = fn; }

typedef void (*kprop_host_lcd_fn)(const char *line1, const char *line2);
static kprop_host_lcd_fn g_kprop_host_lcd = nullptr;
void kprop_set_host_lcd(kprop_host_lcd_fn fn) { g_kprop_host_lcd = fn; }

typedef void (*kprop_host_lcd_raw_fn)(const uint8_t *row1, const uint8_t *row2, const uint8_t *cgram);
static kprop_host_lcd_raw_fn g_kprop_host_lcd_raw = nullptr;
void kprop_set_host_lcd_raw(kprop_host_lcd_raw_fn fn) { g_kprop_host_lcd_raw = fn; }

typedef bool (*kprop_host_panel_pop_fn)(uint8_t *row, uint8_t *bit, uint16_t *len_ms,
	double emu_seconds);
static kprop_host_panel_pop_fn g_kprop_host_panel_pop = nullptr;
void kprop_set_host_panel_pop(kprop_host_panel_pop_fn fn) { g_kprop_host_panel_pop = fn; }

typedef bool (*kprop_host_adin_pop_fn)(uint8_t *source, uint8_t *value, double emu_seconds);
static kprop_host_adin_pop_fn g_kprop_host_adin_pop = nullptr;
void kprop_set_host_adin_pop(kprop_host_adin_pop_fn fn) { g_kprop_host_adin_pop = fn; }

typedef void (*kprop_host_led_fn)(uint8_t bank, uint8_t data);
static kprop_host_led_fn g_kprop_host_led = nullptr;
void kprop_set_host_led(kprop_host_led_fn fn) { g_kprop_host_led = fn; }
}

void korgprophecy_state::machine_start()
{
	m_dsp2_input_route_mode = u8(parse_dsp2_input_route_env(std::getenv("KPROP_DSP2_INPUT_ROUTE")));

	// Serial input staging model. Default = SNAPSHOT (false), which is VALIDATED against the hardware
	// real-patch corpus (2026-07-04): B52 "303Growler" reproduces the captured hw voice under snapshot
	// (~0.83x hw at matched envelope) but is DEAD SILENT under the LRCK-subframe model — so the
	// "LRCK-aware" one-frame-delay approximation is WRONG for hardware, not more accurate. Matches the
	// release driver + GUI (both use snapshot). The old default (lrck_subframe=true) silenced B52 + the
	// ~20% modulation/feedback patch class and was the recurring "B52 is silent" trap in research runs.
	// Snapshot staging is hardware-validated; the superseded LRCK-subframe
	// approximation silenced a substantial modulation/feedback patch class.
	m_dsp1->set_serial_frame_model(false);
	m_dsp2->set_serial_frame_model(false);
	m_dsp3->set_serial_frame_model(false);
	int dsp_frame_clocks = 512;
	m_dsp1->set_serial_frame_clocks(dsp_frame_clocks);
	m_dsp2->set_serial_frame_clocks(dsp_frame_clocks);
	m_dsp3->set_serial_frame_clocks(dsp_frame_clocks);
	m_dsp1->set_sync_polarity(1);
	m_dsp2->set_sync_polarity(1);
	m_dsp3->set_sync_polarity(1);
	// Prophecy DSP3 incoming subframe orientation differs from the generic
	// helper default. Keep the correction driver-local so DSP1/DSP2 and other
	// TMS57002 users are unaffected.
	m_dsp3->set_serial_frame_flip_input(true);
	m_dsp1->set_serial_frame_flip_output(false);
	// The DOS-write-point output latch follows the voice program (host window
	// 1); it is assigned after KPROP_DSP_HOST_MAP parsing below.
	// Later framing/latch fixes made the old DSP2 subframe-flip bring-up
	// workaround unnecessary; hardware-scoreboard A/Bs were byte-identical with
	// it disabled, so use the plain serial orientation.
	m_dsp2->set_serial_frame_flip_output(false);
	m_dsp3->set_serial_frame_flip_output(false);
	// Silicon truncation is on the multiplier A port (the device default), not
	// the SMHC store. Keep the store at full macc>>16; the old value 8 applied
	// truncation twice after the multiplier fix landed.
	int smhc_trunc_bits = 0;
	if (const char *e = std::getenv("KPROP_DSP_SMHC_TRUNC_BITS"))
		smhc_trunc_bits = std::clamp(std::atoi(e), 0, 16);
	m_dsp1->set_smhc_trunc_bits(smhc_trunc_bits);
	m_dsp2->set_smhc_trunc_bits(smhc_trunc_bits);
	m_dsp3->set_smhc_trunc_bits(smhc_trunc_bits);

	// TMS57002 pooled dynarec DEFAULT-ON for the Prophecy. Gated by the 2026-07-07 F6 corpus sweep:
	// 36 distinct programs (16 factory + 20 synthetic configs) WAV byte-identical interp-vs-pooled,
	// busy-load ~130-145% of realtime on this class of machine, hardware serial scoreboard unchanged
	// (36/36 programs). KPROP_DSP_PERFRAME overrides: 4 = force on, any
	// other value = force interpreter (kill switch). Non-x86-64 hosts fall back to the interpreter.
	m_dsp1->set_dynarec_default(true);
	m_dsp2->set_dynarec_default(true);
	m_dsp3->set_dynarec_default(true);

	// KPSHIP-KEEP: hardware-bit-exact VERIFICATION tap. The DSP serial output that the 06-08 hardware
	// scoreboard (scripts/korgprophecy_serial_scoreboard.py) and the injection word-compare grade against
	// is observed here through the tms57002 core's generic set_serial_output_observer -- so the shared DSP
	// core stays getenv/log-free and this fork-specific diagnostic lives in the driver. Default-INACTIVE
	// (census/normal runs are byte-identical); enable with KPROP_TRACE_DSP_SERIAL_OUTPUT_TAG=<tag-substr>
	// (+ optional _START/_END/_MAX). Emits the KPROP_DSPSEROUT rows the scoreboard parses.
	if (const char *tagsel = std::getenv("KPROP_TRACE_DSP_SERIAL_OUTPUT_TAG"))
	{
		m_dspserout_start = 0.0;
		m_dspserout_end = 30.0;
		m_dspserout_max = 1024;
		if (const char *e = std::getenv("KPROP_TRACE_DSP_SERIAL_OUTPUT_START")) m_dspserout_start = std::strtod(e, nullptr);
		if (const char *e = std::getenv("KPROP_TRACE_DSP_SERIAL_OUTPUT_END"))   m_dspserout_end = std::strtod(e, nullptr);
		if (const char *e = std::getenv("KPROP_TRACE_DSP_SERIAL_OUTPUT_MAX"))
		{
			const long v = std::strtol(e, nullptr, 0);
			if (v > 0 && v <= 100000)
				m_dspserout_max = u32(v);
		}
		tms57002_device *const dsps[3] = { &*m_dsp1, &*m_dsp2, &*m_dsp3 };
		for (int i = 0; i < 3; i++)
		{
			if (tagsel[0] && std::strstr(dsps[i]->tag(), tagsel) == nullptr)
				continue;
			const int slot = i;
			tms57002_device *const dev = dsps[i];
			dev->set_serial_output_observer(
				[this, slot, dev](const char *event, int index, u32 value, int halfcycles, u8 mask)
				{
					const double now = machine().time().as_double();
					if (now < m_dspserout_start || now > m_dspserout_end)
						return;
					if (m_dspserout_count[slot] >= m_dspserout_max)
						return;
					m_dspserout_count[slot]++;
					logerror("KPROP_DSPSEROUT,T=%.6f,TAG=%s,EV=%s,IDX=%d,VAL=%06X,HC=%d,MASK=%02X\n",
						now, dev->tag(), event, index, value & 0x00ffffffU, halfcycles, unsigned(mask));
				});
		}
	}

	save_item(NAME(m_mmio_shadow));
	save_item(NAME(m_v55_scratch));
	save_item(NAME(m_lcd_data_mode));
	save_item(NAME(m_lcd_swap));
	save_item(NAME(m_lcd_ddram));
	save_item(NAME(m_lcd_cgram));
	save_item(NAME(m_lcd_addr));
	save_item(NAME(m_lcd_entry_inc));
	save_item(NAME(m_lcd_entry_shift));
	save_item(NAME(m_lcd_cgram_mode));
	save_item(NAME(m_lcd_disp_shift));
	save_item(NAME(m_lcd_dump_count));
	save_item(NAME(m_scan_sel));
	save_item(NAME(m_last_scan_sel));
	save_item(NAME(m_scan_pattern));
	save_item(NAME(m_input_ctrl));
	save_item(NAME(m_led_output_bank));
	save_item(NAME(m_m66311_shift));
	save_item(NAME(m_m66311_latch));
	save_item(NAME(m_m66311_bits));
	save_item(NAME(m_m66311_seen_p2));
	save_item(NAME(m_h8_sram));
	save_item(NAME(m_last_input_by_row));
	save_item(NAME(m_dsp2_input_route_mode));
	save_item(NAME(m_dsp_sdn_input));
	save_item(NAME(m_dsp_optionin_input));
	save_item(NAME(m_pulse_queue_inject));
	save_item(NAME(m_pulse_scanq_inject));
	save_item(NAME(m_pulse_virtual_only));
	save_item(NAME(m_evtq_note_inject));
	save_item(NAME(m_evtq_note_start));
	save_item(NAME(m_evtq_note_len));
	save_item(NAME(m_v55_h8_tx_active));
	save_item(NAME(m_last_h8_tx_wptr));
	save_item(NAME(m_h8_txd_decode_active));
	save_item(NAME(m_h8_txd_decode_byte));
	save_item(NAME(m_h8_txd_decode_bit));
	save_item(NAME(m_last_scanq_host_valid));
	save_item(NAME(m_last_scanq_host));
	save_item(NAME(m_adc_level));
	save_item(NAME(m_adc_level_override));
	save_item(NAME(m_adin_level));
	save_item(NAME(m_adc_mux_enable));
	save_item(NAME(m_adc_mux_from_p2));
	save_item(NAME(m_adc_mux_from_phase_shadow));
	save_item(NAME(m_adc_mux_autoscan));
	save_item(NAME(m_adc_mux_phase_on_fint));
	save_item(NAME(m_adc_sel));
	save_item(NAME(m_adc_sel_locked));
	save_item(NAME(m_adc_autoscan_sel));
	save_item(NAME(m_last_adc_value));
	save_item(NAME(m_h8_adc_level));
	save_item(NAME(m_last_stim_step));
	save_item(NAME(m_evtq_note));
	save_item(NAME(m_evtq_velocity));
	save_item(NAME(m_evtq_channel));
	save_item(NAME(m_evtq_force_seg));
	save_item(NAME(m_gui_evtq_seg));
	save_item(NAME(m_boot_testmode_sel));
	save_item(NAME(m_evtq_note_state));
	save_item(NAME(m_boot_mode));
	save_item(NAME(m_boot_hold_secs));
	save_item(NAME(m_boot_combo_requested));
	save_item(NAME(m_pulse_row));
	save_item(NAME(m_pulse_bit));
	save_item(NAME(m_pulse_start));
	save_item(NAME(m_pulse_len));
	save_item(NAME(m_kbd_pulse_row));
	save_item(NAME(m_kbd_pulse_bit));
	save_item(NAME(m_kbd_pulse_start));
	save_item(NAME(m_kbd_pulse_len));
	save_item(NAME(m_kbd_script_pulse_count));
	save_item(NAME(m_kbd_script_pulse_row));
	save_item(NAME(m_kbd_script_pulse_bit));
	save_item(NAME(m_kbd_script_pulse_start));
	save_item(NAME(m_kbd_script_pulse_len));
	save_item(NAME(m_script_pulse_count));
	save_item(NAME(m_script_pulse_row));
	save_item(NAME(m_script_pulse_bit));
	save_item(NAME(m_script_pulse_start));
	save_item(NAME(m_script_pulse_len));
	save_item(NAME(m_script_pulse_queue_state));
	save_item(NAME(m_h8_porta));
	save_item(NAME(m_h8_portc));
	save_item(NAME(m_h8_port8));
	save_item(NAME(m_h8_cts_state));
	save_item(NAME(m_v55_txd_state));
	save_item(NAME(m_v55_txd1_state));
	save_item(NAME(m_h8_txd_state));
	save_item(NAME(m_v55_txd_edges));
	save_item(NAME(m_v55_txd1_edges));
	save_item(NAME(m_h8_txd_edges));
	save_item(NAME(m_v55_txd1_decode_active));
	save_item(NAME(m_v55_txd1_decode_byte));
	save_item(NAME(m_v55_txd1_decode_bit));
	save_item(NAME(m_h8_busctrl));
	save_item(NAME(m_dsp_pload_level));
	save_item(NAME(m_dsp_empty));
	save_item(NAME(m_card_present));
	save_item(NAME(m_card_write_protect));
	save_item(NAME(m_last_card_p0_status));
	save_item(NAME(m_dsp_sync_ticks));
	save_item(NAME(m_dsp_sync_hz));
	save_item(NAME(m_dsp_feedback_mode));
	save_item(NAME(m_dsp_host_map));
	save_item(NAME(m_dsp2_so1_prev));
	save_item(NAME(m_dsp3_so_prev));

	// KPSHIP-BAKE: default false is the working LCD ctrl/data wiring (single hw variant).
	// Delete the env override; m_lcd_swap folds to constant false.
	if (const char *env = std::getenv("KPROP_LCD_SWAP"))
		m_lcd_swap = (std::strtol(env, nullptr, 0) != 0);

	m_script_pulse_count = 0;
	m_script_pulse_row.fill(-1);
	m_script_pulse_bit.fill(-1);
	m_script_pulse_start.fill(0.0);
	m_script_pulse_len.fill(0.0);
	m_script_pulse_queue_state.fill(0);
	m_kbd_script_pulse_count = 0;
	m_kbd_script_pulse_row.fill(-1);
	m_kbd_script_pulse_bit.fill(-1);
	m_kbd_script_pulse_start.fill(0.0);
	m_kbd_script_pulse_len.fill(0.0);

	// Backward compatibility: single pulse env vars still work if no script was provided.
	if (m_script_pulse_count == 0 && m_pulse_row >= 0 && m_pulse_bit >= 0)
	{
		m_script_pulse_count = 1;
		m_script_pulse_row[0] = m_pulse_row;
		m_script_pulse_bit[0] = m_pulse_bit;
		m_script_pulse_start[0] = m_pulse_start;
		m_script_pulse_len[0] = m_pulse_len;
	}
	if (m_kbd_script_pulse_count == 0 && m_kbd_pulse_row >= 0 && m_kbd_pulse_bit >= 0)
	{
		m_kbd_script_pulse_count = 1;
		m_kbd_script_pulse_row[0] = m_kbd_pulse_row;
		m_kbd_script_pulse_bit[0] = m_kbd_pulse_bit;
		m_kbd_script_pulse_start[0] = m_kbd_pulse_start;
		m_kbd_script_pulse_len[0] = m_kbd_pulse_len;
	}

	m_adc_level.fill(0x00);
	m_adc_level_override.fill(0x00);
	m_adin_level.fill(0x00);
	// Physical rest positions.
	// WHEEL1 (ADIN8) is spring-centered, so it rests at mid-scale 0x80; this
	// makes the 54/70/20 hardware note-on family exact.
	// RIBON_CTL_Y (ADIN13) is the SPRING-CENTERED "log" control (service
	// test-mode "wheel 3"), NOT the touch strip and NOT un-sprung. It rests at
	// the sprung mid-HIGH center. Bench test-mode ADC (2026-07-04, 7-bit
	// display): fully-down 0x01, RELEASED(rest) 0x3a, fully-up 0x7d ->
	// firmware 8-bit rest ~= 0x74. (The old 0x1C "rests at the bottom, no
	// spring return" was wrong: it evaluated the ribbon-Y mod at the bottom
	// extreme instead of the sprung center.) At 0x74 the A00 OSC1-mixer level
	// mod sits in its 0.25 clamp (firmware clamps ADIN13 >= ~0x68) matching
	// hardware base, AND B52 (303Growler) sounds (0.99x hw) - both correct at
	// one rest, no touch-gate, no firmware edit. Verified vs the 2026-07-04
	// real-patch corpus: A40 1.00x / A30 1.01x / A00 0.95x / B30 0.86x / B52
	// 0.99x (was A00 0.32x at 0x1C). 0x74 is a x2 estimate of the 7-bit
	// display; refine against captures-2026-07-04/ribbon-log/ down/rest/up if a
	// tighter scale is needed. KPROP_ADIN8/13 and live GUI input still override.
	// SPEED/tempo (ADIN0) is a physical pot and never reaches the emulation-only
	// exact-zero discontinuity. Hardware sweeps prove all nonzero values are
	// voice-invariant, so 0x01 represents its physical minimum.
	m_adin_level[0] = 0x01;
	// BATT_SENSE (ADIN7) represents a fresh ~3.25 V lithium cell on the V55 ADC
	// path; values below ~0x94 intentionally exercise the battery warning.
	m_adin_level[7] = 0xa6;
	m_adin_level[8] = 0x80;
	// WHEEL2 and ribbon X are spring-centered in the controller model.
	m_adin_level[9] = 0x80;
	m_adin_level[12] = 0x80;
	m_adin_level[13] = 0x74;
	m_adc_mux_from_p2 = false;
	m_adc_mux_from_phase_shadow = true;
	m_adc_mux_autoscan = false;
	m_adc_mux_phase_on_fint = false;
	m_adc_autoscan_sel = 0;
	m_adc_sel_locked = false;
	m_h8_adc_level.fill(0x0200);
	// H8 ADIN7 is the internal battery voltmeter. A fresh 3V backup battery
	// reads near the top of the divider range, so rest it at full scale so the
	// firmware's ADC battery check always sees a healthy battery (no
	// "*INTERNAL BATTERY IS LOW" warning) without any crutch.
	// KPSHIP-KEEP: intentional. Present a fresh battery (full scale) to the firmware's
	// ADC check so it never warns — the clean way, no completion crutch. Do not remove.
	m_h8_adc_level[7] = 0x03ff;
	// Keep the parser as an explicit topology A/B seam; the compiled default is
	// the schematic identity map.
	if (const char *env = std::getenv("KPROP_DSP_HOST_MAP"))
	{
		std::array<u8, 3> parsed{};
		std::array<bool, 3> seen{};
		const std::string spec(env);
		size_t pos = 0;
		u8 index = 0;
		bool ok = true;
		while (ok && index < 3)
		{
			while (pos < spec.size() && (spec[pos] == ',' || spec[pos] == ';' || std::isspace(u8(spec[pos]))))
				pos++;
			const size_t start = pos;
			while (pos < spec.size() && spec[pos] != ',' && spec[pos] != ';' && !std::isspace(u8(spec[pos])))
				pos++;
			if (start == pos)
			{
				ok = false;
				break;
			}

			char *end = nullptr;
			const std::string token = spec.substr(start, pos - start);
			const long value = std::strtol(token.c_str(), &end, 0);
			if (!end || *end != '\0' || value < 1 || value > 3 || seen[value - 1])
			{
				ok = false;
				break;
			}
			parsed[index++] = u8(value - 1);
			seen[value - 1] = true;
		}
		while (pos < spec.size() && (spec[pos] == ',' || spec[pos] == ';' || std::isspace(u8(spec[pos]))))
			pos++;
		if (ok && index == 3 && pos == spec.size())
		{
			m_dsp_host_map = parsed;
		}
		else
		{
			logerror("KPROP_DSPHOSTMAP,INVALID=%s\n", env);
		}
	}
	logerror("KPROP_DSPHOSTMAP,HOST1=DSP%u,HOST2=DSP%u,HOST3=DSP%u\n",
		u32(m_dsp_host_map[0]) + 1,
		u32(m_dsp_host_map[1]) + 1,
		u32(m_dsp_host_map[2]) + 1);
	// The DOS-write-point output latch is a property of the voice program
	// (host window 1, which computes SO before the modeled end-of-frame
	// handoff), not of a fixed device: it must follow the window-1 owner or
	// the voice program's SO writes never reach the framed output
	// (BUILD/FRAME freeze).
	m_dsp1->set_serial_output_write_handoff(m_dsp_host_map[0] == 0);
	m_dsp2->set_serial_output_write_handoff(m_dsp_host_map[0] == 1);
	m_dsp3->set_serial_output_write_handoff(m_dsp_host_map[0] == 2);
	// KPSHIP-FORK: GUI bridge event-queue segment override.
	if (const char *env = std::getenv("KPROP_GUI_EVTQ_SEG"))
	{
		const int v = std::strtol(env, nullptr, 0);
		if (v >= 0x0000 && v <= 0xffff)
			m_gui_evtq_seg = v;
	}
	// KPSHIP-BAKE: the baked m_adin_level rests (WHEEL1 0x80, RIBBON-Y 0x74, else 0x00)
	// are the hw-validated defaults. Delete this per-channel env override loop; keep the
	// constants. (Live GUI input is a separate path.)
	for (u8 ch = 0; ch < 16; ch++)
	{
		const std::string key = "KPROP_ADIN" + std::to_string(ch);
		if (const char *env = std::getenv(key.c_str()))
		{
			const int v = std::strtol(env, nullptr, 0);
			if (v >= 0x00 && v <= 0xff)
			{
				m_adin_level[ch] = u8(v);
				m_adc_mux_enable = true;
			}
		}
	}
	m_mb_lie_timer = timer_alloc(FUNC(korgprophecy_state::mailbox_lie_tick), this);
	m_v55_h8_service_timer = timer_alloc(FUNC(korgprophecy_state::v55_h8_service_tick), this);
	m_v55_h8_tx_timer = timer_alloc(FUNC(korgprophecy_state::v55_h8_tx_bit_tick), this);
	m_h8_txd_sample_timer = timer_alloc(FUNC(korgprophecy_state::h8_txd_sample_tick), this);
	m_v55_txd1_sample_timer = timer_alloc(FUNC(korgprophecy_state::v55_txd1_sample_tick), this);
	m_gui_midi_rx_timer = timer_alloc(FUNC(korgprophecy_state::gui_midi_rx_bit_tick), this);
	m_host_service_timer = timer_alloc(FUNC(korgprophecy_state::host_service_tick), this);
	m_host_service_timer->adjust(attotime::from_hz(1000), 0, attotime::from_hz(1000));
	m_inject_note_timer = timer_alloc(FUNC(korgprophecy_state::inject_note_tick), this);
	// KPSHIP-FORK: deterministic tick-exact note-injection test harness.
	if (const char *env = std::getenv("KPROP_INJECT_NOTE"))
	{
		// format: <on_secs>[:<off_secs>[:<note>[:<vel>]]]
		double on = -1.0, off = -1.0; int note = 60, vel = 64;
		std::sscanf(env, "%lf:%lf:%d:%d", &on, &off, &note, &vel);
		m_inject_note_on = on;
		m_inject_note_off = off;
		m_inject_note = u8(std::clamp(note, 0, 127));
		m_inject_note_vel = u8(std::clamp(vel, 1, 127));
		m_inject_note_phase = 0;
		if (m_inject_note_on >= 0.0)
			m_inject_note_timer->adjust(attotime::from_double(m_inject_note_on));
	}

	m_inject_sysex_timer = timer_alloc(FUNC(korgprophecy_state::inject_sysex_tick), this);
	// KPSHIP-FORK: load a .syx blob via the GUI-midi path (test harness; opens a host file).
	if (const char *env = std::getenv("KPROP_INJECT_SYSEX"))
	{
		// format: <path>[:<at_secs>]  (Unix path has no ':'; split only if the tail parses as a number)
		std::string spec(env);
		double at = 11.0;   // default: past the ~10s boot
		std::string path = spec;
		auto colon = spec.rfind(':');
		if (colon != std::string::npos)
		{
			char *end = nullptr;
			double v = std::strtod(spec.c_str() + colon + 1, &end);
			if (end && *end == '\0') { at = v; path = spec.substr(0, colon); }
		}
		if (FILE *f = std::fopen(path.c_str(), "rb"))
		{
			int c;
			while ((c = std::fgetc(f)) != EOF) m_inject_sysex_bytes.push_back(u8(c));
			std::fclose(f);
			if (!m_inject_sysex_bytes.empty() && at >= 0.0)
				m_inject_sysex_timer->adjust(attotime::from_double(at));
			logerror("KPROP_INJECT_SYSEX,LOADED,PATH=%s,BYTES=%zu,AT=%.3f\n", path.c_str(), m_inject_sysex_bytes.size(), at);
		}
		else
			logerror("KPROP_INJECT_SYSEX,ERROR=OPEN,PATH=%s\n", path.c_str());
	}

	m_dsp_sync_timer = timer_alloc(FUNC(korgprophecy_state::dsp_board_sync_tick), this);
	for (u8 i = 0; i < 3; i++)
		m_dsp_empty_timer[i] = timer_alloc(FUNC(korgprophecy_state::dsp_empty_ready_tick), this);

	m_v55_status_tap.remove();
	m_h8_cmdq_tap.remove();
	m_h8_rxfsm_read_tap.remove();
	m_h8_rxfsm_write_tap.remove();
	m_h8_pload_selector_live_tap.remove();
	m_h8_pload_selector_table_tap.remove();
	m_h8_dsp_state_tap.remove();
	m_h8_dsp_filter_source_read_tap.remove();
	m_h8_dsp_filter_source_write_tap.remove();
	m_h8_source_state_write_tap.remove();
}

void korgprophecy_state::machine_reset()
{
	m_h8_porta = 0xff;
	m_h8_portc = 0xff;
	m_h8_port8 = 0xff;
	m_h8_cts_state = 0;
	m_v55_txd_state = 1;
	m_v55_txd_byte_end = attotime::zero;
	m_v55_txd1_state = 1;
	m_h8_txd_state = 1;
	m_v55_txd_edges = 0;
	m_v55_txd1_edges = 0;
	m_h8_txd_edges = 0;
	m_v55_h8_tx_active = false;
	m_last_h8_tx_wptr = 0xffffffffU;
	m_h8_txd_decode_active = false;
	m_h8_txd_decode_byte = 0x00;
	m_h8_txd_decode_bit = 0x00;
	m_v55_txd1_decode_active = false;
	m_v55_txd1_decode_byte = 0x00;
	m_v55_txd1_decode_bit = 0x00;
	m_midi_tx_sysex.clear();
	m_lcd_ddram.fill(' ');
	m_lcd_cgram.fill(0);
	m_lcd_addr = 0x00;
	m_lcd_entry_inc = true;
	m_lcd_entry_shift = false;
	m_lcd_cgram_mode = false;
	m_lcd_disp_shift = 0;
	for (host_panel_pulse &pulse : m_host_panel_pulses)
		pulse = host_panel_pulse{};
	m_host_panel_pulse_next = 0;
	m_host_lcd_last_push = 0.0;
	m_host_lcd_last_l1.clear();
	m_host_lcd_last_l2.clear();
	m_host_lcd_last_raw1.fill(0);
	m_host_lcd_last_raw2.fill(0);
	m_host_lcd_last_cgram.fill(0);
	update_dsp_mute();
	m_txsm_latched_mbseg = 0xffff;
	m_last_scanq_host_valid = false;
	m_last_scanq_host.fill(0xff);
	m_last_card_p0_status = 0xff;
	m_script_pulse_queue_state.fill(0);
	m_dsp_sync_ticks = 0;
	m_dsp2_so1_prev.fill(0);
	m_dsp_pload_level = 1;
	m_evtq_note_state = 0;
	m_dsp_empty = 0x07;
	update_dsp_host_ctrl();
	m_dsp2->resume(SUSPEND_REASON_DISABLE);
	m_dsp3->resume(SUSPEND_REASON_DISABLE);
	m_subcpu->sci_rx_w<0>(1);
	m_gui_midi_rx_queue.clear();
	m_gui_midi_rx_active = false;
	m_gui_midi_rx_frame = 0x03ff;
	m_gui_midi_rx_bit = 0;
	m_gui_midi_rx_enqueued = 0;
	m_gui_midi_rx_sent = 0;
	m_gui_midi_rx_last_progress_sent = 0;
	m_gui_midi_rx_last_progress_time = 0.0;
	midi_rxd1_w(1);
	m_maincpu->cts_w(m_h8_cts_state);
	if (m_mb_lie_timer != nullptr)
	{
		if (m_pulse_queue_inject || m_pulse_scanq_inject || m_evtq_note_inject)
			m_mb_lie_timer->adjust(attotime::from_usec(100), 0, attotime::from_usec(100));
		else
			m_mb_lie_timer->adjust(attotime::never);
	}
	if (m_v55_h8_service_timer != nullptr)
		m_v55_h8_service_timer->adjust(attotime::from_usec(100), 0, attotime::from_usec(100));
	if (m_v55_h8_tx_timer != nullptr)
		m_v55_h8_tx_timer->adjust(attotime::never);
	if (m_h8_txd_sample_timer != nullptr)
		m_h8_txd_sample_timer->adjust(attotime::never);
	if (m_v55_txd1_sample_timer != nullptr)
		m_v55_txd1_sample_timer->adjust(attotime::never);
	if (m_gui_midi_rx_timer != nullptr)
		m_gui_midi_rx_timer->adjust(attotime::never);
	if (m_dsp_sync_timer != nullptr)
	{
		if (m_dsp_feedback_mode == u8(dsp_board_feedback_mode::delay1) || m_dsp_sdn_input || m_dsp_optionin_input)
			m_dsp_sync_timer->adjust(attotime::from_hz(m_dsp_sync_hz), 0, attotime::from_hz(m_dsp_sync_hz));
		else
			m_dsp_sync_timer->adjust(attotime::never);
	}
	for (emu_timer *timer : m_dsp_empty_timer)
	{
		if (timer != nullptr)
			timer->adjust(attotime::never);
	}
}

void korgprophecy_state::inject_v55_input_code(address_space &space, u16 code, u8 pulse_idx, bool release)
{
	const u16 seg = space.read_word(INPUTQ_PHYS_SEG);
	const u16 count = space.read_word(INPUTQ_PHYS_CNT);
	u16 wptr = space.read_word(INPUTQ_PHYS_WPTR);
	if (count >= 0x0100)
		return;

	const u32 linear = ((u32(seg) << 4) + wptr) & 0x000fffffU;
	space.write_word(linear, code);
	wptr = (wptr + 2) & 0x01ff;
	space.write_word(INPUTQ_PHYS_WPTR, wptr);
	space.write_word(INPUTQ_PHYS_CNT, count + 1);
}

void korgprophecy_state::inject_v55_scanq_code(address_space &space, u8 code, u8 pulse_idx, bool release)
{
	const u16 count = space.read_word(SCANQ_PHYS_CNT);
	u16 wptr = space.read_word(SCANQ_PHYS_WPTR);
	if (count >= 0x0040)
		return;

	space.write_byte(SCANQ_PHYS_BUF + wptr, code);
	wptr++;
	if (wptr >= 0x0040)
		wptr = 0x0000;
	space.write_word(SCANQ_PHYS_WPTR, wptr);
	space.write_word(SCANQ_PHYS_CNT, count + 1);
}

void korgprophecy_state::inject_v55_evtq_record(address_space &space, u16 ax, u16 dx, const char *tag, u8 seq, s16 forced_seg)
{
	const u16 seg = (forced_seg >= 0) ? u16(forced_seg) : (m_evtq_force_seg >= 0) ? u16(m_evtq_force_seg) : space.read_word(EVTQ_PHYS_SEG_PTR);
	const u16 count = space.read_word(EVTQ_PHYS_CNT);
	u16 wptr = space.read_word(EVTQ_PHYS_WPTR);
	if (count >= 0x00ff)
		return;

	const u32 linear = ((u32(seg) << 4) + wptr) & 0x000fffffU;
	space.write_word(linear, ax);
	space.write_word((linear + 2) & 0x000fffffU, dx);
	wptr = (wptr + 4) & 0x03ff;
	space.write_word(EVTQ_PHYS_WPTR, wptr);
	space.write_word(EVTQ_PHYS_CNT, count + 1);
}

void korgprophecy_state::poll_host_scanq_inputs(address_space &space)
{
	const u8 enabled = space.read_byte(SCANQ_PHYS_ENABLE);
	if ((enabled & 0x01) == 0)
	{
		m_last_scanq_host_valid = false;
		return;
	}

	std::array<u8, 6> current{};
	for (u8 row = 0; row < current.size(); row++)
	{
		current[row] = read_matrix_row(row);
		// Host pulses are already visible to firmware through the real matrix
		// read. Mirroring the same transition into the synthetic scan queue
		// turns one faceplate click into two key events.
		if (m_last_scanq_host_valid && host_panel_stimulus_active(row))
			current[row] = m_last_scanq_host[row];
	}

	if (!m_last_scanq_host_valid)
	{
		m_last_scanq_host = current;
		m_last_scanq_host_valid = true;
		return;
	}

	for (u8 row = 0; row < current.size(); row++)
	{
		const u8 changed = current[row] ^ m_last_scanq_host[row];
		if (!changed)
			continue;

		for (u8 bit = 0; bit < 8; bit++)
		{
			if (!BIT(changed, bit))
				continue;

			const bool pressed = !BIT(current[row], bit);
			const u8 code = u8((row << 3) | bit | (pressed ? 0x00 : 0x80));
			inject_v55_scanq_code(space, code, 0xff, !pressed);
		}
	}

	m_last_scanq_host = current;
}

attotime korgprophecy_state::h8_sci0_bit_period() const
{
	// Fallback for pre-reset / unsupported SCI modes: the boot rate the
	// firmware's SMR=00/BRR=0b config produces from the H8 clock.
	const u32 boot_rate = u32(double(m_subcpu->unscaled_clock()) / 384.0 + 0.5);
	if (machine().phase() < machine_phase::RESET)
		return attotime::from_hz(boot_rate);

	address_space &space = m_subcpu->space(AS_PROGRAM);
	const u8 smr = space.read_byte(0x0fffb0);
	const u8 brr = space.read_byte(0x0fffb1);
	const u8 scr = space.read_byte(0x0fffb2);

	// Prophecy boots SCI0 in internal async mode (`SMR=00`, `BRR=0b`, `SCR=70`).
	// Fall back to the observed boot bitrate if firmware temporarily switches to
	// an unsupported mode while the driver-side bridge catches up.
	if ((smr & 0x80) || (scr & 0x02))
		return attotime::from_hz(boot_rate);

	const u64 divider = (u64(2) << (2 * (smr & 0x03))) * (u64(brr) + 1);
	if (!divider)
		return attotime::from_hz(boot_rate);

	const double bps = double(m_subcpu->system_clock()) / double(divider * 16);
	if (bps <= 1.0)
		return attotime::from_hz(boot_rate);

	return attotime::from_hz(u32(bps + 0.5));
}

void korgprophecy_state::start_v55_h8_tx(u8 data)
{
	m_v55_h8_tx_active = true;
	// Channel 0 (`FFF73..FFF76`) is the traced V55<->H8 link. The driver still
	// has to drain the older RAM queue, but byte launch now goes through the
	// native V55 channel-0 SFR path so the core emits real `TXD0` edges.
	v55_sfr_write_byte(0x75, data);
	// The H8 firmware enables only IRQ1 and uses that external interrupt
	// to budget SCI0 receive parsing. The traced schematic omits the real
	// source, but coupling it to V55 byte launch matches observed behavior
	// much better than scribbling the budget counter directly.
	pulse_h8_irq1("TXSTART");
	m_v55_h8_tx_timer->adjust(h8_sci0_bit_period() * 10);
}

void korgprophecy_state::pulse_h8_irq1(const char *reason)
{
	m_subcpu->set_input_line(1, ASSERT_LINE);
	m_subcpu->set_input_line(1, CLEAR_LINE);
}

void korgprophecy_state::poll_h8_tx_ring()
{
	address_space &h8space = m_subcpu->space(AS_PROGRAM);
	const u32 wptr = h8space.read_dword(0x0483a0) & 0x000000ffU;

	// The real board link is H8 SCI0 TXD -> V55 UART0 RXD. This poller only
	// tracks the ring write pointer for observability; the direct byte
	// injection it once performed duplicated the real TXD-sampled path and
	// could overrun UART0, so it was removed.
	m_last_h8_tx_wptr = wptr;
}

TIMER_CALLBACK_MEMBER(korgprophecy_state::v55_h8_service_tick)
{
	address_space &space = m_maincpu->space(AS_PROGRAM);
	poll_h8_tx_ring();


	// KPSHIP-ACCURACY: bring-up scan-queue mirror; the real producer path still stalls.
	// The late UI consumes the scan queue, but the regular producer path still
	// stalls in bring-up. Mirror real host input into that ring once firmware
	// enables it so the machine stays interactive.
	if (space.read_byte(SCANQ_PHYS_ENABLE) & 0x01)
		poll_host_scanq_inputs(space);

	// Shipping uses the V55 UART0/INTST0 worker. Keep this service timer for
	// host scan-queue polling, but do not let the legacy RAM-queue TX bridge
	// race the native firmware transport.
	if (m_v55_h8_tx_active || NATIVE_V55_H8_TRANSPORT)
		return;

	const u16 count = space.read_word(MB_PHYS_A70E);
	if (count == 0x0000)
	{
		const u32 pc = u32(m_maincpu->pc());
		const bool pc_in_ack_wait =
			((pc >= 0x0b4008) && (pc <= 0x0b4018)) ||
			((pc >= 0x0b4077) && (pc <= 0x0b4083));
		const u16 ctrl_wptr = space.read_word(MB_PHYS_A710);
		const u16 ctrl_rptr = space.read_word(MB_PHYS_A712);
		const u16 pending_wptr = space.read_word(V55_PENDING_PARAM_WPTR) & 0x001f;
		const u16 pending_rptr = space.read_word(V55_PENDING_PARAM_RPTR) & 0x001f;
		const u16 pending_delta = (pending_wptr - pending_rptr) & 0x001f;
		const u8 pending_head_param = space.read_byte(V55_PENDING_PARAM_BASE + pending_rptr);
		const bool late_program_load_reply = (ctrl_wptr == 0x01b4) && (ctrl_rptr == 0x01b2);
		const bool pending_p155_at_head = (pending_head_param == 0x9b) && (pending_delta >= 0x10);
		// KPSHIP-ACCURACY: A716 V55<->H8 mailbox poke over hardcoded ROM PC ranges;
		// not proven hw. Real fix = model the native channel-0 RX/parser path.
		// FIXME REMOVE_HACK_BEFORE_REAL_EMULATION:
		// This is not proven hardware behavior. It bridges a missing piece of the
		// V55<->H8 transport model at the late program-load reply boundary where
		// the V55 can otherwise leave parameter 155 at the head of the pending
		// parameter ring until it is overwritten. Once the native channel-0
		// receive/parser path is modeled faithfully, delete this bridge and
		// require the firmware to update A716 itself.
		if (late_program_load_reply && pending_p155_at_head &&
			pc_in_ack_wait &&
			(space.read_byte(MB_PHYS_A716) == 0x00) &&
			(space.read_word(MB_PHYS_A70A) == space.read_word(MB_PHYS_A70C)))
		{
			space.write_byte(MB_PHYS_A716, 0x01);
		}
		m_txsm_latched_mbseg = 0xffff;
		if (v55_sfr_byte(0x74) & 0x20)
		{
			const u8 old73 = v55_sfr_byte(0x73);
			const u8 new73 = old73 & ~0x80;
			if (new73 != old73)
				v55_sfr_write_byte(0x73, new73);
		}
		return;
	}

	{
		u16 ctrl_rptr = space.read_word(MB_PHYS_A712);
		const u16 ctrl_wptr = space.read_word(MB_PHYS_A710);
		const u16 ds0 = u16(m_maincpu->state_int(NEC_DS0));
		const u32 ds0_base = (u32(ds0) << 4) & 0x000fffffU;
		const u16 live_mbseg = m_maincpu->debug_logical_read_word(ds0_base + 0x008c);
		if (m_txsm_latched_mbseg == 0xffff)
			m_txsm_latched_mbseg = live_mbseg;
		const u16 mbseg = m_txsm_latched_mbseg;
		const u32 dyn_data_base = ((u32(mbseg) << 4) + 0x0006) & 0x000fffffU;
		const u32 dyn_ctrl_base = ((u32(mbseg) << 4) + 0x0406) & 0x000fffffU;

		if (ctrl_rptr == ctrl_wptr)
			return;

		u16 remaining = space.read_word((dyn_ctrl_base + (ctrl_rptr & 0x03ff)) & 0x000fffffU);
		if (remaining == 0x0000)
		{
			if (space.read_byte(MB_PHYS_A716) != 0x00)
			{
				const u8 old_a717 = space.read_byte(MB_PHYS_A717);
				space.write_byte(MB_PHYS_A716, 0x00);
				space.write_byte(MB_PHYS_A717, old_a717 + 1);
				ctrl_rptr = (ctrl_rptr + 2) & 0x03ff;
				space.write_word(MB_PHYS_A712, ctrl_rptr);
				if (ctrl_rptr == ctrl_wptr)
					return;
				remaining = space.read_word((dyn_ctrl_base + (ctrl_rptr & 0x03ff)) & 0x000fffffU);
				if (remaining == 0x0000)
				{
					return;
				}
			}
			else
			{
				if (v55_sfr_byte(0x74) & 0x20)
				{
					const u8 old73 = v55_sfr_byte(0x73);
					const u8 new73 = old73 & ~0x80;
					if (new73 != old73)
						v55_sfr_write_byte(0x73, new73);
				}
				return;
			}
		}

		space.write_word((dyn_ctrl_base + (ctrl_rptr & 0x03ff)) & 0x000fffffU, remaining - 1);

		const u16 old_rptr = space.read_word(MB_PHYS_A70C);
		const u32 data_addr = (dyn_data_base + (old_rptr & 0x03ff)) & 0x000fffffU;
		const u8 data = space.read_byte(data_addr);
		u16 rptr = (old_rptr + 1) & 0x03ff;
		space.write_word(MB_PHYS_A70C, rptr);
		space.write_word(MB_PHYS_A70E, count - 1);
		start_v55_h8_tx(data);
		return;
	}
}

TIMER_CALLBACK_MEMBER(korgprophecy_state::v55_h8_tx_bit_tick)
{
	if (!m_v55_h8_tx_active)
		return;

	m_v55_h8_tx_active = false;
	m_v55_h8_tx_timer->adjust(attotime::never);
}

TIMER_CALLBACK_MEMBER(korgprophecy_state::h8_txd_sample_tick)
{
	if (!m_h8_txd_decode_active)
		return;

	if (m_h8_txd_decode_bit < 8)
	{
		if (m_h8_txd_state)
			m_h8_txd_decode_byte |= u8(1U << m_h8_txd_decode_bit);
		m_h8_txd_decode_bit++;
		m_h8_txd_sample_timer->adjust(h8_sci0_bit_period());
		return;
	}

	if (!m_h8_txd_state)
		logerror("KPROP_H8V55_RXERR,T=%.6f,STOP=0,B=%02X\n", machine().time().as_double(), m_h8_txd_decode_byte);

	m_h8_txd_decode_active = false;
	m_h8_txd_decode_byte = 0x00;
	m_h8_txd_decode_bit = 0x00;
	m_h8_txd_sample_timer->adjust(attotime::never);
}

TIMER_CALLBACK_MEMBER(korgprophecy_state::mailbox_lie_tick)
{
	// The disassembly maps DS:F5F2 offsets A053/A054/A056 onto physical
	// FFF73/FFF74/FFF76, while A721 wraps into low RAM at 0x00641. If the
	// firmware later parks A721 at 0xF0 waiting for a success result, feed back
	// the success code the ROM already tests for. This keeps the lie narrow:
	// observe the V55's own state machine rather than inventing a whole backend.
	address_space &space = m_maincpu->space(AS_PROGRAM);
	const double now = machine().time().as_double();

	if (m_evtq_note_inject && m_evtq_note >= 0)
	{
		const bool active = (now >= m_evtq_note_start) && (now < (m_evtq_note_start + m_evtq_note_len));
		if (active && !m_evtq_note_state)
		{
			const u8 note = u8(m_evtq_note & 0x7f);
			const u8 vel = u8(m_evtq_velocity & 0x7f);
			u8 seq = 0;
			if (m_evtq_channel >= 0)
			{
				inject_v55_evtq_record(space, u16(0x90 | (m_evtq_channel & 0x0f) | (note << 8)), vel, "NOTEON", seq);
			}
			else
			{
				for (u8 ch = 0; ch < 16; ch++)
					inject_v55_evtq_record(space, u16(0x90 | ch | (note << 8)), vel, "NOTEON", seq++);
			}
			m_evtq_note_state = 1;
		}
		else if (!active && m_evtq_note_state)
		{
			const u8 note = u8(m_evtq_note & 0x7f);
			u8 seq = 0;
			if (m_evtq_channel >= 0)
			{
				inject_v55_evtq_record(space, u16(0x80 | (m_evtq_channel & 0x0f) | (note << 8)), 0x0000, "NOTEOFF", seq);
			}
			else
			{
				for (u8 ch = 0; ch < 16; ch++)
					inject_v55_evtq_record(space, u16(0x80 | ch | (note << 8)), 0x0000, "NOTEOFF", seq++);
			}
			m_evtq_note_state = 0;
		}
	}

	if (!m_pulse_queue_inject && !m_pulse_scanq_inject)
		return;

	if (m_pulse_queue_inject || m_pulse_scanq_inject)
	{
		for (u8 i = 0; i < m_script_pulse_count; i++)
		{
			if (m_script_pulse_row[i] < 0 || m_script_pulse_row[i] > 5 || m_script_pulse_bit[i] < 0 || m_script_pulse_bit[i] > 7)
				continue;

			const double start = m_script_pulse_start[i];
			const double end = start + m_script_pulse_len[i];
			const bool active = (now >= start) && (now < end);
			const u16 code = u16((m_script_pulse_row[i] << 3) | m_script_pulse_bit[i]);
			if (active && !m_script_pulse_queue_state[i])
			{
				if (m_pulse_queue_inject)
					inject_v55_input_code(space, code, i, false);
				if (m_pulse_scanq_inject)
					inject_v55_scanq_code(space, u8(code & 0x7f), i, false);
				m_script_pulse_queue_state[i] = 1;
			}
			else if (!active && m_script_pulse_queue_state[i])
			{
				if (m_pulse_queue_inject)
					inject_v55_input_code(space, code | 0x0080, i, true);
				if (m_pulse_scanq_inject)
					inject_v55_scanq_code(space, u8((code | 0x0080) & 0x00ff), i, true);
				m_script_pulse_queue_state[i] = 0;
			}
		}
	}
}

TIMER_CALLBACK_MEMBER(korgprophecy_state::dsp_empty_ready_tick)
{
	const u8 dsp = u8(param);
	if (dsp >= 3)
		return;

	m_dsp_empty |= 1U << dsp;
}

TIMER_CALLBACK_MEMBER(korgprophecy_state::dsp_board_sync_tick)
{
	m_dsp_sync_ticks++;

	// Approximate the traced serial graph under the board's 512FS/FS-derived
	// sample clock. This bypasses MAME's sound-stream pull model so the DSPs can
	// advance even in headless bring-up runs where no audio sink is active.
	//
	// The printed original schematic indicates the IC24/IC25 glue is DAC-side
	// WS/LRCK framing, not the source of the DSPs' direct FS/SYNC inputs.
	if (m_dsp_feedback_mode == u8(dsp_board_feedback_mode::delay1))
	{
		m_dsp1->set_serial_input(0, m_dsp2_so1_prev[0]);
		m_dsp1->set_serial_input(1, m_dsp2_so1_prev[1]);
		m_dsp1->set_serial_input(2, m_dsp_sdn_input);
		m_dsp1->set_serial_input(3, m_dsp_sdn_input);
		m_dsp3->set_serial_input(2, m_dsp_optionin_input);
		m_dsp3->set_serial_input(3, m_dsp_optionin_input);
		// Read the existing latch without forcing a mid-tick stream update. The
		// forced update pulled DSP2's inputs and double-stepped DSP1, breaking its
		// delay-line recirculation. This intentionally costs one sample of feedback
		// delay, matching the other cyclic board latch below.
		m_dsp2_so1_prev[0] = dsp_route_so(*m_dsp2, 2);
		m_dsp2_so1_prev[1] = dsp_route_so(*m_dsp2, 3);
	}
	else
	{
		m_dsp1->set_serial_input(2, m_dsp_sdn_input);
		m_dsp1->set_serial_input(3, m_dsp_sdn_input);
		m_dsp3->set_serial_input(2, m_dsp_optionin_input);
		m_dsp3->set_serial_input(3, m_dsp_optionin_input);
	}
	// E3 topology probe: dsp3 -> dsp2 is a cycle in the stream graph, so
	// like the dsp2_so1 -> dsp1 feedback above it is realized as a
	// one-sample-delayed serial-input latch.
	if (m_dsp2_input_route_mode == u8(dsp_board_dsp2_input_mode::dsp3_so1_to_si0)
		|| m_dsp2_input_route_mode == u8(dsp_board_dsp2_input_mode::dsp3_so0_to_si0)
		|| m_dsp2_input_route_mode == u8(dsp_board_dsp2_input_mode::dsp3_so1_dsp1_so0))
	{
		const bool so0_first = (m_dsp2_input_route_mode == u8(dsp_board_dsp2_input_mode::dsp3_so0_to_si0));
		m_dsp2->set_serial_input(0, so0_first ? m_dsp3_so_prev[0] : m_dsp3_so_prev[2]);
		m_dsp2->set_serial_input(1, so0_first ? m_dsp3_so_prev[1] : m_dsp3_so_prev[3]);
		if (m_dsp2_input_route_mode == u8(dsp_board_dsp2_input_mode::dsp3_so1_dsp1_so0))
		{
			m_dsp2->set_serial_input(2, dsp_route_so(*m_dsp1, 0));
			m_dsp2->set_serial_input(3, dsp_route_so(*m_dsp1, 1));
		}
		else
		{
			m_dsp2->set_serial_input(2, so0_first ? m_dsp3_so_prev[2] : m_dsp3_so_prev[0]);
			m_dsp2->set_serial_input(3, so0_first ? m_dsp3_so_prev[3] : m_dsp3_so_prev[1]);
		}
		// No forced stream update here: a mid-tick update() on dsp3
		// double-steps its stream against the U2 DAC's pull and freezes
		// the WAV at one sample. Reading the latched outputs costs one
		// sample of delay, same as the dsp2_so1 feedback idiom.
		for (int lane = 0; lane < 4; lane++)
			m_dsp3_so_prev[lane] = dsp_route_so(*m_dsp3, lane);
	}
}

void korgprophecy_state::lcd_trace_dump()
{
	// Retired dev-harness LCD state dump (KPROP_LCD) + DSP3 waveform-test seed.
}



void korgprophecy_state::lcd_trace_control(u8 value)
{
	if (value == 0x01)
	{
		m_lcd_ddram.fill(' ');
		m_lcd_addr = 0x00;
		m_lcd_cgram_mode = false;
		m_lcd_disp_shift = 0;
		lcd_trace_dump();
		return;
	}

	if (value == 0x02)
	{
		m_lcd_addr = 0x00;
		m_lcd_cgram_mode = false;
		m_lcd_disp_shift = 0;
		lcd_trace_dump();
		return;
	}

	if ((value & 0xfc) == 0x04)
	{
		m_lcd_entry_inc = BIT(value, 1);
		m_lcd_entry_shift = BIT(value, 0);
		return;
	}

	if ((value & 0xf0) == 0x10)
	{
		const int direction = BIT(value, 2) ? 1 : -1;
		if (BIT(value, 3))
		{
			m_lcd_disp_shift += direction;
			if (m_lcd_disp_shift == 0x50)
				m_lcd_disp_shift = 0;
			else if (m_lcd_disp_shift == -1)
				m_lcd_disp_shift = 0x4f;
			lcd_trace_dump();
		}
		return;
	}

	if ((value & 0xc0) == 0x40)
	{
		m_lcd_addr = value & 0x3f;
		m_lcd_cgram_mode = true;
		return;
	}

	if (value & 0x80)
	{
		m_lcd_addr = value & 0x7f;
		m_lcd_cgram_mode = false;
		lcd_trace_dump();
		return;
	}
}

void korgprophecy_state::lcd_trace_data(u8 value)
{
	if (m_lcd_cgram_mode)
	{
		m_lcd_cgram[m_lcd_addr & 0x3f] = value & 0x1f;
		m_lcd_addr = (m_lcd_entry_inc ? (m_lcd_addr + 1) : (m_lcd_addr - 1)) & 0x3f;
		if ((m_lcd_addr & 0x07) == 0)
			lcd_trace_dump();
		return;
	}

	m_lcd_ddram[m_lcd_addr & 0x7f] = value;
	const int direction = m_lcd_entry_inc ? 1 : -1;
	m_lcd_addr = (m_lcd_addr + direction) & 0x7f;
	if (m_lcd_entry_shift)
	{
		m_lcd_disp_shift += direction;
		if (m_lcd_disp_shift == 0x50)
			m_lcd_disp_shift = 0;
		else if (m_lcd_disp_shift == -1)
			m_lcd_disp_shift = 0x4f;
	}

	// Limit logs to completed visible lines for readability.
	const u8 vis_addr = m_lcd_addr & 0x7f;
	if (vis_addr == 0x28 || vis_addr == 0x68)
		lcd_trace_dump();
}

u8 korgprophecy_state::v55_shadow_byte(u8 reg) const
{
	const u16 word = m_mmio_shadow[reg >> 1];
	return BIT(reg, 0) ? u8(word >> 8) : u8(word & 0xff);
}

void korgprophecy_state::set_v55_shadow_byte(u8 reg, u8 data)
{
	u16 &word = m_mmio_shadow[reg >> 1];
	if (BIT(reg, 0))
		word = (word & 0x00ff) | (u16(data) << 8);
	else
		word = (word & 0xff00) | data;
}

u8 korgprophecy_state::v55_sfr_byte(u8 reg) const
{
	if (machine().phase() >= machine_phase::RESET)
		return m_maincpu->space(AS_DATA).read_byte(0x100 + reg);
	return v55_shadow_byte(reg);
}

void korgprophecy_state::v55_sfr_write_byte(u8 reg, u8 data)
{
	set_v55_shadow_byte(reg, data);
	if (machine().phase() >= machine_phase::RESET)
		m_maincpu->space(AS_DATA).write_byte(0x100 + reg, data);
}

u8 korgprophecy_state::v55_p0_r()
{
	const u8 value = read_v55_p0_status();
	set_v55_shadow_byte(0x00, value);
	return value;
}

void korgprophecy_state::v55_p0_w(u8 data)
{
	set_v55_shadow_byte(0x00, data);
	m_input_ctrl = data;
}

u8 korgprophecy_state::v55_p1_r()
{
	const u8 value = read_special_inputs();
	set_v55_shadow_byte(0x01, value);
	return value;
}

u8 korgprophecy_state::v55_p2_r()
{
	const u8 value = v55_shadow_byte(0x02);
	return value;
}

void korgprophecy_state::v55_p2_w(u8 data)
{
	const u8 old = v55_shadow_byte(0x02);
	const bool first_p2_write = !m_m66311_seen_p2;
	set_v55_shadow_byte(0x02, data);
	m_scan_pattern = data;


	// KLM-1849 LED13 SPEED is not on the M66311 serial LED driver.  The
	// service-manual and KiCad traces show V55 IC4 pin 35 / P24 driving the
	// TEMPO net, which biases DT1 and sinks the SPEED LED.
	static constexpr u8 ARPEGGIATOR_SPEED_LED = 80;
	set_led_output(ARPEGGIATOR_SPEED_LED, BIT(data, 4));
	if (first_p2_write || (BIT(old, 4) != BIT(data, 4)))
	{
		[[maybe_unused]] const u8 prdc = v55_sfr_byte(0x0c);
		const u8 pm2 = v55_sfr_byte(0x12);
		const u8 pmc2 = v55_sfr_byte(0x22);
		[[maybe_unused]] const bool p24_port_output = !BIT(pmc2, 4) && !BIT(pm2, 4);
		log_led_output_bank_if_changed(ARPEGGIATOR_SPEED_LED >> 3);
	}

	update_panel_led_serial(old, data);
}

void korgprophecy_state::v55_p3_w(u8 data)
{
	set_v55_shadow_byte(0x03, data);
	update_led_bank(data);
}

u8 korgprophecy_state::v55_p4_r()
{
	const u8 value = read_scanned_inputs();
	set_v55_shadow_byte(0x04, value);
	return value;
}

u8 korgprophecy_state::v55_p8_r()
{
	const u8 value = v55_shadow_byte(0x08);
	return value;
}

void korgprophecy_state::v55_p8_w(u8 data)
{
	set_v55_shadow_byte(0x08, data);
}

void korgprophecy_state::v55_adc_fint_w(int state)
{
	if (!state || !m_adc_mux_phase_on_fint)
		return;

	const u16 ds0 = u16(m_maincpu->state_int(NEC_DS0));
	const u32 ds0_base = (u32(ds0) << 4) & 0x000fffffU;
	const u32 phase_addr = ds0_base + 0x0a8e0;
	const u32 flags_addr = ds0_base + 0x0a8e1;
	const u8 flags = m_maincpu->debug_logical_read_byte(flags_addr);

	if (BIT(flags, 0))
	{
		return;
	}

	const u8 old_phase = m_maincpu->debug_logical_read_byte(phase_addr) & 0x03;
	const u8 new_phase = (old_phase + 1) & 0x03;
	const u8 old_p8 = v55_shadow_byte(0x08);
	const u8 new_p8 = (old_p8 & 0xfc) | new_phase;

	v55_sfr_write_byte(0x08, new_p8);
	m_maincpu->debug_logical_write_byte(phase_addr, new_phase);
	m_maincpu->debug_logical_write_byte(flags_addr, flags | 0x02);

}

u8 korgprophecy_state::v55_p7_r()
{
	u8 value = v55_shadow_byte(0x07);
	const u8 ctrl = v55_shadow_byte(0x05);

	// The LCD bus lives on P5/P7:
	//   P5 bit 0 = R/W
	//   P5 bit 1 = RS
	//   P5 bit 2 = E
	// The firmware reads P7 while E is asserted; model that directly.
	if (BIT(ctrl, 0) && BIT(ctrl, 2))
	{
		const bool data_mode = BIT(ctrl, 1);
		value = data_mode ? m_lcdc->data_r() : m_lcdc->control_r();

		// The current HD44780 timing model is still too strict for the Prophecy
		// bring-up path. Preserve address/status bits but clear busy on status reads.
		if (!data_mode)
			value &= 0x7f;
	}

	set_v55_shadow_byte(0x07, value);
	return value;
}

void korgprophecy_state::v55_p5_w(u8 data)
{
	const u8 old = v55_shadow_byte(0x05);
	set_v55_shadow_byte(0x05, data);

	// Latch LCD writes on the falling edge of E while R/W=0.
	if (BIT(old, 2) && !BIT(data, 2) && !BIT(old, 0))
	{
		const u8 value = v55_shadow_byte(0x07);
		m_lcd_data_mode = BIT(old, 1);
		if (m_lcd_data_mode)
		{
			m_lcdc->data_w(value);
			lcd_trace_data(value);
		}
		else
		{
			m_lcdc->control_w(value);
			lcd_trace_control(value);
		}

		m_lcd_data_mode = false;
	}
}

void korgprophecy_state::v55_p7_w(u8 data)
{
	set_v55_shadow_byte(0x07, data);
}

template <unsigned Channel>
u8 korgprophecy_state::v55_adc_r()
{
	static_assert(Channel < 4);
	u8 value = m_adc_level[Channel] & 0xff;
	int src = int(Channel);
	u8 sel = m_adc_sel & 0x03;
	if (!m_adc_level_override[Channel] && m_adc_mux_enable)
	{
		// IC1 and IC11 are 74HC4052 dual 4:1 muxes feeding V55 AIN0..AIN3.
		// The traced netlist gives the input groups:
		//   AIN0 <- ADIN0..3
		//   AIN1 <- ADIN4..7
		//   AIN2 <- ADIN8..10,(11 missing in the traced export)
		//   AIN3 <- ADIN12..14,(15 missing in the traced export)
		// Under the firmware's DS value, the disassembly alias 9FE8 is physical
		// FFF08: the V55 P8 latch.  Its low two bits are the 4052 select phase.
		const u8 phase_shadow = v55_shadow_byte(0x08) & 0x03;
		if (!m_adc_sel_locked && m_adc_mux_from_p2)
		{
			const u8 p2 = v55_shadow_byte(0x02);
			sel = (BIT(p2, 2) << 1) | BIT(p2, 0);
		}
		else if (!m_adc_sel_locked && m_adc_mux_from_phase_shadow)
			sel = phase_shadow;
		if (!m_adc_sel_locked && m_adc_mux_autoscan)
			sel = m_adc_autoscan_sel & 0x03;
		switch (Channel)
		{
		case 0:
			src = 0 + sel;
			value = m_adin_level[src];
			break;
		case 1:
			src = 4 + sel;
			value = m_adin_level[src];
			break;
		case 2:
			if (sel < 3)
			{
				src = 8 + sel;
				value = m_adin_level[src];
			}
			else
			{
				src = 11;
				value = m_adin_level[src];
			}
			break;
		case 3:
			if (sel < 3)
			{
				src = 12 + sel;
				value = m_adin_level[src];
			}
			else
			{
				src = 15;
				value = m_adin_level[src];
			}
			break;
		}
	}
	if (m_adc_mux_autoscan && Channel == 3)
		m_adc_autoscan_sel = (m_adc_autoscan_sel + 1) & 0x03;

	return value;
}

template <unsigned Channel>
u16 korgprophecy_state::h8_adc_r()
{
	static_assert(Channel < 8);
	return m_h8_adc_level[Channel] & 0x03ff;
}

u8 korgprophecy_state::active_scan_row() const
{
	// The V55 writes a 5-entry table (05,0d,06,0e,07) to FFF00 low while scanning
	// the primary front-panel matrix through FFF04 low.
	switch (m_input_ctrl & 0x0f)
	{
	case 0x5: return 0;
	case 0xd: return 1;
	case 0x6: return 2;
	case 0xe: return 3;
	case 0x7: return 4;
	}

	// KPSHIP-ACCURACY: front-panel scan-mask model unconfirmed (control mapping, not audio).
	// Working hypothesis: low-byte writes at FFF02 drive an active-low scan mask.
	for (u8 row = 0; row < 8; row++)
	{
		if (!BIT(m_scan_pattern, row))
			return row;
	}

	return m_scan_sel & 0x07;
}

void korgprophecy_state::apply_auto_stimulus(u8 row, u8 &panel)
{
	// KPSHIP-ACCURACY: unattended-scan panel-bit stimulus is a bring-up guess (control mapping).
	// Unattended scan: once boot settles, pulse one guessed panel bit repeatedly.
	const double t = machine().time().as_double();
	for (const host_panel_pulse &pulse : m_host_panel_pulses)
	{
		if (pulse.row == row && pulse.bit >= 0 && t < pulse.end)
			panel &= ~(1U << u8(pulse.bit));
	}
	if (!m_pulse_virtual_only)
	{
		for (u8 i = 0; i < m_script_pulse_count; i++)
		{
			if (m_script_pulse_row[i] < 0 || m_script_pulse_bit[i] < 0)
				continue;
			if (row == u8(m_script_pulse_row[i]) && t >= m_script_pulse_start[i] && t < (m_script_pulse_start[i] + m_script_pulse_len[i]))
				panel &= ~(1U << u8(m_script_pulse_bit[i]));
		}
	}

	if ((m_cfg.read_safe(0x00) & 0x02) && t >= 3.0)
	{
		const s16 step = s16((t - 3.0) / 0.30) % 40;
		const u8 target_row = step / 8;
		const u8 target_bit = step & 7;

		if (step != m_last_stim_step)
		{
			m_last_stim_step = step;
		}

		if (row == target_row)
			panel &= ~(1U << target_bit);
	}
}

bool korgprophecy_state::host_panel_stimulus_active(u8 row) const
{
	const double now = machine().time().as_double();
	for (const host_panel_pulse &pulse : m_host_panel_pulses)
	{
		if (pulse.row == row && pulse.bit >= 0 && now < pulse.end)
			return true;
	}
	return false;
}

void korgprophecy_state::apply_auto_kbd_stimulus(u8 row, u8 &kbd)
{
	const double t = machine().time().as_double();

	if (m_kbd_pulse_row >= 0 && m_kbd_pulse_bit >= 0)
	{
		if (row == u8(m_kbd_pulse_row) && t >= m_kbd_pulse_start && t < (m_kbd_pulse_start + m_kbd_pulse_len))
			kbd &= ~(1U << u8(m_kbd_pulse_bit));
	}

	for (u8 i = 0; i < m_kbd_script_pulse_count; i++)
	{
		if (m_kbd_script_pulse_row[i] < 0 || m_kbd_script_pulse_bit[i] < 0)
			continue;
		if (row == u8(m_kbd_script_pulse_row[i]) && t >= m_kbd_script_pulse_start[i] && t < (m_kbd_script_pulse_start[i] + m_kbd_script_pulse_len[i]))
			kbd &= ~(1U << u8(m_kbd_script_pulse_bit[i]));
	}
}

bool korgprophecy_state::auto_kbd_stimulus_active(u8 row) const
{
	const double t = machine().time().as_double();

	if (m_kbd_pulse_row >= 0 && m_kbd_pulse_bit >= 0 &&
		row == u8(m_kbd_pulse_row) &&
		t >= m_kbd_pulse_start &&
		t < (m_kbd_pulse_start + m_kbd_pulse_len))
		return true;

	for (u8 i = 0; i < m_kbd_script_pulse_count; i++)
	{
		if (m_kbd_script_pulse_row[i] < 0 || m_kbd_script_pulse_bit[i] < 0)
			continue;
		if (row == u8(m_kbd_script_pulse_row[i]) &&
			t >= m_kbd_script_pulse_start[i] &&
			t < (m_kbd_script_pulse_start[i] + m_kbd_script_pulse_len[i]))
			return true;
	}

	return false;
}

u8 korgprophecy_state::read_matrix_row(u8 row)
{
	if (machine().phase() < machine_phase::RUNNING)
		return 0xff;

	u8 panel = m_panel[row].read_safe(0xff);
	u8 kbd = m_kbd[row].read_safe(0xff);

	// Optional unattended assist: hold test-mode startup combos for early boot.
	// Service manual combos (wording normalized):
	//   Wheel 3 Hold + Pattern Define : ordinary test mode
	//   Wheel 3 Hold + Octave         : test mode excluding MIDI+Card test
	//   Wheel 3 Hold + Latch          : test mode excluding MIDI test
	//   Wheel 3 Hold + Portamento     : test mode excluding Card test
	//
	// Important uncertainty:
	// - On the current tree, ordinary test mode does *not* come from the old
	//   row-0 Pattern Define guess.
	// - A fresh isolated boot now proves the working partner is row 2 bit 7
	//   alongside Wheel 3 Hold. That likely means our current semantic label
	//   for row-2 bit-7 is wrong, even though the electrical row/bit is right.
	if (((m_cfg.read_safe(0x00) & 0x01) || m_boot_combo_requested) &&
		machine().time() < attotime::from_seconds(m_boot_hold_secs))
	{
		if (m_boot_testmode_sel >= 0 && m_boot_testmode_sel <= 8)
		{
			// ROM helper B1AD8/B1B03 resolves logical switch IDs through CS:22BB,
			// and B8617 uses these exact logical pairs to seed the top-level
			// test-mode selector AA90 during boot. This table is the direct
			// row/bit reduction of those ROM entries:
			//   0: 1E+18  1: 1E+1A  2: 1E+1B  3: 1E+1F  4: 1E+1C
			//   5: 20+22  6: 16+17  7: 1E+1D  8: 1E+19
			static constexpr struct
			{
				u8 row_a;
				u8 bit_a;
				u8 row_b;
				u8 bit_b;
			} bootsel_map[9] =
			{
				{ 3, 7, 3, 1 }, // sel 0
				{ 3, 7, 3, 2 }, // sel 1
				{ 3, 7, 3, 3 }, // sel 2
				{ 3, 7, 3, 6 }, // sel 3
				{ 3, 7, 3, 5 }, // sel 4
				{ 1, 0, 1, 2 }, // sel 5
				{ 0, 0, 2, 7 }, // sel 6
				{ 3, 7, 3, 4 }, // sel 7
				{ 3, 7, 3, 0 }, // sel 8
			};

			const auto &sel = bootsel_map[m_boot_testmode_sel];
			if (row == sel.row_a)
				panel &= ~(1U << sel.bit_a);
			if (row == sel.row_b)
				panel &= ~(1U << sel.bit_b);
		}
		else
		{
			if (row == 0)
				panel &= ~0x01; // Wheel 3 Hold

			switch (m_boot_mode & 0x03)
			{
			case 0x00:
				if (row == 2)
					panel &= ~0x80; // Empirically proven ordinary test-mode partner
				break;
			// KPSHIP-ACCURACY: Octave/Latch/Portamento panel-bit assignments still provisional.
			case 0x01:
				if (row == 0)
					panel &= ~0x10; // Octave (still provisional)
				break;
			case 0x02:
				if (row == 0)
					panel &= ~0x08; // Latch/Key Sync (still provisional)
				break;
			case 0x03:
				if (row == 0)
					panel &= ~0x04; // Portamento (still provisional)
				break;
			}
		}
	}

	apply_auto_stimulus(row, panel);
	apply_auto_kbd_stimulus(row, kbd);

	const u8 merged = panel & kbd;
	m_last_input_by_row[row] = merged;

	return merged;
}

u8 korgprophecy_state::read_scanned_inputs()
{
	return read_matrix_row(active_scan_row());
}

u8 korgprophecy_state::read_special_inputs()
{
	// The V55 samples a sixth input group across two ports:
	//   P1 bits 2-6 <= row-5 bits 1-5
	//   P0 bit 7    <= row-5 bit 0
	// See the scan worker at B1A5B/B1A9D. Bits 6-7 of the raw row are still
	// unresolved and stay high for now.
	const u8 special = read_matrix_row(5);
	return 0x83 | ((special & 0x3e) << 1);
}

u8 korgprophecy_state::read_v55_p0_status()
{
	// FFF00 low is not purely the scan/control latch. The later BBE13 helper
	// reads bits 4-6 as raw V55 P04/P05/P06 status pins, which the schematic
	// ties directly to WPT, CDDT and ALM from the card controller.
	//
	// Current modeled polarity:
	//   bit 4 = WPT  (1 when write-protected)
	//   bit 5 = CDDT (1 when card present)
	//   bit 6 = ALM  (0 when battery alarm active; card battery is healthy)
	u8 value = m_input_ctrl & 0x0f;
	if (m_card_write_protect)
		value |= 0x10;
	if (m_card_present)
		value |= 0x20;
	// ALM is inactive: the card's backup battery reads healthy.
	value |= 0x40;
	if (machine().phase() >= machine_phase::RUNNING)
	{
		const u8 special = read_matrix_row(5);
		if (BIT(special, 0))
			value |= 0x80;
	}
	else
	{
		value |= 0x80;
	}
	return value;
}

void korgprophecy_state::update_led_bank(u8 value)
{
	// Legacy/test-mode LED observation path.  Normal panel LEDs on KLM-1847 and
	// KLM-1849 are serial M66311FP drivers decoded in update_panel_led_serial().
	const u8 bank = active_scan_row() & 0x07;
	for (u8 bit = 0; bit < 8; bit++)
		set_led_output((bank * 8) + bit, BIT(value, bit));

	log_led_output_bank_if_changed(bank);
}

void korgprophecy_state::set_led_output(u8 index, bool state)
{
	if (index >= 96)
		return;

	const u8 bank = index >> 3;
	const u8 mask = 1U << (index & 0x07);
	if (state)
		m_led_output_bank[bank] |= mask;
	else
		m_led_output_bank[bank] &= ~mask;
	m_leds[index] = state ? 1 : 0;
}

void korgprophecy_state::log_led_output_bank(u8 bank)
{
	if (bank >= m_led_output_bank.size())
		return;
	if (g_kprop_host_led != nullptr)
		g_kprop_host_led(bank, m_led_output_bank[bank]);
}

void korgprophecy_state::log_led_output_bank_if_changed(u8 bank)
{
	if (bank >= m_led_output_bank.size())
		return;

	if (m_led_output_logged_valid[bank] && m_led_output_logged_bank[bank] == m_led_output_bank[bank])
		return;

	m_led_output_logged_bank[bank] = m_led_output_bank[bank];
	m_led_output_logged_valid[bank] = true;
	log_led_output_bank(bank);
}

void korgprophecy_state::update_panel_led_driver(u8 driver, u16 value)
{
	static constexpr u8 NC = 0xff;
	static constexpr u8 RIGHT_PANEL_BASE = 48;
	static constexpr u8 ARPEGGIATOR_BASE = 64;
	// raw value bit -> GUI LED output, derived EMPIRICALLY from the firmware's
	// own `LED&SW TEST` (each prompt lights its switch's LED) cross-checked
	// against normal-mode edit-section LEDs.
	// The earlier "raw bit b == output Q(b)" assumption was wrong: the two
	// M66311 chips are clocked/latched differently, so each driver needs its
	// own measured table (no single serial-order fix works for both).
	//   ARP driver uses raw bits 2..13 (LED(ALL)=0x3FFF); bits 0,1,14,15 idle.
	static constexpr std::array<u8, 16> ARP_LED_FROM_Q = {{
		NC,                    // bit0  unused (always 0 in LED(ALL)=0x3FFF... bit set but no LED)
		NC,                    // bit1  unused
		ARPEGGIATOR_BASE + 0,  // bit2  -> led64 LATCH
		ARPEGGIATOR_BASE + 1,  // bit3  -> led65 KEY SYNC
		ARPEGGIATOR_BASE + 2,  // bit4  -> led66 LATCH&K.S.
		ARPEGGIATOR_BASE + 3,  // bit5  -> led67 OCTAVE 2
		ARPEGGIATOR_BASE + 4,  // bit6  -> led68 OCTAVE 3
		ARPEGGIATOR_BASE + 5,  // bit7  -> led69 OCTAVE 4
		ARPEGGIATOR_BASE + 10, // bit8  -> led74 PATTERN
		ARPEGGIATOR_BASE + 11, // bit9  -> led75 ARPEGGIO
		ARPEGGIATOR_BASE + 12, // bit10 -> led76 WHEEL3 HOLD
		ARPEGGIATOR_BASE + 13, // bit11 -> led77 PORTAMENTO
		ARPEGGIATOR_BASE + 14, // bit12 -> led78 OCTAVE DOWN
		ARPEGGIATOR_BASE + 15, // bit13 -> led79 OCTAVE UP
		NC,                    // bit14 unused
		NC,                    // bit15 unused
	}};
	static constexpr std::array<u8, 16> RIGHT_LED_FROM_Q = {{
		RIGHT_PANEL_BASE + 5,  // bit0  -> led53 lower selector arrow
		RIGHT_PANEL_BASE + 3,  // bit1  -> led51 COMPARE
		RIGHT_PANEL_BASE + 4,  // bit2  -> led52 lower selector arrow
		RIGHT_PANEL_BASE + 15, // bit3  -> led63 GLOBAL
		RIGHT_PANEL_BASE + 14, // bit4  -> led62 COMMON
		RIGHT_PANEL_BASE + 13, // bit5  -> led61 LFO
		RIGHT_PANEL_BASE + 12, // bit6  -> led60 EG
		RIGHT_PANEL_BASE + 11, // bit7  -> led59 EFFECT
		RIGHT_PANEL_BASE + 10, // bit8  -> led58 AMP
		RIGHT_PANEL_BASE + 9,  // bit9  -> led57 FILTER
		RIGHT_PANEL_BASE + 8,  // bit10 -> led56 MIXER
		RIGHT_PANEL_BASE + 7,  // bit11 -> led55 W.SHAPE
		RIGHT_PANEL_BASE + 6,  // bit12 -> led54 OSC
		RIGHT_PANEL_BASE + 1,  // bit13 -> led49 upper selector arrow
		RIGHT_PANEL_BASE + 0,  // bit14 -> led48 upper selector arrow
		RIGHT_PANEL_BASE + 2,  // bit15 -> led50 WRITE
	}};

	if (driver >= 2)
		return;

	const u8 base = (driver == 0) ? ARPEGGIATOR_BASE : RIGHT_PANEL_BASE;
	const u8 first_bank = base >> 3;
	const u8 second_bank = first_bank + 1;
	const std::array<u8, 16> &led_from_q = (driver == 0) ? ARP_LED_FROM_Q : RIGHT_LED_FROM_Q;
	for (u8 bit = 0; bit < 16; bit++)
	{
		const u8 led = led_from_q[bit];
		if (led != NC)
			set_led_output(led, BIT(value, bit));
	}

	log_led_output_bank_if_changed(first_bank);
	log_led_output_bank_if_changed(second_bank);
}

void korgprophecy_state::update_panel_led_serial(u8 old_data, u8 data)
{
	if (!m_m66311_seen_p2)
	{
		m_m66311_seen_p2 = true;
		return;
	}

	// Original panel scans show both front-panel LED sections use Mitsubishi
	// M66311FP 16-output serial LED drivers:
	//   KLM-1849 arpeggiator: SD1/CKI/CKS/OE1
	//   KLM-1847 edit panel:  SD2/CKI/CKS/OE2
	//
	// The interconnect crop routes those lines through the V55 P20..P23 bundle.
	// Firmware traces match this active-low sequence:
	//   P20 = serial data
	//   P21 = CKI shift clock
	//   P22 = KLM-1849 latch/strobe
	//   P23 = KLM-1847 latch/strobe
	//
	// P24 is the separate KLM-1849 TEMPO net for LED13/SPEED, handled in
	// v55_p2_w(); it is not part of the M66311 serial driver.
	const bool clock_rise = !BIT(old_data, 1) && BIT(data, 1);
	if (clock_rise)
	{
		const u16 serial_bit = BIT(data, 0) ? 1 : 0;
		for (u8 driver = 0; driver < 2; driver++)
		{
			m_m66311_shift[driver] = ((m_m66311_shift[driver] << 1) | serial_bit) & 0xffff;
			m_m66311_bits[driver] = std::min<u8>(16, m_m66311_bits[driver] + 1);
		}
	}

	const bool arp_latch = !BIT(old_data, 2) && BIT(data, 2);
	const bool right_latch = !BIT(old_data, 3) && BIT(data, 3);
	if (arp_latch)
	{
		m_m66311_latch[0] = m_m66311_shift[0];
		update_panel_led_driver(0, m_m66311_latch[0]);
		m_m66311_bits[0] = 0;
	}
	if (right_latch)
	{
		m_m66311_latch[1] = m_m66311_shift[1];
		update_panel_led_driver(1, m_m66311_latch[1]);
		m_m66311_bits[1] = 0;
	}
}

u16 korgprophecy_state::v55_scratch_r(offs_t offset, u16 mem_mask)
{
	return m_v55_scratch[offset & 0x7f];
}

void korgprophecy_state::v55_scratch_w(offs_t offset, u16 data, u16 mem_mask)
{
	COMBINE_DATA(&m_v55_scratch[offset & 0x7f]);
}

u16 korgprophecy_state::sysram_r(offs_t offset, u16 mem_mask)
{
	const u16 data = m_sysram[offset];
	return data;
}

void korgprophecy_state::sysram_w(offs_t offset, u16 data, u16 mem_mask)
{
	COMBINE_DATA(&m_sysram[offset]);
}

u8 korgprophecy_state::h8_porta_r()
{
	// PA0/2/3/4 drive the shared DSP control bus; returning the current latch
	// avoids floating reads while the board-level decode remains incomplete.
	return m_h8_porta;
}

u8 korgprophecy_state::h8_portb_r()
{
	// PB0-2 are the confirmed direct DSP EMPTY feedback lines. The firmware
	// polls the PB bit matching the host window it is pacing, so when
	// KPROP_DSP_HOST_MAP remaps host windows to DSP devices the EMPTY bits
	// must be presented through the same permutation (identity map: no-op).
	u8 empty_bits = 0;
	for (int slot = 0; slot < 3; slot++)
		if (BIT(m_dsp_empty, m_dsp_host_map[slot]))
			empty_bits |= u8(1U << slot);
	const u8 value = 0xf8 | empty_bits;
	return value;
}

u8 korgprophecy_state::h8_portc_r()
{
	// PC4 gates the 74HC139 DSP chip-select decode and PC7 drives all DSP MUTE pins.
	return m_h8_portc;
}

void korgprophecy_state::h8_porta_w(u8 data)
{
	if (data != m_h8_porta)
	{
		m_h8_porta = data;
		update_dsp_host_ctrl();
	}
}

void korgprophecy_state::update_dsp_mute()
{
	const int mute_high = BIT(m_h8_portc, 7);
	m_dsp1->mute_w(mute_high);
	m_dsp2->mute_w(mute_high);
	m_dsp3->mute_w(mute_high);
}

void korgprophecy_state::h8_portc_w(u8 data)
{
	if (data != m_h8_portc)
	{
		m_h8_portc = data;
		update_dsp_mute();
	}
}

void korgprophecy_state::h8_port8_w(u8 data)
{
	if (data != m_h8_port8)
	{
		m_h8_port8 = data;
		update_h8_cts_from_port8();
	}
}

void korgprophecy_state::v55_txd_w(int state)
{
	const int old_state = m_v55_txd_state;
	if (state != m_v55_txd_state)
	{
		m_v55_txd_state = state;
		m_v55_txd_edges++;
	}

	// The H8 firmware budgets SCI0 receive parsing from IRQ1. With native V55
	// UART0 transmission, pulse it once per start bit instead of from the
	// legacy driver-side queue bridge.
	if (NATIVE_V55_H8_TRANSPORT && old_state == 1 && state == 0 &&
		machine().time() >= m_v55_txd_byte_end)
	{
		m_v55_txd_byte_end = machine().time() + h8_sci0_bit_period() * 10;
		pulse_h8_irq1("TXD0_START");
	}

	m_subcpu->sci_rx_w<0>(state);
}

void korgprophecy_state::v55_txd1_w(int state)
{
	const int old_state = m_v55_txd1_state;
	if (state != m_v55_txd1_state)
	{
		m_v55_txd1_state = state;
		m_v55_txd1_edges++;
	}

	if (!m_v55_txd1_decode_active && old_state == 1 && state == 0)
	{
		m_v55_txd1_decode_active = true;
		m_v55_txd1_decode_byte = 0x00;
		m_v55_txd1_decode_bit = 0x00;
		m_v55_txd1_sample_timer->adjust(attotime::from_hz(31'250) + (attotime::from_hz(31'250) / 2));
	}
}

TIMER_CALLBACK_MEMBER(korgprophecy_state::v55_txd1_sample_tick)
{
	if (!m_v55_txd1_decode_active)
		return;

	if (m_v55_txd1_decode_bit < 8)
	{
		if (m_v55_txd1_state)
			m_v55_txd1_decode_byte |= u8(1U << m_v55_txd1_decode_bit);
		m_v55_txd1_decode_bit++;
		m_v55_txd1_sample_timer->adjust(attotime::from_hz(31'250));
		return;
	}

	if (m_v55_txd1_state)
	{
		if (g_kprop_host_midi_tx_byte != nullptr)
			g_kprop_host_midi_tx_byte(m_v55_txd1_decode_byte, machine().time().as_double());
		handle_midi_tx_byte(m_v55_txd1_decode_byte);
	}

	m_v55_txd1_decode_active = false;
	m_v55_txd1_decode_byte = 0x00;
	m_v55_txd1_decode_bit = 0x00;
	m_v55_txd1_sample_timer->adjust(attotime::never);
}

void korgprophecy_state::handle_midi_tx_byte(u8 data)
{
	if (data == 0xf0)
	{
		m_midi_tx_sysex.clear();
		m_midi_tx_sysex.push_back(data);
		return;
	}

	if (m_midi_tx_sysex.empty())
		return;

	m_midi_tx_sysex.push_back(data);
	if (m_midi_tx_sysex.size() > (1024U * 1024U))
	{
		m_midi_tx_sysex.clear();
		return;
	}

	if (data == 0xf7)
	{
		if (g_kprop_host_midi_tx != nullptr)
			g_kprop_host_midi_tx(m_midi_tx_sysex.data(), m_midi_tx_sysex.size());
		m_midi_tx_sysex.clear();
	}
}

void korgprophecy_state::drain_host_midi()
{
	if (g_kprop_host_midi_pop == nullptr)
		return;

	uint8_t bytes[256];
	size_t count = 0;
	while (g_kprop_host_midi_pop(bytes, std::size(bytes), &count, machine().time().as_double()) && count > 0)
		enqueue_gui_midi_bytes(std::vector<u8>(bytes, bytes + count), "host_midi");
}

void korgprophecy_state::drain_host_panel()
{
	if (g_kprop_host_panel_pop == nullptr)
		return;

	uint8_t row = 0;
	uint8_t bit = 0;
	uint16_t len_ms = 0;
	const double now = machine().time().as_double();
	while (g_kprop_host_panel_pop(&row, &bit, &len_ms, now))
	{
		if (row > 7 || bit > 7)
			continue;

		double length = len_ms / 1000.0;
		if (length <= 0.0)
			length = 0.075;
		if (length > 2.0)
			length = 2.0;

		host_panel_pulse &pulse = m_host_panel_pulses[m_host_panel_pulse_next++ % MAX_HOST_PANEL_PULSES];
		pulse.row = s16(row);
		pulse.bit = s16(bit);
		pulse.end = now + length;
	}
}

void korgprophecy_state::drain_host_adin()
{
	if (g_kprop_host_adin_pop == nullptr)
		return;

	uint8_t source = 0;
	uint8_t value = 0;
	const double now = machine().time().as_double();
	while (g_kprop_host_adin_pop(&source, &value, now))
	{
		if (source >= m_adin_level.size() || m_adin_level[source] == value)
			continue;

		m_adin_level[source] = value;
		m_adc_mux_enable = true;
		if (source < m_adc_level.size())
			m_adc_level[source] = value;
		if (source < m_h8_adc_level.size())
			m_h8_adc_level[source] = u16(value << 2);
	}
}

void korgprophecy_state::render_lcd_snapshot(char *line1, char *line2) const
{
	auto as_ascii = [](u8 ch) -> char
	{
		return (ch >= 0x20 && ch <= 0x7e) ? char(ch) : '.';
	};

	const int visible_shift = ((m_lcd_disp_shift % 40) + 40) % 40;
	for (int column = 0; column < 40; column++)
	{
		const u8 c1 = m_lcd_ddram[0x00 + ((column + visible_shift) % 40)];
		const u8 c2 = m_lcd_ddram[0x40 + ((column + visible_shift) % 40)];
		line1[column] = as_ascii(c1 ? c1 : ' ');
		line2[column] = as_ascii(c2 ? c2 : ' ');
	}
	line1[40] = '\0';
	line2[40] = '\0';
}

void korgprophecy_state::push_host_lcd()
{
	if (g_kprop_host_lcd == nullptr && g_kprop_host_lcd_raw == nullptr)
		return;

	const double now = machine().time().as_double();
	if (now - m_host_lcd_last_push < (1.0 / 30.0))
		return;
	m_host_lcd_last_push = now;

	char line1[41]{};
	char line2[41]{};
	render_lcd_snapshot(line1, line2);
	auto all_blank = [](const char *text)
	{
		for (; *text; ++text)
			if (*text != ' ')
				return false;
		return true;
	};
	if (all_blank(line1) && all_blank(line2))
		return;

	std::array<u8, 40> raw1{};
	std::array<u8, 40> raw2{};
	const int visible_shift = ((m_lcd_disp_shift % 40) + 40) % 40;
	for (int column = 0; column < 40; column++)
	{
		const u8 c1 = m_lcd_ddram[0x00 + ((column + visible_shift) % 40)];
		const u8 c2 = m_lcd_ddram[0x40 + ((column + visible_shift) % 40)];
		raw1[column] = c1 ? c1 : 0x20;
		raw2[column] = c2 ? c2 : 0x20;
	}

	const bool text_changed = !(m_host_lcd_last_l1 == line1 && m_host_lcd_last_l2 == line2);
	const bool raw_changed = text_changed || raw1 != m_host_lcd_last_raw1 || raw2 != m_host_lcd_last_raw2
		|| m_lcd_cgram != m_host_lcd_last_cgram;
	if (!raw_changed)
		return;

	m_host_lcd_last_l1.assign(line1);
	m_host_lcd_last_l2.assign(line2);
	m_host_lcd_last_raw1 = raw1;
	m_host_lcd_last_raw2 = raw2;
	m_host_lcd_last_cgram = m_lcd_cgram;
	if (text_changed && g_kprop_host_lcd != nullptr)
		g_kprop_host_lcd(line1, line2);
	if (g_kprop_host_lcd_raw != nullptr)
		g_kprop_host_lcd_raw(raw1.data(), raw2.data(), m_lcd_cgram.data());
}

TIMER_CALLBACK_MEMBER(korgprophecy_state::host_service_tick)
{
	drain_host_midi();
	drain_host_panel();
	drain_host_adin();
	push_host_lcd();
}

void korgprophecy_state::enqueue_gui_midi_bytes(std::vector<u8> &&bytes, const std::string &tag)
{
	if (bytes.empty())
		return;

	const bool was_idle = !m_gui_midi_rx_active && m_gui_midi_rx_queue.empty() && m_gui_midi_rx_enqueued == 0;
	const size_t count = bytes.size();
	for (u8 byte : bytes)
		m_gui_midi_rx_queue.push_back(byte);

	m_gui_midi_rx_enqueued += count;
	if (was_idle)
	{
		m_gui_midi_rx_sent = 0;
		m_gui_midi_rx_last_progress_sent = 0;
		m_gui_midi_rx_last_progress_time = machine().time().as_double();
	}
	logerror("KPROP_GUI,SYSEX_QUEUE,T=%.6f,BYTES=%zu,SENT=%llu,TOTAL=%llu,QUEUE=%zu,PATH=\"%s\"\n",
		machine().time().as_double(),
		count,
		static_cast<unsigned long long>(m_gui_midi_rx_sent),
		static_cast<unsigned long long>(m_gui_midi_rx_enqueued),
		m_gui_midi_rx_queue.size(),
		tag.c_str());

	if (!m_gui_midi_rx_active && m_gui_midi_rx_timer != nullptr)
		m_gui_midi_rx_timer->adjust(attotime::zero);
}

TIMER_CALLBACK_MEMBER(korgprophecy_state::inject_note_tick)
{
	if (m_inject_note_phase == 0)
	{
		enqueue_gui_midi_bytes({0x90, m_inject_note, m_inject_note_vel}, "inject_note_on");
		logerror("KPROP_INJECT_NOTE,ON,T=%.6f,NOTE=%u,VEL=%u\n",
			machine().time().as_double(), unsigned(m_inject_note), unsigned(m_inject_note_vel));
		m_inject_note_phase = 1;
		if (m_inject_note_off > m_inject_note_on)
			m_inject_note_timer->adjust(attotime::from_double(m_inject_note_off - m_inject_note_on));
	}
	else
	{
		enqueue_gui_midi_bytes({0x80, m_inject_note, 0}, "inject_note_off");
		logerror("KPROP_INJECT_NOTE,OFF,T=%.6f,NOTE=%u\n",
			machine().time().as_double(), unsigned(m_inject_note));
	}
}

TIMER_CALLBACK_MEMBER(korgprophecy_state::inject_sysex_tick)
{
	const size_t n = m_inject_sysex_bytes.size();
	std::vector<u8> bytes = m_inject_sysex_bytes;
	enqueue_gui_midi_bytes(std::move(bytes), "inject_sysex");
	logerror("KPROP_INJECT_SYSEX,SENT,T=%.6f,BYTES=%zu\n", machine().time().as_double(), n);
}

TIMER_CALLBACK_MEMBER(korgprophecy_state::gui_midi_rx_bit_tick)
{
	if (!m_gui_midi_rx_active)
	{
		if (m_gui_midi_rx_queue.empty())
		{
			midi_rxd1_w(1);
			if (m_gui_midi_rx_enqueued != 0)
			{
				logerror("KPROP_GUI,SYSEX_DONE,T=%.6f,SENT=%llu,TOTAL=%llu\n",
					machine().time().as_double(),
					static_cast<unsigned long long>(m_gui_midi_rx_sent),
					static_cast<unsigned long long>(m_gui_midi_rx_enqueued));
				m_gui_midi_rx_enqueued = 0;
				m_gui_midi_rx_sent = 0;
				m_gui_midi_rx_last_progress_sent = 0;
				m_gui_midi_rx_last_progress_time = 0.0;
			}
			m_gui_midi_rx_timer->adjust(attotime::never);
			return;
		}

		const u8 byte = m_gui_midi_rx_queue.front();
		m_gui_midi_rx_queue.pop_front();
		// MIDI is 8N1, LSB-first, idle high: start bit 0, 8 data bits, stop 1.
		m_gui_midi_rx_frame = u16(0x0200 | (u16(byte) << 1));
		m_gui_midi_rx_bit = 0;
		m_gui_midi_rx_active = true;
	}

	midi_rxd1_w(BIT(m_gui_midi_rx_frame, m_gui_midi_rx_bit));
	m_gui_midi_rx_bit++;
	if (m_gui_midi_rx_bit >= 10)
	{
		m_gui_midi_rx_sent++;
		if (m_gui_midi_rx_enqueued != 0)
		{
			const double now = machine().time().as_double();
			const bool byte_interval = m_gui_midi_rx_sent >= m_gui_midi_rx_last_progress_sent + 512;
			const bool time_interval = (now - m_gui_midi_rx_last_progress_time) >= 0.25;
			const bool complete = m_gui_midi_rx_sent >= m_gui_midi_rx_enqueued;
			if (byte_interval || time_interval || complete)
			{
				const double pct = 100.0 * double(m_gui_midi_rx_sent) / double(m_gui_midi_rx_enqueued);
				logerror("KPROP_GUI,SYSEX_PROGRESS,T=%.6f,SENT=%llu,TOTAL=%llu,PCT=%.1f,QUEUE=%zu\n",
					now,
					static_cast<unsigned long long>(m_gui_midi_rx_sent),
					static_cast<unsigned long long>(m_gui_midi_rx_enqueued),
					pct,
					m_gui_midi_rx_queue.size());
				m_gui_midi_rx_last_progress_sent = m_gui_midi_rx_sent;
				m_gui_midi_rx_last_progress_time = now;
			}
		}
		m_gui_midi_rx_active = false;
	}
	m_gui_midi_rx_timer->adjust(attotime::from_hz(31'250));
}

void korgprophecy_state::midi_rxd1_w(int state)
{
	if (state != m_midi_rxd1_state)
	{
		m_midi_rxd1_state = state;
		m_midi_rxd1_edges++;
	}

	m_maincpu->rxd1_w(state);
}

void korgprophecy_state::h8_txd0_w(int state)
{
	const int old_state = m_h8_txd_state;
	if (state != m_h8_txd_state)
	{
		m_h8_txd_state = state;
		m_h8_txd_edges++;
	}

	if (!m_h8_txd_decode_active && old_state == 1 && state == 0)
	{
		m_h8_txd_decode_active = true;
		m_h8_txd_decode_byte = 0x00;
		m_h8_txd_decode_bit = 0x00;
		m_h8_txd_sample_timer->adjust(h8_sci0_bit_period() + (h8_sci0_bit_period() / 2));
	}

	m_maincpu->rxd_w(state);
}

void korgprophecy_state::update_h8_cts_from_port8()
{
	const bool p81 = BIT(m_h8_port8, 1);
	// CTS is active-low on the H8 serial link.
	const u8 state = p81 ? 0 : 1;
	m_maincpu->cts_w(state);
	if (state != m_h8_cts_state)
	{
		m_h8_cts_state = state;
	}
}

void korgprophecy_state::dsp1_empty_w(int state)
{
	if (state)
		m_dsp_empty |= 0x01;
	else
		m_dsp_empty &= ~0x01;
}

void korgprophecy_state::dsp2_empty_w(int state)
{
	if (state)
		m_dsp_empty |= 0x02;
	else
		m_dsp_empty &= ~0x02;
}

void korgprophecy_state::dsp3_empty_w(int state)
{
	if (state)
		m_dsp_empty |= 0x04;
	else
		m_dsp_empty &= ~0x04;
}

void korgprophecy_state::update_dsp_host_ctrl()
{
	// The H8 DSP loaders consistently drive Port A bit 2 low for the longer
	// program-upload path and bit 4 low for the 5-byte command/update path.
	const int pload = BIT(m_h8_porta, 2);
	const int cload = BIT(m_h8_porta, 4);
	const u8 old_pload = m_dsp_pload_level;

	if (m_dsp_strobe_addressed_only && m_dsp_strobe_last_target >= 0)
	{
		tms57002_device *const target =
				(m_dsp_strobe_last_target == 0) ? m_dsp1.target() :
				(m_dsp_strobe_last_target == 1) ? m_dsp2.target() : m_dsp3.target();
		target->pload_w(pload);
		target->cload_w(cload);
	}
	else
	{
		m_dsp1->pload_w(pload);
		m_dsp1->cload_w(cload);
		m_dsp2->pload_w(pload);
		m_dsp3->pload_w(pload);
		m_dsp2->cload_w(cload);
		m_dsp3->cload_w(cload);
	}

	if (old_pload != pload)
		m_dsp_pload_level = pload;
}

u8 korgprophecy_state::h8_nocycle_r8(u32 addr)
{
	addr &= 0x00ffffff;
	if (addr >= 0x040000 && addr <= 0x07ffff)
		return m_h8_sram[(addr - 0x040000) & 0x1ffff];
	// Instrumentation read: must never perturb side-effectful handlers
	// (e.g. a stack-derived dump address landing in the DSP host window).
	auto dis = machine().disable_side_effects();
	return m_subcpu->space(AS_PROGRAM).read_byte(addr);
}

u16 korgprophecy_state::h8_nocycle_r16(u32 addr)
{
	return (u16(h8_nocycle_r8(addr)) << 8) | h8_nocycle_r8(addr + 1);
}

u32 korgprophecy_state::h8_nocycle_r32(u32 addr)
{
	return (u32(h8_nocycle_r16(addr)) << 16) | h8_nocycle_r16(addr + 2);
}

u8 korgprophecy_state::h8_sram_r8(offs_t offset)
{
	return m_h8_sram[offset & 0x1ffff];
}

void korgprophecy_state::h8_sram_w8(offs_t offset, u8 data)
{
	m_h8_sram[offset & 0x1ffff] = data;
}

u8 korgprophecy_state::h8_dsp_r8(offs_t offset)
{
	// Debug taps and debugger views must not consume DSP host-port state.
	if (machine().side_effects_disabled())
		return 0xff;

	if (offset < 2 || offset > 7)
		return 0xff;

	const int host_slot = ((offset - 2) >> 1);
	if (host_slot < 0 || host_slot > 2)
		return 0xff;
	const int dsp = m_dsp_host_map[host_slot];

	tms57002_device *const target = (dsp == 0) ? &*m_dsp1 : (dsp == 1) ? &*m_dsp2 : &*m_dsp3;
	const u8 value = target->data_r();
	return value;
}

void korgprophecy_state::h8_dsp_w8(offs_t offset, u8 data)
{
	if (offset < 2 || offset > 7)
		return;

	const int host_slot = ((offset - 2) >> 1);
	if (host_slot < 0 || host_slot > 2)
		return;
	const int dsp = m_dsp_host_map[host_slot];
	m_dsp_strobe_last_target = dsp;
	tms57002_device *const target = (dsp == 0) ? &*m_dsp1 : (dsp == 1) ? &*m_dsp2 : &*m_dsp3;
	target->data_w(data);

	// The generic TMS57002 EMPTY callback tracks the internal update queue very
	// literally, but Prophecy firmware uses PB0-2 as a host-transfer pacing
	// signal. Keep the real DSP core behind the window while preserving the
	// short board-level busy pulse that let the H8 upload sequences run.
	if (BIT(m_dsp_empty, dsp))
	{
		m_dsp_empty &= ~(1U << dsp);
		if (m_dsp_empty_timer[dsp] != nullptr)
			m_dsp_empty_timer[dsp]->adjust(attotime::from_usec(m_dsp_empty_hold_us), dsp);
	}
}
u16 korgprophecy_state::h8_busctrl_r(offs_t offset, u16 mem_mask)
{
	const u8 reg = (offset & 1) << 1;
	return (u16(m_h8_busctrl[reg]) << 8) | m_h8_busctrl[reg + 1];
}

void korgprophecy_state::h8_busctrl_w(offs_t offset, u16 data, u16 mem_mask)
{
	const u8 reg = (offset & 1) << 1;
	if (mem_mask & 0xff00)
		m_h8_busctrl[reg] = (data >> 8) & 0xff;
	if (mem_mask & 0x00ff)
		m_h8_busctrl[reg + 1] = data & 0xff;
}


u16 korgprophecy_state::io_r(offs_t offset, u16 mem_mask)
{
	// Return idle-high data until actual peripherals are mapped.
	const u16 data = 0xffff;
	return data;
}

void korgprophecy_state::io_w(offs_t offset, u16 data, u16 mem_mask)
{
}

HD44780_PIXEL_UPDATE(korgprophecy_state::lcd_pixel_update)
{
	if (x < 5 && y < 8 && line < 2 && pos < 40)
		bitmap.pix(line * 8 + y, pos * 6 + x) = state;
}

u16 korgprophecy_state::mmio_r(offs_t offset, u16 mem_mask)
{
	offset &= 0x7f;
	const u32 addr = 0x0fff00 + (offset << 1);
	u16 data = m_mmio_shadow[offset];
	const u32 lcd_payload_addr = m_lcd_swap ? MMIO_LCD_CTRL : MMIO_LCD_DATA;

	switch (addr)
	{
	case MMIO_INPUT_STATUS:
	{
		// FFF00 is a mixed port:
		// - low byte: V55 P0 latch/status mix; bits 0-3/7 follow the control
		//   latch while bits 4-6 are direct card-status inputs
		// - high byte: a separate 5-bit input group folded into the sixth
		//   matrix byte by the firmware
		if (mem_mask & 0x00ff)
		{
			const u8 value = read_v55_p0_status();
			data = (data & 0xff00) | value;
			m_mmio_shadow[offset] = data;
			m_last_card_p0_status = value;
		}
		if (mem_mask & 0xff00)
			data = (data & 0x00ff) | (u16(read_special_inputs()) << 8);
		m_mmio_shadow[offset] = data;
		break;
	}

	case MMIO_INPUT_DATA:
		// FFF02 is used as a read/modify/write output latch in the boot/test
		// code, so plain readback of the current shadow is closer than treating
		// it as an input port.
		m_mmio_shadow[offset] = data;
		break;

	case MMIO_LCD_DATA:
		if (mem_mask & 0xff00)
		{
			u8 value = (data >> 8) & 0xff;

			if (addr == lcd_payload_addr)
			{
				value = m_lcd_data_mode ? m_lcdc->data_r() : m_lcdc->control_r();
				// KPSHIP-ACCURACY: LCD busy-bit is faked clear so boot progresses (2 sites).
				// Bring-up heuristic: firmware currently stalls forever polling busy=1.
				// Keep lower status/address bits but clear busy to let boot progress.
				if (!m_lcd_data_mode)
					value &= 0x7f;
			}

			data = (data & 0x00ff) | (u16(value) << 8);
			m_mmio_shadow[offset] = data;
		}
		break;

	case MMIO_LCD_CTRL:
		if (mem_mask & 0x00ff)
		{
			const u8 value = read_scanned_inputs();
			data = (data & 0xff00) | value;
			m_mmio_shadow[offset] = data;
		}
		if (mem_mask & 0xff00)
		{
			u8 value = (data >> 8) & 0xff;

			if (addr == lcd_payload_addr)
			{
				value = m_lcd_data_mode ? m_lcdc->data_r() : m_lcdc->control_r();
				// KPSHIP-ACCURACY: LCD busy-bit is faked clear so boot progresses (2 sites).
				// Bring-up heuristic: firmware currently stalls forever polling busy=1.
				// Keep lower status/address bits but clear busy to let boot progress.
				if (!m_lcd_data_mode)
					value &= 0x7f;
			}

			data = (data & 0x00ff) | (u16(value) << 8);
			m_mmio_shadow[offset] = data;
		}
		break;

	case 0x0fff30:
		// KPSHIP-ACCURACY: FFF30 modeled as an A/D result register (unconfirmed).
		// Bring-up hypothesis: this register behaves like A/D conversion result.
		if (mem_mask & 0xff00)
		{
			const u8 ch = m_adc_sel & 0x07;
			u8 value = m_adc_level[ch];
			data = (data & 0x00ff) | (u16(value) << 8);
			m_mmio_shadow[offset] = data;
			if (value != m_last_adc_value)
			{
				logerror("KPROP_ADC,ADDR=FFF30,CH=%u,VAL=%02X\n", ch, value);
				m_last_adc_value = value;
			}
		}
		break;

	case 0x0fff74:
		// After the corrected panel glue, the V55 advances to a wait loop at
		// B40DA that polls bit 5 of FFF74 low before setting a local ready flag.
		// Keep that status asserted until the surrounding peripheral block is
		// mapped more accurately.
		if (mem_mask & 0x00ff)
		{
			const u8 value = u8((data | 0x0020) & 0x00ff);
			data = (data & 0xff00) | value;
			m_mmio_shadow[offset] = data;
		}
		break;

	case 0x0fff76:
		// The queue-consumer path at B43F8 samples the adjacent low byte as a
		// reply/status code. The real backend response is not modeled here, so the
		// firmware reads back whatever the bus presents.
		break;

	case 0x0fff7c:
		// Later startup code at B4E2A polls bit 5 of FFF7C low in the same style
		// as the earlier FFF74-ready gate. The second backend channel is not
		// modeled here; the firmware reads back whatever the bus presents.
		break;
	}

	return data;
}

void korgprophecy_state::mmio_w(offs_t offset, u16 data, u16 mem_mask)
{
	offset &= 0x7f;
	COMBINE_DATA(&m_mmio_shadow[offset]);
	const u32 addr = 0x0fff00 + (offset << 1);
	const u32 lcd_mode_addr = m_lcd_swap ? MMIO_LCD_DATA : MMIO_LCD_CTRL;
	const u32 lcd_payload_addr = m_lcd_swap ? MMIO_LCD_CTRL : MMIO_LCD_DATA;

	switch (addr)
	{
	case MMIO_INPUT_DATA:
		// KPSHIP-ACCURACY: LED-data/scan-latch split at MMIO_INPUT_DATA is a working hypothesis.
		// Working hypothesis: upper byte carries LED data and scan latch strobes elsewhere.
		if (mem_mask & 0x00ff)
		{
			m_scan_pattern = m_mmio_shadow[offset] & 0xff;
		}
		if (mem_mask & 0xff00)
			update_led_bank((m_mmio_shadow[offset] >> 8) & 0xff);
		break;

	case MMIO_INPUT_STATUS:
		if (mem_mask & 0x00ff)
			m_input_ctrl = m_mmio_shadow[offset] & 0xff;
		break;

	case MMIO_LCD_CTRL:
	case MMIO_LCD_DATA:
		if (addr == lcd_mode_addr)
		{
			// Earlier trace-producing revisions only treated this register as an
			// RS/data-select latch, not as a direct HD44780 command port.
			if (m_mmio_shadow[offset] & 0x0200)
				m_lcd_data_mode = true;
		}
		else if ((addr == lcd_payload_addr) && (mem_mask & 0xff00))
		{
			const u8 value = (m_mmio_shadow[offset] >> 8) & 0xff;
			if (m_lcd_data_mode)
			{
				m_lcdc->data_w(value);
				lcd_trace_data(value);
			}
			else
			{
				m_lcdc->control_w(value);
				lcd_trace_control(value);
			}

			m_lcd_data_mode = false;
		}
		break;

	case MMIO_SCAN_SEL:
		if (mem_mask & 0xff00)
		{
			m_scan_sel = (m_mmio_shadow[offset] >> 8) & 0xff;
			if (!m_adc_sel_locked)
				m_adc_sel = m_scan_sel & 0x07;
			m_last_scan_sel = m_scan_sel;
		}
		break;
	}
}

void korgprophecy_state::prog_map(address_map &map)
{
	// Coarse V55 decode from the traced A19/A18 NAND tree:
	//   A19=0,A18=0 -> onboard battery-backed system SRAM (IC9/IC10)
	//   A19=0,A18=1 -> card window
	//   A19=1       -> system ROM
	map(0x00000, 0x3ffff).rw(FUNC(korgprophecy_state::sysram_r), FUNC(korgprophecy_state::sysram_w)).share("sysram");
	map(0x40000, 0x7ffff).ram();
	map(0x80000, 0xfffff).rom().region("maincpu", 0);
}

void korgprophecy_state::h8_map(address_map &map)
{
	// Traced decode shows IC22 (H8 ROM) and IC23 (H8 SRAM) behind a 74HC139.
	// The KiCad export is incomplete, so keep the conservative mirrored layout
	// until board traces or execution logs confirm the exact windows.
	map(0x000000, 0x07ffff).rom().region("subcpu", 0);
	map(0x040000, 0x05ffff).rw(FUNC(korgprophecy_state::h8_sram_r8), FUNC(korgprophecy_state::h8_sram_w8)).mirror(0x020000);
	map(0x0c0000, 0x0c0007).rw(FUNC(korgprophecy_state::h8_dsp_r8), FUNC(korgprophecy_state::h8_dsp_w8));
	map(0x0fffec, 0x0fffef).rw(FUNC(korgprophecy_state::h8_busctrl_r), FUNC(korgprophecy_state::h8_busctrl_w));
}

void korgprophecy_state::dsp_ram_map(address_map &map)
{
	// DSP1 and DSP3 each use a pair of LH64256BK-80 DRAMs. In the board docs
	// and service manual these are 256Kx4 parts, so the effective external
	// byte-wide data space is 0x40000 bytes, not the older 64K placeholder.
	map(0x00000, 0x3ffff).ram();
}

void korgprophecy_state::dsp_noram_map(address_map &map)
{
	// DSP2 has no external RAM on the printed schematic.
}

void korgprophecy_state::io_map(address_map &map)
{
	map(0x0000, 0xffff).rw(FUNC(korgprophecy_state::io_r), FUNC(korgprophecy_state::io_w));
}

void korgprophecy_state::palette_init(palette_device &palette)
{
	palette.set_pen_color(0, rgb_t(69, 63, 66));
	palette.set_pen_color(1, rgb_t(131, 136, 139));
}


static INPUT_PORTS_START(prophecy)
	PORT_START("CFG")
	PORT_CONFNAME(0x01, 0x00, "Auto Test Mode Combo")
	PORT_CONFSETTING(0x00, DEF_STR(Off))
	PORT_CONFSETTING(0x01, DEF_STR(On))
	PORT_CONFNAME(0x02, 0x00, "Auto Stimulus Sweep")
	PORT_CONFSETTING(0x00, DEF_STR(Off))
	PORT_CONFSETTING(0x02, DEF_STR(On))

		// Front-panel map as currently proven by a mix of:
		// - service-manual OCR and firmware-backed LED&SW TEST mapping
		// - normal-mode edit-page section jumps on the KLM-1847 bank
		// - PANEL0 is now backed by the firmware's own LED&SW test sequence.
		//   The row is stateful: the LCD shows the next expected switch, not
		//   always the one just accepted. The current proven physical corridor is:
		//   WRITE -> PERFORM/EDIT -> PERFORM/EDIT -> A -> B -> CARD ->
		//   INT PAT -> CARD PAT -> PE1 -> PE2 -> PE3 -> PE4 -> PE DEFINE ->
		//   COMPARE -> PROG/PAT SELECT -> 0 -> 1 -> 2 -> 3 -> 4 -> 5 ->
		//   6 -> 7 -> 8 -> 9 -> top-level PRELOAD.
		//   The later continuation is currently recorded in the docs rather than
		//   promoted wholesale into the live input labels below, because the
		//   dual-label numbering block still needs careful reconciliation.
		// - PANEL1 is now firmware-backed through the `LED&SW TEST` navigation
		//   corridor reached through the former selector-5 test harness:
		//   PAGE DOWN -> PAGE UP -> CURSOR < -> CURSOR > -> VALUE - ->
		//   VALUE + -> ENTER -> EXIT -> WRITE.
		//   In test mode, VALUE + also behaves like the manual's `PATS` helper:
		//   from `LED(ALL)`, row1 bit5 advances to the first switch prompt.
		//   One remaining nuance is the top-level `11 PRELOAD` page, where only
		//   row1 bit0 and row1 bit4 are currently observed to be live and both
		//   return to `1 LED&SW TEST`.
		// - A March 31 normal-mode follow-up from `.OSC-cmn1[Oscillator Set]`
		//   proved that part of the still-missing KLM-1847 edit-section bank is
		//   already present as dual-label secondary functions on PANEL0/PANEL4:
		//     PANEL0 bit2..7 = OSC, W.SHAPE, MIXER, FILTER, AMP, EFFECT
		//     PANEL4 bit4..7 = GLOBAL, COMMON, LFO, EG
		//   So the earlier "missing PANEL5..PANEL7 edit bank" assumption was
		//   too broad. The remaining gap is narrower: other KLM-1847 dual labels
		//   still need direct confirmation, especially around COMPARE/arrow/PAT.
		// - The later `CMPARE -> PROG/PAT SELECT -> 0..9` continuation is now
		//   also proven on the physical matrix (`PANEL2` and `PANEL4` low bits),
		//   but those labels stay conservative here until the normal-mode and
		//   service-mode dual-label semantics are reconciled fully.
		// Remaining rows are still provisional until the LED&SW sweep advances
		// farther through the service-manual switch list.
		PORT_START("PANEL0")
		PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("WRITE SW (LED&SW-derived)") PORT_CODE(KEYCODE_F1)
		PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("PERFORM/EDIT SW (LED&SW-derived)") PORT_CODE(KEYCODE_F2)
		PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("OSC / A / 0 (LED&SW-derived)") PORT_CODE(KEYCODE_F3)
		PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("W.SHAPE / B / 1 (LED&SW-derived)") PORT_CODE(KEYCODE_F4)
		PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("MIXER / CARD / 2 (LED&SW-derived)") PORT_CODE(KEYCODE_F5)
		PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("FILTER / INT PAT / 3 (LED&SW-derived)") PORT_CODE(KEYCODE_F6)
		PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("AMP / CARDPAT / 4 (LED&SW-derived)") PORT_CODE(KEYCODE_F7)
		PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("EFFECT / PE1 / 5 (LED&SW-derived)") PORT_CODE(KEYCODE_F8)

		PORT_START("PANEL1")
		PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("PAGE DOWN (LED&SW-derived)") PORT_CODE(KEYCODE_PGDN)
		PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("PAGE UP (LED&SW-derived)") PORT_CODE(KEYCODE_PGUP)
		PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("CURSOR < (LED&SW-derived)") PORT_CODE(KEYCODE_LEFT)
		PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("CURSOR > (LED&SW-derived)") PORT_CODE(KEYCODE_RIGHT)
		PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("VALUE - (LED&SW-derived)") PORT_CODE(KEYCODE_MINUS)
		PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("VALUE + / PATS (LED&SW-derived)") PORT_CODE(KEYCODE_EQUALS)
		PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("ENTER SW (LED&SW-derived)") PORT_CODE(KEYCODE_ENTER)
		PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("EXIT (LED&SW-derived)") PORT_CODE(KEYCODE_ESC)

	PORT_START("KBD0")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("KBD Row0 Bit0 (force)") PORT_CODE(KEYCODE_A)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("KBD Row0 Bit1 (force)") PORT_CODE(KEYCODE_S)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("KBD Row0 Bit2 (force)") PORT_CODE(KEYCODE_D)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("KBD Row0 Bit3 (force)") PORT_CODE(KEYCODE_F)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("KBD Row0 Bit4 (force)") PORT_CODE(KEYCODE_G)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("KBD Row0 Bit5 (force)") PORT_CODE(KEYCODE_H)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("KBD Row0 Bit6 (force)") PORT_CODE(KEYCODE_J)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("KBD Row0 Bit7 (force)") PORT_CODE(KEYCODE_K)

		// Later LED&SW-derived continuation after the early PANEL0/PANEL4 corridor:
		//   COMPARE -> PROG/PAT SELECT -> 0 -> 1 -> 2 -> 3 -> 4 -> 5
		PORT_START("PANEL2")
		PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("2 / CARD (LED&SW-derived)") PORT_CODE(KEYCODE_2)
		PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("3 / INT PAT (LED&SW-derived)") PORT_CODE(KEYCODE_3)
		PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("4 / CARD PAT (LED&SW-derived)") PORT_CODE(KEYCODE_4)
		PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("0 / A (LED&SW-derived)") PORT_CODE(KEYCODE_0)
		PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("5 / PE1 (LED&SW-derived)") PORT_CODE(KEYCODE_5)
		PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("1 / B (LED&SW-derived)") PORT_CODE(KEYCODE_1)
		PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("PROG/PAT SELECT SW (LED&SW-derived)") PORT_CODE(KEYCODE_OPENBRACE)
		PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("COMPARE SW (LED&SW-derived)") PORT_CODE(KEYCODE_C)

		// Early switch block now mostly recovered from `LED&SW TEST`:
		//   ARPEGGIO ON/OFF -> PATTERN DEFINE -> OCTAVE SW
		//   (OCTAVE 4/3/2 prompts) -> LATCH/KEY SYNC SW
		//   (LATCH&K.S./KEYSYNC/LATCH prompts) -> WHEEL 3 HOLD ->
		//   PORTAMENTO -> OCTAVE DOWN -> OCTAVE UP -> PAGE DOWN,
		//   then it rejoins PANEL1.
		PORT_START("PANEL3")
		PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("ARPEGGIO ON/OFF SW (LED&SW-derived)") PORT_CODE(KEYCODE_Q)
		PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("PATTERN DEFINE SW (LED&SW-derived)") PORT_CODE(KEYCODE_W)
		PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("OCTAVE SW (LED&SW-derived)") PORT_CODE(KEYCODE_6)
		PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("LATCH/KEY SYNC SW (LED&SW-derived)") PORT_CODE(KEYCODE_7)
		PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("OCTAVE UP SW (LED&SW-derived)") PORT_CODE(KEYCODE_8)
		PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("OCTAVE DOWN SW (LED&SW-derived)") PORT_CODE(KEYCODE_9)
		PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("PORTAMENTO SW (LED&SW-derived)") PORT_CODE(KEYCODE_CLOSEBRACE)
		PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("WHEEL 3 HOLD SW (LED&SW-derived)") PORT_CODE(KEYCODE_OPENBRACE)

		// Continued later corridor:
		//   6 -> 7 -> 8 -> 9, then the LED&SW page rolls into 11 PRELOAD.
	PORT_START("PANEL4")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("6 / PE2 (LED&SW-derived)") PORT_CODE(KEYCODE_6)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("7 / PE3 (LED&SW-derived)") PORT_CODE(KEYCODE_7)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("8 / PE4 (LED&SW-derived)") PORT_CODE(KEYCODE_8)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("9 / PE DEFINE (LED&SW-derived)") PORT_CODE(KEYCODE_9)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("GLOBAL / PE DEFINE SW (LED&SW-derived)") PORT_CODE(KEYCODE_O)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("COMMON / PE4 SW (LED&SW-derived)") PORT_CODE(KEYCODE_I)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("LFO / PE3 SW (LED&SW-derived)") PORT_CODE(KEYCODE_U)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("EG / PE2 SW (LED&SW-derived)") PORT_CODE(KEYCODE_Y)

	PORT_START("PANEL5")
	PORT_BIT(0xff, IP_ACTIVE_LOW, IPT_UNKNOWN)

	PORT_START("PANEL6")
	PORT_BIT(0xff, IP_ACTIVE_LOW, IPT_UNKNOWN)

	PORT_START("PANEL7")
	PORT_BIT(0xff, IP_ACTIVE_LOW, IPT_UNKNOWN)

	PORT_START("KBD1")
	PORT_BIT(0xff, IP_ACTIVE_LOW, IPT_UNKNOWN)

	PORT_START("KBD2")
	PORT_BIT(0xff, IP_ACTIVE_LOW, IPT_UNKNOWN)

	PORT_START("KBD3")
	PORT_BIT(0xff, IP_ACTIVE_LOW, IPT_UNKNOWN)

	PORT_START("KBD4")
	PORT_BIT(0xff, IP_ACTIVE_LOW, IPT_UNKNOWN)

	PORT_START("KBD5")
	PORT_BIT(0xff, IP_ACTIVE_LOW, IPT_UNKNOWN)

	PORT_START("KBD6")
	PORT_BIT(0xff, IP_ACTIVE_LOW, IPT_UNKNOWN)

	PORT_START("KBD7")
	PORT_BIT(0xff, IP_ACTIVE_LOW, IPT_UNKNOWN)
INPUT_PORTS_END

void korgprophecy_state::prophecy(machine_config &config)
{
	const u32 v55_clock = (16_MHz_XTAL).value();
	// H8 default: 24.576 MHz audio crystal / 2. The old 16 MHz guess ran every
	// H8 timebase 1.302x fast; at 12.288 MHz the firmware's WOVI task lands at
	// 4.0625 ms, matching the hardware bus heartbeat (4.063 ms) exactly, and
	// all scoreboard onsets move toward hardware by the same ratio.
	// (korgprophecy_20260612_clock_fix_validation.md)
	const u32 h8_clock = (XTAL(24'576'000) / 2).value();

	V55(config, m_maincpu, v55_clock); // NEC V55 sub-CPU (uPD70433, V5x-based)
	m_maincpu->set_addrmap(AS_PROGRAM, &korgprophecy_state::prog_map);
	m_maincpu->set_addrmap(AS_IO, &korgprophecy_state::io_map);
	m_maincpu->in_p_cb<0>().set(FUNC(korgprophecy_state::v55_p0_r));
	m_maincpu->out_p_cb<0>().set(FUNC(korgprophecy_state::v55_p0_w));
	m_maincpu->in_p_cb<1>().set(FUNC(korgprophecy_state::v55_p1_r));
	m_maincpu->in_p_cb<2>().set(FUNC(korgprophecy_state::v55_p2_r));
	m_maincpu->out_p_cb<2>().set(FUNC(korgprophecy_state::v55_p2_w));
	m_maincpu->out_p_cb<3>().set(FUNC(korgprophecy_state::v55_p3_w));
	m_maincpu->in_p_cb<4>().set(FUNC(korgprophecy_state::v55_p4_r));
	m_maincpu->out_p_cb<5>().set(FUNC(korgprophecy_state::v55_p5_w));
	m_maincpu->in_p_cb<7>().set(FUNC(korgprophecy_state::v55_p7_r));
	m_maincpu->out_p_cb<7>().set(FUNC(korgprophecy_state::v55_p7_w));
	m_maincpu->in_p_cb<8>().set(FUNC(korgprophecy_state::v55_p8_r));
	m_maincpu->out_p_cb<8>().set(FUNC(korgprophecy_state::v55_p8_w));
	m_maincpu->adc_fint_cb().set(FUNC(korgprophecy_state::v55_adc_fint_w));
	m_maincpu->read_adc<0>().set(FUNC(korgprophecy_state::v55_adc_r<0>));
	m_maincpu->read_adc<1>().set(FUNC(korgprophecy_state::v55_adc_r<1>));
	m_maincpu->read_adc<2>().set(FUNC(korgprophecy_state::v55_adc_r<2>));
	m_maincpu->read_adc<3>().set(FUNC(korgprophecy_state::v55_adc_r<3>));
	// Shipping uses the native channel-0 RX/TX workers and channel-1 MIDI/SysEx
	// workers. The driver-side RAM-queue transport remains structurally idle.
	m_maincpu->set_serial_irq_mode(v55_device::serial_irq_mode::rx0_tx0);
	// The V55 board-link TX rate must track the H8 SCI0 rate (= h8_clock/384
	// for Prophecy's boot SMR=00/BRR=0b config); it was previously hardcoded
	// to the 16 MHz-derived 41,667 baud.
	m_maincpu->set_uart0_bit_rate(u32(double(h8_clock) / 384.0 + 0.5));
	// V55 register-bank interrupt targets come from the low nibble of each
	// vector-table entry. Prophecy's saved vector 20 entry selects bank 13;
	// leave KPROP_V55_TM2_BANK as an explicit diagnostic override only.

	NVRAM(config, "sysram", nvram_device::DEFAULT_ALL_0);

	H83003(config, m_subcpu, h8_clock).set_mode_a20();
	m_subcpu->set_addrmap(AS_PROGRAM, &korgprophecy_state::h8_map);
	m_subcpu->read_adc<0>().set(FUNC(korgprophecy_state::h8_adc_r<0>));
	m_subcpu->read_adc<1>().set(FUNC(korgprophecy_state::h8_adc_r<1>));
	m_subcpu->read_adc<2>().set(FUNC(korgprophecy_state::h8_adc_r<2>));
	m_subcpu->read_adc<3>().set(FUNC(korgprophecy_state::h8_adc_r<3>));
	m_subcpu->read_adc<4>().set(FUNC(korgprophecy_state::h8_adc_r<4>));
	m_subcpu->read_adc<5>().set(FUNC(korgprophecy_state::h8_adc_r<5>));
	m_subcpu->read_adc<6>().set(FUNC(korgprophecy_state::h8_adc_r<6>));
	m_subcpu->read_adc<7>().set(FUNC(korgprophecy_state::h8_adc_r<7>));
	m_subcpu->read_porta().set(FUNC(korgprophecy_state::h8_porta_r));
	m_subcpu->read_portb().set(FUNC(korgprophecy_state::h8_portb_r));
	m_subcpu->read_portc().set(FUNC(korgprophecy_state::h8_portc_r));
	m_subcpu->write_porta().set(FUNC(korgprophecy_state::h8_porta_w));
	m_subcpu->write_portc().set(FUNC(korgprophecy_state::h8_portc_w));
	m_subcpu->write_port8().set(FUNC(korgprophecy_state::h8_port8_w));

	// Confirmed board links: V55 TXD0/RXD0 <-> H8 RXD0/TXD0 and H8 P81 -> V55 P33.
	// P33 is grouped with the serial pins on the original schematic even though
	// it is not explicitly text-labeled CTS0 there.
	m_maincpu->txd_handler_cb().set(FUNC(korgprophecy_state::v55_txd_w));
	m_maincpu->txd1_handler_cb().set(FUNC(korgprophecy_state::v55_txd1_w));
	m_maincpu->txd1_handler_cb().append("mdout", FUNC(midi_port_device::write_txd));
	m_subcpu->write_sci_tx<0>().set(FUNC(korgprophecy_state::h8_txd0_w));

	auto &mdin(MIDI_PORT(config, "mdin"));
	midiin_slot(mdin);
	mdin.rxd_handler().set(FUNC(korgprophecy_state::midi_rxd1_w));
	midiout_slot(MIDI_PORT(config, "mdout"));

	TMS57002(config, m_dsp1, XTAL(24'576'000));
	m_dsp1->set_addrmap(AS_DATA, &korgprophecy_state::dsp_ram_map);
	m_dsp1->empty_callback().set(FUNC(korgprophecy_state::dsp1_empty_w));

	TMS57002(config, m_dsp2, XTAL(24'576'000));
	m_dsp2->set_addrmap(AS_DATA, &korgprophecy_state::dsp_noram_map);
	m_dsp2->empty_callback().set(FUNC(korgprophecy_state::dsp2_empty_w));

	TMS57002(config, m_dsp3, XTAL(24'576'000));
	m_dsp3->set_addrmap(AS_DATA, &korgprophecy_state::dsp_ram_map);
	m_dsp3->empty_callback().set(FUNC(korgprophecy_state::dsp3_empty_w));

	KORGPROPHECY_U2(config, m_u2);

	// First-pass DSP serial graph, matching the traced forward links:
	//   DSP1 SO0 -> DSP2 SI0
	//   DSP1 SO1 -> DSP2 SI1
	//   DSP2 SO0 -> DSP3 SI0
	//   DSP2 SO1 -> DSP1 SI0 (modeled by a one-sample delayed latch below)
	// KPSHIP-ACCURACY: board-sync timer disabled because the current core double-syncs DSP3;
	// a symptom that the DSP3 sync/framing model is not yet faithful.
	// The TMS57002 core already pulses SYNC once per sound-stream sample. An
	// extra 48 kHz board-sync timer aimed at DSP3 was useful during earlier
	// headless bring-up, but with the current core it double-syncs DSP3 and
	// produces the alternating/aliased final-stage output. Keep the board-sync
	// timer disabled by default for normal audio runs; the env presets below can
	// still re-enable it for targeted experiments or no-audio/headless cases.
	// The external SDN -> DSP1 SI1 path stays silent until its source is
	// identified. The DSP2 SO1 feedback edge cannot be represented as a direct
	// sound route because it forms a stream cycle with DSP1 -> DSP2; the
	// 48 kHz feedback latch models it as previous-frame serial input.
	// The separate OPTIONIN source behind the final-stage program is pulled up
	// through R7 in the extracted netlist. The DAC side is a separate block:
	// the printed original schematic shows only DSP3 SO0 reaching U2 DATA, with
	// IC24/IC25 deriving the DAC WS/LRCK framing.
	const dsp_board_dsp2_input_mode dsp2_input_route = parse_dsp2_input_route_env(std::getenv("KPROP_DSP2_INPUT_ROUTE"));
	switch (dsp2_input_route)
	{
	case dsp_board_dsp2_input_mode::so1_to_si0:
		m_dsp1->add_route(2, *m_dsp2, 1.0, 0);
		m_dsp1->add_route(3, *m_dsp2, 1.0, 1);
		m_dsp1->add_route(0, *m_dsp2, 1.0, 2);
		m_dsp1->add_route(1, *m_dsp2, 1.0, 3);
		break;

	case dsp_board_dsp2_input_mode::so1_r_to_si0_r:
		m_dsp1->add_route(0, *m_dsp2, 1.0, 0);
		m_dsp1->add_route(3, *m_dsp2, 1.0, 1);
		m_dsp1->add_route(2, *m_dsp2, 1.0, 2);
		m_dsp1->add_route(3, *m_dsp2, 1.0, 3);
		break;

	case dsp_board_dsp2_input_mode::so1_l_to_si0_r:
		m_dsp1->add_route(0, *m_dsp2, 1.0, 0);
		m_dsp1->add_route(2, *m_dsp2, 1.0, 1);
		m_dsp1->add_route(2, *m_dsp2, 1.0, 2);
		m_dsp1->add_route(3, *m_dsp2, 1.0, 3);
		break;

	case dsp_board_dsp2_input_mode::so1_l_to_si0_l:
		m_dsp1->add_route(2, *m_dsp2, 1.0, 0);
		m_dsp1->add_route(1, *m_dsp2, 1.0, 1);
		m_dsp1->add_route(2, *m_dsp2, 1.0, 2);
		m_dsp1->add_route(3, *m_dsp2, 1.0, 3);
		break;

	case dsp_board_dsp2_input_mode::so1_r_to_si0_l:
		m_dsp1->add_route(3, *m_dsp2, 1.0, 0);
		m_dsp1->add_route(1, *m_dsp2, 1.0, 1);
		m_dsp1->add_route(2, *m_dsp2, 1.0, 2);
		m_dsp1->add_route(3, *m_dsp2, 1.0, 3);
		break;

	case dsp_board_dsp2_input_mode::dsp3_so1_to_si0:
	case dsp_board_dsp2_input_mode::dsp3_so0_to_si0:
	case dsp_board_dsp2_input_mode::dsp3_so1_dsp1_so0:
		// These cyclic routes are realized by dsp_board_sync_tick's one-sample
		// latches; adding a sound-stream edge here would create a graph cycle.
		break;

	case dsp_board_dsp2_input_mode::normal:
	default:
		m_dsp1->add_route(0, *m_dsp2, 1.0, 0);
		m_dsp1->add_route(1, *m_dsp2, 1.0, 1);
		m_dsp1->add_route(2, *m_dsp2, 1.0, 2);
		m_dsp1->add_route(3, *m_dsp2, 1.0, 3);
		break;
	}

	// DSP3 (effects/final stage) is fed by DSP2's SO0 pair.
	m_dsp2->add_route(0, *m_dsp3, 1.0, 0);
	m_dsp2->add_route(1, *m_dsp3, 1.0, 1);

	screen_device &screen(SCREEN(config, "screen").set_lcd());
	screen.set_color(rgb_t::green());
	screen.set_refresh_hz(60);
	screen.set_vblank_time(ATTOSECONDS_IN_USEC(2500));
	screen.set_screen_update("lcdc", FUNC(hd44780_a00_reconstructed_device::screen_update));
	screen.set_size(6 * 40, 8 * 2);
	screen.set_visarea_full();
	screen.set_palette("palette");

	PALETTE(config, "palette", FUNC(korgprophecy_state::palette_init), 2);

	SPEAKER(config, "lspeaker").front_left();
	SPEAKER(config, "rspeaker").front_right();

	// KPSHIP-ACCURACY: HD44780 clock is a placeholder; not measured off hardware.
	// The A00 character generator is a datasheet reconstruction compiled into
	// this explicit device variant, so users only need to provide Korg firmware.
	HD44780_A00_RECONSTRUCTED(config, m_lcdc, 270'000); // TODO: clock not measured
	m_lcdc->set_lcd_size(2, 40);
	m_lcdc->set_function_set_at_any_time();
	m_lcdc->set_pixel_update_cb(FUNC(korgprophecy_state::lcd_pixel_update));

	// The board only wires DSP3 SO0 to U2 DATA. In the TMS57002 core, SO0 is
	// exposed as a left/right word pair (`so0_l` / `so0_r`), so route those two
	// words into the placeholder U2 DAC while the exact IC24/IC25-derived
	// WS/LRCK edge timing remains simplified.
	//
	// The board only wires DSP3 SO0 (a left/right word pair) to the U2 DAC.
	m_dsp1->set_stream_output_raw(false);
	m_dsp2->set_stream_output_raw(false);
	m_dsp3->add_route(0, *m_u2, 1.0, 0);
	m_dsp3->add_route(1, *m_u2, 1.0, 1);
	m_u2->add_route(0, "lspeaker", 0.50);
	m_u2->add_route(1, "rspeaker", 0.50);
}


ROM_START(korgprop)
	ROM_REGION(0x80000, "maincpu", 0)
	ROM_LOAD("ic12_v17.bin", 0x00000, 0x80000, CRC(55d9aef8) SHA1(2dc101c1fad46366cfec43e450844a9e3b613d8c))

	ROM_REGION(0x80000, "subcpu", 0)
	ROM_LOAD16_WORD_SWAP("ic22_v17.bin", 0x00000, 0x20000, CRC(80ac49ad) SHA1(d394c371279fa7a8c3660bf6c78ea65a0c2a6db0))
	ROM_RELOAD(0x20000, 0x20000)
	ROM_RELOAD(0x40000, 0x20000)
	ROM_RELOAD(0x60000, 0x20000)
ROM_END

ROM_START(korgpro101)
	ROM_REGION(0x80000, "maincpu", 0)
	ROM_LOAD("ic2_v101.bin", 0x00000, 0x80000, CRC(0d4b97f1) SHA1(76abf1dc49c7669ea0c3a36bf6d27aaab9fed3c1))

	ROM_REGION(0x80000, "subcpu", 0)
	ROM_LOAD16_WORD_SWAP("ic22_v101.bin", 0x00000, 0x20000, CRC(de0ff6b7) SHA1(79a6e61a6d3aecb9ecebfcddec643ff13c28f62e))
	ROM_RELOAD(0x20000, 0x20000)
	ROM_RELOAD(0x40000, 0x20000)
	ROM_RELOAD(0x60000, 0x20000)
ROM_END

ROM_START(korgpro20)
	ROM_REGION(0x80000, "maincpu", 0)
	ROM_LOAD("ic12_v20.bin", 0x00000, 0x80000, CRC(f33b7932) SHA1(950baf1f6c54394e780898634b7776e49f43acce))

	ROM_REGION(0x80000, "subcpu", 0)
	ROM_LOAD16_WORD_SWAP("ic22_v20.bin", 0x00000, 0x80000, CRC(0d321a5f) SHA1(cd4cb221da19dd6f746dc7f3c145b1494993f6d8))
ROM_END


SYST(1995, korgprop,   0,        0, prophecy, prophecy, korgprophecy_state, empty_init, "Korg", "Prophecy Solo Synthesizer (v1.7)",  MACHINE_NOT_WORKING)
SYST(1995, korgpro101, korgprop, 0, prophecy, prophecy, korgprophecy_state, empty_init, "Korg", "Prophecy Solo Synthesizer (v1.01)", MACHINE_NOT_WORKING)
SYST(1995, korgpro20,  korgprop, 0, prophecy, prophecy, korgprophecy_state, empty_init, "Korg", "Prophecy Solo Synthesizer (v2.0)",  MACHINE_NOT_WORKING)
