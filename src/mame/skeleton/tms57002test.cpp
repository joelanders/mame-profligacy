// license:BSD-3-Clause
// copyright-holders:Joe Landers
/***************************************************************************

    tms57002test.cpp

    Minimal TMS57002 semantics harness.

***************************************************************************/

#include "emu.h"

#include "cpu/tms57002/tms57002.h"

#include "screen.h"

#include <cstring>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace {

class tms57002test_state : public driver_device
{
public:
	tms57002test_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_dsp(*this, "dsp")
	{
	}

	void tms57002test(machine_config &config);
	void dsp_ram_map(address_map &map);

protected:
	virtual void machine_start() override ATTR_COLD;

private:
	void run_tests_now();
	u32 screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);

	static s64 sx48(u64 value)
	{
		constexpr u64 mask48 = (u64(1) << 48) - 1;
		value &= mask48;
		return BIT(value, 47) ? s64(value | ~mask48) : s64(value);
	}

	static s64 sx52(u64 value)
	{
		constexpr u64 mask52 = (u64(1) << 52) - 1;
		value &= mask52;
		return BIT(value, 51) ? s64(value | ~mask52) : s64(value);
	}

	static s64 sx24(u32 value)
	{
		constexpr u64 mask24 = 0x00ffffffU;
		u64 wide = value & mask24;
		return BIT(wide, 23) ? s64(wide | ~mask24) : s64(wide);
	}

	void check_equal(const char *name, s64 actual, s64 expected);
	void check_true(const char *name, bool actual);
	void check_false(const char *name, bool actual);
	void test_macc_output_clipping();
	void test_domh_movm_rounding_oracle_boundaries();
	void test_macc_output_unrounded_exceptions();
	void test_aovm_neg_abs_int32_min();
	void test_mac_a_d_full_width_port();
	void test_multiplier_a_port_drops_low_byte();
	void test_sequential_pc_wrap_halts_until_sync();
	void test_smld_raw_low_port();
	void test_mac_a_d_and_smld_port_semantics();
	void test_domh_sfmo2_trace_cases();
	[[maybe_unused]] void test_dsp3_a02_f8_ff_tail_chain();
	void test_instruction_level_serial_passthrough();
	void test_example53_bank_switched_serial_access();
	void test_example53_one_frame_output_latency();
	void test_example53_dos_deadline();
	[[maybe_unused]] void test_dsp3_mpy_dis_uses_prior_dmem();
	void test_dsp3_mpy_dis_uses_incoming_dmem();
	void test_dsp3_wre_smhd_orders_store_before_xwr();
	void test_dsp3_rde_sacc_uses_store_updated_cmem_for_xoa();
	void test_dsp3_lmhd_srbd_uses_store_updated_dmem_for_macc();
	void test_lacc_lira_same_word_uses_pre_lacc_aacc();
	void test_cmem_update_multiword_sequence();
	void test_cmem_update_waits_for_cload_high();
	void test_cmem_update_new_address_breaks_active_run();
	void test_pc33_mpy_mac_direct_coeff_chain();
	void test_pc33_producer_smhc_phase();
	void test_dsp3_rde_word0_sel1_dram_latency();
	void test_xmem_micro_oracles();
	void test_dsp3_snapshot_replay_case_with_program(const char *label, const char *program_path, const char *snapshot_path, const char *exec_path, const char *ram_path = nullptr);
	void test_dsp3_snapshot_replay_case(const char *label, const char *snapshot_path, const char *exec_path, const char *ram_path = nullptr);
	void test_dsp3_input_sensitivity_case_with_program(const char *label, const char *program_path, const char *snapshot_path, const char *exec_path, const char *ram_path);
	void test_dsp3_input_sensitivity_case(const char *label, const char *snapshot_path, const char *exec_path, const char *ram_path);
	void test_dsp3_a02_su480002_input_sensitivity();
	void test_dsp3_a02_su480003_input_sensitivity();
	void test_dsp3_a02_su480004_snapshot_replay();
	void test_dsp3_a02_sine_su480003_snapshot_replay();
	void test_dsp3_a02_su480004_input_sensitivity();
	void test_dsp3_a02_su480005_snapshot_replay();
	void test_dsp3_a02_su480005_input_sensitivity();
	void probe_dsp3_a02_su480005_state_sensitivity();
	void probe_dsp3_a02_su480005_first_diff_variants();
	void probe_dsp3_a02_su480004_producers();
	void probe_dsp3_a02_su480004_producer_site_ba0_variants();
	void probe_dsp3_a02_su480005_ba0_phase_sensitivity();
	void probe_dsp3_a02_su480005_site_ba0_variants();
	void probe_dsp3_a02_su480005_repeat_site_ba0_variants();
	void probe_dsp3_a02_tape_su480003_group_sensitivity();
	void probe_dsp3_a02_tape_su480003_producer_sensitivity();
	void probe_dsp3_a02_tape_su480002_hot_producer_sensitivity();
	void probe_dsp3_a02_tape_su480002_serial_firstdiff();
	void probe_dsp3_a02_tape_su480002_serial_path_checks();
	void probe_dsp3_serial_side_boundary_sensitivity();
	void probe_dsp3_a02_fix_su95299_coeff_sensitivity();
	void probe_dsp3_a02_fix_su95298_b3_operand_source();
	void probe_dsp3_a02_onset_boundary_replay();
	void probe_dsp3_a12_drone_regrowth();
	void probe_dsp3_a02_tape_boundary_replay();
	void probe_dsp3_a02_tape_cycle_budget();
	void test_dsp3_a02_su480006_snapshot_replay();
	void test_dsp3_a02_su480006_input_sensitivity();
	void test_dsp3_a02_su480001_pc68_mac_matches_input();
	void dump_dsp3_a02_su480006_replay_block();
	void test_dsp3_a02_su480082_snapshot_replay();
	void run_filter_replay_from_env();

	void run_all_focused_tests();
	void run_legacy_research_tests();

	required_device<tms57002_device> m_dsp;
	int m_failures = 0;
};


void tms57002test_state::machine_start()
{
	run_tests_now();
}

u32 tms57002test_state::screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect)
{
	return 0;
}

void tms57002test_state::check_equal(const char *name, s64 actual, s64 expected)
{
	if (actual != expected)
	{
		m_failures++;
		osd_printf_error("FAIL %-40s actual=%016llx expected=%016llx\n",
			name,
			(unsigned long long)actual,
			(unsigned long long)expected);
	}
	else
	{
		osd_printf_info("PASS %-40s value=%016llx\n", name, (unsigned long long)actual);
	}
}

void tms57002test_state::check_true(const char *name, bool actual)
{
	if (!actual)
	{
		m_failures++;
		osd_printf_error("FAIL %-40s actual=false expected=true\n", name);
	}
	else
	{
		osd_printf_info("PASS %-40s true\n", name);
	}
}

void tms57002test_state::check_false(const char *name, bool actual)
{
	if (actual)
	{
		m_failures++;
		osd_printf_error("FAIL %-40s actual=true expected=false\n", name);
	}
	else
	{
		osd_printf_info("PASS %-40s false\n", name);
	}
}

namespace {

static std::string legacy_asset_path(const char *path)
{
	const char *root = std::getenv("TMS57TEST_LEGACY_ASSET_ROOT");
	if (!root || !root[0])
		return path;

	std::string resolved(root);
	if (!resolved.empty() && resolved.back() != '/')
		resolved.push_back('/');
	return resolved + path;
}

static std::vector<std::string> load_text_lines(const char *path)
{
	const std::string resolved = legacy_asset_path(path);
	std::ifstream input(resolved);
	if (!input)
		throw emu_fatalerror("Unable to open %s (asset %s)", resolved, path);

	std::vector<std::string> lines;
	for (std::string line; std::getline(input, line); )
	{
		if (!line.empty())
			lines.push_back(line);
	}
	return lines;
}

static std::string field_value(const std::string &line, const char *key)
{
	const std::string needle(key);
	const std::size_t pos = line.find(needle);
	if (pos == std::string::npos)
		throw emu_fatalerror("Missing field %s in line: %s", key, line);

	const std::size_t start = pos + needle.size();
	const std::size_t end = line.find(',', start);
	return line.substr(start, end == std::string::npos ? std::string::npos : end - start);
}

static u32 parse_hex_u32(const std::string &text)
{
	return u32(std::strtoul(text.c_str(), nullptr, 16));
}

static u64 parse_hex_u64(const std::string &text)
{
	return u64(std::strtoull(text.c_str(), nullptr, 16));
}

static u64 parse_dec_u64(const std::string &text)
{
	return u64(std::strtoull(text.c_str(), nullptr, 10));
}

static int parse_dec_int(const std::string &text)
{
	return int(std::strtol(text.c_str(), nullptr, 10));
}

static std::array<u32, 4> parse_hex_slash4(const std::string &text)
{
	std::array<u32, 4> values{};
	std::stringstream ss(text);
	for (u32 i = 0; i < 4; i++)
	{
		std::string part;
		if (!std::getline(ss, part, '/'))
			throw emu_fatalerror("Expected 4 slash-separated hex values in %s", text);
		values[i] = parse_hex_u32(part);
	}
	return values;
}

static std::vector<u32> load_program_words(const char *path)
{
	const std::string resolved = legacy_asset_path(path);
	std::ifstream input(resolved, std::ios::binary);
	if (!input)
		throw emu_fatalerror("Unable to open %s (asset %s)", resolved, path);

	std::vector<u8> bytes((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
	if ((bytes.size() & 3) != 0)
		throw emu_fatalerror("Program image %s has non-word-aligned size %zu", path, bytes.size());

	std::vector<u32> words(bytes.size() / 4);
	for (std::size_t i = 0; i < words.size(); i++)
	{
		words[i] = u32(bytes[i * 4 + 0])
			| (u32(bytes[i * 4 + 1]) << 8)
			| (u32(bytes[i * 4 + 2]) << 16)
			| (u32(bytes[i * 4 + 3]) << 24);
	}
	return words;
}

static std::vector<u32> load_hex_words(const char *path, u32 mask = 0xffffffffU)
{
	std::ifstream input(path);
	if (!input)
		throw emu_fatalerror("Unable to open %s", path);

	std::vector<u32> words;
	for (std::string line; std::getline(input, line); )
	{
		const std::size_t comment = line.find('#');
		if (comment != std::string::npos)
			line.resize(comment);
		const std::size_t first = line.find_first_not_of(" \t\r\n");
		if (first == std::string::npos)
			continue;
		const std::size_t last = line.find_last_not_of(" \t\r\n");
		const std::string text = line.substr(first, last - first + 1);
		words.push_back(u32(std::strtoul(text.c_str(), nullptr, 0)) & mask);
	}
	return words;
}

static const char *required_env(const char *name)
{
	if (const char *value = std::getenv(name); value && value[0])
		return value;
	throw emu_fatalerror("Missing required environment variable %s", name);
}

static u32 env_hex(const char *name, u32 fallback = 0)
{
	// Base 16 (with or without 0x prefix). Base-0 auto-detect silently
	// misparsed bare-hex values like "8a0000" (-> 8) or leading-zero values
	// (-> octal).
	if (const char *value = std::getenv(name); value && value[0])
		return u32(std::strtoul(value, nullptr, 16));
	return fallback;
}

static int env_int(const char *name, int fallback = 0)
{
	if (const char *value = std::getenv(name); value && value[0])
		return int(std::strtol(value, nullptr, 0));
	return fallback;
}

static bool env_bool(const char *name, bool fallback = false)
{
	if (const char *value = std::getenv(name); value && value[0])
		return std::strtol(value, nullptr, 0) != 0;
	return fallback;
}

static std::vector<std::string> split_tab_line(const std::string &line)
{
	std::vector<std::string> parts;
	std::stringstream ss(line);
	for (std::string part; std::getline(ss, part, '\t'); )
		parts.push_back(part);
	return parts;
}

struct serial_replay_row
{
	u32 frame = 0;
	std::array<u32, 4> si{};
};

struct cload_replay_row
{
	u32 frame = 0;
	u8 sa = 0;
	u32 value = 0;
};

static std::vector<serial_replay_row> load_serial_replay_rows(const char *path)
{
	std::ifstream input(path);
	if (!input)
		throw emu_fatalerror("Unable to open %s", path);

	std::vector<serial_replay_row> rows;
	for (std::string line; std::getline(input, line); )
	{
		if (line.empty() || line[0] == '#')
			continue;
		const std::vector<std::string> parts = split_tab_line(line);
		if (parts.empty() || parts[0] == "frame")
			continue;
		if (parts.size() < 5)
			throw emu_fatalerror("Expected frame/si0/si1/si2/si3 in %s line: %s", path, line.c_str());

		serial_replay_row row;
		row.frame = u32(std::strtoul(parts[0].c_str(), nullptr, 0));
		for (u32 i = 0; i < 4; i++)
			row.si[i] = u32(std::strtoul(parts[1 + i].c_str(), nullptr, 0)) & 0x00ffffffU;
		rows.push_back(row);
	}
	return rows;
}

static std::vector<cload_replay_row> load_cload_replay_rows(const char *path)
{
	if (!path || !path[0])
		return {};
	std::ifstream input(path);
	if (!input)
		throw emu_fatalerror("Unable to open %s", path);

	std::vector<cload_replay_row> rows;
	for (std::string line; std::getline(input, line); )
	{
		if (line.empty() || line[0] == '#')
			continue;
		const std::vector<std::string> parts = split_tab_line(line);
		if (parts.empty() || parts[0] == "frame")
			continue;
		if (parts.size() < 3)
			throw emu_fatalerror("Expected frame/sa/value in %s line: %s", path, line.c_str());

		cload_replay_row row;
		row.frame = u32(std::strtoul(parts[0].c_str(), nullptr, 0));
		row.sa = u8(std::strtoul(parts[1].c_str(), nullptr, 0));
		row.value = u32(std::strtoul(parts[2].c_str(), nullptr, 0));
		rows.push_back(row);
	}
	return rows;
}

static void apply_replay_cload(tms57002_device &dsp, u8 sa, u32 value)
{
	dsp.cload_w(0);
	dsp.data_w(sa);
	dsp.data_w(u8((value >> 24) & 0xff));
	dsp.data_w(u8((value >> 16) & 0xff));
	dsp.data_w(u8((value >> 8) & 0xff));
	dsp.data_w(u8(value & 0xff));
	dsp.cload_w(1);
}

static void host_write_word24(tms57002_device &dsp, u32 value)
{
	dsp.data_w(u8((value >> 16) & 0xff));
	dsp.data_w(u8((value >> 8) & 0xff));
	dsp.data_w(u8(value & 0xff));
}

static void host_write_word32(tms57002_device &dsp, u32 value)
{
	dsp.data_w(u8((value >> 24) & 0xff));
	dsp.data_w(u8((value >> 16) & 0xff));
	dsp.data_w(u8((value >> 8) & 0xff));
	dsp.data_w(u8(value & 0xff));
}

static void host_download_cmem_image(tms57002_device &dsp, const std::array<u32, 256> &cmem)
{
	dsp.pload_w(0);
	dsp.cload_w(0);
	for (u32 value : cmem)
		host_write_word32(dsp, value);
}

static void host_download_program_image(tms57002_device &dsp, const std::vector<u32> &program, u32 st0, u32 st1)
{
	dsp.pload_w(0);
	dsp.cload_w(1);
	host_write_word24(dsp, st0);
	host_write_word24(dsp, st1);
	for (u32 addr = 0; addr < 256; addr++)
		host_write_word24(dsp, addr < program.size() ? program[addr] : 0);
}

static void write_hex24(std::ostream &out, u32 value)
{
	out << "0x" << std::hex << std::nouppercase << std::setw(6) << std::setfill('0') << (value & 0x00ffffffU) << std::dec << std::setfill(' ');
}

static void load_data_space_bytes(const char *path, address_space &space)
{
	if (!path)
		return;

	const std::string resolved = legacy_asset_path(path);
	std::ifstream input(resolved, std::ios::binary);
	if (!input)
		throw emu_fatalerror("Unable to open %s (asset %s)", resolved, path);

	std::vector<u8> bytes((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
	for (std::size_t i = 0; i < bytes.size(); i++)
		space.write_byte(u32(i), bytes[i]);
}

static tms57002_device::debug_snapshot load_snapshot_from_log(const char *path)
{
	tms57002_device::debug_snapshot snapshot;

	for (const std::string &line : load_text_lines(path))
	{
		if (line.find("KPROP_DSPSNAP_PRE_CORE") != std::string::npos)
		{
			snapshot.sound_updates = parse_dec_u64(field_value(line, "SU="));
			snapshot.pc = parse_hex_u32(field_value(line, "PC="));
			snapshot.hpc = parse_hex_u32(field_value(line, "HPC="));
			snapshot.ca = parse_hex_u32(field_value(line, "CA="));
			snapshot.id = parse_hex_u32(field_value(line, "ID="));
			snapshot.ba0 = parse_hex_u32(field_value(line, "BA0="));
			snapshot.ba1 = parse_hex_u32(field_value(line, "BA1="));
			snapshot.rptc = parse_hex_u32(field_value(line, "RPTC="));
			snapshot.rptc_next = parse_hex_u32(field_value(line, "RPTN="));
			snapshot.sa = parse_hex_u32(field_value(line, "SA="));
			snapshot.hidx = parse_hex_u32(field_value(line, "HIDX="));
			snapshot.allow_update = parse_hex_u32(field_value(line, "ALLOW="));
			snapshot.st0 = parse_hex_u32(field_value(line, "ST0="));
			snapshot.st1 = parse_hex_u32(field_value(line, "ST1="));
			snapshot.sti = parse_hex_u32(field_value(line, "STI="));
			snapshot.aacc = parse_hex_u32(field_value(line, "AACC="));
			snapshot.macc = parse_hex_u64(field_value(line, "MACC="));
			snapshot.macc_read = parse_hex_u64(field_value(line, "MACCR="));
			snapshot.macc_write = parse_hex_u64(field_value(line, "MACCW="));
			snapshot.creg = parse_hex_u32(field_value(line, "CREG="));
		}
		else if (line.find("KPROP_DSPSNAP_PRE_XMEM") != std::string::npos)
		{
			snapshot.xoa = parse_hex_u32(field_value(line, "XOA="));
			snapshot.xba = parse_hex_u32(field_value(line, "XBA="));
			snapshot.xwr = parse_hex_u32(field_value(line, "XWR="));
			snapshot.xrd = parse_hex_u32(field_value(line, "XRD="));
			snapshot.txrd = parse_hex_u32(field_value(line, "TXRD="));
			snapshot.xm_adr = parse_hex_u32(field_value(line, "XMADR="));
			snapshot.xm_cycles = parse_hex_u32(field_value(line, "XMCYC="));
			snapshot.xm_fetches = parse_hex_u32(field_value(line, "XMFET="));
		}
		else if (line.find("KPROP_DSPSNAP_PRE_SERIAL") != std::string::npos)
		{
			snapshot.si = parse_hex_slash4(field_value(line, "SI="));
			snapshot.so = parse_hex_slash4(field_value(line, "SO="));
			snapshot.serial_input_latch = parse_hex_slash4(field_value(line, "LATCH="));
			snapshot.serial_input_active = parse_hex_slash4(field_value(line, "ACTIVE="));
			snapshot.serial_input_frame = parse_hex_slash4(field_value(line, "FRAME="));
			snapshot.serial_input_prev_frame = parse_hex_slash4(field_value(line, "PREV="));
			snapshot.serial_input_pending = parse_hex_slash4(field_value(line, "PEND="));
			snapshot.serial_input_valid = parse_hex_u32(field_value(line, "LVALID="));
			snapshot.serial_input_active_valid = parse_hex_u32(field_value(line, "AVALID="));
			snapshot.serial_input_prev_valid = parse_hex_u32(field_value(line, "PVALID="));
			snapshot.serial_input_pending_valid = parse_hex_u32(field_value(line, "IPEND="));
			snapshot.serial_output_pending_valid = parse_hex_u32(field_value(line, "OPEND="));
			snapshot.serial_frame_mode = parse_hex_u32(field_value(line, "FMODE="));
			snapshot.serial_input_timing_mode = parse_hex_u32(field_value(line, "ITMODE="));
			snapshot.serial_frame_clocks = parse_dec_int(field_value(line, "FCLKS="));
			snapshot.serial_exec_halfcycles = parse_dec_int(field_value(line, "EXECHC="));
			snapshot.serial_input_pending_halfcycle = parse_dec_int(field_value(line, "INHC="));
			snapshot.serial_output_pending_halfcycle = parse_dec_int(field_value(line, "OUTHC="));
			snapshot.sync_polarity_rising = parse_hex_u32(field_value(line, "SYNCPOL="));
			snapshot.serial_output_muted = parse_hex_u32(field_value(line, "MUTED="));
		}
		else if (line.find("KPROP_DSPSNAP_PRE_PREXFER") != std::string::npos)
		{
			snapshot.pending_pre_transfer = parse_dec_int(field_value(line, "TYPE="));
			snapshot.pending_pre_transfer_addr = parse_hex_u32(field_value(line, "ADDR="));
			snapshot.pending_pre_transfer_value = parse_hex_u32(field_value(line, "VALUE="));
			snapshot.update_counter_head = parse_hex_u32(field_value(line, "HEAD="));
			snapshot.update_counter_tail = parse_hex_u32(field_value(line, "TAIL="));
		}
		else if (line.find("KPROP_DSPSNAP_PRE_CMEM") != std::string::npos)
		{
			const std::size_t base_pos = line.find("BASE=");
			if (base_pos == std::string::npos)
				throw emu_fatalerror("Missing BASE in CMEM line: %s", line);
			std::stringstream ss(line.substr(base_pos));
			std::string token;
			if (!std::getline(ss, token, ','))
				throw emu_fatalerror("Bad CMEM line: %s", line);
			const u32 base = parse_hex_u32(token.substr(5));
			for (u32 i = 0; i < 16; i++)
			{
				if (!std::getline(ss, token, ','))
					throw emu_fatalerror("Short CMEM line: %s", line);
				snapshot.cmem[(base + i) & 0xff] = parse_hex_u32(token);
			}
		}
		else if (line.find("KPROP_DSPSNAP_PRE_DMEM0") != std::string::npos)
		{
			const std::size_t base_pos = line.find("BASE=");
			if (base_pos == std::string::npos)
				throw emu_fatalerror("Missing BASE in DMEM0 line: %s", line);
			std::stringstream ss(line.substr(base_pos));
			std::string token;
			if (!std::getline(ss, token, ','))
				throw emu_fatalerror("Bad DMEM0 line: %s", line);
			const u32 base = parse_hex_u32(token.substr(5));
			for (u32 i = 0; i < 16; i++)
			{
				if (!std::getline(ss, token, ','))
					throw emu_fatalerror("Short DMEM0 line: %s", line);
				snapshot.dmem0[(base + i) & 0xff] = parse_hex_u32(token);
			}
		}
		else if (line.find("KPROP_DSPSNAP_PRE_DMEM1") != std::string::npos)
		{
			const std::size_t base_pos = line.find("BASE=");
			if (base_pos == std::string::npos)
				throw emu_fatalerror("Missing BASE in DMEM1 line: %s", line);
			std::stringstream ss(line.substr(base_pos));
			std::string token;
			if (!std::getline(ss, token, ','))
				throw emu_fatalerror("Bad DMEM1 line: %s", line);
			const u32 base = parse_hex_u32(token.substr(5));
			for (u32 i = 0; i < 16; i++)
			{
				if (!std::getline(ss, token, ','))
					throw emu_fatalerror("Short DMEM1 line: %s", line);
				snapshot.dmem1[(base + i) & 0x1f] = parse_hex_u32(token);
			}
		}
		else if (line.find("KPROP_DSPSNAP_PRE_UPDATE") != std::string::npos)
		{
			const u32 idx = parse_hex_u32(field_value(line, "IDX="));
			if (idx >= 16)
				throw emu_fatalerror("Bad update index %u in %s", idx, line);
			snapshot.update_sa[idx] = parse_hex_u32(field_value(line, "SA="));
			snapshot.update[idx] = parse_hex_u32(field_value(line, "VAL="));
			snapshot.update_enqueue_su[idx] = parse_dec_u64(field_value(line, "ENQSU="));
			snapshot.update_read_delay_seen[idx] = parse_dec_int(field_value(line, "RDSEEN="));
		}
	}

	return snapshot;
}

static tms57002_device::debug_snapshot load_snapshot_for_su_from_lines(const std::vector<std::string> &lines, u64 su)
{
	tms57002_device::debug_snapshot snapshot;
	const std::string su_needle = util::string_format("SU=%llu,", (unsigned long long)su);
	bool found = false;

	for (std::size_t line_index = 0; line_index < lines.size(); line_index++)
	{
		const std::string &line = lines[line_index];
		if (!found)
		{
			if (line.find("KPROP_DSPSNAP_PRE_CORE") == std::string::npos || line.find("TAG=:dsp3") == std::string::npos || line.find(su_needle) == std::string::npos)
				continue;
			found = true;
		}
		else if (line.find("KPROP_DSPSNAP_PRE_CORE") != std::string::npos)
		{
			break;
		}

		if (line.find("KPROP_DSPSNAP_PRE_") == std::string::npos)
			continue;

		if (line.find("KPROP_DSPSNAP_PRE_CORE") != std::string::npos && line.find("TAG=:dsp3") != std::string::npos)
		{
			snapshot.sound_updates = parse_dec_u64(field_value(line, "SU="));
			snapshot.pc = parse_hex_u32(field_value(line, "PC="));
			snapshot.hpc = parse_hex_u32(field_value(line, "HPC="));
			snapshot.ca = parse_hex_u32(field_value(line, "CA="));
			snapshot.id = parse_hex_u32(field_value(line, "ID="));
			snapshot.ba0 = parse_hex_u32(field_value(line, "BA0="));
			snapshot.ba1 = parse_hex_u32(field_value(line, "BA1="));
			snapshot.rptc = parse_hex_u32(field_value(line, "RPTC="));
			snapshot.rptc_next = parse_hex_u32(field_value(line, "RPTN="));
			snapshot.sa = parse_hex_u32(field_value(line, "SA="));
			snapshot.hidx = parse_hex_u32(field_value(line, "HIDX="));
			snapshot.allow_update = parse_hex_u32(field_value(line, "ALLOW="));
			snapshot.st0 = parse_hex_u32(field_value(line, "ST0="));
			snapshot.st1 = parse_hex_u32(field_value(line, "ST1="));
			snapshot.sti = parse_hex_u32(field_value(line, "STI="));
			snapshot.aacc = parse_hex_u32(field_value(line, "AACC="));
			snapshot.macc = parse_hex_u64(field_value(line, "MACC="));
			snapshot.macc_read = parse_hex_u64(field_value(line, "MACCR="));
			snapshot.macc_write = parse_hex_u64(field_value(line, "MACCW="));
			snapshot.creg = parse_hex_u32(field_value(line, "CREG="));
		}
		else if (line.find("KPROP_DSPSNAP_PRE_XMEM") != std::string::npos && line.find("TAG=:dsp3") != std::string::npos)
		{
			snapshot.xoa = parse_hex_u32(field_value(line, "XOA="));
			snapshot.xba = parse_hex_u32(field_value(line, "XBA="));
			snapshot.xwr = parse_hex_u32(field_value(line, "XWR="));
			snapshot.xrd = parse_hex_u32(field_value(line, "XRD="));
			snapshot.txrd = parse_hex_u32(field_value(line, "TXRD="));
			snapshot.xm_adr = parse_hex_u32(field_value(line, "XMADR="));
			snapshot.xm_cycles = parse_hex_u32(field_value(line, "XMCYC="));
			snapshot.xm_fetches = parse_hex_u32(field_value(line, "XMFET="));
		}
		else if (line.find("KPROP_DSPSNAP_PRE_SERIAL") != std::string::npos && line.find("TAG=:dsp3") != std::string::npos)
		{
			snapshot.si = parse_hex_slash4(field_value(line, "SI="));
			snapshot.so = parse_hex_slash4(field_value(line, "SO="));
			snapshot.serial_input_latch = parse_hex_slash4(field_value(line, "LATCH="));
			snapshot.serial_input_valid = parse_hex_u32(field_value(line, "LVALID="));
			snapshot.serial_input_active = parse_hex_slash4(field_value(line, "ACTIVE="));
			snapshot.serial_input_active_valid = parse_hex_u32(field_value(line, "AVALID="));
			snapshot.serial_input_frame = parse_hex_slash4(field_value(line, "FRAME="));
			snapshot.serial_input_prev_frame = parse_hex_slash4(field_value(line, "PREV="));
			snapshot.serial_input_prev_valid = parse_hex_u32(field_value(line, "PVALID="));
			snapshot.serial_input_pending = parse_hex_slash4(field_value(line, "PEND="));
			snapshot.serial_input_pending_valid = parse_hex_u32(field_value(line, "IPEND="));
			snapshot.serial_output_pending_valid = parse_hex_u32(field_value(line, "OPEND="));
			snapshot.serial_frame_mode = parse_dec_int(field_value(line, "FMODE="));
			snapshot.serial_input_timing_mode = parse_dec_int(field_value(line, "ITMODE="));
			snapshot.serial_frame_clocks = parse_dec_int(field_value(line, "FCLKS="));
			snapshot.serial_exec_halfcycles = parse_dec_int(field_value(line, "EXECHC="));
			snapshot.serial_input_pending_halfcycle = parse_dec_int(field_value(line, "INHC="));
			snapshot.serial_output_pending_halfcycle = parse_dec_int(field_value(line, "OUTHC="));
			snapshot.sync_polarity_rising = parse_dec_int(field_value(line, "SYNCPOL="));
			snapshot.serial_output_muted = parse_dec_int(field_value(line, "MUTED="));
		}
		else if (line.find("KPROP_DSPSNAP_PRE_CMEM") != std::string::npos)
		{
			const u32 base = parse_hex_u32(field_value(line, "BASE="));
			const std::size_t pos = line.find("BASE=");
			const std::size_t comma = line.find(',', pos);
			std::stringstream ss(line.substr(comma + 1));
			for (u32 i = 0; i < 16; i++)
			{
				std::string part;
				std::getline(ss, part, ',');
				snapshot.cmem[base + i] = parse_hex_u32(part);
			}
		}
		else if (line.find("KPROP_DSPSNAP_PRE_DMEM0") != std::string::npos)
		{
			const u32 base = parse_hex_u32(field_value(line, "BASE="));
			const std::size_t pos = line.find("BASE=");
			const std::size_t comma = line.find(',', pos);
			std::stringstream ss(line.substr(comma + 1));
			for (u32 i = 0; i < 16; i++)
			{
				std::string part;
				std::getline(ss, part, ',');
				snapshot.dmem0[base + i] = parse_hex_u32(part);
			}
		}
		else if (line.find("KPROP_DSPSNAP_PRE_DMEM1") != std::string::npos)
		{
			const u32 base = parse_hex_u32(field_value(line, "BASE="));
			const std::size_t pos = line.find("BASE=");
			const std::size_t comma = line.find(',', pos);
			std::stringstream ss(line.substr(comma + 1));
			for (u32 i = 0; i < 16; i++)
			{
				std::string part;
				std::getline(ss, part, ',');
				snapshot.dmem1[base + i] = parse_hex_u32(part);
			}
		}
	}

	if (!found)
		throw emu_fatalerror("Unable to find PRE snapshot for SU=%llu", (unsigned long long)su);

	return snapshot;
}

struct exec_post_expectation
{
	u8 executed_pc = 0;
	u8 ca = 0;
	u8 id = 0;
	u32 st1 = 0;
	u32 aacc = 0;
	u64 macc = 0;
	u64 maccr = 0;
	u64 maccw = 0;
	u32 creg = 0;
	u32 d00 = 0;
	u32 ddc = 0;
	u32 ddd = 0;
	u32 dde = 0;
	u32 ddf = 0;
	u32 xrd = 0;
	u32 txrd = 0;
	u32 d56 = 0;
	u32 d5d = 0;
	u32 d5f = 0;
	u32 so0 = 0;
	u32 so1 = 0;
	bool has_extended = false;
};

static std::vector<exec_post_expectation> load_exec_post_expectations(const char *path)
{
	std::vector<exec_post_expectation> expectations;
	for (const std::string &line : load_text_lines(path))
	{
		if (line.find("KPROP_DSPEXEC_POST") == std::string::npos)
			continue;

		exec_post_expectation exp;
		exp.executed_pc = parse_hex_u32(field_value(line, "PC="));
		exp.ca = parse_hex_u32(field_value(line, "CA="));
		exp.id = parse_hex_u32(field_value(line, "ID="));
		exp.st1 = parse_hex_u32(field_value(line, "ST1="));
		exp.aacc = parse_hex_u32(field_value(line, "AACC="));
		exp.macc = parse_hex_u64(field_value(line, "MACC="));
		exp.maccr = parse_hex_u64(field_value(line, "MACCR="));
		exp.maccw = parse_hex_u64(field_value(line, "MACCW="));
		exp.creg = parse_hex_u32(field_value(line, "CREG="));
		exp.has_extended = (line.find("D00=") != std::string::npos);
		if (exp.has_extended)
		{
			exp.d00 = parse_hex_u32(field_value(line, "D00="));
			exp.ddc = parse_hex_u32(field_value(line, "DDC="));
			exp.ddd = parse_hex_u32(field_value(line, "DDD="));
			exp.dde = parse_hex_u32(field_value(line, "DDE="));
			exp.ddf = parse_hex_u32(field_value(line, "DDF="));
			exp.xrd = parse_hex_u32(field_value(line, "XRD="));
			exp.txrd = parse_hex_u32(field_value(line, "TXRD="));
		}
		exp.d56 = parse_hex_u32(field_value(line, "D56="));
		exp.d5d = parse_hex_u32(field_value(line, "D5D="));
		exp.d5f = parse_hex_u32(field_value(line, "D5F="));
		const auto so = parse_hex_slash4(field_value(line, "SO="));
		exp.so0 = so[0];
		exp.so1 = so[1];
		expectations.push_back(exp);
	}

	if (expectations.empty())
		throw emu_fatalerror("No KPROP_DSPEXEC_POST lines found in %s", path);

	return expectations;
}

} // anonymous namespace

void tms57002test_state::test_macc_output_clipping()
{
	static constexpr u64 positive_overflow = 0x1000000000000ULL;
	static constexpr u64 negative_overflow = 0xf000000000000ULL;
	static constexpr u64 in_range = 0x0123456789abcULL;

	struct round_case
	{
		int rnd;
		u64 pos_expected;
		u64 neg_expected;
	};

	// The overflow rail is masked by the active RND width.  The negative
	// rail already has all discarded low bits clear, so it is unchanged.
	// These expectations are backed by the 436/436 standalone DOMH sweep.
	static constexpr round_case manual_cases[] = {
		{ 0, 0x7fffffffffffULL, 0x800000000000ULL },
		{ 1, 0x7fffffff0000ULL, 0x800000000000ULL },
		{ 2, 0x7fffff000000ULL, 0x800000000000ULL },
		{ 3, 0x7ffff0000000ULL, 0x800000000000ULL },
		{ 4, 0x7fff00000000ULL, 0x800000000000ULL },
	};

	auto eval = m_dsp->debug_eval_macc_output(in_range, 0, 0, tms57002_device::debug_macc_clip_mode::rounded);
	check_false("in-range keeps MOV clear", eval.mov);
	check_equal("in-range value unchanged", eval.value, sx52(in_range));

	for (const auto &tc : manual_cases)
	{
		char name[96];

		auto pos = m_dsp->debug_eval_macc_output(positive_overflow, 0, tc.rnd, tms57002_device::debug_macc_clip_mode::rounded);
		std::snprintf(name, sizeof(name), "SMOM positive overflow MOV rnd=%d", tc.rnd);
		check_true(name, pos.mov);
		std::snprintf(name, sizeof(name), "SMOM positive overflow value rnd=%d", tc.rnd);
		check_equal(name, pos.value, sx48(tc.pos_expected));

		auto neg = m_dsp->debug_eval_macc_output(negative_overflow, 0, tc.rnd, tms57002_device::debug_macc_clip_mode::rounded);
		std::snprintf(name, sizeof(name), "SMOM negative overflow MOV rnd=%d", tc.rnd);
		check_true(name, neg.mov);
		std::snprintf(name, sizeof(name), "SMOM negative overflow value rnd=%d", tc.rnd);
		check_equal(name, neg.value, sx48(tc.neg_expected));
	}
}

void tms57002test_state::test_domh_movm_rounding_oracle_boundaries()
{
	// Exact standalone-silicon programs and steady-state words from the
	// 2026-05-31 final-domh MOVM sweep.  These two points exposed the MOVM
	// saturation rule: the positive rail is masked by the active RND width.
	static constexpr u32 ST0_ORACLE = 0x000680;
	static constexpr u32 RAW_POSITIVE_OVERFLOW = 0x7fffffff;
	static constexpr u32 OP_LMHC_C00__SMOM = 0xcc0500;
	static constexpr u32 OP_NOP = 0x020800;
	static constexpr u32 OP_DOMH_SO0_L = 0x010000;
	static constexpr u32 OP_DOMH_SO0_R = 0x010800;
	static constexpr u32 OP_DOMH_SO1_L = 0x011000;
	static constexpr u32 OP_DOMH_SO1_R = 0x011800;
	static constexpr u32 OP_IDLE = 0xfc4000;

	struct oracle_case
	{
		const char *name;
		u32 rnd_opcode;
		u32 expected_word;
	};

	static constexpr oracle_case cases[] = {
		{ "MOVM RND20", 0x035800, 0x7ffff0 },
		{ "MOVM RND16", 0x036000, 0x7fff00 },
	};

	for (const oracle_case &test : cases)
	{
		const std::array<u32, 8> program{
			OP_LMHC_C00__SMOM,
			OP_NOP,
			test.rnd_opcode,
			OP_DOMH_SO0_L,
			OP_DOMH_SO0_R,
			OP_DOMH_SO1_L,
			OP_DOMH_SO1_R,
			OP_IDLE,
		};

		m_dsp->debug_load_program(program.data(), u32(program.size()), ST0_ORACLE, 0);
		m_dsp->debug_write_cmem(0x00, RAW_POSITIVE_OVERFLOW);
		m_dsp->debug_set_exec_state(0, 0, 0, 0, 0, ST0_ORACLE, 0, 0);
		m_dsp->debug_set_accumulators(0, 0, 0, 0, 0);
		m_dsp->set_serial_frame_model(true);
		m_dsp->set_serial_frame_clocks(512);
		m_dsp->set_sync_polarity(1);
		for (int frame = 0; frame < 3; frame++)
			m_dsp->debug_run_sample_frame({ 0, 0, 0, 0 }, 16);

		for (int lane = 0; lane < 4; lane++)
		{
			char label[96];
			std::snprintf(label, sizeof(label), "%s DOMH SO%d", test.name, lane);
			check_equal(label, m_dsp->debug_serial_output_register(lane), test.expected_word);
		}
	}
}

void tms57002test_state::test_macc_output_unrounded_exceptions()
{
	static constexpr u64 positive_overflow = 0x1000000000000ULL;
	static constexpr u64 negative_overflow = 0xf000000000000ULL;

	for (int rnd = 0; rnd <= 4; rnd++)
	{
		char name[96];

		auto pos = m_dsp->debug_eval_macc_output(positive_overflow, 0, rnd, tms57002_device::debug_macc_clip_mode::unrounded);
		std::snprintf(name, sizeof(name), "DOMH positive overflow MOV rnd=%d", rnd);
		check_true(name, pos.mov);
		std::snprintf(name, sizeof(name), "DOMH positive overflow value rnd=%d", rnd);
		check_equal(name, pos.value, sx48(0x7fffffffffffULL));

		auto neg = m_dsp->debug_eval_macc_output(negative_overflow, 0, rnd, tms57002_device::debug_macc_clip_mode::unrounded);
		std::snprintf(name, sizeof(name), "DOMH negative overflow MOV rnd=%d", rnd);
		check_true(name, neg.mov);
		std::snprintf(name, sizeof(name), "DOMH negative overflow value rnd=%d", rnd);
		check_equal(name, neg.value, sx48(0x800000000000ULL));
	}
}

void tms57002test_state::test_aovm_neg_abs_int32_min()
{
	// Silicon oracle D0C-06 (2026-07-26): NEG and ABS both treat AACC as
	// signed 32-bit, set AOV on INT32_MIN, and clamp to INT32_MAX when
	// SAOM/AOVM is enabled.  RAOM preserves the wrapped 0x80000000 result.
	//
	// Keep this as a tiny instruction-level regression independent of the
	// retained full D0-C capture.  The opcodes below are the exact assembler
	// encodings used by that silicon program.
	static constexpr u32 ST0 = 0x01c083;
	static constexpr u32 OP_RAOM = 0x01e000;
	static constexpr u32 OP_SAOM = 0x01e800;
	static constexpr u32 OP_RAOV = 0x01c000;
	static constexpr u32 OP_LACC_C22 = 0x480522;
	static constexpr u32 OP_NEG = 0x080000;
	static constexpr u32 OP_ABS = 0x040000;
	static constexpr u32 OP_SACD_DE0 = 0x0011e0;
	static constexpr u32 OP_IDLE = 0xfc4000;

	struct boundary_case
	{
		const char *name;
		u32 mode_op;
		u32 arithmetic_op;
		u32 coefficient;
		u32 expected_dmem;
		bool expected_aov;
		bool expected_aovm;
	};

	static constexpr boundary_case cases[] = {
		{ "RAOM NEG INT32_MIN wraps", OP_RAOM, OP_NEG, 0x80000000, 0x800000, true, false },
		{ "SAOM NEG INT32_MIN clamps", OP_SAOM, OP_NEG, 0x80000000, 0x7fffff, true, true },
		{ "RAOM ABS INT32_MIN wraps", OP_RAOM, OP_ABS, 0x80000000, 0x800000, true, false },
		{ "SAOM ABS INT32_MIN clamps", OP_SAOM, OP_ABS, 0x80000000, 0x7fffff, true, true },
		{ "RAOM NEG ordinary positive", OP_RAOM, OP_NEG, 0x00000100, 0xffffff, false, false },
		{ "SAOM NEG ordinary positive", OP_SAOM, OP_NEG, 0x00000100, 0xffffff, false, true },
		{ "RAOM ABS ordinary negative", OP_RAOM, OP_ABS, 0xffffff00, 0x000001, false, false },
		{ "SAOM ABS ordinary negative", OP_SAOM, OP_ABS, 0xffffff00, 0x000001, false, true },
	};

	m_dsp->set_abs_saturate(false);
	for (const boundary_case &test : cases)
	{
		const std::array<u32, 6> program{
			test.mode_op,
			OP_RAOV,
			OP_LACC_C22,
			test.arithmetic_op,
			OP_SACD_DE0,
			OP_IDLE,
		};
		m_dsp->debug_load_program(program.data(), u32(program.size()), ST0, 0);
		m_dsp->debug_write_cmem(0x22, test.coefficient);
		m_dsp->debug_write_dmem0(0xe0, 0x55aa55);
		m_dsp->debug_set_exec_state(0, 0, 0, 0, 0, ST0, 0, 0);
		m_dsp->debug_run_cycles(16);
		const tms57002_device::debug_snapshot state = m_dsp->debug_capture_snapshot();

		char label[96];
		std::snprintf(label, sizeof(label), "%s result", test.name);
		check_equal(label, m_dsp->dmem0_value(0xe0), test.expected_dmem);
		std::snprintf(label, sizeof(label), "%s AOV", test.name);
		if (test.expected_aov)
			check_true(label, bool(state.st1 & tms57002_device::jit_st1_aov()));
		else
			check_false(label, bool(state.st1 & tms57002_device::jit_st1_aov()));
		std::snprintf(label, sizeof(label), "%s AOVM", test.name);
		if (test.expected_aovm)
			check_true(label, bool(state.st1 & tms57002_device::jit_st1_aovm()));
		else
			check_false(label, bool(state.st1 & tms57002_device::jit_st1_aovm()));
	}
}

void tms57002test_state::test_mac_a_d_full_width_port()
{
	static constexpr u32 ST0 = 0x01c083;
	static constexpr u32 OP_MAC_A_D15 = 0x00940115;

	std::array<u32, 2> program{ OP_MAC_A_D15, 0 };
	m_dsp->debug_load_program(program.data(), u32(program.size()), ST0, 0);
	m_dsp->debug_set_exec_state(0, 0, 0, 0, 0, ST0, 0, 0);
	m_dsp->debug_write_dmem0(0x15, 0xffff00);
	m_dsp->debug_set_accumulators(0x7fffffff, 0x100000000ULL, 0x100000000ULL, 0x100000000ULL, 0);
	m_dsp->debug_run_cycles(1);
	const auto state = m_dsp->debug_capture_snapshot();
	check_equal("MAC A,D keeps full-width AACC on C input", state.macc, 2);
	check_equal("MAC A,D writes full-width AACC to CREG", state.creg, 0x7fffffff);
}

void tms57002test_state::test_multiplier_a_port_drops_low_byte()
{
	static constexpr u32 ST0 = 0x01c083;
	static constexpr u32 OP_MPY_C02_A = 0x00880502;
	static constexpr u32 OP_MAC_C02_A = 0x00980502;
	static constexpr u32 OP_MACS_C02_A = 0x00b80502;

	struct test_case
	{
		const char *name;
		u32 opcode;
		u32 aacc;
		s64 macc_before;
		s64 expected;
	};
	static constexpr test_case cases[] = {
		{ "MPY C,A positive low byte", OP_MPY_C02_A, 0x000001ff, 0, 512 },
		{ "MPY C,A negative low byte", OP_MPY_C02_A, 0xffffff01, 0, -512 },
		{ "MAC C,A positive low byte", OP_MAC_C02_A, 0x000001ff, 37, 549 },
		{ "MACS C,A positive low byte", OP_MACS_C02_A, 0x000001ff, 37, 1061 },
	};

	for (const test_case &tc : cases)
	{
		std::array<u32, 2> program{ tc.opcode, 0 };
		m_dsp->debug_load_program(program.data(), u32(program.size()), ST0, 0);
		m_dsp->debug_set_exec_state(0, 0, 0, 0, 0, ST0, 0, 0);
		m_dsp->debug_write_cmem(0x02, 0x00010000);
		m_dsp->debug_set_accumulators(tc.aacc, u64(tc.macc_before), u64(tc.macc_before), u64(tc.macc_before), 0);
		m_dsp->debug_run_cycles(1);
		check_equal(tc.name, s64(m_dsp->debug_macc()), tc.expected);
	}
}

void tms57002test_state::test_sequential_pc_wrap_halts_until_sync()
{
	// A full 256-word program with no explicit IDLE consumes exactly one
	// silicon sample pass.  Sequential 0xff -> 0x00 therefore waits for the
	// next SYNC; it must not begin a second pass from the remaining MAME cycle
	// budget.  Zero words are inert here, so this isolates the control rule.
	std::array<u32, 0x100> program{};
	m_dsp->debug_load_program(program.data(), u32(program.size()), 0, 0);
	m_dsp->debug_begin_sample_frame({ 0, 0, 0, 0 });
	m_dsp->debug_run_cycles(256);

	check_equal("sequential PC wrap returns to PC 0", m_dsp->debug_capture_snapshot().pc, 0);
	check_equal("sequential PC wrap halts until SYNC", m_dsp->jit_is_idle(), true);

	m_dsp->debug_begin_sample_frame({ 0, 0, 0, 0 });
	check_equal("next SYNC releases PC-wrap halt", m_dsp->jit_is_idle(), false);
	m_dsp->debug_run_cycles(1);
	check_equal("next sample resumes at PC 0", m_dsp->debug_capture_snapshot().pc, 1);

	// The halt is tied to sequential carry, not merely to observing PC 0.
	// Category-3 B 0 at PC ff takes the S_BRANCH path and must remain live.
	program[0xff] = 0x00fe4000;
	m_dsp->debug_load_program(program.data(), u32(program.size()), 0, 0);
	m_dsp->debug_begin_sample_frame({ 0, 0, 0, 0 });
	m_dsp->debug_run_cycles(256);
	check_equal("explicit branch target is PC 0", m_dsp->debug_capture_snapshot().pc, 0);
	check_equal("explicit branch to PC 0 remains live", m_dsp->jit_is_idle(), false);
}

void tms57002test_state::test_smld_raw_low_port()
{
	static constexpr u32 ST0 = 0x01c083;
	static constexpr u32 OP_SMLD_DE1 = 0x000021e1;

	static constexpr u64 raw_macc = 0x123456789abcdULL;
	for (u32 sfmo = 0; sfmo < 4; sfmo++)
	{
		const u32 st1 = sfmo << 11;
		std::array<u32, 2> program{ OP_SMLD_DE1, 0 };
		m_dsp->debug_load_program(program.data(), u32(program.size()), ST0, st1);
		m_dsp->debug_set_exec_state(0, 0, 0, 0, 0, ST0, st1, 0);
		m_dsp->debug_set_accumulators(0, raw_macc, raw_macc, raw_macc, 0);
		m_dsp->debug_run_cycles(1);

		char name[80];
		std::snprintf(name, sizeof(name), "SMLD raw low field sfmo=%u", sfmo);
		check_equal(name, m_dsp->dmem0_value(0xe1), 0x89abcd);
	}
}

void tms57002test_state::test_mac_a_d_and_smld_port_semantics()
{
	test_mac_a_d_full_width_port();
	test_smld_raw_low_port();
}

void tms57002test_state::test_domh_sfmo2_trace_cases()
{
	static constexpr u32 ST0_DSP3 = 0x018082;
	static constexpr u32 ST1_DSP3 = 0x000820;
	static constexpr u32 OPCODE_DOMH_SO0L = 0x00010000;
	static constexpr u32 OPCODE_DOMH_SO0R = 0x00010800;

	struct trace_case
	{
		const char *name;
		u32 opcode;
		u64 macc;
		u64 macc_read;
		u64 macc_write;
		int so_index;
		u32 so_expected;
		bool mov_expected;
	};

	static constexpr trace_case cases[] = {
		// These use the exact A02 controlled trace states, but account for execute_run()
		// advancing macc_read <- macc_write before the DOMH instruction executes.
		{ "DOMH sfmo2 impulse FE output", OPCODE_DOMH_SO0L, 0x012ff47f36cbaULL, 0xffffbb52e500895aULL, 0x00133b602aec38ULL, 0, 0x004ced80, false },
		{ "DOMH sfmo2 impulse FF output", OPCODE_DOMH_SO0R, 0x012ff47f36cbaULL, 0x00133b602aec38ULL, 0x012ff47f36cbaULL, 1, 0x004bfd1f, false },
		{ "DOMH sfmo2 sine FE output",    OPCODE_DOMH_SO0L, 0x020ffc0d97f64ULL, 0xffff88b4c600ee96ULL, 0x002168225e40cdULL, 0, 0x007fffff, true  },
		{ "DOMH sfmo2 sine FF output",    OPCODE_DOMH_SO0R, 0x020ffc0d97f64ULL, 0x002168225e40cdULL,   0x020ffc0d97f64ULL, 1, 0x007fffff, true  },
	};

	for (const auto &tc : cases)
	{
		std::array<u32, 2> program{ tc.opcode, 0 };
		m_dsp->debug_load_program(program.data(), u32(program.size()), ST0_DSP3, ST1_DSP3);
		m_dsp->debug_set_exec_state(0x00, 0x00, 0x00, 0x00, 0x00, ST0_DSP3, ST1_DSP3, 0);
		m_dsp->debug_set_accumulators(0, tc.macc, tc.macc_read, tc.macc_write, 0);
		m_dsp->debug_run_cycles(1);

		char name[96];
		std::snprintf(name, sizeof(name), "%s MOV", tc.name);
		const bool mov = bool(m_dsp->state_int(TMS57002_ST1) & 0x000040);
		if (tc.mov_expected) check_true(name, mov);
		else check_false(name, mov);

		std::snprintf(name, sizeof(name), "%s SO", tc.name);
		check_equal(name, m_dsp->debug_serial_output_register(tc.so_index), tc.so_expected);
	}
}

void tms57002test_state::test_dsp3_a02_f8_ff_tail_chain()
{
	static constexpr u32 ST0_DSP3 = 0x018082;
	static constexpr u32 ST1_START = 0x000020;
	static constexpr u32 OPCODE_F8 = 0x00880502; // mpy c(02),a
	static constexpr u32 OPCODE_F9 = 0x00901b5f; // mac d(5f),c*+ ; smhd d(5f)
	static constexpr u32 OPCODE_FA = 0x00fcc0c0; // lcak c0
	static constexpr u32 OPCODE_FB = 0x0000195d; // smhd d(5d)
	static constexpr u32 OPCODE_FC = 0x00840309; // mpy d(09),c*+
	static constexpr u32 OPCODE_FD = 0x00870b5d; // mpy d(5d),c*+ ; sfmo 2
	static constexpr u32 OPCODE_FE = 0x00010000; // domh so0_l
	static constexpr u32 OPCODE_FF = 0x00010800; // domh so0_r

	std::array<u32, 0x100> program{};
	program.fill(0);
	program[0xf8] = OPCODE_F8;
	program[0xf9] = OPCODE_F9;
	program[0xfa] = OPCODE_FA;
	program[0xfb] = OPCODE_FB;
	program[0xfc] = OPCODE_FC;
	program[0xfd] = OPCODE_FD;
	program[0xfe] = OPCODE_FE;
	program[0xff] = OPCODE_FF;

	struct tail_case
	{
		const char *tag;
		u8 ba0;
		u8 ba1;
		u32 d09;
		u32 d5d;
		u32 d5f;
		u32 aacc_after_f8;
		u64 macc_start;
		u64 macc_after_fc;
		u64 macc_after_fd;
		u64 macc_after_fe;
		u64 maccr_start;
		u64 maccw_start;
		u32 d_store;
		u32 so0_after_fe;
		u32 so1_after_ff;
		u32 st1_after_fe;
		u32 st1_after_ff;
	};

	static constexpr tail_case cases[] = {
		{
			"sine",
			0x6b, 0x95,
			0x0024c1, 0xe220a8, 0xeef68f,
			0xfff5f0b3,
			0xffff88b4c600ee96ULL,
			0x002168225e40cdULL,
			0x020ffc0d97f64ULL,
			0x020ffc0d97f64ULL,
			0xfffffffba1e49512ULL,
			0xfffffffcf0b3341bULL,
			0x88b4c6,
			0x007fffff,
			0x007fffff,
			0x000860,
			0x000860,
		},
		{
			"impulse",
			0x6d, 0x93,
			0x02679b, 0xda97ff, 0xe1dd00,
			0xfff5f0b3,
			0xffffbb52e500895aULL,
			0x00133b602aec38ULL,
			0x012ff47f36cbaULL,
			0x012ff47f36cbaULL,
			0xfffffffba1e49512ULL,
			0xfffffffcf0b3341bULL,
			0xbb52e5,
			0x004ced80,
			0x004bfd1f,
			0x000820,
			0x000820,
		},
	};

	for (const auto &tc : cases)
	{
		char name[128];
		const u8 phys_d09 = u8(tc.ba0 + 0x09);
		const u8 phys_d56 = u8(tc.ba0 + 0x56);
		const u8 phys_d5d = u8(tc.ba0 + 0x5d);
		const u8 phys_d5f = u8(tc.ba0 + 0x5f);

		m_dsp->debug_load_program(program.data(), u32(program.size()), ST0_DSP3, ST1_START);
		m_dsp->debug_write_cmem(0x02, 0x7fffffff);
		m_dsp->debug_write_cmem(0x94, 0x00000000);
		m_dsp->debug_write_cmem(0xc0, 0xde36bc8f);
		m_dsp->debug_write_cmem(0xc1, 0xea05d421);
		m_dsp->debug_write_dmem0(phys_d09, tc.d09);
		m_dsp->debug_write_dmem0(phys_d5d, tc.d5d);
		m_dsp->debug_write_dmem0(phys_d5f, tc.d5f);
		m_dsp->debug_set_exec_state(0xf8, 0x94, 0x59, tc.ba0, tc.ba1, ST0_DSP3, ST1_START, 0);
		m_dsp->debug_set_accumulators(0xffff0000U, tc.macc_start, tc.maccr_start, tc.maccw_start, 0x7fffffff);

		m_dsp->debug_run_cycles(1); // f8
		std::snprintf(name, sizeof(name), "dsp3 a02 %s f8 pc", tc.tag);
		check_equal(name, m_dsp->state_int(TMS57002_PC), 0xf9);
		std::snprintf(name, sizeof(name), "dsp3 a02 %s f8 aacc", tc.tag);
		check_equal(name, u32(m_dsp->debug_aacc()), tc.aacc_after_f8);
		std::snprintf(name, sizeof(name), "dsp3 a02 %s f8 macc", tc.tag);
		check_equal(name, s64(m_dsp->debug_macc()), s64(tc.macc_start));
		std::snprintf(name, sizeof(name), "dsp3 a02 %s f8 maccr", tc.tag);
		check_equal(name, s64(m_dsp->debug_macc_read()), s64(tc.maccw_start));
		std::snprintf(name, sizeof(name), "dsp3 a02 %s f8 maccw", tc.tag);
		check_equal(name, s64(m_dsp->debug_macc_write()), s64(tc.macc_start));
		std::snprintf(name, sizeof(name), "dsp3 a02 %s f8 creg", tc.tag);
		check_equal(name, m_dsp->debug_creg(), 0x7fffffffU);

		m_dsp->debug_run_cycles(1); // f9
		std::snprintf(name, sizeof(name), "dsp3 a02 %s f9 pc", tc.tag);
		check_equal(name, m_dsp->state_int(TMS57002_PC), 0xfa);
		std::snprintf(name, sizeof(name), "dsp3 a02 %s f9 macc", tc.tag);
		check_equal(name, s64(m_dsp->debug_macc()), s64(tc.macc_start));
		std::snprintf(name, sizeof(name), "dsp3 a02 %s f9 maccr", tc.tag);
		check_equal(name, s64(m_dsp->debug_macc_read()), s64(tc.macc_start));
		std::snprintf(name, sizeof(name), "dsp3 a02 %s f9 maccw", tc.tag);
		check_equal(name, s64(m_dsp->debug_macc_write()), s64(tc.macc_start));
		std::snprintf(name, sizeof(name), "dsp3 a02 %s f9 creg", tc.tag);
		check_equal(name, m_dsp->debug_creg(), tc.aacc_after_f8);
		std::snprintf(name, sizeof(name), "dsp3 a02 %s f9 d5f", tc.tag);
		check_equal(name, m_dsp->dmem0_value(phys_d5f), tc.d_store);

		m_dsp->debug_run_cycles(1); // fa
		std::snprintf(name, sizeof(name), "dsp3 a02 %s fa pc", tc.tag);
		check_equal(name, m_dsp->state_int(TMS57002_PC), 0xfb);
		std::snprintf(name, sizeof(name), "dsp3 a02 %s fa ca", tc.tag);
		check_equal(name, m_dsp->state_int(TMS57002_CA), 0xc0);
		std::snprintf(name, sizeof(name), "dsp3 a02 %s fa creg", tc.tag);
		check_equal(name, m_dsp->debug_creg(), tc.aacc_after_f8);

		m_dsp->debug_run_cycles(1); // fb
		std::snprintf(name, sizeof(name), "dsp3 a02 %s fb pc", tc.tag);
		check_equal(name, m_dsp->state_int(TMS57002_PC), 0xfc);
		std::snprintf(name, sizeof(name), "dsp3 a02 %s fb d5d", tc.tag);
		check_equal(name, m_dsp->dmem0_value(phys_d5d), tc.d_store);

		m_dsp->debug_run_cycles(1); // fc
		std::snprintf(name, sizeof(name), "dsp3 a02 %s fc pc", tc.tag);
		check_equal(name, m_dsp->state_int(TMS57002_PC), 0xfd);
		std::snprintf(name, sizeof(name), "dsp3 a02 %s fc ca", tc.tag);
		check_equal(name, m_dsp->state_int(TMS57002_CA), 0xc1);
		std::snprintf(name, sizeof(name), "dsp3 a02 %s fc macc", tc.tag);
		check_equal(name, s64(m_dsp->debug_macc()), s64(tc.macc_after_fc));
		std::snprintf(name, sizeof(name), "dsp3 a02 %s fc creg", tc.tag);
		check_equal(name, m_dsp->debug_creg(), 0xdc27b873U);
		std::snprintf(name, sizeof(name), "dsp3 a02 %s fc d56", tc.tag);
		check_equal(name, m_dsp->dmem0_value(phys_d56), tc.d_store);

		m_dsp->debug_run_cycles(1); // fd
		std::snprintf(name, sizeof(name), "dsp3 a02 %s fd pc", tc.tag);
		check_equal(name, m_dsp->state_int(TMS57002_PC), 0xfe);
		std::snprintf(name, sizeof(name), "dsp3 a02 %s fd ca", tc.tag);
		check_equal(name, m_dsp->state_int(TMS57002_CA), 0xc2);
		std::snprintf(name, sizeof(name), "dsp3 a02 %s fd macc", tc.tag);
		check_equal(name, s64(m_dsp->debug_macc()), s64(tc.macc_after_fd));
		std::snprintf(name, sizeof(name), "dsp3 a02 %s fd maccr", tc.tag);
		check_equal(name, s64(m_dsp->debug_macc_read()), s64(tc.macc_start));
		std::snprintf(name, sizeof(name), "dsp3 a02 %s fd maccw", tc.tag);
		check_equal(name, s64(m_dsp->debug_macc_write()), s64(tc.macc_after_fc));
		std::snprintf(name, sizeof(name), "dsp3 a02 %s fd creg", tc.tag);
		check_equal(name, m_dsp->debug_creg(), 0xdc97b953U);
		std::snprintf(name, sizeof(name), "dsp3 a02 %s fd st1", tc.tag);
		check_equal(name, m_dsp->state_int(TMS57002_ST1), 0x000820);

		m_dsp->debug_run_cycles(1); // fe
		std::snprintf(name, sizeof(name), "dsp3 a02 %s fe pc", tc.tag);
		check_equal(name, m_dsp->state_int(TMS57002_PC), 0xff);
		std::snprintf(name, sizeof(name), "dsp3 a02 %s fe macc", tc.tag);
		check_equal(name, s64(m_dsp->debug_macc()), s64(tc.macc_after_fe));
		std::snprintf(name, sizeof(name), "dsp3 a02 %s fe maccr", tc.tag);
		check_equal(name, s64(m_dsp->debug_macc_read()), s64(tc.macc_after_fc));
		std::snprintf(name, sizeof(name), "dsp3 a02 %s fe maccw", tc.tag);
		check_equal(name, s64(m_dsp->debug_macc_write()), s64(tc.macc_after_fe));
		std::snprintf(name, sizeof(name), "dsp3 a02 %s fe st1", tc.tag);
		check_equal(name, m_dsp->state_int(TMS57002_ST1), tc.st1_after_fe);
		std::snprintf(name, sizeof(name), "dsp3 a02 %s fe so0", tc.tag);
		check_equal(name, m_dsp->debug_serial_output_register(0), tc.so0_after_fe);

		m_dsp->debug_run_cycles(1); // ff
		std::snprintf(name, sizeof(name), "dsp3 a02 %s ff pc", tc.tag);
		check_equal(name, m_dsp->state_int(TMS57002_PC), 0x00);
		std::snprintf(name, sizeof(name), "dsp3 a02 %s ff macc", tc.tag);
		check_equal(name, s64(m_dsp->debug_macc()), s64(tc.macc_after_fe));
		std::snprintf(name, sizeof(name), "dsp3 a02 %s ff maccr", tc.tag);
		check_equal(name, s64(m_dsp->debug_macc_read()), s64(tc.macc_after_fc));
		std::snprintf(name, sizeof(name), "dsp3 a02 %s ff maccw", tc.tag);
		check_equal(name, s64(m_dsp->debug_macc_write()), s64(tc.macc_after_fe));
		std::snprintf(name, sizeof(name), "dsp3 a02 %s ff st1", tc.tag);
		check_equal(name, m_dsp->state_int(TMS57002_ST1), tc.st1_after_ff);
		std::snprintf(name, sizeof(name), "dsp3 a02 %s ff so1", tc.tag);
		check_equal(name, m_dsp->debug_serial_output_register(1), tc.so1_after_ff);
	}
}

void tms57002test_state::test_instruction_level_serial_passthrough()
{
	static constexpr u32 ST0_DSP3 = 0x018082;
	static constexpr u32 ST1_DSP3 = 0x000820;
	static constexpr u32 OPCODE_NOP = 0x000000;
	static constexpr u32 OPCODE_DIS_SI0L_D02 = 0x008102;
	static constexpr u32 OPCODE_DOS_SO0L_D02 = 0x00E102;
	static constexpr u32 OPCODE_IDLE = 0xFC4000;
	static constexpr u32 INPUT_SAMPLE = 0x00123456;

	std::array<u32, 0x3f> program{};
	program.fill(OPCODE_NOP);
	program[0x3c] = OPCODE_DIS_SI0L_D02;
	program[0x3d] = OPCODE_DOS_SO0L_D02;
	program[0x3e] = OPCODE_IDLE;

	m_dsp->set_serial_frame_model(true);
	m_dsp->set_serial_frame_clocks(512);
	m_dsp->set_sync_polarity(1);
	m_dsp->debug_load_program(program.data(), u32(program.size()), ST0_DSP3, ST1_DSP3);

	check_equal("serial load program st0", m_dsp->state_int(TMS57002_ST0), ST0_DSP3);
	check_equal("serial load program st1", m_dsp->state_int(TMS57002_ST1), ST1_DSP3);
	check_equal("serial load program pc", m_dsp->state_int(TMS57002_PC), 0);

	const auto one_cycle = m_dsp->debug_run_sample_frame({INPUT_SAMPLE, 0, 0, 0}, 1);
	check_equal("serial one-cycle so0", one_cycle[0], 0);
	check_equal("serial one-cycle so1", one_cycle[1], 0);
	check_equal("serial one-cycle pc", m_dsp->state_int(TMS57002_PC), 1);

	std::array<u32, 3> dis_program{ OPCODE_DIS_SI0L_D02, OPCODE_NOP, OPCODE_IDLE };
	m_dsp->debug_load_program(dis_program.data(), u32(dis_program.size()), ST0_DSP3, ST1_DSP3);
	const auto dis_frame = m_dsp->debug_run_sample_frame({INPUT_SAMPLE, 0, 0, 0}, 8);
	check_equal("serial dis ba0 after sync", m_dsp->debug_ba0(), 0xff);
	check_equal("serial dis physical d01", m_dsp->dmem0_value(0x01), INPUT_SAMPLE);
	check_equal("serial dis so0 remains zero", dis_frame[0], 0);
	check_equal("serial dis pc after idle", m_dsp->state_int(TMS57002_PC), 3);

	std::array<u32, 3> dos_program{ OPCODE_DIS_SI0L_D02, OPCODE_DOS_SO0L_D02, OPCODE_IDLE };
	m_dsp->debug_load_program(dos_program.data(), u32(dos_program.size()), ST0_DSP3, ST1_DSP3);
	const auto dos_frame = m_dsp->debug_run_sample_frame({INPUT_SAMPLE, 0, 0, 0}, 8);
	check_equal("serial dos physical d01", m_dsp->dmem0_value(0x01), INPUT_SAMPLE);
	check_equal("serial dos so0 register", m_dsp->debug_serial_output_register(0), INPUT_SAMPLE);
	check_equal("serial dos so0 pin first frame", dos_frame[0], 0);
	check_equal("serial dos pc after idle", m_dsp->state_int(TMS57002_PC), 3);
}

void tms57002test_state::test_example53_bank_switched_serial_access()
{
	static constexpr u32 ST0_DSP3 = 0x018082;
	static constexpr u32 ST1_DSP3 = 0x000820;
	static constexpr u32 OPCODE_IDLE = 0xFC4000;
	static constexpr u32 OPCODE_LDPK_1 = 0x022800;
	static constexpr u32 OPCODE_DIS_SI1L_D00 = 0x009100;
	static constexpr u32 OPCODE_DOS_SO1L_D00 = 0x00F100;
	static constexpr u32 INPUT_SAMPLE = 0x00543210;
	static constexpr u32 DMEM0_SENTINEL = 0x00111111;
	static constexpr u32 DMEM1_SENTINEL = 0x00222222;

	m_dsp->set_serial_frame_model(true);
	m_dsp->set_serial_frame_clocks(512);
	m_dsp->set_sync_polarity(1);

	{
		std::array<u32, 3> program{ OPCODE_LDPK_1, OPCODE_DIS_SI1L_D00, OPCODE_IDLE };
		m_dsp->debug_load_program(program.data(), u32(program.size()), ST0_DSP3, ST1_DSP3);
		m_dsp->debug_write_dmem0(0x01, DMEM0_SENTINEL);
		m_dsp->debug_write_dmem1(0x01, DMEM1_SENTINEL);
		const auto frame = m_dsp->debug_run_sample_frame({ 0, 0, INPUT_SAMPLE, 0 }, 8);
		check_equal("example53 dis bank-switched ba1", m_dsp->debug_ba1(), 0x01);
		check_equal("example53 dis writes dmem1[1]", m_dsp->dmem1_value(0x01), INPUT_SAMPLE);
		check_equal("example53 dis keeps dmem0[1]", m_dsp->dmem0_value(0x01), DMEM0_SENTINEL);
		check_equal("example53 dis frame pin stays zero", frame[2], 0);
	}

	{
		std::array<u32, 3> program{ OPCODE_LDPK_1, OPCODE_DOS_SO1L_D00, OPCODE_IDLE };
		m_dsp->debug_load_program(program.data(), u32(program.size()), ST0_DSP3, ST1_DSP3);
		m_dsp->debug_write_dmem0(0x01, DMEM0_SENTINEL);
		m_dsp->debug_write_dmem1(0x01, DMEM1_SENTINEL);
		const auto frame = m_dsp->debug_run_sample_frame({ 0, 0, 0, 0 }, 8);
		check_equal("example53 dos bank-switched so1_l reg", m_dsp->debug_serial_output_register(2), DMEM1_SENTINEL);
		check_equal("example53 dos bank-switched pin first frame", frame[2], 0);
	}
}

void tms57002test_state::test_example53_one_frame_output_latency()
{
	static constexpr u32 ST0_DSP3 = 0x018082;
	static constexpr u32 ST1_DSP3 = 0x000820;
	static constexpr u32 OPCODE_IDLE = 0xFC4000;
	static constexpr u32 OPCODE_DOS_SO0L_D01 = 0x00E101;
	static constexpr u32 OUTPUT_SAMPLE = 0x00654321;

	std::array<u32, 0x100> program{};
	program.fill(0);
	program[0x00] = OPCODE_DOS_SO0L_D01;
	program[0xff] = OPCODE_IDLE;

	m_dsp->set_serial_frame_model(true);
	m_dsp->set_serial_frame_clocks(512);
	m_dsp->set_sync_polarity(1);
	m_dsp->debug_load_program(program.data(), u32(program.size()), ST0_DSP3, ST1_DSP3);
	m_dsp->debug_write_dmem0(0x00, OUTPUT_SAMPLE);

	const auto frame0 = m_dsp->debug_run_sample_frame({ 0, 0, 0, 0 }, 0x200);
	check_equal("example53 latency so0 reg after frame0", m_dsp->debug_serial_output_register(0), OUTPUT_SAMPLE);
	check_equal("example53 latency pin frame0", frame0[0], OUTPUT_SAMPLE);

	const auto frame1 = m_dsp->debug_run_sample_frame({ 0, 0, 0, 0 }, 0x200);
	check_equal("example53 latency pin frame1", frame1[0], 0);
}

void tms57002test_state::test_example53_dos_deadline()
{
	static constexpr u32 ST0_DSP3 = 0x018082;
	static constexpr u32 ST1_DSP3 = 0x000820;
	static constexpr u32 OPCODE_IDLE = 0xFC4000;
	static constexpr u32 OPCODE_DOS_SO0L_D01 = 0x00E101;
	static constexpr u32 OUTPUT_SAMPLE = 0x003579bd;

	auto run_case = [this](u8 dos_pc) -> std::array<u32, 2>
	{
		std::array<u32, 0x100> program{};
		program.fill(0);
		program[dos_pc] = OPCODE_DOS_SO0L_D01;
		program[0xff] = OPCODE_IDLE;

		m_dsp->set_serial_frame_model(true);
		m_dsp->set_serial_frame_clocks(512);
		m_dsp->set_sync_polarity(1);
		m_dsp->debug_load_program(program.data(), u32(program.size()), ST0_DSP3, ST1_DSP3);
		m_dsp->debug_write_dmem0(0x00, OUTPUT_SAMPLE);

		const auto frame0 = m_dsp->debug_run_sample_frame({ 0, 0, 0, 0 }, 0x200);
		const auto frame1 = m_dsp->debug_run_sample_frame({ 0, 0, 0, 0 }, 0x200);
		return { frame0[0], frame1[0] };
	};

	const auto at_fa = run_case(0xfa);
	check_equal("example53 dos@fa frame0", at_fa[0], OUTPUT_SAMPLE);
	check_equal("example53 dos@fa frame1", at_fa[1], 0);

	const auto at_fb = run_case(0xfb);
	check_equal("example53 dos@fb frame0", at_fb[0], OUTPUT_SAMPLE);
	check_equal("example53 dos@fb frame1", at_fb[1], 0);
}

void tms57002test_state::test_dsp3_mpy_dis_uses_prior_dmem()
{
	static constexpr u32 ST0_DSP3 = 0x018082;
	static constexpr u32 ST1_DSP3 = 0x000820;
	static constexpr u32 OPCODE_IDLE = 0xFC4000;
	static constexpr u32 OPCODE_DSP3_PC3C = 0x00848302; // mpy d(02),c*+ ; dis si0_l,d(02)
	static constexpr u32 OLD_SAMPLE = 0x00011111;
	static constexpr u32 NEW_SAMPLE = 0x00022222;
	static constexpr u32 UNITY_COEFF = 0x00000080;

	std::array<u32, 3> program{ OPCODE_DSP3_PC3C, OPCODE_IDLE, 0 };

	m_dsp->set_serial_frame_model(true);
	m_dsp->set_serial_frame_clocks(512);
	m_dsp->set_sync_polarity(1);
	m_dsp->debug_load_program(program.data(), u32(program.size()), ST0_DSP3, ST1_DSP3);
	m_dsp->debug_set_accumulators(0, 0, 0, 0, 0);
	m_dsp->debug_write_cmem(0x00, UNITY_COEFF);
	m_dsp->debug_write_dmem0(0x01, OLD_SAMPLE); // d(02) after sync with BA0=ff

	const auto frame = m_dsp->debug_run_sample_frame({ NEW_SAMPLE, 0, 0, 0 }, 8);
	check_equal("dsp3 mpy/dis updates d(02) from SI", m_dsp->dmem0_value(0x01), NEW_SAMPLE);
	check_equal("dsp3 mpy/dis pin frame stays zero", frame[0], 0);
	check_equal("dsp3 mpy/dis macc uses prior dmem", s64(m_dsp->debug_macc()), s64(OLD_SAMPLE));
	check_equal("dsp3 mpy/dis macc_read holds prior cycle", s64(m_dsp->debug_macc_read()), 0);
}

void tms57002test_state::test_dsp3_mpy_dis_uses_incoming_dmem()
{
	// Four standalone silicon runs (312 exact words) establish that a DIS
	// co-issued with a DMEM-reading MPY writes early enough for MPY to consume
	// the incoming serial word. This is the live DSP3 distortion idiom.
	static constexpr u32 ST0_DSP3 = 0x018082;
	static constexpr u32 ST1_DSP3 = 0x000820;
	static constexpr u32 OPCODE_IDLE = 0xFC4000;
	static constexpr u32 OPCODE_MPY_D02_CSTAR__DIS_SI0L_D02 = 0x00848302;
	static constexpr u32 OLD_SAMPLE = 0x00011111;
	static constexpr u32 INCOMING_SAMPLE = 0x00022222;
	static constexpr u32 UNITY_COEFF = 0x00000080;

	std::array<u32, 3> program{ OPCODE_MPY_D02_CSTAR__DIS_SI0L_D02, OPCODE_IDLE, 0 };
	m_dsp->set_serial_frame_model(true);
	m_dsp->set_serial_frame_clocks(512);
	m_dsp->set_sync_polarity(1);
	m_dsp->debug_load_program(program.data(), u32(program.size()), ST0_DSP3, ST1_DSP3);
	m_dsp->debug_set_accumulators(0, 0, 0, 0, 0);
	m_dsp->debug_write_cmem(0x00, UNITY_COEFF);
	m_dsp->debug_write_dmem0(0x01, OLD_SAMPLE); // logical d(02) after SYNC makes BA0=ff

	const auto frame = m_dsp->debug_run_sample_frame({ INCOMING_SAMPLE, 0, 0, 0 }, 8);
	check_equal("dsp3 mpy/dis stores incoming SI word", m_dsp->dmem0_value(0x01), INCOMING_SAMPLE);
	check_equal("dsp3 mpy/dis pin frame stays zero", frame[0], 0);
	check_equal("dsp3 mpy/dis multiplier consumes incoming word", s64(m_dsp->debug_macc()), s64(INCOMING_SAMPLE));
	check_equal("dsp3 mpy/dis macc_read holds prior cycle", s64(m_dsp->debug_macc_read()), 0);
}

void tms57002test_state::test_dsp3_wre_smhd_orders_store_before_xwr()
{
	static constexpr u32 ST0_DSP3 = 0x018082;
	static constexpr u32 ST1_CLEAR = 0x000000;
	static constexpr u32 OPCODE_WRE_D00_CSTAR__SMHD_D00 = 0x00e01b00;
	static constexpr u32 DMEM_SAMPLE = 0x00123400;
	static constexpr u64 MACC_SAMPLE = 0x0123456789012ULL;

	std::array<u32, 0x20> program{};
	program.fill(0);
	program[0x00] = OPCODE_WRE_D00_CSTAR__SMHD_D00;
	m_dsp->debug_load_program(program.data(), u32(program.size()), ST0_DSP3, ST1_CLEAR);
	m_dsp->debug_write_dmem0(0x00, DMEM_SAMPLE);
	m_dsp->debug_write_cmem(0x7d, 0x00000001);
	m_dsp->debug_set_exec_state(0x00, 0x7d, 0x00, 0x00, 0x00, ST0_DSP3, ST1_CLEAR, 0);
	m_dsp->debug_set_accumulators(0, MACC_SAMPLE, MACC_SAMPLE, MACC_SAMPLE, 0);
	m_dsp->debug_set_xmem_state(0, 0x00008, 0, 0, 0, 0, 0, 0);

	address_space &ext = m_dsp->space(AS_DATA);
	ext.write_byte(0x12, 0x00);
	ext.write_byte(0x13, 0x00);

	m_dsp->debug_run_cycles(32);

	check_equal("dsp3 wre/smhd xwr uses store-updated dmem", m_dsp->debug_xwr(), 0x00123456);
	check_equal("dsp3 wre/smhd dmem updated after wre", m_dsp->dmem0_value(0x00), 0x00123456);
	check_equal("dsp3 wre/smhd ext byte hi", ext.read_byte(0x12), 0x12);
	check_equal("dsp3 wre/smhd ext byte lo", ext.read_byte(0x13), 0x34);
}

void tms57002test_state::test_dsp3_rde_sacc_uses_store_updated_cmem_for_xoa()
{
	static constexpr u32 ST0_DSP3 = 0x018082;
	static constexpr u32 ST1_CLEAR = 0x000000;
	static constexpr u32 OPCODE_NOP = 0x000000;
	static constexpr u32 OPCODE_RDE_C46__SACC_C46 = 0x00e40d46;

	std::array<u32, 4> program{ OPCODE_RDE_C46__SACC_C46, OPCODE_NOP, OPCODE_NOP, OPCODE_NOP };
	m_dsp->debug_load_program(program.data(), u32(program.size()), ST0_DSP3, ST1_CLEAR);
	m_dsp->debug_write_cmem(0x46, 0x00000001);
	m_dsp->debug_set_exec_state(0x00, 0x00, 0x00, 0x00, 0x00, ST0_DSP3, ST1_CLEAR, 0);
	m_dsp->debug_set_accumulators(0x12345678, 0, 0, 0, 0);
	m_dsp->debug_set_xmem_state(0, 0x00008, 0, 0, 0, 0, 0, 0);

	m_dsp->debug_run_cycles(1);

	// The 2026-07-28 direct XRAM-bus discriminator proved that the category-2
	// store retires before RDE latches its address operand.
	check_equal("dsp3 rde/sacc xoa uses store-updated cmem", m_dsp->debug_xoa(), 0x12345678);
	check_equal("dsp3 rde/sacc cmem stores aacc", m_dsp->cmem_value(0x46), 0x12345678);
}

void tms57002test_state::test_dsp3_lmhd_srbd_uses_store_updated_dmem_for_macc()
{
	static constexpr u32 ST0_DSP3 = 0x018082;
	static constexpr u32 ST1_CLEAR = 0x000000;
	static constexpr u32 OPCODE_LMHD_D46__SRBD_D46 = 0x00c47946;
	static constexpr u32 OLD_DMEM = 0x00abcdef;
	static constexpr u32 XRD_VALUE = 0x00123456;

	std::array<u32, 2> program{ OPCODE_LMHD_D46__SRBD_D46, 0 };
	m_dsp->debug_load_program(program.data(), u32(program.size()), ST0_DSP3, ST1_CLEAR);
	m_dsp->debug_write_dmem0(0x46, OLD_DMEM);
	m_dsp->debug_set_exec_state(0x00, 0x00, 0x00, 0x00, 0x00, ST0_DSP3, ST1_CLEAR, 0);
	m_dsp->debug_set_accumulators(0, 0, 0, 0, 0);
	m_dsp->debug_set_xmem_state(0, 0, 0, XRD_VALUE, 0, 0, 0, 0);

	m_dsp->debug_run_cycles(1);

	// SRBD retires before the co-issued LMHD reads DMEM, matching the same
	// silicon-proven store-before-category-1 rule.
	check_equal("dsp3 lmhd/srbd dmem becomes xrd", m_dsp->dmem0_value(0x46), XRD_VALUE);
	check_equal("dsp3 lmhd/srbd macc uses store-updated dmem", s64(m_dsp->debug_macc()), sx24(XRD_VALUE) << 24);
}

void tms57002test_state::test_lacc_lira_same_word_uses_pre_lacc_aacc()
{
	static constexpr u32 ST0_DSP3 = 0x018082;
	static constexpr u32 ST1_CLEAR = 0x000000;
	static constexpr u32 OPCODE_LACC_C79__LIRA = 0x00484d79;
	static constexpr u32 OLD_AACC = 0x46000000;
	static constexpr u32 C79_VALUE = 0xe08a067a;

	std::array<u32, 2> program{ OPCODE_LACC_C79__LIRA, 0 };
	m_dsp->debug_load_program(program.data(), u32(program.size()), ST0_DSP3, ST1_CLEAR);
	m_dsp->debug_write_cmem(0x79, C79_VALUE);
	m_dsp->debug_set_exec_state(0x00, 0x00, 0x00, 0x00, 0x00, ST0_DSP3, ST1_CLEAR, 0);
	m_dsp->debug_set_accumulators(OLD_AACC, 0, 0, 0, 0);

	m_dsp->debug_run_cycles(1);

	check_equal("lacc/lira same-word aacc", u32(m_dsp->debug_aacc()), C79_VALUE);
	check_equal("lacc/lira same-word id", m_dsp->state_int(TMS57002_ID), OLD_AACC >> 24);
}

void tms57002test_state::test_cmem_update_multiword_sequence()
{
	static constexpr u32 ST0_DSP2 = 0x01c082;
	static constexpr u32 ST1_DSP2 = 0x000020;
	static constexpr u32 OPCODE_NOP = 0x000000;
	static constexpr u32 OPCODE_LACC_C_BASE = 0x00480500;
	static constexpr u32 UPDATE0 = 0x00010000;
	static constexpr u32 UPDATE1 = 0x00020000;
	static constexpr u32 UPDATE2 = 0x00030000;

	const auto write_update_word = [this](u32 value)
	{
		m_dsp->data_w(u8(value >> 24));
		m_dsp->data_w(u8(value >> 16));
		m_dsp->data_w(u8(value >> 8));
		m_dsp->data_w(u8(value));
	};

	std::array<u32, 4> program{
		OPCODE_LACC_C_BASE | 0x10,
		OPCODE_LACC_C_BASE | 0x11,
		OPCODE_LACC_C_BASE | 0x12,
		OPCODE_NOP
	};

	m_dsp->debug_load_program(program.data(), u32(program.size()), ST0_DSP2, ST1_DSP2);
	m_dsp->debug_write_cmem(0x10, 0x01010101);
	m_dsp->debug_write_cmem(0x11, 0x02020202);
	m_dsp->debug_write_cmem(0x12, 0x03030303);
	m_dsp->debug_set_exec_state(0x00, 0x00, 0x00, 0x00, 0x00, ST0_DSP2, ST1_DSP2, 0);
	m_dsp->debug_set_accumulators(0, 0, 0, 0, 0);

	m_dsp->cload_w(0);
	m_dsp->data_w(0x10);
	write_update_word(UPDATE0);
	write_update_word(UPDATE1);
	write_update_word(UPDATE2);
	m_dsp->cload_w(1);

	check_equal("cmem update queued words", m_dsp->update_pending_count(), 3);

	m_dsp->debug_run_cycles(1);
	check_equal("cmem update0 aacc", u32(m_dsp->debug_aacc()), UPDATE0);
	check_equal("cmem update0 target", m_dsp->cmem_value(0x10), UPDATE0);
	check_equal("cmem update after first read pending", m_dsp->update_pending_count(), 2);

	m_dsp->debug_run_cycles(1);
	check_equal("cmem update1 aacc", u32(m_dsp->debug_aacc()), UPDATE1);
	check_equal("cmem update1 target", m_dsp->cmem_value(0x11), UPDATE1);
	check_equal("cmem update after second read pending", m_dsp->update_pending_count(), 1);

	m_dsp->debug_run_cycles(1);
	check_equal("cmem update2 aacc", u32(m_dsp->debug_aacc()), UPDATE2);
	check_equal("cmem update2 target", m_dsp->cmem_value(0x12), UPDATE2);
	check_equal("cmem update empty after sequence", m_dsp->update_pending_count(), 0);
}

void tms57002test_state::test_cmem_update_waits_for_cload_high()
{
	static constexpr u32 ST0_DSP2 = 0x01c082;
	static constexpr u32 ST1_DSP2 = 0x000020;
	static constexpr u32 OPCODE_LACC_C10 = 0x00480510;
	static constexpr u32 OPCODE_NOP = 0x000000;
	static constexpr u32 OLD_C10 = 0x01010101;
	static constexpr u32 UPDATE0 = 0x00010000;

	const auto write_update_word = [this](u32 value)
	{
		m_dsp->data_w(u8(value >> 24));
		m_dsp->data_w(u8(value >> 16));
		m_dsp->data_w(u8(value >> 8));
		m_dsp->data_w(u8(value));
	};

	std::array<u32, 3> program{
		OPCODE_LACC_C10,
		OPCODE_LACC_C10,
		OPCODE_NOP
	};

	m_dsp->debug_load_program(program.data(), u32(program.size()), ST0_DSP2, ST1_DSP2);
	m_dsp->debug_write_cmem(0x10, OLD_C10);
	m_dsp->debug_set_exec_state(0x00, 0x00, 0x00, 0x00, 0x00, ST0_DSP2, ST1_DSP2, 0);
	m_dsp->debug_set_accumulators(0, 0, 0, 0, 0);

	m_dsp->cload_w(0);
	m_dsp->data_w(0x10);
	write_update_word(UPDATE0);

	check_equal("cmem cload-low update queued", m_dsp->update_pending_count(), 1);

	m_dsp->debug_run_cycles(1);
	check_equal("cmem cload-low aacc remains old", u32(m_dsp->debug_aacc()), OLD_C10);
	check_equal("cmem cload-low target remains old", m_dsp->cmem_value(0x10), OLD_C10);
	check_equal("cmem cload-low update still pending", m_dsp->update_pending_count(), 1);

	m_dsp->cload_w(1);
	m_dsp->debug_run_cycles(1);
	check_equal("cmem cload-high aacc receives update", u32(m_dsp->debug_aacc()), UPDATE0);
	check_equal("cmem cload-high target updated", m_dsp->cmem_value(0x10), UPDATE0);
	check_equal("cmem cload-high update consumed", m_dsp->update_pending_count(), 0);
}

void tms57002test_state::test_cmem_update_new_address_breaks_active_run()
{
	static constexpr u32 ST0_DSP2 = 0x01c082;
	static constexpr u32 ST1_DSP2 = 0x000020;
	static constexpr u32 OPCODE_NOP = 0x000000;
	static constexpr u32 OPCODE_LACC_C_BASE = 0x00480500;
	static constexpr u32 OLD_C37 = 0x37373737;
	static constexpr u32 OLD_C38 = 0x38383838;
	static constexpr u32 OLD_C39 = 0x39393939;
	static constexpr u32 UPDATE38 = 0x11111111;
	static constexpr u32 UPDATE37 = 0x22222222;

	const auto write_update_word = [this](u32 value)
	{
		m_dsp->data_w(u8(value >> 24));
		m_dsp->data_w(u8(value >> 16));
		m_dsp->data_w(u8(value >> 8));
		m_dsp->data_w(u8(value));
	};

	std::array<u32, 4> program{
		OPCODE_LACC_C_BASE | 0x38,
		OPCODE_LACC_C_BASE | 0x39,
		OPCODE_LACC_C_BASE | 0x37,
		OPCODE_NOP
	};

	m_dsp->debug_load_program(program.data(), u32(program.size()), ST0_DSP2, ST1_DSP2);
	m_dsp->debug_write_cmem(0x37, OLD_C37);
	m_dsp->debug_write_cmem(0x38, OLD_C38);
	m_dsp->debug_write_cmem(0x39, OLD_C39);
	m_dsp->debug_set_exec_state(0x00, 0x00, 0x00, 0x00, 0x00, ST0_DSP2, ST1_DSP2, 0);
	m_dsp->debug_set_accumulators(0, 0, 0, 0, 0);

	m_dsp->cload_w(0);
	m_dsp->data_w(0x38);
	write_update_word(UPDATE38);
	m_dsp->cload_w(1);
	m_dsp->cload_w(0);
	m_dsp->data_w(0x37);
	write_update_word(UPDATE37);
	m_dsp->cload_w(1);

	check_equal("cmem explicit-address queue words", m_dsp->update_pending_count(), 2);

	m_dsp->debug_run_cycles(1);
	check_equal("cmem explicit update38 aacc", u32(m_dsp->debug_aacc()), UPDATE38);
	check_equal("cmem explicit update38 target", m_dsp->cmem_value(0x38), UPDATE38);
	check_equal("cmem explicit after c38 pending", m_dsp->update_pending_count(), 1);

	m_dsp->debug_run_cycles(1);
	check_equal("cmem explicit c39 remains old", u32(m_dsp->debug_aacc()), OLD_C39);
	check_equal("cmem explicit c39 target unchanged", m_dsp->cmem_value(0x39), OLD_C39);
	check_equal("cmem explicit after c39 pending", m_dsp->update_pending_count(), 1);

	m_dsp->debug_run_cycles(1);
	check_equal("cmem explicit update37 aacc", u32(m_dsp->debug_aacc()), UPDATE37);
	check_equal("cmem explicit update37 target", m_dsp->cmem_value(0x37), UPDATE37);
	check_equal("cmem explicit consumed after c37", m_dsp->update_pending_count(), 0);
}

void tms57002test_state::test_pc33_mpy_mac_direct_coeff_chain()
{
	static constexpr u32 ST0_DSP2 = 0x01c082;
	static constexpr u32 ST1_DSP2 = 0x000020;
	static constexpr u32 OPCODE_NOP = 0x000000;
	static constexpr u32 OPCODE_MPY_DSTAR_C40 = 0x00840740;
	static constexpr u32 OPCODE_MAC_DSTAR_C41 = 0x00900741;
	static constexpr u32 OPCODE_MAC_DSTAR_C42 = 0x00900742;
	static constexpr u32 OPCODE_MAC_DSTAR_C43 = 0x00900743;
	static constexpr u32 OPCODE_MPY_DSTAR_C45 = 0x00840745;
	static constexpr u32 OPCODE_MAC_DSTAR_C46 = 0x00900746;
	static constexpr u32 OPCODE_MAC_DSTAR_C47 = 0x00900747;
	static constexpr u32 OPCODE_MAC_DSTAR_C48 = 0x00900748;
	static constexpr u32 D40_VALUE = 0x000100;
	static constexpr u32 D41_VALUE = 0x000080;
	static constexpr u32 D42_VALUE = 0x000200;
	static constexpr u32 D43_VALUE = 0x000040;
	static constexpr u32 C40_VALUE = 0x00010000;
	static constexpr u32 C41_ZERO = 0x00000000;
	static constexpr u32 C42_ZERO = 0x00000000;
	static constexpr u32 C42_VALUE = 0x00020000;
	static constexpr u32 C43_ZERO = 0x00000000;
	static constexpr u32 C45_VALUE = 0x00030000;
	static constexpr u32 C46_ZERO = 0x00000000;
	static constexpr u32 C47_ZERO = 0x00000000;
	static constexpr u32 C47_VALUE = 0x00050000;
	static constexpr u32 C48_ZERO = 0x00000000;

	std::array<u32, 0x40> program;
	program.fill(OPCODE_NOP);
	program[0x33] = OPCODE_MPY_DSTAR_C40;
	program[0x34] = OPCODE_MAC_DSTAR_C41;
	program[0x35] = OPCODE_MAC_DSTAR_C42;
	program[0x36] = OPCODE_MAC_DSTAR_C43;
	program[0x3b] = OPCODE_MPY_DSTAR_C45;
	program[0x3c] = OPCODE_MAC_DSTAR_C46;
	program[0x3d] = OPCODE_MAC_DSTAR_C47;
	program[0x3e] = OPCODE_MAC_DSTAR_C48;

	const auto run_case = [&](u32 c42_value) -> std::array<u64, 3>
	{
		m_dsp->debug_load_program(program.data(), u32(program.size()), ST0_DSP2, ST1_DSP2);
		m_dsp->debug_write_dmem0(0x40, D40_VALUE);
		m_dsp->debug_write_dmem0(0x41, D41_VALUE);
		m_dsp->debug_write_dmem0(0x42, D42_VALUE);
		m_dsp->debug_write_cmem(0x40, C40_VALUE);
		m_dsp->debug_write_cmem(0x41, C41_ZERO);
		m_dsp->debug_write_cmem(0x42, c42_value);
		m_dsp->debug_set_exec_state(0x33, 0x00, 0x40, 0x00, 0x00, ST0_DSP2, ST1_DSP2, 0);
		m_dsp->debug_set_accumulators(0, 0, 0, 0, 0);

		m_dsp->debug_run_cycles(1);
		const u64 after_mpy = m_dsp->debug_macc();
		m_dsp->debug_run_cycles(1);
		const u64 after_mac_c41 = m_dsp->debug_macc();
		m_dsp->debug_run_cycles(1);
		const u64 after_mac_c42 = m_dsp->debug_macc();
		return { after_mpy, after_mac_c41, after_mac_c42 };
	};

	const auto run_pc33_block = [&](u32 c42_value) -> std::array<u64, 4>
	{
		m_dsp->debug_load_program(program.data(), u32(program.size()), ST0_DSP2, ST1_DSP2);
		m_dsp->debug_write_dmem0(0x07, D40_VALUE);
		m_dsp->debug_write_dmem0(0x08, D41_VALUE);
		m_dsp->debug_write_dmem0(0x09, D42_VALUE);
		m_dsp->debug_write_dmem0(0x0a, D43_VALUE);
		m_dsp->debug_write_cmem(0x40, C40_VALUE);
		m_dsp->debug_write_cmem(0x41, C41_ZERO);
		m_dsp->debug_write_cmem(0x42, c42_value);
		m_dsp->debug_write_cmem(0x43, C43_ZERO);
		m_dsp->debug_set_exec_state(0x33, 0x00, 0x07, 0x00, 0x00, ST0_DSP2, ST1_DSP2, 0);
		m_dsp->debug_set_accumulators(0, 0, 0, 0, 0);

		std::array<u64, 4> out{};
		for (u32 i = 0; i < 4; i++)
		{
			m_dsp->debug_run_cycles(1);
			out[i] = m_dsp->debug_macc();
		}
		return out;
	};

	const auto run_pc3b_block = [&](u32 c47_value) -> std::array<u64, 4>
	{
		m_dsp->debug_load_program(program.data(), u32(program.size()), ST0_DSP2, ST1_DSP2);
		m_dsp->debug_write_dmem0(0x07, D40_VALUE);
		m_dsp->debug_write_dmem0(0x08, D41_VALUE);
		m_dsp->debug_write_dmem0(0x09, D42_VALUE);
		m_dsp->debug_write_dmem0(0x0a, D43_VALUE);
		m_dsp->debug_write_cmem(0x45, C45_VALUE);
		m_dsp->debug_write_cmem(0x46, C46_ZERO);
		m_dsp->debug_write_cmem(0x47, c47_value);
		m_dsp->debug_write_cmem(0x48, C48_ZERO);
		m_dsp->debug_set_exec_state(0x3b, 0x00, 0x07, 0x00, 0x00, ST0_DSP2, ST1_DSP2, 0);
		m_dsp->debug_set_accumulators(0, 0, 0, 0, 0);

		std::array<u64, 4> out{};
		for (u32 i = 0; i < 4; i++)
		{
			m_dsp->debug_run_cycles(1);
			out[i] = m_dsp->debug_macc();
		}
		return out;
	};

	const auto c42_zero = run_case(C42_ZERO);
	const auto c42_nonzero = run_case(C42_VALUE);
	const auto pc33_zero = run_pc33_block(C42_ZERO);
	const auto pc33_nonzero = run_pc33_block(C42_VALUE);
	const auto pc3b_zero = run_pc3b_block(C47_ZERO);
	const auto pc3b_nonzero = run_pc3b_block(C47_VALUE);

	osd_printf_info("PC33 chain c42=0      after_mpy=%013llx after_c41=%013llx after_c42=%013llx\n",
		(unsigned long long)c42_zero[0],
		(unsigned long long)c42_zero[1],
		(unsigned long long)c42_zero[2]);
	osd_printf_info("PC33 chain c42=20000  after_mpy=%013llx after_c41=%013llx after_c42=%013llx\n",
		(unsigned long long)c42_nonzero[0],
		(unsigned long long)c42_nonzero[1],
		(unsigned long long)c42_nonzero[2]);
	osd_printf_info("PC33..36 exact c42=0      pc33=%013llx pc34=%013llx pc35=%013llx pc36=%013llx\n",
		(unsigned long long)pc33_zero[0],
		(unsigned long long)pc33_zero[1],
		(unsigned long long)pc33_zero[2],
		(unsigned long long)pc33_zero[3]);
	osd_printf_info("PC33..36 exact c42=20000  pc33=%013llx pc34=%013llx pc35=%013llx pc36=%013llx\n",
		(unsigned long long)pc33_nonzero[0],
		(unsigned long long)pc33_nonzero[1],
		(unsigned long long)pc33_nonzero[2],
		(unsigned long long)pc33_nonzero[3]);
	osd_printf_info("PC3B..3E exact c47=0      pc3b=%013llx pc3c=%013llx pc3d=%013llx pc3e=%013llx\n",
		(unsigned long long)pc3b_zero[0],
		(unsigned long long)pc3b_zero[1],
		(unsigned long long)pc3b_zero[2],
		(unsigned long long)pc3b_zero[3]);
	osd_printf_info("PC3B..3E exact c47=50000  pc3b=%013llx pc3c=%013llx pc3d=%013llx pc3e=%013llx\n",
		(unsigned long long)pc3b_nonzero[0],
		(unsigned long long)pc3b_nonzero[1],
		(unsigned long long)pc3b_nonzero[2],
		(unsigned long long)pc3b_nonzero[3]);

	check_true("pc33 chain c40 produces macc", c42_zero[0] != 0);
	check_equal("pc33 chain zero c41 unchanged", s64(c42_zero[1]), s64(c42_zero[0]));
	check_equal("pc33 chain zero c42 unchanged", s64(c42_zero[2]), s64(c42_zero[1]));
	check_equal("pc33 chain nonzero c41 unchanged", s64(c42_nonzero[1]), s64(c42_nonzero[0]));
	check_true("pc33 chain nonzero c42 accumulates", c42_nonzero[2] != c42_nonzero[1]);
	check_true("pc33 exact c40 produces macc", pc33_zero[0] != 0);
	check_equal("pc33 exact zero c41 unchanged", s64(pc33_zero[1]), s64(pc33_zero[0]));
	check_equal("pc33 exact zero c42 unchanged", s64(pc33_zero[2]), s64(pc33_zero[1]));
	check_equal("pc33 exact zero c43 unchanged", s64(pc33_zero[3]), s64(pc33_zero[2]));
	check_equal("pc33 exact nonzero c41 unchanged", s64(pc33_nonzero[1]), s64(pc33_nonzero[0]));
	check_true("pc33 exact nonzero c42 accumulates", pc33_nonzero[2] != pc33_nonzero[1]);
	check_equal("pc33 exact nonzero c43 unchanged", s64(pc33_nonzero[3]), s64(pc33_nonzero[2]));
	check_true("pc3b exact c45 produces macc", pc3b_zero[0] != 0);
	check_equal("pc3b exact zero c46 unchanged", s64(pc3b_zero[1]), s64(pc3b_zero[0]));
	check_equal("pc3b exact zero c47 unchanged", s64(pc3b_zero[2]), s64(pc3b_zero[1]));
	check_equal("pc3b exact zero c48 unchanged", s64(pc3b_zero[3]), s64(pc3b_zero[2]));
	check_equal("pc3b exact nonzero c46 unchanged", s64(pc3b_nonzero[1]), s64(pc3b_nonzero[0]));
	check_true("pc3b exact nonzero c47 accumulates", pc3b_nonzero[2] != pc3b_nonzero[1]);
	check_equal("pc3b exact nonzero c48 unchanged", s64(pc3b_nonzero[3]), s64(pc3b_nonzero[2]));
}

void tms57002test_state::test_pc33_producer_smhc_phase()
{
	static constexpr u32 ST0_DSP2 = 0x01c082;
	static constexpr u32 ST1_DSP2 = 0x000020;
	static constexpr u32 OPCODE_NOP = 0x000000;
	static constexpr u32 OPCODE_LACC_C42 = 0x00480542;
	static constexpr u32 OPCODE_MPY_D0 = 0x00980200;
	static constexpr u32 OPCODE_LACC_CSTAR = 0x00480200;
	static constexpr u32 OPCODE_SMHC_C42 = 0x00002d42;
	static constexpr u32 OPCODE_SFMO = 0x00880200;
	static constexpr u32 OPCODE_LACC_C43 = 0x00480543;
	static constexpr u32 OPCODE_SMHC_C43 = 0x00002d43;
	static constexpr u32 OPCODE_LACC_C47 = 0x00480547;
	static constexpr u32 OPCODE_SMHC_C47 = 0x00002d47;
	static constexpr u32 OPCODE_LACC_C48 = 0x00480548;
	static constexpr u32 OPCODE_SMHC_C48 = 0x00002d48;
	static constexpr u8 PC_CD = 0xcd;
	static constexpr u8 PC_D0 = 0xd0;
	static constexpr u8 PC_D1 = 0xd1;
	static constexpr u8 PC_D5 = 0xd5;
	static constexpr u8 PC_E6 = 0xe6;
	static constexpr u8 PC_E9 = 0xe9;
	static constexpr u8 PC_ED = 0xed;
	static constexpr u8 PC_EE = 0xee;
	static constexpr u8 CA_DA = 0xda;
	static constexpr u8 CA_E9 = 0xe9;
	static constexpr u8 ID_TRACE = 0x3a;
	static constexpr u8 ID_E_TRACE = 0x90;
	static constexpr u8 BA0_TRACE = 0xaa;
	static constexpr u8 BA1_TRACE = 0x00;
	static constexpr u8 BA1_E_TRACE = 0x56;
	static constexpr u32 D_MPY = 0x366a00;
	static constexpr u32 D_C42_SIDE = 0x803138;
	static constexpr u32 D_C43_SIDE = 0x36cd7b;
	static constexpr u32 D_C47_SIDE = 0x000087;
	static constexpr u32 D_C48_SIDE = 0x00007a;
	static constexpr u32 CDB_SOURCE = 0x0e8b1d16;
	static constexpr u32 CED_SOURCE = 0x0ced19da;
	static constexpr u64 C46_MACCR_TRACE = 0x01ae43620cc01ULL;
	static constexpr u32 C42_SEED = 0x00020000;
	static constexpr u32 C47_SEED = 0x00050000;

	std::array<u32, 0x100> program;
	program.fill(OPCODE_NOP);
	program[0xcd] = OPCODE_LACC_C42;
	program[0xce] = OPCODE_MPY_D0;
	program[0xcf] = OPCODE_LACC_CSTAR;
	program[0xd0] = OPCODE_SMHC_C42;
	program[0xd1] = OPCODE_SFMO;
	program[0xd2] = OPCODE_LACC_C43;
	program[0xd3] = OPCODE_MPY_D0;
	program[0xd4] = OPCODE_LACC_CSTAR;
	program[0xd5] = OPCODE_SMHC_C43;
	program[0xe6] = OPCODE_LACC_C47;
	program[0xe7] = OPCODE_MPY_D0;
	program[0xe8] = OPCODE_LACC_CSTAR;
	program[0xe9] = OPCODE_SMHC_C47;
	program[0xea] = OPCODE_SFMO;
	program[0xeb] = OPCODE_LACC_C48;
	program[0xec] = OPCODE_MPY_D0;
	program[0xed] = OPCODE_LACC_CSTAR;
	program[0xee] = OPCODE_SMHC_C48;

	struct producer_result
	{
		u32 c42_after_d0 = 0;
		u64 macc_after_d1 = 0;
		u32 c43_after_d5 = 0;
	};
	struct producer_e_result
	{
		u32 c47_after_e9 = 0;
		u32 aacc_after_ed = 0;
		u32 c48_after_ee = 0;
	};

	const auto run_case = [&](u32 c42_seed) -> producer_result
	{
		m_dsp->debug_load_program(program.data(), u32(program.size()), ST0_DSP2, ST1_DSP2);
		m_dsp->debug_write_cmem(0x42, c42_seed);
		m_dsp->debug_write_cmem(0x43, 0);
		m_dsp->debug_write_cmem(0xda, 0x7f000000);
		m_dsp->debug_write_cmem(0xdb, CDB_SOURCE);
		m_dsp->debug_write_cmem(0xdc, 0x00ffffff);
		m_dsp->debug_write_cmem(0xdd, 0x7f000000);
		m_dsp->debug_write_cmem(0xde, 0);
		m_dsp->debug_write_cmem(0xdf, 0x00ffffff);
		m_dsp->debug_write_dmem0(u8(0x00 + BA0_TRACE), D_MPY);
		m_dsp->debug_write_dmem0(u8(0x42 + BA0_TRACE), D_C42_SIDE);
		m_dsp->debug_write_dmem0(u8(0x43 + BA0_TRACE), D_C43_SIDE);
		m_dsp->debug_set_exec_state(PC_CD, CA_DA, ID_TRACE, BA0_TRACE, BA1_TRACE, ST0_DSP2, ST1_DSP2, 0);
		m_dsp->debug_set_accumulators(0, 0, 0, 0, 0);

		producer_result out;
		for (u32 pc = PC_CD; pc <= PC_D5; pc++)
		{
			m_dsp->debug_run_cycles(1);
			if (pc == PC_D0)
				out.c42_after_d0 = m_dsp->cmem_value(0x42);
			if (pc == PC_D1)
				out.macc_after_d1 = m_dsp->debug_macc();
			if (pc == PC_D5)
				out.c43_after_d5 = m_dsp->cmem_value(0x43);
		}
		return out;
	};

	const auto run_e_case = [&](u32 c47_seed) -> producer_e_result
	{
		m_dsp->debug_load_program(program.data(), u32(program.size()), ST0_DSP2, ST1_DSP2);
		m_dsp->debug_write_cmem(0x47, c47_seed);
		m_dsp->debug_write_cmem(0x48, 0);
		m_dsp->debug_write_cmem(0xe9, 0x7f000000);
		m_dsp->debug_write_cmem(0xea, 0);
		m_dsp->debug_write_cmem(0xeb, 0x00ffffff);
		m_dsp->debug_write_cmem(0xec, 0x7f000000);
		m_dsp->debug_write_cmem(0xed, CED_SOURCE);
		m_dsp->debug_write_cmem(0xee, 0);
		m_dsp->debug_write_dmem0(u8(0x00 + BA0_TRACE), D_MPY);
		m_dsp->debug_write_dmem0(u8(0x47 + BA0_TRACE), D_C47_SIDE);
		m_dsp->debug_write_dmem0(u8(0x48 + BA0_TRACE), D_C48_SIDE);
		m_dsp->debug_set_exec_state(PC_E6, CA_E9, ID_E_TRACE, BA0_TRACE, BA1_E_TRACE, ST0_DSP2, ST1_DSP2, 0);
		m_dsp->debug_set_accumulators(0, 0, C46_MACCR_TRACE, 0, 0x00ffffff);

		producer_e_result out;
		for (u32 pc = PC_E6; pc <= PC_EE; pc++)
		{
			m_dsp->debug_run_cycles(1);
			if (pc == PC_E9)
				out.c47_after_e9 = m_dsp->cmem_value(0x47);
			if (pc == PC_ED)
				out.aacc_after_ed = u32(m_dsp->debug_aacc());
			if (pc == PC_EE)
				out.c48_after_ee = m_dsp->cmem_value(0x48);
		}
		return out;
	};

	const producer_result zero_seed = run_case(0);
	const producer_result nonzero_seed = run_case(C42_SEED);
	const producer_e_result e_zero_seed = run_e_case(0);
	const producer_e_result e_nonzero_seed = run_e_case(C47_SEED);

	osd_printf_info("PC CD..D5 producer c42=0      c42_after_d0=%08x macc_after_d1=%013llx c43_after_d5=%08x\n",
		zero_seed.c42_after_d0,
		(unsigned long long)zero_seed.macc_after_d1,
		zero_seed.c43_after_d5);
	osd_printf_info("PC CD..D5 producer c42=20000  c42_after_d0=%08x macc_after_d1=%013llx c43_after_d5=%08x\n",
		nonzero_seed.c42_after_d0,
		(unsigned long long)nonzero_seed.macc_after_d1,
		nonzero_seed.c43_after_d5);
	osd_printf_info("PC E6..EE producer c47=0      c47_after_e9=%08x aacc_after_ed=%08x c48_after_ee=%08x\n",
		e_zero_seed.c47_after_e9,
		e_zero_seed.aacc_after_ed,
		e_zero_seed.c48_after_ee);
	osd_printf_info("PC E6..EE producer c47=50000  c47_after_e9=%08x aacc_after_ed=%08x c48_after_ee=%08x\n",
		e_nonzero_seed.c47_after_e9,
		e_nonzero_seed.aacc_after_ed,
		e_nonzero_seed.c48_after_ee);

	check_equal("pc33 producer zero seed keeps c42 zero", zero_seed.c42_after_d0, 0);
	check_true("pc33 producer zero seed creates later macc", zero_seed.macc_after_d1 != 0);
	check_true("pc33 producer zero seed fills c43", zero_seed.c43_after_d5 != 0);
	check_true("pc33 producer nonzero seed fills c42", nonzero_seed.c42_after_d0 != 0);
	check_equal("pc33 producer e zero seed keeps c47 zero", e_zero_seed.c47_after_e9, 0);
	check_true("pc33 producer e zero seed reaches c48 source", e_zero_seed.aacc_after_ed != 0);
	check_equal("pc33 producer e zero seed keeps c48 zero", e_zero_seed.c48_after_ee, 0);
	check_true("pc33 producer e nonzero seed fills c47", e_nonzero_seed.c47_after_e9 != 0);
}

void tms57002test_state::test_dsp3_rde_word0_sel1_dram_latency()
{
	static constexpr u32 ST0_DSP3 = 0x018082;
	static constexpr u32 ST1_CLEAR = 0x000000;
	static constexpr u32 OPCODE_NOP = 0x000000;
	static constexpr u32 OPCODE_RDE_C46__SACC_C46 = 0x00e40d46;
	static constexpr u32 OPCODE_RDE_C47__SACC_C47 = 0x00e40d47;

	std::array<u32, 8> program{
		OPCODE_RDE_C46__SACC_C46,
		OPCODE_NOP, OPCODE_NOP, OPCODE_NOP, OPCODE_NOP, OPCODE_NOP,
		OPCODE_RDE_C47__SACC_C47,
		OPCODE_NOP
	};
	m_dsp->debug_load_program(program.data(), u32(program.size()), ST0_DSP3, ST1_CLEAR);
	m_dsp->debug_write_cmem(0x46, 0x00000001);
	m_dsp->debug_write_cmem(0x47, 0x00000002);
	m_dsp->debug_set_exec_state(0x00, 0x00, 0x00, 0x00, 0x00, ST0_DSP3, ST1_CLEAR, 0);
	m_dsp->debug_set_accumulators(0, 0x123456780000ULL, 0x123456780000ULL, 0x123456780000ULL, 0);
	m_dsp->debug_set_xmem_state(0, 0x00008, 0, 0, 0, 0, 0, 0);

	address_space &ext = m_dsp->space(AS_DATA);
	ext.write_byte(0x12, 0x12);
	ext.write_byte(0x13, 0x34);

	m_dsp->debug_run_cycles(1);
	check_equal("dsp3 rde/sacc new CMEM address after issue", m_dsp->debug_xoa(), 0x00000000);
	check_equal("dsp3 rde latency xrd after issue", m_dsp->debug_xrd(), 0);

	m_dsp->debug_run_cycles(5);
	check_equal("dsp3 rde latency xrd after five followups", m_dsp->debug_xrd(), 0);

	m_dsp->debug_run_cycles(1);
	check_equal("dsp3 rde/sacc new-address read result", m_dsp->debug_xrd(), 0x00000000);
	check_equal("dsp3 rde/sacc competing issue uses updated address", m_dsp->debug_xoa(), 0x00000000);
}

void tms57002test_state::test_xmem_micro_oracles()
{
	// ST0 configurations (M=memory size, SEL/WORD=fetch width).
	static constexpr u32 ST0_16BIT_256K = 0x018082; // SEL, !WORD, 256K  (real DSP3)
	static constexpr u32 ST0_24BIT_256K = 0x01c082; // SEL,  WORD, 256K  (24-bit, real DSP2 word mode)
	static constexpr u32 ST0_16BIT_64K  = 0x008082; // SEL, !WORD, 64K   (DSP1-like small)

	// sti state bits (tms57002.h enum): S_READ=0x40, S_WRITE=0x80.
	static constexpr u32 S_READ_BIT  = 0x00000040;
	static constexpr u32 S_WRITE_BIT = 0x00000080;

	// WRE D00 C* / SMHD D00 : byte addr <- cmem[ca=0x7d] (NOT clobbered by smhd),
	// xwr <- dmem0[0x00]. Used only to exercise xm_init's address arithmetic.
	static constexpr u32 OP_WRE = 0x00e01b00;

	address_space &ext = m_dsp->space(AS_DATA);

	auto mem_mask = [](u32 st0) -> u32 {
		switch (st0 & 0x030000) {
		case 0x000000: return 0x0ffff;
		case 0x010000: return 0x3ffff;
		default:       return 0xfffff;
		}
	};
	// Byte address the core SHOULD compute for a (xoa,xba) word pair (mirrors xm_init).
	auto want_addr = [&](u32 st0, u32 xoa_word, u32 xba_word) -> u32 {
		u32 adr = xoa_word + xba_word;
		adr <<= (st0 & 0x004000) ? 2 : 1;   // WORD -> <<2, else <<1
		if (!(st0 & 0x008000)) adr <<= 1;    // !SEL -> extra <<1
		return adr & mem_mask(st0);
	};

	// (A) READ ASSEMBLY: seed a read at xm_adr=base and pump until it delivers.
	auto pump_read = [&](u32 st0, u32 base) -> u32 {
		std::array<u32, 64> program{};   // all NOPs; they only pump xm_step_read
		program.fill(0);
		m_dsp->debug_load_program(program.data(), u32(program.size()), st0, 0);
		m_dsp->debug_set_exec_state(0x00, 0x00, 0x00, 0x00, 0x00, st0, 0, S_READ_BIT);
		// (xoa, xba, xwr, xrd, txrd, xm_adr, xm_cycles, xm_fetches)
		m_dsp->debug_set_xmem_state(0, 0, 0, 0, 0, base, 0, 0);
		m_dsp->debug_run_cycles(24);
		return m_dsp->debug_xrd();
	};
	// (B) WRITE ASSEMBLY: seed a write of xwr=sample at xm_adr=base and pump.
	auto pump_write = [&](u32 st0, u32 base, u32 sample24) {
		std::array<u32, 64> program{};
		program.fill(0);
		m_dsp->debug_load_program(program.data(), u32(program.size()), st0, 0);
		m_dsp->debug_set_exec_state(0x00, 0x00, 0x00, 0x00, 0x00, st0, 0, S_WRITE_BIT);
		m_dsp->debug_set_xmem_state(0, 0, sample24, 0, 0, base, 0, 0);
		m_dsp->debug_run_cycles(24);
	};
	// (C) ADDRESS ARITH: WRE op computes byte addr from cmem[0x7d]=xoa and xba.
	auto wre_addr_probe = [&](u32 st0, u32 xoa_word, u32 xba_word, u32 sample24) {
		std::array<u32, 64> program{};
		program.fill(0);
		program[0] = OP_WRE;
		m_dsp->debug_load_program(program.data(), u32(program.size()), st0, 0);
		m_dsp->debug_write_dmem0(0x00, sample24);
		m_dsp->debug_write_cmem(0x7d, xoa_word);
		m_dsp->debug_set_exec_state(0x00, 0x7d, 0x00, 0x00, 0x00, st0, 0, 0);
		// SMHD (co-issued) reads MACC; seed it like the shipped wre/smhd test.
		m_dsp->debug_set_accumulators(0, 0x0123456789012ULL, 0x0123456789012ULL, 0x0123456789012ULL, 0);
		m_dsp->debug_set_xmem_state(0, xba_word, 0, 0, 0, 0, 0, 0);
		m_dsp->debug_run_cycles(32);
	};

	// ==== (A) READ ASSEMBLY =================================================
	// A1. 16-bit mode: three ADJACENT words, distinct patterns -> catches byte
	//     swap, neighbor-word bleed, and the top-16 masking (low byte -> 0).
	{
		const u32 st0 = ST0_16BIT_256K;
		struct { u32 base; u8 hi; u8 lo; } w[] = {
			{ 0x100, 0x11, 0x33 }, { 0x102, 0xAB, 0xCD }, { 0x104, 0x55, 0xAA },
		};
		for (auto &c : w) { ext.write_byte(c.base, c.hi); ext.write_byte(c.base + 1, c.lo); }
		for (auto &c : w)
		{
			const u32 got = pump_read(st0, c.base);
			const u32 want = (u32(c.hi) << 16) | (u32(c.lo) << 8);
			osd_printf_info("XMEM readA 16b base=%05x bytes=%02x%02x read=%06x want=%06x\n",
				c.base, c.hi, c.lo, got, want);
			check_equal("xmem readA 16b lane+mask", s64(got), s64(want));
		}
	}
	// A2. 24-bit WORD+SEL mode: full 24-bit fidelity from three byte lanes.
	{
		const u32 st0 = ST0_24BIT_256K;
		const u32 base = 0x200;   // 4-byte aligned
		ext.write_byte(base + 0, 0xAB);
		ext.write_byte(base + 1, 0xCD);
		ext.write_byte(base + 2, 0xEF);
		const u32 got = pump_read(st0, base);
		osd_printf_info("XMEM readA 24b base=%05x bytes=ABCDEF read=%06x\n", base, got);
		check_equal("xmem readA 24b three-lane", s64(got), s64(0x00ABCDEF));
	}

	// ==== (B) WRITE ASSEMBLY ================================================
	// B1. 16-bit: write splits xwr[23:8] to two big-endian bytes, drops low byte.
	{
		const u32 st0 = ST0_16BIT_256K;
		const u32 base = 0x300;
		pump_write(st0, base, 0x00ABCDEF);
		const u8 b0 = ext.read_byte(base), b1 = ext.read_byte(base + 1);
		osd_printf_info("XMEM writeB 16b base=%05x wrote=ABCDEF -> bytes=%02x%02x\n", base, b0, b1);
		check_equal("xmem writeB 16b byte hi", s64(b0), s64(0xAB));
		check_equal("xmem writeB 16b byte mid", s64(b1), s64(0xCD));
	}
	// B2. 24-bit: three big-endian bytes carry the full word.
	{
		const u32 st0 = ST0_24BIT_256K;
		const u32 base = 0x304;
		pump_write(st0, base, 0x00123456);
		osd_printf_info("XMEM writeB 24b base=%05x wrote=123456 -> bytes=%02x%02x%02x\n",
			base, ext.read_byte(base), ext.read_byte(base + 1), ext.read_byte(base + 2));
		check_equal("xmem writeB 24b byte0", s64(ext.read_byte(base + 0)), s64(0x12));
		check_equal("xmem writeB 24b byte1", s64(ext.read_byte(base + 1)), s64(0x34));
		check_equal("xmem writeB 24b byte2", s64(ext.read_byte(base + 2)), s64(0x56));
	}

	// ==== (A)+(B) ROUND TRIP at LARGE addresses ============================
	// Write then read back at the very top of the 256K space (byte 0x3fffe).
	{
		const u32 st0 = ST0_16BIT_256K;
		const u32 base = 0x3fffe;   // highest 16-bit word in 256K
		pump_write(st0, base, 0x007F1200);
		const u32 got = pump_read(st0, base);
		osd_printf_info("XMEM rt 16b top base=%05x wrote=7F1200 read=%06x\n", base, got);
		check_equal("xmem rt 16b top-of-256K", s64(got), s64(0x007F1200 & 0xffff00));
	}

	// ==== (C) ADDRESS ARITHMETIC + CIRCULAR WRAP (via WRE) ==================
	// PAYLOAD: under the current general rule the co-issued store retires before
	// WRE latches its payload.  SMHD therefore stores the seeded MACC value into
	// DMEM first, and each WRE observes its high byte (0x12).  The five probes
	// still discriminate the computed addresses and wrap behavior.
	static constexpr u8 STORE_UPDATED_XWR_HIGH = 0x12;

	// C1. Sanity: xoa=1,xba=8 in 16-bit 256K -> byte 0x12.
	{
		const u32 st0 = ST0_16BIT_256K;
		const u32 a = want_addr(st0, 1, 8);
		ext.write_byte(a, 0x00);
		wre_addr_probe(st0, 1, 8, 0x00A5C300);
		osd_printf_info("XMEM addrC 16b xoa=1 xba=8 -> want=%05x landed@%05x=%02x\n",
			a, a, ext.read_byte(a));
		check_equal("xmem addrC 16b basic want addr", s64(a), s64(0x12));
		check_equal("xmem addrC 16b basic landed here", s64(ext.read_byte(a)), s64(STORE_UPDATED_XWR_HIGH));
	}
	// C2. Large address near top of 256K: word 0x1ffff -> byte 0x3fffe.
	{
		const u32 st0 = ST0_16BIT_256K;
		const u32 xoa = 1, xba = 0x1fffe;    // sum 0x1ffff
		const u32 a = want_addr(st0, xoa, xba);
		ext.write_byte(a, 0x00);
		wre_addr_probe(st0, xoa, xba, 0x007E0100);
		osd_printf_info("XMEM addrC 16b large xoa+xba=%05x -> want=%05x landed@%05x=%02x\n",
			xoa + xba, a, a, ext.read_byte(a));
		check_equal("xmem addrC 16b large want addr", s64(a), s64(0x3fffe));
		check_equal("xmem addrC 16b large landed here", s64(ext.read_byte(a)), s64(STORE_UPDATED_XWR_HIGH));
	}
	// C3. Wrap: (xoa+xba) overflows the 128K-word space -> aliases into low mem.
	//     word 0x20001 -> (0x20001<<1)&0x3ffff = byte 2. Decoy byte 0x40 must
	//     stay zero (no spurious placement).
	{
		const u32 st0 = ST0_16BIT_256K;
		const u32 xoa = 2, xba = 0x1ffff;    // sum 0x20001
		const u32 a = want_addr(st0, xoa, xba);
		ext.write_byte(0x02, 0x00); ext.write_byte(0x40, 0x00);
		wre_addr_probe(st0, xoa, xba, 0x00246800);
		osd_printf_info("XMEM addrC 16b wrap xoa+xba=%05x -> want=%05x landed@0002=%02x decoy@0040=%02x\n",
			xoa + xba, a, ext.read_byte(0x02), ext.read_byte(0x40));
		check_equal("xmem addrC 16b wrap want addr", s64(a), s64(0x00002));
		check_equal("xmem addrC 16b wrap landed low", s64(ext.read_byte(0x02)), s64(STORE_UPDATED_XWR_HIGH));
		check_equal("xmem addrC 16b wrap no spurious", s64(ext.read_byte(0x40)), s64(0x00));
	}
	// C4. xba full-range wrap continuity: the sync_w decrement wraps xba at
	//     0x80000 = 4 * the 0x20000-word physical space. A delay-line pointer
	//     that has wrapped (xba=0x7ffff) with offset must alias cleanly: word
	//     (1 + 0x7ffff) = 0x80000 -> byte 0 (== word 0). If it tore, byte != 0.
	{
		const u32 st0 = ST0_16BIT_256K;
		const u32 a = want_addr(st0, 1, 0x7ffff);
		ext.write_byte(0x00, 0x00);
		wre_addr_probe(st0, 1, 0x7ffff, 0x00CAFE00);
		osd_printf_info("XMEM addrC 16b xba-wrap want=%05x landed@0000=%02x\n", a, ext.read_byte(0x00));
		check_equal("xmem addrC 16b xba-wrap aliases word0", s64(a), s64(0x00000));
		check_equal("xmem addrC 16b xba-wrap landed", s64(ext.read_byte(0x00)), s64(STORE_UPDATED_XWR_HIGH));
	}
	// C5. Small (64K) memory wrap: word 0x8000 -> (0x8000<<1)&0xffff = byte 0.
	{
		const u32 st0 = ST0_16BIT_64K;
		const u32 a = want_addr(st0, 0x8000, 0);
		ext.write_byte(0x00, 0x00);
		wre_addr_probe(st0, 0x8000, 0, 0x00BE1100);
		osd_printf_info("XMEM addrC 64k wrap want=%05x landed@0000=%02x\n", a, ext.read_byte(0x00));
		check_equal("xmem addrC 64k wrap want addr", s64(a), s64(0x00000));
		check_equal("xmem addrC 64k wrap landed", s64(ext.read_byte(0x00)), s64(STORE_UPDATED_XWR_HIGH));
	}

	// ==== READ PIPELINE LATENCY ============================================
	// Seed a read, pump one instruction at a time, record the instruction
	// distance at which xrd is delivered and confirm it then holds.
	{
		const u32 st0 = ST0_16BIT_256K;
		const u32 base = 0x400;
		ext.write_byte(base, 0x5A); ext.write_byte(base + 1, 0xC3);
		const u32 want = 0x005AC300;

		std::array<u32, 64> program{};
		program.fill(0);
		m_dsp->debug_load_program(program.data(), u32(program.size()), st0, 0);
		m_dsp->debug_set_exec_state(0x00, 0x00, 0x00, 0x00, 0x00, st0, 0, S_READ_BIT);
		m_dsp->debug_set_xmem_state(0, 0, 0, 0, 0, base, 0, 0);

		int delivered_at = -1;
		for (int step = 1; step <= 16; step++)
		{
			m_dsp->debug_run_cycles(1);
			if (m_dsp->debug_xrd() == want && delivered_at < 0)
				delivered_at = step;
		}
		osd_printf_info("XMEM latency: value delivered at instruction distance %d, final xrd=%06x\n",
			delivered_at, m_dsp->debug_xrd());
		check_true("xmem read eventually delivered", delivered_at > 0);
		check_equal("xmem read value stable after delivery", s64(m_dsp->debug_xrd()), s64(want));
	}
}


void tms57002test_state::test_dsp3_a02_su480082_snapshot_replay()
{
	static constexpr char kSnapshotPath[] = "00-extracted/kprop_a02_replay_seed/snapshot_su480082_pc00_pre.log";
	static constexpr char kExecPath[] = "00-extracted/kprop_a02_replay_seed/execpost_su480082_pc00_10_sine.log";

	test_dsp3_snapshot_replay_case("dsp3 a02 su480082 replay", kSnapshotPath, kExecPath);
}

void tms57002test_state::test_dsp3_a02_su480006_snapshot_replay()
{
	static constexpr char kProgramPath[] = "00-extracted/kprop_a02_sine_live_dump/dump_dsp3.bin";
	static constexpr char kSnapshotPath[] = "00-extracted/kprop_a02_sine_sample_tape_long_20260408/snapshot_su480006_pc00_pre.log";
	static constexpr char kExecPath[] = "00-extracted/kprop_a02_sine_sample_tape_long_20260408/execpost_su480006_full.log";
	static constexpr char kRamPath[] = "00-extracted/kprop_a02_sine_sample_tape_long_20260408/ram/su480006_pc00_pre.bin";

	test_dsp3_snapshot_replay_case_with_program("dsp3 a02 sine su480006 replay", kProgramPath, kSnapshotPath, kExecPath, kRamPath);
}

void tms57002test_state::test_dsp3_a02_su480004_snapshot_replay()
{
	static constexpr char kProgramPath[] = "00-extracted/kprop_a02_sine_live_dump/dump_dsp3.bin";
	static constexpr char kSnapshotPath[] = "00-extracted/kprop_a02_sine_sample_tape_long_20260408/snapshot_su480004_pc00_pre.log";
	static constexpr char kExecPath[] = "00-extracted/kprop_a02_sine_sample_tape_long_20260408/execpost_su480004_full.log";
	static constexpr char kRamPath[] = "00-extracted/kprop_a02_sine_sample_tape_long_20260408/ram/su480004_pc00_pre.bin";

	test_dsp3_snapshot_replay_case_with_program("dsp3 a02 sine su480004 replay", kProgramPath, kSnapshotPath, kExecPath, kRamPath);
}

void tms57002test_state::test_dsp3_a02_sine_su480003_snapshot_replay()
{
	static constexpr char kProgramPath[] = "00-extracted/kprop_a02_sine_live_dump/dump_dsp3.bin";
	static constexpr char kSnapshotPath[] = "00-extracted/kprop_a02_sine_sample_tape_long_20260408/snapshot_su480003_pc00_pre.log";
	static constexpr char kExecPath[] = "00-extracted/kprop_a02_sine_sample_tape_long_20260408/execpost_su480003_full.log";
	static constexpr char kRamPath[] = "00-extracted/kprop_a02_sine_sample_tape_long_20260408/ram/su480003_pc00_pre.bin";

	test_dsp3_snapshot_replay_case_with_program("dsp3 a02 sine su480003 replay", kProgramPath, kSnapshotPath, kExecPath, kRamPath);
}

void tms57002test_state::test_dsp3_a02_su480005_snapshot_replay()
{
	static constexpr char kProgramPath[] = "00-extracted/kprop_a02_sine_live_dump/dump_dsp3.bin";
	static constexpr char kSnapshotPath[] = "00-extracted/kprop_a02_sine_sample_tape_long_20260408/snapshot_su480005_pc00_pre.log";
	static constexpr char kExecPath[] = "00-extracted/kprop_a02_sine_sample_tape_long_20260408/execpost_su480005_full.log";
	static constexpr char kRamPath[] = "00-extracted/kprop_a02_sine_sample_tape_long_20260408/ram/su480005_pc00_pre.bin";

	test_dsp3_snapshot_replay_case_with_program("dsp3 a02 sine su480005 replay", kProgramPath, kSnapshotPath, kExecPath, kRamPath);
}

void tms57002test_state::test_dsp3_input_sensitivity_case_with_program(const char *label, const char *program_path, const char *snapshot_path, const char *exec_path, const char *ram_path)
{
	const std::vector<u32> program = load_program_words(program_path);
	const tms57002_device::debug_snapshot base_snapshot = load_snapshot_from_log(snapshot_path);
	const std::vector<exec_post_expectation> expected = load_exec_post_expectations(exec_path);

	auto run_variant = [&](const char *variant, u32 input0)
	{
		tms57002_device::debug_snapshot snapshot = base_snapshot;
		snapshot.si[0] = input0;
		snapshot.serial_input_frame[0] = input0;
		snapshot.serial_input_prev_frame[0] = input0;
		m_dsp->debug_load_program(program.data(), u32(program.size()), snapshot.st0, snapshot.st1);
		load_data_space_bytes(ram_path, m_dsp->space(AS_DATA));
		m_dsp->debug_restore_snapshot(snapshot);
		for (std::size_t i = 0; i < expected.size(); i++)
			m_dsp->debug_run_cycles(1);
		osd_printf_info("INFO %s input %s so=%06x/%06x aacc=%08x macc=%013llx st1=%06x d5d=%06x d5f=%06x d56=%06x ba0=%02x\n",
			label,
			variant,
			m_dsp->debug_serial_output_register(0),
			m_dsp->debug_serial_output_register(1),
			u32(m_dsp->debug_aacc()),
			(unsigned long long)m_dsp->debug_macc(),
			m_dsp->state_int(TMS57002_ST1),
			m_dsp->dmem0_value(0x5d),
			m_dsp->dmem0_value(0x5f),
			m_dsp->dmem0_value(0x56),
			m_dsp->state_int(TMS57002_BA0));
	};

	run_variant("base", base_snapshot.si[0]);
	run_variant("zero", 0);
	run_variant("half", base_snapshot.si[0] >> 1);
	run_variant("posfs", 0x007fffff);
	run_variant("negfs", 0x00800000);
}

void tms57002test_state::test_dsp3_input_sensitivity_case(const char *label, const char *snapshot_path, const char *exec_path, const char *ram_path)
{
	static constexpr char kProgramPath[] = "00-extracted/korgprop_dsp_offline_extract/dsp3_init.bin";

	test_dsp3_input_sensitivity_case_with_program(label, kProgramPath, snapshot_path, exec_path, ram_path);
}

void tms57002test_state::test_dsp3_a02_su480002_input_sensitivity()
{
	static constexpr char kProgramPath[] = "00-extracted/kprop_a02_sine_live_dump/dump_dsp3.bin";
	static constexpr char kSnapshotPath[] = "00-extracted/kprop_a02_sine_sample_tape_480001_480006_20260408/extracted/snapshot_su480002_pc00_pre.log";
	static constexpr char kExecPath[] = "00-extracted/kprop_a02_sine_sample_tape_480001_480006_20260408/extracted/execpost_su480002_full.log";
	static constexpr char kRamPath[] = "00-extracted/kprop_a02_sine_sample_tape_480001_480006_20260408/ram/su480002_pc00_pre.bin";

	test_dsp3_input_sensitivity_case_with_program("dsp3 a02 sine su480002", kProgramPath, kSnapshotPath, kExecPath, kRamPath);
}

void tms57002test_state::test_dsp3_a02_su480003_input_sensitivity()
{
	static constexpr char kProgramPath[] = "00-extracted/kprop_a02_sine_live_dump/dump_dsp3.bin";
	static constexpr char kSnapshotPath[] = "00-extracted/kprop_a02_sine_sample_tape_480001_480006_20260408/extracted/snapshot_su480003_pc00_pre.log";
	static constexpr char kExecPath[] = "00-extracted/kprop_a02_sine_sample_tape_480001_480006_20260408/extracted/execpost_su480003_full.log";
	static constexpr char kRamPath[] = "00-extracted/kprop_a02_sine_sample_tape_480001_480006_20260408/ram/su480003_pc00_pre.bin";

	test_dsp3_input_sensitivity_case_with_program("dsp3 a02 sine su480003", kProgramPath, kSnapshotPath, kExecPath, kRamPath);
}

void tms57002test_state::test_dsp3_a02_su480004_input_sensitivity()
{
	static constexpr char kSnapshotPath[] = "tmp/kprop_a02_su480004_seed/error.log";
	static constexpr char kExecPath[] = "tmp/kprop_a02_su480004_seed/error.log";
	static constexpr char kRamPath[] = "tmp/kprop_a02_su480004_seed/xmem_su480004_pc00_pre_sine.bin";

	test_dsp3_input_sensitivity_case("dsp3 a02 su480004", kSnapshotPath, kExecPath, kRamPath);
}

void tms57002test_state::test_dsp3_a02_su480005_input_sensitivity()
{
	static constexpr char kSnapshotPath[] = "00-extracted/kprop_a02_firstbad_seed/snapshot_su480005_pc00_pre_sine.log";
	static constexpr char kExecPath[] = "00-extracted/kprop_a02_firstbad_seed/execpost_only_su480005_sine.log";
	static constexpr char kRamPath[] = "00-extracted/kprop_a02_firstbad_seed/xmem_su480005_pc00_pre_sine.bin";

	test_dsp3_input_sensitivity_case("dsp3 a02 su480005", kSnapshotPath, kExecPath, kRamPath);
}

void tms57002test_state::probe_dsp3_a02_su480005_state_sensitivity()
{
	static constexpr char kProgramPath[] = "00-extracted/korgprop_dsp_offline_extract/dsp3_init.bin";
	static constexpr char kSnapshotPath[] = "00-extracted/kprop_a02_firstbad_seed/snapshot_su480005_pc00_pre_sine.log";
	static constexpr char kPrevSnapshotPath[] = "tmp/kprop_a02_su480004_seed/error.log";
	static constexpr char kExecPath[] = "00-extracted/kprop_a02_firstbad_seed/execpost_only_su480005_sine.log";
	static constexpr char kRamPath[] = "00-extracted/kprop_a02_firstbad_seed/xmem_su480005_pc00_pre_sine.bin";

	const std::vector<u32> program = load_program_words(kProgramPath);
	const tms57002_device::debug_snapshot snapshot = load_snapshot_from_log(kSnapshotPath);
	const tms57002_device::debug_snapshot prev_snapshot = load_snapshot_from_log(kPrevSnapshotPath);
	const std::vector<exec_post_expectation> expected = load_exec_post_expectations(kExecPath);
	const int cycles_per_sample = int(expected.size());

	auto run_variant = [&](const char *label, const tms57002_device::debug_snapshot &base) -> void
	{
		m_dsp->debug_load_program(program.data(), u32(program.size()), base.st0, base.st1);
		load_data_space_bytes(kRamPath, m_dsp->space(AS_DATA));
		m_dsp->debug_restore_snapshot(base);
		for (int sample = 0; sample < 3; sample++)
		{
			for (int i = 0; i < cycles_per_sample; i++)
				m_dsp->debug_run_cycles(1);

			osd_printf_info(
				"INFO dsp3 a02 su480005 state %s sample+%d so=%06x/%06x c00=%08x c37=%08x c42=%08x d40=%06x d41=%06x d42=%06x d45=%06x d48=%06x d49=%06x df0=%06x df1=%06x df2=%06x df3=%06x\n",
				label,
				sample + 1,
				m_dsp->debug_serial_output_register(0),
				m_dsp->debug_serial_output_register(1),
				m_dsp->cmem_value(0x00),
				m_dsp->cmem_value(0x37),
				m_dsp->cmem_value(0x42),
				m_dsp->dmem0_value(0x40),
				m_dsp->dmem0_value(0x41),
				m_dsp->dmem0_value(0x42),
				m_dsp->dmem0_value(0x45),
				m_dsp->dmem0_value(0x48),
				m_dsp->dmem0_value(0x49),
				m_dsp->dmem0_value(0xf0),
				m_dsp->dmem0_value(0xf1),
				m_dsp->dmem0_value(0xf2),
				m_dsp->dmem0_value(0xf3));
		}
	};

	run_variant("base", snapshot);

	{
		tms57002_device::debug_snapshot variant = snapshot;
		variant.cmem[0x00] = prev_snapshot.cmem[0x00];
		run_variant("prev_c00", variant);
	}
	{
		tms57002_device::debug_snapshot variant = snapshot;
		variant.cmem[0x37] = prev_snapshot.cmem[0x37];
		run_variant("prev_c37", variant);
	}
	{
		tms57002_device::debug_snapshot variant = snapshot;
		variant.cmem[0x42] = prev_snapshot.cmem[0x42];
		run_variant("prev_c42", variant);
	}
	{
		tms57002_device::debug_snapshot variant = snapshot;
		variant.cmem[0x00] = prev_snapshot.cmem[0x00];
		variant.cmem[0x37] = prev_snapshot.cmem[0x37];
		variant.cmem[0x42] = prev_snapshot.cmem[0x42];
		run_variant("prev_c00_c37_c42", variant);
	}
	{
		tms57002_device::debug_snapshot variant = snapshot;
		for (u8 addr = 0x40; addr <= 0x4f; addr++)
			variant.dmem0[addr] = prev_snapshot.dmem0[addr];
		run_variant("prev_d40_4f", variant);
	}
	{
		tms57002_device::debug_snapshot variant = snapshot;
		for (u8 addr = 0xf0; addr <= 0xf3; addr++)
			variant.dmem0[addr] = prev_snapshot.dmem0[addr];
		run_variant("prev_df0_df3", variant);
	}
	{
		tms57002_device::debug_snapshot variant = snapshot;
		for (u8 addr = 0x40; addr <= 0x4f; addr++)
			variant.dmem0[addr] = prev_snapshot.dmem0[addr];
		for (u8 addr = 0xf0; addr <= 0xf3; addr++)
			variant.dmem0[addr] = prev_snapshot.dmem0[addr];
		run_variant("prev_hot_dmem", variant);
	}
	{
		tms57002_device::debug_snapshot variant = snapshot;
		variant.cmem[0x00] = prev_snapshot.cmem[0x00];
		variant.cmem[0x37] = prev_snapshot.cmem[0x37];
		variant.cmem[0x42] = prev_snapshot.cmem[0x42];
		for (u8 addr = 0x40; addr <= 0x4f; addr++)
			variant.dmem0[addr] = prev_snapshot.dmem0[addr];
		for (u8 addr = 0xf0; addr <= 0xf3; addr++)
			variant.dmem0[addr] = prev_snapshot.dmem0[addr];
		run_variant("prev_all_hot", variant);
	}
	{
		tms57002_device::debug_snapshot variant = snapshot;
		variant.ba0 = prev_snapshot.ba0;
		run_variant("prev_ba0", variant);
	}
	{
		tms57002_device::debug_snapshot variant = snapshot;
		variant.ba0 = snapshot.ba0 - 1;
		run_variant("next_ba0", variant);
	}
	{
		tms57002_device::debug_snapshot variant = snapshot;
		for (u8 addr = 0x40; addr <= 0x47; addr++)
			variant.dmem0[addr] = prev_snapshot.dmem0[addr];
		run_variant("prev_d40_47", variant);
	}
	{
		tms57002_device::debug_snapshot variant = snapshot;
		for (u8 addr = 0x48; addr <= 0x4f; addr++)
			variant.dmem0[addr] = prev_snapshot.dmem0[addr];
		run_variant("prev_d48_4f", variant);
	}
	{
		tms57002_device::debug_snapshot variant = snapshot;
		for (u8 addr : { u8(0x41), u8(0x42), u8(0x45) })
			variant.dmem0[addr] = prev_snapshot.dmem0[addr];
		run_variant("prev_d41_42_45", variant);
	}
	{
		tms57002_device::debug_snapshot variant = snapshot;
		for (u8 addr : { u8(0x48), u8(0x49), u8(0x4c) })
			variant.dmem0[addr] = prev_snapshot.dmem0[addr];
		run_variant("prev_d48_49_4c", variant);
	}
	for (u8 addr : { u8(0x41), u8(0x42), u8(0x45), u8(0x48), u8(0x49), u8(0x4c) })
	{
		tms57002_device::debug_snapshot variant = snapshot;
		variant.dmem0[addr] = prev_snapshot.dmem0[addr];
		char label[32];
		std::snprintf(label, sizeof(label), "prev_d%02x", addr);
		run_variant(label, variant);
	}
}

void tms57002test_state::probe_dsp3_a02_su480005_first_diff_variants()
{
	static constexpr char kProgramPath[] = "00-extracted/korgprop_dsp_offline_extract/dsp3_init.bin";
	static constexpr char kSnapshotPath[] = "00-extracted/kprop_a02_firstbad_seed/snapshot_su480005_pc00_pre_sine.log";
	static constexpr char kPrevSnapshotPath[] = "tmp/kprop_a02_su480004_seed/error.log";
	static constexpr char kRamPath[] = "00-extracted/kprop_a02_firstbad_seed/xmem_su480005_pc00_pre_sine.bin";
	static constexpr int kCyclesPerSample = 512;

	struct replay_step
	{
		u8 pc = 0;
		u8 ca = 0;
		u8 id = 0;
		u32 st1 = 0;
		u32 aacc = 0;
		s64 macc = 0;
		u32 creg = 0;
		u32 so0 = 0;
		u32 so1 = 0;
		u32 c00 = 0;
		u32 c37 = 0;
		u32 c42 = 0;
		u32 d42 = 0;
		u32 d45 = 0;
		u32 d49 = 0;
		u32 d4c = 0;
	};

	auto capture = [&](const tms57002_device::debug_snapshot &base) -> std::vector<replay_step>
	{
		const std::vector<u32> program = load_program_words(kProgramPath);
		if (program.size() != 0x100)
			throw emu_fatalerror("Unexpected DSP3 program size %zu in %s", program.size(), kProgramPath);

		m_dsp->debug_load_program(program.data(), u32(program.size()), base.st0, base.st1);
		load_data_space_bytes(kRamPath, m_dsp->space(AS_DATA));
		m_dsp->debug_restore_snapshot(base);

		std::vector<replay_step> out;
		out.reserve(kCyclesPerSample);
		for (int i = 0; i < kCyclesPerSample; i++)
		{
			m_dsp->debug_run_cycles(1);
			replay_step step;
			step.pc = u8(m_dsp->state_int(TMS57002_PC));
			step.ca = u8(m_dsp->state_int(TMS57002_CA));
			step.id = u8(m_dsp->state_int(TMS57002_ID));
			step.st1 = m_dsp->state_int(TMS57002_ST1);
			step.aacc = u32(m_dsp->debug_aacc());
			step.macc = s64(m_dsp->debug_macc());
			step.creg = m_dsp->debug_creg();
			step.so0 = m_dsp->debug_serial_output_register(0);
			step.so1 = m_dsp->debug_serial_output_register(1);
			step.c00 = m_dsp->cmem_value(0x00);
			step.c37 = m_dsp->cmem_value(0x37);
			step.c42 = m_dsp->cmem_value(0x42);
			step.d42 = m_dsp->dmem0_value(0x42);
			step.d45 = m_dsp->dmem0_value(0x45);
			step.d49 = m_dsp->dmem0_value(0x49);
			step.d4c = m_dsp->dmem0_value(0x4c);
			out.push_back(step);
		}
		return out;
	};

	auto report_first_diff = [&](const char *label, u8 ignored_addr, const std::vector<replay_step> &base, const std::vector<replay_step> &variant) -> void
	{
		for (std::size_t i = 0; i < base.size() && i < variant.size(); i++)
		{
			const replay_step &a = base[i];
			const replay_step &b = variant[i];
			const bool d42_diff = (ignored_addr != 0x42) && (a.d42 != b.d42);
			const bool d45_diff = (ignored_addr != 0x45) && (a.d45 != b.d45);
			const bool d49_diff = (ignored_addr != 0x49) && (a.d49 != b.d49);
			const bool d4c_diff = (ignored_addr != 0x4c) && (a.d4c != b.d4c);
			if (a.pc != b.pc || a.ca != b.ca || a.id != b.id || a.st1 != b.st1 || a.aacc != b.aacc || a.macc != b.macc || a.creg != b.creg || a.so0 != b.so0 || a.so1 != b.so1 || a.c00 != b.c00 || a.c37 != b.c37 || a.c42 != b.c42 || d42_diff || d45_diff || d49_diff || d4c_diff)
			{
				osd_printf_info(
					"INFO dsp3 a02 su480005 firstdiff %s step=%zu pc=%02x/%02x ca=%02x/%02x id=%02x/%02x st1=%06x/%06x aacc=%08x/%08x macc=%013llx/%013llx creg=%08x/%08x so=%06x/%06x vs %06x/%06x c00=%08x/%08x c37=%08x/%08x c42=%08x/%08x d42=%06x/%06x d45=%06x/%06x d49=%06x/%06x d4c=%06x/%06x\n",
					label,
					i + 1,
					a.pc, b.pc,
					a.ca, b.ca,
					a.id, b.id,
					a.st1, b.st1,
					a.aacc, b.aacc,
					(unsigned long long)a.macc, (unsigned long long)b.macc,
					a.creg, b.creg,
					a.so0, a.so1, b.so0, b.so1,
					a.c00, b.c00,
					a.c37, b.c37,
					a.c42, b.c42,
					a.d42, b.d42,
					a.d45, b.d45,
					a.d49, b.d49,
					a.d4c, b.d4c);
				return;
			}
		}

		osd_printf_info("INFO dsp3 a02 su480005 firstdiff %s none\n", label);
	};

	const tms57002_device::debug_snapshot snapshot = load_snapshot_from_log(kSnapshotPath);
	const tms57002_device::debug_snapshot prev_snapshot = load_snapshot_from_log(kPrevSnapshotPath);
	const std::vector<replay_step> base = capture(snapshot);

	for (u8 addr : { u8(0x42), u8(0x45), u8(0x49), u8(0x4c) })
	{
		tms57002_device::debug_snapshot variant = snapshot;
		variant.dmem0[addr] = prev_snapshot.dmem0[addr];
		char label[32];
		std::snprintf(label, sizeof(label), "prev_d%02x", addr);
		report_first_diff(label, addr, base, capture(variant));
	}
}

void tms57002test_state::probe_dsp3_a02_su480004_producers()
{
	static constexpr char kProgramPath[] = "00-extracted/korgprop_dsp_offline_extract/dsp3_init.bin";
	static constexpr char kSnapshotPath[] = "tmp/kprop_a02_su480004_seed/error.log";
	static constexpr char kRamPath[] = "tmp/kprop_a02_su480004_seed/xmem_su480004_pc00_pre_sine.bin";
	static constexpr int kCyclesPerSample = 512;

	const std::vector<u32> program = load_program_words(kProgramPath);
	const tms57002_device::debug_snapshot snapshot = load_snapshot_from_log(kSnapshotPath);

	if (program.size() != 0x100)
		throw emu_fatalerror("Unexpected DSP3 program size %zu in %s", program.size(), kProgramPath);

	m_dsp->debug_load_program(program.data(), u32(program.size()), snapshot.st0, snapshot.st1);
	load_data_space_bytes(kRamPath, m_dsp->space(AS_DATA));
	m_dsp->debug_restore_snapshot(snapshot);

	for (int i = 0; i < kCyclesPerSample; i++)
	{
		const u8 execpc = u8(m_dsp->state_int(TMS57002_PC));
		const u8 execid = u8(m_dsp->state_int(TMS57002_ID));
		m_dsp->debug_run_cycles(1);

		if (execid != 0x1f)
			continue;

		u8 phys = 0xff;
		switch (execpc)
		{
		case 0xb0: phys = 0x42; break; // d53 with BA0=EF
		case 0xb6: phys = 0x45; break; // d56 with BA0=EF
		case 0xf5: phys = 0x49; break; // d5a with BA0=EF
		case 0xfb: phys = 0x4c; break; // d5d with BA0=EF
		default: break;
		}

		if (phys != 0xff)
		{
			const u64 macc = m_dsp->debug_macc();
			const u64 maccr = m_dsp->debug_macc_read();
			const u64 maccw = m_dsp->debug_macc_write();
			const u32 st1 = u32(m_dsp->state_int(TMS57002_ST1));
			const int sfmo_mode = int((st1 & 0x001800U) >> 11);
			const int rnd_mode = int((st1 & 0x038000U) >> 15);
			const auto mr_none = m_dsp->debug_eval_macc_output(maccr, sfmo_mode, rnd_mode, tms57002_device::debug_macc_clip_mode::none);
			const auto mr_round = m_dsp->debug_eval_macc_output(maccr, sfmo_mode, rnd_mode, tms57002_device::debug_macc_clip_mode::rounded);
			const auto mr_unrnd = m_dsp->debug_eval_macc_output(maccr, sfmo_mode, rnd_mode, tms57002_device::debug_macc_clip_mode::unrounded);
			const auto mw_none = m_dsp->debug_eval_macc_output(maccw, sfmo_mode, rnd_mode, tms57002_device::debug_macc_clip_mode::none);
			const auto mm_none = m_dsp->debug_eval_macc_output(macc, sfmo_mode, rnd_mode, tms57002_device::debug_macc_clip_mode::none);
			osd_printf_info(
				"INFO dsp3 a02 su480004 producer pc=%02x ba0=%02x phys=%02x val=%06x st1=%06x sfmo=%d rnd=%d "
				"macc=%013llx m24=%06x maccr=%013llx r24=%06x maccw=%013llx w24=%06x "
				"rnone=%06x rround=%06x runrnd=%06x wnone=%06x mnone=%06x so=%06x/%06x\n",
				execpc,
				m_dsp->debug_ba0(),
				phys,
				m_dsp->dmem0_value(phys),
				st1,
				sfmo_mode,
				rnd_mode,
				(unsigned long long)macc, u32((macc >> 24) & 0xffffff),
				(unsigned long long)maccr, u32((maccr >> 24) & 0xffffff),
				(unsigned long long)maccw, u32((maccw >> 24) & 0xffffff),
				u32((mr_none.value >> 24) & 0xffffff),
				u32((mr_round.value >> 24) & 0xffffff),
				u32((mr_unrnd.value >> 24) & 0xffffff),
				u32((mw_none.value >> 24) & 0xffffff),
				u32((mm_none.value >> 24) & 0xffffff),
				m_dsp->debug_serial_output_register(0),
				m_dsp->debug_serial_output_register(1));
		}
	}
}

void tms57002test_state::probe_dsp3_a02_su480004_producer_site_ba0_variants()
{
	static constexpr char kProgramPath[] = "00-extracted/korgprop_dsp_offline_extract/dsp3_init.bin";
	static constexpr char kSnapshotPath[] = "tmp/kprop_a02_su480004_seed/error.log";
	static constexpr char kRamPath[] = "tmp/kprop_a02_su480004_seed/xmem_su480004_pc00_pre_sine.bin";
	static constexpr int kCyclesPerSample = 512;

	const std::vector<u32> program = load_program_words(kProgramPath);
	const tms57002_device::debug_snapshot snapshot = load_snapshot_from_log(kSnapshotPath);

	auto run_variant = [&](const char *label, u8 target_pc, int ba0_adjust, bool repeat) -> void
	{
		m_dsp->debug_load_program(program.data(), u32(program.size()), snapshot.st0, snapshot.st1);
		load_data_space_bytes(kRamPath, m_dsp->space(AS_DATA));
		m_dsp->debug_restore_snapshot(snapshot);

		bool patched_once = false;
		bool restore_next = false;
		for (int step = 0; step < kCyclesPerSample * 4; step++)
		{
			if ((!patched_once || repeat) && m_dsp->state_int(TMS57002_ID) == 0x1f && m_dsp->state_int(TMS57002_PC) == target_pc)
			{
				u8 new_ba0 = snapshot.ba0;
				u8 new_ba1 = snapshot.ba1;
				if (ba0_adjust < 0)
				{
					new_ba0 = snapshot.ba0 + 1;
					new_ba1 = snapshot.ba1 - 1;
				}
				else if (ba0_adjust > 0)
				{
					new_ba0 = snapshot.ba0 - 1;
					new_ba1 = snapshot.ba1 + 1;
				}
				m_dsp->debug_set_banks(new_ba0, new_ba1);
				patched_once = true;
				restore_next = true;
			}

			m_dsp->debug_run_cycles(1);

			if (restore_next)
			{
				m_dsp->debug_set_banks(snapshot.ba0, snapshot.ba1);
				restore_next = false;
			}

			if ((step + 1) % kCyclesPerSample == 0)
			{
				const int sample = (step + 1) / kCyclesPerSample;
				osd_printf_info(
					"INFO dsp3 a02 su480004 producersite %s pc=%02x sample+%d so=%06x/%06x c42=%08x d42=%06x d45=%06x d49=%06x d4c=%06x\n",
					label,
					target_pc,
					sample,
					m_dsp->debug_serial_output_register(0),
					m_dsp->debug_serial_output_register(1),
					m_dsp->cmem_value(0x42),
					m_dsp->dmem0_value(0x42),
					m_dsp->dmem0_value(0x45),
					m_dsp->dmem0_value(0x49),
					m_dsp->dmem0_value(0x4c));
			}
		}
	};

	for (u8 pc : { u8(0xb6), u8(0xfb) })
	{
		run_variant("prev_once", pc, -1, false);
		run_variant("next_once", pc, +1, false);
		run_variant("prev_each", pc, -1, true);
		run_variant("next_each", pc, +1, true);
	}
}

void tms57002test_state::probe_dsp3_a02_su480005_ba0_phase_sensitivity()
{
	static constexpr char kProgramPath[] = "00-extracted/korgprop_dsp_offline_extract/dsp3_init.bin";
	static constexpr char kSnapshotPath[] = "00-extracted/kprop_a02_firstbad_seed/snapshot_su480005_pc00_pre_sine.log";
	static constexpr char kPrevSnapshotPath[] = "tmp/kprop_a02_su480004_seed/error.log";
	static constexpr char kRamPath[] = "00-extracted/kprop_a02_firstbad_seed/xmem_su480005_pc00_pre_sine.bin";
	static constexpr int kCyclesPerSample = 512;

	const std::vector<u32> program = load_program_words(kProgramPath);
	const tms57002_device::debug_snapshot snapshot = load_snapshot_from_log(kSnapshotPath);
	const tms57002_device::debug_snapshot prev_snapshot = load_snapshot_from_log(kPrevSnapshotPath);

	auto run_variant = [&](const char *label, int ba0_adjust) -> void
	{
		m_dsp->debug_load_program(program.data(), u32(program.size()), snapshot.st0, snapshot.st1);
		load_data_space_bytes(kRamPath, m_dsp->space(AS_DATA));
		m_dsp->debug_restore_snapshot(snapshot);

		bool patched = false;
		for (int step = 0; step < kCyclesPerSample * 3; step++)
		{
			if (!patched && m_dsp->state_int(TMS57002_PC) == 0x00 && m_dsp->state_int(TMS57002_ID) == 0x1f)
			{
				u8 new_ba0 = snapshot.ba0;
				u8 new_ba1 = snapshot.ba1;
				if (ba0_adjust < 0)
				{
					new_ba0 = prev_snapshot.ba0;
					new_ba1 = prev_snapshot.ba1;
				}
				else if (ba0_adjust > 0)
				{
					new_ba0 = snapshot.ba0 - 1;
					new_ba1 = snapshot.ba1 + 1;
				}

				m_dsp->debug_set_exec_state(
					u8(m_dsp->state_int(TMS57002_PC)),
					u8(m_dsp->state_int(TMS57002_CA)),
					u8(m_dsp->state_int(TMS57002_ID)),
					new_ba0,
					new_ba1,
					m_dsp->state_int(TMS57002_ST0),
					m_dsp->state_int(TMS57002_ST1),
					snapshot.sti);
				patched = true;
			}

			m_dsp->debug_run_cycles(1);

			if ((step + 1) % kCyclesPerSample == 0)
			{
				const int sample = (step + 1) / kCyclesPerSample;
				osd_printf_info(
					"INFO dsp3 a02 su480005 ba0phase %s sample+%d so=%06x/%06x ba0=%02x ba1=%02x c00=%08x c37=%08x c42=%08x d42=%06x d45=%06x d49=%06x d4c=%06x\n",
					label,
					sample,
					m_dsp->debug_serial_output_register(0),
					m_dsp->debug_serial_output_register(1),
					m_dsp->debug_ba0(),
					u8(m_dsp->state_int(TMS57002_BA1)),
					m_dsp->cmem_value(0x00),
					m_dsp->cmem_value(0x37),
					m_dsp->cmem_value(0x42),
					m_dsp->dmem0_value(0x42),
					m_dsp->dmem0_value(0x45),
					m_dsp->dmem0_value(0x49),
					m_dsp->dmem0_value(0x4c));
			}
		}
	};

	run_variant("base", 0);
	run_variant("prev_on_id1f", -1);
	run_variant("next_on_id1f", +1);
}

void tms57002test_state::probe_dsp3_a02_su480005_site_ba0_variants()
{
	static constexpr char kProgramPath[] = "00-extracted/korgprop_dsp_offline_extract/dsp3_init.bin";
	static constexpr char kSnapshotPath[] = "00-extracted/kprop_a02_firstbad_seed/snapshot_su480005_pc00_pre_sine.log";
	static constexpr char kPrevSnapshotPath[] = "tmp/kprop_a02_su480004_seed/error.log";
	static constexpr char kRamPath[] = "00-extracted/kprop_a02_firstbad_seed/xmem_su480005_pc00_pre_sine.bin";
	static constexpr int kCyclesPerSample = 512;

	const std::vector<u32> program = load_program_words(kProgramPath);
	const tms57002_device::debug_snapshot snapshot = load_snapshot_from_log(kSnapshotPath);
	const tms57002_device::debug_snapshot prev_snapshot = load_snapshot_from_log(kPrevSnapshotPath);

	auto run_variant = [&](const char *label, u8 target_pc, int ba0_adjust) -> void
	{
		m_dsp->debug_load_program(program.data(), u32(program.size()), snapshot.st0, snapshot.st1);
		load_data_space_bytes(kRamPath, m_dsp->space(AS_DATA));
		m_dsp->debug_restore_snapshot(snapshot);

		bool patched = false;
		bool restore_next = false;
		for (int step = 0; step < kCyclesPerSample * 2; step++)
		{
			if (!patched && m_dsp->state_int(TMS57002_ID) == 0x1f && m_dsp->state_int(TMS57002_PC) == target_pc)
			{
				u8 new_ba0 = snapshot.ba0;
				u8 new_ba1 = snapshot.ba1;
				if (ba0_adjust < 0)
				{
					new_ba0 = prev_snapshot.ba0;
					new_ba1 = prev_snapshot.ba1;
				}
				else if (ba0_adjust > 0)
				{
					new_ba0 = snapshot.ba0 - 1;
					new_ba1 = snapshot.ba1 + 1;
				}
				m_dsp->debug_set_banks(new_ba0, new_ba1);
				patched = true;
				restore_next = true;
			}

			m_dsp->debug_run_cycles(1);

			if (restore_next)
			{
				m_dsp->debug_set_banks(snapshot.ba0, snapshot.ba1);
				restore_next = false;
			}

			if ((step + 1) % kCyclesPerSample == 0)
			{
				const int sample = (step + 1) / kCyclesPerSample;
				osd_printf_info(
					"INFO dsp3 a02 su480005 siteba0 %s pc=%02x sample+%d so=%06x/%06x c00=%08x c37=%08x c42=%08x d42=%06x d45=%06x d49=%06x d4c=%06x\n",
					label,
					target_pc,
					sample,
					m_dsp->debug_serial_output_register(0),
					m_dsp->debug_serial_output_register(1),
					m_dsp->cmem_value(0x00),
					m_dsp->cmem_value(0x37),
					m_dsp->cmem_value(0x42),
					m_dsp->dmem0_value(0x42),
					m_dsp->dmem0_value(0x45),
					m_dsp->dmem0_value(0x49),
					m_dsp->dmem0_value(0x4c));
			}
		}
	};

	for (u8 pc : { u8(0xad), u8(0xb3), u8(0xf4), u8(0xf6) })
	{
		run_variant("prev_once", pc, -1);
		run_variant("next_once", pc, +1);
	}
}

void tms57002test_state::probe_dsp3_a02_su480005_repeat_site_ba0_variants()
{
	static constexpr char kProgramPath[] = "00-extracted/korgprop_dsp_offline_extract/dsp3_init.bin";
	static constexpr char kSnapshotPath[] = "00-extracted/kprop_a02_firstbad_seed/snapshot_su480005_pc00_pre_sine.log";
	static constexpr char kPrevSnapshotPath[] = "tmp/kprop_a02_su480004_seed/error.log";
	static constexpr char kRamPath[] = "00-extracted/kprop_a02_firstbad_seed/xmem_su480005_pc00_pre_sine.bin";
	static constexpr int kCyclesPerSample = 512;

	const std::vector<u32> program = load_program_words(kProgramPath);
	const tms57002_device::debug_snapshot snapshot = load_snapshot_from_log(kSnapshotPath);
	const tms57002_device::debug_snapshot prev_snapshot = load_snapshot_from_log(kPrevSnapshotPath);

	auto run_variant = [&](const char *label, u8 target_pc, int ba0_adjust) -> void
	{
		m_dsp->debug_load_program(program.data(), u32(program.size()), snapshot.st0, snapshot.st1);
		load_data_space_bytes(kRamPath, m_dsp->space(AS_DATA));
		m_dsp->debug_restore_snapshot(snapshot);

		bool restore_next = false;
		for (int step = 0; step < kCyclesPerSample * 4; step++)
		{
			if (m_dsp->state_int(TMS57002_ID) == 0x1f && m_dsp->state_int(TMS57002_PC) == target_pc)
			{
				u8 new_ba0 = snapshot.ba0;
				u8 new_ba1 = snapshot.ba1;
				if (ba0_adjust < 0)
				{
					new_ba0 = prev_snapshot.ba0;
					new_ba1 = prev_snapshot.ba1;
				}
				else if (ba0_adjust > 0)
				{
					new_ba0 = snapshot.ba0 - 1;
					new_ba1 = snapshot.ba1 + 1;
				}
				m_dsp->debug_set_banks(new_ba0, new_ba1);
				restore_next = true;
			}

			m_dsp->debug_run_cycles(1);

			if (restore_next)
			{
				m_dsp->debug_set_banks(snapshot.ba0, snapshot.ba1);
				restore_next = false;
			}

			if ((step + 1) % kCyclesPerSample == 0)
			{
				const int sample = (step + 1) / kCyclesPerSample;
				osd_printf_info(
					"INFO dsp3 a02 su480005 repeatsite %s pc=%02x sample+%d so=%06x/%06x c00=%08x c37=%08x c42=%08x d42=%06x d45=%06x d49=%06x d4c=%06x\n",
					label,
					target_pc,
					sample,
					m_dsp->debug_serial_output_register(0),
					m_dsp->debug_serial_output_register(1),
					m_dsp->cmem_value(0x00),
					m_dsp->cmem_value(0x37),
					m_dsp->cmem_value(0x42),
					m_dsp->dmem0_value(0x42),
					m_dsp->dmem0_value(0x45),
					m_dsp->dmem0_value(0x49),
					m_dsp->dmem0_value(0x4c));
			}
		}
	};

	for (u8 pc : { u8(0xb3), u8(0xf6) })
	{
		run_variant("prev_each", pc, -1);
		run_variant("next_each", pc, +1);
	}
}

void tms57002test_state::probe_dsp3_a02_tape_su480003_group_sensitivity()
{
	static constexpr char kProgramPath[] = "00-extracted/kprop_a02_sine_live_dump/dump_dsp3.bin";
	static constexpr char kSineLogPath[] = "00-extracted/kprop_a02_sine_sample_tape_480001_480006_20260408/extracted/snapshot_su480003_pc00_pre.log";
	static constexpr char kImpulseLogPath[] = "00-extracted/kprop_a02_impulse_sample_tape_long_20260408/extracted/snapshot_su480003_pc00_pre.log";
	static constexpr char kSineRamPath[] = "00-extracted/kprop_a02_sine_sample_tape_480001_480006_20260408/ram/su480003_pc00_pre.bin";
	static constexpr int kCyclesPerSample = 512;

	const std::vector<u32> program = load_program_words(kProgramPath);
	const tms57002_device::debug_snapshot sine = load_snapshot_from_log(kSineLogPath);
	const tms57002_device::debug_snapshot impulse = load_snapshot_from_log(kImpulseLogPath);

	auto phys = [&](u8 logical) -> u8 { return u8(logical + sine.ba0); };

	auto run_variant = [&](const char *label, const tms57002_device::debug_snapshot &base) -> void
	{
		m_dsp->debug_load_program(program.data(), u32(program.size()), base.st0, base.st1);
		load_data_space_bytes(kSineRamPath, m_dsp->space(AS_DATA));
		m_dsp->debug_restore_snapshot(base);
		for (int sample = 0; sample < 4; sample++)
		{
			for (int i = 0; i < kCyclesPerSample; i++)
				m_dsp->debug_run_cycles(1);
			osd_printf_info(
				"INFO dsp3 a02 tapegroup %s sample+%d so=%06x/%06x ba0=%02x d02=%06x d03=%06x d04=%06x d05=%06x d06=%06x d07=%06x d20=%06x d21=%06x d22=%06x d31=%06x d32=%06x d33=%06x d51=%06x d52=%06x d54=%06x d57=%06x\n",
				label,
				sample + 1,
				m_dsp->debug_serial_output_register(0),
				m_dsp->debug_serial_output_register(1),
				m_dsp->debug_ba0(),
				m_dsp->dmem0_value(phys(0x02)),
				m_dsp->dmem0_value(phys(0x03)),
				m_dsp->dmem0_value(phys(0x04)),
				m_dsp->dmem0_value(phys(0x05)),
				m_dsp->dmem0_value(phys(0x06)),
				m_dsp->dmem0_value(phys(0x07)),
				m_dsp->dmem0_value(phys(0x20)),
				m_dsp->dmem0_value(phys(0x21)),
				m_dsp->dmem0_value(phys(0x22)),
				m_dsp->dmem0_value(phys(0x31)),
				m_dsp->dmem0_value(phys(0x32)),
				m_dsp->dmem0_value(phys(0x33)),
				m_dsp->dmem0_value(phys(0x51)),
				m_dsp->dmem0_value(phys(0x52)),
				m_dsp->dmem0_value(phys(0x54)),
				m_dsp->dmem0_value(phys(0x57)));
		}
	};

	run_variant("sine_base", sine);
	run_variant("impulse_base", impulse);

	for (auto group : {
			std::vector<u8>{0x02,0x03,0x04,0x05,0x06,0x07},
			std::vector<u8>{0x20,0x21,0x22,0x23,0x24},
			std::vector<u8>{0x31,0x32,0x33,0x34,0x35,0x36},
			std::vector<u8>{0x51,0x52,0x53,0x54,0x55,0x57},
			std::vector<u8>{0x51},
			std::vector<u8>{0x52},
			std::vector<u8>{0x54},
			std::vector<u8>{0x55},
			std::vector<u8>{0x57},
			std::vector<u8>{0x53},
			std::vector<u8>{0x56},
			std::vector<u8>{0x51,0x54,0x57},
			std::vector<u8>{0x54,0x55,0x57},
			std::vector<u8>{0xc1,0xc2,0xc3,0xc4,0xc5} })
	{
		tms57002_device::debug_snapshot variant = sine;
		char label[64];
		std::snprintf(label, sizeof(label), "imp_");
		std::size_t off = std::strlen(label);
		for (std::size_t i = 0; i < group.size(); i++)
		{
			const u8 logical = group[i];
			variant.dmem0[(logical + variant.ba0) & 0xff] = impulse.dmem0[(logical + impulse.ba0) & 0xff];
			off += std::snprintf(label + off, sizeof(label) - off, i ? "_%02x" : "%02x", logical);
		}
		run_variant(label, variant);
	}
}

void tms57002test_state::probe_dsp3_a02_tape_su480003_producer_sensitivity()
{
	static constexpr char kProgramPath[] = "00-extracted/kprop_a02_sine_live_dump/dump_dsp3.bin";
	static constexpr char kSineLogPath[] = "00-extracted/kprop_a02_sine_sample_tape_480001_480006_20260408/extracted/snapshot_su480003_pc00_pre.log";
	static constexpr char kImpulseLogPath[] = "00-extracted/kprop_a02_impulse_sample_tape_long_20260408/extracted/snapshot_su480003_pc00_pre.log";
	static constexpr char kSineRamPath[] = "00-extracted/kprop_a02_sine_sample_tape_480001_480006_20260408/ram/su480003_pc00_pre.bin";
	static constexpr int kCyclesPerSample = 512;

	const std::vector<u32> program = load_program_words(kProgramPath);
	const tms57002_device::debug_snapshot sine = load_snapshot_from_log(kSineLogPath);
	const tms57002_device::debug_snapshot impulse = load_snapshot_from_log(kImpulseLogPath);

	auto copy_logical = [](tms57002_device::debug_snapshot &dst, const tms57002_device::debug_snapshot &src, std::initializer_list<u8> logicals) -> void
	{
		for (const u8 logical : logicals)
			dst.dmem0[(logical + dst.ba0) & 0xff] = src.dmem0[(logical + src.ba0) & 0xff];
	};

	auto run_variant = [&](const char *label, const tms57002_device::debug_snapshot &base) -> void
	{
		m_dsp->debug_load_program(program.data(), u32(program.size()), base.st0, base.st1);
		load_data_space_bytes(kSineRamPath, m_dsp->space(AS_DATA));
		m_dsp->debug_restore_snapshot(base);

		bool logged_b1 = false;
		bool logged_b7 = false;
		for (int step = 0; step < kCyclesPerSample * 4; step++)
		{
			m_dsp->debug_run_cycles(1);

			const int sample_index = step / kCyclesPerSample;
			if (sample_index == 0)
			{
				const u8 pc = u8(m_dsp->state_int(TMS57002_PC));
				const u8 id = u8(m_dsp->state_int(TMS57002_ID));
				const u8 ba0 = m_dsp->debug_ba0();
				auto logical = [&](u8 addr) { return m_dsp->dmem0_value((addr + ba0) & 0xff); };

				if (!logged_b1 && pc == 0xb1)
				{
					logged_b1 = true;
					osd_printf_info(
						"INFO dsp3 a02 tapeprodmid %s site=b0 id=%02x ba0=%02x aacc=%08x macc=%013llx so=%06x/%06x d50=%06x d51=%06x d52=%06x d53=%06x d54=%06x d55=%06x\n",
						label,
						id,
						ba0,
						u32(m_dsp->debug_aacc()),
						(unsigned long long)m_dsp->debug_macc(),
						m_dsp->debug_serial_output_register(0),
						m_dsp->debug_serial_output_register(1),
						logical(0x50),
						logical(0x51),
						logical(0x52),
						logical(0x53),
						logical(0x54),
						logical(0x55));
				}
				if (!logged_b7 && pc == 0xb7)
				{
					logged_b7 = true;
					osd_printf_info(
						"INFO dsp3 a02 tapeprodmid %s site=b6 id=%02x ba0=%02x aacc=%08x macc=%013llx so=%06x/%06x d53=%06x d54=%06x d55=%06x d56=%06x d57=%06x d58=%06x\n",
						label,
						id,
						ba0,
						u32(m_dsp->debug_aacc()),
						(unsigned long long)m_dsp->debug_macc(),
						m_dsp->debug_serial_output_register(0),
						m_dsp->debug_serial_output_register(1),
						logical(0x53),
						logical(0x54),
						logical(0x55),
						logical(0x56),
						logical(0x57),
						logical(0x58));
				}
			}

			if ((step + 1) % kCyclesPerSample == 0)
			{
				const int sample = (step + 1) / kCyclesPerSample;
				const u8 ba0 = m_dsp->debug_ba0();
				auto logical = [&](u8 addr) { return m_dsp->dmem0_value((addr + ba0) & 0xff); };
				osd_printf_info(
					"INFO dsp3 a02 tapeprod %s sample+%d so=%06x/%06x ba0=%02x aacc=%08x macc=%013llx d50=%06x d51=%06x d52=%06x d53=%06x d54=%06x d55=%06x d56=%06x d57=%06x d58=%06x\n",
					label,
					sample,
					m_dsp->debug_serial_output_register(0),
					m_dsp->debug_serial_output_register(1),
					ba0,
					u32(m_dsp->debug_aacc()),
					(unsigned long long)m_dsp->debug_macc(),
					logical(0x50),
					logical(0x51),
					logical(0x52),
					logical(0x53),
					logical(0x54),
					logical(0x55),
					logical(0x56),
					logical(0x57),
					logical(0x58));
			}
		}
	};

	run_variant("sine_base", sine);
	run_variant("impulse_base", impulse);

	{
		tms57002_device::debug_snapshot variant = sine;
		variant.aacc = impulse.aacc;
		variant.macc = impulse.macc;
		variant.macc_read = impulse.macc_read;
		variant.macc_write = impulse.macc_write;
		variant.creg = impulse.creg;
		run_variant("imp_core", variant);
	}
	{
		tms57002_device::debug_snapshot variant = sine;
		copy_logical(variant, impulse, { 0x50, 0x51, 0x52 });
		run_variant("imp_d50_52", variant);
	}
	{
		tms57002_device::debug_snapshot variant = sine;
		copy_logical(variant, impulse, { 0x53 });
		run_variant("imp_d53", variant);
	}
	{
		tms57002_device::debug_snapshot variant = sine;
		copy_logical(variant, impulse, { 0x54, 0x55 });
		run_variant("imp_d54_55", variant);
	}
	{
		tms57002_device::debug_snapshot variant = sine;
		copy_logical(variant, impulse, { 0x57, 0x58 });
		run_variant("imp_d57_58", variant);
	}
	{
		tms57002_device::debug_snapshot variant = sine;
		copy_logical(variant, impulse, { 0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58 });
		run_variant("imp_d50_58", variant);
	}
	{
		tms57002_device::debug_snapshot variant = sine;
		variant.aacc = impulse.aacc;
		variant.macc = impulse.macc;
		variant.macc_read = impulse.macc_read;
		variant.macc_write = impulse.macc_write;
		variant.creg = impulse.creg;
		copy_logical(variant, impulse, { 0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58 });
		run_variant("imp_core_d50_58", variant);
	}
}

void tms57002test_state::probe_dsp3_a02_tape_su480002_hot_producer_sensitivity()
{
	static constexpr char kProgramPath[] = "00-extracted/kprop_a02_sine_live_dump/dump_dsp3.bin";
	static constexpr char kSineLogPath[] = "00-extracted/kprop_a02_sine_sample_tape_480001_480006_20260408/extracted/snapshot_su480002_pc00_pre.log";
	static constexpr char kImpulseLogPath[] = "00-extracted/kprop_a02_impulse_sample_tape_long_20260408/extracted/snapshot_su480002_pc00_pre.log";
	static constexpr char kSineRamPath[] = "00-extracted/kprop_a02_sine_sample_tape_480001_480006_20260408/ram/su480002_pc00_pre.bin";
	static constexpr char kImpulseRamPath[] = "00-extracted/kprop_a02_impulse_sample_tape_long_20260408/ram/su480002_pc00_pre.bin";
	static constexpr int kCyclesPerSample = 512;

	const std::vector<u32> program = load_program_words(kProgramPath);
	const tms57002_device::debug_snapshot sine = load_snapshot_from_log(kSineLogPath);
	const tms57002_device::debug_snapshot impulse = load_snapshot_from_log(kImpulseLogPath);

	auto copy_logical = [](tms57002_device::debug_snapshot &dst, const tms57002_device::debug_snapshot &src, std::initializer_list<u8> logicals) -> void
	{
		for (const u8 logical : logicals)
			dst.dmem0[(logical + dst.ba0) & 0xff] = src.dmem0[(logical + src.ba0) & 0xff];
	};

	auto copy_serial = [](tms57002_device::debug_snapshot &dst, const tms57002_device::debug_snapshot &src) -> void
	{
		dst.si = src.si;
		dst.serial_input_latch = src.serial_input_latch;
		dst.serial_input_active = src.serial_input_active;
		dst.serial_input_frame = src.serial_input_frame;
		dst.serial_input_prev_frame = src.serial_input_prev_frame;
		dst.serial_input_pending = src.serial_input_pending;
		dst.serial_input_valid = src.serial_input_valid;
		dst.serial_input_active_valid = src.serial_input_active_valid;
		dst.serial_input_prev_valid = src.serial_input_prev_valid;
		dst.serial_input_pending_valid = src.serial_input_pending_valid;
		dst.serial_output_pending_valid = src.serial_output_pending_valid;
		dst.serial_frame_mode = src.serial_frame_mode;
		dst.serial_input_timing_mode = src.serial_input_timing_mode;
		dst.serial_frame_clocks = src.serial_frame_clocks;
		dst.serial_exec_halfcycles = src.serial_exec_halfcycles;
		dst.serial_input_pending_halfcycle = src.serial_input_pending_halfcycle;
		dst.serial_output_pending_halfcycle = src.serial_output_pending_halfcycle;
		dst.sync_polarity_rising = src.sync_polarity_rising;
		dst.serial_output_muted = src.serial_output_muted;
	};

	auto copy_core = [](tms57002_device::debug_snapshot &dst, const tms57002_device::debug_snapshot &src) -> void
	{
		dst.aacc = src.aacc;
		dst.macc = src.macc;
		dst.macc_read = src.macc_read;
		dst.macc_write = src.macc_write;
		dst.creg = src.creg;
		dst.st1 = src.st1;
	};

	auto copy_xmem = [](tms57002_device::debug_snapshot &dst, const tms57002_device::debug_snapshot &src) -> void
	{
		dst.xoa = src.xoa;
		dst.xba = src.xba;
		dst.xwr = src.xwr;
		dst.xrd = src.xrd;
		dst.txrd = src.txrd;
		dst.xm_adr = src.xm_adr;
		dst.xm_cycles = src.xm_cycles;
		dst.xm_fetches = src.xm_fetches;
	};

	auto log_hot_state = [&](const char *label, u8 executed_pc) -> void
	{
		const u8 ba0 = m_dsp->debug_ba0();
		auto logical = [&](u8 addr) { return m_dsp->dmem0_value((addr + ba0) & 0xff); };
		osd_printf_info(
			"INFO dsp3 a02 su480002 hotprod %s pc=%02x id=%02x ca=%02x ba0=%02x aacc=%08x macc=%013llx creg=%08x "
			"d02=%06x d03=%06x d04=%06x d05=%06x d10=%06x d11=%06x d62=%06x d63=%06x d64=%06x "
			"l50=%06x l51=%06x l52=%06x l53=%06x l54=%06x l55=%06x l56=%06x l57=%06x l58=%06x "
			"xoa=%05x xba=%05x xrd=%06x txrd=%06x so=%06x/%06x\n",
			label,
			executed_pc,
			u8(m_dsp->state_int(TMS57002_ID)),
			u8(m_dsp->state_int(TMS57002_CA)),
			ba0,
			u32(m_dsp->debug_aacc()),
			(unsigned long long)m_dsp->debug_macc(),
			m_dsp->debug_creg(),
			logical(0x02),
			logical(0x03),
			logical(0x04),
			logical(0x05),
			logical(0x10),
			logical(0x11),
			logical(0x62),
			logical(0x63),
			logical(0x64),
			logical(0x50),
			logical(0x51),
			logical(0x52),
			logical(0x53),
			logical(0x54),
			logical(0x55),
			logical(0x56),
			logical(0x57),
			logical(0x58),
			m_dsp->debug_xoa(),
			m_dsp->debug_xba(),
			m_dsp->debug_xrd(),
			m_dsp->debug_txrd(),
			m_dsp->debug_serial_output_register(0),
			m_dsp->debug_serial_output_register(1));
	};

	auto run_variant = [&](const char *label, const tms57002_device::debug_snapshot &base, const char *ram_path) -> void
	{
		m_dsp->debug_load_program(program.data(), u32(program.size()), base.st0, base.st1);
		load_data_space_bytes(ram_path, m_dsp->space(AS_DATA));
		m_dsp->debug_restore_snapshot(base);

		for (int step = 0; step < kCyclesPerSample * 2; step++)
		{
			const int sample_index = step / kCyclesPerSample;
			const u8 executed_pc = u8(m_dsp->state_int(TMS57002_PC));
			const u8 executed_id = u8(m_dsp->state_int(TMS57002_ID));
			m_dsp->debug_run_cycles(1);

			if (sample_index == 0 && executed_id == 0x1f && (executed_pc == 0x82 || executed_pc == 0x83 || executed_pc == 0x88 || executed_pc == 0x8e || executed_pc == 0x8f || executed_pc == 0x90))
				log_hot_state(label, executed_pc);

			if ((step + 1) % kCyclesPerSample == 0)
			{
				const int sample = (step + 1) / kCyclesPerSample;
				const u8 ba0 = m_dsp->debug_ba0();
				auto logical = [&](u8 addr) { return m_dsp->dmem0_value((addr + ba0) & 0xff); };
				osd_printf_info(
					"INFO dsp3 a02 su480002 hotsummary %s sample+%d so=%06x/%06x ba0=%02x macc=%013llx l50=%06x l51=%06x l52=%06x l53=%06x l54=%06x l55=%06x l56=%06x l57=%06x l58=%06x\n",
					label,
					sample,
					m_dsp->debug_serial_output_register(0),
					m_dsp->debug_serial_output_register(1),
					ba0,
					(unsigned long long)m_dsp->debug_macc(),
					logical(0x50),
					logical(0x51),
					logical(0x52),
					logical(0x53),
					logical(0x54),
					logical(0x55),
					logical(0x56),
					logical(0x57),
					logical(0x58));
			}
		}
	};

	run_variant("sine_base", sine, kSineRamPath);
	run_variant("impulse_base", impulse, kImpulseRamPath);

	{
		tms57002_device::debug_snapshot variant = sine;
		copy_serial(variant, impulse);
		run_variant("imp_serial", variant, kSineRamPath);
	}
	{
		tms57002_device::debug_snapshot variant = sine;
		copy_core(variant, impulse);
		run_variant("imp_core", variant, kSineRamPath);
	}
	{
		tms57002_device::debug_snapshot variant = sine;
		copy_logical(variant, impulse, { 0x02, 0x03, 0x04, 0x05 });
		run_variant("imp_d02_05", variant, kSineRamPath);
	}
	{
		tms57002_device::debug_snapshot variant = sine;
		copy_logical(variant, impulse, { 0x10, 0x11 });
		run_variant("imp_d10_11", variant, kSineRamPath);
	}
	{
		tms57002_device::debug_snapshot variant = sine;
		copy_xmem(variant, impulse);
		run_variant("imp_xmem", variant, kSineRamPath);
	}
	{
		tms57002_device::debug_snapshot variant = sine;
		for (u8 addr = 0; addr <= 0x10; addr++)
			variant.cmem[addr] = impulse.cmem[addr];
		run_variant("imp_c00_10", variant, kSineRamPath);
	}
}

void tms57002test_state::probe_dsp3_a02_tape_su480002_serial_firstdiff()
{
	static constexpr char kProgramPath[] = "00-extracted/kprop_a02_sine_live_dump/dump_dsp3.bin";
	static constexpr char kSineLogPath[] = "00-extracted/kprop_a02_sine_sample_tape_480001_480006_20260408/extracted/snapshot_su480002_pc00_pre.log";
	static constexpr char kImpulseLogPath[] = "00-extracted/kprop_a02_impulse_sample_tape_long_20260408/extracted/snapshot_su480002_pc00_pre.log";
	static constexpr char kSineRamPath[] = "00-extracted/kprop_a02_sine_sample_tape_480001_480006_20260408/ram/su480002_pc00_pre.bin";
	static constexpr int kCyclesPerSample = 512;

	struct replay_step
	{
		u8 pc = 0;
		u8 ca = 0;
		u8 id = 0;
		u32 st1 = 0;
		u32 aacc = 0;
		s64 macc = 0;
		u32 creg = 0;
		u32 xrd = 0;
		u32 so0 = 0;
		u32 so1 = 0;
		u32 d02 = 0;
		u32 d03 = 0;
		u32 d04 = 0;
		u32 d05 = 0;
		u32 d10 = 0;
		u32 d11 = 0;
		u32 d50 = 0;
		u32 d53 = 0;
		u32 d56 = 0;
	};

	const std::vector<u32> program = load_program_words(kProgramPath);
	const tms57002_device::debug_snapshot sine = load_snapshot_from_log(kSineLogPath);
	const tms57002_device::debug_snapshot impulse = load_snapshot_from_log(kImpulseLogPath);
	tms57002_device::debug_snapshot variant = sine;

	variant.si = impulse.si;
	variant.serial_input_latch = impulse.serial_input_latch;
	variant.serial_input_active = impulse.serial_input_active;
	variant.serial_input_frame = impulse.serial_input_frame;
	variant.serial_input_prev_frame = impulse.serial_input_prev_frame;
	variant.serial_input_pending = impulse.serial_input_pending;
	variant.serial_input_valid = impulse.serial_input_valid;
	variant.serial_input_active_valid = impulse.serial_input_active_valid;
	variant.serial_input_prev_valid = impulse.serial_input_prev_valid;
	variant.serial_input_pending_valid = impulse.serial_input_pending_valid;
	variant.serial_output_pending_valid = impulse.serial_output_pending_valid;
	variant.serial_frame_mode = impulse.serial_frame_mode;
	variant.serial_input_timing_mode = impulse.serial_input_timing_mode;
	variant.serial_frame_clocks = impulse.serial_frame_clocks;
	variant.serial_exec_halfcycles = impulse.serial_exec_halfcycles;
	variant.serial_input_pending_halfcycle = impulse.serial_input_pending_halfcycle;
	variant.serial_output_pending_halfcycle = impulse.serial_output_pending_halfcycle;
	variant.sync_polarity_rising = impulse.sync_polarity_rising;
	variant.serial_output_muted = impulse.serial_output_muted;

	auto capture = [&](const tms57002_device::debug_snapshot &base) -> std::vector<replay_step>
	{
		m_dsp->debug_load_program(program.data(), u32(program.size()), base.st0, base.st1);
		load_data_space_bytes(kSineRamPath, m_dsp->space(AS_DATA));
		m_dsp->debug_restore_snapshot(base);

		std::vector<replay_step> out;
		out.reserve(kCyclesPerSample);
		for (int i = 0; i < kCyclesPerSample; i++)
		{
			m_dsp->debug_run_cycles(1);
			const u8 ba0 = m_dsp->debug_ba0();
			auto logical = [&](u8 addr) { return m_dsp->dmem0_value((addr + ba0) & 0xff); };

			replay_step step;
			step.pc = u8(m_dsp->state_int(TMS57002_PC));
			step.ca = u8(m_dsp->state_int(TMS57002_CA));
			step.id = u8(m_dsp->state_int(TMS57002_ID));
			step.st1 = m_dsp->state_int(TMS57002_ST1);
			step.aacc = u32(m_dsp->debug_aacc());
			step.macc = s64(m_dsp->debug_macc());
			step.creg = m_dsp->debug_creg();
			step.xrd = m_dsp->debug_xrd();
			step.so0 = m_dsp->debug_serial_output_register(0);
			step.so1 = m_dsp->debug_serial_output_register(1);
			step.d02 = logical(0x02);
			step.d03 = logical(0x03);
			step.d04 = logical(0x04);
			step.d05 = logical(0x05);
			step.d10 = logical(0x10);
			step.d11 = logical(0x11);
			step.d50 = logical(0x50);
			step.d53 = logical(0x53);
			step.d56 = logical(0x56);
			out.push_back(step);
		}
		return out;
	};

	const std::vector<replay_step> base = capture(sine);
	const std::vector<replay_step> altered = capture(variant);

	int diffs_logged = 0;
	bool meaningful_logged = false;
	bool carry_logged = false;
	for (std::size_t i = 0; i < base.size() && i < altered.size(); i++)
	{
		const replay_step &a = base[i];
		const replay_step &b = altered[i];
		const bool any_diff = (a.pc != b.pc || a.ca != b.ca || a.id != b.id || a.st1 != b.st1 || a.aacc != b.aacc || a.macc != b.macc || a.creg != b.creg || a.xrd != b.xrd || a.so0 != b.so0 || a.so1 != b.so1 || a.d02 != b.d02 || a.d03 != b.d03 || a.d04 != b.d04 || a.d05 != b.d05 || a.d10 != b.d10 || a.d11 != b.d11 || a.d50 != b.d50 || a.d53 != b.d53 || a.d56 != b.d56);
		if (any_diff)
		{
			if (diffs_logged < 12)
			{
				osd_printf_info(
					"INFO dsp3 a02 su480002 serialdiff idx=%d step=%zu pc=%02x/%02x ca=%02x/%02x id=%02x/%02x st1=%06x/%06x "
					"aacc=%08x/%08x macc=%013llx/%013llx creg=%08x/%08x xrd=%06x/%06x so=%06x/%06x vs %06x/%06x "
					"d02=%06x/%06x d03=%06x/%06x d04=%06x/%06x d05=%06x/%06x d10=%06x/%06x d11=%06x/%06x d50=%06x/%06x d53=%06x/%06x d56=%06x/%06x\n",
					diffs_logged + 1,
					i + 1,
					a.pc, b.pc,
					a.ca, b.ca,
					a.id, b.id,
					a.st1, b.st1,
					a.aacc, b.aacc,
					(unsigned long long)a.macc, (unsigned long long)b.macc,
					a.creg, b.creg,
					a.xrd, b.xrd,
					a.so0, a.so1, b.so0, b.so1,
					a.d02, b.d02,
					a.d03, b.d03,
					a.d04, b.d04,
					a.d05, b.d05,
					a.d10, b.d10,
					a.d11, b.d11,
					a.d50, b.d50,
					a.d53, b.d53,
					a.d56, b.d56);
				diffs_logged++;
			}

			const bool meaningful = (a.macc != b.macc || a.so0 != b.so0 || a.so1 != b.so1 || a.d50 != b.d50 || a.d53 != b.d53 || a.d56 != b.d56);
			if (meaningful && !meaningful_logged)
			{
				osd_printf_info(
					"INFO dsp3 a02 su480002 serialmeaningful step=%zu pc=%02x/%02x ca=%02x/%02x id=%02x/%02x "
					"macc=%013llx/%013llx so=%06x/%06x vs %06x/%06x d50=%06x/%06x d53=%06x/%06x d56=%06x/%06x d02=%06x/%06x d03=%06x/%06x d04=%06x/%06x d05=%06x/%06x\n",
					i + 1,
					a.pc, b.pc,
					a.ca, b.ca,
					a.id, b.id,
					(unsigned long long)a.macc, (unsigned long long)b.macc,
					a.so0, a.so1, b.so0, b.so1,
					a.d50, b.d50,
					a.d53, b.d53,
					a.d56, b.d56,
					a.d02, b.d02,
					a.d03, b.d03,
					a.d04, b.d04,
					a.d05, b.d05);
				meaningful_logged = true;
			}

			const bool carry = (a.d50 != b.d50 || a.d53 != b.d53 || a.d56 != b.d56);
			if (carry && !carry_logged)
			{
				osd_printf_info(
					"INFO dsp3 a02 su480002 serialcarry step=%zu pc=%02x/%02x ca=%02x/%02x id=%02x/%02x "
					"d50=%06x/%06x d53=%06x/%06x d56=%06x/%06x macc=%013llx/%013llx d02=%06x/%06x\n",
					i + 1,
					a.pc, b.pc,
					a.ca, b.ca,
					a.id, b.id,
					a.d50, b.d50,
					a.d53, b.d53,
					a.d56, b.d56,
					(unsigned long long)a.macc, (unsigned long long)b.macc,
					a.d02, b.d02);
				carry_logged = true;
			}
		}
	}

	if (diffs_logged == 0)
		osd_printf_info("INFO dsp3 a02 su480002 serialdiff none\n");
	if (!meaningful_logged)
		osd_printf_info("INFO dsp3 a02 su480002 serialmeaningful none\n");
	if (!carry_logged)
		osd_printf_info("INFO dsp3 a02 su480002 serialcarry none\n");
}

void tms57002test_state::probe_dsp3_a02_tape_su480002_serial_path_checks()
{
	static constexpr char kProgramPath[] = "00-extracted/kprop_a02_sine_live_dump/dump_dsp3.bin";
	static constexpr char kSineLogPath[] = "00-extracted/kprop_a02_sine_sample_tape_480001_480006_20260408/extracted/snapshot_su480002_pc00_pre.log";
	static constexpr char kImpulseLogPath[] = "00-extracted/kprop_a02_impulse_sample_tape_long_20260408/extracted/snapshot_su480002_pc00_pre.log";
	static constexpr char kSineRamPath[] = "00-extracted/kprop_a02_sine_sample_tape_480001_480006_20260408/ram/su480002_pc00_pre.bin";

	const std::vector<u32> program = load_program_words(kProgramPath);
	const tms57002_device::debug_snapshot sine = load_snapshot_from_log(kSineLogPath);
	const tms57002_device::debug_snapshot impulse = load_snapshot_from_log(kImpulseLogPath);
	tms57002_device::debug_snapshot imp_serial = sine;
	imp_serial.si = impulse.si;
	imp_serial.serial_input_latch = impulse.serial_input_latch;
	imp_serial.serial_input_active = impulse.serial_input_active;
	imp_serial.serial_input_frame = impulse.serial_input_frame;
	imp_serial.serial_input_prev_frame = impulse.serial_input_prev_frame;
	imp_serial.serial_input_pending = impulse.serial_input_pending;
	imp_serial.serial_input_valid = impulse.serial_input_valid;
	imp_serial.serial_input_active_valid = impulse.serial_input_active_valid;
	imp_serial.serial_input_prev_valid = impulse.serial_input_prev_valid;
	imp_serial.serial_input_pending_valid = impulse.serial_input_pending_valid;
	imp_serial.serial_output_pending_valid = impulse.serial_output_pending_valid;
	imp_serial.serial_frame_mode = impulse.serial_frame_mode;
	imp_serial.serial_input_timing_mode = impulse.serial_input_timing_mode;
	imp_serial.serial_frame_clocks = impulse.serial_frame_clocks;
	imp_serial.serial_exec_halfcycles = impulse.serial_exec_halfcycles;
	imp_serial.serial_input_pending_halfcycle = impulse.serial_input_pending_halfcycle;
	imp_serial.serial_output_pending_halfcycle = impulse.serial_output_pending_halfcycle;
	imp_serial.sync_polarity_rising = impulse.sync_polarity_rising;
	imp_serial.serial_output_muted = impulse.serial_output_muted;

	struct checkpoint
	{
		u32 aacc52_pre = 0;
		s64 macc52_pre = 0;
		s64 macc52_post = 0;
		u32 d02_52 = 0;
		s64 macc82_pre = 0;
		s64 macc82_post = 0;
		u32 d50_82 = 0;
		u8 ba0_82 = 0;
		s64 macc88_pre = 0;
		s64 macc88_post = 0;
		u32 d53_88 = 0;
		s64 macc8e_pre = 0;
		s64 macc8e_post = 0;
		u32 d56_8e = 0;
		bool have52 = false;
		bool have82 = false;
		bool have88 = false;
		bool have8e = false;
	};

	auto run = [&](const tms57002_device::debug_snapshot &base) -> checkpoint
	{
		m_dsp->debug_load_program(program.data(), u32(program.size()), base.st0, base.st1);
		load_data_space_bytes(kSineRamPath, m_dsp->space(AS_DATA));
		m_dsp->debug_restore_snapshot(base);

		checkpoint out;
		for (int step = 0; step < 512; step++)
		{
			const u8 executed_pc = u8(m_dsp->state_int(TMS57002_PC));
			const u8 executed_id = u8(m_dsp->state_int(TMS57002_ID));
			const u8 ba0 = m_dsp->debug_ba0();
			auto logical = [&](u8 addr) { return m_dsp->dmem0_value((addr + ba0) & 0xff); };

			if (executed_id == 0x00 && executed_pc == 0x52)
			{
				out.have52 = true;
				out.aacc52_pre = u32(m_dsp->debug_aacc());
				out.macc52_pre = s64(m_dsp->debug_macc());
				out.d02_52 = logical(0x02);
			}

			if (executed_id == 0x1f && executed_pc == 0x82)
			{
				out.have82 = true;
				out.macc82_pre = s64(m_dsp->debug_macc());
				out.ba0_82 = ba0;
			}
			if (executed_id == 0x1f && executed_pc == 0x88)
			{
				out.have88 = true;
				out.macc88_pre = s64(m_dsp->debug_macc());
			}
			if (executed_id == 0x1f && executed_pc == 0x8e)
			{
				out.have8e = true;
				out.macc8e_pre = s64(m_dsp->debug_macc());
			}

			m_dsp->debug_run_cycles(1);

			if (executed_id == 0x00 && executed_pc == 0x52)
				out.macc52_post = s64(m_dsp->debug_macc());
			if (executed_id == 0x1f && executed_pc == 0x82)
			{
				out.macc82_post = s64(m_dsp->debug_macc());
				out.d50_82 = m_dsp->dmem0_value((out.ba0_82 + 0x50) & 0xff);
			}
			if (executed_id == 0x1f && executed_pc == 0x88)
			{
				out.macc88_post = s64(m_dsp->debug_macc());
				out.d53_88 = m_dsp->dmem0_value((ba0 + 0x53) & 0xff);
			}
			if (executed_id == 0x1f && executed_pc == 0x8e)
			{
				out.macc8e_post = s64(m_dsp->debug_macc());
				out.d56_8e = m_dsp->dmem0_value((ba0 + 0x56) & 0xff);
				break;
			}
		}
		return out;
	};

	const checkpoint a = run(sine);
	const checkpoint b = run(imp_serial);
	const s64 expected_delta_a = (s64(s32(a.aacc52_pre)) * sx24(a.d02_52)) >> 7;
	const s64 expected_delta_b = (s64(s32(b.aacc52_pre)) * sx24(b.d02_52)) >> 7;

	osd_printf_info(
		"INFO dsp3 a02 su480002 serialpath pc52 sine aacc=%08x d02=%06x macc=%013llx->%013llx delta=%013llx expect=%013llx\n",
		a.aacc52_pre, a.d02_52,
		(unsigned long long)a.macc52_pre, (unsigned long long)a.macc52_post,
		(unsigned long long)(a.macc52_post - a.macc52_pre),
		(unsigned long long)expected_delta_a);
	osd_printf_info(
		"INFO dsp3 a02 su480002 serialpath pc52 impserial aacc=%08x d02=%06x macc=%013llx->%013llx delta=%013llx expect=%013llx\n",
		b.aacc52_pre, b.d02_52,
		(unsigned long long)b.macc52_pre, (unsigned long long)b.macc52_post,
		(unsigned long long)(b.macc52_post - b.macc52_pre),
		(unsigned long long)expected_delta_b);
	osd_printf_info(
		"INFO dsp3 a02 su480002 serialpath pc82 sine macc=%013llx->%013llx d50=%06x\n",
		(unsigned long long)a.macc82_pre, (unsigned long long)a.macc82_post, a.d50_82);
	osd_printf_info(
		"INFO dsp3 a02 su480002 serialpath pc82 impserial macc=%013llx->%013llx d50=%06x\n",
		(unsigned long long)b.macc82_pre, (unsigned long long)b.macc82_post, b.d50_82);
	osd_printf_info(
		"INFO dsp3 a02 su480002 serialpath pc88 sine macc=%013llx->%013llx d53=%06x\n",
		(unsigned long long)a.macc88_pre, (unsigned long long)a.macc88_post, a.d53_88);
	osd_printf_info(
		"INFO dsp3 a02 su480002 serialpath pc88 impserial macc=%013llx->%013llx d53=%06x\n",
		(unsigned long long)b.macc88_pre, (unsigned long long)b.macc88_post, b.d53_88);
	osd_printf_info(
		"INFO dsp3 a02 su480002 serialpath pc8e sine macc=%013llx->%013llx d56=%06x\n",
		(unsigned long long)a.macc8e_pre, (unsigned long long)a.macc8e_post, a.d56_8e);
	osd_printf_info(
		"INFO dsp3 a02 su480002 serialpath pc8e impserial macc=%013llx->%013llx d56=%06x\n",
		(unsigned long long)b.macc8e_pre, (unsigned long long)b.macc8e_post, b.d56_8e);
}

void tms57002test_state::probe_dsp3_a02_tape_boundary_replay()
{
	static constexpr char kProgramPath[] = "00-extracted/korgprop_dsp_offline_extract/dsp3_init.bin";
	static constexpr char kLogPath[] = "00-extracted/kprop_a02_sample_tape_20260408/error.log";
	static constexpr char kRamDir[] = "00-extracted/kprop_a02_sample_tape_20260408/ram";
	static constexpr int kCyclesPerSample = 512;

	const std::vector<u32> program = load_program_words(kProgramPath);
	const std::vector<std::string> lines = load_text_lines(kLogPath);

	auto load_ram = [](u64 su) -> std::vector<u8>
	{
		const std::string path = util::string_format("%s/su%06llu_pc00_pre.bin", kRamDir, (unsigned long long)su);
		std::ifstream input(path, std::ios::binary);
		if (!input)
			throw emu_fatalerror("Unable to open %s", path);
		return std::vector<u8>((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
	};

	auto compare_and_log = [&](u64 su) -> void
	{
		const tms57002_device::debug_snapshot start = load_snapshot_for_su_from_lines(lines, su);
		const tms57002_device::debug_snapshot expect = load_snapshot_for_su_from_lines(lines, su + 1);
		const std::vector<u8> expect_ram = load_ram(su + 1);
		const std::string start_ram_path = util::string_format("%s/su%06llu_pc00_pre.bin", kRamDir, (unsigned long long)su);

		m_dsp->debug_load_program(program.data(), u32(program.size()), start.st0, start.st1);
		load_data_space_bytes(start_ram_path.c_str(), m_dsp->space(AS_DATA));
		m_dsp->debug_restore_snapshot(start);
		for (int steps = 0; steps < kCyclesPerSample; steps++)
			m_dsp->debug_run_cycles(1);
		m_dsp->debug_begin_sample_frame(expect.serial_input_frame);

		const tms57002_device::debug_snapshot actual = m_dsp->debug_capture_snapshot();

		struct field_diff
		{
			const char *name;
			s64 actual;
			s64 expect;
		};

		std::vector<field_diff> diffs;
		auto note_diff = [&](const char *name, s64 actual_value, s64 expected_value)
		{
			if (actual_value != expected_value)
				diffs.push_back({ name, actual_value, expected_value });
		};

		note_diff("sound_updates", actual.sound_updates, expect.sound_updates);
		note_diff("pc", actual.pc, expect.pc);
		note_diff("ca", actual.ca, expect.ca);
		note_diff("id", actual.id, expect.id);
		note_diff("ba0", actual.ba0, expect.ba0);
		note_diff("ba1", actual.ba1, expect.ba1);
		note_diff("st1", actual.st1, expect.st1);
		note_diff("aacc", actual.aacc, expect.aacc);
		note_diff("macc", s64(actual.macc), s64(expect.macc));
		note_diff("maccr", s64(actual.macc_read), s64(expect.macc_read));
		note_diff("maccw", s64(actual.macc_write), s64(expect.macc_write));
		note_diff("creg", actual.creg, expect.creg);
		note_diff("xoa", actual.xoa, expect.xoa);
		note_diff("xba", actual.xba, expect.xba);
		note_diff("xrd", actual.xrd, expect.xrd);
		note_diff("txrd", actual.txrd, expect.txrd);
		note_diff("xwr", actual.xwr, expect.xwr);
		note_diff("si0", actual.si[0], expect.si[0]);
		note_diff("si1", actual.si[1], expect.si[1]);
		note_diff("so0", actual.so[0], expect.so[0]);
		note_diff("so1", actual.so[1], expect.so[1]);
		note_diff("l53", actual.dmem0[(actual.ba0 + 0x53) & 0xff], expect.dmem0[(expect.ba0 + 0x53) & 0xff]);
		note_diff("l54", actual.dmem0[(actual.ba0 + 0x54) & 0xff], expect.dmem0[(expect.ba0 + 0x54) & 0xff]);
		note_diff("l55", actual.dmem0[(actual.ba0 + 0x55) & 0xff], expect.dmem0[(expect.ba0 + 0x55) & 0xff]);
		note_diff("l56", actual.dmem0[(actual.ba0 + 0x56) & 0xff], expect.dmem0[(expect.ba0 + 0x56) & 0xff]);
		note_diff("l57", actual.dmem0[(actual.ba0 + 0x57) & 0xff], expect.dmem0[(expect.ba0 + 0x57) & 0xff]);
		note_diff("c00", actual.cmem[0x00], expect.cmem[0x00]);
		note_diff("c37", actual.cmem[0x37], expect.cmem[0x37]);
		note_diff("c42", actual.cmem[0x42], expect.cmem[0x42]);

		u32 first_ram_diff = 0xffffffffU;
		u8 actual_ram = 0;
		u8 expected_ram = 0;
		for (u32 addr = 0; addr < expect_ram.size(); addr++)
		{
			const u8 have = m_dsp->space(AS_DATA).read_byte(addr);
			const u8 want = expect_ram[addr];
			if (have != want)
			{
				first_ram_diff = addr;
				actual_ram = have;
				expected_ram = want;
				break;
			}
		}

		if (diffs.empty() && first_ram_diff == 0xffffffffU)
		{
			osd_printf_info("INFO dsp3 a02 tapeboundary su=%llu exact_to_next cycles=%d\n", (unsigned long long)su, kCyclesPerSample);
			return;
		}

		osd_printf_info("INFO dsp3 a02 tapeboundary su=%llu mismatches=%zu cycles=%d\n", (unsigned long long)su, diffs.size(), kCyclesPerSample);
		for (const auto &diff : diffs)
		{
			osd_printf_info(
				"INFO dsp3 a02 tapeboundary su=%llu field=%s actual=%016llx expect=%016llx\n",
				(unsigned long long)su,
				diff.name,
				(unsigned long long)diff.actual,
				(unsigned long long)diff.expect);
		}
		if (first_ram_diff != 0xffffffffU)
		{
			osd_printf_info(
				"INFO dsp3 a02 tapeboundary su=%llu ramdiff=%05x actual=%02x expect=%02x\n",
				(unsigned long long)su,
				first_ram_diff,
				actual_ram,
				expected_ram);
		}
	};

	for (u64 su = 480003; su <= 480005; su++)
		compare_and_log(su);
}

void tms57002test_state::probe_dsp3_a02_onset_boundary_replay()
{
	static constexpr char kProgramPath[] = "tmp/kprop_current_dump_2s/cur_dsp3.bin";
	static constexpr char kLogPath[] = "tmp/kprop_a02_onset_92847_92853/dspsnap.log";
	static constexpr char kRamDir[] = "tmp/kprop_a02_onset_92847_92853/ram";
	const char *program_path = kProgramPath;
	const char *log_path = kLogPath;
	const char *ram_dir = kRamDir;
	if (const char *env = std::getenv("TMS57TEST_ONSET_PROGRAM_PATH"))
		program_path = env;
	if (const char *env = std::getenv("TMS57TEST_ONSET_LOG_PATH"))
		log_path = env;
	if (const char *env = std::getenv("TMS57TEST_ONSET_RAM_DIR"))
		ram_dir = env;
	u64 start_su = 92851;
	if (const char *env = std::getenv("TMS57TEST_ONSET_SU"))
		start_su = parse_dec_u64(env);
	static constexpr int kCyclesPerSample = 512;

	const std::vector<u32> program = load_program_words(program_path);
	const std::vector<std::string> lines = load_text_lines(log_path);
	const tms57002_device::debug_snapshot start = load_snapshot_for_su_from_lines(lines, start_su);
	const tms57002_device::debug_snapshot expect = load_snapshot_for_su_from_lines(lines, start_su + 1);
	const std::string start_ram_path = util::string_format("%s/su%06llu_pc00_pre.bin", ram_dir, (unsigned long long)start_su);
	const std::string expect_ram_path = util::string_format("%s/su%06llu_pc00_pre.bin", ram_dir, (unsigned long long)(start_su + 1));

	std::ifstream expect_ram_input(expect_ram_path, std::ios::binary);
	if (!expect_ram_input)
		throw emu_fatalerror("Unable to open %s", expect_ram_path);
	const std::vector<u8> expect_ram((std::istreambuf_iterator<char>(expect_ram_input)), std::istreambuf_iterator<char>());

	m_dsp->debug_load_program(program.data(), u32(program.size()), start.st0, start.st1);
	load_data_space_bytes(start_ram_path.c_str(), m_dsp->space(AS_DATA));
	m_dsp->debug_restore_snapshot(start);
	for (int steps = 0; steps < kCyclesPerSample; steps++)
		m_dsp->debug_run_cycles(1);
	m_dsp->debug_begin_sample_frame(expect.serial_input_frame);

	const tms57002_device::debug_snapshot actual = m_dsp->debug_capture_snapshot();

	auto logical0 = [](const tms57002_device::debug_snapshot &snapshot, u8 logical) -> u32
	{
		return snapshot.dmem0[(snapshot.ba0 + logical) & 0xff];
	};

	osd_printf_info(
		"INFO dsp3 onset boundary su=%llu->%llu actual so=%06x/%06x macc=%013llx aacc=%08x ba0=%02x st1=%06x "
		"expect so=%06x/%06x macc=%013llx aacc=%08x ba0=%02x st1=%06x\n",
		(unsigned long long)start_su,
		(unsigned long long)(start_su + 1),
		actual.so[0],
		actual.so[1],
		(unsigned long long)actual.macc,
		actual.aacc,
		actual.ba0,
		actual.st1,
		expect.so[0],
		expect.so[1],
		(unsigned long long)expect.macc,
		expect.aacc,
		expect.ba0,
		expect.st1);

	for (const u8 logical : { u8(0x02), u8(0x1f), u8(0x3a), u8(0x56) })
	{
		osd_printf_info(
			"INFO dsp3 onset boundary logical L%02X actual=%06x expect=%06x\n",
			logical,
			logical0(actual, logical),
			logical0(expect, logical));
	}

	for (const u8 cmem_addr : { u8(0x13), u8(0x23), u8(0x38), u8(0x3a), u8(0x3c), u8(0x3d), u8(0x42), u8(0xa4), u8(0xc0), u8(0xc1) })
	{
		osd_printf_info(
			"INFO dsp3 onset boundary cmem C%02X actual=%08x expect=%08x\n",
			cmem_addr,
			actual.cmem[cmem_addr],
			expect.cmem[cmem_addr]);
	}

	u32 first_ram_diff = 0xffffffffU;
	u8 actual_ram = 0;
	u8 expected_ram = 0;
	for (u32 addr = 0; addr < expect_ram.size(); addr++)
	{
		const u8 have = m_dsp->space(AS_DATA).read_byte(addr);
		const u8 want = expect_ram[addr];
		if (have != want)
		{
			first_ram_diff = addr;
			actual_ram = have;
			expected_ram = want;
			break;
		}
	}

	if (first_ram_diff == 0xffffffffU)
		osd_printf_info("INFO dsp3 onset boundary ram exact\n");
	else
		osd_printf_info("INFO dsp3 onset boundary ramdiff=%05x actual=%02x expect=%02x\n", first_ram_diff, actual_ram, expected_ram);

	check_equal("dsp3 onset boundary sound_updates", s64(actual.sound_updates), s64(expect.sound_updates));
	check_equal("dsp3 onset boundary pc", s64(actual.pc), s64(expect.pc));
	check_equal("dsp3 onset boundary ca", s64(actual.ca), s64(expect.ca));
	check_equal("dsp3 onset boundary id", s64(actual.id), s64(expect.id));
	check_equal("dsp3 onset boundary ba0", s64(actual.ba0), s64(expect.ba0));
	check_equal("dsp3 onset boundary ba1", s64(actual.ba1), s64(expect.ba1));
	check_equal("dsp3 onset boundary st1", s64(actual.st1), s64(expect.st1));
	check_equal("dsp3 onset boundary aacc", s64(actual.aacc), s64(expect.aacc));
	check_equal("dsp3 onset boundary macc", s64(actual.macc), s64(expect.macc));
	check_equal("dsp3 onset boundary so0", s64(actual.so[0]), s64(expect.so[0]));
	check_equal("dsp3 onset boundary so1", s64(actual.so[1]), s64(expect.so[1]));
	check_equal("dsp3 onset boundary ram", s64(first_ram_diff), s64(0xffffffffU));

	m_dsp->debug_load_program(program.data(), u32(program.size()), start.st0, start.st1);
	load_data_space_bytes(start_ram_path.c_str(), m_dsp->space(AS_DATA));
	m_dsp->debug_restore_snapshot(start);

	bool saw_so0_clip = false;
	for (int step = 0; step < kCyclesPerSample; step++)
	{
		const tms57002_device::debug_snapshot pre = m_dsp->debug_capture_snapshot();
		m_dsp->debug_run_cycles(1);
		const tms57002_device::debug_snapshot post = m_dsp->debug_capture_snapshot();

			const bool key_chain =
				pre.pc == 0x3c || pre.pc == 0x68 || pre.pc == 0x6a ||
				pre.pc == 0x93 || pre.pc == 0xa8 || pre.pc == 0xb6;
			const bool d3a_seed = pre.pc >= 0x6e && pre.pc <= 0x93;
			const bool d56_seed = pre.pc >= 0x9c && pre.pc <= 0xa8;
			const bool d56_producer = pre.pc >= 0xa9 && pre.pc <= 0xb6;
			const bool d4b_producer = pre.pc >= 0xc1 && pre.pc <= 0xd3;
			const bool final_mix = pre.pc >= 0xe9;
			const bool so_change = pre.so != post.so;
			if (!key_chain && !d3a_seed && !d56_seed && !d56_producer && !d4b_producer && !final_mix && !so_change)
				continue;

		if (key_chain)
		{
			osd_printf_info(
				"INFO dsp3 onset scan step=%03d pc=%02x raw=%08x macc=%013llx->%013llx aacc=%08x->%08x "
				"L02=%06x->%06x L1F=%06x->%06x L3A=%06x->%06x L56=%06x->%06x so=%06x/%06x->%06x/%06x\n",
				step,
				pre.pc,
				pre.pc < program.size() ? program[pre.pc] : 0,
				(unsigned long long)pre.macc,
				(unsigned long long)post.macc,
				pre.aacc,
				post.aacc,
				logical0(pre, 0x02),
				logical0(post, 0x02),
				logical0(pre, 0x1f),
				logical0(post, 0x1f),
				logical0(pre, 0x3a),
				logical0(post, 0x3a),
				logical0(pre, 0x56),
				logical0(post, 0x56),
				pre.so[0],
				pre.so[1],
					post.so[0],
					post.so[1]);
			}

			if (d3a_seed)
			{
				osd_printf_info(
					"INFO dsp3 onset d3aseed step=%03d pc=%02x raw=%08x ca=%02x id=%02x xoa=%05x xba=%05x xrd=%06x xwr=%06x "
					"macc=%013llx->%013llx aacc=%08x->%08x st1=%06x->%06x maccr=%013llx->%013llx maccw=%013llx->%013llx creg=%08x->%08x "
					"L00=%06x->%06x L01=%06x L02=%06x L1F=%06x L30=%06x->%06x L31=%06x L32=%06x->%06x L33=%06x L34=%06x->%06x L35=%06x "
					"L36=%06x->%06x L37=%06x L38=%06x->%06x L39=%06x L3A=%06x->%06x L3C=%06x L3D=%06x LA7=%06x->%06x LA8=%06x "
					"C00=%08x C02=%08x C30=%08x C31=%08x C32=%08x C33=%08x C34=%08x C35=%08x C36=%08x C37=%08x C38=%08x "
					"C39=%08x->%08x C3A=%08x->%08x C3C=%08x->%08x C3D=%08x->%08x CA2=%08x CA3=%08x so=%06x/%06x->%06x/%06x\n",
					step,
					pre.pc,
					pre.pc < program.size() ? program[pre.pc] : 0,
					pre.ca,
					pre.id,
					pre.xoa,
					pre.xba,
					pre.xrd,
					pre.xwr,
					(unsigned long long)pre.macc,
					(unsigned long long)post.macc,
					pre.aacc,
					post.aacc,
					pre.st1,
					post.st1,
					(unsigned long long)pre.macc_read,
					(unsigned long long)post.macc_read,
					(unsigned long long)pre.macc_write,
					(unsigned long long)post.macc_write,
					pre.creg,
					post.creg,
					logical0(pre, 0x00),
					logical0(post, 0x00),
					logical0(pre, 0x01),
					logical0(pre, 0x02),
					logical0(pre, 0x1f),
					logical0(pre, 0x30),
					logical0(post, 0x30),
					logical0(pre, 0x31),
					logical0(pre, 0x32),
					logical0(post, 0x32),
					logical0(pre, 0x33),
					logical0(pre, 0x34),
					logical0(post, 0x34),
					logical0(pre, 0x35),
					logical0(pre, 0x36),
					logical0(post, 0x36),
					logical0(pre, 0x37),
					logical0(pre, 0x38),
					logical0(post, 0x38),
					logical0(pre, 0x39),
					logical0(pre, 0x3a),
					logical0(post, 0x3a),
					logical0(pre, 0x3c),
					logical0(pre, 0x3d),
					logical0(pre, 0xa7),
					logical0(post, 0xa7),
					logical0(pre, 0xa8),
					pre.cmem[0x00],
					pre.cmem[0x02],
					pre.cmem[0x30],
					pre.cmem[0x31],
					pre.cmem[0x32],
					pre.cmem[0x33],
					pre.cmem[0x34],
					pre.cmem[0x35],
					pre.cmem[0x36],
					pre.cmem[0x37],
					pre.cmem[0x38],
					pre.cmem[0x39],
					post.cmem[0x39],
					pre.cmem[0x3a],
					post.cmem[0x3a],
					pre.cmem[0x3c],
					post.cmem[0x3c],
					pre.cmem[0x3d],
					post.cmem[0x3d],
					pre.cmem[0xa2],
					pre.cmem[0xa3],
					pre.so[0],
					pre.so[1],
					post.so[0],
					post.so[1]);
			}

			if (d56_seed)
			{
				osd_printf_info(
					"INFO dsp3 onset d56seed step=%03d pc=%02x raw=%08x ca=%02x id=%02x xoa=%05x xba=%05x xrd=%06x xwr=%06x "
					"macc=%013llx->%013llx aacc=%08x->%08x st1=%06x->%06x maccr=%013llx->%013llx maccw=%013llx->%013llx creg=%08x->%08x "
					"L00=%06x->%06x L01=%06x->%06x L02=%06x L3A=%06x L50=%06x->%06x L51=%06x L52=%06x L53=%06x L54=%06x L55=%06x L56=%06x "
					"LA4=%06x LA9=%06x LAA=%06x C00=%08x C02=%08x C0C=%08x CA4=%08x CA9=%08x CAA=%08x so=%06x/%06x->%06x/%06x\n",
					step,
					pre.pc,
					pre.pc < program.size() ? program[pre.pc] : 0,
					pre.ca,
					pre.id,
					pre.xoa,
					pre.xba,
					pre.xrd,
					pre.xwr,
					(unsigned long long)pre.macc,
					(unsigned long long)post.macc,
					pre.aacc,
					post.aacc,
					pre.st1,
					post.st1,
					(unsigned long long)pre.macc_read,
					(unsigned long long)post.macc_read,
					(unsigned long long)pre.macc_write,
					(unsigned long long)post.macc_write,
					pre.creg,
					post.creg,
					logical0(pre, 0x00),
					logical0(post, 0x00),
					logical0(pre, 0x01),
					logical0(post, 0x01),
					logical0(pre, 0x02),
					logical0(pre, 0x3a),
					logical0(pre, 0x50),
					logical0(post, 0x50),
					logical0(pre, 0x51),
					logical0(pre, 0x52),
					logical0(pre, 0x53),
					logical0(pre, 0x54),
					logical0(pre, 0x55),
					logical0(pre, 0x56),
					logical0(pre, 0xa4),
					logical0(pre, 0xa9),
					logical0(pre, 0xaa),
					pre.cmem[0x00],
					pre.cmem[0x02],
					pre.cmem[0x0c],
					pre.cmem[0xa4],
					pre.cmem[0xa9],
					pre.cmem[0xaa],
					pre.so[0],
					pre.so[1],
					post.so[0],
					post.so[1]);
			}

			if (d56_producer)
			{
				osd_printf_info(
					"INFO dsp3 onset d56prod step=%03d pc=%02x raw=%08x ca=%02x id=%02x "
					"macc=%013llx->%013llx aacc=%08x->%08x st1=%06x->%06x maccr=%013llx->%013llx maccw=%013llx->%013llx "
					"L50=%06x L51=%06x L52=%06x L53=%06x->%06x L54=%06x L55=%06x L56=%06x->%06x L57=%06x L58=%06x "
					"C50=%08x C51=%08x C52=%08x C53=%08x C54=%08x C55=%08x C56=%08x C57=%08x C58=%08x so=%06x/%06x->%06x/%06x\n",
					step,
					pre.pc,
					pre.pc < program.size() ? program[pre.pc] : 0,
					pre.ca,
					pre.id,
					(unsigned long long)pre.macc,
					(unsigned long long)post.macc,
					pre.aacc,
					post.aacc,
					pre.st1,
					post.st1,
					(unsigned long long)pre.macc_read,
					(unsigned long long)post.macc_read,
					(unsigned long long)pre.macc_write,
					(unsigned long long)post.macc_write,
					logical0(pre, 0x50),
					logical0(pre, 0x51),
					logical0(pre, 0x52),
					logical0(pre, 0x53),
					logical0(post, 0x53),
					logical0(pre, 0x54),
					logical0(pre, 0x55),
					logical0(pre, 0x56),
					logical0(post, 0x56),
					logical0(pre, 0x57),
					logical0(pre, 0x58),
					pre.cmem[0x50],
					pre.cmem[0x51],
					pre.cmem[0x52],
					pre.cmem[0x53],
					pre.cmem[0x54],
					pre.cmem[0x55],
					pre.cmem[0x56],
					pre.cmem[0x57],
					pre.cmem[0x58],
					pre.so[0],
					pre.so[1],
					post.so[0],
					post.so[1]);
			}

			if (d4b_producer)
			{
				osd_printf_info(
					"INFO dsp3 onset d4bprod step=%03d pc=%02x raw=%08x ca=%02x id=%02x xoa=%05x xba=%05x xrd=%06x xwr=%06x "
					"macc=%013llx->%013llx aacc=%08x->%08x st1=%06x->%06x maccr=%013llx->%013llx maccw=%013llx->%013llx "
					"L40=%06x L41=%06x L45=%06x->%06x L46=%06x->%06x L47=%06x->%06x L4B=%06x->%06x "
					"C44=%08x C45=%08x C46=%08x C47=%08x C48=%08x C49=%08x so=%06x/%06x->%06x/%06x\n",
					step,
					pre.pc,
					pre.pc < program.size() ? program[pre.pc] : 0,
					pre.ca,
					pre.id,
					pre.xoa,
					pre.xba,
					pre.xrd,
					pre.xwr,
					(unsigned long long)pre.macc,
					(unsigned long long)post.macc,
					pre.aacc,
					post.aacc,
					pre.st1,
					post.st1,
					(unsigned long long)pre.macc_read,
					(unsigned long long)post.macc_read,
					(unsigned long long)pre.macc_write,
					(unsigned long long)post.macc_write,
					logical0(pre, 0x40),
					logical0(pre, 0x41),
					logical0(pre, 0x45),
					logical0(post, 0x45),
					logical0(pre, 0x46),
					logical0(post, 0x46),
					logical0(pre, 0x47),
					logical0(post, 0x47),
					logical0(pre, 0x4b),
					logical0(post, 0x4b),
					pre.cmem[0x44],
					pre.cmem[0x45],
					pre.cmem[0x46],
					pre.cmem[0x47],
					pre.cmem[0x48],
					pre.cmem[0x49],
					pre.so[0],
					pre.so[1],
					post.so[0],
					post.so[1]);
			}

			if (final_mix || so_change)
			{
			osd_printf_info(
				"INFO dsp3 onset mix step=%03d pc=%02x raw=%08x macc=%013llx->%013llx aacc=%08x->%08x st1=%06x->%06x "
				"maccr=%013llx->%013llx maccw=%013llx->%013llx "
				"L06=%06x L07=%06x L09=%06x L0A=%06x L0B=%06x L4D=%06x L59=%06x L5A=%06x L5B=%06x L5D=%06x L5E=%06x L5F=%06x "
				"C02=%08x C4A=%08x C64=%08x CC0=%08x CC1=%08x so=%06x/%06x->%06x/%06x\n",
				step,
				pre.pc,
				pre.pc < program.size() ? program[pre.pc] : 0,
				(unsigned long long)pre.macc,
				(unsigned long long)post.macc,
				pre.aacc,
				post.aacc,
				pre.st1,
				post.st1,
				(unsigned long long)pre.macc_read,
				(unsigned long long)post.macc_read,
				(unsigned long long)pre.macc_write,
				(unsigned long long)post.macc_write,
				logical0(pre, 0x06),
				logical0(pre, 0x07),
				logical0(pre, 0x09),
				logical0(pre, 0x0a),
				logical0(pre, 0x0b),
				logical0(pre, 0x4d),
				logical0(pre, 0x59),
				logical0(pre, 0x5a),
				logical0(pre, 0x5b),
				logical0(pre, 0x5d),
				logical0(pre, 0x5e),
				logical0(pre, 0x5f),
				pre.cmem[0x02],
				pre.cmem[0x4a],
				pre.cmem[0x64],
				pre.cmem[0xc0],
				pre.cmem[0xc1],
				pre.so[0],
				pre.so[1],
				post.so[0],
				post.so[1]);
		}

		if (!saw_so0_clip && post.so[0] == 0x7fffff)
		{
			saw_so0_clip = true;
			osd_printf_info(
				"INFO dsp3 onset first so0 clip at step=%03d pc=%02x raw=%08x pre_macc=%013llx post_macc=%013llx so=%06x->%06x\n",
				step,
				pre.pc,
				pre.pc < program.size() ? program[pre.pc] : 0,
				(unsigned long long)pre.macc,
				(unsigned long long)post.macc,
				pre.so[0],
				post.so[0]);
		}
	}

	if (start_su > 92847)
	{
		auto capture_second_pcfc = [&](u64 su) -> tms57002_device::debug_snapshot
		{
			const tms57002_device::debug_snapshot seed = load_snapshot_for_su_from_lines(lines, su);
			const std::string ram_path = util::string_format("%s/su%06llu_pc00_pre.bin", ram_dir, (unsigned long long)su);
			m_dsp->debug_load_program(program.data(), u32(program.size()), seed.st0, seed.st1);
			load_data_space_bytes(ram_path.c_str(), m_dsp->space(AS_DATA));
			m_dsp->debug_restore_snapshot(seed);

			tms57002_device::debug_snapshot found;
			int hits = 0;
			for (int step = 0; step < kCyclesPerSample; step++)
			{
				const tms57002_device::debug_snapshot pre = m_dsp->debug_capture_snapshot();
				if (pre.pc == 0xfc)
				{
					found = pre;
					hits++;
				}
				m_dsp->debug_run_cycles(1);
			}
			if (hits < 2)
				throw emu_fatalerror("Unable to find second PC=FC for SU=%llu", (unsigned long long)su);
			return found;
		};

		const tms57002_device::debug_snapshot prev_pcfc = capture_second_pcfc(start_su - 1);
		const tms57002_device::debug_snapshot cur_pcfc = capture_second_pcfc(start_su);
		const u32 prev_l09 = logical0(prev_pcfc, 0x09);
		const u32 cur_l09 = logical0(cur_pcfc, 0x09);
		const u32 prev_c0 = prev_pcfc.cmem[0xc0];
		const u32 cur_c0 = cur_pcfc.cmem[0xc0];

		auto run_pcfc_variant = [&](const char *label, bool patch_l09, bool patch_c0) -> void
		{
			m_dsp->debug_load_program(program.data(), u32(program.size()), cur_pcfc.st0, cur_pcfc.st1);
			m_dsp->debug_restore_snapshot(cur_pcfc);
			if (patch_l09)
				m_dsp->debug_write_dmem0((cur_pcfc.ba0 + 0x09) & 0xff, prev_l09);
			if (patch_c0)
				m_dsp->debug_write_cmem(0xc0, prev_c0);

			const tms57002_device::debug_snapshot before_fc = m_dsp->debug_capture_snapshot();
			m_dsp->debug_run_cycles(1);
			const tms57002_device::debug_snapshot after_fc = m_dsp->debug_capture_snapshot();
			m_dsp->debug_run_cycles(1);
			const tms57002_device::debug_snapshot after_fd = m_dsp->debug_capture_snapshot();
			m_dsp->debug_run_cycles(1);
			const tms57002_device::debug_snapshot after_fe = m_dsp->debug_capture_snapshot();
			m_dsp->debug_run_cycles(1);
			const tms57002_device::debug_snapshot after_ff = m_dsp->debug_capture_snapshot();

			osd_printf_info(
				"INFO dsp3 onset pcfc variant label=%s l09=%06x c0=%08x fc_macc=%013llx fd_macc=%013llx "
				"fe_maccr=%013llx so_after_fe=%06x/%06x so_after_ff=%06x/%06x st1_after_fe=%06x\n",
				label,
				logical0(before_fc, 0x09),
				before_fc.cmem[0xc0],
				(unsigned long long)after_fc.macc,
				(unsigned long long)after_fd.macc,
				(unsigned long long)after_fe.macc_read,
				after_fe.so[0],
				after_fe.so[1],
				after_ff.so[0],
				after_ff.so[1],
				after_fe.st1);
		};

		osd_printf_info(
			"INFO dsp3 onset pcfc compare prev_su=%llu cur_su=%llu prev_l09=%06x cur_l09=%06x prev_c0=%08x cur_c0=%08x\n",
			(unsigned long long)(start_su - 1),
			(unsigned long long)start_su,
			prev_l09,
			cur_l09,
			prev_c0,
			cur_c0);
		run_pcfc_variant("cur", false, false);
		run_pcfc_variant("prev_l09", true, false);
		run_pcfc_variant("prev_c0", false, true);
		run_pcfc_variant("prev_l09_prev_c0", true, true);

		auto capture_second_pc = [&](u64 su, u8 target_pc) -> tms57002_device::debug_snapshot
		{
			const tms57002_device::debug_snapshot seed = load_snapshot_for_su_from_lines(lines, su);
			const std::string ram_path = util::string_format("%s/su%06llu_pc00_pre.bin", ram_dir, (unsigned long long)su);
			m_dsp->debug_load_program(program.data(), u32(program.size()), seed.st0, seed.st1);
			load_data_space_bytes(ram_path.c_str(), m_dsp->space(AS_DATA));
			m_dsp->debug_restore_snapshot(seed);

			tms57002_device::debug_snapshot found;
			int hits = 0;
			for (int step = 0; step < kCyclesPerSample; step++)
			{
				const tms57002_device::debug_snapshot pre = m_dsp->debug_capture_snapshot();
				if (pre.pc == target_pc)
				{
					found = pre;
					hits++;
				}
				m_dsp->debug_run_cycles(1);
			}
			if (hits < 2)
				throw emu_fatalerror("Unable to find second PC=%02x for SU=%llu", target_pc, (unsigned long long)su);
			return found;
		};

		const tms57002_device::debug_snapshot prev_pced = capture_second_pc(start_su - 1, 0xed);
		const tms57002_device::debug_snapshot cur_pced = capture_second_pc(start_su, 0xed);
		const u32 prev_l0b = logical0(prev_pced, 0x0b);
		const u32 cur_l0b = logical0(cur_pced, 0x0b);

		auto run_pced_variant = [&](const char *label, bool patch_aacc, bool patch_l0b) -> void
		{
			m_dsp->debug_load_program(program.data(), u32(program.size()), cur_pced.st0, cur_pced.st1);
			m_dsp->debug_restore_snapshot(cur_pced);
			if (patch_aacc)
				m_dsp->debug_set_accumulators(prev_pced.aacc, cur_pced.macc, cur_pced.macc_read, cur_pced.macc_write, cur_pced.creg);
			if (patch_l0b)
				m_dsp->debug_write_dmem0((cur_pced.ba0 + 0x0b) & 0xff, prev_l0b);

			tms57002_device::debug_snapshot before_fc;
			tms57002_device::debug_snapshot after_fc;
			tms57002_device::debug_snapshot after_fe;
			tms57002_device::debug_snapshot after_ff;
			bool saw_fc = false;
			bool saw_fe = false;
			bool saw_ff = false;
			for (int step = 0; step < 32; step++)
			{
				const tms57002_device::debug_snapshot pre = m_dsp->debug_capture_snapshot();
				m_dsp->debug_run_cycles(1);
				const tms57002_device::debug_snapshot post = m_dsp->debug_capture_snapshot();
				if (pre.pc == 0xfc)
				{
					before_fc = pre;
					after_fc = post;
					saw_fc = true;
				}
				else if (pre.pc == 0xfe)
				{
					after_fe = post;
					saw_fe = true;
				}
				else if (pre.pc == 0xff)
				{
					after_ff = post;
					saw_ff = true;
					break;
				}
			}
			if (!saw_fc || !saw_fe || !saw_ff)
				throw emu_fatalerror("PC=ED variant %s did not reach final mix", label);

			osd_printf_info(
				"INFO dsp3 onset pced variant label=%s aacc=%08x l0b=%06x l09_at_fc=%06x fc_macc=%013llx "
				"fe_maccr=%013llx so_after_fe=%06x/%06x so_after_ff=%06x/%06x\n",
				label,
				patch_aacc ? prev_pced.aacc : cur_pced.aacc,
				patch_l0b ? prev_l0b : cur_l0b,
				logical0(before_fc, 0x09),
				(unsigned long long)after_fc.macc,
				(unsigned long long)after_fe.macc_read,
				after_fe.so[0],
				after_fe.so[1],
				after_ff.so[0],
				after_ff.so[1]);
		};

		osd_printf_info(
			"INFO dsp3 onset pced compare prev_su=%llu cur_su=%llu prev_aacc=%08x cur_aacc=%08x prev_l0b=%06x cur_l0b=%06x c02=%08x c64=%08x\n",
			(unsigned long long)(start_su - 1),
			(unsigned long long)start_su,
			prev_pced.aacc,
			cur_pced.aacc,
			prev_l0b,
			cur_l0b,
			cur_pced.cmem[0x02],
			cur_pced.cmem[0x64]);
		run_pced_variant("cur", false, false);
		run_pced_variant("prev_aacc", true, false);
		run_pced_variant("prev_l0b", false, true);
		run_pced_variant("prev_aacc_prev_l0b", true, true);

			auto capture_nth_pc = [&](u64 su, u8 target_pc, int wanted_hit) -> tms57002_device::debug_snapshot
			{
				const tms57002_device::debug_snapshot seed = load_snapshot_for_su_from_lines(lines, su);
				const std::string ram_path = util::string_format("%s/su%06llu_pc00_pre.bin", ram_dir, (unsigned long long)su);
				m_dsp->debug_load_program(program.data(), u32(program.size()), seed.st0, seed.st1);
			load_data_space_bytes(ram_path.c_str(), m_dsp->space(AS_DATA));
			m_dsp->debug_restore_snapshot(seed);

			int hits = 0;
			for (int step = 0; step < kCyclesPerSample; step++)
			{
				const tms57002_device::debug_snapshot pre = m_dsp->debug_capture_snapshot();
				if (pre.pc == target_pc)
				{
					hits++;
					if (hits == wanted_hit)
						return pre;
				}
				m_dsp->debug_run_cycles(1);
				}
				throw emu_fatalerror("Unable to find PC=%02x hit=%d for SU=%llu", target_pc, wanted_hit, (unsigned long long)su);
			};

			struct final_loop_summary
			{
				int step_e9 = -1;
				tms57002_device::debug_snapshot before_e9;
				tms57002_device::debug_snapshot after_ea;
				tms57002_device::debug_snapshot before_eb;
				tms57002_device::debug_snapshot after_eb;
				tms57002_device::debug_snapshot after_ec;
				tms57002_device::debug_snapshot after_ed;
				tms57002_device::debug_snapshot after_ee;
				tms57002_device::debug_snapshot after_f0;
				tms57002_device::debug_snapshot before_fc;
				tms57002_device::debug_snapshot after_fc;
				tms57002_device::debug_snapshot after_fe;
				tms57002_device::debug_snapshot after_ff;
			};

			auto capture_final_loop_summary = [&](u64 su, int wanted_hit) -> final_loop_summary
			{
				const tms57002_device::debug_snapshot seed = load_snapshot_for_su_from_lines(lines, su);
				const std::string ram_path = util::string_format("%s/su%06llu_pc00_pre.bin", ram_dir, (unsigned long long)su);
				m_dsp->debug_load_program(program.data(), u32(program.size()), seed.st0, seed.st1);
				load_data_space_bytes(ram_path.c_str(), m_dsp->space(AS_DATA));
				m_dsp->debug_restore_snapshot(seed);

				final_loop_summary out;
				bool armed = false;
				bool saw_after_ea = false;
				bool saw_before_eb = false;
				bool saw_after_ec = false;
				bool saw_after_ed = false;
				bool saw_after_ee = false;
				bool saw_after_f0 = false;
				bool saw_fc = false;
				bool saw_fe = false;
				bool saw_ff = false;
				int hits = 0;
				for (int step = 0; step < kCyclesPerSample; step++)
				{
					const tms57002_device::debug_snapshot pre = m_dsp->debug_capture_snapshot();
					if (pre.pc == 0xe9)
					{
						hits++;
						if (hits == wanted_hit)
						{
							armed = true;
							out.step_e9 = step;
							out.before_e9 = pre;
						}
					}

					m_dsp->debug_run_cycles(1);
					const tms57002_device::debug_snapshot post = m_dsp->debug_capture_snapshot();
					if (!armed)
						continue;

					if (pre.pc == 0xea)
					{
						out.after_ea = post;
						saw_after_ea = true;
					}
					else if (pre.pc == 0xeb)
					{
						out.before_eb = pre;
						out.after_eb = post;
						saw_before_eb = true;
					}
					else if (pre.pc == 0xec)
					{
						out.after_ec = post;
						saw_after_ec = true;
					}
					else if (pre.pc == 0xed)
					{
						out.after_ed = post;
						saw_after_ed = true;
					}
					else if (pre.pc == 0xee)
					{
						out.after_ee = post;
						saw_after_ee = true;
					}
					else if (pre.pc == 0xf0)
					{
						out.after_f0 = post;
						saw_after_f0 = true;
					}
					else if (pre.pc == 0xfc)
					{
						out.before_fc = pre;
						out.after_fc = post;
						saw_fc = true;
					}
					else if (pre.pc == 0xfe)
					{
						out.after_fe = post;
						saw_fe = true;
					}
					else if (pre.pc == 0xff)
					{
						out.after_ff = post;
						saw_ff = true;
						break;
					}
				}

				if (!armed || !saw_after_ea || !saw_before_eb || !saw_after_ec || !saw_after_ed || !saw_after_ee || !saw_after_f0 || !saw_fc || !saw_fe || !saw_ff)
					throw emu_fatalerror("Unable to summarize final loop hit=%d for SU=%llu", wanted_hit, (unsigned long long)su);

				return out;
			};

			auto log_final_loop_summary = [&](u64 su, int wanted_hit) -> void
			{
				const final_loop_summary summary = capture_final_loop_summary(su, wanted_hit);
				osd_printf_info(
					"INFO dsp3 onset loophist su=%llu hit=%d step_e9=%03d ba0=%02x phys_l0a=%02x phys_l09=%02x "
					"d06_e9=%06x d07_e9=%06x aacc_after_ea=%08x l0a_eb=%06x eb_creg=%08x eb_macc=%013llx "
					"after_ec_creg=%08x c02=%08x c64=%08x ed_macc=%013llx l0b_after_ee=%06x ee_macc=%013llx "
					"l09_after_f0=%06x l09_at_fc=%06x l5d_at_fc=%06x fc_macc=%013llx so_fe=%06x/%06x so_ff=%06x/%06x\n",
					(unsigned long long)su,
					wanted_hit,
					summary.step_e9,
					summary.before_eb.ba0,
					(summary.before_eb.ba0 + 0x0a) & 0xff,
					(summary.before_fc.ba0 + 0x09) & 0xff,
					logical0(summary.before_e9, 0x06),
					logical0(summary.before_e9, 0x07),
					summary.after_ea.aacc,
					logical0(summary.before_eb, 0x0a),
					summary.before_eb.creg,
					(unsigned long long)summary.after_eb.macc,
					summary.after_ec.creg,
					summary.before_eb.cmem[0x02],
					summary.before_eb.cmem[0x64],
					(unsigned long long)summary.after_ed.macc,
					logical0(summary.after_ee, 0x0b),
					(unsigned long long)summary.after_ee.macc,
					logical0(summary.after_f0, 0x09),
					logical0(summary.before_fc, 0x09),
					logical0(summary.before_fc, 0x5d),
					(unsigned long long)summary.after_fc.macc,
					summary.after_fe.so[0],
					summary.after_fe.so[1],
					summary.after_ff.so[0],
					summary.after_ff.so[1]);
			};

			const u64 first_history_su = start_su > 92851 ? start_su - 4 : 92847;
				for (u64 su = first_history_su; su <= start_su; su++)
				{
					log_final_loop_summary(su, 1);
					log_final_loop_summary(su, 2);
				}

				struct producer_summary
				{
					int step_b5 = -1;
					int step_e4 = -1;
					tms57002_device::debug_snapshot before_b5;
					tms57002_device::debug_snapshot after_b6;
					tms57002_device::debug_snapshot after_b8;
					tms57002_device::debug_snapshot before_ba;
					tms57002_device::debug_snapshot after_ba;
					tms57002_device::debug_snapshot after_bb;
					tms57002_device::debug_snapshot after_bc;
					tms57002_device::debug_snapshot before_e4;
					tms57002_device::debug_snapshot after_e5;
					tms57002_device::debug_snapshot before_e6;
					tms57002_device::debug_snapshot after_e6;
					tms57002_device::debug_snapshot after_e7;
					tms57002_device::debug_snapshot after_e8;
					tms57002_device::debug_snapshot after_ea;
					tms57002_device::debug_snapshot before_eb;
					tms57002_device::debug_snapshot after_f0;
					tms57002_device::debug_snapshot before_fc;
					tms57002_device::debug_snapshot after_fc;
					tms57002_device::debug_snapshot after_fe;
					tms57002_device::debug_snapshot after_ff;
				};

				auto capture_producer_summary = [&](u64 su, int wanted_hit) -> producer_summary
				{
					const tms57002_device::debug_snapshot seed = load_snapshot_for_su_from_lines(lines, su);
					const std::string ram_path = util::string_format("%s/su%06llu_pc00_pre.bin", ram_dir, (unsigned long long)su);
					m_dsp->debug_load_program(program.data(), u32(program.size()), seed.st0, seed.st1);
					load_data_space_bytes(ram_path.c_str(), m_dsp->space(AS_DATA));
					m_dsp->debug_restore_snapshot(seed);

					producer_summary out;
					bool armed_b = false;
					bool armed_e = false;
					bool saw_after_b6 = false;
					bool saw_after_b8 = false;
					bool saw_before_ba = false;
					bool saw_after_ba = false;
					bool saw_after_bb = false;
					bool saw_after_bc = false;
					bool saw_after_e5 = false;
					bool saw_before_e6 = false;
					bool saw_after_e6 = false;
					bool saw_after_e7 = false;
					bool saw_after_e8 = false;
					bool saw_after_ea = false;
					bool saw_before_eb = false;
					bool saw_after_f0 = false;
					bool saw_fc = false;
					bool saw_fe = false;
					bool saw_ff = false;
					int b_hits = 0;
					int e_hits = 0;

					for (int step = 0; step < kCyclesPerSample; step++)
					{
						const tms57002_device::debug_snapshot pre = m_dsp->debug_capture_snapshot();
						if (pre.pc == 0xb5)
						{
							b_hits++;
							if (b_hits == wanted_hit)
							{
								armed_b = true;
								out.step_b5 = step;
								out.before_b5 = pre;
							}
						}
						if (pre.pc == 0xe4)
						{
							e_hits++;
							if (e_hits == wanted_hit)
							{
								armed_e = true;
								out.step_e4 = step;
								out.before_e4 = pre;
							}
						}

						m_dsp->debug_run_cycles(1);
						const tms57002_device::debug_snapshot post = m_dsp->debug_capture_snapshot();

						if (armed_b)
						{
							if (pre.pc == 0xb6)
							{
								out.after_b6 = post;
								saw_after_b6 = true;
							}
							else if (pre.pc == 0xb8)
							{
								out.after_b8 = post;
								saw_after_b8 = true;
							}
							else if (pre.pc == 0xba)
							{
								out.before_ba = pre;
								out.after_ba = post;
								saw_before_ba = true;
								saw_after_ba = true;
							}
							else if (pre.pc == 0xbb)
							{
								out.after_bb = post;
								saw_after_bb = true;
							}
							else if (pre.pc == 0xbc)
							{
								out.after_bc = post;
								saw_after_bc = true;
							}
						}

						if (!armed_e)
							continue;

						if (pre.pc == 0xe5)
						{
							out.after_e5 = post;
							saw_after_e5 = true;
						}
						else if (pre.pc == 0xe6)
						{
							out.before_e6 = pre;
							out.after_e6 = post;
							saw_before_e6 = true;
							saw_after_e6 = true;
						}
						else if (pre.pc == 0xe7)
						{
							out.after_e7 = post;
							saw_after_e7 = true;
						}
						else if (pre.pc == 0xe8)
						{
							out.after_e8 = post;
							saw_after_e8 = true;
						}
						else if (pre.pc == 0xea)
						{
							out.after_ea = post;
							saw_after_ea = true;
						}
						else if (pre.pc == 0xeb)
						{
							out.before_eb = pre;
							saw_before_eb = true;
						}
						else if (pre.pc == 0xf0)
						{
							out.after_f0 = post;
							saw_after_f0 = true;
						}
						else if (pre.pc == 0xfc)
						{
							out.before_fc = pre;
							out.after_fc = post;
							saw_fc = true;
						}
						else if (pre.pc == 0xfe)
						{
							out.after_fe = post;
							saw_fe = true;
						}
						else if (pre.pc == 0xff)
						{
							out.after_ff = post;
							saw_ff = true;
							break;
						}
					}

					if (!armed_b || !armed_e || !saw_after_b6 || !saw_after_b8 || !saw_before_ba || !saw_after_ba || !saw_after_bb || !saw_after_bc
						|| !saw_after_e5 || !saw_before_e6 || !saw_after_e6 || !saw_after_e7 || !saw_after_e8 || !saw_after_ea
						|| !saw_before_eb || !saw_after_f0 || !saw_fc || !saw_fe || !saw_ff)
						throw emu_fatalerror("Unable to summarize producer hit=%d for SU=%llu", wanted_hit, (unsigned long long)su);

					return out;
				};

				auto log_producer_summary = [&](u64 su, int wanted_hit) -> void
				{
					const producer_summary summary = capture_producer_summary(su, wanted_hit);
					osd_printf_info(
						"INFO dsp3 onset prodhist su=%llu hit=%d step_b5=%03d step_e4=%03d "
						"b5_l4a=%06x b5_l4b=%06x b5_l4c=%06x b5_l56=%06x b5_l59=%06x "
						"b6_l56=%06x b8_c4b=%08x ba_l56=%06x ba_c40=%08x ba_macc=%013llx bb_l4c=%06x bb_c41=%08x bb_macc=%013llx bc_l59=%06x bc_macc=%013llx "
						"e5_l4d=%06x e6_l4b=%06x e6_l59=%06x e6_c4a=%08x e6_c4b=%08x e6_macc=%013llx e7_macc=%013llx e8_macc=%013llx "
						"d06_after_ea=%06x aacc_after_ea=%08x l09_after_f0=%06x l09_at_fc=%06x fc_macc=%013llx so_fe=%06x/%06x so_ff=%06x/%06x\n",
						(unsigned long long)su,
						wanted_hit,
						summary.step_b5,
						summary.step_e4,
						logical0(summary.before_b5, 0x4a),
						logical0(summary.before_b5, 0x4b),
						logical0(summary.before_b5, 0x4c),
						logical0(summary.before_b5, 0x56),
						logical0(summary.before_b5, 0x59),
						logical0(summary.after_b6, 0x56),
						summary.after_b8.cmem[0x4b],
						logical0(summary.before_ba, 0x56),
						summary.before_ba.cmem[0x40],
						(unsigned long long)summary.after_ba.macc,
						logical0(summary.after_bb, 0x4c),
						summary.before_ba.cmem[0x41],
						(unsigned long long)summary.after_bb.macc,
						logical0(summary.after_bc, 0x59),
						(unsigned long long)summary.after_bc.macc,
						logical0(summary.after_e5, 0x4d),
						logical0(summary.before_e6, 0x4b),
						logical0(summary.before_e6, 0x59),
						summary.before_e6.cmem[0x4a],
						summary.before_e6.cmem[0x4b],
						(unsigned long long)summary.after_e6.macc,
						(unsigned long long)summary.after_e7.macc,
						(unsigned long long)summary.after_e8.macc,
						logical0(summary.after_ea, 0x06),
						summary.after_ea.aacc,
						logical0(summary.after_f0, 0x09),
						logical0(summary.before_fc, 0x09),
						(unsigned long long)summary.after_fc.macc,
						summary.after_fe.so[0],
						summary.after_fe.so[1],
						summary.after_ff.so[0],
						summary.after_ff.so[1]);
				};

				for (u64 su = first_history_su; su <= start_su; su++)
				{
					log_producer_summary(su, 1);
					log_producer_summary(su, 2);
				}

				auto signed_half24 = [](u32 value) -> u32
				{
					value &= 0x00ffffffU;
					const s32 signed_value = (value & 0x00800000U) ? s32(value | 0xff000000U) : s32(value);
					return u32(signed_value / 2) & 0x00ffffffU;
				};

				auto signed_half32_for_producer = [](u32 value) -> u32
				{
					return u32(s32(value) / 2);
				};

				const tms57002_device::debug_snapshot prev_pce61 = capture_nth_pc(start_su - 1, 0xe6, 1);
				const tms57002_device::debug_snapshot cur_pce61 = capture_nth_pc(start_su, 0xe6, 1);
				const tms57002_device::debug_snapshot cur_pce62 = capture_nth_pc(start_su, 0xe6, 2);

				auto run_pce6_variant = [&](const char *label, bool patch_l4b, u32 l4b_value, bool patch_l59, u32 l59_value, bool patch_c4a, u32 c4a_value, bool patch_c4b, u32 c4b_value) -> void
				{
					m_dsp->debug_load_program(program.data(), u32(program.size()), start.st0, start.st1);
					load_data_space_bytes(start_ram_path.c_str(), m_dsp->space(AS_DATA));
					m_dsp->debug_restore_snapshot(start);

					tms57002_device::debug_snapshot before_e6;
					tms57002_device::debug_snapshot after_e6;
					tms57002_device::debug_snapshot after_e7;
					tms57002_device::debug_snapshot after_e8;
					tms57002_device::debug_snapshot after_ea1;
					tms57002_device::debug_snapshot before_eb2;
					tms57002_device::debug_snapshot after_f0_2;
					tms57002_device::debug_snapshot before_fc2;
					tms57002_device::debug_snapshot after_fc2;
					tms57002_device::debug_snapshot after_fe2;
					tms57002_device::debug_snapshot after_ff2;
					bool patched = false;
					bool saw_e6 = false;
					bool saw_e7 = false;
					bool saw_e8 = false;
					bool saw_ea1 = false;
					bool saw_eb2 = false;
					bool saw_f0_2 = false;
					bool saw_fc2 = false;
					bool saw_fe2 = false;
					bool saw_ff2 = false;
					int e6_hits = 0;
					int eb_hits = 0;
					int fc_hits = 0;
					int hit_step = -1;

					for (int step = 0; step < kCyclesPerSample; step++)
					{
						tms57002_device::debug_snapshot pre = m_dsp->debug_capture_snapshot();
						if (pre.pc == 0xe6)
						{
							e6_hits++;
							if (e6_hits == 1)
							{
								if (patch_l4b)
									m_dsp->debug_write_dmem0((pre.ba0 + 0x4b) & 0xff, l4b_value & 0x00ffffffU);
								if (patch_l59)
									m_dsp->debug_write_dmem0((pre.ba0 + 0x59) & 0xff, l59_value & 0x00ffffffU);
								if (patch_c4a)
									m_dsp->debug_write_cmem(0x4a, c4a_value);
								if (patch_c4b)
									m_dsp->debug_write_cmem(0x4b, c4b_value);
								pre = m_dsp->debug_capture_snapshot();
								before_e6 = pre;
								patched = true;
								saw_e6 = true;
								hit_step = step;
							}
						}

						m_dsp->debug_run_cycles(1);
						const tms57002_device::debug_snapshot post = m_dsp->debug_capture_snapshot();

						if (!patched)
							continue;

						if (pre.pc == 0xe6 && e6_hits == 1)
							after_e6 = post;
						else if (pre.pc == 0xe7 && e6_hits == 1)
						{
							after_e7 = post;
							saw_e7 = true;
						}
						else if (pre.pc == 0xe8 && e6_hits == 1)
						{
							after_e8 = post;
							saw_e8 = true;
						}
						else if (pre.pc == 0xea && e6_hits == 1)
						{
							after_ea1 = post;
							saw_ea1 = true;
						}
						else if (pre.pc == 0xeb)
						{
							eb_hits++;
							if (eb_hits == 2)
							{
								before_eb2 = pre;
								saw_eb2 = true;
							}
						}
						else if (pre.pc == 0xf0 && eb_hits == 2)
						{
							after_f0_2 = post;
							saw_f0_2 = true;
						}
						else if (pre.pc == 0xfc)
						{
							fc_hits++;
							if (fc_hits == 2)
							{
								before_fc2 = pre;
								after_fc2 = post;
								saw_fc2 = true;
							}
						}
						else if (pre.pc == 0xfe && fc_hits == 2)
						{
							after_fe2 = post;
							saw_fe2 = true;
						}
						else if (pre.pc == 0xff && fc_hits == 2)
						{
							after_ff2 = post;
							saw_ff2 = true;
							break;
						}
					}

					if (!saw_e6 || !saw_e7 || !saw_e8 || !saw_ea1 || !saw_eb2 || !saw_f0_2 || !saw_fc2 || !saw_fe2 || !saw_ff2)
						throw emu_fatalerror("PC=E6 variant %s did not reach second final path", label);

					osd_printf_info(
						"INFO dsp3 onset pce6 variant label=%s step=%d l4b=%06x l59=%06x c4a=%08x c4b=%08x "
						"e6_macc=%013llx e7_macc=%013llx e8_macc=%013llx d06_after_ea1=%06x aacc_after_ea1=%08x "
						"hit2_l0a=%06x hit2_l0b=%06x hit2_aacc_after_f0=%08x hit2_l09=%06x fc_macc=%013llx so_after_fe=%06x/%06x so_after_ff=%06x/%06x\n",
						label,
						hit_step,
						logical0(before_e6, 0x4b),
						logical0(before_e6, 0x59),
						before_e6.cmem[0x4a],
						before_e6.cmem[0x4b],
						(unsigned long long)after_e6.macc,
						(unsigned long long)after_e7.macc,
						(unsigned long long)after_e8.macc,
						logical0(after_ea1, 0x06),
						after_ea1.aacc,
						logical0(before_eb2, 0x0a),
						logical0(before_eb2, 0x0b),
						after_f0_2.aacc,
						logical0(after_f0_2, 0x09),
						(unsigned long long)after_fc2.macc,
						after_fe2.so[0],
						after_fe2.so[1],
						after_ff2.so[0],
						after_ff2.so[1]);
				};

				osd_printf_info(
					"INFO dsp3 onset pce6 compare prev_l4b=%06x cur_l4b=%06x prev_l59=%06x cur_l59=%06x "
					"hit2_l4b=%06x hit2_l59=%06x prev_c4a=%08x cur_c4a=%08x prev_c4b=%08x cur_c4b=%08x\n",
					logical0(prev_pce61, 0x4b),
					logical0(cur_pce61, 0x4b),
					logical0(prev_pce61, 0x59),
					logical0(cur_pce61, 0x59),
					logical0(cur_pce62, 0x4b),
					logical0(cur_pce62, 0x59),
					prev_pce61.cmem[0x4a],
					cur_pce61.cmem[0x4a],
					prev_pce61.cmem[0x4b],
					cur_pce61.cmem[0x4b]);
				run_pce6_variant("cur", false, 0, false, 0, false, 0, false, 0);
				run_pce6_variant("prev_l4b", true, logical0(prev_pce61, 0x4b), false, 0, false, 0, false, 0);
				run_pce6_variant("prev_l59", false, 0, true, logical0(prev_pce61, 0x59), false, 0, false, 0);
				run_pce6_variant("prev_l4b_prev_l59", true, logical0(prev_pce61, 0x4b), true, logical0(prev_pce61, 0x59), false, 0, false, 0);
				run_pce6_variant("l4b_half", true, signed_half24(logical0(cur_pce61, 0x4b)), false, 0, false, 0, false, 0);
				run_pce6_variant("l59_half", false, 0, true, signed_half24(logical0(cur_pce61, 0x59)), false, 0, false, 0);
				run_pce6_variant("l4b_hit2", true, logical0(cur_pce62, 0x4b), false, 0, false, 0, false, 0);
				run_pce6_variant("l4b_hit2_l59_fullscale", true, logical0(cur_pce62, 0x4b), true, 0x3fffff, false, 0, false, 0);
				run_pce6_variant("hit2_operands", true, logical0(cur_pce62, 0x4b), true, logical0(cur_pce62, 0x59), false, 0, false, 0);
				run_pce6_variant("l59_fullscale", false, 0, true, 0x3fffff, false, 0, false, 0);
				run_pce6_variant("l4b_zero", true, 0, false, 0, false, 0, false, 0);
				run_pce6_variant("l59_zero", false, 0, true, 0, false, 0, false, 0);
				run_pce6_variant("c4a_half", false, 0, false, 0, true, signed_half32_for_producer(cur_pce61.cmem[0x4a]), false, 0);
				run_pce6_variant("c4b_half", false, 0, false, 0, false, 0, true, signed_half32_for_producer(cur_pce61.cmem[0x4b]));
				run_pce6_variant("c4a_zero", false, 0, false, 0, true, 0, false, 0);
				run_pce6_variant("c4b_zero", false, 0, false, 0, false, 0, true, 0);

				auto run_pcce_variant = [&](const char *label, bool forward_d46_from_xrd, bool forward_d47_from_xrd) -> void
				{
					m_dsp->debug_load_program(program.data(), u32(program.size()), start.st0, start.st1);
					load_data_space_bytes(start_ram_path.c_str(), m_dsp->space(AS_DATA));
					m_dsp->debug_restore_snapshot(start);

					tms57002_device::debug_snapshot before_ce;
					tms57002_device::debug_snapshot after_ce;
					tms57002_device::debug_snapshot before_d1;
					tms57002_device::debug_snapshot after_d1;
					tms57002_device::debug_snapshot after_d3;
					tms57002_device::debug_snapshot before_e6;
					tms57002_device::debug_snapshot after_e8;
					tms57002_device::debug_snapshot after_ea1;
					tms57002_device::debug_snapshot before_eb2;
					tms57002_device::debug_snapshot after_f0_2;
					tms57002_device::debug_snapshot after_fc2;
					tms57002_device::debug_snapshot after_fe2;
					tms57002_device::debug_snapshot after_ff2;
					bool patched = false;
					bool saw_ce = false;
					bool saw_d1 = false;
					bool saw_d3 = false;
					bool saw_e6 = false;
					bool saw_e8 = false;
					bool saw_ea1 = false;
					bool saw_eb2 = false;
					bool saw_f0_2 = false;
					bool saw_fc2 = false;
					bool saw_fe2 = false;
					bool saw_ff2 = false;
					int ce_hits = 0;
					int d1_hits = 0;
					int eb_hits = 0;
					int fc_hits = 0;

					for (int step = 0; step < kCyclesPerSample; step++)
					{
						tms57002_device::debug_snapshot pre = m_dsp->debug_capture_snapshot();
						if (pre.pc == 0xce)
						{
							ce_hits++;
							if (ce_hits == 1)
							{
								if (forward_d46_from_xrd)
									m_dsp->debug_write_dmem0((pre.ba0 + 0x46) & 0xff, pre.xrd & 0x00ffffffU);
								pre = m_dsp->debug_capture_snapshot();
								before_ce = pre;
								patched = true;
								saw_ce = true;
							}
						}
						else if (pre.pc == 0xd1)
						{
							d1_hits++;
							if (d1_hits == 1)
							{
								if (forward_d47_from_xrd)
									m_dsp->debug_write_dmem0((pre.ba0 + 0x47) & 0xff, pre.xrd & 0x00ffffffU);
								pre = m_dsp->debug_capture_snapshot();
								before_d1 = pre;
								saw_d1 = true;
							}
						}

						m_dsp->debug_run_cycles(1);
						const tms57002_device::debug_snapshot post = m_dsp->debug_capture_snapshot();

						if (!patched)
							continue;

						if (pre.pc == 0xce && ce_hits == 1)
							after_ce = post;
						else if (pre.pc == 0xd1 && d1_hits == 1)
							after_d1 = post;
						else if (pre.pc == 0xd3 && ce_hits == 1)
						{
							after_d3 = post;
							saw_d3 = true;
						}
						else if (pre.pc == 0xe6 && ce_hits == 1)
						{
							before_e6 = pre;
							saw_e6 = true;
						}
						else if (pre.pc == 0xe8 && ce_hits == 1)
						{
							after_e8 = post;
							saw_e8 = true;
						}
						else if (pre.pc == 0xea && ce_hits == 1)
						{
							after_ea1 = post;
							saw_ea1 = true;
						}
						else if (pre.pc == 0xeb)
						{
							eb_hits++;
							if (eb_hits == 2)
							{
								before_eb2 = pre;
								saw_eb2 = true;
							}
						}
						else if (pre.pc == 0xf0 && eb_hits == 2)
						{
							after_f0_2 = post;
							saw_f0_2 = true;
						}
						else if (pre.pc == 0xfc)
						{
							fc_hits++;
							if (fc_hits == 2)
							{
								after_fc2 = post;
								saw_fc2 = true;
							}
						}
						else if (pre.pc == 0xfe && fc_hits == 2)
						{
							after_fe2 = post;
							saw_fe2 = true;
						}
						else if (pre.pc == 0xff && fc_hits == 2)
						{
							after_ff2 = post;
							saw_ff2 = true;
							break;
						}
					}

					if (!saw_ce || !saw_d1 || !saw_d3 || !saw_e6 || !saw_e8 || !saw_ea1 || !saw_eb2 || !saw_f0_2 || !saw_fc2 || !saw_fe2 || !saw_ff2)
						throw emu_fatalerror("PC=CE variant %s did not reach second final path", label);

					osd_printf_info(
						"INFO dsp3 onset pcce variant label=%s ce_xrd=%06x ce_l46=%06x ce_macc=%013llx d1_xrd=%06x d1_l47=%06x d1_macc=%013llx "
						"d3_l4b=%06x e6_l4b=%06x e6_l59=%06x e8_macc=%013llx d06_after_ea1=%06x "
						"hit2_l0a=%06x hit2_l0b=%06x hit2_aacc_after_f0=%08x hit2_l09=%06x fc_macc=%013llx so_after_fe=%06x/%06x so_after_ff=%06x/%06x\n",
						label,
						before_ce.xrd,
						logical0(before_ce, 0x46),
						(unsigned long long)after_ce.macc,
						before_d1.xrd,
						logical0(before_d1, 0x47),
						(unsigned long long)after_d1.macc,
						logical0(after_d3, 0x4b),
						logical0(before_e6, 0x4b),
						logical0(before_e6, 0x59),
						(unsigned long long)after_e8.macc,
						logical0(after_ea1, 0x06),
						logical0(before_eb2, 0x0a),
						logical0(before_eb2, 0x0b),
						after_f0_2.aacc,
						logical0(after_f0_2, 0x09),
						(unsigned long long)after_fc2.macc,
						after_fe2.so[0],
						after_fe2.so[1],
						after_ff2.so[0],
						after_ff2.so[1]);
				};

				run_pcce_variant("cur", false, false);
				run_pcce_variant("forward_d46_xrd", true, false);
				run_pcce_variant("forward_d47_xrd", false, true);
				run_pcce_variant("forward_d46_d47_xrd", true, true);

				const tms57002_device::debug_snapshot cur_pcb02 = capture_nth_pc(start_su, 0xb0, 2);
				auto run_pcb0_ce_variant = [&](const char *label, bool patch_b0_l53, bool forward_ce_d46) -> void
				{
					m_dsp->debug_load_program(program.data(), u32(program.size()), start.st0, start.st1);
					load_data_space_bytes(start_ram_path.c_str(), m_dsp->space(AS_DATA));
					m_dsp->debug_restore_snapshot(start);

					tms57002_device::debug_snapshot before_b0;
					tms57002_device::debug_snapshot after_b6;
					tms57002_device::debug_snapshot before_ce;
					tms57002_device::debug_snapshot after_d3;
					tms57002_device::debug_snapshot before_e6;
					tms57002_device::debug_snapshot after_e8;
					tms57002_device::debug_snapshot after_ea1;
					tms57002_device::debug_snapshot after_f0_2;
					tms57002_device::debug_snapshot after_fc2;
					tms57002_device::debug_snapshot after_fe2;
					tms57002_device::debug_snapshot after_ff2;
					bool saw_b0 = false;
					bool saw_b6 = false;
					bool saw_ce = false;
					bool saw_d3 = false;
					bool saw_e6 = false;
					bool saw_e8 = false;
					bool saw_ea1 = false;
					bool saw_f0_2 = false;
					bool saw_fc2 = false;
					bool saw_fe2 = false;
					bool saw_ff2 = false;
					int b0_hits = 0;
					int ce_hits = 0;
					int eb_hits = 0;
					int fc_hits = 0;
					const u32 hit2_l53 = logical0(cur_pcb02, 0x53);

					for (int step = 0; step < kCyclesPerSample; step++)
					{
						tms57002_device::debug_snapshot pre = m_dsp->debug_capture_snapshot();
						if (pre.pc == 0xb0)
						{
							b0_hits++;
							if (b0_hits == 1)
							{
								if (patch_b0_l53)
									m_dsp->debug_write_dmem0((pre.ba0 + 0x53) & 0xff, hit2_l53);
								pre = m_dsp->debug_capture_snapshot();
								before_b0 = pre;
								saw_b0 = true;
							}
						}
						else if (pre.pc == 0xce)
						{
							ce_hits++;
							if (ce_hits == 1)
							{
								if (forward_ce_d46)
									m_dsp->debug_write_dmem0((pre.ba0 + 0x46) & 0xff, pre.xrd & 0x00ffffffU);
								pre = m_dsp->debug_capture_snapshot();
								before_ce = pre;
								saw_ce = true;
							}
						}

						m_dsp->debug_run_cycles(1);
						const tms57002_device::debug_snapshot post = m_dsp->debug_capture_snapshot();

						if (pre.pc == 0xb6 && b0_hits >= 1 && !saw_b6)
						{
							after_b6 = post;
							saw_b6 = true;
						}
						else if (pre.pc == 0xd3 && ce_hits >= 1 && !saw_d3)
						{
							after_d3 = post;
							saw_d3 = true;
						}
						else if (pre.pc == 0xe6 && ce_hits >= 1 && !saw_e6)
						{
							before_e6 = pre;
							saw_e6 = true;
						}
						else if (pre.pc == 0xe8 && ce_hits >= 1 && !saw_e8)
						{
							after_e8 = post;
							saw_e8 = true;
						}
						else if (pre.pc == 0xea && ce_hits >= 1 && !saw_ea1)
						{
							after_ea1 = post;
							saw_ea1 = true;
						}
						else if (pre.pc == 0xeb)
						{
							eb_hits++;
						}
						else if (pre.pc == 0xf0 && eb_hits == 2)
						{
							after_f0_2 = post;
							saw_f0_2 = true;
						}
						else if (pre.pc == 0xfc)
						{
							fc_hits++;
							if (fc_hits == 2)
							{
								after_fc2 = post;
								saw_fc2 = true;
							}
						}
						else if (pre.pc == 0xfe && fc_hits == 2)
						{
							after_fe2 = post;
							saw_fe2 = true;
						}
						else if (pre.pc == 0xff && fc_hits == 2)
						{
							after_ff2 = post;
							saw_ff2 = true;
							break;
						}
					}

					if (!saw_b0 || !saw_b6 || !saw_ce || !saw_d3 || !saw_e6 || !saw_e8 || !saw_ea1 || !saw_f0_2 || !saw_fc2 || !saw_fe2 || !saw_ff2)
						throw emu_fatalerror("PC=B0/CE variant %s did not reach second final path", label);

					osd_printf_info(
						"INFO dsp3 onset pcb0ce variant label=%s b0_l53=%06x b6_l56=%06x ce_xrd=%06x ce_l46=%06x d3_l4b=%06x "
						"e6_l4b=%06x e6_l59=%06x e8_macc=%013llx d06_after_ea1=%06x hit2_l09=%06x fc_macc=%013llx so_after_fe=%06x/%06x so_after_ff=%06x/%06x\n",
						label,
						logical0(before_b0, 0x53),
						logical0(after_b6, 0x56),
						before_ce.xrd,
						logical0(before_ce, 0x46),
						logical0(after_d3, 0x4b),
						logical0(before_e6, 0x4b),
						logical0(before_e6, 0x59),
						(unsigned long long)after_e8.macc,
						logical0(after_ea1, 0x06),
						logical0(after_f0_2, 0x09),
						(unsigned long long)after_fc2.macc,
						after_fe2.so[0],
						after_fe2.so[1],
						after_ff2.so[0],
						after_ff2.so[1]);
				};

				run_pcb0_ce_variant("cur", false, false);
				run_pcb0_ce_variant("b0_l53_hit2", true, false);
				run_pcb0_ce_variant("ce_d46_xrd", false, true);
				run_pcb0_ce_variant("b0_l53_hit2_ce_d46_xrd", true, true);

				const tms57002_device::debug_snapshot prev_pceb1 = capture_nth_pc(start_su - 1, 0xeb, 1);
				const tms57002_device::debug_snapshot cur_pceb1 = capture_nth_pc(start_su, 0xeb, 1);
			const tms57002_device::debug_snapshot prev_pceb2 = capture_nth_pc(start_su - 1, 0xeb, 2);
			const tms57002_device::debug_snapshot cur_pceb2 = capture_nth_pc(start_su, 0xeb, 2);

		auto run_pceb_variant = [&](const char *label, int wanted_hit, const tms57002_device::debug_snapshot &prev_ref, bool patch_l0a, bool patch_creg) -> void
		{
			m_dsp->debug_load_program(program.data(), u32(program.size()), start.st0, start.st1);
			load_data_space_bytes(start_ram_path.c_str(), m_dsp->space(AS_DATA));
			m_dsp->debug_restore_snapshot(start);

			tms57002_device::debug_snapshot before_eb;
			tms57002_device::debug_snapshot after_eb;
			tms57002_device::debug_snapshot first_after_ee;
			tms57002_device::debug_snapshot before_fc;
			tms57002_device::debug_snapshot after_fc;
			tms57002_device::debug_snapshot after_fe;
			tms57002_device::debug_snapshot after_ff;
			bool patched = false;
			bool saw_eb = false;
			bool saw_first_ee = false;
			bool saw_fc = false;
			bool saw_fe = false;
			bool saw_ff = false;
			int hits = 0;
			int hit_step = -1;

			for (int step = 0; step < kCyclesPerSample; step++)
			{
				tms57002_device::debug_snapshot pre = m_dsp->debug_capture_snapshot();
				if (pre.pc == 0xeb)
				{
					hits++;
					if (hits == wanted_hit)
					{
						if (patch_l0a)
							m_dsp->debug_write_dmem0((pre.ba0 + 0x0a) & 0xff, logical0(prev_ref, 0x0a));
						if (patch_creg)
							m_dsp->debug_set_accumulators(pre.aacc, pre.macc, pre.macc_read, pre.macc_write, prev_ref.creg);
						pre = m_dsp->debug_capture_snapshot();
						before_eb = pre;
						patched = true;
						saw_eb = true;
						hit_step = step;
					}
				}

				m_dsp->debug_run_cycles(1);
				const tms57002_device::debug_snapshot post = m_dsp->debug_capture_snapshot();

				if (patched && pre.pc == 0xeb && hits == wanted_hit)
					after_eb = post;
				if (patched && pre.pc == 0xee && !saw_first_ee)
				{
					first_after_ee = post;
					saw_first_ee = true;
				}
				if (patched && pre.pc == 0xfc)
				{
					before_fc = pre;
					after_fc = post;
					saw_fc = true;
				}
				if (patched && pre.pc == 0xfe)
				{
					after_fe = post;
					saw_fe = true;
				}
				if (patched && pre.pc == 0xff)
				{
					after_ff = post;
					saw_ff = true;
				}
			}

			if (!saw_eb || !saw_first_ee || !saw_fc || !saw_fe || !saw_ff)
				throw emu_fatalerror("PC=EB variant %s did not reach expected final path", label);

			osd_printf_info(
				"INFO dsp3 onset pceb variant label=%s hit=%d step=%d l0a=%06x creg=%08x eb_macc=%013llx "
				"first_ee_l0b=%06x first_ee_maccr=%013llx l09_at_fc=%06x l0b_at_fc=%06x fc_macc=%013llx "
				"fe_maccr=%013llx so_after_fe=%06x/%06x so_after_ff=%06x/%06x\n",
				label,
				wanted_hit,
				hit_step,
				logical0(before_eb, 0x0a),
				before_eb.creg,
				(unsigned long long)after_eb.macc,
				logical0(first_after_ee, 0x0b),
				(unsigned long long)first_after_ee.macc_read,
				logical0(before_fc, 0x09),
				logical0(before_fc, 0x0b),
				(unsigned long long)after_fc.macc,
				(unsigned long long)after_fe.macc_read,
				after_fe.so[0],
				after_fe.so[1],
				after_ff.so[0],
				after_ff.so[1]);
		};

			osd_printf_info(
				"INFO dsp3 onset pceb compare hit=1 prev_l0a=%06x cur_l0a=%06x prev_creg=%08x cur_creg=%08x "
				"hit=2 prev_l0a=%06x cur_l0a=%06x prev_creg=%08x cur_creg=%08x\n",
				logical0(prev_pceb1, 0x0a),
				logical0(cur_pceb1, 0x0a),
			prev_pceb1.creg,
			cur_pceb1.creg,
				logical0(prev_pceb2, 0x0a),
				logical0(cur_pceb2, 0x0a),
				prev_pceb2.creg,
				cur_pceb2.creg);
			osd_printf_info(
				"INFO dsp3 onset pceb cstar compare hit=1 prev_ca=%02x prev_cstar=%08x cur_ca=%02x cur_cstar=%08x "
				"hit=2 prev_ca=%02x prev_cstar=%08x cur_ca=%02x cur_cstar=%08x\n",
				prev_pceb1.ca,
				prev_pceb1.cmem[prev_pceb1.ca],
				cur_pceb1.ca,
				cur_pceb1.cmem[cur_pceb1.ca],
				prev_pceb2.ca,
				prev_pceb2.cmem[prev_pceb2.ca],
				cur_pceb2.ca,
				cur_pceb2.cmem[cur_pceb2.ca]);
			run_pceb_variant("hit1_cur", 1, prev_pceb1, false, false);
			run_pceb_variant("hit1_prev_l0a", 1, prev_pceb1, true, false);
			run_pceb_variant("hit1_prev_creg", 1, prev_pceb1, false, true);
			run_pceb_variant("hit1_prev_l0a_prev_creg", 1, prev_pceb1, true, true);
			run_pceb_variant("hit2_prev_l0a", 2, prev_pceb2, true, false);

			auto signed_half32 = [](u32 value) -> u32
			{
				return u32(s32(value) / 2);
			};

			auto signed_neg32 = [](u32 value) -> u32
			{
				return u32(-s64(s32(value)));
			};

			auto run_hit1_mix_variant = [&](const char *label, bool patch_l0a, bool patch_eb_creg, u32 eb_creg_value, bool patch_eb_cmem, u32 eb_cmem_value, bool patch_c02, u32 c02_value, bool patch_c64, u32 c64_value) -> void
			{
				m_dsp->debug_load_program(program.data(), u32(program.size()), start.st0, start.st1);
				load_data_space_bytes(start_ram_path.c_str(), m_dsp->space(AS_DATA));
				m_dsp->debug_restore_snapshot(start);

				tms57002_device::debug_snapshot before_eb;
				tms57002_device::debug_snapshot after_eb;
				tms57002_device::debug_snapshot after_ec;
				tms57002_device::debug_snapshot after_ed;
				tms57002_device::debug_snapshot first_after_ee;
				tms57002_device::debug_snapshot before_fc;
				tms57002_device::debug_snapshot after_fc;
				tms57002_device::debug_snapshot after_fe;
				tms57002_device::debug_snapshot after_ff;
				bool patched = false;
				bool saw_eb = false;
				bool saw_ec = false;
				bool saw_ed = false;
				bool saw_first_ee = false;
				bool saw_fc = false;
				bool saw_fe = false;
				bool saw_ff = false;
				int hits = 0;
				int hit_step = -1;

				for (int step = 0; step < kCyclesPerSample; step++)
				{
					tms57002_device::debug_snapshot pre = m_dsp->debug_capture_snapshot();
					if (pre.pc == 0xeb)
					{
						hits++;
						if (hits == 1)
						{
							if (patch_l0a)
								m_dsp->debug_write_dmem0((pre.ba0 + 0x0a) & 0xff, logical0(prev_pceb1, 0x0a));
							if (patch_eb_cmem)
								m_dsp->debug_write_cmem(pre.ca, eb_cmem_value);
							if (patch_c02)
								m_dsp->debug_write_cmem(0x02, c02_value);
							if (patch_c64)
								m_dsp->debug_write_cmem(0x64, c64_value);
							if (patch_eb_creg)
								m_dsp->debug_set_accumulators(pre.aacc, pre.macc, pre.macc_read, pre.macc_write, eb_creg_value);
							pre = m_dsp->debug_capture_snapshot();
							before_eb = pre;
							patched = true;
							saw_eb = true;
							hit_step = step;
						}
					}

					m_dsp->debug_run_cycles(1);
					const tms57002_device::debug_snapshot post = m_dsp->debug_capture_snapshot();

					if (patched && pre.pc == 0xeb && hits == 1)
						after_eb = post;
					if (patched && pre.pc == 0xec && !saw_ec)
					{
						after_ec = post;
						saw_ec = true;
					}
					if (patched && pre.pc == 0xed && !saw_ed)
					{
						after_ed = post;
						saw_ed = true;
					}
					if (patched && pre.pc == 0xee && !saw_first_ee)
					{
						first_after_ee = post;
						saw_first_ee = true;
					}
					if (patched && pre.pc == 0xfc)
					{
						before_fc = pre;
						after_fc = post;
						saw_fc = true;
					}
					if (patched && pre.pc == 0xfe)
					{
						after_fe = post;
						saw_fe = true;
					}
					if (patched && pre.pc == 0xff)
					{
						after_ff = post;
						saw_ff = true;
					}
				}

				if (!saw_eb || !saw_ec || !saw_ed || !saw_first_ee || !saw_fc || !saw_fe || !saw_ff)
					throw emu_fatalerror("hit1 mix variant %s did not reach expected final path", label);

				osd_printf_info(
					"INFO dsp3 onset hit1mix variant label=%s step=%d l0a=%06x eb_ca=%02x eb_cstar=%08x eb_creg=%08x c02=%08x c64=%08x "
					"eb_macc=%013llx after_ec_creg=%08x ed_macc=%013llx first_l0b_after_ee=%06x first_ee_macc=%013llx "
					"l09_at_fc=%06x l0b_at_fc=%06x fc_macc=%013llx fe_maccr=%013llx so_after_fe=%06x/%06x so_after_ff=%06x/%06x\n",
					label,
					hit_step,
					logical0(before_eb, 0x0a),
					before_eb.ca,
					before_eb.cmem[before_eb.ca],
					before_eb.creg,
					before_eb.cmem[0x02],
					before_eb.cmem[0x64],
					(unsigned long long)after_eb.macc,
					after_ec.creg,
					(unsigned long long)after_ed.macc,
					logical0(first_after_ee, 0x0b),
					(unsigned long long)first_after_ee.macc,
					logical0(before_fc, 0x09),
					logical0(before_fc, 0x0b),
					(unsigned long long)after_fc.macc,
					(unsigned long long)after_fe.macc_read,
					after_fe.so[0],
					after_fe.so[1],
					after_ff.so[0],
					after_ff.so[1]);
			};

			const u32 cur_eb_creg = cur_pceb1.creg;
			const u32 cur_eb_cstar = cur_pceb1.cmem[cur_pceb1.ca];
			const u32 prev_eb_cstar = prev_pceb1.cmem[prev_pceb1.ca];
			const u32 cur_c02 = cur_pceb1.cmem[0x02];
			const u32 cur_c64 = cur_pceb1.cmem[0x64];
			run_hit1_mix_variant("cur", false, false, 0, false, 0, false, 0, false, 0);
			run_hit1_mix_variant("prev_l0a", true, false, 0, false, 0, false, 0, false, 0);
			run_hit1_mix_variant("eb_creg_half", false, true, signed_half32(cur_eb_creg), false, 0, false, 0, false, 0);
			run_hit1_mix_variant("eb_creg_zero", false, true, 0, false, 0, false, 0, false, 0);
			run_hit1_mix_variant("eb_cstar_prev", false, false, 0, true, prev_eb_cstar, false, 0, false, 0);
			run_hit1_mix_variant("eb_cstar_half", false, false, 0, true, signed_half32(cur_eb_cstar), false, 0, false, 0);
			run_hit1_mix_variant("eb_cstar_zero", false, false, 0, true, 0, false, 0, false, 0);
			run_hit1_mix_variant("eb_cstar_neg", false, false, 0, true, signed_neg32(cur_eb_cstar), false, 0, false, 0);
			run_hit1_mix_variant("c02_half", false, false, 0, false, 0, true, signed_half32(cur_c02), false, 0);
			run_hit1_mix_variant("c02_zero", false, false, 0, false, 0, true, 0, false, 0);
			run_hit1_mix_variant("c64_half", false, false, 0, false, 0, false, 0, true, signed_half32(cur_c64));
			run_hit1_mix_variant("c64_zero", false, false, 0, false, 0, false, 0, true, 0);
			run_hit1_mix_variant("c64_neg", false, false, 0, false, 0, false, 0, true, signed_neg32(cur_c64));
		}
	}

void tms57002test_state::probe_dsp3_serial_side_boundary_sensitivity()
{
	static constexpr char kProgramPath[] = "00-extracted/kprop_a02_sine_live_dump/dump_dsp3.bin";
	static constexpr char kBaseLogPath[] = "00-extracted/kprop_serial_side_boundary_20260408/baseline/error.log";
	static constexpr char kFlipLogPath[] = "00-extracted/kprop_serial_side_boundary_20260408/flip_input/error.log";
	static constexpr char kBaseRamPath[] = "00-extracted/kprop_serial_side_boundary_20260408/baseline/ram/su480001_pc00_pre.bin";
	static constexpr char kFlipRamPath[] = "00-extracted/kprop_serial_side_boundary_20260408/flip_input/ram/su480001_pc00_pre.bin";

	struct pc28_state
	{
		u64 macc_pre = 0;
		u64 macc_post = 0;
		u32 d03 = 0;
		u32 d04 = 0;
		u32 d05 = 0;
		u32 so0 = 0;
		u32 so1 = 0;
	};

	const std::vector<u32> program = load_program_words(kProgramPath);
	const std::vector<std::string> base_lines = load_text_lines(kBaseLogPath);
	const std::vector<std::string> flip_lines = load_text_lines(kFlipLogPath);
	const tms57002_device::debug_snapshot base = load_snapshot_for_su_from_lines(base_lines, 480001);
	const tms57002_device::debug_snapshot flip = load_snapshot_for_su_from_lines(flip_lines, 480001);

	auto run_variant = [&](const char *label, bool use_flip_seed, bool patch_d05_only, bool patch_d05_to_d0f, bool patch_dmem1) -> pc28_state
	{
		const tms57002_device::debug_snapshot &seed = use_flip_seed ? flip : base;
		const char *ram_path = use_flip_seed ? kFlipRamPath : kBaseRamPath;

		m_dsp->debug_load_program(program.data(), u32(program.size()), seed.st0, seed.st1);
		load_data_space_bytes(ram_path, m_dsp->space(AS_DATA));
		m_dsp->debug_restore_snapshot(seed);

		if (!use_flip_seed)
		{
			if (patch_d05_only)
			{
				const u8 phys = u8(base.ba0 + 0x05);
				m_dsp->debug_write_dmem0(phys, flip.dmem0[phys]);
			}
			if (patch_d05_to_d0f)
			{
				for (u8 logical = 0x05; logical <= 0x0f; logical++)
				{
					const u8 phys = u8(base.ba0 + logical);
					m_dsp->debug_write_dmem0(phys, flip.dmem0[phys]);
				}
			}
			if (patch_dmem1)
			{
				for (u8 idx = 0; idx < 0x20; idx++)
					m_dsp->debug_write_dmem1(idx, flip.dmem1[idx]);
			}
		}

		for (int step = 0; step < 0x200; step++)
		{
			if (m_dsp->state_int(TMS57002_PC) == 0x28 && m_dsp->state_int(TMS57002_ID) == 0x00)
			{
				pc28_state out;
				out.macc_pre = m_dsp->debug_macc();
				out.d03 = m_dsp->dmem0_value(u8(m_dsp->debug_ba0() + 0x03));
				out.d04 = m_dsp->dmem0_value(u8(m_dsp->debug_ba0() + 0x04));
				out.d05 = m_dsp->dmem0_value(u8(m_dsp->debug_ba0() + 0x05));
				m_dsp->debug_run_cycles(1);
				out.macc_post = m_dsp->debug_macc();
				out.so0 = m_dsp->debug_serial_output_register(0);
				out.so1 = m_dsp->debug_serial_output_register(1);
				osd_printf_info(
					"INFO dsp3 serialside boundary label=%s d03=%06x d04=%06x d05=%06x macc=%013llx->%013llx so=%06x/%06x\n",
					label,
					out.d03,
					out.d04,
					out.d05,
					(unsigned long long)out.macc_pre,
					(unsigned long long)out.macc_post,
					out.so0,
					out.so1);
				return out;
			}
			m_dsp->debug_run_cycles(1);
		}

		throw emu_fatalerror("Failed to reach DSP3 serial-side boundary PC28 for %s", label);
	};

	const pc28_state base_state = run_variant("base", false, false, false, false);
	const pc28_state flip_state = run_variant("flip", true, false, false, false);
	const pc28_state d05_state = run_variant("base+d05", false, true, false, false);
	const pc28_state d05_0f_state = run_variant("base+d05_0f", false, false, true, false);
	const pc28_state dmem1_state = run_variant("base+dmem1", false, false, false, true);
	const pc28_state combo_state = run_variant("base+d05_0f+dmem1", false, false, true, true);

	check_equal("dsp3 serialside base pc28 live macc", s64(base_state.macc_post), s64(0x00004f3fbfff5ULL));
	check_equal("dsp3 serialside flip pc28 live macc", s64(flip_state.macc_post), s64(0x0000531fffff5ULL));

	osd_printf_info(
		"INFO dsp3 serialside boundary delta base->flip=%013llx d05only=%013llx d05_0f=%013llx dmem1=%013llx combo=%013llx\n",
		(unsigned long long)(flip_state.macc_post - base_state.macc_post),
		(unsigned long long)(d05_state.macc_post - base_state.macc_post),
		(unsigned long long)(d05_0f_state.macc_post - base_state.macc_post),
		(unsigned long long)(dmem1_state.macc_post - base_state.macc_post),
		(unsigned long long)(combo_state.macc_post - base_state.macc_post));
}

void tms57002test_state::probe_dsp3_a02_fix_su95299_coeff_sensitivity()
{
	static constexpr char kProgramPath[] = "00-extracted/kprop_a02_sine_live_dump/dump_dsp3.bin";
	static constexpr char kLogPath[] = "tmp/kprop_a02_clip_tape_95296_v3/dspsnap.log";
	static constexpr char kRamDir[] = "tmp/kprop_a02_clip_tape_95296_v3/ram";
	static constexpr u64 kStartSu = 95299;
	static constexpr int kCyclesPerSample = 512;

	const std::vector<u32> program = load_program_words(kProgramPath);
	const std::vector<std::string> lines = load_text_lines(kLogPath);
	const tms57002_device::debug_snapshot prev = load_snapshot_for_su_from_lines(lines, kStartSu - 1);
	const tms57002_device::debug_snapshot start = load_snapshot_for_su_from_lines(lines, kStartSu);
	const tms57002_device::debug_snapshot expect = load_snapshot_for_su_from_lines(lines, kStartSu + 1);
	const tms57002_device::debug_snapshot expect2 = load_snapshot_for_su_from_lines(lines, kStartSu + 2);
	const tms57002_device::debug_snapshot expect3 = load_snapshot_for_su_from_lines(lines, kStartSu + 3);
	const std::string prev_ram_path = util::string_format("%s/su%06llu_pc00_pre.bin", kRamDir, (unsigned long long)(kStartSu - 1));
	const std::string start_ram_path = util::string_format("%s/su%06llu_pc00_pre.bin", kRamDir, (unsigned long long)kStartSu);

	struct boundary_state
	{
		u32 so0 = 0;
		u32 so1 = 0;
		s64 macc = 0;
		u32 aacc = 0;
		u8 ba0 = 0;
		u32 l50 = 0;
		u32 l53 = 0;
		u32 l56 = 0;
		u32 l59 = 0;
		u32 c42 = 0;
		u32 c4a = 0;
		u32 c4b = 0;
		u32 cc0 = 0;
		u32 cc1 = 0;
	};

	auto capture_boundary = [&](const char *label, const tms57002_device::debug_snapshot &seed, const char *ram_path = nullptr) -> boundary_state
	{
		m_dsp->debug_load_program(program.data(), u32(program.size()), seed.st0, seed.st1);
		load_data_space_bytes(ram_path ? ram_path : start_ram_path.c_str(), m_dsp->space(AS_DATA));
		m_dsp->debug_restore_snapshot(seed);
		for (int i = 0; i < kCyclesPerSample; i++)
			m_dsp->debug_run_cycles(1);
		m_dsp->debug_begin_sample_frame(expect.serial_input_frame);

		boundary_state out;
		out.so0 = m_dsp->debug_serial_output_register(0);
		out.so1 = m_dsp->debug_serial_output_register(1);
		out.macc = s64(m_dsp->debug_macc());
		out.aacc = u32(m_dsp->debug_aacc());
		out.ba0 = m_dsp->debug_ba0();
		out.l50 = m_dsp->dmem0_value((out.ba0 + 0x50) & 0xff);
		out.l53 = m_dsp->dmem0_value((out.ba0 + 0x53) & 0xff);
		out.l56 = m_dsp->dmem0_value((out.ba0 + 0x56) & 0xff);
		out.l59 = m_dsp->dmem0_value((out.ba0 + 0x59) & 0xff);
		out.c42 = m_dsp->cmem_value(0x42);
		out.c4a = m_dsp->cmem_value(0x4a);
		out.c4b = m_dsp->cmem_value(0x4b);
		out.cc0 = m_dsp->cmem_value(0xc0);
		out.cc1 = m_dsp->cmem_value(0xc1);

		osd_printf_info(
			"INFO dsp3 a02 fixcoeff label=%s so=%06x/%06x aacc=%08x macc=%013llx ba0=%02x l50=%06x l53=%06x l56=%06x l59=%06x "
			"c42=%08x c4a=%08x c4b=%08x cc0=%08x cc1=%08x\n",
			label,
			out.so0,
			out.so1,
			out.aacc,
			(unsigned long long)out.macc,
			out.ba0,
			out.l50,
			out.l53,
			out.l56,
			out.l59,
			out.c42,
			out.c4a,
			out.c4b,
			out.cc0,
			out.cc1);
		return out;
	};

	const boundary_state base = capture_boundary("base", start);

	auto copy_logical_from_prev = [&](tms57002_device::debug_snapshot &variant, std::initializer_list<u8> logical_addrs)
	{
		for (u8 logical : logical_addrs)
		{
			const u8 dst = u8(start.ba0 + logical);
			const u8 src = u8(prev.ba0 + logical);
			variant.dmem0[dst] = prev.dmem0[src];
		}
	};

	auto copy_serial = [&](tms57002_device::debug_snapshot &variant, const tms57002_device::debug_snapshot &src)
	{
		variant.si = src.si;
		variant.serial_input_latch = src.serial_input_latch;
		variant.serial_input_active = src.serial_input_active;
		variant.serial_input_frame = src.serial_input_frame;
		variant.serial_input_prev_frame = src.serial_input_prev_frame;
		variant.serial_input_pending = src.serial_input_pending;
		variant.serial_input_valid = src.serial_input_valid;
		variant.serial_input_active_valid = src.serial_input_active_valid;
		variant.serial_input_prev_valid = src.serial_input_prev_valid;
		variant.serial_input_pending_valid = src.serial_input_pending_valid;
		variant.serial_output_pending_valid = src.serial_output_pending_valid;
		variant.serial_frame_mode = src.serial_frame_mode;
		variant.serial_input_timing_mode = src.serial_input_timing_mode;
		variant.serial_frame_clocks = src.serial_frame_clocks;
		variant.serial_exec_halfcycles = src.serial_exec_halfcycles;
		variant.serial_input_pending_halfcycle = src.serial_input_pending_halfcycle;
		variant.serial_output_pending_halfcycle = src.serial_output_pending_halfcycle;
		variant.sync_polarity_rising = src.sync_polarity_rising;
		variant.serial_output_muted = src.serial_output_muted;
	};

	auto capture_boundary_with_runtime_patch = [&](const char *label, bool patch_cc0, bool patch_cc1) -> boundary_state
	{
		m_dsp->debug_load_program(program.data(), u32(program.size()), start.st0, start.st1);
		load_data_space_bytes(start_ram_path.c_str(), m_dsp->space(AS_DATA));
		m_dsp->debug_restore_snapshot(start);
		for (int i = 0; i < kCyclesPerSample; i++)
		{
			const u8 executed_pc = u8(m_dsp->state_int(TMS57002_PC));
			const u8 executed_id = u8(m_dsp->state_int(TMS57002_ID));
			m_dsp->debug_run_cycles(1);
			if (executed_id == 0x1f && executed_pc == 0x34 && patch_cc0)
				m_dsp->debug_write_cmem(0xc0, prev.cmem[0xc0]);
			if (executed_id == 0x1f && executed_pc == 0x3a && patch_cc1)
				m_dsp->debug_write_cmem(0xc1, prev.cmem[0xc1]);
		}
		m_dsp->debug_begin_sample_frame(expect.serial_input_frame);

		boundary_state out;
		out.so0 = m_dsp->debug_serial_output_register(0);
		out.so1 = m_dsp->debug_serial_output_register(1);
		out.macc = s64(m_dsp->debug_macc());
		out.aacc = u32(m_dsp->debug_aacc());
		out.ba0 = m_dsp->debug_ba0();
		out.l50 = m_dsp->dmem0_value((out.ba0 + 0x50) & 0xff);
		out.l53 = m_dsp->dmem0_value((out.ba0 + 0x53) & 0xff);
		out.l56 = m_dsp->dmem0_value((out.ba0 + 0x56) & 0xff);
		out.l59 = m_dsp->dmem0_value((out.ba0 + 0x59) & 0xff);
		out.c42 = m_dsp->cmem_value(0x42);
		out.c4a = m_dsp->cmem_value(0x4a);
		out.c4b = m_dsp->cmem_value(0x4b);
		out.cc0 = m_dsp->cmem_value(0xc0);
		out.cc1 = m_dsp->cmem_value(0xc1);

		osd_printf_info(
			"INFO dsp3 a02 fixcoeff label=%s so=%06x/%06x aacc=%08x macc=%013llx ba0=%02x l50=%06x l53=%06x l56=%06x l59=%06x "
			"c42=%08x c4a=%08x c4b=%08x cc0=%08x cc1=%08x\n",
			label,
			out.so0,
			out.so1,
			out.aacc,
			(unsigned long long)out.macc,
			out.ba0,
			out.l50,
			out.l53,
			out.l56,
			out.l59,
			out.c42,
			out.c4a,
			out.c4b,
			out.cc0,
			out.cc1);
		return out;
	};

	auto capture_boundary_with_store_restore = [&](const char *label, bool restore_l50, bool restore_l53, bool restore_l56, bool restore_l59) -> boundary_state
	{
		auto start_logical = [&](u8 logical) -> u32
		{
			return start.dmem0[u8(start.ba0 + logical)];
		};

		m_dsp->debug_load_program(program.data(), u32(program.size()), start.st0, start.st1);
		load_data_space_bytes(start_ram_path.c_str(), m_dsp->space(AS_DATA));
		m_dsp->debug_restore_snapshot(start);
		for (int i = 0; i < kCyclesPerSample; i++)
		{
			const u8 executed_pc = u8(m_dsp->state_int(TMS57002_PC));
			const u8 executed_id = u8(m_dsp->state_int(TMS57002_ID));
			m_dsp->debug_run_cycles(1);
			if (executed_id != 0x1f)
				continue;
			const u8 ba0_cur = m_dsp->debug_ba0();
			if (executed_pc == 0xaa && restore_l50)
				m_dsp->debug_write_dmem0(u8(ba0_cur + 0x50), start_logical(0x50));
			if (executed_pc == 0xb0 && restore_l53)
				m_dsp->debug_write_dmem0(u8(ba0_cur + 0x53), start_logical(0x53));
			if (executed_pc == 0xb6 && restore_l56)
				m_dsp->debug_write_dmem0(u8(ba0_cur + 0x56), start_logical(0x56));
			if (executed_pc == 0xbc && restore_l59)
				m_dsp->debug_write_dmem0(u8(ba0_cur + 0x59), start_logical(0x59));
		}
		m_dsp->debug_begin_sample_frame(expect.serial_input_frame);

		boundary_state out;
		out.so0 = m_dsp->debug_serial_output_register(0);
		out.so1 = m_dsp->debug_serial_output_register(1);
		out.macc = s64(m_dsp->debug_macc());
		out.aacc = u32(m_dsp->debug_aacc());
		out.ba0 = m_dsp->debug_ba0();
		out.l50 = m_dsp->dmem0_value((out.ba0 + 0x50) & 0xff);
		out.l53 = m_dsp->dmem0_value((out.ba0 + 0x53) & 0xff);
		out.l56 = m_dsp->dmem0_value((out.ba0 + 0x56) & 0xff);
		out.l59 = m_dsp->dmem0_value((out.ba0 + 0x59) & 0xff);
		out.c42 = m_dsp->cmem_value(0x42);
		out.c4a = m_dsp->cmem_value(0x4a);
		out.c4b = m_dsp->cmem_value(0x4b);
		out.cc0 = m_dsp->cmem_value(0xc0);
		out.cc1 = m_dsp->cmem_value(0xc1);

		osd_printf_info(
			"INFO dsp3 a02 fixcoeff label=%s so=%06x/%06x aacc=%08x macc=%013llx ba0=%02x l50=%06x l53=%06x l56=%06x l59=%06x "
			"c42=%08x c4a=%08x c4b=%08x cc0=%08x cc1=%08x\n",
			label,
			out.so0,
			out.so1,
			out.aacc,
			(unsigned long long)out.macc,
			out.ba0,
			out.l50,
			out.l53,
			out.l56,
			out.l59,
			out.c42,
			out.c4a,
			out.c4b,
			out.cc0,
			out.cc1);
		return out;
	};

	auto capture_boundary_h3_with_store_restore = [&](const char *label, bool restore_l50, bool restore_l53, bool restore_l56, bool restore_l59) -> boundary_state
	{
		auto start_logical = [&](u8 logical) -> u32
		{
			return start.dmem0[u8(start.ba0 + logical)];
		};

		m_dsp->debug_load_program(program.data(), u32(program.size()), start.st0, start.st1);
		load_data_space_bytes(start_ram_path.c_str(), m_dsp->space(AS_DATA));
		m_dsp->debug_restore_snapshot(start);
		for (int i = 0; i < kCyclesPerSample; i++)
		{
			const u8 executed_pc = u8(m_dsp->state_int(TMS57002_PC));
			const u8 executed_id = u8(m_dsp->state_int(TMS57002_ID));
			m_dsp->debug_run_cycles(1);
			if (executed_id != 0x1f)
				continue;
			const u8 ba0_cur = m_dsp->debug_ba0();
			if (executed_pc == 0xaa && restore_l50)
				m_dsp->debug_write_dmem0(u8(ba0_cur + 0x50), start_logical(0x50));
			if (executed_pc == 0xb0 && restore_l53)
				m_dsp->debug_write_dmem0(u8(ba0_cur + 0x53), start_logical(0x53));
			if (executed_pc == 0xb6 && restore_l56)
				m_dsp->debug_write_dmem0(u8(ba0_cur + 0x56), start_logical(0x56));
			if (executed_pc == 0xbc && restore_l59)
				m_dsp->debug_write_dmem0(u8(ba0_cur + 0x59), start_logical(0x59));
		}
		m_dsp->debug_begin_sample_frame(expect.serial_input_frame);
		for (int i = 0; i < kCyclesPerSample; i++)
			m_dsp->debug_run_cycles(1);
		m_dsp->debug_begin_sample_frame(expect2.serial_input_frame);
		for (int i = 0; i < kCyclesPerSample; i++)
			m_dsp->debug_run_cycles(1);
		m_dsp->debug_begin_sample_frame(expect3.serial_input_frame);

		boundary_state out;
		out.so0 = m_dsp->debug_serial_output_register(0);
		out.so1 = m_dsp->debug_serial_output_register(1);
		out.macc = s64(m_dsp->debug_macc());
		out.aacc = u32(m_dsp->debug_aacc());
		out.ba0 = m_dsp->debug_ba0();
		out.l50 = m_dsp->dmem0_value((out.ba0 + 0x50) & 0xff);
		out.l53 = m_dsp->dmem0_value((out.ba0 + 0x53) & 0xff);
		out.l56 = m_dsp->dmem0_value((out.ba0 + 0x56) & 0xff);
		out.l59 = m_dsp->dmem0_value((out.ba0 + 0x59) & 0xff);
		out.c42 = m_dsp->cmem_value(0x42);
		out.c4a = m_dsp->cmem_value(0x4a);
		out.c4b = m_dsp->cmem_value(0x4b);
		out.cc0 = m_dsp->cmem_value(0xc0);
		out.cc1 = m_dsp->cmem_value(0xc1);

		osd_printf_info(
			"INFO dsp3 a02 fixcoeff_h3 label=%s so=%06x/%06x aacc=%08x macc=%013llx ba0=%02x l50=%06x l53=%06x l56=%06x l59=%06x "
			"c42=%08x c4a=%08x c4b=%08x cc0=%08x cc1=%08x\n",
			label,
			out.so0,
			out.so1,
			out.aacc,
			(unsigned long long)out.macc,
			out.ba0,
			out.l50,
			out.l53,
			out.l56,
			out.l59,
			out.c42,
			out.c4a,
			out.c4b,
			out.cc0,
			out.cc1);
		return out;
	};

	auto capture_boundary_with_accum_patch = [&](const char *label, bool patch_second_b4, bool patch_second_ba) -> boundary_state
	{
		static constexpr u32 kB4Aacc = 0x4031676e;
		static constexpr u64 kB4Macc = 0x017e59e65d99cULL;
		static constexpr u64 kB4Maccr = 0xfffff0850ecabf49ULL;
		static constexpr u64 kB4Maccw = 0xfffff81b0eb5d99cULL;
		static constexpr u32 kB4Creg = 0xe0357050;

		static constexpr u32 kBaAacc = 0x7f4bca68;
		static constexpr u64 kBaMacc = 0x02fcb3c7f40d3ULL;
		static constexpr u64 kBaMaccr = 0x017e59e65d99cULL;
		static constexpr u64 kBaMaccw = 0x017e59e65d99cULL;
		static constexpr u32 kBaCreg = 0x3fffffff;

		int b4_seen = 0;
		int ba_seen = 0;

		m_dsp->debug_load_program(program.data(), u32(program.size()), start.st0, start.st1);
		load_data_space_bytes(start_ram_path.c_str(), m_dsp->space(AS_DATA));
		m_dsp->debug_restore_snapshot(start);
		for (int i = 0; i < kCyclesPerSample; i++)
		{
			const u8 executed_pc = u8(m_dsp->state_int(TMS57002_PC));
			const u8 executed_id = u8(m_dsp->state_int(TMS57002_ID));
			m_dsp->debug_run_cycles(1);
			if (executed_id != 0x1f)
				continue;
			if (executed_pc == 0xb4)
			{
				b4_seen++;
				if (patch_second_b4 && b4_seen == 2)
					m_dsp->debug_set_accumulators(kB4Aacc, kB4Macc, kB4Maccr, kB4Maccw, kB4Creg);
			}
			if (executed_pc == 0xba)
			{
				ba_seen++;
				if (patch_second_ba && ba_seen == 2)
					m_dsp->debug_set_accumulators(kBaAacc, kBaMacc, kBaMaccr, kBaMaccw, kBaCreg);
			}
		}
		m_dsp->debug_begin_sample_frame(expect.serial_input_frame);

		boundary_state out;
		out.so0 = m_dsp->debug_serial_output_register(0);
		out.so1 = m_dsp->debug_serial_output_register(1);
		out.macc = s64(m_dsp->debug_macc());
		out.aacc = u32(m_dsp->debug_aacc());
		out.ba0 = m_dsp->debug_ba0();
		out.l50 = m_dsp->dmem0_value((out.ba0 + 0x50) & 0xff);
		out.l53 = m_dsp->dmem0_value((out.ba0 + 0x53) & 0xff);
		out.l56 = m_dsp->dmem0_value((out.ba0 + 0x56) & 0xff);
		out.l59 = m_dsp->dmem0_value((out.ba0 + 0x59) & 0xff);
		out.c42 = m_dsp->cmem_value(0x42);
		out.c4a = m_dsp->cmem_value(0x4a);
		out.c4b = m_dsp->cmem_value(0x4b);
		out.cc0 = m_dsp->cmem_value(0xc0);
		out.cc1 = m_dsp->cmem_value(0xc1);

		osd_printf_info(
			"INFO dsp3 a02 fixcoeff label=%s so=%06x/%06x aacc=%08x macc=%013llx ba0=%02x l50=%06x l53=%06x l56=%06x l59=%06x "
			"c42=%08x c4a=%08x c4b=%08x cc0=%08x cc1=%08x\n",
			label,
			out.so0,
			out.so1,
			out.aacc,
			(unsigned long long)out.macc,
			out.ba0,
			out.l50,
			out.l53,
			out.l56,
			out.l59,
			out.c42,
			out.c4a,
			out.c4b,
			out.cc0,
			out.cc1);
		return out;
	};

	auto capture_boundary_with_next_frame_override = [&](const char *label, const std::array<u32, 4> &next_frame) -> boundary_state
	{
		m_dsp->debug_load_program(program.data(), u32(program.size()), start.st0, start.st1);
		load_data_space_bytes(start_ram_path.c_str(), m_dsp->space(AS_DATA));
		m_dsp->debug_restore_snapshot(start);
		for (int i = 0; i < kCyclesPerSample; i++)
			m_dsp->debug_run_cycles(1);
		m_dsp->debug_begin_sample_frame(next_frame);

		boundary_state out;
		out.so0 = m_dsp->debug_serial_output_register(0);
		out.so1 = m_dsp->debug_serial_output_register(1);
		out.macc = s64(m_dsp->debug_macc());
		out.aacc = u32(m_dsp->debug_aacc());
		out.ba0 = m_dsp->debug_ba0();
		out.l50 = m_dsp->dmem0_value((out.ba0 + 0x50) & 0xff);
		out.l53 = m_dsp->dmem0_value((out.ba0 + 0x53) & 0xff);
		out.l56 = m_dsp->dmem0_value((out.ba0 + 0x56) & 0xff);
		out.l59 = m_dsp->dmem0_value((out.ba0 + 0x59) & 0xff);
		out.c42 = m_dsp->cmem_value(0x42);
		out.c4a = m_dsp->cmem_value(0x4a);
		out.c4b = m_dsp->cmem_value(0x4b);
		out.cc0 = m_dsp->cmem_value(0xc0);
		out.cc1 = m_dsp->cmem_value(0xc1);

		osd_printf_info(
			"INFO dsp3 a02 fixcoeff label=%s so=%06x/%06x aacc=%08x macc=%013llx ba0=%02x l50=%06x l53=%06x l56=%06x l59=%06x "
			"c42=%08x c4a=%08x c4b=%08x cc0=%08x cc1=%08x\n",
			label,
			out.so0,
			out.so1,
			out.aacc,
			(unsigned long long)out.macc,
			out.ba0,
			out.l50,
			out.l53,
			out.l56,
			out.l59,
			out.c42,
			out.c4a,
			out.c4b,
			out.cc0,
			out.cc1);
		return out;
	};

	auto capture_boundary_h3_with_frame_override = [&](const char *label, const std::array<u32, 4> &frame1, const std::array<u32, 4> &frame2, const std::array<u32, 4> &frame3) -> boundary_state
	{
		m_dsp->debug_load_program(program.data(), u32(program.size()), start.st0, start.st1);
		load_data_space_bytes(start_ram_path.c_str(), m_dsp->space(AS_DATA));
		m_dsp->debug_restore_snapshot(start);
		for (int i = 0; i < kCyclesPerSample; i++)
			m_dsp->debug_run_cycles(1);
		m_dsp->debug_begin_sample_frame(frame1);
		for (int i = 0; i < kCyclesPerSample; i++)
			m_dsp->debug_run_cycles(1);
		m_dsp->debug_begin_sample_frame(frame2);
		for (int i = 0; i < kCyclesPerSample; i++)
			m_dsp->debug_run_cycles(1);
		m_dsp->debug_begin_sample_frame(frame3);

		boundary_state out;
		out.so0 = m_dsp->debug_serial_output_register(0);
		out.so1 = m_dsp->debug_serial_output_register(1);
		out.macc = s64(m_dsp->debug_macc());
		out.aacc = u32(m_dsp->debug_aacc());
		out.ba0 = m_dsp->debug_ba0();
		out.l50 = m_dsp->dmem0_value((out.ba0 + 0x50) & 0xff);
		out.l53 = m_dsp->dmem0_value((out.ba0 + 0x53) & 0xff);
		out.l56 = m_dsp->dmem0_value((out.ba0 + 0x56) & 0xff);
		out.l59 = m_dsp->dmem0_value((out.ba0 + 0x59) & 0xff);
		out.c42 = m_dsp->cmem_value(0x42);
		out.c4a = m_dsp->cmem_value(0x4a);
		out.c4b = m_dsp->cmem_value(0x4b);
		out.cc0 = m_dsp->cmem_value(0xc0);
		out.cc1 = m_dsp->cmem_value(0xc1);

		osd_printf_info(
			"INFO dsp3 a02 fixcoeff_h3 label=%s so=%06x/%06x aacc=%08x macc=%013llx ba0=%02x l50=%06x l53=%06x l56=%06x l59=%06x "
			"c42=%08x c4a=%08x c4b=%08x cc0=%08x cc1=%08x\n",
			label,
			out.so0,
			out.so1,
			out.aacc,
			(unsigned long long)out.macc,
			out.ba0,
			out.l50,
			out.l53,
			out.l56,
			out.l59,
			out.c42,
			out.c4a,
			out.c4b,
			out.cc0,
			out.cc1);
		return out;
	};

	auto run_variant = [&](const char *label, std::initializer_list<u8> cmem_addrs)
	{
		tms57002_device::debug_snapshot variant = start;
		for (u8 addr : cmem_addrs)
			variant.cmem[addr] = prev.cmem[addr];
		const boundary_state state = capture_boundary(label, variant);
		osd_printf_info(
			"INFO dsp3 a02 fixcoeff delta label=%s dso=%d/%d dl50=%d dl53=%d dl56=%d dl59=%d dmacc=%lld\n",
			label,
			s32(state.so0) - s32(base.so0),
			s32(state.so1) - s32(base.so1),
			s32(state.l50) - s32(base.l50),
			s32(state.l53) - s32(base.l53),
			s32(state.l56) - s32(base.l56),
			s32(state.l59) - s32(base.l59),
			(long long)(state.macc - base.macc));
	};

	run_variant("prev_c42", { 0x42 });
	run_variant("prev_c4a", { 0x4a });
	run_variant("prev_c4b", { 0x4b });
	run_variant("prev_c42_4a_4b", { 0x42, 0x4a, 0x4b });
	run_variant("prev_c13", { 0x13 });
	run_variant("prev_c23", { 0x23 });
	run_variant("prev_c38", { 0x38 });
	run_variant("prev_c3a", { 0x3a });
	run_variant("prev_c3c", { 0x3c });
	run_variant("prev_c3d", { 0x3d });
	run_variant("prev_ca4", { 0xa4 });
	run_variant("prev_dyncoeffs", { 0x13, 0x23, 0x38, 0x3a, 0x3c, 0x3d, 0xa4 });
	run_variant("prev_dyncoeffs_cc", { 0x13, 0x23, 0x38, 0x3a, 0x3c, 0x3d, 0xa4, 0xc0, 0xc1 });
	run_variant("prev_cc0_cc1", { 0xc0, 0xc1 });
	run_variant("prev_all5", { 0x42, 0x4a, 0x4b, 0xc0, 0xc1 });
	{
		tms57002_device::debug_snapshot variant = start;
		variant.aacc = prev.aacc;
		variant.creg = prev.creg;
		const boundary_state state = capture_boundary("prev_aacc_creg", variant);
		osd_printf_info(
			"INFO dsp3 a02 fixcoeff delta label=prev_aacc_creg dso=%d/%d dl50=%d dl53=%d dl56=%d dl59=%d dmacc=%lld\n",
			s32(state.so0) - s32(base.so0),
			s32(state.so1) - s32(base.so1),
			s32(state.l50) - s32(base.l50),
			s32(state.l53) - s32(base.l53),
			s32(state.l56) - s32(base.l56),
			s32(state.l59) - s32(base.l59),
			(long long)(state.macc - base.macc));
	}
	{
		tms57002_device::debug_snapshot variant = start;
		variant.macc = prev.macc;
		variant.macc_read = prev.macc_read;
		variant.macc_write = prev.macc_write;
		const boundary_state state = capture_boundary("prev_macc_triplet", variant);
		osd_printf_info(
			"INFO dsp3 a02 fixcoeff delta label=prev_macc_triplet dso=%d/%d dl50=%d dl53=%d dl56=%d dl59=%d dmacc=%lld\n",
			s32(state.so0) - s32(base.so0),
			s32(state.so1) - s32(base.so1),
			s32(state.l50) - s32(base.l50),
			s32(state.l53) - s32(base.l53),
			s32(state.l56) - s32(base.l56),
			s32(state.l59) - s32(base.l59),
			(long long)(state.macc - base.macc));
	}
	{
		tms57002_device::debug_snapshot variant = start;
		variant.aacc = 0;
		variant.creg = 0;
		variant.macc = 0;
		variant.macc_read = 0;
		variant.macc_write = 0;
		const boundary_state state = capture_boundary("zero_accums", variant);
		osd_printf_info(
			"INFO dsp3 a02 fixcoeff delta label=zero_accums dso=%d/%d dl50=%d dl53=%d dl56=%d dl59=%d dmacc=%lld\n",
			s32(state.so0) - s32(base.so0),
			s32(state.so1) - s32(base.so1),
			s32(state.l50) - s32(base.l50),
			s32(state.l53) - s32(base.l53),
			s32(state.l56) - s32(base.l56),
			s32(state.l59) - s32(base.l59),
			(long long)(state.macc - base.macc));
	}

	{
		tms57002_device::debug_snapshot variant = start;
		copy_logical_from_prev(variant, { 0x56 });
		const boundary_state state = capture_boundary("prev_l56", variant);
		osd_printf_info(
			"INFO dsp3 a02 fixcoeff delta label=prev_l56 dso=%d/%d dl50=%d dl53=%d dl56=%d dl59=%d dmacc=%lld\n",
			s32(state.so0) - s32(base.so0),
			s32(state.so1) - s32(base.so1),
			s32(state.l50) - s32(base.l50),
			s32(state.l53) - s32(base.l53),
			s32(state.l56) - s32(base.l56),
			s32(state.l59) - s32(base.l59),
			(long long)(state.macc - base.macc));
	}
	{
		tms57002_device::debug_snapshot variant = start;
		copy_logical_from_prev(variant, { 0x59 });
		const boundary_state state = capture_boundary("prev_l59", variant);
		osd_printf_info(
			"INFO dsp3 a02 fixcoeff delta label=prev_l59 dso=%d/%d dl50=%d dl53=%d dl56=%d dl59=%d dmacc=%lld\n",
			s32(state.so0) - s32(base.so0),
			s32(state.so1) - s32(base.so1),
			s32(state.l50) - s32(base.l50),
			s32(state.l53) - s32(base.l53),
			s32(state.l56) - s32(base.l56),
			s32(state.l59) - s32(base.l59),
			(long long)(state.macc - base.macc));
	}
	{
		tms57002_device::debug_snapshot variant = start;
		copy_logical_from_prev(variant, { 0x50, 0x53, 0x56, 0x59 });
		const boundary_state state = capture_boundary("prev_l50_53_56_59", variant);
		osd_printf_info(
			"INFO dsp3 a02 fixcoeff delta label=prev_l50_53_56_59 dso=%d/%d dl50=%d dl53=%d dl56=%d dl59=%d dmacc=%lld\n",
			s32(state.so0) - s32(base.so0),
			s32(state.so1) - s32(base.so1),
			s32(state.l50) - s32(base.l50),
			s32(state.l53) - s32(base.l53),
			s32(state.l56) - s32(base.l56),
			s32(state.l59) - s32(base.l59),
			(long long)(state.macc - base.macc));
	}
	{
		tms57002_device::debug_snapshot variant = start;
		variant.si = { 0, 0, 0, 0 };
		variant.serial_input_latch = { 0, 0, 0, 0 };
		variant.serial_input_active = { 0, 0, 0, 0 };
		variant.serial_input_frame = { 0, 0, 0, 0 };
		variant.serial_input_prev_frame = { 0, 0, 0, 0 };
		variant.serial_input_pending = { 0, 0, 0, 0 };
		variant.serial_input_valid = 0;
		variant.serial_input_active_valid = 0;
		variant.serial_input_prev_valid = 0;
		variant.serial_input_pending_valid = 0;
		variant.serial_output_pending_valid = 0;
		const boundary_state state = capture_boundary("zero_serial", variant);
		osd_printf_info(
			"INFO dsp3 a02 fixcoeff delta label=zero_serial dso=%d/%d dl50=%d dl53=%d dl56=%d dl59=%d dmacc=%lld\n",
			s32(state.so0) - s32(base.so0),
			s32(state.so1) - s32(base.so1),
			s32(state.l50) - s32(base.l50),
			s32(state.l53) - s32(base.l53),
			s32(state.l56) - s32(base.l56),
			s32(state.l59) - s32(base.l59),
			(long long)(state.macc - base.macc));
	}
	{
		tms57002_device::debug_snapshot variant = start;
		copy_serial(variant, prev);
		const boundary_state state = capture_boundary("prev_serial", variant);
		osd_printf_info(
			"INFO dsp3 a02 fixcoeff delta label=prev_serial dso=%d/%d dl50=%d dl53=%d dl56=%d dl59=%d dmacc=%lld\n",
			s32(state.so0) - s32(base.so0),
			s32(state.so1) - s32(base.so1),
			s32(state.l50) - s32(base.l50),
			s32(state.l53) - s32(base.l53),
			s32(state.l56) - s32(base.l56),
			s32(state.l59) - s32(base.l59),
			(long long)(state.macc - base.macc));
	}
	{
		const boundary_state state = capture_boundary("prev_xmem", start, prev_ram_path.c_str());
		osd_printf_info(
			"INFO dsp3 a02 fixcoeff delta label=prev_xmem dso=%d/%d dl50=%d dl53=%d dl56=%d dl59=%d dmacc=%lld\n",
			s32(state.so0) - s32(base.so0),
			s32(state.so1) - s32(base.so1),
			s32(state.l50) - s32(base.l50),
			s32(state.l53) - s32(base.l53),
			s32(state.l56) - s32(base.l56),
			s32(state.l59) - s32(base.l59),
			(long long)(state.macc - base.macc));
	}
	{
		const boundary_state state = capture_boundary_with_runtime_patch("mid_prev_cc0", true, false);
		osd_printf_info(
			"INFO dsp3 a02 fixcoeff delta label=mid_prev_cc0 dso=%d/%d dl50=%d dl53=%d dl56=%d dl59=%d dmacc=%lld\n",
			s32(state.so0) - s32(base.so0),
			s32(state.so1) - s32(base.so1),
			s32(state.l50) - s32(base.l50),
			s32(state.l53) - s32(base.l53),
			s32(state.l56) - s32(base.l56),
			s32(state.l59) - s32(base.l59),
			(long long)(state.macc - base.macc));
	}
	{
		const boundary_state state = capture_boundary_with_runtime_patch("mid_prev_cc1", false, true);
		osd_printf_info(
			"INFO dsp3 a02 fixcoeff delta label=mid_prev_cc1 dso=%d/%d dl50=%d dl53=%d dl56=%d dl59=%d dmacc=%lld\n",
			s32(state.so0) - s32(base.so0),
			s32(state.so1) - s32(base.so1),
			s32(state.l50) - s32(base.l50),
			s32(state.l53) - s32(base.l53),
			s32(state.l56) - s32(base.l56),
			s32(state.l59) - s32(base.l59),
			(long long)(state.macc - base.macc));
	}
	{
		const boundary_state state = capture_boundary_with_runtime_patch("mid_prev_cc0_cc1", true, true);
		osd_printf_info(
			"INFO dsp3 a02 fixcoeff delta label=mid_prev_cc0_cc1 dso=%d/%d dl50=%d dl53=%d dl56=%d dl59=%d dmacc=%lld\n",
			s32(state.so0) - s32(base.so0),
			s32(state.so1) - s32(base.so1),
			s32(state.l50) - s32(base.l50),
			s32(state.l53) - s32(base.l53),
			s32(state.l56) - s32(base.l56),
			s32(state.l59) - s32(base.l59),
			(long long)(state.macc - base.macc));
	}
	{
		const boundary_state state = capture_boundary_with_store_restore("mid_start_l50", true, false, false, false);
		osd_printf_info(
			"INFO dsp3 a02 fixcoeff delta label=mid_start_l50 dso=%d/%d dl50=%d dl53=%d dl56=%d dl59=%d dmacc=%lld\n",
			s32(state.so0) - s32(base.so0),
			s32(state.so1) - s32(base.so1),
			s32(state.l50) - s32(base.l50),
			s32(state.l53) - s32(base.l53),
			s32(state.l56) - s32(base.l56),
			s32(state.l59) - s32(base.l59),
			(long long)(state.macc - base.macc));
	}
	{
		const boundary_state state = capture_boundary_with_store_restore("mid_start_l53", false, true, false, false);
		osd_printf_info(
			"INFO dsp3 a02 fixcoeff delta label=mid_start_l53 dso=%d/%d dl50=%d dl53=%d dl56=%d dl59=%d dmacc=%lld\n",
			s32(state.so0) - s32(base.so0),
			s32(state.so1) - s32(base.so1),
			s32(state.l50) - s32(base.l50),
			s32(state.l53) - s32(base.l53),
			s32(state.l56) - s32(base.l56),
			s32(state.l59) - s32(base.l59),
			(long long)(state.macc - base.macc));
	}
	{
		const boundary_state state = capture_boundary_with_store_restore("mid_start_l56", false, false, true, false);
		osd_printf_info(
			"INFO dsp3 a02 fixcoeff delta label=mid_start_l56 dso=%d/%d dl50=%d dl53=%d dl56=%d dl59=%d dmacc=%lld\n",
			s32(state.so0) - s32(base.so0),
			s32(state.so1) - s32(base.so1),
			s32(state.l50) - s32(base.l50),
			s32(state.l53) - s32(base.l53),
			s32(state.l56) - s32(base.l56),
			s32(state.l59) - s32(base.l59),
			(long long)(state.macc - base.macc));
	}
	{
		const boundary_state state = capture_boundary_with_store_restore("mid_start_l59", false, false, false, true);
		osd_printf_info(
			"INFO dsp3 a02 fixcoeff delta label=mid_start_l59 dso=%d/%d dl50=%d dl53=%d dl56=%d dl59=%d dmacc=%lld\n",
			s32(state.so0) - s32(base.so0),
			s32(state.so1) - s32(base.so1),
			s32(state.l50) - s32(base.l50),
			s32(state.l53) - s32(base.l53),
			s32(state.l56) - s32(base.l56),
			s32(state.l59) - s32(base.l59),
			(long long)(state.macc - base.macc));
	}
	{
		const boundary_state state = capture_boundary_with_store_restore("mid_start_l56_l59", false, false, true, true);
		osd_printf_info(
			"INFO dsp3 a02 fixcoeff delta label=mid_start_l56_l59 dso=%d/%d dl50=%d dl53=%d dl56=%d dl59=%d dmacc=%lld\n",
			s32(state.so0) - s32(base.so0),
			s32(state.so1) - s32(base.so1),
			s32(state.l50) - s32(base.l50),
			s32(state.l53) - s32(base.l53),
			s32(state.l56) - s32(base.l56),
			s32(state.l59) - s32(base.l59),
			(long long)(state.macc - base.macc));
	}
	{
		const boundary_state base_h3 = capture_boundary_h3_with_store_restore("base_h3", false, false, false, false);
		const boundary_state state = capture_boundary_h3_with_store_restore("mid_start_l56_h3", false, false, true, false);
		osd_printf_info(
			"INFO dsp3 a02 fixcoeff_h3 delta label=mid_start_l56_h3 dso=%d/%d dl50=%d dl53=%d dl56=%d dl59=%d dmacc=%lld\n",
			s32(state.so0) - s32(base_h3.so0),
			s32(state.so1) - s32(base_h3.so1),
			s32(state.l50) - s32(base_h3.l50),
			s32(state.l53) - s32(base_h3.l53),
			s32(state.l56) - s32(base_h3.l56),
			s32(state.l59) - s32(base_h3.l59),
			(long long)(state.macc - base_h3.macc));
	}
	{
		const boundary_state base_h3 = capture_boundary_h3_with_store_restore("base_h3", false, false, false, false);
		const boundary_state state = capture_boundary_h3_with_store_restore("mid_start_l59_h3", false, false, false, true);
		osd_printf_info(
			"INFO dsp3 a02 fixcoeff_h3 delta label=mid_start_l59_h3 dso=%d/%d dl50=%d dl53=%d dl56=%d dl59=%d dmacc=%lld\n",
			s32(state.so0) - s32(base_h3.so0),
			s32(state.so1) - s32(base_h3.so1),
			s32(state.l50) - s32(base_h3.l50),
			s32(state.l53) - s32(base_h3.l53),
			s32(state.l56) - s32(base_h3.l56),
			s32(state.l59) - s32(base_h3.l59),
			(long long)(state.macc - base_h3.macc));
	}
	{
		const boundary_state base_h3 = capture_boundary_h3_with_store_restore("base_h3", false, false, false, false);
		const boundary_state state = capture_boundary_h3_with_store_restore("mid_start_l56_l59_h3", false, false, true, true);
		osd_printf_info(
			"INFO dsp3 a02 fixcoeff_h3 delta label=mid_start_l56_l59_h3 dso=%d/%d dl50=%d dl53=%d dl56=%d dl59=%d dmacc=%lld\n",
			s32(state.so0) - s32(base_h3.so0),
			s32(state.so1) - s32(base_h3.so1),
			s32(state.l50) - s32(base_h3.l50),
			s32(state.l53) - s32(base_h3.l53),
			s32(state.l56) - s32(base_h3.l56),
			s32(state.l59) - s32(base_h3.l59),
			(long long)(state.macc - base_h3.macc));
	}
	{
		const boundary_state state = capture_boundary_with_accum_patch("mid_firstpass_b4", true, false);
		osd_printf_info(
			"INFO dsp3 a02 fixcoeff delta label=mid_firstpass_b4 dso=%d/%d dl50=%d dl53=%d dl56=%d dl59=%d dmacc=%lld\n",
			s32(state.so0) - s32(base.so0),
			s32(state.so1) - s32(base.so1),
			s32(state.l50) - s32(base.l50),
			s32(state.l53) - s32(base.l53),
			s32(state.l56) - s32(base.l56),
			s32(state.l59) - s32(base.l59),
			(long long)(state.macc - base.macc));
	}
	{
		const boundary_state state = capture_boundary_with_accum_patch("mid_firstpass_ba", false, true);
		osd_printf_info(
			"INFO dsp3 a02 fixcoeff delta label=mid_firstpass_ba dso=%d/%d dl50=%d dl53=%d dl56=%d dl59=%d dmacc=%lld\n",
			s32(state.so0) - s32(base.so0),
			s32(state.so1) - s32(base.so1),
			s32(state.l50) - s32(base.l50),
			s32(state.l53) - s32(base.l53),
			s32(state.l56) - s32(base.l56),
			s32(state.l59) - s32(base.l59),
			(long long)(state.macc - base.macc));
	}
	{
		const boundary_state state = capture_boundary_with_accum_patch("mid_firstpass_b4_ba", true, true);
		osd_printf_info(
			"INFO dsp3 a02 fixcoeff delta label=mid_firstpass_b4_ba dso=%d/%d dl50=%d dl53=%d dl56=%d dl59=%d dmacc=%lld\n",
			s32(state.so0) - s32(base.so0),
			s32(state.so1) - s32(base.so1),
			s32(state.l50) - s32(base.l50),
			s32(state.l53) - s32(base.l53),
			s32(state.l56) - s32(base.l56),
			s32(state.l59) - s32(base.l59),
			(long long)(state.macc - base.macc));
	}
	{
		const boundary_state state = capture_boundary_with_next_frame_override("zero_next_frame", { 0, 0, 0, 0 });
		osd_printf_info(
			"INFO dsp3 a02 fixcoeff delta label=zero_next_frame dso=%d/%d dl50=%d dl53=%d dl56=%d dl59=%d dmacc=%lld\n",
			s32(state.so0) - s32(base.so0),
			s32(state.so1) - s32(base.so1),
			s32(state.l50) - s32(base.l50),
			s32(state.l53) - s32(base.l53),
			s32(state.l56) - s32(base.l56),
			s32(state.l59) - s32(base.l59),
			(long long)(state.macc - base.macc));
	}
	{
		const boundary_state state = capture_boundary_with_next_frame_override("hold_start_frame", start.serial_input_frame);
		osd_printf_info(
			"INFO dsp3 a02 fixcoeff delta label=hold_start_frame dso=%d/%d dl50=%d dl53=%d dl56=%d dl59=%d dmacc=%lld\n",
			s32(state.so0) - s32(base.so0),
			s32(state.so1) - s32(base.so1),
			s32(state.l50) - s32(base.l50),
			s32(state.l53) - s32(base.l53),
			s32(state.l56) - s32(base.l56),
			s32(state.l59) - s32(base.l59),
			(long long)(state.macc - base.macc));
	}
	{
		const boundary_state base_h3 = capture_boundary_h3_with_frame_override("base_h3_frame", expect.serial_input_frame, expect2.serial_input_frame, expect3.serial_input_frame);
		const boundary_state state = capture_boundary_h3_with_frame_override("zero_future_frames_h3", { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 });
		osd_printf_info(
			"INFO dsp3 a02 fixcoeff_h3 delta label=zero_future_frames_h3 dso=%d/%d dl50=%d dl53=%d dl56=%d dl59=%d dmacc=%lld\n",
			s32(state.so0) - s32(base_h3.so0),
			s32(state.so1) - s32(base_h3.so1),
			s32(state.l50) - s32(base_h3.l50),
			s32(state.l53) - s32(base_h3.l53),
			s32(state.l56) - s32(base_h3.l56),
			s32(state.l59) - s32(base_h3.l59),
			(long long)(state.macc - base_h3.macc));
	}
	{
		const boundary_state base_h3 = capture_boundary_h3_with_frame_override("base_h3_frame", expect.serial_input_frame, expect2.serial_input_frame, expect3.serial_input_frame);
		const boundary_state state = capture_boundary_h3_with_frame_override("hold_start_frames_h3", start.serial_input_frame, start.serial_input_frame, start.serial_input_frame);
		osd_printf_info(
			"INFO dsp3 a02 fixcoeff_h3 delta label=hold_start_frames_h3 dso=%d/%d dl50=%d dl53=%d dl56=%d dl59=%d dmacc=%lld\n",
			s32(state.so0) - s32(base_h3.so0),
			s32(state.so1) - s32(base_h3.so1),
			s32(state.l50) - s32(base_h3.l50),
			s32(state.l53) - s32(base_h3.l53),
			s32(state.l56) - s32(base_h3.l56),
			s32(state.l59) - s32(base_h3.l59),
			(long long)(state.macc - base_h3.macc));
	}
}

void tms57002test_state::probe_dsp3_a02_fix_su95298_b3_operand_source()
{
	static constexpr char kProgramPathPrimary[] = "tmp/kprop_dsp3_dump.sTfkHh/dump_dsp3.bin";
	static constexpr char kProgramPathFallback[] = "00-extracted/kprop_a02_sine_live_dump/dump_dsp3.bin";
	static constexpr char kLogPath[] = "tmp/kprop_a02_fix_sample_tape_text95297/dspsnap.log";
	static constexpr char kRamPath[] = "tmp/kprop_a02_fix_sample_tape_text95297/ram/su095298_pc00_pre.bin";
	static constexpr u64 kStartSu = 95298;

	const char *program_path = std::ifstream(kProgramPathPrimary).good() ? kProgramPathPrimary : kProgramPathFallback;
	const std::vector<u32> program = load_program_words(program_path);
	const std::vector<std::string> lines = load_text_lines(kLogPath);
	const tms57002_device::debug_snapshot start = load_snapshot_for_su_from_lines(lines, kStartSu);

	struct b3_state
	{
		u64 macc = 0;
		u64 maccr = 0;
		u64 maccw = 0;
		u32 creg = 0;
		u32 p06 = 0;
		u32 p09 = 0;
		u32 p0f = 0;
		u32 p10 = 0;
	};

	auto run_variant = [&](const char *label, std::initializer_list<u8> zero_dmem0, std::initializer_list<u8> zero_dmem1) -> b3_state
	{
		m_dsp->debug_load_program(program.data(), u32(program.size()), start.st0, start.st1);
		load_data_space_bytes(kRamPath, m_dsp->space(AS_DATA));
		m_dsp->debug_restore_snapshot(start);
		for (u8 addr : zero_dmem0)
			m_dsp->debug_write_dmem0(addr, 0);
		for (u8 addr : zero_dmem1)
			m_dsp->debug_write_dmem1(addr, 0);

		int seen = 0;
		for (int step = 0; step < 0x400; step++)
		{
			const u8 pc = u8(m_dsp->state_int(TMS57002_PC));
			const u8 id = u8(m_dsp->state_int(TMS57002_ID));
			m_dsp->debug_run_cycles(1);
			if (pc == 0xb3 && id == 0x1f)
			{
				seen++;
				if (seen == 2)
				{
					b3_state out;
					out.macc = m_dsp->debug_macc();
					out.maccr = m_dsp->debug_macc_read();
					out.maccw = m_dsp->debug_macc_write();
					out.creg = m_dsp->debug_creg();
					out.p06 = m_dsp->dmem0_value(0x06);
					out.p09 = m_dsp->dmem0_value(0x09);
					out.p0f = m_dsp->dmem0_value(0x0f);
					out.p10 = m_dsp->dmem0_value(0x10);
					osd_printf_info(
						"INFO dsp3 fixb3src label=%s macc=%013llx maccr=%013llx maccw=%013llx creg=%08x "
						"p06=%06x p09=%06x p0f=%06x p10=%06x\n",
						label,
						(unsigned long long)out.macc,
						(unsigned long long)out.maccr,
						(unsigned long long)out.maccw,
						out.creg,
						out.p06,
						out.p09,
						out.p0f,
						out.p10);
					return out;
				}
			}
		}

		throw emu_fatalerror("Failed to reach second DSP3 fix B3 target for %s", label);
	};

	const b3_state base = run_variant("base", {}, {});
	const b3_state low_group = run_variant("zero_dmem0_07080f1011", { 0x07, 0x08, 0x0f, 0x10, 0x11 }, {});
	const b3_state high_group = run_variant("zero_dmem0_bbbcbdbeff", { 0xbb, 0xbc, 0xbd, 0xbe, 0xbf }, {});
	const b3_state only_bb = run_variant("zero_dmem0_bb", { 0xbb }, {});
	const b3_state only_bc = run_variant("zero_dmem0_bc", { 0xbc }, {});
	const b3_state only_bd = run_variant("zero_dmem0_bd", { 0xbd }, {});
	const b3_state only_be = run_variant("zero_dmem0_be", { 0xbe }, {});
	const b3_state only_bf = run_variant("zero_dmem0_bf", { 0xbf }, {});
	const b3_state dmem1_1e = run_variant("zero_dmem1_1e", {}, { 0x1e });
	const b3_state dmem1_1f = run_variant("zero_dmem1_1f", {}, { 0x1f });

	osd_printf_info(
		"INFO dsp3 fixb3src delta low=%013llx high=%013llx bb=%013llx bc=%013llx bd=%013llx be=%013llx bf=%013llx d1_1e=%013llx d1_1f=%013llx\n",
		(unsigned long long)(low_group.macc - base.macc),
		(unsigned long long)(high_group.macc - base.macc),
		(unsigned long long)(only_bb.macc - base.macc),
		(unsigned long long)(only_bc.macc - base.macc),
		(unsigned long long)(only_bd.macc - base.macc),
		(unsigned long long)(only_be.macc - base.macc),
		(unsigned long long)(only_bf.macc - base.macc),
		(unsigned long long)(dmem1_1e.macc - base.macc),
		(unsigned long long)(dmem1_1f.macc - base.macc));
}

void tms57002test_state::probe_dsp3_a02_tape_cycle_budget()
{
	static constexpr char kProgramPath[] = "00-extracted/korgprop_dsp_offline_extract/dsp3_init.bin";
	static constexpr char kLogPath[] = "00-extracted/kprop_a02_sample_tape_20260408/error.log";
	static constexpr char kRamDir[] = "00-extracted/kprop_a02_sample_tape_20260408/ram";

	const std::vector<u32> program = load_program_words(kProgramPath);
	const std::vector<std::string> lines = load_text_lines(kLogPath);
	const tms57002_device::debug_snapshot start = load_snapshot_for_su_from_lines(lines, 480003);
	const tms57002_device::debug_snapshot expect = load_snapshot_for_su_from_lines(lines, 480004);
	const std::string start_ram_path = util::string_format("%s/su480003_pc00_pre.bin", kRamDir);

	auto score_variant = [&](int cycles) -> void
	{
		m_dsp->debug_load_program(program.data(), u32(program.size()), start.st0, start.st1);
		load_data_space_bytes(start_ram_path.c_str(), m_dsp->space(AS_DATA));
		m_dsp->debug_restore_snapshot(start);
		for (int i = 0; i < cycles; i++)
			m_dsp->debug_run_cycles(1);
		m_dsp->debug_begin_sample_frame(expect.serial_input_frame);
		const tms57002_device::debug_snapshot actual = m_dsp->debug_capture_snapshot();

		int score = 0;
		auto bump = [&](bool cond) { if (cond) score++; };
		bump(actual.aacc != expect.aacc);
		bump(actual.macc != expect.macc);
		bump(actual.macc_read != expect.macc_read);
		bump(actual.macc_write != expect.macc_write);
		bump(actual.xoa != expect.xoa);
		bump(actual.xwr != expect.xwr);
		bump(actual.so[0] != expect.so[0]);
		bump(actual.so[1] != expect.so[1]);
		bump(actual.dmem0[(actual.ba0 + 0x54) & 0xff] != expect.dmem0[(expect.ba0 + 0x54) & 0xff]);
		bump(actual.cmem[0x42] != expect.cmem[0x42]);

		osd_printf_info(
			"INFO dsp3 a02 tapecycles su=480003 cycles=%d score=%d aacc=%08x macc=%013llx xoa=%05x xwr=%06x so=%06x/%06x l54=%06x c42=%08x\n",
			cycles,
			score,
			actual.aacc,
			(unsigned long long)actual.macc,
			actual.xoa,
			actual.xwr,
			actual.so[0],
			actual.so[1],
			actual.dmem0[(actual.ba0 + 0x54) & 0xff],
			actual.cmem[0x42]);
	};

	for (int cycles = 504; cycles <= 520; cycles++)
		score_variant(cycles);
}

void tms57002test_state::test_dsp3_a02_su480006_input_sensitivity()
{
	static constexpr char kSnapshotPath[] = "00-extracted/kprop_a02_firstbad_seed/snapshot_su480006_pc00_pre_sine.log";
	static constexpr char kExecPath[] = "00-extracted/kprop_a02_firstbad_seed/execpost_su480006_full_sine.log";
	static constexpr char kRamPath[] = "00-extracted/kprop_a02_firstbad_seed/xmem_su480006_pc00_pre_sine.bin";

	test_dsp3_input_sensitivity_case("dsp3 a02 su480006", kSnapshotPath, kExecPath, kRamPath);
}

void tms57002test_state::test_dsp3_a02_su480001_pc68_mac_matches_input()
{
	static constexpr char kProgramPath[] = "00-extracted/korgprop_dsp_offline_extract/dsp3_init.bin";
	static constexpr char kSinePath[] = "tmp/kprop_a02_su480001_start_sine/error.log";
	static constexpr char kImpulsePath[] = "tmp/kprop_a02_su480001_start_impulse/error.log";
	static constexpr char kRamPath[] = "tmp/kprop_a02_su480001_start_sine/xmem.bin";

	struct pc68_state
	{
		u32 aacc = 0;
		s64 macc_pre = 0;
		s64 macc_post = 0;
		u32 logical_d02 = 0;
		u8 ba0 = 0;
	};

	auto run_to_pc68 = [&](const tms57002_device::debug_snapshot &snapshot) -> pc68_state
	{
		const std::vector<u32> program = load_program_words(kProgramPath);
		if (program.size() != 0x100)
			throw emu_fatalerror("Unexpected DSP3 program size %zu in %s", program.size(), kProgramPath);

		m_dsp->debug_load_program(program.data(), u32(program.size()), snapshot.st0, snapshot.st1);
		load_data_space_bytes(kRamPath, m_dsp->space(AS_DATA));
		m_dsp->debug_restore_snapshot(snapshot);

		for (int step = 0; step < 0x200; step++)
		{
			if (m_dsp->state_int(TMS57002_PC) == 0x68 && m_dsp->state_int(TMS57002_ID) == 0x00)
			{
				pc68_state out;
				out.aacc = u32(m_dsp->debug_aacc());
				out.macc_pre = s64(m_dsp->debug_macc());
				out.ba0 = m_dsp->debug_ba0();
				out.logical_d02 = m_dsp->dmem0_value((out.ba0 + 0x02) & 0xff);
				m_dsp->debug_run_cycles(1);
				out.macc_post = s64(m_dsp->debug_macc());
				return out;
			}
			m_dsp->debug_run_cycles(1);
		}

		throw emu_fatalerror("Failed to reach DSP3 SU480001 PC68 ID00 replay target");
	};

	const tms57002_device::debug_snapshot sine = load_snapshot_from_log(kSinePath);
	const tms57002_device::debug_snapshot impulse = load_snapshot_from_log(kImpulsePath);
	const pc68_state sine_state = run_to_pc68(sine);
	const pc68_state impulse_state = run_to_pc68(impulse);

	check_equal("dsp3 a02 su480001 pc68 pre aacc match", sine_state.aacc, impulse_state.aacc);
	check_equal("dsp3 a02 su480001 pc68 pre macc match", sine_state.macc_pre, impulse_state.macc_pre);
	check_equal("dsp3 a02 su480001 pc68 ba0 match", sine_state.ba0, impulse_state.ba0);
	check_equal("dsp3 a02 su480001 pc68 sine logical d02", sine_state.logical_d02, 0);
	check_equal("dsp3 a02 su480001 pc68 impulse logical d02", impulse_state.logical_d02, 0x001000);
	check_equal("dsp3 a02 su480001 pc68 sine mac delta", sine_state.macc_post - sine_state.macc_pre, 0);

	const s64 expected_delta = (s64(s32(sine_state.aacc)) * sx24(impulse_state.logical_d02)) >> 7;
	check_equal("dsp3 a02 su480001 pc68 impulse mac delta", impulse_state.macc_post - impulse_state.macc_pre, expected_delta);
}

void tms57002test_state::dump_dsp3_a02_su480006_replay_block()
{
	static constexpr char kProgramPath[] = "00-extracted/korgprop_dsp_offline_extract/dsp3_init.bin";
	static constexpr char kSnapshotPath[] = "00-extracted/kprop_a02_firstbad_seed/snapshot_su480006_pc00_pre_sine.log";

	const std::vector<u32> program = load_program_words(kProgramPath);
	const tms57002_device::debug_snapshot snapshot = load_snapshot_from_log(kSnapshotPath);

	m_dsp->debug_load_program(program.data(), u32(program.size()), snapshot.st0, snapshot.st1);
	m_dsp->debug_restore_snapshot(snapshot);

	for (int step = 0; step < 0x2a; step++)
	{
		const u32 curpc = m_dsp->state_int(TMS57002_PC);
		m_dsp->debug_run_cycles(1);
		if (curpc >= 0x1b && curpc <= 0x29)
		{
			osd_printf_info(
				"INFO dsp3 a02 su480006 replay pc%02x ca=%02x id=%02x aacc=%08x macc=%013llx maccr=%013llx maccw=%013llx creg=%08x xoa=%05x xba=%05x xmadr=%05x xmcyc=%02x xmfet=%02x d00=%06x ddc=%06x ddd=%06x dde=%06x ddf=%06x xrd=%06x so=%06x/%06x\n",
				curpc,
				m_dsp->state_int(TMS57002_CA),
				m_dsp->state_int(TMS57002_ID),
				u32(m_dsp->debug_aacc()),
				(unsigned long long)m_dsp->debug_macc(),
				(unsigned long long)m_dsp->debug_macc_read(),
				(unsigned long long)m_dsp->debug_macc_write(),
				m_dsp->debug_creg(),
				m_dsp->debug_xoa(),
				m_dsp->debug_xba(),
				m_dsp->debug_xm_adr(),
				m_dsp->debug_xm_cycles(),
				m_dsp->debug_xm_fetches(),
				m_dsp->dmem0_value(0x00),
				m_dsp->dmem0_value(0xdc),
				m_dsp->dmem0_value(0xdd),
				m_dsp->dmem0_value(0xde),
				m_dsp->dmem0_value(0xdf),
				m_dsp->debug_xrd(),
				m_dsp->debug_serial_output_register(0),
				m_dsp->debug_serial_output_register(1));
		}
	}
}

void tms57002test_state::test_dsp3_snapshot_replay_case_with_program(const char *label, const char *program_path, const char *snapshot_path, const char *exec_path, const char *ram_path)
{
	const std::vector<u32> program = load_program_words(program_path);
	const tms57002_device::debug_snapshot snapshot = load_snapshot_from_log(snapshot_path);
	const std::vector<exec_post_expectation> expected = load_exec_post_expectations(exec_path);

	if (program.size() != 0x100)
		throw emu_fatalerror("Unexpected DSP3 program size %zu in %s", program.size(), program_path);

	m_dsp->debug_load_program(program.data(), u32(program.size()), snapshot.st0, snapshot.st1);
	load_data_space_bytes(ram_path, m_dsp->space(AS_DATA));
	m_dsp->debug_restore_snapshot(snapshot);

	for (std::size_t step = 0; step < expected.size(); step++)
	{
		const auto &exp = expected[step];
		const u8 expected_next_pc = (step + 1 < expected.size()) ? expected[step + 1].executed_pc : u8(exp.executed_pc + 1);

		m_dsp->debug_run_cycles(1);

		char name[128];
		std::snprintf(name, sizeof(name), "%s pc%02x next pc", label, exp.executed_pc);
		check_equal(name, m_dsp->state_int(TMS57002_PC), expected_next_pc);
		std::snprintf(name, sizeof(name), "%s pc%02x ca", label, exp.executed_pc);
		check_equal(name, m_dsp->state_int(TMS57002_CA), exp.ca);
		std::snprintf(name, sizeof(name), "%s pc%02x id", label, exp.executed_pc);
		check_equal(name, m_dsp->state_int(TMS57002_ID), exp.id);
		std::snprintf(name, sizeof(name), "%s pc%02x st1", label, exp.executed_pc);
		check_equal(name, m_dsp->state_int(TMS57002_ST1), exp.st1);
		std::snprintf(name, sizeof(name), "%s pc%02x aacc", label, exp.executed_pc);
		check_equal(name, u32(m_dsp->debug_aacc()), exp.aacc);
		std::snprintf(name, sizeof(name), "%s pc%02x macc", label, exp.executed_pc);
		check_equal(name, s64(m_dsp->debug_macc()), s64(exp.macc));
		std::snprintf(name, sizeof(name), "%s pc%02x maccr", label, exp.executed_pc);
		check_equal(name, s64(m_dsp->debug_macc_read()), s64(exp.maccr));
		std::snprintf(name, sizeof(name), "%s pc%02x maccw", label, exp.executed_pc);
		check_equal(name, s64(m_dsp->debug_macc_write()), s64(exp.maccw));
		std::snprintf(name, sizeof(name), "%s pc%02x creg", label, exp.executed_pc);
		check_equal(name, m_dsp->debug_creg(), exp.creg);
		if (exp.has_extended)
		{
			std::snprintf(name, sizeof(name), "%s pc%02x d00", label, exp.executed_pc);
			check_equal(name, m_dsp->dmem0_value(0x00), exp.d00);
			std::snprintf(name, sizeof(name), "%s pc%02x ddc", label, exp.executed_pc);
			check_equal(name, m_dsp->dmem0_value(0xdc), exp.ddc);
			std::snprintf(name, sizeof(name), "%s pc%02x ddd", label, exp.executed_pc);
			check_equal(name, m_dsp->dmem0_value(0xdd), exp.ddd);
			std::snprintf(name, sizeof(name), "%s pc%02x dde", label, exp.executed_pc);
			check_equal(name, m_dsp->dmem0_value(0xde), exp.dde);
			std::snprintf(name, sizeof(name), "%s pc%02x ddf", label, exp.executed_pc);
			check_equal(name, m_dsp->dmem0_value(0xdf), exp.ddf);
			std::snprintf(name, sizeof(name), "%s pc%02x xrd", label, exp.executed_pc);
			check_equal(name, m_dsp->debug_xrd(), exp.xrd);
		}
		std::snprintf(name, sizeof(name), "%s pc%02x d56", label, exp.executed_pc);
		check_equal(name, m_dsp->dmem0_value(0x56), exp.d56);
		std::snprintf(name, sizeof(name), "%s pc%02x d5d", label, exp.executed_pc);
		check_equal(name, m_dsp->dmem0_value(0x5d), exp.d5d);
		std::snprintf(name, sizeof(name), "%s pc%02x d5f", label, exp.executed_pc);
		check_equal(name, m_dsp->dmem0_value(0x5f), exp.d5f);
		std::snprintf(name, sizeof(name), "%s pc%02x so0", label, exp.executed_pc);
		check_equal(name, m_dsp->debug_serial_output_register(0), exp.so0);
		std::snprintf(name, sizeof(name), "%s pc%02x so1", label, exp.executed_pc);
		check_equal(name, m_dsp->debug_serial_output_register(1), exp.so1);
	}
}

void tms57002test_state::test_dsp3_snapshot_replay_case(const char *label, const char *snapshot_path, const char *exec_path, const char *ram_path)
{
	static constexpr char kProgramPath[] = "00-extracted/korgprop_dsp_offline_extract/dsp3_init.bin";
	test_dsp3_snapshot_replay_case_with_program(label, kProgramPath, snapshot_path, exec_path, ram_path);
}

void tms57002test_state::run_filter_replay_from_env()
{
	const char *pmem_path = required_env("TMS57TEST_REPLAY_PMEM");
	const char *cmem_path = required_env("TMS57TEST_REPLAY_CMEM");
	const char *serial_path = required_env("TMS57TEST_REPLAY_SERIAL");
	const char *out_path = required_env("TMS57TEST_REPLAY_OUT");
	const char *dmem0_path = std::getenv("TMS57TEST_REPLAY_DMEM0");
	const char *dmem1_path = std::getenv("TMS57TEST_REPLAY_DMEM1");
	const char *cload_path = std::getenv("TMS57TEST_REPLAY_CLOAD");

	const std::vector<u32> program = load_hex_words(pmem_path, 0x00ffffffU);
	const std::vector<u32> cmem = load_hex_words(cmem_path);
	const std::vector<u32> dmem0 = (dmem0_path && dmem0_path[0]) ? load_hex_words(dmem0_path, 0x00ffffffU) : std::vector<u32>{};
	const std::vector<u32> dmem1 = (dmem1_path && dmem1_path[0]) ? load_hex_words(dmem1_path, 0x00ffffffU) : std::vector<u32>{};
	const std::vector<serial_replay_row> rows = load_serial_replay_rows(serial_path);
	const std::vector<cload_replay_row> cload_rows = load_cload_replay_rows(cload_path);

	if (program.empty())
		throw emu_fatalerror("No PMEM words in %s", pmem_path);
	if (rows.empty())
		throw emu_fatalerror("No serial rows in %s", serial_path);

	m_dsp->debug_load_program(program.data(), u32(program.size()), env_hex("TMS57TEST_REPLAY_ST0"), env_hex("TMS57TEST_REPLAY_ST1"));
	m_dsp->set_sync_polarity(env_bool("TMS57TEST_REPLAY_SYNC_RISING", true));
	m_dsp->set_serial_frame_model(env_bool("TMS57TEST_REPLAY_FRAME_MODEL", true));
	m_dsp->set_serial_frame_clocks(env_int("TMS57TEST_REPLAY_FRAME_CLOCKS", 64));
	m_dsp->set_serial_frame_flip_input(env_bool("TMS57TEST_REPLAY_FLIP_INPUT", false));
	m_dsp->set_serial_frame_flip_output(env_bool("TMS57TEST_REPLAY_FLIP_OUTPUT", false));
	m_dsp->set_serial_output_write_handoff(
			env_bool("TMS57TEST_REPLAY_WRITE_HANDOFF", false));
	m_dsp->set_stream_output_raw(env_bool("TMS57TEST_REPLAY_STREAM_RAW", false));
	m_dsp->mute_w(1);

	for (u32 i = 0; i < cmem.size() && i < 0x100; i++)
		m_dsp->debug_write_cmem(u8(i), cmem[i]);
	for (u32 i = 0; i < dmem0.size() && i < 0x100; i++)
		m_dsp->debug_write_dmem0(u8(i), dmem0[i]);
	for (u32 i = 0; i < dmem1.size() && i < 0x20; i++)
		m_dsp->debug_write_dmem1(u8(i), dmem1[i]);

	const int max_cycles = env_int("TMS57TEST_REPLAY_MAX_CYCLES", 4096);

	// Optional mid-run reload: at TMS57TEST_REPLAY_RELOAD_FRAME, swap in a new
	// program image (PC/control reset, like a host PLOAD) and overwrite all
	// provided CMEM cells, leaving DMEM/XMEM/accumulators untouched — mirrors
	// the Prophecy firmware's mute-protected hardware-image reload.
	const char *reload_pmem_path = std::getenv("TMS57TEST_REPLAY_RELOAD_PMEM");
	const char *reload_cmem_path = std::getenv("TMS57TEST_REPLAY_RELOAD_CMEM");
	const int reload_frame = env_int("TMS57TEST_REPLAY_RELOAD_FRAME", -1);
	const std::vector<u32> reload_pmem = (reload_pmem_path && reload_pmem_path[0]) ? load_hex_words(reload_pmem_path, 0x00ffffffU) : std::vector<u32>{};
	const std::vector<u32> reload_cmem = (reload_cmem_path && reload_cmem_path[0]) ? load_hex_words(reload_cmem_path) : std::vector<u32>{};
	if (reload_frame >= 0 && reload_pmem.empty())
		throw emu_fatalerror("TMS57TEST_REPLAY_RELOAD_FRAME set but no reload PMEM");

	// Optional reload schedule: TMS57TEST_REPLAY_RELOAD_SCHEDULE names a text
	// file of lines "FRAME PMEM_PATH CMEM_PATH ST0 ST1" (CMEM_PATH may be "-"
	// for program-only loads). Each entry applies before the named frame, in
	// file order — replays a whole boot/setup PLOAD lifecycle standalone.
	struct reload_entry { u32 frame; std::vector<u32> pmem; std::vector<u32> cmem; u32 st0; u32 st1; };
	std::vector<reload_entry> reload_schedule;
	if (const char *sched_path = std::getenv("TMS57TEST_REPLAY_RELOAD_SCHEDULE"); sched_path && sched_path[0])
	{
		for (const std::string &line : load_text_lines(sched_path))
		{
			if (line.empty() || line[0] == '#')
				continue;
			std::istringstream ss(line);
			std::string pmem_file, cmem_file, st0_s, st1_s;
			u32 frame = 0;
			if (!(ss >> frame >> pmem_file >> cmem_file >> st0_s >> st1_s))
				throw emu_fatalerror("Bad reload schedule line: %s", line);
			reload_entry e;
			e.frame = frame;
			e.pmem = load_hex_words(pmem_file.c_str(), 0x00ffffffU);
			if (cmem_file != "-")
				e.cmem = load_hex_words(cmem_file.c_str());
			e.st0 = u32(std::strtoul(st0_s.c_str(), nullptr, 16));
			e.st1 = u32(std::strtoul(st1_s.c_str(), nullptr, 16));
			if (e.pmem.empty())
				throw emu_fatalerror("Empty reload PMEM %s", pmem_file);
			reload_schedule.push_back(std::move(e));
		}
	}

	std::ofstream out(out_path);
	if (!out)
		throw emu_fatalerror("Unable to open %s for writing", out_path);

	out << "frame\tsi0\tsi1\tsi2\tsi3\tso0\tso1\tso2\tso3\n";
	u32 cload_applied = 0;
	for (const serial_replay_row &row : rows)
	{
		if (reload_frame >= 0 && row.frame == u32(reload_frame))
		{
			m_dsp->debug_load_program(reload_pmem.data(), u32(reload_pmem.size()),
					env_hex("TMS57TEST_REPLAY_RELOAD_ST0", env_hex("TMS57TEST_REPLAY_ST0")),
					env_hex("TMS57TEST_REPLAY_RELOAD_ST1", env_hex("TMS57TEST_REPLAY_ST1")));
			for (u32 i = 0; i < reload_cmem.size() && i < 0x100; i++)
				m_dsp->debug_write_cmem(u8(i), reload_cmem[i]);
			osd_printf_info("TMS57REPLAY reload applied at frame %d\n", reload_frame);
		}
		for (const reload_entry &e : reload_schedule)
		{
			if (e.frame == row.frame)
			{
				m_dsp->debug_load_program(e.pmem.data(), u32(e.pmem.size()), e.st0, e.st1);
				for (u32 i = 0; i < e.cmem.size() && i < 0x100; i++)
					m_dsp->debug_write_cmem(u8(i), e.cmem[i]);
				osd_printf_info("TMS57REPLAY schedule reload at frame %u (st0=%06x st1=%06x)\n", e.frame, e.st0, e.st1);
			}
		}
		for (const cload_replay_row &cload_row : cload_rows)
		{
			if (cload_row.frame == row.frame)
			{
				apply_replay_cload(*m_dsp, cload_row.sa, cload_row.value);
				cload_applied++;
			}
		}
		const std::array<u32, 4> so = m_dsp->debug_run_sample_frame(row.si, max_cycles);
		out << row.frame;
		for (u32 i = 0; i < 4; i++)
		{
			out << '\t';
			write_hex24(out, row.si[i]);
		}
		for (u32 i = 0; i < 4; i++)
		{
			out << '\t';
			write_hex24(out, so[i]);
		}
		out << '\n';
	}

	const char *dump_path = std::getenv("TMS57TEST_REPLAY_STATE_DUMP");
	if (dump_path && dump_path[0])
	{
		std::ofstream dump(dump_path);
		if (!dump)
			throw emu_fatalerror("Unable to open %s for writing", dump_path);
		for (u32 i = 0; i < 0x100; i++)
			dump << util::string_format("dmem0 %02x 0x%06x\n", i, m_dsp->dmem0_value(u8(i)));
		for (u32 i = 0; i < 0x20; i++)
			dump << util::string_format("dmem1 %02x 0x%06x\n", i, m_dsp->dmem1_value(u8(i)));
	}

	osd_printf_info("TMS57REPLAY rows=%zu cload=%u out=%s\n", rows.size(), cload_applied, out_path);
}

void tms57002test_state::probe_dsp3_a12_drone_regrowth()
{
	// Free-running replay of the A12 voice program from a mid-drone MAME
	// snapshot, with optional CMEM overrides/zeroing — the standalone repro
	// for the post-all-off regrowth. Prints per-sample SO + key cells.
	const char *program_path = required_env("TMS57TEST_DRONE_PROGRAM_PATH");
	const char *snap_log = required_env("TMS57TEST_DRONE_SNAP_LOG");
	const u64 snap_su = parse_dec_u64(required_env("TMS57TEST_DRONE_SNAP_SU"));
	const char *ram_path = std::getenv("TMS57TEST_DRONE_RAM_PATH");
	const int samples = env_int("TMS57TEST_DRONE_SAMPLES", 300);
	const bool fresh_state = env_bool("TMS57TEST_DRONE_FRESH_STATE");
	const int cycles_per_sample = env_int("TMS57TEST_DRONE_CYCLES_PER_SAMPLE", 512);
	const int serial_frame_clocks = env_int("TMS57TEST_DRONE_SERIAL_FRAME_CLOCKS", cycles_per_sample);
	const bool frame_model = env_bool("TMS57TEST_DRONE_FRAME_MODEL");
	const bool sample_frame_api = env_bool("TMS57TEST_DRONE_SAMPLE_FRAME_API");
	const bool host_sequence = env_bool("TMS57TEST_DRONE_HOST_SEQUENCE");
	m_dsp->set_ca_inc_delayed(env_bool("TMS57TEST_CA_INC_DELAYED"));
	m_dsp->set_abs_saturate(env_bool("TMS57TEST_ABS_SATURATE"));
	m_dsp->set_smhc_post_slot(env_bool("TMS57TEST_SMHC_POST_SLOT"));

	const std::vector<u32> program = load_program_words(program_path);
	const std::vector<std::string> lines = load_text_lines(snap_log);
	tms57002_device::debug_snapshot start = load_snapshot_for_su_from_lines(lines, snap_su);
	start.st0 = env_hex("TMS57TEST_DRONE_ST0", start.st0);
	start.st1 = env_hex("TMS57TEST_DRONE_ST1", start.st1);
	if (env_bool("TMS57TEST_DRONE_RESET_EXEC"))
	{
		start.pc = 0;
		start.hpc = 0;
		start.ca = 0;
		start.id = 0;
		start.ba0 = 0;
		start.ba1 = 0;
		start.rptc = 0;
		start.rptc_next = 0;
		start.sa = 0;
		start.hidx = 0;
		start.sti = 0;
		start.aacc = 0;
		start.macc = 0;
		start.macc_read = 0;
		start.macc_write = 0;
		start.creg = 0;
		start.xoa = 0;
		start.xba = 0;
		start.xwr = 0;
		start.xrd = 0;
		start.txrd = 0;
		start.xm_adr = 0;
		start.xm_cycles = 0;
		start.xm_fetches = 0;
	}
	if (env_bool("TMS57TEST_DRONE_SERIAL_ZERO"))
	{
		start.si.fill(0);
		start.serial_input_latch.fill(0);
		start.serial_input_active.fill(0);
		start.serial_input_frame.fill(0);
		start.serial_input_prev_frame.fill(0);
		start.serial_input_pending.fill(0);
		start.serial_input_valid = 0;
		start.serial_input_active_valid = 0;
		start.serial_input_prev_valid = 0;
		start.serial_input_pending_valid = 0;
	}

	m_dsp->debug_load_program(program.data(), u32(program.size()), start.st0, start.st1);
	if (frame_model)
	{
		m_dsp->set_serial_frame_model(true);
		m_dsp->set_serial_frame_clocks(serial_frame_clocks);
	}
	if (!fresh_state)
	{
		if (ram_path && ram_path[0])
			load_data_space_bytes(ram_path, m_dsp->space(AS_DATA));
		m_dsp->debug_restore_snapshot(start);
	}

	if (const char *set = std::getenv("TMS57TEST_DRONE_CMEM_SET"))
	{
		// Malformed entries are skipped LOUDLY, never silently dropped along
		// with the rest of the list (a bisect with a typo must not produce a
		// quiet wrong verdict).
		int applied = 0, skipped = 0;
		const char *p = set;
		while (*p)
		{
			char *eq = nullptr;
			const unsigned long addr = std::strtoul(p, &eq, 16);
			char *next = nullptr;
			unsigned long val = 0;
			bool ok = eq && eq != p && *eq == '=';
			if (ok)
			{
				val = std::strtoul(eq + 1, &next, 16);
				ok = next && next != eq + 1 && addr <= 0xff;
			}
			if (ok)
			{
				m_dsp->debug_write_cmem(u8(addr), u32(val));
				applied++;
				p = next;
			}
			else
			{
				skipped++;
				osd_printf_error("TMS57TEST_DRONE_CMEM_SET: malformed entry at '%.16s' (skipped)\n", p);
				while (*p && *p != ',' && *p != ' ')
					p++;
			}
			while (*p == ',' || *p == ' ')
				p++;
		}
		osd_printf_info("TMS57TEST_DRONE_CMEM_SET: applied=%d skipped=%d\n", applied, skipped);
		if (skipped)
			throw emu_fatalerror("TMS57TEST_DRONE_CMEM_SET had %d malformed entries; refusing to run a partial force", skipped);
	}
	if (const char *zero = std::getenv("TMS57TEST_DRONE_CMEM_ZERO"))
	{
		const char *p = zero;
		while (*p)
		{
			char *next = nullptr;
			const unsigned long addr = std::strtoul(p, &next, 16);
			if (next == p)
				break;
			if (addr <= 0xff)
				m_dsp->debug_write_cmem(u8(addr), 0);
			p = next;
			while (*p == ',' || *p == ' ')
				p++;
		}
	}
	if (env_bool("TMS57TEST_DRONE_ZERO_DMEM"))
	{
		for (u32 a = 0; a < 256; a++)
			m_dsp->debug_write_dmem0(u8(a), 0);
		for (u32 a = 0; a < 32; a++)
			m_dsp->debug_write_dmem1(u8(a), 0);
	}
	if (env_bool("TMS57TEST_DRONE_ZERO_XRAM"))
	{
		address_space &space = m_dsp->space(AS_DATA);
		for (u32 addr = 0; addr < 0x40000; addr++)
			space.write_byte(addr, 0);
	}
	if (host_sequence)
	{
		const std::array<u32, 256> cmem_image = m_dsp->debug_capture_snapshot().cmem;

		m_dsp->pload_w(1);
		m_dsp->cload_w(1);
		m_dsp->debug_reset_live_state(false);
		host_download_cmem_image(*m_dsp, cmem_image);
		m_dsp->debug_reset_live_state(false);
		host_download_program_image(*m_dsp, program, start.st0, start.st1);
		m_dsp->debug_reset_live_state(false);
		m_dsp->pload_w(1);
		m_dsp->cload_w(1);
		if (frame_model)
		{
			m_dsp->set_serial_frame_model(true);
			m_dsp->set_serial_frame_clocks(serial_frame_clocks);
		}
	}

	osd_printf_info(
		"DRONEREGROW setup ST0=%06x ST1=%06x PC=%02x CA=%02x ID=%02x BA0=%02x BA1=%02x "
		"AACC=%08x MACC=%013llx SI=%06x/%06x/%06x/%06x RESET_EXEC=%u SERIAL_ZERO=%u ZERO_DMEM=%u ZERO_XRAM=%u "
		"FRESH_STATE=%u CYCLES_PER_SAMPLE=%d SERIAL_FRAME_CLOCKS=%d FRAME_MODEL=%u SAMPLE_FRAME_API=%u HOST_SEQUENCE=%u\n",
		start.st0, start.st1, start.pc, start.ca, start.id, start.ba0, start.ba1,
		start.aacc, (unsigned long long)start.macc,
		start.serial_input_frame[0], start.serial_input_frame[1],
		start.serial_input_frame[2], start.serial_input_frame[3],
		u32(env_bool("TMS57TEST_DRONE_RESET_EXEC")),
		u32(env_bool("TMS57TEST_DRONE_SERIAL_ZERO")),
		u32(env_bool("TMS57TEST_DRONE_ZERO_DMEM")),
		u32(env_bool("TMS57TEST_DRONE_ZERO_XRAM")),
		u32(fresh_state),
		cycles_per_sample,
		serial_frame_clocks,
		u32(frame_model),
		u32(sample_frame_api),
		u32(host_sequence));
	// Optional extra per-sample cells: "d30,c31,c32,..." appended to each row as name=value.
	struct extra_cell { char kind; u8 addr; };
	std::vector<extra_cell> extra_cells;
	if (const char *cells = std::getenv("TMS57TEST_DRONE_PRINT_CELLS"))
	{
		std::string s = cells;
		for (size_t p = 0; p < s.size(); )
		{
			size_t c = s.find(',', p);
			if (c == std::string::npos) c = s.size();
			std::string tok = s.substr(p, c - p);
			if (tok.size() >= 2 && (tok[0] == 'd' || tok[0] == 'c'))
				extra_cells.push_back({ tok[0], u8(std::strtol(tok.c_str() + 1, nullptr, 16)) });
			p = c + 1;
		}
	}
	osd_printf_info("DRONEREGROW header sample,so0,so1,so2,so3,c21,c00,cD8,cDD,d02,d03,d05,d06,ba0\n");
	for (int n = 0; n < samples; n++)
	{
		std::array<u32, 4> observed_so;
		if (sample_frame_api)
			observed_so = m_dsp->debug_run_sample_frame(start.serial_input_frame, cycles_per_sample);
		else if (n < env_int("TMS57TEST_DRONE_STEP_SAMPLES", 0))
		{
			u8 last_pc = 0xff;
			u64 last_macc = ~u64(0);
			u32 last_aacc = ~u32(0);
			for (int cyc = 0; cyc < cycles_per_sample; cyc++)
			{
				m_dsp->debug_run_cycles(1);
				const tms57002_device::debug_snapshot ss = m_dsp->debug_capture_snapshot();
				if (ss.pc != last_pc || ss.macc != last_macc || ss.aacc != last_aacc)
					osd_printf_info("DRONESTEP %d,%d,pc=%02x,ca=%02x,ba0=%02x,aacc=%08x,macc=%013llx,creg=%08x\n",
						n, cyc, ss.pc, ss.ca, ss.ba0, ss.aacc, (unsigned long long)ss.macc, ss.creg);
				last_pc = ss.pc;
				last_macc = ss.macc;
				last_aacc = ss.aacc;
			}
			m_dsp->debug_begin_sample_frame(start.serial_input_frame);
		}
		else
		{
			m_dsp->debug_run_cycles(cycles_per_sample);
			m_dsp->debug_begin_sample_frame(start.serial_input_frame);
		}
		const tms57002_device::debug_snapshot s = m_dsp->debug_capture_snapshot();
		if (!sample_frame_api)
			observed_so = s.so;
		const u32 d02 = s.dmem0[(s.ba0 + 0x02) & 0xff];
		const u32 d03 = s.dmem0[(s.ba0 + 0x03) & 0xff];
		const u32 d05 = s.dmem0[(s.ba0 + 0x05) & 0xff];
		const u32 d06 = s.dmem0[(s.ba0 + 0x06) & 0xff];
		std::string extra;
		for (const extra_cell &ec : extra_cells)
		{
			const u32 v = (ec.kind == 'c') ? s.cmem[ec.addr] : s.dmem0[(s.ba0 + ec.addr) & 0xff];
			extra += util::string_format(",%c%02X=%08x", ec.kind, ec.addr, v);
		}
		osd_printf_info(
			"DRONEREGROW %d,%06x,%06x,%06x,%06x,%08x,%08x,%08x,%08x,%06x,%06x,%06x,%06x,%02x%s\n",
			n, observed_so[0], observed_so[1], observed_so[2], observed_so[3],
			s.cmem[0x21], s.cmem[0x00], s.cmem[0xd8], s.cmem[0xdd],
			d02, d03, d05, d06, s.ba0, extra);
	}
}

void tms57002test_state::run_all_focused_tests()
{
	struct entry { const char *name; void (tms57002test_state::*fn)(); };
	static const entry tests[] = {
		{ "aovm_neg_abs_int32_min", &tms57002test_state::test_aovm_neg_abs_int32_min },
		{ "cmem_update_multiword_sequence", &tms57002test_state::test_cmem_update_multiword_sequence },
		{ "cmem_update_new_address_breaks_active_run", &tms57002test_state::test_cmem_update_new_address_breaks_active_run },
		{ "cmem_update_waits_for_cload_high", &tms57002test_state::test_cmem_update_waits_for_cload_high },
		{ "domh_sfmo2_trace_cases", &tms57002test_state::test_domh_sfmo2_trace_cases },
		{ "dsp3_lmhd_srbd_uses_store_updated_dmem_for_macc", &tms57002test_state::test_dsp3_lmhd_srbd_uses_store_updated_dmem_for_macc },
		{ "dsp3_mpy_dis_uses_incoming_dmem", &tms57002test_state::test_dsp3_mpy_dis_uses_incoming_dmem },
		{ "dsp3_rde_sacc_uses_store_updated_cmem_for_xoa", &tms57002test_state::test_dsp3_rde_sacc_uses_store_updated_cmem_for_xoa },
		{ "dsp3_rde_word0_sel1_dram_latency", &tms57002test_state::test_dsp3_rde_word0_sel1_dram_latency },
		{ "dsp3_wre_smhd_orders_store_before_xwr", &tms57002test_state::test_dsp3_wre_smhd_orders_store_before_xwr },
		{ "example53_bank_switched_serial_access", &tms57002test_state::test_example53_bank_switched_serial_access },
		{ "example53_dos_deadline", &tms57002test_state::test_example53_dos_deadline },
		{ "example53_one_frame_output_latency", &tms57002test_state::test_example53_one_frame_output_latency },
		{ "instruction_level_serial_passthrough", &tms57002test_state::test_instruction_level_serial_passthrough },
		{ "lacc_lira_same_word_uses_pre_lacc_aacc", &tms57002test_state::test_lacc_lira_same_word_uses_pre_lacc_aacc },
		{ "mac_a_d_and_smld_port_semantics", &tms57002test_state::test_mac_a_d_and_smld_port_semantics },
		{ "multiplier_a_port_drops_low_byte", &tms57002test_state::test_multiplier_a_port_drops_low_byte },
		{ "sequential_pc_wrap_halts_until_sync", &tms57002test_state::test_sequential_pc_wrap_halts_until_sync },
		{ "macc_output_clipping", &tms57002test_state::test_macc_output_clipping },
		{ "macc_output_unrounded_exceptions", &tms57002test_state::test_macc_output_unrounded_exceptions },
		{ "pc33_mpy_mac_direct_coeff_chain", &tms57002test_state::test_pc33_mpy_mac_direct_coeff_chain },
		{ "pc33_producer_smhc_phase", &tms57002test_state::test_pc33_producer_smhc_phase },
		{ "xmem_micro_oracles", &tms57002test_state::test_xmem_micro_oracles },
	};

	int passed = 0, failed = 0, errored = 0;
	for (const entry &t : tests)
	{
		const int before = m_failures;
		osd_printf_info("=== TMS57TEST_ALL: %s ===\n", t.name);
		try { (this->*t.fn)(); }
		catch (const std::exception &e)
		{
			errored++;
			osd_printf_info("ERRORTEST %s %s\n", t.name, e.what());
			continue;
		}
		catch (...) { errored++; osd_printf_info("ERRORTEST %s (unknown exception)\n", t.name); continue; }
		const int delta = m_failures - before;
		if (delta == 0) { passed++; osd_printf_info("PASSTEST %s\n", t.name); }
		else { failed++; osd_printf_info("FAILTEST %s %d check(s)\n", t.name, delta); }
	}
	osd_printf_info("TMS57TEST_ALL_SUMMARY tests=%d passed=%d failed=%d errored=%d blocked=%d failed_checks=%d\n",
			int(std::size(tests)), passed, failed, errored, 0, m_failures);
}

void tms57002test_state::run_legacy_research_tests()
{
	struct entry { const char *name; void (tms57002test_state::*fn)(); };
	static const entry tests[] = {
		{ "dsp3_a02_sine_su480003_snapshot_replay", &tms57002test_state::test_dsp3_a02_sine_su480003_snapshot_replay },
		{ "dsp3_a02_su480001_pc68_mac_matches_input", &tms57002test_state::test_dsp3_a02_su480001_pc68_mac_matches_input },
		{ "dsp3_a02_su480002_input_sensitivity", &tms57002test_state::test_dsp3_a02_su480002_input_sensitivity },
		{ "dsp3_a02_su480003_input_sensitivity", &tms57002test_state::test_dsp3_a02_su480003_input_sensitivity },
		{ "dsp3_a02_su480004_input_sensitivity", &tms57002test_state::test_dsp3_a02_su480004_input_sensitivity },
		{ "dsp3_a02_su480004_snapshot_replay", &tms57002test_state::test_dsp3_a02_su480004_snapshot_replay },
		{ "dsp3_a02_su480005_input_sensitivity", &tms57002test_state::test_dsp3_a02_su480005_input_sensitivity },
		{ "dsp3_a02_su480005_snapshot_replay", &tms57002test_state::test_dsp3_a02_su480005_snapshot_replay },
		{ "dsp3_a02_su480006_input_sensitivity", &tms57002test_state::test_dsp3_a02_su480006_input_sensitivity },
		{ "dsp3_a02_su480006_snapshot_replay", &tms57002test_state::test_dsp3_a02_su480006_snapshot_replay },
		{ "dsp3_a02_su480082_snapshot_replay", &tms57002test_state::test_dsp3_a02_su480082_snapshot_replay },
	};

	int passed = 0, failed = 0, errored = 0, blocked = 1;
	osd_printf_info(
			"BLOCKEDTEST dsp3_a02_f8_ff_tail_chain incomplete synthetic fixture retained for research history; "
			"portable hardware coverage is tracked separately\n");
	for (const entry &t : tests)
	{
		const int before = m_failures;
		osd_printf_info("=== TMS57TEST_LEGACY: %s ===\n", t.name);
		try { (this->*t.fn)(); }
		catch (const std::exception &e)
		{
			if (std::strstr(e.what(), "Unable to open"))
			{
				blocked++;
				osd_printf_info("BLOCKEDTEST %s %s\n", t.name, e.what());
			}
			else
			{
				errored++;
				osd_printf_info("ERRORTEST %s %s\n", t.name, e.what());
			}
			continue;
		}
		catch (...) { errored++; osd_printf_info("ERRORTEST %s (unknown exception)\n", t.name); continue; }
		const int delta = m_failures - before;
		if (delta == 0) { passed++; osd_printf_info("PASSTEST %s\n", t.name); }
		else { failed++; osd_printf_info("FAILTEST %s %d check(s)\n", t.name, delta); }
	}
	osd_printf_info("TMS57TEST_LEGACY_SUMMARY tests=%d passed=%d failed=%d errored=%d blocked=%d failed_checks=%d release_denominator=0\n",
			int(std::size(tests) + 1), passed, failed, errored, blocked, m_failures);
}

void tms57002test_state::run_tests_now()
{
	if (const char *only = std::getenv("TMS57TEST_ONLY"))
	{
		osd_printf_info("Running focused TMS57002 test: %s\n", only);
		if (!std::strcmp(only, "all"))
		{
			run_all_focused_tests();
			machine().schedule_exit();
			if (m_failures != 0)
				throw emu_fatalerror(1, "%d TMS57002 tests failed", m_failures);
			return;
		}
		if (!std::strcmp(only, "legacy"))
		{
			run_legacy_research_tests();
			machine().schedule_exit();
			if (m_failures != 0)
				throw emu_fatalerror(1, "%d legacy TMS57002 tests failed", m_failures);
			return;
		}
		if (!std::strcmp(only, "filter-replay"))
		{
			run_filter_replay_from_env();
			machine().schedule_exit();
			return;
		}
		else if (!std::strcmp(only, "fixcoeff"))
			probe_dsp3_a02_fix_su95299_coeff_sensitivity();
		else if (!std::strcmp(only, "fixb3src"))
			probe_dsp3_a02_fix_su95298_b3_operand_source();
		else if (!std::strcmp(only, "tapeboundary"))
			probe_dsp3_a02_tape_boundary_replay();
		else if (!std::strcmp(only, "onset"))
			probe_dsp3_a02_onset_boundary_replay();
		else if (!std::strcmp(only, "drone_regrowth"))
			probe_dsp3_a12_drone_regrowth();
		else if (!std::strcmp(only, "serialside"))
			probe_dsp3_serial_side_boundary_sensitivity();
		else if (!std::strcmp(only, "lacc_lira"))
			test_lacc_lira_same_word_uses_pre_lacc_aacc();
		else if (!std::strcmp(only, "cmem_update"))
			test_cmem_update_multiword_sequence();
		else if (!std::strcmp(only, "cmem_update_cload_boundary"))
			test_cmem_update_waits_for_cload_high();
		else if (!std::strcmp(only, "cmem_update_address_run"))
			test_cmem_update_new_address_breaks_active_run();
		else if (!std::strcmp(only, "pc33_chain"))
			test_pc33_mpy_mac_direct_coeff_chain();
		else if (!std::strcmp(only, "pc33_producer"))
			test_pc33_producer_smhc_phase();
		else if (!std::strcmp(only, "store_mpy_ports"))
			test_mac_a_d_and_smld_port_semantics();
		else if (!std::strcmp(only, "mac_a_d_port"))
			test_mac_a_d_full_width_port();
		else if (!std::strcmp(only, "mpy_a_port"))
			test_multiplier_a_port_drops_low_byte();
		else if (!std::strcmp(only, "pc_wrap_halt"))
			test_sequential_pc_wrap_halts_until_sync();
		else if (!std::strcmp(only, "smld_raw_port"))
			test_smld_raw_low_port();
		else if (!std::strcmp(only, "aovm_neg_abs"))
			test_aovm_neg_abs_int32_min();
		else if (!std::strcmp(only, "mpy_dis_incoming"))
			test_dsp3_mpy_dis_uses_incoming_dmem();
		else if (!std::strcmp(only, "domh_movm_rounding"))
			test_domh_movm_rounding_oracle_boundaries();
		else if (!std::strcmp(only, "dos_deadline"))
			test_example53_dos_deadline();
		else if (!std::strcmp(only, "rde_latency"))
			test_dsp3_rde_word0_sel1_dram_latency();
		else if (!std::strcmp(only, "wre_smhd"))
			test_dsp3_wre_smhd_orders_store_before_xwr();
		else if (!std::strcmp(only, "xmem_micro"))
			test_xmem_micro_oracles();
		else
			throw emu_fatalerror(1, "Unknown TMS57TEST_ONLY=%s", only);

		if (m_failures != 0)
			throw emu_fatalerror(1, "%d TMS57002 tests failed", m_failures);
		throw emu_fatalerror(0, "Focused TMS57002 test completed");
	}

	osd_printf_info("Running TMS57002 helper semantics tests\n");
	test_macc_output_clipping();
	test_macc_output_unrounded_exceptions();
	test_domh_sfmo2_trace_cases();
	test_instruction_level_serial_passthrough();
	test_example53_bank_switched_serial_access();
	test_example53_one_frame_output_latency();
	test_example53_dos_deadline();
	test_dsp3_mpy_dis_uses_incoming_dmem();
	test_dsp3_wre_smhd_orders_store_before_xwr();
	test_dsp3_rde_sacc_uses_store_updated_cmem_for_xoa();
	test_dsp3_lmhd_srbd_uses_store_updated_dmem_for_macc();
	test_lacc_lira_same_word_uses_pre_lacc_aacc();
	test_cmem_update_multiword_sequence();
	test_cmem_update_waits_for_cload_high();
	test_cmem_update_new_address_breaks_active_run();
	test_pc33_mpy_mac_direct_coeff_chain();
	test_dsp3_rde_word0_sel1_dram_latency();
	test_dsp3_a02_su480002_input_sensitivity();
	test_dsp3_a02_su480003_input_sensitivity();
	test_dsp3_a02_sine_su480003_snapshot_replay();
	test_dsp3_a02_su480004_snapshot_replay();
	test_dsp3_a02_su480004_input_sensitivity();
	test_dsp3_a02_su480005_snapshot_replay();
	test_dsp3_a02_su480005_input_sensitivity();
	probe_dsp3_a02_su480005_state_sensitivity();
	probe_dsp3_a02_su480005_first_diff_variants();
	probe_dsp3_a02_su480004_producers();
	probe_dsp3_a02_su480004_producer_site_ba0_variants();
	probe_dsp3_a02_su480005_ba0_phase_sensitivity();
	probe_dsp3_a02_su480005_site_ba0_variants();
	probe_dsp3_a02_su480005_repeat_site_ba0_variants();
	probe_dsp3_a02_tape_su480003_group_sensitivity();
	probe_dsp3_a02_tape_su480003_producer_sensitivity();
	probe_dsp3_a02_tape_su480002_hot_producer_sensitivity();
	probe_dsp3_a02_tape_su480002_serial_firstdiff();
	probe_dsp3_a02_tape_su480002_serial_path_checks();
	probe_dsp3_serial_side_boundary_sensitivity();
	probe_dsp3_a02_fix_su95299_coeff_sensitivity();
	probe_dsp3_a02_tape_boundary_replay();
	probe_dsp3_a02_tape_cycle_budget();
	test_dsp3_a02_su480006_snapshot_replay();
	test_dsp3_a02_su480006_input_sensitivity();
	test_dsp3_a02_su480001_pc68_mac_matches_input();
	dump_dsp3_a02_su480006_replay_block();
	test_dsp3_a02_su480082_snapshot_replay();

	if (m_failures != 0)
		throw emu_fatalerror(1, "%d TMS57002 tests failed", m_failures);

	throw emu_fatalerror(0, "All TMS57002 tests passed");
}


static INPUT_PORTS_START(tms57002test)
INPUT_PORTS_END


void tms57002test_state::tms57002test(machine_config &config)
{
	TMS57002(config, m_dsp, XTAL(24'576'000));
	m_dsp->set_addrmap(AS_DATA, &tms57002test_state::dsp_ram_map);

	screen_device &screen(SCREEN(config, "screen"));
	screen.set_raw(1'000'000, 80, 0, 64, 80, 0, 64);
	screen.set_screen_update(FUNC(tms57002test_state::screen_update));
}

void tms57002test_state::dsp_ram_map(address_map &map)
{
	map(0x00000, 0x3ffff).ram();
}


ROM_START(tms57002test)
	ROM_REGION(0x10, "dsp", ROMREGION_ERASE00)
ROM_END

} // anonymous namespace


GAME(2026, tms57002test, 0, tms57002test, tms57002test, tms57002test_state, empty_init, ROT0, "MAME", "TMS57002 Semantics Harness", MACHINE_NO_SOUND_HW)
