// license:BSD-3-Clause
// copyright-holders:Patrick Mackinlay

#ifndef MAME_CPU_NEC_V5X_H
#define MAME_CPU_NEC_V5X_H

#pragma once

#include "nec.h"

#include "machine/am9517a.h"
#include "machine/i8251.h"
#include "machine/pic8259.h"
#include "machine/pit8253.h"
#include "machine/timer.h"

class device_v5x_interface : public device_interface
{
public:
	// TCU
	void set_tclk(double clk) { m_tclk = clk; }
	void set_tclk(const XTAL &xtal) { set_tclk(xtal.dvalue()); }
	void tclk_w(int state);

	// DMAU
	auto out_hreq_cb() { return m_dmau.lookup()->out_hreq_callback(); }
	auto out_eop_cb() { return m_dmau.lookup()->out_eop_callback(); }
	auto in_memr_cb() { return m_dmau.lookup()->in_memr_callback(); }
	auto in_mem16r_cb() { return m_dmau.lookup()->in_mem16r_callback(); }
	auto out_memw_cb() { return m_dmau.lookup()->out_memw_callback(); }
	auto out_mem16w_cb() { return m_dmau.lookup()->out_mem16w_callback(); }
	template <unsigned Channel> auto in_ior_cb() { return m_dmau.lookup()->in_ior_callback<Channel>(); }
	template <unsigned Channel> auto in_io16r_cb() { return m_dmau.lookup()->in_io16r_callback<Channel>(); }
	template <unsigned Channel> auto out_iow_cb() { return m_dmau.lookup()->out_iow_callback<Channel>(); }
	template <unsigned Channel> auto out_io16w_cb() { return m_dmau.lookup()->out_io16w_callback<Channel>(); }
	template <unsigned Channel> auto out_dack_cb() { return m_dmau.lookup()->out_dack_callback<Channel>(); }

	// SCU
	auto txd_handler_cb() { return m_scu.lookup()->txd_handler(); }
	void rxd_w(int state) { m_scu->write_rxd(state); }

protected:
	device_v5x_interface(const machine_config &mconfig, nec_common_device &device, u32 clock, bool is_16bit);

	// device_interface overrides
	virtual void interface_post_start() override;
	virtual void interface_pre_reset() override;
	virtual void interface_post_load() override;
	virtual void interface_clock_changed(bool sync_on_new_clock_domain) override;

	void v5x_set_input(int inputnum, int state);
	void v5x_add_mconfig(machine_config &config);

	virtual void install_peripheral_io() = 0;

	const int AS_INTERNAL_IO = AS_OPCODES + 1;
	const u8 INTERNAL_IO_ADDR_WIDTH = (1 << 3);
	const u8 INTERNAL_IO_ADDR_MASK = (1 << INTERNAL_IO_ADDR_WIDTH) - 1;
	const u16 OPHA_MASK = INTERNAL_IO_ADDR_MASK << INTERNAL_IO_ADDR_WIDTH;

	inline u16 OPHA() { return (m_OPHA << INTERNAL_IO_ADDR_WIDTH) & OPHA_MASK; }
	inline u16 io_mask(u8 base) { return 0x00ff << ((base & 1) << 3); }
	inline bool check_OPHA(offs_t a)
	{
		return ((m_OPSEL & OPSEL_MASK) != 0) && (m_OPHA != 0xff) && ((a & OPHA_MASK) == OPHA()); // 256 bytes boundary, ignore system io area
	}

	inline u8 internal_io_read_byte(offs_t a) { return m_internal_io->read_byte(a & INTERNAL_IO_ADDR_MASK); }
	inline u16 internal_io_read_word(offs_t a) { return m_internal_io->read_word_unaligned(a & INTERNAL_IO_ADDR_MASK); }
	inline void internal_io_write_byte(offs_t a, u8 v) { m_internal_io->write_byte(a & INTERNAL_IO_ADDR_MASK, v); }
	inline void internal_io_write_word(offs_t a, u16 v) { m_internal_io->write_word_unaligned(a & INTERNAL_IO_ADDR_MASK, v); }

	void remappable_io_map(address_map &map) ATTR_COLD;
	virtual u8 temp_io_byte_r(offs_t offset) = 0;
	virtual void temp_io_byte_w(offs_t offset, u8 data) = 0;

	void BSEL_w(u8 data) {}
	void BADR_w(u8 data) {}
	void BRC_w(u8 data);
	void WMB0_w(u8 data) {}
	void WCY1_w(u8 data) {}
	void WCY0_w(u8 data) {}
	void WAC_w(u8 data) {}
	u8 TCKS_r();
	void TCKS_w(u8 data);
	void SBCR_w(u8 data) {}
	void RFC_w(u8 data) {}
	void WMB1_w(u8 data) {}
	void WCY2_w(u8 data) {}
	void WCY3_w(u8 data) {}
	void WCY4_w(u8 data) {}
	u8 SULA_r();
	void SULA_w(u8 data);
	u8 TULA_r();
	void TULA_w(u8 data);
	u8 IULA_r();
	void IULA_w(u8 data);
	u8 DULA_r();
	void DULA_w(u8 data);
	u8 OPHA_r();
	void OPHA_w(u8 data);
	u8 OPSEL_r();
	void OPSEL_w(u8 data);
	virtual u8 get_pic_ack(offs_t offset) { return 0; }
	void internal_irq_w(int state);

	void tcu_clock_update();
	void brc_update();

	TIMER_DEVICE_CALLBACK_MEMBER(brc_timer_tick);

	virtual void sint_w(int state) = 0;

	required_device<pit8253_device> m_tcu;
	required_device<v5x_dmau_device> m_dmau;
	required_device<v5x_icu_device> m_icu;
	required_device<v5x_scu_device> m_scu;
	required_device<timer_device> m_brc_timer;

	address_space_config m_internal_io_config;
	address_space *m_internal_io;

	double m_tclk;

	enum opsel_mask
	{
		OPSEL_DS = 0x01, // dmau enabled
		OPSEL_IS = 0x02, // icu enabled
		OPSEL_TS = 0x04, // tcu enabled
		OPSEL_SS = 0x08, // scu enabled
		OPSEL_MASK = OPSEL_DS | OPSEL_IS | OPSEL_TS | OPSEL_SS
	};
	u8 m_OPSEL;

	u8 m_SULA;
	u8 m_TULA;
	u8 m_IULA;
	u8 m_DULA;
	u8 m_OPHA;
	u8 m_TCKS;
	u8 m_BRC;

	bool m_brc_enable;
};

class v50_base_device : public nec_common_device, public device_v5x_interface
{
public:
	template <unsigned Channel> void dreq_w(int state) { m_dmau->dreq_w<Channel>(state); }
	void hack_w(int state) { m_dmau->hack_w(state); }
	void tctl2_w(int state) { m_tcu->write_gate2(state); }

	auto tout1_cb() { return m_tout1_callback.bind(); }
	auto tout2_cb() { return m_tcu.lookup()->out_handler<2>(); }

	auto icu_slave_ack_cb() { return m_icu_slave_ack.bind(); }

protected:
	v50_base_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, u32 clock, bool is_16bit, u8 prefetch_size, u8 prefetch_cycles, u32 chip_type);

	// device-specific overrides
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

	// device_execute_interface overrides
	virtual uint64_t execute_clocks_to_cycles(uint64_t clocks) const noexcept override { return (clocks / 2); }
	virtual uint64_t execute_cycles_to_clocks(uint64_t cycles) const noexcept override { return (cycles * 2); }
	virtual void execute_set_input(int inputnum, int state) override;

	// device_memory_interface overrides
	virtual space_config_vector memory_space_config() const override;

	virtual u8 temp_io_byte_r(offs_t offset) override { return nec_common_device::io_read_byte(OPHA() | (offset & INTERNAL_IO_ADDR_MASK)); }
	virtual void temp_io_byte_w(offs_t offset, u8 data) override { nec_common_device::io_write_byte(OPHA() | (offset & INTERNAL_IO_ADDR_MASK), data); }

	virtual u8 io_read_byte(offs_t a) override;
	virtual u16 io_read_word(offs_t a) override;
	virtual void io_write_byte(offs_t a, u8 v) override;
	virtual void io_write_word(offs_t a, u16 v) override;

	void internal_port_map(address_map &map) ATTR_COLD;

	u8 OPCN_r();
	void OPCN_w(u8 data);

	// TODO: non-offset 7 configuration
	// Currently used by pc88va only, which uses the canonical IRQ7 for cascading an external PIC to the internal one.
	virtual u8 get_pic_ack(offs_t offset) override
	{
		if (offset == 7)
			return m_icu_slave_ack(0);
		return 0;
	}


private:
	void tout1_w(int state);
	virtual void sint_w(int state) override;

	devcb_write_line m_tout1_callback;
	devcb_read8 m_icu_slave_ack;

	u8 m_OPCN;
	u8 m_tout1;
	u8 m_sint;
	u8 m_intp1;
	u8 m_intp2;
};

class v40_device : public v50_base_device
{
public:
	v40_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

protected:
	virtual void install_peripheral_io() override;
};

class v50_device : public v50_base_device
{
public:
	v50_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

protected:
	virtual void install_peripheral_io() override;
};

class v53_device : public v33_base_device, public device_v5x_interface
{
public:
	v53_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	template <unsigned Channel> void dreq_w(int state)
	{
		// dreq0 could be wrong / nonexistent
		if (!(m_SCTL & 0x02))
		{
			m_dmau->dreq_w<Channel>(state);
		}
		else
		{
			logerror("dreq%d not in 71071mode\n", Channel);
		}
	}
	void hack_w(int state);

	template <unsigned Timer> auto tout_handler() { return m_tcu.lookup()->out_handler<Timer>(); }

	// SCU
	auto dtr_handler_cb() { return m_scu.lookup()->dtr_handler(); }
	auto rts_handler_cb() { return m_scu.lookup()->rts_handler(); }
	auto rxrdy_handler_cb() { return m_scu.lookup()->rxrdy_handler(); }
	auto txrdy_handler_cb() { return m_scu.lookup()->txrdy_handler(); }
	auto txempty_handler_cb() { return m_scu.lookup()->txempty_handler(); }
	auto syndet_handler_cb() { return m_scu.lookup()->syndet_handler(); }
	auto sint_handler_cb() { return m_sint_w.bind(); }

	void dsr_w(int state) { m_scu->write_dsr(state); }
	void cts_w(int state) { m_scu->write_cts(state); }

	template <unsigned Timer> auto v53_tout_handler()
	{
		if (Timer != 1)
		{
			return m_tcu.lookup()->out_handler<Timer>();
		}
		else
		{
			return m_tout1_w.bind();
		}
	}

protected:
	v53_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, u32 clock, bool v55_extensions = false);

	// device-specific overrides
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

	// device_execute_interface overrides
	virtual void execute_set_input(int inputnum, int state) override;

	// device_memory_interface overrides
	virtual space_config_vector memory_space_config() const override;

	virtual u8 temp_io_byte_r(offs_t offset) override { return nec_common_device::io_read_byte(OPHA() | (offset & INTERNAL_IO_ADDR_MASK)); }
	virtual void temp_io_byte_w(offs_t offset, u8 data) override { nec_common_device::io_write_byte(OPHA() | (offset & INTERNAL_IO_ADDR_MASK), data); }

	virtual u8 io_read_byte(offs_t a) override;
	virtual u16 io_read_word(offs_t a) override;
	virtual void io_write_byte(offs_t a, u8 v) override;
	virtual void io_write_word(offs_t a, u16 v) override;

	void internal_port_map(address_map &map) ATTR_COLD;
	virtual void install_peripheral_io() override;

	u8 SCTL_r();
	void SCTL_w(u8 data);

	void tout1_w(int state);

	virtual void sint_w(int state) override;

private:
	devcb_write_line m_sint_w, m_tout1_w;
	u8 m_SCTL;
};

class v53a_device : public v53_device
{
public:
	v53a_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);
};

class v55_device : public v53_device
{
public:
	enum serial_irq_source : u8
	{
		SERIAL_IRQ_INTSER0 = 0,
		SERIAL_IRQ_INTSER1,
		SERIAL_IRQ_INTSR0,
		SERIAL_IRQ_INTSR1,
		SERIAL_IRQ_INTST0,
		SERIAL_IRQ_INTST1,
		SERIAL_IRQ_COUNT
	};

	enum timer_irq_source : u8
	{
		TIMER_IRQ_INTCM21 = 0,
		TIMER_IRQ_INTCM31,
		TIMER_IRQ_COUNT
	};
	static constexpr unsigned SPECIAL_IRQ_ADC = unsigned(SERIAL_IRQ_COUNT) + unsigned(TIMER_IRQ_COUNT);
	static constexpr unsigned SPECIAL_IRQ_COUNT = SPECIAL_IRQ_ADC + 1;

	v55_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	template <unsigned Port> auto in_p_cb()
	{
		static_assert(Port < 9);
		return m_port_in_cb[Port].bind();
	}

	template <unsigned Port> auto out_p_cb()
	{
		static_assert(Port < 9);
		return m_port_out_cb[Port].bind();
	}

	template <unsigned Channel> auto read_adc()
	{
		static_assert(Channel < 4);
		return m_adc_in_cb[Channel].bind();
	}

	auto txd_handler_cb() { return m_txd0_handler.bind(); }
	auto txd1_handler_cb() { return m_txd1_handler.bind(); }
	auto adc_fint_cb() { return m_adc_fint_cb.bind(); }
	void rxd_w(int state);
	void inject_uart0_rx_byte(u8 data);
	void cts_w(int state);
	void rxd1_w(int state);
	void cts1_w(int state);
	void set_timer_irq_bank(timer_irq_source source, u8 bank) { m_timer_irq_bank[unsigned(source)] = bank & 0x0f; }
	enum class serial_irq_mode : u8
	{
		off = 0,
		tx0_only,
		rx0_tx0,
		broad,
		rx0_only,
		rx0_late_tx0,
		rx0_rx1
	};
	void set_serial_irq_mode(serial_irq_mode mode) { m_default_serial_irq_mode = mode; }
	u8 debug_uart0_mode() const { return m_sfr[0x173]; }
	u8 debug_uart0_status() const { return m_sfr[0x174]; }
	u8 debug_uart0_data() const { return m_sfr[0x175]; }
	u8 debug_uart0_tx_byte() const { return m_uart0_tx_byte; }
	u8 debug_uart0_tx_bit() const { return m_uart0_tx_bit; }
	bool debug_uart0_tx_active() const { return m_uart0_tx_active; }
	bool debug_uart0_tx_loaded() const { return m_uart0_tx_loaded; }
	u64 debug_uart0_tx_completions() const { return m_uart0_tx_completions; }
	bool debug_uart0_rx_full() const { return m_uart0_rx_full; }
	u8 debug_uart0_cts() const { return m_cts0; }
	u8 debug_serial_irq_control(serial_irq_source source) const
	{
		return m_sfr[0x0da + unsigned(source)];
	}
	u64 debug_uart1_rx_bytes() const { return m_uart1_rx_bytes; }
	u64 debug_uart1_rx_consumed() const { return m_uart1_rx_consumed; }
	u64 debug_uart1_rx_overruns() const { return m_uart1_rx_overruns; }
	u64 debug_uart1_rx_framing_errors() const { return m_uart1_rx_framing_errors; }
	u8 debug_uart1_status() const { return m_sfr[0x17c]; }
	u8 debug_uart1_data() const { return m_sfr[0x17e]; }
	u8 debug_serial_irq_pending() const { return m_internal_serial_irq_pending; }
	u8 debug_serial_irq_in_service() const { return m_internal_serial_irq_in_service; }
	u8 debug_logical_read_byte(offs_t address) { return mem_read_byte(address); }
	u16 debug_logical_read_word(offs_t address) { return mem_read_word(address); }
	void debug_logical_write_byte(offs_t address, u8 data) { mem_write_byte(address, data); }
	void debug_logical_write_word(offs_t address, u16 data) { mem_write_word(address, data); }

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

	virtual space_config_vector memory_space_config() const override;
	virtual bool memory_translate(int spacenum, int intention, offs_t &address, address_space *&target_space) override;
	virtual u8 mem_read_byte(offs_t a) override;
	virtual u16 mem_read_word(offs_t a) override;
	virtual void mem_write_byte(offs_t a, u8 v) override;
	virtual void mem_write_word(offs_t a, u16 v) override;
	virtual bool handle_special_int_ack() override;
	virtual void v55_fint() override;

private:
	void sfr_map(address_map &map) ATTR_COLD;
	u8 sfr_r(offs_t offset);
	void sfr_w(offs_t offset, u8 data);
	u8 port_r(unsigned port);
	void port_w(unsigned port, u8 data);
	bool port2_output_enabled() const;
	void update_port2_output();
	void update_timer(timer_irq_source source);
	attotime timer_tick_period(timer_irq_source source) const;
	TIMER_CALLBACK_MEMBER(timer_tick);
	void request_timer_irq(timer_irq_source source);
	bool timer_irq_experiment_enabled() const;
	bool timer_irq_enabled(timer_irq_source source) const;
	bool timer_irq_bankswitch(timer_irq_source source) const;
	u8 timer_irq_priority(timer_irq_source source) const;
	u8 timer_irq_ic(timer_irq_source source) const;
	u8 timer_irq_vector(timer_irq_source source) const;
	u16 timer_compare(timer_irq_source source) const;
	u8 timer_control_enable_bit(timer_irq_source source) const;
	u8 timer_control_prescale_bit(timer_irq_source source) const;
	int timer_irq_index_from_source(int source) const;
	bool adc_irq_experiment_enabled() const;
	bool adc_irq_enabled() const;
	bool adc_irq_bankswitch() const;
	u8 adc_irq_priority() const;
	u8 adc_irq_bank();
	void update_adc_timer();
	attotime adc_tick_period() const;
	void update_adc_results();
	void request_adc_irq();
	TIMER_CALLBACK_MEMBER(adc_tick);
	void update_uart0_status();
	bool uart0_tx_enabled() const;
	void start_uart0_tx();
	attotime uart_bit_period(unsigned channel, bool transmit) const;
	attotime uart0_tx_bit_period() const { return uart_bit_period(0, true); }
	attotime uart0_rx_bit_period() const { return uart_bit_period(0, false); }
	TIMER_CALLBACK_MEMBER(uart0_tx_tick);
	TIMER_CALLBACK_MEMBER(uart0_rx_tick);
	void update_uart1_status();
	void start_uart1_tx();
	attotime uart1_tx_bit_period() const { return uart_bit_period(1, true); }
	attotime uart1_rx_bit_period() const { return uart_bit_period(1, false); }
	TIMER_CALLBACK_MEMBER(uart1_tx_tick);
	TIMER_CALLBACK_MEMBER(uart1_rx_tick);
	int select_internal_special_irq() const;
	int current_internal_special_irq_source() const;
	void update_internal_serial_irq_line();
	void request_internal_serial_irq(serial_irq_source source);
	bool tx0_late_irq_window() const;
	bool internal_serial_irq_enabled(serial_irq_source source) const;
	bool internal_serial_irq_bankswitch(serial_irq_source source) const;
	u8 internal_serial_irq_bank(serial_irq_source source);
	int select_internal_serial_irq() const;
	int current_internal_serial_irq_source() const;
	u8 internal_serial_irq_ic(serial_irq_source source) const;
	serial_irq_mode current_serial_irq_mode() const;
	u8 interrupt_vector_bank(u8 vector);
	u16 interrupt_vector_address(u8 vector) const;
	u8 special_irq_vector(int source) const;
	u8 special_irq_priority(int source) const;
	bool priority_is_tracked(u8 priority) const;
	bool can_accept_priority(u8 priority) const;
	void mark_special_irq_in_service(int source);
	int current_special_irq_stack_source() const;
	int pop_special_irq_stack_source();
	void clear_special_irq_source(int source);

	address_space_config m_sfr_config;
	std::array<u8, 0x200> m_sfr;
	devcb_read8::array<4> m_adc_in_cb;
	devcb_read8::array<9> m_port_in_cb;
	devcb_write8::array<9> m_port_out_cb;
	devcb_write_line m_txd0_handler;
	devcb_write_line m_txd1_handler;
	devcb_write_line m_adc_fint_cb;
	std::array<emu_timer *, TIMER_IRQ_COUNT> m_timer;
	emu_timer *m_adc_timer;
	emu_timer *m_uart0_tx_timer;
	emu_timer *m_uart0_rx_timer;
	emu_timer *m_uart1_tx_timer;
	emu_timer *m_uart1_rx_timer;
	bool m_adc_running;
	u8 m_rxd0;
	u8 m_cts0;
	u8 m_uart0_txd_state;
	u8 m_uart0_tx_byte;
	u8 m_uart0_tx_bit;
	u8 m_uart0_rx_byte;
	u8 m_uart0_rx_bit;
	u8 m_uart0_rx_prev;
	bool m_uart0_tx_active;
	bool m_uart0_tx_loaded;
	bool m_uart0_rx_active;
	bool m_uart0_rx_full;
	u64 m_uart0_tx_completions;
	u8 m_rxd1;
	u8 m_cts1;
	u8 m_uart1_txd_state;
	u8 m_uart1_tx_byte;
	u8 m_uart1_tx_bit;
	u8 m_uart1_rx_byte;
	u8 m_uart1_rx_bit;
	u8 m_uart1_rx_prev;
	bool m_uart1_tx_active;
	bool m_uart1_tx_loaded;
	bool m_uart1_rx_active;
	bool m_uart1_rx_full;
	u64 m_uart1_rx_bytes;
	u64 m_uart1_rx_consumed;
	u64 m_uart1_rx_overruns;
	u64 m_uart1_rx_framing_errors;
	serial_irq_mode m_default_serial_irq_mode = serial_irq_mode::off;
	std::array<u8, TIMER_IRQ_COUNT> m_timer_irq_bank;
	std::array<bool, TIMER_IRQ_COUNT> m_timer_irq_pending;
	std::array<bool, TIMER_IRQ_COUNT> m_timer_irq_in_service;
	bool m_adc_irq_pending;
	bool m_adc_irq_in_service;
	u8 m_internal_serial_irq_pending;
	u8 m_internal_serial_irq_in_service;
	std::array<u8, SPECIAL_IRQ_COUNT> m_special_irq_stack;
	u8 m_special_irq_stack_depth;
	serial_irq_mode m_serial_irq_mode;
	bool m_timer_irq_experiment;
	bool m_adc_irq_experiment;
	u8 m_adc_irq_bank;
};

DECLARE_DEVICE_TYPE(V40,  v40_device)
DECLARE_DEVICE_TYPE(V50,  v50_device)
DECLARE_DEVICE_TYPE(V53,  v53_device)
DECLARE_DEVICE_TYPE(V53A, v53a_device)
DECLARE_DEVICE_TYPE(V55,  v55_device)

#endif // MAME_CPU_NEC_V5X_H
