// license:BSD-3-Clause
// copyright-holders:Olivier Galibert
/***************************************************************************

    h83003.h

    H8/3003

    H8/300H-based mcus.

***************************************************************************/

#ifndef MAME_CPU_H8_H83003_H
#define MAME_CPU_H8_H83003_H

#pragma once

#include "h8h.h"
#include "h8_adc.h"
#include "h8_dma.h"
#include "h8_port.h"
#include "h8_intc.h"
#include "h8_timer16.h"
#include "h8_sci.h"
#include "h8_watchdog.h"

class h83003_device : public h8h_device {
public:
	h83003_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	auto tend0() { return m_tend_cb[0].bind(); }
	auto tend1() { return m_tend_cb[1].bind(); }
	auto tend2() { return m_tend_cb[2].bind(); }
	auto tend3() { return m_tend_cb[3].bind(); }

	auto read_port4()  { return m_read_port [PORT_4].bind(); }
	auto write_port4() { return m_write_port[PORT_4].bind(); }
	auto read_port5()  { return m_read_port [PORT_5].bind(); }
	auto write_port5() { return m_write_port[PORT_5].bind(); }
	auto read_port6()  { return m_read_port [PORT_6].bind(); }
	auto write_port6() { return m_write_port[PORT_6].bind(); }
	auto read_port7()  { return m_read_port [PORT_7].bind(); }
	auto read_port8()  { return m_read_port [PORT_8].bind(); }
	auto write_port8() { return m_write_port[PORT_8].bind(); }
	auto read_port9()  { return m_read_port [PORT_9].bind(); }
	auto write_port9() { return m_write_port[PORT_9].bind(); }
	auto read_porta()  { return m_read_port [PORT_A].bind(); }
	auto write_porta() { return m_write_port[PORT_A].bind(); }
	auto read_portb()  { return m_read_port [PORT_B].bind(); }
	auto write_portb() { return m_write_port[PORT_B].bind(); }
	auto read_portc()  { return m_read_port [PORT_C].bind(); }
	auto write_portc() { return m_write_port[PORT_C].bind(); }

	void set_mode_a20(bool initial_bus_16bit = false) { m_mode_a20 = true; m_initial_bus_16bit = initial_bus_16bit; }
	void set_mode_a24(bool initial_bus_16bit = false) { m_mode_a20 = false; m_initial_bus_16bit = initial_bus_16bit; }
	void set_external_bus_timing(bool enable) { m_external_bus_timing = enable; }

	u8 syscr_r();
	void syscr_w(u8 data);

	u8 rtmcsr_r();
	void rtmcsr_w(u8 data);
	u8 abwcr_r();
	void abwcr_w(u8 data);
	u8 astcr_r();
	void astcr_w(u8 data);
	u8 wcr_r();
	void wcr_w(u8 data);
	u8 wcer_r();
	void wcer_w(u8 data);
	u8 mdcr_r();
	u8 brcr_r();
	void brcr_w(u8 data);

protected:
	required_device<h8h_intc_device> m_intc;
	required_device<h8_adc_device> m_adc;
	required_device<h8h_dma_device> m_dma;
	required_device<h8h_dma_channel_device> m_dma0;
	required_device<h8h_dma_channel_device> m_dma1;
	required_device<h8h_dma_channel_device> m_dma2;
	required_device<h8h_dma_channel_device> m_dma3;
	required_device<h8_port_device> m_port4;
	required_device<h8_port_device> m_port5;
	required_device<h8_port_device> m_port6;
	required_device<h8_port_device> m_port7;
	required_device<h8_port_device> m_port8;
	required_device<h8_port_device> m_port9;
	required_device<h8_port_device> m_porta;
	required_device<h8_port_device> m_portb;
	required_device<h8_port_device> m_portc;
	required_device<h8_timer16_device> m_timer16;
	required_device<h8h_timer16_channel_device> m_timer16_0;
	required_device<h8h_timer16_channel_device> m_timer16_1;
	required_device<h8h_timer16_channel_device> m_timer16_2;
	required_device<h8h_timer16_channel_device> m_timer16_3;
	required_device<h8h_timer16_channel_device> m_timer16_4;
	required_device<h8_watchdog_device> m_watchdog;
	memory_view m_ram_view;

	devcb_write_line::array<4> m_tend_cb;

	u8 m_syscr;
	u8 m_rtmcsr;
	u8 m_abwcr;
	u8 m_astcr;
	u8 m_wcr;
	u8 m_wcer;
	u8 m_brcr;
	bool m_initial_bus_16bit = false;
	bool m_external_bus_timing = false;

	virtual void update_irq_filter() override;
	virtual void interrupt_taken() override;
	virtual int trapa_setup() override;
	virtual void irq_setup() override;
	virtual void internal_update(u64 current_time) override;
	using h8_device::internal_update;
	virtual void notify_standby(int state) override;
	virtual int reset_processing_cycles() const override;
	virtual int interrupt_priority_cycles() const override;
	virtual void interrupt_priority_complete() override;
	virtual bool interrupt_post_accept_prefetch() const override;
	virtual bool internal_phase_checkpointing_enabled() const override;
	virtual void interrupt_instruction_boundary() override;
	virtual int dma_bus_acquisition_cycles(int channel) const override;
	virtual int memory_access_cycles(u32 address, int size) const override;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	void map(address_map &map) ATTR_COLD;

	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void execute_set_input(int inputnum, int state) override;
};

DECLARE_DEVICE_TYPE(H83003, h83003_device)

#endif // MAME_CPU_H8_H83003_H
