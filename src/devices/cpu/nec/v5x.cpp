// license:BSD-3-Clause
// copyright-holders:Patrick Mackinlay

/*
 * NEC V5x devices consist of a V3x CPU core plus integrated peripherals. The
 * CPU cores within each device are as follows:
 *
 *   Device            CPU
 *   V40 (µPD70208)    V20 (µPD70108)
 *   V50 (µPD70216)    V30 (µPD70116)
 *   V53 (µPD70236)    V33 (µPD70136)
 *   V53A (µPD70236A)  V33A (µPD70136A)
 *   V55 (µPD70433)    V33A-compatible core with V55 extensions (partial)
 *
 *   V40HL and V50HL (µPD70208h and µPD70216h) exist and have additional
 *   features like the V53.
 *   In particular, they have a V53 binary compatible SCTL register at $FFF7,
 *   including 8/16 bit select and SCU clock source select.  They also have
 *   the same independent baud rate generator as the V53.  If those variants
 *   are added, that needs to be hooked up for proper emulation.
 *
 * The peripherals are nearly identical between all four devices:
 *
 *   Name  Description             Device
 *   TCU   Timer/Counter Unit      µPD71054/i8254 subset
 *   DMAU  DMA Control Unit        µPD71071 equivalent
 *   ICU   Interrupt control Unit  µPD71059/i8259 equivalent
 *   SCU   Serial Control Unit     µPD71051/i8251 subset (async only)
 *
 * The V53/V53A DMAU also supports a configurable µPD71037/i8237A mode.
 *
 * Sources:
 *
 *   http://www.chipfind.net/datasheet/pdf/nec/upd70236.pdf
 *   https://datasheet.datasheetarchive.com/originals/scans/Scans-107/DSASCANS15-59637.pdf
 *
 */
#include "emu.h"
#include "v5x.h"

#include "necpriv.ipp"

#define VERBOSE 0
#include "logmacro.h"

DEFINE_DEVICE_TYPE(V40,  v40_device,  "v40",  "NEC V40")
DEFINE_DEVICE_TYPE(V50,  v50_device,  "v50",  "NEC V50")
DEFINE_DEVICE_TYPE(V53,  v53_device,  "v53",  "NEC V53")
DEFINE_DEVICE_TYPE(V53A, v53a_device, "v53a", "NEC V53A")
DEFINE_DEVICE_TYPE(V55,  v55_device,  "v55",  "NEC V55")

u8 device_v5x_interface::SULA_r()
{
	return m_SULA;
}

void device_v5x_interface::SULA_w(u8 data)
{
	if (VERBOSE)
		device().logerror("SULA_w %02x\n", data);
	m_SULA = data;
	install_peripheral_io();
}

u8 device_v5x_interface::TULA_r()
{
	return m_TULA;
}

void device_v5x_interface::TULA_w(u8 data)
{
	if (VERBOSE)
		device().logerror("TULA_w %02x\n", data);
	m_TULA = data;
	install_peripheral_io();
}

u8 device_v5x_interface::IULA_r()
{
	return m_IULA;
}

void device_v5x_interface::IULA_w(u8 data)
{
	if (VERBOSE)
		device().logerror("IULA_w %02x\n", data);
	m_IULA = data;
	install_peripheral_io();
}

u8 device_v5x_interface::DULA_r()
{
	return m_DULA;
}

void device_v5x_interface::DULA_w(u8 data)
{
	if (VERBOSE)
		device().logerror("DULA_w %02x\n", data);
	m_DULA = data;
	install_peripheral_io();
}

u8 device_v5x_interface::OPHA_r()
{
	return m_OPHA;
}

void device_v5x_interface::OPHA_w(u8 data)
{
	if (VERBOSE)
	{
		device().logerror("OPHA_w %02x\n", data);
		if (data == 0xff)
			device().logerror("OPHA is mapped in system IO area!\n", data);
	}
	m_OPHA = data;
}

u8 device_v5x_interface::OPSEL_r()
{
	return m_OPSEL;
}

void device_v5x_interface::OPSEL_w(u8 data)
{
	if (VERBOSE)
		device().logerror("OPSEL_w %02x\n", data);
	m_OPSEL = data & 0x0f;
	install_peripheral_io();
}

u8 device_v5x_interface::TCKS_r()
{
	return m_TCKS;
}

void device_v5x_interface::TCKS_w(u8 data)
{
	m_TCKS = data;
	tcu_clock_update();
}

void device_v5x_interface::interface_clock_changed(bool sync_on_new_clock_domain)
{
	tcu_clock_update();
	brc_update();
}

void device_v5x_interface::tcu_clock_update()
{
	for (int i = 0; i < 3; i++)
		m_tcu->set_clockin(i, BIT(m_TCKS, i + 2) ? m_tclk : device().clock() / double(4 << (m_TCKS & 3)));
}

void device_v5x_interface::BRC_w(u8 data)
{
	m_BRC = data;
	brc_update();
}

void device_v5x_interface::brc_update()
{
	if (m_brc_enable)
	{
		const int divider = (m_BRC < 2) ? 2 : m_BRC;
		const int period = (device().clock() / 2) / divider;

		m_brc_timer->adjust(attotime::from_hz(period), 0, attotime::from_hz(period));
	}
}

TIMER_DEVICE_CALLBACK_MEMBER(device_v5x_interface::brc_timer_tick)
{
	m_scu->write_txc(0);
	m_scu->write_txc(1);
	m_scu->write_rxc(0);
	m_scu->write_rxc(1);
}

void device_v5x_interface::tclk_w(int state)
{
	if (BIT(m_TCKS, 2))
		m_tcu->write_clk0(state);
	if (BIT(m_TCKS, 3))
		m_tcu->write_clk1(state);
	if (BIT(m_TCKS, 4))
		m_tcu->write_clk2(state);
}

void device_v5x_interface::interface_pre_reset()
{
	m_OPSEL= 0x00;

	// peripheral addresses
	m_SULA = 0x00;
	m_TULA = 0x00;
	m_IULA = 0x00;
	m_DULA = 0x00;
	m_OPHA = 0x00;
	m_BRC  = 0x00;

	m_TCKS = 0x00;
	m_brc_enable = false;
	tcu_clock_update();
}

void device_v5x_interface::interface_post_start()
{
	device().save_item(NAME(m_OPSEL));
	device().save_item(NAME(m_SULA));
	device().save_item(NAME(m_TULA));
	device().save_item(NAME(m_IULA));
	device().save_item(NAME(m_DULA));
	device().save_item(NAME(m_OPHA));
	device().save_item(NAME(m_TCKS));
	device().save_item(NAME(m_BRC));
	device().save_item(NAME(m_brc_enable));
}

void device_v5x_interface::interface_post_load()
{
	install_peripheral_io();
}

// the external interface provides no external access to the usual IRQ line of the V33, everything goes through the interrupt controller
void device_v5x_interface::v5x_set_input(int irqline, int state)
{
	switch (irqline)
	{
		case INPUT_LINE_IRQ0: m_icu->ir0_w(state); break;
		case INPUT_LINE_IRQ1: m_icu->ir1_w(state); break;
		case INPUT_LINE_IRQ2: m_icu->ir2_w(state); break;
		case INPUT_LINE_IRQ3: m_icu->ir3_w(state); break;
		case INPUT_LINE_IRQ4: m_icu->ir4_w(state); break;
		case INPUT_LINE_IRQ5: m_icu->ir5_w(state); break;
		case INPUT_LINE_IRQ6: m_icu->ir6_w(state); break;
		case INPUT_LINE_IRQ7: m_icu->ir7_w(state); break;

		case INPUT_LINE_NMI: downcast<nec_common_device &>(device()).set_nmi_line(state); break;
		case NEC_INPUT_LINE_POLL: downcast<nec_common_device &>(device()).set_poll_line(state); break;
	}
}

// for hooking the interrupt controller output up to the core
void device_v5x_interface::internal_irq_w(int state)
{
	downcast<nec_common_device &>(device()).set_int_line(state);
}

void device_v5x_interface::v5x_add_mconfig(machine_config &config)
{
	PIT8254(config, m_tcu);

	V5X_DMAU(config, m_dmau, DERIVED_CLOCK(1, 4));

	V5X_ICU(config, m_icu);
	m_icu->out_int_callback().set(FUNC(device_v5x_interface::internal_irq_w));
	m_icu->in_sp_callback().set_constant(1);
	m_icu->read_slave_ack_callback().set(FUNC(device_v5x_interface::get_pic_ack));

	V5X_SCU(config, m_scu);

	TIMER(config, m_brc_timer);
	m_brc_timer->set_callback(FUNC(device_v5x_interface::brc_timer_tick));
}

void device_v5x_interface::remappable_io_map(address_map &map)
{
	map(0, INTERNAL_IO_ADDR_MASK).rw(FUNC(device_v5x_interface::temp_io_byte_r), FUNC(device_v5x_interface::temp_io_byte_w));
}

device_v5x_interface::device_v5x_interface(const machine_config &mconfig, nec_common_device &device, u32 clock, bool is_16bit)
	: device_interface(device, "v5x")
	, m_tcu(device, "tcu")
	, m_dmau(device, "dmau")
	, m_icu(device, "icu")
	, m_scu(device, "scu")
	, m_brc_timer(device, "brc_timer")
	, m_internal_io_config("internal_io", ENDIANNESS_LITTLE, is_16bit ? 16 : 8, INTERNAL_IO_ADDR_WIDTH, 0, address_map_constructor(FUNC(device_v5x_interface::remappable_io_map), this))
	, m_tclk(0.0)
	, m_OPSEL(0)
	, m_SULA(0)
	, m_TULA(0)
	, m_IULA(0)
	, m_DULA(0)
	, m_OPHA(0)
	, m_TCKS(0)
	, m_brc_enable(false)
{
}


u8 v50_base_device::io_read_byte(offs_t a)
{
	if (check_OPHA(a))
		return device_v5x_interface::internal_io_read_byte(a);
	else
		return nec_common_device::io_read_byte(a);
}

u16 v50_base_device::io_read_word(offs_t a)
{
	if (check_OPHA(a))
	{
		if ((a & INTERNAL_IO_ADDR_MASK) == INTERNAL_IO_ADDR_MASK)
		{
			return (device_v5x_interface::internal_io_read_byte(a) & 0x00ff)
				| ((nec_common_device::io_read_byte(a + 1) << 8) & 0xff00);
		}
		else
			return device_v5x_interface::internal_io_read_word(a);
	}
	else
		return nec_common_device::io_read_word(a);
}

void v50_base_device::io_write_byte(offs_t a, u8 v)
{
	if (check_OPHA(a))
	{
		device_v5x_interface::internal_io_write_byte(a, v);
	}
	else
		nec_common_device::io_write_byte(a, v);
}

void v50_base_device::io_write_word(offs_t a, u16 v)
{
	if (check_OPHA(a))
	{
		if ((a & INTERNAL_IO_ADDR_MASK) == INTERNAL_IO_ADDR_MASK)
		{
			device_v5x_interface::internal_io_write_byte(a, v & 0xff);
			nec_common_device::io_write_byte(a + 1, (v >> 8) & 0xff);
		}
		else
		{
			device_v5x_interface::internal_io_write_word(a, v);
		}
	}
	else
		nec_common_device::io_write_word(a, v);
}


u8 v50_base_device::OPCN_r()
{
	return m_OPCN;
}

void v50_base_device::OPCN_w(u8 data)
{
	// bit 7: unused
	// bit 6: unused
	// bit 5: unused
	// bit 4: unused
	// bit 3: IRSW (INT2 source select)
	// bit 2: IRSW (INT1 source select)
	// bit 1: PF (DMA3/SCU I/O select)
	// bit 0: PF (INTAK/SRDY/TOUT1 output select)

	LOG("OPCN_w %02x\n", data);
	m_OPCN = data & 0x0f;

	m_tout1_callback((data & 0x03) == 0x03 ? m_tout1 : 1);
	m_icu->ir1_w(BIT(data, 2) ? m_sint : m_intp1);
	m_icu->ir2_w(BIT(data, 3) ? m_tout1 : m_intp2);
}

void v50_base_device::tout1_w(int state)
{
	m_tout1 = state;
	if ((m_OPCN & 0x03) == 0x01)
		m_tout1_callback(state);
	if (BIT(m_OPCN, 3))
		m_icu->ir2_w(state);
}

void v50_base_device::sint_w(int state)
{
	m_sint = state;
	if (BIT(m_OPCN, 2))
		m_icu->ir1_w(state);
}

void v50_base_device::device_reset()
{
	nec_common_device::device_reset();

	m_OPCN = 0;
	m_tout1_callback(1);
}

void v50_base_device::device_start()
{
	nec_common_device::device_start();
	m_internal_io = &space(AS_INTERNAL_IO);

	set_irq_acknowledge_callback(*m_icu, FUNC(v5x_icu_device::inta_cb));

	m_scu->write_cts(0);

	save_item(NAME(m_OPCN));
	save_item(NAME(m_tout1));
	save_item(NAME(m_sint));
	save_item(NAME(m_intp1));
	save_item(NAME(m_intp2));
}

void v40_device::install_peripheral_io()
{
	// unmap everything in I/O space up to the fixed position registers (we avoid overwriting them, it isn't a valid config)
	space(AS_INTERNAL_IO).unmap_readwrite(0, INTERNAL_IO_ADDR_MASK);
	space(AS_INTERNAL_IO).install_readwrite_handler(0, INTERNAL_IO_ADDR_MASK,
		read8sm_delegate(*this, FUNC(v40_device::temp_io_byte_r)),
		write8sm_delegate(*this, FUNC(v40_device::temp_io_byte_w)));

	if (m_OPSEL & OPSEL_DS)
	{
		u16 const base = m_DULA & INTERNAL_IO_ADDR_MASK;

		space(AS_INTERNAL_IO).unmap_readwrite(base & ~0x0f, base | 0x0f);
		space(AS_INTERNAL_IO).install_readwrite_handler(base & ~0x0f, base | 0x0f,
			read8sm_delegate(*m_dmau, FUNC(v5x_dmau_device::read)),
			write8sm_delegate(*m_dmau, FUNC(v5x_dmau_device::write)));
	}

	if (m_OPSEL & OPSEL_IS)
	{
		u16 const base = m_IULA & INTERNAL_IO_ADDR_MASK;

		space(AS_INTERNAL_IO).unmap_readwrite(base & ~0x01, base | 0x01);
		space(AS_INTERNAL_IO).install_readwrite_handler(base & ~0x01, base | 0x01,
			read8sm_delegate(*m_icu, FUNC(v5x_icu_device::read)),
			write8sm_delegate(*m_icu, FUNC(v5x_icu_device::write)));
	}

	if (m_OPSEL & OPSEL_TS)
	{
		u16 const base = m_TULA & INTERNAL_IO_ADDR_MASK;

		space(AS_INTERNAL_IO).unmap_readwrite(base & ~0x03, base | 0x03);
		space(AS_INTERNAL_IO).install_readwrite_handler(base & ~0x03, base | 0x03,
			read8sm_delegate(*m_tcu, FUNC(pit8253_device::read)),
			write8sm_delegate(*m_tcu, FUNC(pit8253_device::write)));
	}

	if (m_OPSEL & OPSEL_SS)
	{
		u16 const base = m_SULA & INTERNAL_IO_ADDR_MASK;

		space(AS_INTERNAL_IO).unmap_readwrite(base & ~0x03, base | 0x03);
		space(AS_INTERNAL_IO).install_readwrite_handler(base & ~0x03, base | 0x03,
			read8sm_delegate(*m_scu, FUNC(v5x_scu_device::read)),
			write8sm_delegate(*m_scu, FUNC(v5x_scu_device::write)));
	}
}

void v50_device::install_peripheral_io()
{
	// unmap everything in I/O space up to the fixed position registers (we avoid overwriting them, it isn't a valid config)
	space(AS_INTERNAL_IO).unmap_readwrite(0, INTERNAL_IO_ADDR_MASK);
	space(AS_INTERNAL_IO).install_readwrite_handler(0, INTERNAL_IO_ADDR_MASK,
		read8sm_delegate(*this, FUNC(v50_device::temp_io_byte_r)),
		write8sm_delegate(*this, FUNC(v50_device::temp_io_byte_w)));

	if (m_OPSEL & OPSEL_DS)
	{
		u16 const base = m_DULA & INTERNAL_IO_ADDR_MASK;

		space(AS_INTERNAL_IO).unmap_readwrite(base & ~0x0f, base | 0x0f);
		space(AS_INTERNAL_IO).install_readwrite_handler(base & ~0x0f, base | 0x0f,
			read8sm_delegate(*m_dmau, FUNC(v5x_dmau_device::read)),
			write8sm_delegate(*m_dmau, FUNC(v5x_dmau_device::write)), 0xffff);
	}

	if (m_OPSEL & OPSEL_IS)
	{
		u16 const base = m_IULA & INTERNAL_IO_ADDR_MASK;

		space(AS_INTERNAL_IO).unmap_readwrite(base & ~0x03, base | 0x03);
		space(AS_INTERNAL_IO).install_readwrite_handler(base & ~0x03, base | 0x03,
			read8sm_delegate(*m_icu, FUNC(v5x_icu_device::read)),
			write8sm_delegate(*m_icu, FUNC(v5x_icu_device::write)), io_mask(base));
	}

	if (m_OPSEL & OPSEL_TS)
	{
		u16 const base = m_TULA & INTERNAL_IO_ADDR_MASK;

		space(AS_INTERNAL_IO).unmap_readwrite(base & ~0x07, base | 0x07);
		space(AS_INTERNAL_IO).install_readwrite_handler(base & ~0x07, base | 0x07,
			read8sm_delegate(*m_tcu, FUNC(pit8253_device::read)),
			write8sm_delegate(*m_tcu, FUNC(pit8253_device::write)), io_mask(base));
	}

	if (m_OPSEL & OPSEL_SS)
	{
		u16 const base = m_SULA & INTERNAL_IO_ADDR_MASK;

		space(AS_INTERNAL_IO).unmap_readwrite(base & ~0x07, base | 0x07);
		space(AS_INTERNAL_IO).install_readwrite_handler(base & ~0x07, base | 0x07,
			read8sm_delegate(*m_scu, FUNC(v5x_scu_device::read)),
			write8sm_delegate(*m_scu, FUNC(v5x_scu_device::write)), io_mask(base));
	}
}

void v50_base_device::internal_port_map(address_map &map)
{
	map(0xfff0, 0xfff0).rw(FUNC(v50_base_device::TCKS_r), FUNC(v50_base_device::TCKS_w));

	map(0xfff2, 0xfff2).w(FUNC(v50_base_device::RFC_w));

	map(0xfff4, 0xfff4).w(FUNC(v50_base_device::WMB0_w)); // actually WMB on V50
	map(0xfff5, 0xfff5).w(FUNC(v50_base_device::WCY1_w));
	map(0xfff6, 0xfff6).w(FUNC(v50_base_device::WCY2_w));

	map(0xfff8, 0xfff8).rw(FUNC(v50_base_device::SULA_r), FUNC(v50_base_device::SULA_w));
	map(0xfff9, 0xfff9).rw(FUNC(v50_base_device::TULA_r), FUNC(v50_base_device::TULA_w));
	map(0xfffa, 0xfffa).rw(FUNC(v50_base_device::IULA_r), FUNC(v50_base_device::IULA_w));
	map(0xfffb, 0xfffb).rw(FUNC(v50_base_device::DULA_r), FUNC(v50_base_device::DULA_w));
	map(0xfffc, 0xfffc).rw(FUNC(v50_base_device::OPHA_r), FUNC(v50_base_device::OPHA_w));
	map(0xfffd, 0xfffd).rw(FUNC(v50_base_device::OPSEL_r), FUNC(v50_base_device::OPSEL_w));
	map(0xfffe, 0xfffe).rw(FUNC(v50_base_device::OPCN_r), FUNC(v50_base_device::OPCN_w));
}

void v50_base_device::execute_set_input(int irqline, int state)
{
	switch (irqline)
	{
	case INPUT_LINE_IRQ1:
		m_intp1 = state;
		if (BIT(m_OPCN, 2))
			return;
		break;

	case INPUT_LINE_IRQ2:
		m_intp2 = state;
		if (BIT(m_OPCN, 3))
			return;
		break;
	}

	v5x_set_input(irqline, state);
}

void v50_base_device::device_add_mconfig(machine_config &config)
{
	v5x_add_mconfig(config);

	// Timer 0 is internally connected to INT0
	m_tcu->out_handler<0>().set(m_icu, FUNC(pic8259_device::ir0_w));

	// Timer 1 is internally connected to RxC/TxC
	m_tcu->out_handler<1>().set(m_scu, FUNC(v5x_scu_device::write_rxc));
	m_tcu->out_handler<1>().append(m_scu, FUNC(v5x_scu_device::write_txc));
	m_tcu->out_handler<1>().append(FUNC(v50_base_device::tout1_w));

	m_scu->sint_handler().set(FUNC(v50_base_device::sint_w));
}

device_memory_interface::space_config_vector v50_base_device::memory_space_config() const
{
	space_config_vector spaces = {
			std::make_pair(AS_PROGRAM,     &m_program_config),
			std::make_pair(AS_IO,          &m_io_config),
			std::make_pair(AS_INTERNAL_IO, &m_internal_io_config)
		};
	return spaces;
}

v50_base_device::v50_base_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, u32 clock, bool is_16bit, u8 prefetch_size, u8 prefetch_cycles, u32 chip_type)
	: nec_common_device(mconfig, type, tag, owner, clock, is_16bit, prefetch_size, prefetch_cycles, chip_type, false, address_map_constructor(FUNC(v50_base_device::internal_port_map), this))
	, device_v5x_interface(mconfig, *this, clock, is_16bit)
	, m_tout1_callback(*this)
	, m_icu_slave_ack(*this, 0)
	, m_OPCN(0)
	, m_tout1(false)
	, m_intp1(false)
	, m_intp2(false)
{
}

v40_device::v40_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: v50_base_device(mconfig, V40, tag, owner, clock, false, 4, 4, V20_TYPE)
{
}

v50_device::v50_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: v50_base_device(mconfig, V50, tag, owner, clock, true, 6, 2, V30_TYPE)
{
}

void v53_device::tout1_w(int state)
{
	if (!BIT(m_SCTL, 4))
	{
		m_scu->write_rxc(state);
		m_scu->write_txc(state);
	}

	m_tout1_w(state);
}

void v53_device::sint_w(int state)
{
	m_sint_w(state);
}

u8 v53_device::io_read_byte(offs_t a)
{
	if (check_OPHA(a))
		return device_v5x_interface::internal_io_read_byte(a);
	else
		return nec_common_device::io_read_byte(a);
}

u16 v53_device::io_read_word(offs_t a)
{
	if (check_OPHA(a))
	{
		if ((a & INTERNAL_IO_ADDR_MASK) == INTERNAL_IO_ADDR_MASK)
		{
			return (device_v5x_interface::internal_io_read_byte(a) & 0x00ff)
				| ((nec_common_device::io_read_byte(a + 1) << 8) & 0xff00);
		}
		else
			return device_v5x_interface::internal_io_read_word(a);
	}
	else
		return nec_common_device::io_read_word(a);
}

void v53_device::io_write_byte(offs_t a, u8 v)
{
	if (check_OPHA(a))
	{
		device_v5x_interface::internal_io_write_byte(a, v);
	}
	else
		nec_common_device::io_write_byte(a, v);
}

void v53_device::io_write_word(offs_t a, u16 v)
{
	if (check_OPHA(a))
	{
		if ((a & INTERNAL_IO_ADDR_MASK) == INTERNAL_IO_ADDR_MASK)
		{
			device_v5x_interface::internal_io_write_byte(a, v & 0xff);
			nec_common_device::io_write_byte(a + 1, (v >> 8) & 0xff);
		}
		else
		{
			device_v5x_interface::internal_io_write_word(a, v);
		}
	}
	else
		nec_common_device::io_write_word(a, v);
}


u8 v53_device::SCTL_r()
{
	return m_SCTL;
}

void v53_device::SCTL_w(u8 data)
{
	// bit 7: unused
	// bit 6: unused
	// bit 5: unused
	// bit 4: SCU input clock source
	// bit 3: uPD71037 DMA mode - Carry A20
	// bit 2: uPD71037 DMA mode - Carry A16
	// bit 1: uPD71037 DMA mode enable (otherwise in uPD71071 mode)
	// bit 0: Onboard pripheral I/O maps to 8-bit boundaries? (otherwise 16-bit)

	LOG("SCTL_w %02x\n", data);
	m_SCTL = data & 0x1f;
	m_brc_enable = BIT(data, 4);
	install_peripheral_io();

	if (m_brc_enable)
	{
		brc_update();
	}
	else
	{
		m_brc_timer->adjust(attotime::never, 0, attotime::never);
	}
}

void v53_device::device_reset()
{
	v33_base_device::device_reset();

	m_SCTL = 0x00;
}

void v53_device::device_start()
{
	v33_base_device::device_start();
	m_internal_io = &space(AS_INTERNAL_IO);

	set_irq_acknowledge_callback(*m_icu, FUNC(v5x_icu_device::inta_cb));

	save_item(NAME(m_SCTL));
}

void v53_device::install_peripheral_io()
{
	// unmap everything in I/O space up to the fixed position registers (we avoid overwriting them, it isn't a valid config)
	space(AS_INTERNAL_IO).unmap_readwrite(0, INTERNAL_IO_ADDR_MASK);
	space(AS_INTERNAL_IO).install_readwrite_handler(0, INTERNAL_IO_ADDR_MASK,
		read8sm_delegate(*this, FUNC(v53_device::temp_io_byte_r)),
		write8sm_delegate(*this, FUNC(v53_device::temp_io_byte_w)));

	// IOAG determines if the handlers used 8-bit or 16-bit access
	// the hng64.cpp games first set everything up in 8-bit mode, then
	// do the procedure again in 16-bit mode before using them?!

	bool const IOAG = m_SCTL & 1;

	if (m_OPSEL & OPSEL_DS)
	{
		u16 const base = m_DULA & INTERNAL_IO_ADDR_MASK;

		if (m_SCTL & 0x02) // uPD71037 mode
		{
			if (IOAG) // 8-bit
			{
				space(AS_INTERNAL_IO).unmap_readwrite(base & ~0x0f, base | 0x0f);
			}
			else
			{
				space(AS_INTERNAL_IO).unmap_readwrite(base & ~0x1f, base | 0x1f);
			}
		}
		else // uPD71071 mode
		{
			space(AS_INTERNAL_IO).unmap_readwrite(base & ~0x0f, base | 0x0f);
			space(AS_INTERNAL_IO).install_readwrite_handler(base & ~0x0f, base | 0x0f,
				read8sm_delegate(*m_dmau, FUNC(v5x_dmau_device::read)),
				write8sm_delegate(*m_dmau, FUNC(v5x_dmau_device::write)), 0xffff);
		}
	}

	if (m_OPSEL & OPSEL_IS)
	{
		u16 const base = m_IULA & INTERNAL_IO_ADDR_MASK;

		if (IOAG) // 8-bit
		{
			space(AS_INTERNAL_IO).unmap_readwrite(base & ~0x01, base | 0x01);
			space(AS_INTERNAL_IO).install_readwrite_handler(base & ~0x01, base | 0x01,
				read8sm_delegate(*m_icu, FUNC(v5x_icu_device::read)),
				write8sm_delegate(*m_icu, FUNC(v5x_icu_device::write)), 0xffff);
		}
		else
		{
			space(AS_INTERNAL_IO).unmap_readwrite(base & ~0x03, base | 0x03);
			space(AS_INTERNAL_IO).install_readwrite_handler(base & ~0x03, base | 0x03,
				read8sm_delegate(*m_icu, FUNC(v5x_icu_device::read)),
				write8sm_delegate(*m_icu, FUNC(v5x_icu_device::write)), io_mask(base));
		}
	}

	if (m_OPSEL & OPSEL_TS)
	{
		u16 const base = m_TULA & INTERNAL_IO_ADDR_MASK;

		if (IOAG) // 8-bit
		{
			space(AS_INTERNAL_IO).unmap_readwrite(base & ~0x03, base | 0x03);
			space(AS_INTERNAL_IO).install_readwrite_handler(base & ~0x03, base | 0x03,
				read8sm_delegate(*m_tcu, FUNC(pit8253_device::read)),
				write8sm_delegate(*m_tcu, FUNC(pit8253_device::write)), 0xffff);
		}
		else
		{
			space(AS_INTERNAL_IO).unmap_readwrite(base & ~0x07, base | 0x07);
			space(AS_INTERNAL_IO).install_readwrite_handler(base & ~0x07, base | 0x07,
				read8sm_delegate(*m_tcu, FUNC(pit8253_device::read)),
				write8sm_delegate(*m_tcu, FUNC(pit8253_device::write)), io_mask(base));
		}
	}

	if (m_OPSEL & OPSEL_SS)
	{
		u16 const base = m_SULA & INTERNAL_IO_ADDR_MASK;

		if (IOAG) // 8-bit
		{
			space(AS_INTERNAL_IO).unmap_readwrite(base & ~0x03, base | 0x03);
			space(AS_INTERNAL_IO).install_readwrite_handler(base & ~0x03, base | 0x03,
				read8sm_delegate(*m_scu, FUNC(v5x_scu_device::read)),
				write8sm_delegate(*m_scu, FUNC(v5x_scu_device::write)), 0xffff);
		}
		else
		{
			space(AS_INTERNAL_IO).unmap_readwrite(base & ~0x07, base | 0x07);
			space(AS_INTERNAL_IO).install_readwrite_handler(base & ~0x07, base | 0x07,
				read8sm_delegate(*m_scu, FUNC(v5x_scu_device::read)),
				write8sm_delegate(*m_scu, FUNC(v5x_scu_device::write)), io_mask(base));
		}
	}
}

void v53_device::hack_w(int state)
{
	if (!(m_SCTL & 0x02))
		m_dmau->hack_w(state);
	else
		LOG("hack_w not in 71071mode\n");
}

void v53_device::internal_port_map(address_map &map)
{
	v33_internal_port_map(map);

	map(0xffe0, 0xffe0).w(FUNC(v53_device::BSEL_w));  // uPD71037 DMA mode bank selection register
	map(0xffe1, 0xffe1).w(FUNC(v53_device::BADR_w));  // uPD71037 DMA mode bank register peripheral mapping (also uses OPHA)
	// 0xffe2-0xffe9 reserved
	map(0xffe9, 0xffe9).w(FUNC(v53_device::BRC_w));   // baud rate counter (used for serial peripheral)
	map(0xffea, 0xffea).w(FUNC(v53_device::WMB0_w));  // waitstate control
	map(0xffeb, 0xffeb).w(FUNC(v53_device::WCY1_w));  // waitstate control
	map(0xffec, 0xffec).w(FUNC(v53_device::WCY0_w));  // waitstate control
	map(0xffed, 0xffed).w(FUNC(v53_device::WAC_w));   // waitstate control
	// 0xffee-0xffef reserved
	map(0xfff0, 0xfff0).rw(FUNC(v53_device::TCKS_r), FUNC(v53_device::TCKS_w));  // timer clocks
	map(0xfff1, 0xfff1).w(FUNC(v53_device::SBCR_w));  // internal clock divider, halt behavior etc.
	map(0xfff2, 0xfff2).w(FUNC(v53_device::RFC_w));   // ram refresh control
	map(0xfff3, 0xfff3).w(FUNC(v53_device::WMB1_w));  // waitstate control
	map(0xfff4, 0xfff4).w(FUNC(v53_device::WCY2_w));  // waitstate control
	map(0xfff5, 0xfff5).w(FUNC(v53_device::WCY3_w));  // waitstate control
	map(0xfff6, 0xfff6).w(FUNC(v53_device::WCY4_w));  // waitstate control
	// 0xfff6 reserved
	map(0xfff8, 0xfff8).rw(FUNC(v53_device::SULA_r), FUNC(v53_device::SULA_w));  // scu mapping
	map(0xfff9, 0xfff9).rw(FUNC(v53_device::TULA_r), FUNC(v53_device::TULA_w));  // tcu mapping
	map(0xfffa, 0xfffa).rw(FUNC(v53_device::IULA_r), FUNC(v53_device::IULA_w));  // icu mapping
	map(0xfffb, 0xfffb).rw(FUNC(v53_device::DULA_r), FUNC(v53_device::DULA_w));  // dmau mapping
	map(0xfffc, 0xfffc).rw(FUNC(v53_device::OPHA_r), FUNC(v53_device::OPHA_w));  // peripheral mapping (upper bits, common)
	map(0xfffd, 0xfffd).rw(FUNC(v53_device::OPSEL_r), FUNC(v53_device::OPSEL_w)); // peripheral enabling
	map(0xfffe, 0xfffe).rw(FUNC(v53_device::SCTL_r), FUNC(v53_device::SCTL_w));  // peripheral configuration (& byte / word mapping)
	// 0xffff reserved
}

void v53_device::execute_set_input(int irqline, int state)
{
	v5x_set_input(irqline, state);
}

void v53_device::device_add_mconfig(machine_config &config)
{
	v5x_add_mconfig(config);

	m_tcu->out_handler<1>().set(FUNC(v53_device::tout1_w));
	m_scu->sint_handler().set(FUNC(v53_device::sint_w));
}

device_memory_interface::space_config_vector v53_device::memory_space_config() const
{
	return space_config_vector {
		std::make_pair(AS_PROGRAM,     &m_program_config),
		std::make_pair(AS_IO,          &m_io_config),
		std::make_pair(AS_INTERNAL_IO, &m_internal_io_config)
	};
}

v53_device::v53_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, u32 clock, bool v55_extensions)
	: v33_base_device(mconfig, type, tag, owner, clock, address_map_constructor(FUNC(v53_device::internal_port_map), this), v55_extensions)
	, device_v5x_interface(mconfig, *this, clock, true)
	, m_sint_w(*this)
	, m_tout1_w(*this)
{
}

v53_device::v53_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: v53_device(mconfig, V53, tag, owner, clock, false)
{
}

v53a_device::v53a_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: v53_device(mconfig, V53A, tag, owner, clock, false)
{
}

v55_device::v55_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: v53_device(mconfig, V55, tag, owner, clock, true)
	, m_sfr_config("sfr", ENDIANNESS_LITTLE, 16, 9, 0, address_map_constructor(FUNC(v55_device::sfr_map), this))
	, m_sfr{}
	, m_adc_in_cb(*this, 0x00)
	, m_port_in_cb(*this, 0xff)
	, m_port_out_cb(*this)
	, m_txd0_handler(*this)
	, m_txd1_handler(*this)
	, m_adc_fint_cb(*this)
	, m_timer{}
	, m_adc_timer(nullptr)
	, m_uart0_tx_timer(nullptr)
	, m_uart0_rx_timer(nullptr)
	, m_uart1_tx_timer(nullptr)
	, m_uart1_rx_timer(nullptr)
	, m_adc_running(false)
	, m_rxd0(1)
	, m_cts0(0)
	, m_uart0_txd_state(1)
	, m_uart0_tx_byte(0)
	, m_uart0_tx_bit(0)
	, m_uart0_rx_byte(0)
	, m_uart0_rx_bit(0)
	, m_uart0_rx_prev(1)
	, m_uart0_tx_active(false)
	, m_uart0_tx_loaded(false)
	, m_uart0_rx_active(false)
	, m_uart0_rx_full(false)
	, m_rxd1(1)
	, m_cts1(0)
	, m_uart1_txd_state(1)
	, m_uart1_tx_byte(0)
	, m_uart1_tx_bit(0)
	, m_uart1_rx_byte(0)
	, m_uart1_rx_bit(0)
	, m_uart1_rx_prev(1)
	, m_uart1_tx_active(false)
	, m_uart1_tx_loaded(false)
	, m_uart1_rx_active(false)
	, m_uart1_rx_full(false)
	, m_timer_irq_bank{}
	, m_timer_irq_pending{}
	, m_timer_irq_in_service{}
	, m_adc_irq_pending(false)
	, m_adc_irq_in_service(false)
	, m_internal_serial_irq_pending(0)
	, m_internal_serial_irq_in_service(0)
	, m_special_irq_stack{}
	, m_special_irq_stack_depth(0)
	, m_serial_irq_mode(serial_irq_mode::off)
	, m_timer_irq_experiment(true)
	, m_adc_irq_experiment(true)
	, m_adc_irq_bank(0xff)
{
	m_timer_irq_bank.fill(0xff);
	m_timer.fill(nullptr);
}

u8 v55_device::internal_serial_irq_ic(serial_irq_source source) const
{
	return m_sfr[0x0da + unsigned(source)];
}

v55_device::serial_irq_mode v55_device::current_serial_irq_mode() const
{
	return m_serial_irq_mode;
}

bool v55_device::tx0_late_irq_window() const
{
	// Prophecy-specific experiment: only allow INTST0 while the main CPU is in
	// the final B1271/B127B completion wait corridor. Broad TX0 service
	// destabilizes earlier boot, so this isolates the suspected missing late
	// finalization path without changing the rest of transport setup.
	const u32 curpc = pc();
	return (curpc >= 0x0b1271) && (curpc <= 0x0b1285);
}

bool v55_device::internal_serial_irq_enabled(serial_irq_source source) const
{
	const serial_irq_mode mode = current_serial_irq_mode();
	if (mode == serial_irq_mode::off)
		return false;

	const u8 ic = internal_serial_irq_ic(source);
	if ((ic == 0x00) || BIT(ic, 6))
		return false;

	if (mode == serial_irq_mode::tx0_only)
	{
		// Narrow experiment: only the channel-0 TX bankswitched worker
		// (`IC30 -> bank 7`) is allowed through.
		return (source == SERIAL_IRQ_INTST0) && BIT(ic, 4);
	}

	if (mode == serial_irq_mode::rx0_only)
	{
		// Middle-ground experiment: only the observed Prophecy channel-0 RX
		// bankswitched parser path (`IC28 -> bank 8`) is allowed through. This
		// is intentionally narrower than the broader V25/V35-inspired model.
		return (source == SERIAL_IRQ_INTSR0) && BIT(ic, 4);
	}

	if (mode == serial_irq_mode::rx0_rx1)
	{
		// Narrow default for Prophecy: keep the proven channel-0 RX parser
		// (`IC28 -> bank 8`) and the channel-1 bankswitched paths used by the
		// MIDI/SysEx transport. RX1 (`IC29 -> bank 11`) is already proven; the
		// matching TX-ready worker (`IC31 -> bank 10`) is the lowest-risk next
		// addition before opening the broader serial source set.
		return ((source == SERIAL_IRQ_INTSR0) || (source == SERIAL_IRQ_INTSR1) || (source == SERIAL_IRQ_INTST1)) && BIT(ic, 4);
	}

	if (mode == serial_irq_mode::rx0_late_tx0)
	{
		if ((source == SERIAL_IRQ_INTSR0) && BIT(ic, 4))
			return true;

		if ((source == SERIAL_IRQ_INTST0) && BIT(ic, 4))
			return tx0_late_irq_window();

		return false;
	}

	if (mode == serial_irq_mode::rx0_tx0)
	{
		// Prophecy's board-link RX/TX workers plus the channel-1 MIDI/SysEx pair.
		return ((source == SERIAL_IRQ_INTSR0) || (source == SERIAL_IRQ_INTST0) ||
			(source == SERIAL_IRQ_INTSR1) || (source == SERIAL_IRQ_INTST1)) && BIT(ic, 4);
	}

	return true;
}

bool v55_device::internal_serial_irq_bankswitch(serial_irq_source source) const
{
	return BIT(internal_serial_irq_ic(source), 4);
}

u16 v55_device::interrupt_vector_address(u8 vector) const
{
	const u16 reloc = (vector >= 8 && vector <= 47) ? (u16(m_sfr[0x0c5] & 0x03) << 8) : 0;
	return reloc + (u16(vector) << 2);
}

u8 v55_device::interrupt_vector_bank(u8 vector)
{
	// V55PI hardware manual: register-bank interrupt response selects the
	// target bank from the low four bits of the source's vector-table entry.
	return u8(mem_read_word(interrupt_vector_address(vector)) & 0x000f);
}

int v55_device::timer_irq_index_from_source(int source) const
{
	const int timer = source - SERIAL_IRQ_COUNT;
	return (timer >= 0 && timer < TIMER_IRQ_COUNT) ? timer : -1;
}

u8 v55_device::special_irq_vector(int source) const
{
	if (source >= 0 && source < SERIAL_IRQ_COUNT)
		return u8(26 + source);
	const int timer = timer_irq_index_from_source(source);
	if (timer >= 0)
		return timer_irq_vector(timer_irq_source(timer));
	if (source == SPECIAL_IRQ_ADC)
		return 37;

	return 0xff;
}

u8 v55_device::special_irq_priority(int source) const
{
	if (source >= 0 && source < SERIAL_IRQ_COUNT)
		return internal_serial_irq_ic(serial_irq_source(source)) & 0x03;
	const int timer = timer_irq_index_from_source(source);
	if (timer >= 0)
		return timer_irq_priority(timer_irq_source(timer));
	if (source == SPECIAL_IRQ_ADC)
		return adc_irq_priority();

	return 0xff;
}

bool v55_device::priority_is_tracked(u8 priority) const
{
	return (priority < 3) || BIT(m_sfr[0x0c5], 7);
}

bool v55_device::can_accept_priority(u8 priority) const
{
	if (priority > 3)
		return false;

	const u8 highest_blocking = ((priority == 3) && !priority_is_tracked(priority)) ? 2 : priority;
	for (u8 level = 0; level <= highest_blocking; level++)
	{
		if (BIT(m_sfr[0x0c4], level))
			return false;
	}

	return true;
}

void v55_device::mark_special_irq_in_service(int source)
{
	if (m_special_irq_stack_depth < m_special_irq_stack.size())
		m_special_irq_stack[m_special_irq_stack_depth++] = u8(source);

	const u8 priority = special_irq_priority(source);
	if (priority_is_tracked(priority))
		m_sfr[0x0c4] |= u8(1U << priority);
}

int v55_device::current_special_irq_stack_source() const
{
	if (m_special_irq_stack_depth == 0)
		return -1;

	return m_special_irq_stack[m_special_irq_stack_depth - 1];
}

int v55_device::pop_special_irq_stack_source()
{
	if (m_special_irq_stack_depth == 0)
		return -1;

	const int source = m_special_irq_stack[--m_special_irq_stack_depth];
	m_special_irq_stack[m_special_irq_stack_depth] = 0xff;

	for (u8 level = 0; level < 4; level++)
	{
		if (BIT(m_sfr[0x0c4], level))
		{
			m_sfr[0x0c4] &= ~u8(1U << level);
			break;
		}
	}

	return source;
}

void v55_device::clear_special_irq_source(int source)
{
	const int timer = timer_irq_index_from_source(source);
	if (timer >= 0)
	{
		m_timer_irq_in_service[timer] = false;
		m_sfr[timer_irq_ic(timer_irq_source(timer))] &= ~u8(0x80);
		return;
	}

	if (source == SPECIAL_IRQ_ADC)
	{
		m_adc_irq_in_service = false;
		m_sfr[0x0e5] &= ~u8(0x80);
		m_adc_fint_cb(1);
		return;
	}

	if (source >= 0 && source < SERIAL_IRQ_COUNT)
	{
		m_internal_serial_irq_in_service &= ~u8(1U << unsigned(source));
		m_sfr[0x0da + unsigned(source)] &= ~u8(0x80);
	}
}

u8 v55_device::internal_serial_irq_bank(serial_irq_source source)
{
	switch (source)
	{
	case SERIAL_IRQ_INTSR0:
	case SERIAL_IRQ_INTST0:
	case SERIAL_IRQ_INTSR1:
	case SERIAL_IRQ_INTST1:
	case SERIAL_IRQ_INTSER0:
	case SERIAL_IRQ_INTSER1:
		return interrupt_vector_bank(26 + unsigned(source));
	default:
		break;
	}

	return 0xff;
}

int v55_device::select_internal_serial_irq() const
{
	int best = -1;
	u8 best_pri = 0xff;

	for (unsigned source = 0; source < SERIAL_IRQ_COUNT; source++)
	{
		if (!BIT(m_internal_serial_irq_pending, source))
			continue;
		if (BIT(m_internal_serial_irq_in_service, source))
			continue;
		if (!internal_serial_irq_enabled(serial_irq_source(source)))
			continue;

		const u8 pri = internal_serial_irq_ic(serial_irq_source(source)) & 0x03;
		if (!can_accept_priority(pri))
			continue;
		if ((best < 0) || (pri < best_pri))
		{
			best = int(source);
			best_pri = pri;
		}
	}

	return best;
}

int v55_device::current_internal_serial_irq_source() const
{
	for (unsigned source = 0; source < SERIAL_IRQ_COUNT; source++)
	{
		if (BIT(m_internal_serial_irq_in_service, source))
			return int(source);
	}

	return -1;
}

int v55_device::select_internal_special_irq() const
{
	int best_source = -1;
	u8 best_priority = 0xff;

	for (unsigned timer = 0; timer < TIMER_IRQ_COUNT; timer++)
	{
		if (!m_timer_irq_pending[timer] || m_timer_irq_in_service[timer] || !timer_irq_enabled(timer_irq_source(timer)))
			continue;

		const u8 timer_priority = timer_irq_priority(timer_irq_source(timer));
		if (can_accept_priority(timer_priority) && ((best_source < 0) || (timer_priority < best_priority)))
		{
			best_source = SERIAL_IRQ_COUNT + int(timer);
			best_priority = timer_priority;
		}
	}

	if (m_adc_irq_pending && !m_adc_irq_in_service && adc_irq_enabled())
	{
		const int adc_source = SPECIAL_IRQ_ADC;
		const u8 adc_priority = adc_irq_priority();
		if (can_accept_priority(adc_priority) && ((best_source < 0) || (adc_priority < best_priority)))
		{
			best_source = adc_source;
			best_priority = adc_priority;
		}
	}

	const int serial_source = select_internal_serial_irq();
	if (serial_source >= 0)
	{
		const u8 serial_priority = internal_serial_irq_ic(serial_irq_source(serial_source)) & 0x03;
		if ((best_source < 0) || (serial_priority < best_priority))
		{
			best_source = serial_source;
			best_priority = serial_priority;
		}
	}

	return best_source;
}

int v55_device::current_internal_special_irq_source() const
{
	const int stack_source = current_special_irq_stack_source();
	if (stack_source >= 0)
		return stack_source;

	return current_internal_serial_irq_source();
}

void v55_device::update_internal_serial_irq_line()
{
	if (current_serial_irq_mode() == serial_irq_mode::off)
		return;

	// INTST0 is a latched UART event, raised when TxB0 transfers to the shift
	// register, when transmission is enabled with TxB0 empty, or on All Sent.

	// Channel 1 uses the same Prophecy-side "ready while idle" transport style
	// for MIDI/SysEx traffic. The bank-10 worker is expected to keep draining
	// the queued-byte ring while UARTS1 bit 5 stays high and no byte is in
	// flight.
	if (internal_serial_irq_enabled(SERIAL_IRQ_INTST1))
	{
		const u8 mask = u8(1U << unsigned(SERIAL_IRQ_INTST1));
		const bool active = BIT(m_sfr[0x17b], 7) && BIT(m_sfr[0x17c], 5) && !m_uart1_tx_active &&
			!BIT(m_internal_serial_irq_in_service, unsigned(SERIAL_IRQ_INTST1));
		const bool pending = BIT(m_internal_serial_irq_pending, unsigned(SERIAL_IRQ_INTST1));

		if (active && !pending)
		{
			m_internal_serial_irq_pending |= mask;
			m_sfr[0x0df] |= 0x80;
		}
		else if (!active && pending)
		{
			m_internal_serial_irq_pending &= ~mask;
			if (!BIT(m_internal_serial_irq_in_service, unsigned(SERIAL_IRQ_INTST1)))
				m_sfr[0x0df] &= ~u8(0x80);
		}
	}

	set_int_line((select_internal_special_irq() >= 0) ? ASSERT_LINE : CLEAR_LINE);
}

void v55_device::request_internal_serial_irq(serial_irq_source source)
{
	if (!internal_serial_irq_enabled(source))
		return;

	m_internal_serial_irq_pending |= u8(1U << unsigned(source));
	m_sfr[0x0da + unsigned(source)] |= 0x80;
	update_internal_serial_irq_line();
}

bool v55_device::handle_special_int_ack()
{
	const int source = select_internal_special_irq();
	if (source < 0)
		return false;

	const int timer = timer_irq_index_from_source(source);
	if (timer >= 0)
	{
		const auto timer_source = timer_irq_source(timer);
		if (!timer_irq_bankswitch(timer_source))
			return false;

		const u8 vector = special_irq_vector(source);
		const u16 vector_addr = interrupt_vector_address(vector);
		const u16 vector_word = mem_read_word(vector_addr);
		const u8 override_bank = m_timer_irq_bank[timer];
		const u8 bank = (override_bank == 0xff) ? u8(vector_word & 0x000f) : override_bank;
		const u8 ic = timer_irq_ic(timer_source);
		m_timer_irq_pending[timer] = false;
		m_timer_irq_in_service[timer] = true;
		mark_special_irq_in_service(source);
		m_sfr[ic] &= ~u8(0x80);
		update_internal_serial_irq_line();
		v55_interrupt_bankswitch(bank);
		return true;
	}

	if (source == SPECIAL_IRQ_ADC)
	{
		if (!adc_irq_bankswitch())
			return false;

		const u8 bank = adc_irq_bank();
		m_adc_irq_pending = false;
		m_adc_irq_in_service = true;
		mark_special_irq_in_service(source);
		m_sfr[0x0e5] &= ~u8(0x80);
		update_internal_serial_irq_line();
		v55_interrupt_bankswitch(bank);
		return true;
	}

	const auto serial_source = serial_irq_source(source);
	if (!internal_serial_irq_bankswitch(serial_source))
		return false;

	const u8 bank = internal_serial_irq_bank(serial_source);
	m_internal_serial_irq_pending &= ~u8(1U << source);
	m_internal_serial_irq_in_service |= u8(1U << source);
	mark_special_irq_in_service(source);
	v55_interrupt_bankswitch(bank);
	update_internal_serial_irq_line();
	return true;
}

void v55_device::v55_fint()
{
	nec_common_device::v55_fint();

	const int source = current_internal_special_irq_source();
	if (source < 0)
	{
		update_internal_serial_irq_line();
		return;
	}

	const int timer = timer_irq_index_from_source(source);
	if (timer >= 0)
	{
		const int popped = (current_special_irq_stack_source() >= 0) ? pop_special_irq_stack_source() : source;
		clear_special_irq_source(popped);
		update_internal_serial_irq_line();
		return;
	}

	if (source == SPECIAL_IRQ_ADC)
	{
		const int popped = (current_special_irq_stack_source() >= 0) ? pop_special_irq_stack_source() : source;
		clear_special_irq_source(popped);
		update_internal_serial_irq_line();
		return;
	}

	const int popped = (current_special_irq_stack_source() >= 0) ? pop_special_irq_stack_source() : source;
	clear_special_irq_source(popped);
	if (popped >= 0 && popped < SERIAL_IRQ_COUNT)
	{
		update_internal_serial_irq_line();
	}
}

void v55_device::sfr_map(address_map &map)
{
	map(0x000, 0x1ef).rw(FUNC(v55_device::sfr_r), FUNC(v55_device::sfr_w));
}

u8 v55_device::port_r(unsigned port)
{
	const unsigned offset = 0x100 + port;
	if (port == 2)
	{
		const u8 latch = m_sfr[offset];
		const u8 pins = !m_port_in_cb[port].isunset() ? m_port_in_cb[port]() : 0xff;
		const u8 pm2 = m_sfr[0x112];
		const u8 pmc2 = m_sfr[0x122];
		const bool prdc_pin_mode = BIT(m_sfr[0x10c], 0);
		u8 data = 0;

		for (unsigned bit = 0; bit < 8; bit++)
		{
			const u8 mask = u8(1U << bit);
			if (BIT(pmc2, bit))
			{
				// Control-function readback is not fully modeled yet.  Use the
				// board-visible pin callback for now rather than the port latch.
				data |= pins & mask;
			}
			else if (BIT(pm2, bit))
			{
				// Input port: PRDC=0 reads pins; PRDC=1 reads zero.
				if (!prdc_pin_mode)
					data |= pins & mask;
			}
			else
			{
				// Output port: PRDC=0 reads latch; PRDC=1 reads pins.
				data |= (prdc_pin_mode ? pins : latch) & mask;
			}
		}

		return data;
	}
	if (!m_port_in_cb[port].isunset())
		m_sfr[offset] = m_port_in_cb[port]();
	return m_sfr[offset];
}

void v55_device::port_w(unsigned port, u8 data)
{
	const unsigned offset = 0x100 + port;
	m_sfr[offset] = data;
	if (port == 2)
		update_port2_output();
	else if (!m_port_out_cb[port].isunset())
		m_port_out_cb[port](data);
}

bool v55_device::port2_output_enabled() const
{
	// P2 is a six-bit port.  In port mode (PMC2 bit clear), PM2 bit clear
	// selects output.  Control-function outputs are not driven through the
	// generic port callback.
	return ((~m_sfr[0x122] & ~m_sfr[0x112] & 0x3f) != 0);
}

void v55_device::update_port2_output()
{
	if (!m_port_out_cb[2].isunset() && port2_output_enabled())
		m_port_out_cb[2](m_sfr[0x102]);
}

u8 v55_device::timer_irq_ic(timer_irq_source source) const
{
	return (source == TIMER_IRQ_INTCM21) ? 0x0d4 : 0x0d5;
}

u8 v55_device::timer_irq_vector(timer_irq_source source) const
{
	return (source == TIMER_IRQ_INTCM21) ? 20 : 21;
}

u16 v55_device::timer_compare(timer_irq_source source) const
{
	const u16 offset = (source == TIMER_IRQ_INTCM21) ? 0x15a : 0x166;
	return u16(m_sfr[offset] | (u16(m_sfr[offset + 1]) << 8));
}

u8 v55_device::timer_control_enable_bit(timer_irq_source source) const
{
	return (source == TIMER_IRQ_INTCM21) ? 3 : 7;
}

u8 v55_device::timer_control_prescale_bit(timer_irq_source source) const
{
	return (source == TIMER_IRQ_INTCM21) ? 0 : 4;
}

attotime v55_device::timer_tick_period(timer_irq_source source) const
{
	const u8 tmc1 = m_sfr[0x131];
	const u8 ce_bit = timer_control_enable_bit(source);
	if (!BIT(tmc1, ce_bit))
		return attotime::never;

	// V55PI hardware manual: TMC1 controls TM2/TM3, CE starts counting, PRM
	// selects phi/8 or phi/32, and compare interval uses the compare value + 1.
	const u8 prm_bit = timer_control_prescale_bit(source);
	const double divider = BIT(tmc1, prm_bit) ? 32.0 : 8.0;
	const double tick_hz = double(clock()) / divider;
	return attotime::from_hz(tick_hz / double(timer_compare(source) + 1));
}

void v55_device::update_timer(timer_irq_source source)
{
	const unsigned timer = unsigned(source);
	if (m_timer[timer] == nullptr)
		return;

	const attotime period = timer_tick_period(source);
	if (period.is_never())
		m_timer[timer]->adjust(attotime::never);
	else
		m_timer[timer]->adjust(period, int(source), period);
}

bool v55_device::timer_irq_experiment_enabled() const
{
	return m_timer_irq_experiment;
}

bool v55_device::timer_irq_enabled(timer_irq_source source) const
{
	if (!timer_irq_experiment_enabled())
		return false;

	return !BIT(m_sfr[timer_irq_ic(source)], 6);
}

bool v55_device::timer_irq_bankswitch(timer_irq_source source) const
{
	return BIT(m_sfr[timer_irq_ic(source)], 4);
}

u8 v55_device::timer_irq_priority(timer_irq_source source) const
{
	return m_sfr[timer_irq_ic(source)] & 0x03;
}

bool v55_device::adc_irq_experiment_enabled() const
{
	return m_adc_irq_experiment;
}

bool v55_device::adc_irq_enabled() const
{
	return adc_irq_experiment_enabled() && !BIT(m_sfr[0x0e5], 6);
}

bool v55_device::adc_irq_bankswitch() const
{
	return BIT(m_sfr[0x0e5], 4);
}

u8 v55_device::adc_irq_priority() const
{
	return m_sfr[0x0e5] & 0x03;
}

u8 v55_device::adc_irq_bank()
{
	return (m_adc_irq_bank == 0xff) ? interrupt_vector_bank(37) : m_adc_irq_bank;
}

void v55_device::request_timer_irq(timer_irq_source source)
{
	if (!timer_irq_experiment_enabled())
		return;

	const unsigned timer = unsigned(source);
	const u8 ic = timer_irq_ic(source);
	m_timer_irq_pending[timer] = true;
	m_sfr[ic] |= 0x80;
	update_internal_serial_irq_line();
}

void v55_device::request_adc_irq()
{
	const bool experiment = adc_irq_experiment_enabled();
	const bool masked = BIT(m_sfr[0x0e5], 6);
	if (!experiment || masked)
	{
		return;
	}
	if (m_adc_irq_pending)
	{
		return;
	}
	if (m_adc_irq_in_service)
	{
		return;
	}

	m_adc_irq_pending = true;
	m_sfr[0x0e5] |= 0x80;
	update_internal_serial_irq_line();
}

TIMER_CALLBACK_MEMBER(v55_device::timer_tick)
{
	const unsigned timer = unsigned(param);
	if (timer < TIMER_IRQ_COUNT && !timer_tick_period(timer_irq_source(timer)).is_never())
	{
		const auto source = timer_irq_source(timer);
		request_timer_irq(source);
	}
}

attotime v55_device::adc_tick_period() const
{
	// V55PI hardware manual (U10514EJ5V0UM00) Table 13-1: for an internal
	// system clock in the 8-16 MHz range, conversion takes 160 clock cycles.
	return attotime::from_ticks(160, clock());
}

void v55_device::update_adc_results()
{
	const u8 adm = m_sfr[0x020];
	if (!BIT(adm, 7))
		return;
	if (adc_irq_experiment_enabled() && (m_adc_irq_pending || m_adc_irq_in_service))
		return;

	const bool select_mode = BIT(adm, 0);
	const u8 channel = (adm >> 1) & 0x03;
	const bool scan_mode = !select_mode;
	if (scan_mode)
	{
		for (u8 ch = 0; ch <= channel; ch++)
			m_sfr[ch << 1] = m_adc_in_cb[ch]();
	}
	else
	{
		m_sfr[channel << 1] = m_adc_in_cb[channel]();
	}

	request_adc_irq();
}

void v55_device::update_adc_timer()
{
	const u8 adm = m_sfr[0x020];
	const bool enable = BIT(adm, 7);
	m_adc_running = enable;

	if (m_adc_timer == nullptr)
		return;

	if (enable)
	{
		m_adc_timer->adjust(adc_tick_period(), 0, adc_tick_period());
	}
	else
	{
		m_adc_timer->adjust(attotime::never);
	}
}

TIMER_CALLBACK_MEMBER(v55_device::adc_tick)
{
	update_adc_results();
}

u8 v55_device::sfr_r(offs_t offset)
{
	offset &= 0x1ff;

	switch (offset)
	{
	case 0x000:
	case 0x002:
	case 0x004:
	case 0x006:
		return m_sfr[offset];
	case 0x100: return port_r(0);
	case 0x101: return port_r(1);
	case 0x102: return port_r(2);
	case 0x103: return port_r(3);
	case 0x104: return port_r(4);
	case 0x105: return port_r(5);
	case 0x106: return port_r(6);
	case 0x107: return port_r(7);
	case 0x108: return port_r(8);
	case 0x174:
		update_uart0_status();
		return m_sfr[0x174];
	case 0x176:
		update_uart0_status();
		if (!machine().side_effects_disabled())
		{
			m_uart0_rx_full = false;
		}
		return m_sfr[0x176];
	case 0x17c:
		update_uart1_status();
		return m_sfr[0x17c];
	case 0x17e:
		update_uart1_status();
		if (!machine().side_effects_disabled())
			m_uart1_rx_full = false;
		return m_sfr[0x17e];
	}

	return m_sfr[offset];
}

void v55_device::sfr_w(offs_t offset, u8 data)
{
	offset &= 0x1ff;

	const u8 old = m_sfr[offset];
	m_sfr[offset] = data;
	if ((offset >= 0x0da) && (offset <= 0x0df) && !BIT(data, 7))
	{
		const u8 mask = u8(1U << (offset - 0x0da));
		m_internal_serial_irq_pending &= ~mask;
		m_internal_serial_irq_in_service &= ~mask;
		update_internal_serial_irq_line();
	}
	if ((offset == 0x0d4 || offset == 0x0d5) && !BIT(data, 7))
	{
		const unsigned timer = offset - 0x0d4;
		m_timer_irq_pending[timer] = false;
		m_timer_irq_in_service[timer] = false;
		update_internal_serial_irq_line();
	}
	if ((offset == 0x0e5) && !BIT(data, 7))
	{
		m_adc_irq_pending = false;
		m_adc_irq_in_service = false;
		update_internal_serial_irq_line();
	}

	switch (offset)
	{
	case 0x020:
		update_adc_timer();
		break;
	case 0x100: port_w(0, data); break;
	case 0x102: port_w(2, data); break;
	case 0x103: port_w(3, data); break;
	case 0x104: port_w(4, data); break;
	case 0x105: port_w(5, data); break;
	case 0x107: port_w(7, data); break;
	case 0x108: port_w(8, data); break;
	case 0x112:
	case 0x122:
		update_port2_output();
		break;
	case 0x173:
		update_uart0_status();
		if (BIT(data, 7) && !BIT(old, 7) && uart0_tx_enabled())
		{
			if (m_uart0_tx_loaded)
				start_uart0_tx();
			else
				request_internal_serial_irq(SERIAL_IRQ_INTST0);
		}
		break;
	case 0x175:
		m_uart0_tx_loaded = true;
		m_sfr[0x174] &= ~u8(0x80);
		update_uart0_status();
		if (!m_uart0_tx_active && uart0_tx_enabled())
			start_uart0_tx();
		break;
	case 0x17b:
		if (BIT(data, 7))
			start_uart1_tx();
		update_uart1_status();
		break;
	case 0x17d:
		m_uart1_tx_loaded = true;
		if (BIT(m_sfr[0x17b], 7))
			start_uart1_tx();
		update_uart1_status();
		break;
	case 0x17e:
		m_uart1_rx_full = false;
		update_uart1_status();
		break;
	case 0x131:
		update_timer(TIMER_IRQ_INTCM21);
		update_timer(TIMER_IRQ_INTCM31);
		break;
	case 0x144:
	case 0x145:
	case 0x15a:
	case 0x15b:
		update_timer(TIMER_IRQ_INTCM21);
		break;
	case 0x146:
	case 0x147:
	case 0x166:
	case 0x167:
		update_timer(TIMER_IRQ_INTCM31);
		break;
	}
}

void v55_device::device_start()
{
	v53_device::device_start();

	for (unsigned timer = 0; timer < TIMER_IRQ_COUNT; timer++)
		m_timer[timer] = timer_alloc(FUNC(v55_device::timer_tick), this);
	m_adc_timer = timer_alloc(FUNC(v55_device::adc_tick), this);
	m_uart0_tx_timer = timer_alloc(FUNC(v55_device::uart0_tx_tick), this);
	m_uart0_rx_timer = timer_alloc(FUNC(v55_device::uart0_rx_tick), this);
	m_uart1_tx_timer = timer_alloc(FUNC(v55_device::uart1_tx_tick), this);
	m_uart1_rx_timer = timer_alloc(FUNC(v55_device::uart1_rx_tick), this);

	save_item(NAME(m_sfr));
	save_item(NAME(m_adc_running));
	save_item(NAME(m_rxd0));
	save_item(NAME(m_cts0));
	save_item(NAME(m_uart0_txd_state));
	save_item(NAME(m_uart0_tx_byte));
	save_item(NAME(m_uart0_tx_bit));
	save_item(NAME(m_uart0_rx_byte));
	save_item(NAME(m_uart0_rx_bit));
	save_item(NAME(m_uart0_rx_prev));
	save_item(NAME(m_uart0_tx_active));
	save_item(NAME(m_uart0_tx_loaded));
	save_item(NAME(m_uart0_rx_active));
	save_item(NAME(m_uart0_rx_full));
	save_item(NAME(m_rxd1));
	save_item(NAME(m_cts1));
	save_item(NAME(m_uart1_txd_state));
	save_item(NAME(m_uart1_tx_byte));
	save_item(NAME(m_uart1_tx_bit));
	save_item(NAME(m_uart1_rx_byte));
	save_item(NAME(m_uart1_rx_bit));
	save_item(NAME(m_uart1_rx_prev));
	save_item(NAME(m_uart1_tx_active));
	save_item(NAME(m_uart1_tx_loaded));
	save_item(NAME(m_uart1_rx_active));
	save_item(NAME(m_uart1_rx_full));
	save_item(NAME(m_timer_irq_bank));
	save_item(NAME(m_timer_irq_pending));
	save_item(NAME(m_timer_irq_in_service));
	save_item(NAME(m_adc_irq_pending));
	save_item(NAME(m_adc_irq_in_service));
	save_item(NAME(m_internal_serial_irq_pending));
	save_item(NAME(m_internal_serial_irq_in_service));
	save_item(NAME(m_special_irq_stack));
	save_item(NAME(m_special_irq_stack_depth));
	m_serial_irq_mode = m_default_serial_irq_mode;
	// KPSHIP-BAKE: shipping default = m_default_serial_irq_mode (driver sets rx0_rx1). SHARED NEC core:
	// deleting this getenv is bit-exact (golden rendered unset) AND removes an upstream-blocking env knob.
	// Also collapse the 6-way serial_irq_mode enum to the one shipping mode. (Whether rx0_rx1 is the true
	// hw model is a KPSHIP-ACCURACY question, see korgprophecy.cpp set_serial_irq_mode.)
	if (const char *const env = std::getenv("KPROP_V55_SERIAL_BANKSW_EXPERIMENT"))
	{
		if (env[0] && std::strcmp(env, "0"))
		{
			if (!std::strcmp(env, "tx0"))
				m_serial_irq_mode = serial_irq_mode::tx0_only;
			else if (!std::strcmp(env, "rx0"))
				m_serial_irq_mode = serial_irq_mode::rx0_only;
			else if (!std::strcmp(env, "rx0rx1"))
				m_serial_irq_mode = serial_irq_mode::rx0_rx1;
			else if (!std::strcmp(env, "rx0late"))
				m_serial_irq_mode = serial_irq_mode::rx0_late_tx0;
			else if (!std::strcmp(env, "rx0tx0"))
				m_serial_irq_mode = serial_irq_mode::rx0_tx0;
			else
				m_serial_irq_mode = serial_irq_mode::broad;
		}
	}
	// KPSHIP-BAKE: shared-core knob; delete (bit-exact) and rename m_timer_irq_experiment off "experiment".
	if (const char *const env = std::getenv("KPROP_V55_TM2_BANKSW_EXPERIMENT"))
		m_timer_irq_experiment = env[0] && std::strcmp(env, "0");
	// KPSHIP-BAKE: forced-bank override; default (unset) = vector-table-derived is the model. Delete.
	if (const char *const env = std::getenv("KPROP_V55_TM2_BANK"))
	{
		const long bank = std::strtol(env, nullptr, 0);
		if (bank >= 0 && bank <= 15)
			m_timer_irq_bank[unsigned(TIMER_IRQ_INTCM21)] = u8(bank);
	}
	// KPSHIP-BAKE: forced-bank override; default (unset) = vector-table-derived is the model. Delete.
	if (const char *const env = std::getenv("KPROP_V55_TM3_BANK"))
	{
		const long bank = std::strtol(env, nullptr, 0);
		if (bank >= 0 && bank <= 15)
			m_timer_irq_bank[unsigned(TIMER_IRQ_INTCM31)] = u8(bank);
	}
	// INTAD is normal V55PI hardware behavior.  Keep the environment override
	// only for forced-bank comparison or explicitly disabling the model.
	// KPSHIP-BAKE: comment says INTAD is normal V55PI behavior; delete (bit-exact), rename m_adc_irq_experiment.
	if (const char *const env = std::getenv("KPROP_V55_ADC_BANKSW_EXPERIMENT"))
	{
		m_adc_irq_experiment = env[0] && std::strcmp(env, "0");
		m_adc_irq_bank = 0xff;
		if (m_adc_irq_experiment)
		{
			char *end = nullptr;
			const long bank = std::strtol(env, &end, 0);
			if (end != env && *end == '\0' && bank >= 0 && bank <= 15)
				m_adc_irq_bank = u8(bank);
		}
	}
}

void v55_device::device_reset()
{
	v53_device::device_reset();

	m_sfr.fill(0x00);
	m_sfr[0x0c5] = 0x80;
	m_sfr[0x112] = 0xff;
	m_sfr[0x174] = 0x20;
	m_rxd0 = 1;
	m_cts0 = 0;
	m_uart0_txd_state = 1;
	m_uart0_tx_byte = 0x00;
	m_uart0_tx_bit = 0x00;
	m_uart0_rx_byte = 0x00;
	m_uart0_rx_bit = 0x00;
	m_uart0_rx_prev = 1;
	m_uart0_tx_active = false;
	m_uart0_tx_loaded = false;
	m_uart0_rx_active = false;
	m_uart0_rx_full = false;
	m_rxd1 = 1;
	m_cts1 = 0;
	m_uart1_txd_state = 1;
	m_uart1_tx_byte = 0x00;
	m_uart1_tx_bit = 0x00;
	m_uart1_rx_byte = 0x00;
	m_uart1_rx_bit = 0x00;
	m_uart1_rx_prev = 1;
	m_uart1_tx_active = false;
	m_uart1_tx_loaded = false;
	m_uart1_rx_active = false;
	m_uart1_rx_full = false;
	m_adc_running = false;
	m_timer_irq_pending.fill(false);
	m_timer_irq_in_service.fill(false);
	m_adc_irq_pending = false;
	m_adc_irq_in_service = false;
	m_internal_serial_irq_pending = 0x00;
	m_internal_serial_irq_in_service = 0x00;
	m_special_irq_stack.fill(0xff);
	m_special_irq_stack_depth = 0;
	for (unsigned timer = 0; timer < TIMER_IRQ_COUNT; timer++)
		update_timer(timer_irq_source(timer));
	update_uart0_status();
	update_uart1_status();
	update_internal_serial_irq_line();
	m_txd0_handler(1);
	m_txd1_handler(1);
	for (emu_timer *timer : m_timer)
		if (timer != nullptr)
			timer->adjust(attotime::never);
	if (m_adc_timer != nullptr)
		m_adc_timer->adjust(attotime::never);
	if (m_uart0_tx_timer != nullptr)
		m_uart0_tx_timer->adjust(attotime::never);
	if (m_uart0_rx_timer != nullptr)
		m_uart0_rx_timer->adjust(attotime::never);
	if (m_uart1_tx_timer != nullptr)
		m_uart1_tx_timer->adjust(attotime::never);
	if (m_uart1_rx_timer != nullptr)
		m_uart1_rx_timer->adjust(attotime::never);

}

void v55_device::rxd_w(int state)
{
	state = state ? 1 : 0;

	if (!m_uart0_rx_active && m_uart0_rx_prev == 1 && state == 0)
	{
		m_uart0_rx_active = true;
		m_uart0_rx_byte = 0x00;
		m_uart0_rx_bit = 0x00;
		if (m_uart0_rx_timer != nullptr)
			m_uart0_rx_timer->adjust(uart0_bit_period() + uart0_bit_period() / 2);
	}

	m_rxd0 = u8(state);
	m_uart0_rx_prev = u8(state);
}

void v55_device::inject_uart0_rx_byte(u8 data)
{
	if (m_uart0_rx_full)
	{
		m_sfr[0x174] |= 0x01;
		request_internal_serial_irq(SERIAL_IRQ_INTSER0);
	}
	else
	{
		m_sfr[0x176] = data;
		m_uart0_rx_full = true;
		request_internal_serial_irq(SERIAL_IRQ_INTSR0);
	}

	update_uart0_status();
}

void v55_device::cts_w(int state)
{
	const u8 old_cts = m_cts0;
	m_cts0 = state ? 1 : 0;
	update_uart0_status();
	if (old_cts && !m_cts0 && uart0_tx_enabled())
	{
		if (m_uart0_tx_loaded)
			start_uart0_tx();
		else
			request_internal_serial_irq(SERIAL_IRQ_INTST0);
	}
}

attotime v55_device::uart0_bit_period() const
{
	// Channel 0 is the primary board link on Prophecy. Until the full
	// TXBRG/RXBRG/PRS decode is modeled, use the driver-configured rate
	// (must track the H8 SCI0 rate; 41,667 was the observed rate with the
	// H8 modeled at 16 MHz).
	return attotime::from_hz(m_uart0_bit_rate);
}

void v55_device::update_uart0_status()
{
	// V55PI 7.4.5: AS(7), WUPR(6), TxBE(5), RxBF(4), ERP(2), ERF(1), ERO(0).
	// TxBE describes the transmit buffer, not the shift register.
	u8 status = m_sfr[0x174] & 0x87;

	if (!m_uart0_tx_loaded)
		status |= 0x20;
	if (m_uart0_rx_full)
		status |= 0x10;

	m_sfr[0x174] = status;
}

bool v55_device::uart0_tx_enabled() const
{
	return BIT(m_sfr[0x173], 7) && !m_cts0;
}

void v55_device::start_uart0_tx()
{
	if (m_uart0_tx_active || !m_uart0_tx_loaded || !uart0_tx_enabled())
	{
		update_uart0_status();
		return;
	}

	// TxB0 -> shift-register transfer empties the buffer and raises INTST0.
	m_uart0_tx_active = true;
	m_uart0_tx_loaded = false;
	m_uart0_tx_byte = m_sfr[0x175];
	m_uart0_tx_bit = 0x00;
	m_uart0_txd_state = 0;
	m_txd0_handler(0);
	update_uart0_status();
	request_internal_serial_irq(SERIAL_IRQ_INTST0);
	if (m_uart0_tx_timer != nullptr)
		m_uart0_tx_timer->adjust(uart0_bit_period());
}

TIMER_CALLBACK_MEMBER(v55_device::uart0_tx_tick)
{
	if (!m_uart0_tx_active)
		return;

	if (m_uart0_tx_bit < 8)
	{
		m_uart0_txd_state = BIT(m_uart0_tx_byte, m_uart0_tx_bit);
		m_txd0_handler(m_uart0_txd_state);
		m_uart0_tx_bit++;
		m_uart0_tx_timer->adjust(uart0_bit_period());
		return;
	}

	if (m_uart0_tx_bit == 8)
	{
		m_uart0_txd_state = 1;
		m_txd0_handler(1);
		m_uart0_tx_bit++;
		m_uart0_tx_timer->adjust(uart0_bit_period());
		return;
	}

	m_uart0_tx_active = false;
	update_uart0_status();
	m_uart0_tx_timer->adjust(attotime::never);
	if (m_uart0_tx_loaded)
	{
		start_uart0_tx();
	}
	else if (uart0_tx_enabled() && BIT(m_sfr[0x173], 1))
	{
		m_sfr[0x174] |= 0x80;
		request_internal_serial_irq(SERIAL_IRQ_INTST0);
	}
}

TIMER_CALLBACK_MEMBER(v55_device::uart0_rx_tick)
{
	if (!m_uart0_rx_active)
		return;

	if (m_uart0_rx_bit < 8)
	{
		if (m_rxd0)
			m_uart0_rx_byte |= u8(1U << m_uart0_rx_bit);
		m_uart0_rx_bit++;
		m_uart0_rx_timer->adjust(uart0_bit_period());
		return;
	}

	m_uart0_rx_active = false;
	if (m_rxd0)
	{
		m_sfr[0x176] = m_uart0_rx_byte;
		m_uart0_rx_full = true;
		request_internal_serial_irq(SERIAL_IRQ_INTSR0);
	}
	else
	{
		m_sfr[0x174] |= 0x02;
		request_internal_serial_irq(SERIAL_IRQ_INTSER0);
	}
	update_uart0_status();
	m_uart0_rx_timer->adjust(attotime::never);
}

void v55_device::rxd1_w(int state)
{
	state = state ? 1 : 0;

	if (!m_uart1_rx_active && m_uart1_rx_prev == 1 && state == 0)
	{
		m_uart1_rx_active = true;
		m_uart1_rx_byte = 0x00;
		m_uart1_rx_bit = 0x00;
		if (m_uart1_rx_timer != nullptr)
			m_uart1_rx_timer->adjust(uart1_bit_period() + uart1_bit_period() / 2);
	}

	m_rxd1 = u8(state);
	m_uart1_rx_prev = u8(state);
}

void v55_device::cts1_w(int state)
{
	m_cts1 = state ? 1 : 0;
	update_uart1_status();
	if (!m_cts1)
		start_uart1_tx();
}

attotime v55_device::uart1_bit_period() const
{
	// The Prophecy firmware programs UART1 as a MIDI/SysEx transport, so a
	// conservative fixed 31.25 kbaud approximation is preferable to the old
	// always-ready stub until the full PRS/UARTM decode is modeled.
	return attotime::from_hz(31'250);
}

void v55_device::update_uart1_status()
{
	u8 status = m_sfr[0x17c] & 0x03;

	if (!m_cts1 && !m_uart1_tx_active)
		status |= 0x20;
	if (m_uart1_rx_full)
		status |= 0x40;

	m_sfr[0x17c] = status;
}

void v55_device::start_uart1_tx()
{
	if (m_uart1_tx_active || m_cts1 || !BIT(m_sfr[0x17b], 7) || !m_uart1_tx_loaded)
	{
		update_uart1_status();
		return;
	}

	m_uart1_tx_active = true;
	m_uart1_tx_byte = m_sfr[0x17d];
	m_uart1_tx_loaded = false;
	m_uart1_tx_bit = 0x00;
	m_uart1_txd_state = 0;
	m_txd1_handler(0);
	update_uart1_status();
	if (m_uart1_tx_timer != nullptr)
		m_uart1_tx_timer->adjust(uart1_bit_period());
}

TIMER_CALLBACK_MEMBER(v55_device::uart1_tx_tick)
{
	if (!m_uart1_tx_active)
		return;

	if (m_uart1_tx_bit < 8)
	{
		m_uart1_txd_state = BIT(m_uart1_tx_byte, m_uart1_tx_bit);
		m_txd1_handler(m_uart1_txd_state);
		m_uart1_tx_bit++;
		m_uart1_tx_timer->adjust(uart1_bit_period());
		return;
	}

	if (m_uart1_tx_bit == 8)
	{
		m_uart1_txd_state = 1;
		m_txd1_handler(1);
		m_uart1_tx_bit++;
		m_uart1_tx_timer->adjust(uart1_bit_period());
		return;
	}

	m_uart1_tx_active = false;
	update_uart1_status();
	m_uart1_tx_timer->adjust(attotime::never);
	request_internal_serial_irq(SERIAL_IRQ_INTST1);
}

TIMER_CALLBACK_MEMBER(v55_device::uart1_rx_tick)
{
	if (!m_uart1_rx_active)
		return;

	if (m_uart1_rx_bit < 8)
	{
		if (m_rxd1)
			m_uart1_rx_byte |= u8(1U << m_uart1_rx_bit);
		m_uart1_rx_bit++;
		m_uart1_rx_timer->adjust(uart1_bit_period());
		return;
	}

	m_uart1_rx_active = false;
	if (m_rxd1)
	{
		m_sfr[0x17e] = m_uart1_rx_byte;
		m_uart1_rx_full = true;
		request_internal_serial_irq(SERIAL_IRQ_INTSR1);
	}
	else
	{
		m_sfr[0x17c] |= 0x02;
		request_internal_serial_irq(SERIAL_IRQ_INTSER1);
	}
	update_uart1_status();
	m_uart1_rx_timer->adjust(attotime::never);
}

device_memory_interface::space_config_vector v55_device::memory_space_config() const
{
	return space_config_vector {
		std::make_pair(AS_PROGRAM,     &m_program_config),
		std::make_pair(AS_DATA,        &m_sfr_config),
		std::make_pair(AS_IO,          &m_io_config),
		std::make_pair(AS_INTERNAL_IO, &m_internal_io_config)
	};
}

bool v55_device::memory_translate(int spacenum, int intention, offs_t &address, address_space *&target_space)
{
	if (spacenum == AS_PROGRAM)
	{
		address = v33_translate(address);
		if ((intention != TR_FETCH) && (address >= 0x0ffe00) && (address <= 0x0fffef))
		{
			address &= 0x1ff;
			target_space = &space(AS_DATA);
			return true;
		}
	}

	target_space = &space(spacenum);
	return true;
}

u8 v55_device::mem_read_byte(offs_t a)
{
	const offs_t phys = v33_translate(a);
	if ((phys >= 0x0ffe00) && (phys <= 0x0fffef))
		return space(AS_DATA).read_byte(phys & 0x1ff);

	return v53_device::mem_read_byte(a);
}

u16 v55_device::mem_read_word(offs_t a)
{
	const offs_t phys = v33_translate(a);
	if ((phys >= 0x0ffe00) && (phys <= 0x0fffef))
		return space(AS_DATA).read_word_unaligned(phys & 0x1ff);

	return v53_device::mem_read_word(a);
}

void v55_device::mem_write_byte(offs_t a, u8 v)
{
	const offs_t phys = v33_translate(a);
	if ((phys >= 0x0ffe00) && (phys <= 0x0fffef))
	{
		space(AS_DATA).write_byte(phys & 0x1ff, v);
		return;
	}

	v53_device::mem_write_byte(a, v);
}

void v55_device::mem_write_word(offs_t a, u16 v)
{
	const offs_t phys = v33_translate(a);
	if ((phys >= 0x0ffe00) && (phys <= 0x0fffef))
	{
		space(AS_DATA).write_word_unaligned(phys & 0x1ff, v);
		return;
	}

	v53_device::mem_write_word(a, v);
}
