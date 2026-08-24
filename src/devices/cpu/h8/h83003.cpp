// license:BSD-3-Clause
// copyright-holders:Olivier Galibert
#include "emu.h"
#include "h83003.h"

DEFINE_DEVICE_TYPE(H83003, h83003_device, "h83003", "Hitachi H8/3003")

h83003_device::h83003_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock) :
	h8h_device(mconfig, H83003, tag, owner, clock, address_map_constructor(FUNC(h83003_device::map), this)),
	m_intc(*this, "intc"),
	m_adc(*this, "adc"),
	m_dma(*this, "dma"),
	m_dma0(*this, "dma:0"),
	m_dma1(*this, "dma:1"),
	m_dma2(*this, "dma:2"),
	m_dma3(*this, "dma:3"),
	m_port4(*this, "port4"),
	m_port5(*this, "port5"),
	m_port6(*this, "port6"),
	m_port7(*this, "port7"),
	m_port8(*this, "port8"),
	m_port9(*this, "port9"),
	m_porta(*this, "porta"),
	m_portb(*this, "portb"),
	m_portc(*this, "portc"),
	m_timer16(*this, "timer16"),
	m_timer16_0(*this, "timer16:0"),
	m_timer16_1(*this, "timer16:1"),
	m_timer16_2(*this, "timer16:2"),
	m_timer16_3(*this, "timer16:3"),
	m_timer16_4(*this, "timer16:4"),
	m_watchdog(*this, "watchdog"),
	m_ram_view(*this, "ram_view"),
	m_tend_cb(*this),
	m_syscr(0),
	m_rtmcsr(0),
	m_abwcr(0xff),
	m_astcr(0xff),
	m_wcr(0xf3),
	m_wcer(0xff),
	m_brcr(0xfe)
{
}

void h83003_device::map(address_map &map)
{
	const offs_t base = m_mode_a20 ? 0xf0000 : 0xff0000;

	map(base | 0xfd10, base | 0xff0f).view(m_ram_view);
	m_ram_view[0](base | 0xfd10, base | 0xff0f).ram().share(m_internal_ram);

	map(base | 0xff20, base | 0xff21).rw(m_dma0, FUNC(h8h_dma_channel_device::marah_r), FUNC(h8h_dma_channel_device::marah_w));
	map(base | 0xff22, base | 0xff23).rw(m_dma0, FUNC(h8h_dma_channel_device::maral_r), FUNC(h8h_dma_channel_device::maral_w));
	map(base | 0xff24, base | 0xff25).rw(m_dma0, FUNC(h8h_dma_channel_device::etcra_r), FUNC(h8h_dma_channel_device::etcra_w));
	map(base | 0xff26, base | 0xff26).rw(m_dma0, FUNC(h8h_dma_channel_device::ioara8_r), FUNC(h8h_dma_channel_device::ioara8_w));
	map(base | 0xff27, base | 0xff27).rw(m_dma0, FUNC(h8h_dma_channel_device::dtcra_r), FUNC(h8h_dma_channel_device::dtcra_w));
	map(base | 0xff28, base | 0xff29).rw(m_dma0, FUNC(h8h_dma_channel_device::marbh_r), FUNC(h8h_dma_channel_device::marbh_w));
	map(base | 0xff2a, base | 0xff2b).rw(m_dma0, FUNC(h8h_dma_channel_device::marbl_r), FUNC(h8h_dma_channel_device::marbl_w));
	map(base | 0xff2c, base | 0xff2d).rw(m_dma0, FUNC(h8h_dma_channel_device::etcrb_r), FUNC(h8h_dma_channel_device::etcrb_w));
	map(base | 0xff2e, base | 0xff2e).rw(m_dma0, FUNC(h8h_dma_channel_device::ioarb8_r), FUNC(h8h_dma_channel_device::ioarb8_w));
	map(base | 0xff2f, base | 0xff2f).rw(m_dma0, FUNC(h8h_dma_channel_device::dtcrb_r), FUNC(h8h_dma_channel_device::dtcrb_w));
	map(base | 0xff30, base | 0xff31).rw(m_dma1, FUNC(h8h_dma_channel_device::marah_r), FUNC(h8h_dma_channel_device::marah_w));
	map(base | 0xff32, base | 0xff33).rw(m_dma1, FUNC(h8h_dma_channel_device::maral_r), FUNC(h8h_dma_channel_device::maral_w));
	map(base | 0xff34, base | 0xff35).rw(m_dma1, FUNC(h8h_dma_channel_device::etcra_r), FUNC(h8h_dma_channel_device::etcra_w));
	map(base | 0xff36, base | 0xff36).rw(m_dma1, FUNC(h8h_dma_channel_device::ioara8_r), FUNC(h8h_dma_channel_device::ioara8_w));
	map(base | 0xff37, base | 0xff37).rw(m_dma1, FUNC(h8h_dma_channel_device::dtcra_r), FUNC(h8h_dma_channel_device::dtcra_w));
	map(base | 0xff38, base | 0xff39).rw(m_dma1, FUNC(h8h_dma_channel_device::marbh_r), FUNC(h8h_dma_channel_device::marbh_w));
	map(base | 0xff3a, base | 0xff3b).rw(m_dma1, FUNC(h8h_dma_channel_device::marbl_r), FUNC(h8h_dma_channel_device::marbl_w));
	map(base | 0xff3c, base | 0xff3d).rw(m_dma1, FUNC(h8h_dma_channel_device::etcrb_r), FUNC(h8h_dma_channel_device::etcrb_w));
	map(base | 0xff3e, base | 0xff3e).rw(m_dma1, FUNC(h8h_dma_channel_device::ioarb8_r), FUNC(h8h_dma_channel_device::ioarb8_w));
	map(base | 0xff3f, base | 0xff3f).rw(m_dma1, FUNC(h8h_dma_channel_device::dtcrb_r), FUNC(h8h_dma_channel_device::dtcrb_w));
	map(base | 0xff40, base | 0xff41).rw(m_dma2, FUNC(h8h_dma_channel_device::marah_r), FUNC(h8h_dma_channel_device::marah_w));
	map(base | 0xff42, base | 0xff43).rw(m_dma2, FUNC(h8h_dma_channel_device::maral_r), FUNC(h8h_dma_channel_device::maral_w));
	map(base | 0xff44, base | 0xff45).rw(m_dma2, FUNC(h8h_dma_channel_device::etcra_r), FUNC(h8h_dma_channel_device::etcra_w));
	map(base | 0xff46, base | 0xff46).rw(m_dma2, FUNC(h8h_dma_channel_device::ioara8_r), FUNC(h8h_dma_channel_device::ioara8_w));
	map(base | 0xff47, base | 0xff47).rw(m_dma2, FUNC(h8h_dma_channel_device::dtcra_r), FUNC(h8h_dma_channel_device::dtcra_w));
	map(base | 0xff48, base | 0xff49).rw(m_dma2, FUNC(h8h_dma_channel_device::marbh_r), FUNC(h8h_dma_channel_device::marbh_w));
	map(base | 0xff4a, base | 0xff4b).rw(m_dma2, FUNC(h8h_dma_channel_device::marbl_r), FUNC(h8h_dma_channel_device::marbl_w));
	map(base | 0xff4c, base | 0xff4d).rw(m_dma2, FUNC(h8h_dma_channel_device::etcrb_r), FUNC(h8h_dma_channel_device::etcrb_w));
	map(base | 0xff4e, base | 0xff4e).rw(m_dma2, FUNC(h8h_dma_channel_device::ioarb8_r), FUNC(h8h_dma_channel_device::ioarb8_w));
	map(base | 0xff4f, base | 0xff4f).rw(m_dma2, FUNC(h8h_dma_channel_device::dtcrb_r), FUNC(h8h_dma_channel_device::dtcrb_w));
	map(base | 0xff50, base | 0xff51).rw(m_dma3, FUNC(h8h_dma_channel_device::marah_r), FUNC(h8h_dma_channel_device::marah_w));
	map(base | 0xff52, base | 0xff53).rw(m_dma3, FUNC(h8h_dma_channel_device::maral_r), FUNC(h8h_dma_channel_device::maral_w));
	map(base | 0xff54, base | 0xff55).rw(m_dma3, FUNC(h8h_dma_channel_device::etcra_r), FUNC(h8h_dma_channel_device::etcra_w));
	map(base | 0xff56, base | 0xff56).rw(m_dma3, FUNC(h8h_dma_channel_device::ioara8_r), FUNC(h8h_dma_channel_device::ioara8_w));
	map(base | 0xff57, base | 0xff57).rw(m_dma3, FUNC(h8h_dma_channel_device::dtcra_r), FUNC(h8h_dma_channel_device::dtcra_w));

	map(base | 0xff60, base | 0xff60).rw(m_timer16, FUNC(h8_timer16_device::tstr_r), FUNC(h8_timer16_device::tstr_w));
	map(base | 0xff61, base | 0xff61).rw(m_timer16, FUNC(h8_timer16_device::tsyr_r), FUNC(h8_timer16_device::tsyr_w));
	map(base | 0xff62, base | 0xff62).rw(m_timer16, FUNC(h8_timer16_device::tmdr_r), FUNC(h8_timer16_device::tmdr_w));
	map(base | 0xff63, base | 0xff63).rw(m_timer16, FUNC(h8_timer16_device::tfcr_r), FUNC(h8_timer16_device::tfcr_w));
	map(base | 0xff64, base | 0xff64).rw(m_timer16_0, FUNC(h8_timer16_channel_device::tcr_r), FUNC(h8_timer16_channel_device::tcr_w));
	map(base | 0xff65, base | 0xff65).rw(m_timer16_0, FUNC(h8_timer16_channel_device::tior_r), FUNC(h8_timer16_channel_device::tior_w));
	map(base | 0xff66, base | 0xff66).rw(m_timer16_0, FUNC(h8_timer16_channel_device::tier_r), FUNC(h8_timer16_channel_device::tier_w));
	map(base | 0xff67, base | 0xff67).rw(m_timer16_0, FUNC(h8_timer16_channel_device::tsr_r), FUNC(h8_timer16_channel_device::tsr_w));
	map(base | 0xff68, base | 0xff69).rw(m_timer16_0, FUNC(h8_timer16_channel_device::tcnt_r), FUNC(h8_timer16_channel_device::tcnt_w));
	map(base | 0xff6a, base | 0xff6d).rw(m_timer16_0, FUNC(h8_timer16_channel_device::tgr_r), FUNC(h8_timer16_channel_device::tgr_w));
	map(base | 0xff6e, base | 0xff6e).rw(m_timer16_1, FUNC(h8_timer16_channel_device::tcr_r), FUNC(h8_timer16_channel_device::tcr_w));
	map(base | 0xff6f, base | 0xff6f).rw(m_timer16_1, FUNC(h8_timer16_channel_device::tior_r), FUNC(h8_timer16_channel_device::tior_w));
	map(base | 0xff70, base | 0xff70).rw(m_timer16_1, FUNC(h8_timer16_channel_device::tier_r), FUNC(h8_timer16_channel_device::tier_w));
	map(base | 0xff71, base | 0xff71).rw(m_timer16_1, FUNC(h8_timer16_channel_device::tsr_r), FUNC(h8_timer16_channel_device::tsr_w));
	map(base | 0xff72, base | 0xff73).rw(m_timer16_1, FUNC(h8_timer16_channel_device::tcnt_r), FUNC(h8_timer16_channel_device::tcnt_w));
	map(base | 0xff74, base | 0xff77).rw(m_timer16_1, FUNC(h8_timer16_channel_device::tgr_r), FUNC(h8_timer16_channel_device::tgr_w));
	map(base | 0xff78, base | 0xff78).rw(m_timer16_2, FUNC(h8_timer16_channel_device::tcr_r), FUNC(h8_timer16_channel_device::tcr_w));
	map(base | 0xff79, base | 0xff79).rw(m_timer16_2, FUNC(h8_timer16_channel_device::tior_r), FUNC(h8_timer16_channel_device::tior_w));
	map(base | 0xff7a, base | 0xff7a).rw(m_timer16_2, FUNC(h8_timer16_channel_device::tier_r), FUNC(h8_timer16_channel_device::tier_w));
	map(base | 0xff7b, base | 0xff7b).rw(m_timer16_2, FUNC(h8_timer16_channel_device::tsr_r), FUNC(h8_timer16_channel_device::tsr_w));
	map(base | 0xff7c, base | 0xff7d).rw(m_timer16_2, FUNC(h8_timer16_channel_device::tcnt_r), FUNC(h8_timer16_channel_device::tcnt_w));
	map(base | 0xff7e, base | 0xff81).rw(m_timer16_2, FUNC(h8_timer16_channel_device::tgr_r), FUNC(h8_timer16_channel_device::tgr_w));
	map(base | 0xff82, base | 0xff82).rw(m_timer16_3, FUNC(h8_timer16_channel_device::tcr_r), FUNC(h8_timer16_channel_device::tcr_w));
	map(base | 0xff83, base | 0xff83).rw(m_timer16_3, FUNC(h8_timer16_channel_device::tior_r), FUNC(h8_timer16_channel_device::tior_w));
	map(base | 0xff84, base | 0xff84).rw(m_timer16_3, FUNC(h8_timer16_channel_device::tier_r), FUNC(h8_timer16_channel_device::tier_w));
	map(base | 0xff85, base | 0xff85).rw(m_timer16_3, FUNC(h8_timer16_channel_device::tsr_r), FUNC(h8_timer16_channel_device::tsr_w));
	map(base | 0xff86, base | 0xff87).rw(m_timer16_3, FUNC(h8_timer16_channel_device::tcnt_r), FUNC(h8_timer16_channel_device::tcnt_w));
	map(base | 0xff88, base | 0xff8b).rw(m_timer16_3, FUNC(h8_timer16_channel_device::tgr_r), FUNC(h8_timer16_channel_device::tgr_w));
	map(base | 0xff8c, base | 0xff8f).rw(m_timer16_3, FUNC(h8_timer16_channel_device::tbr_r), FUNC(h8_timer16_channel_device::tbr_w));
	map(base | 0xff90, base | 0xff90).rw(m_timer16, FUNC(h8_timer16_device::toer_r), FUNC(h8_timer16_device::toer_w));
	map(base | 0xff91, base | 0xff91).rw(m_timer16, FUNC(h8_timer16_device::tocr_r), FUNC(h8_timer16_device::tocr_w));
	map(base | 0xff92, base | 0xff92).rw(m_timer16_4, FUNC(h8_timer16_channel_device::tcr_r), FUNC(h8_timer16_channel_device::tcr_w));
	map(base | 0xff93, base | 0xff93).rw(m_timer16_4, FUNC(h8_timer16_channel_device::tior_r), FUNC(h8_timer16_channel_device::tior_w));
	map(base | 0xff94, base | 0xff94).rw(m_timer16_4, FUNC(h8_timer16_channel_device::tier_r), FUNC(h8_timer16_channel_device::tier_w));
	map(base | 0xff95, base | 0xff95).rw(m_timer16_4, FUNC(h8_timer16_channel_device::tsr_r), FUNC(h8_timer16_channel_device::tsr_w));
	map(base | 0xff96, base | 0xff97).rw(m_timer16_4, FUNC(h8_timer16_channel_device::tcnt_r), FUNC(h8_timer16_channel_device::tcnt_w));
	map(base | 0xff98, base | 0xff9b).rw(m_timer16_4, FUNC(h8_timer16_channel_device::tgr_r), FUNC(h8_timer16_channel_device::tgr_w));
	map(base | 0xff9c, base | 0xff9f).rw(m_timer16_4, FUNC(h8_timer16_channel_device::tbr_r), FUNC(h8_timer16_channel_device::tbr_w));

	map(base | 0xffa8, base | 0xffa9).rw(m_watchdog, FUNC(h8_watchdog_device::wd_r), FUNC(h8_watchdog_device::wd_w));
	map(base | 0xffaa, base | 0xffab).rw(m_watchdog, FUNC(h8_watchdog_device::rst_r), FUNC(h8_watchdog_device::rst_w));
	map(base | 0xffad, base | 0xffad).rw(FUNC(h83003_device::rtmcsr_r), FUNC(h83003_device::rtmcsr_w));

	map(base | 0xffb0, base | 0xffb0).rw(m_sci[0], FUNC(h8_sci_device::smr_r), FUNC(h8_sci_device::smr_w));
	map(base | 0xffb1, base | 0xffb1).rw(m_sci[0], FUNC(h8_sci_device::brr_r), FUNC(h8_sci_device::brr_w));
	map(base | 0xffb2, base | 0xffb2).rw(m_sci[0], FUNC(h8_sci_device::scr_r), FUNC(h8_sci_device::scr_w));
	map(base | 0xffb3, base | 0xffb3).rw(m_sci[0], FUNC(h8_sci_device::tdr_r), FUNC(h8_sci_device::tdr_w));
	map(base | 0xffb4, base | 0xffb4).rw(m_sci[0], FUNC(h8_sci_device::ssr_r), FUNC(h8_sci_device::ssr_w));
	map(base | 0xffb5, base | 0xffb5).r(m_sci[0], FUNC(h8_sci_device::rdr_r));
	map(base | 0xffb8, base | 0xffb8).rw(m_sci[1], FUNC(h8_sci_device::smr_r), FUNC(h8_sci_device::smr_w));
	map(base | 0xffb9, base | 0xffb9).rw(m_sci[1], FUNC(h8_sci_device::brr_r), FUNC(h8_sci_device::brr_w));
	map(base | 0xffba, base | 0xffba).rw(m_sci[1], FUNC(h8_sci_device::scr_r), FUNC(h8_sci_device::scr_w));
	map(base | 0xffbb, base | 0xffbb).rw(m_sci[1], FUNC(h8_sci_device::tdr_r), FUNC(h8_sci_device::tdr_w));
	map(base | 0xffbc, base | 0xffbc).rw(m_sci[1], FUNC(h8_sci_device::ssr_r), FUNC(h8_sci_device::ssr_w));
	map(base | 0xffbd, base | 0xffbd).r(m_sci[1], FUNC(h8_sci_device::rdr_r));
	map(base | 0xffc5, base | 0xffc5).rw(m_port4, FUNC(h8_port_device::ff_r), FUNC(h8_port_device::ddr_w));
	map(base | 0xffc7, base | 0xffc7).rw(m_port4, FUNC(h8_port_device::port_r), FUNC(h8_port_device::dr_w));
	map(base | 0xffc8, base | 0xffc8).rw(m_port5, FUNC(h8_port_device::ff_r), FUNC(h8_port_device::ddr_w));
	map(base | 0xffc9, base | 0xffc9).rw(m_port6, FUNC(h8_port_device::ff_r), FUNC(h8_port_device::ddr_w));
	map(base | 0xffca, base | 0xffca).rw(m_port5, FUNC(h8_port_device::port_r), FUNC(h8_port_device::dr_w));
	map(base | 0xffcb, base | 0xffcb).rw(m_port6, FUNC(h8_port_device::port_r), FUNC(h8_port_device::dr_w));
	map(base | 0xffcd, base | 0xffcd).rw(m_port8, FUNC(h8_port_device::ff_r), FUNC(h8_port_device::ddr_w));
	map(base | 0xffce, base | 0xffce).r(m_port7, FUNC(h8_port_device::port_r));
	map(base | 0xffcf, base | 0xffcf).rw(m_port8, FUNC(h8_port_device::port_r), FUNC(h8_port_device::dr_w));
	map(base | 0xffd0, base | 0xffd0).rw(m_port9, FUNC(h8_port_device::ff_r), FUNC(h8_port_device::ddr_w));
	map(base | 0xffd1, base | 0xffd1).rw(m_porta, FUNC(h8_port_device::ff_r), FUNC(h8_port_device::ddr_w));
	map(base | 0xffd2, base | 0xffd2).rw(m_port9, FUNC(h8_port_device::port_r), FUNC(h8_port_device::dr_w));
	map(base | 0xffd3, base | 0xffd3).rw(m_porta, FUNC(h8_port_device::port_r), FUNC(h8_port_device::dr_w));
	map(base | 0xffd4, base | 0xffd4).rw(m_portb, FUNC(h8_port_device::ff_r), FUNC(h8_port_device::ddr_w));
	map(base | 0xffd5, base | 0xffd5).rw(m_portc, FUNC(h8_port_device::ff_r), FUNC(h8_port_device::ddr_w));
	map(base | 0xffd6, base | 0xffd6).rw(m_portb, FUNC(h8_port_device::port_r), FUNC(h8_port_device::dr_w));
	map(base | 0xffd7, base | 0xffd7).rw(m_portc, FUNC(h8_port_device::port_r), FUNC(h8_port_device::dr_w));
	map(base | 0xffda, base | 0xffda).rw(m_port4, FUNC(h8_port_device::pcr_r), FUNC(h8_port_device::pcr_w));

	map(base | 0xffe0, base | 0xffe7).r(m_adc, FUNC(h8_adc_device::addr8_r));
	map(base | 0xffe8, base | 0xffe8).rw(m_adc, FUNC(h8_adc_device::adcsr_r), FUNC(h8_adc_device::adcsr_w));
	map(base | 0xffe9, base | 0xffe9).rw(m_adc, FUNC(h8_adc_device::adcr_r), FUNC(h8_adc_device::adcr_w));
	map(base | 0xffec, base | 0xffec).rw(FUNC(h83003_device::abwcr_r), FUNC(h83003_device::abwcr_w));
	map(base | 0xffed, base | 0xffed).rw(FUNC(h83003_device::astcr_r), FUNC(h83003_device::astcr_w));
	map(base | 0xffee, base | 0xffee).rw(FUNC(h83003_device::wcr_r), FUNC(h83003_device::wcr_w));
	map(base | 0xffef, base | 0xffef).rw(FUNC(h83003_device::wcer_r), FUNC(h83003_device::wcer_w));

	map(base | 0xfff1, base | 0xfff1).r(FUNC(h83003_device::mdcr_r));
	map(base | 0xfff2, base | 0xfff2).rw(FUNC(h83003_device::syscr_r), FUNC(h83003_device::syscr_w));
	map(base | 0xfff3, base | 0xfff3).rw(FUNC(h83003_device::brcr_r), FUNC(h83003_device::brcr_w));
	map(base | 0xfff4, base | 0xfff4).rw(m_intc, FUNC(h8h_intc_device::iscr_r), FUNC(h8h_intc_device::iscr_w));
	map(base | 0xfff5, base | 0xfff5).rw(m_intc, FUNC(h8h_intc_device::ier_r), FUNC(h8h_intc_device::ier_w));
	map(base | 0xfff6, base | 0xfff6).rw(m_intc, FUNC(h8h_intc_device::isr_r), FUNC(h8h_intc_device::isr_w));
	map(base | 0xfff8, base | 0xfff9).rw(m_intc, FUNC(h8h_intc_device::icr_r), FUNC(h8h_intc_device::icr_w));
}

void h83003_device::device_add_mconfig(machine_config &config)
{
	H8H_INTC(config, m_intc, *this);
	m_intc->set_instruction_boundary_clear(true);
	H8_ADC_3337(config, m_adc, *this, m_intc, 60);
	H8H_DMA(config, m_dma, *this);
	H8H_DMA_CHANNEL(config, m_dma0, *this, m_dma, m_intc, false, false);
	H8H_DMA_CHANNEL(config, m_dma1, *this, m_dma, m_intc, false, false);
	H8H_DMA_CHANNEL(config, m_dma2, *this, m_dma, m_intc, false, true);
	H8H_DMA_CHANNEL(config, m_dma3, *this, m_dma, m_intc, false, true);
	H8_PORT(config, m_port4, *this, h8_device::PORT_4, 0x00, 0x00);
	H8_PORT(config, m_port5, *this, h8_device::PORT_5, 0x0f, 0x00);
	H8_PORT(config, m_port6, *this, h8_device::PORT_6, 0x80, 0x80);
	H8_PORT(config, m_port7, *this, h8_device::PORT_7, 0x00, 0x00);
	H8_PORT(config, m_port8, *this, h8_device::PORT_8, 0xf0, 0xe0);
	H8_PORT(config, m_port9, *this, h8_device::PORT_9, 0x00, 0xc0);
	H8_PORT(config, m_porta, *this, h8_device::PORT_A, 0x00, 0x00);
	H8_PORT(config, m_portb, *this, h8_device::PORT_B, 0x00, 0x00);
	H8_PORT(config, m_portc, *this, h8_device::PORT_C, 0x00, 0x00);
	H8_TIMER16(config, m_timer16, *this, 5, 0xe0);
	H8H_TIMER16_CHANNEL(config, m_timer16_0, *this, 2, 2, m_intc, 24);
	H8H_TIMER16_CHANNEL(config, m_timer16_1, *this, 2, 2, m_intc, 28);
	H8H_TIMER16_CHANNEL(config, m_timer16_2, *this, 2, 2, m_intc, 32);
	H8H_TIMER16_CHANNEL(config, m_timer16_3, *this, 2, 2, m_intc, 36);
	H8H_TIMER16_CHANNEL(config, m_timer16_4, *this, 2, 2, m_intc, 40);
	H8_SCI(config, m_sci[0], 0, *this, m_intc, 52, 53, 54, 55);
	H8_SCI(config, m_sci[1], 1, *this, m_intc, 56, 57, 58, 59);
	H8_WATCHDOG(config, m_watchdog, *this, m_intc, 20, h8_watchdog_device::H);
}

void h83003_device::execute_set_input(int inputnum, int state)
{
	if(inputnum >= H8_INPUT_LINE_TEND0 && inputnum <= H8_INPUT_LINE_TEND3)
		m_tend_cb[inputnum - H8_INPUT_LINE_TEND0](state);
	else if(inputnum >= H8_INPUT_LINE_DREQ0 && inputnum <= H8_INPUT_LINE_DREQ3)
		m_dma->set_input(inputnum, state);
	else
		m_intc->set_input(inputnum, state);
}

int h83003_device::trapa_setup()
{
	if(m_syscr & 0x08)
		m_CCR |= F_I;
	else
		m_CCR |= F_I|F_UI;
	return 8;
}

void h83003_device::irq_setup()
{
	if(m_syscr & 0x08)
		m_CCR |= F_I;
	else
		m_CCR |= F_I|F_UI;
}

void h83003_device::update_irq_filter()
{
	switch(m_syscr & 0x08) {
	case 0x00:
		if((m_CCR & (F_I|F_UI)) == (F_I|F_UI))
			m_intc->set_filter(2, -1);
		else if(m_CCR & F_I)
			m_intc->set_filter(1, -1);
		else
			m_intc->set_filter(0, -1);
		break;
	case 0x08:
		if(m_CCR & F_I)
			m_intc->set_filter(2, -1);
		else
			m_intc->set_filter(0, -1);
		break;
	}
}

void h83003_device::interrupt_taken()
{
	standard_irq_callback(m_intc->interrupt_taken(m_taken_irq_vector), m_NPC);
}

void h83003_device::internal_update(u64 current_time)
{
	u64 event_time = 0;

	add_event(event_time, m_adc->internal_update(current_time));
	add_event(event_time, m_sci[0]->internal_update(current_time));
	add_event(event_time, m_sci[1]->internal_update(current_time));
	add_event(event_time, m_timer16_0->internal_update(current_time));
	add_event(event_time, m_timer16_1->internal_update(current_time));
	add_event(event_time, m_timer16_2->internal_update(current_time));
	add_event(event_time, m_timer16_3->internal_update(current_time));
	add_event(event_time, m_timer16_4->internal_update(current_time));
	add_event(event_time, m_watchdog->internal_update(current_time));

	recompute_bcount(event_time);
}

void h83003_device::notify_standby(int state)
{
	m_adc->notify_standby(state);
	m_sci[0]->notify_standby(state);
	m_sci[1]->notify_standby(state);
	m_timer16_0->notify_standby(state);
	m_timer16_1->notify_standby(state);
	m_timer16_2->notify_standby(state);
	m_timer16_3->notify_standby(state);
	m_timer16_4->notify_standby(state);
	m_watchdog->notify_standby(state);
}

void h83003_device::device_start()
{
	h8h_device::device_start();
	m_dma_device = m_dma;

	save_item(NAME(m_syscr));
	save_item(NAME(m_rtmcsr));
	save_item(NAME(m_abwcr));
	save_item(NAME(m_astcr));
	save_item(NAME(m_wcr));
	save_item(NAME(m_wcer));
	save_item(NAME(m_brcr));
}

void h83003_device::device_reset()
{
	h8h_device::device_reset();
	m_syscr = 0x0b;
	m_rtmcsr = 0x00;
	m_ram_view.select(0);
	// The mode pins select the external bus width used at reset.
	m_abwcr = m_initial_bus_16bit ? 0x00 : 0xff;
	m_astcr = 0xff;
	m_wcr = 0xf3;
	m_wcer = 0xff;
	m_brcr = 0xfe;
}

int h83003_device::reset_processing_cycles() const
{
	// Hardware manual figure 4-3: internal processing occurs after the
	// reset-vector fetch and before the first instruction prefetch.  Preserve
	// established timing for drivers that have not opted into native bus timing.
	return m_external_bus_timing ? 2 : h8h_device::reset_processing_cycles();
}

int h83003_device::interrupt_priority_cycles() const
{
	// IRQ0-IRQ7 occupy vectors 12-19.  The H8/3003 needs two states
	// to arbitrate external interrupts, but only one for internal sources.
	return m_external_bus_timing
		? (m_taken_irq_vector >= 20 ? 1 : 2)
		: h8h_device::interrupt_priority_cycles();
}

void h83003_device::interrupt_priority_complete()
{
	if (!m_external_bus_timing || !m_irq_vector)
		return;

	// The vector captured at the instruction boundary is tentative until the
	// H8/3003 priority-decision phase ends.  A request that becomes eligible in
	// that one- or two-state window wins only when it has a strictly higher
	// priority, or the same priority and the lower nonzero vector number.
	if ((m_irq_level > m_taken_irq_level) ||
		((m_irq_level == m_taken_irq_level) && (m_irq_vector < m_taken_irq_vector)))
	{
		m_taken_irq_vector = m_irq_vector;
		m_taken_irq_level = m_irq_level;
	}
}

bool h83003_device::interrupt_post_accept_prefetch() const
{
	// Hardware manual figure 5-7 shows a post-accept fetch whose result is
	// discarded before the first internal exception-processing phase.
	return m_external_bus_timing;
}

bool h83003_device::internal_phase_checkpointing_enabled() const
{
	return m_external_bus_timing;
}

void h83003_device::interrupt_instruction_boundary()
{
	m_intc->instruction_boundary();
}

int h83003_device::dma_bus_acquisition_cycles(int channel) const
{
	// Hardware manual ADE-602-055A section 8.4.8, figure 8-15 (printed page
	// 218): one dead state follows CPU-to-DMAC bus acquisition in auto-request
	// burst mode.  The burst then retains the bus for that channel's back-to-back
	// transfers.  A different burst channel must arbitrate again and therefore
	// incurs its own Td.  The figure shows no second Td when the completed burst
	// returns the bus to the CPU; that transition only clears ownership
	// bookkeeping.
	// Other DMA modes keep their pre-existing core timing until their required
	// CPU/DMAC interleave is modeled explicitly.
	return m_external_bus_timing
		&& channel >= 0 && channel < 8
		&& m_dma_channel[channel]
		&& m_dma_channel[channel]->m_trigger_vector == h8gen_dma_channel_device::AUTOREQ_B
		? 1 : 0;
}

int h83003_device::memory_access_cycles(u32 address, int size) const
{
	if (!m_external_bus_timing)
		return h8h_device::memory_access_cycles(address, size);

	address &= m_mode_a20 ? 0x0fffff : 0xffffff;
	const u32 ram_start = m_mode_a20 ? 0x0ffd10 : 0xfffd10;
	const u32 ram_end = m_mode_a20 ? 0x0fff0f : 0xffff0f;
	const u32 register_start = m_mode_a20 ? 0x0fff1c : 0xffff1c;
	if (BIT(m_syscr, 0) && address >= ram_start && address <= ram_end)
		return 2;
	if (address >= register_start)
	{
		// Appendix B.1 marks only these five ITU ranges as 16-bit registers.
		// Appendix A, table A-2 gives a word six states on the other 8-bit
		// supporting-module buses, versus three states on a 16-bit bus.
		const u8 aligned_offset = address & 0xfe;
		const bool bus_16bit = (aligned_offset >= 0x68 && aligned_offset <= 0x6d)
			|| (aligned_offset >= 0x72 && aligned_offset <= 0x77)
			|| (aligned_offset >= 0x7c && aligned_offset <= 0x81)
			|| (aligned_offset >= 0x86 && aligned_offset <= 0x8f)
			|| (aligned_offset >= 0x96 && aligned_offset <= 0x9f);
		return size == 2 && !bus_16bit ? 6 : 3;
	}

	const unsigned area_shift = m_mode_a20 ? 17 : 21;
	const unsigned area = (address >> area_shift) & 7;
	int states = BIT(m_astcr, area) ? 3 : 2;

	if (states == 3 && BIT(m_wcer, area))
	{
		const unsigned wait_mode = (m_wcr >> 2) & 3;
		// Programmable-wait and pin-wait mode 1 always insert WC states.
		// Pin WAIT extension is not modeled; an inactive/high pin adds none.
		if (wait_mode == 0 || wait_mode == 2)
			states += m_wcr & 3;
	}

	if (size == 2 && BIT(m_abwcr, area))
		states *= 2;
	return states;
}

u8 h83003_device::abwcr_r() { return m_abwcr; }
void h83003_device::abwcr_w(u8 data) { m_abwcr = data; }
u8 h83003_device::astcr_r() { return m_astcr; }
void h83003_device::astcr_w(u8 data) { m_astcr = data; }
u8 h83003_device::wcr_r() { return m_wcr; }
void h83003_device::wcr_w(u8 data) { m_wcr = 0xf0 | (data & 0x0f); }
u8 h83003_device::wcer_r() { return m_wcer; }
void h83003_device::wcer_w(u8 data) { m_wcer = data; }

u8 h83003_device::mdcr_r()
{
	// Modes 1/2 use the 20-bit address space and modes 3/4 use 24 bits;
	// the even-numbered mode in each pair starts with a 16-bit bus.
	const u8 mode = (m_mode_a20 ? 1 : 3) + (m_initial_bus_16bit ? 1 : 0);
	return 0xc0 | mode;
}

u8 h83003_device::brcr_r() { return m_brcr; }
void h83003_device::brcr_w(u8 data) { m_brcr = 0xfe | (data & 0x01); }

u8 h83003_device::syscr_r()
{
	return m_syscr;
}

void h83003_device::syscr_w(u8 data)
{
	if (BIT(data, 0))
		m_ram_view.select(0);
	else
		m_ram_view.disable();
	m_intc->set_nmi_edge(BIT(data, 2));
	m_standby_pending = BIT(data, 7);
	m_syscr = data | 0x02;
	update_irq_filter();
	logerror("syscr = %02x\n", data);
}

u8 h83003_device::rtmcsr_r()
{
	// set bit 7 -- Compare Match Flag (CMF): This status flag indicates that the RTCNT and RTCOR values have matched.
	return m_rtmcsr | 0x80;
}

void h83003_device::rtmcsr_w(u8 data)
{
	m_rtmcsr = data;
	logerror("rtmcsr = %02x\n", data);
}
