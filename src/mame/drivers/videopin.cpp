// license:BSD-3-Clause
// copyright-holders:Sebastien Monassa
/*************************************************************************

    Atari Video Pinball driver

    by Sebastien Monassa (smonassa@mail.dotcom.fr) / overhaul by SJ

    Known issues:

    videopin
    - plunger doesn't work in test mode - bug in the game code?

    solarwar
    - coins not working (free play is default for now)
    - needs correct layout file

*************************************************************************/

#include "emu.h"
#include "includes/videopin.h"

#include "cpu/m6502/m6502.h"
#include "machine/watchdog.h"
#include "sound/discrete.h"
#include "speaker.h"

#include "videopin.lh"


void videopin_state::update_plunger()
{
	uint8_t val = ioport("IN2")->read();

	if (m_prev != val)
	{
		if (val == 0)
		{
			m_time_released = machine().time();

			if (!m_mask)
				m_maincpu->set_input_line(INPUT_LINE_NMI, ASSERT_LINE);
		}
		else
			m_time_pushed = machine().time();

		m_prev = val;
	}
}


void videopin_state::device_timer(emu_timer &timer, device_timer_id id, int param)
{
	switch (id)
	{
	case TIMER_INTERRUPT:
		interrupt_callback(param);
		break;
	default:
		throw emu_fatalerror("Unknown id in videopin_state::device_timer");
	}
}


TIMER_CALLBACK_MEMBER(videopin_state::interrupt_callback)
{
	int scanline = param;

	update_plunger();

	m_maincpu->set_input_line(0, ASSERT_LINE);

	scanline = scanline + 32;

	if (scanline >= 263)
		scanline = 32;

	m_interrupt_timer->adjust(m_screen->time_until_pos(scanline), scanline);
}


void videopin_state::machine_start()
{
	m_leds.resolve();
	m_interrupt_timer = timer_alloc(TIMER_INTERRUPT);

	save_item(NAME(m_time_pushed));
	save_item(NAME(m_time_released));
	save_item(NAME(m_prev));
	save_item(NAME(m_mask));
}


void videopin_state::machine_reset()
{
	m_interrupt_timer->adjust(m_screen->time_until_pos(32), 32);

	/* both output latches are cleared on reset */

	out1_w(0);
	out2_w(0);
}


double videopin_state::calc_plunger_pos()
{
	return (machine().time().as_double() - m_time_released.as_double()) * (m_time_released.as_double() - m_time_pushed.as_double() + 0.2);
}


uint8_t videopin_state::misc_r()
{
	double plunger = calc_plunger_pos();

	// The plunger of the ball shooter has a black piece of
	// plastic (flag) attached to it. When the plunger flag passes
	// between the first section of the optical coupler, the MPU
	// receives a non-maskable interrupt. When the flag passes
	// between the second section of the optical coupler, the MPU
	// calculates the time between the PLUNGER1 and PLUNGER2
	// signals received. This results in the MPU displaying the
	// ball being shot onto the playfield at a certain speed.

	uint8_t val = ioport("IN1")->read();

	if (plunger >= 0.000 && plunger <= 0.001)
	{
		val &= ~1;   /* PLUNGER1 */
	}
	if (plunger >= 0.006 && plunger <= 0.007)
	{
		val &= ~2;   /* PLUNGER2 */
	}

	return val;
}


void videopin_state::led_w(uint8_t data)
{
	// LED matrix as seen in Video Pinball manual, fig. 4-14
	// output to "LEDxx" where xx = 01 to 32, videopin START = LED30
	static const int matrix[8][4] =
	{
		{ 26, 18, 11, 13 },
		{ 25, 17, 10,  8 },
		{ 24, 29,  9,  7 },
		{ 23, 28,  4,  6 },
		{ 22, 27,  3,  5 },
		{ 21, 16,  2, 32 },
		{ 20, 15,  1, 31 },
		{ 19, 14, 12, 30 }
	};

	// anode from 32V,64V,128V
	int a = m_screen->vpos() >> 5 & 7;

	for (int c = 0; c < 4; c++)
		m_leds[matrix[a][c] - 1] = BIT(data, c);

	m_maincpu->set_input_line(0, CLEAR_LINE);
}


void videopin_state::out1_w(uint8_t data)
{
	/* D0 => OCTAVE0  */
	/* D1 => OCTACE1  */
	/* D2 => OCTAVE2  */
	/* D3 => LOCKOUT  */
	/* D4 => NMIMASK  */
	/* D5 => NOT USED */
	/* D6 => NOT USED */
	/* D7 => NOT USED */

	m_mask = ~data & 0x10;

	if (m_mask)
		m_maincpu->set_input_line(INPUT_LINE_NMI, CLEAR_LINE);

	machine().bookkeeping().coin_lockout_global_w(~data & 0x08);

	/* Convert octave data to divide value and write to sound */
	m_discrete->write(VIDEOPIN_OCTAVE_DATA, (0x01 << (~data & 0x07)) & 0xfe);
}


void videopin_state::out2_w(uint8_t data)
{
	/* D0 => VOL0      */
	/* D1 => VOL1      */
	/* D2 => VOL2      */
	/* D3 => NOT USED  */
	/* D4 => COIN CNTR */
	/* D5 => BONG      */
	/* D6 => BELL      */
	/* D7 => ATTRACT   */

	machine().bookkeeping().coin_counter_w(0, data & 0x10);

	m_discrete->write(VIDEOPIN_BELL_EN, data & 0x40); // Bell
	m_discrete->write(VIDEOPIN_BONG_EN, data & 0x20); // Bong
	m_discrete->write(VIDEOPIN_ATTRACT_EN, data & 0x80); // Attract
	m_discrete->write(VIDEOPIN_VOL_DATA, data & 0x07); // Vol0,1,2
}


void videopin_state::note_dvsr_w(uint8_t data)
{
	/* note data */
	m_discrete->write(VIDEOPIN_NOTE_DATA, ~data &0xff);
}


/*************************************
 *
 *  Main CPU memory handlers
 *
 *************************************/

void videopin_state::main_map(address_map &map)
{
	map(0x0000, 0x01ff).ram();
	map(0x0200, 0x07ff).ram().w(FUNC(videopin_state::video_ram_w)).share("video_ram");
	map(0x0800, 0x0800).r(FUNC(videopin_state::misc_r)).w(FUNC(videopin_state::note_dvsr_w));
	map(0x0801, 0x0801).w(FUNC(videopin_state::led_w));
	map(0x0802, 0x0802).w("watchdog", FUNC(watchdog_timer_device::reset_w));
	map(0x0804, 0x0804).w(FUNC(videopin_state::ball_w));
	map(0x0805, 0x0805).w(FUNC(videopin_state::out1_w));
	map(0x0806, 0x0806).w(FUNC(videopin_state::out2_w));
	map(0x1000, 0x1000).portr("IN0");
	map(0x1800, 0x1800).portr("DSW");
	map(0x2000, 0x3fff).rom();
	map(0xe000, 0xffff).rom(); // mirror for 6502 vectors
}


/*************************************
 *
 *  Port definitions
 *
 *************************************/

static INPUT_PORTS_START( videopin )
	PORT_START("IN0")   /* IN0 */
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_COIN1 )
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_COIN2 )
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_NAME("Left Flipper") PORT_CODE(KEYCODE_LCONTROL)
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_BUTTON2 ) PORT_NAME("Right Flipper") PORT_CODE(KEYCODE_RCONTROL)
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_START1 )
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_TILT )
	PORT_SERVICE( 0x40, IP_ACTIVE_LOW )
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_START("DSW")   /* IN1 */
	PORT_DIPNAME( 0xc0, 0x80, DEF_STR( Coinage ) )      PORT_DIPLOCATION("DSW:8,7")
	PORT_DIPSETTING(    0xc0, DEF_STR( 2C_1C ) )
	PORT_DIPSETTING(    0x80, DEF_STR( 1C_1C ) )
	PORT_DIPSETTING(    0x40, DEF_STR( 1C_2C ) )
	PORT_DIPSETTING(    0x00, DEF_STR( Free_Play ) )
	PORT_DIPNAME( 0x30, 0x00, DEF_STR( Language ) )     PORT_DIPLOCATION("DSW:6,5")
	PORT_DIPSETTING(    0x00, DEF_STR( English ) )
	PORT_DIPSETTING(    0x10, DEF_STR( German ) )
	PORT_DIPSETTING(    0x20, DEF_STR( French ) )
	PORT_DIPSETTING(    0x30, DEF_STR( Spanish ) )
	PORT_DIPNAME( 0x08, 0x08, "Balls" )                 PORT_DIPLOCATION("DSW:4")
	PORT_DIPSETTING(    0x08, "3" )
	PORT_DIPSETTING(    0x00, "5" )
	PORT_DIPNAME( 0x04, 0x00, "Replay" )                PORT_DIPLOCATION("DSW:3")
	PORT_DIPSETTING(    0x04, "Off (award 80000 points instead)" )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x02, 0x00, "Extra Ball" )            PORT_DIPLOCATION("DSW:2")
	PORT_DIPSETTING(    0x02, "Off (award 50000 points instead)" )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x01, 0x01, "Replay Level" )          PORT_DIPLOCATION("DSW:1")
	PORT_DIPSETTING(    0x00, "180000 (3 balls) / 300000 (5 balls)" )
	PORT_DIPSETTING(    0x01, "210000 (3 balls) / 350000 (5 balls)" )

	PORT_START("IN1")   /* IN2 */
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_CUSTOM ) /* PLUNGER 1 */
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_CUSTOM ) /* PLUNGER 2 */
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_BUTTON3 ) PORT_NAME("Nudge") PORT_CODE(KEYCODE_SPACE)
	PORT_BIT( 0x80, IP_ACTIVE_HIGH, IPT_CUSTOM ) PORT_VBLANK("screen")

	PORT_START("IN2")
	PORT_BIT( 0xff, IP_ACTIVE_HIGH, IPT_BUTTON4 ) PORT_NAME("Ball Shooter") PORT_CODE(KEYCODE_DOWN)
INPUT_PORTS_END

static INPUT_PORTS_START( solarwar )
	PORT_INCLUDE( videopin )
	PORT_MODIFY("DSW")
	PORT_DIPNAME( 0xc0, 0x00, DEF_STR( Coinage ) )      PORT_DIPLOCATION("DSW:8,7")
	PORT_DIPSETTING(    0xc0, DEF_STR( 2C_1C ) )
	PORT_DIPSETTING(    0x80, DEF_STR( 1C_1C ) )
	PORT_DIPSETTING(    0x40, DEF_STR( 1C_2C ) )
	PORT_DIPSETTING(    0x00, DEF_STR( Free_Play ) )
	PORT_DIPUNKNOWN_DIPLOC( 0x30, 0x30, "DSW:6,5" )
	PORT_DIPNAME( 0x01, 0x01, "Replay Level" )          PORT_DIPLOCATION("DSW:1")
	PORT_DIPSETTING(    0x00, "180000 (3 balls) / 300000 (5 balls)" )
	PORT_DIPSETTING(    0x01, "300000 (3 balls) / 500000 (5 balls)" )
INPUT_PORTS_END


/*************************************
 *
 *  Graphics definitions
 *
 *************************************/

static const gfx_layout tile_layout =
{
	8, 8,
	64,
	1,
	{ 0 },
	{
		0, 1, 2, 3, 4, 5, 6, 7
	},
	{
		0x00, 0x08, 0x10, 0x18, 0x20, 0x28, 0x30, 0x38
	},
	0x40
};


static const gfx_layout ball_layout =
{
	16, 16,
	1,
	1,
	{ 0 },
	{
		0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
		0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87

	},
	{
		0x00, 0x08, 0x10, 0x18, 0x20, 0x28, 0x30, 0x38,
		0x40, 0x48, 0x50, 0x58, 0x60, 0x68, 0x70, 0x78
	},
	0x100
};


static GFXDECODE_START( gfx_videopin )
	GFXDECODE_ENTRY( "gfx1", 0x0000, tile_layout, 0, 1 )
	GFXDECODE_ENTRY( "gfx2", 0x0000, ball_layout, 0, 1 )
GFXDECODE_END



/*************************************
 *
 *  Machine driver
 *
 *************************************/

void videopin_state::videopin(machine_config &config)
{
	/* basic machine hardware */
	M6502(config, m_maincpu, 12096000 / 16);
	m_maincpu->set_addrmap(AS_PROGRAM, &videopin_state::main_map);

	WATCHDOG_TIMER(config, "watchdog");

	/* video hardware */
	SCREEN(config, m_screen, SCREEN_TYPE_RASTER);
	m_screen->set_refresh_hz(60);
	m_screen->set_size(304, 263);
	m_screen->set_visarea(0, 303, 0, 255);
	m_screen->set_screen_update(FUNC(videopin_state::screen_update));
	m_screen->set_palette(m_palette);

	GFXDECODE(config, m_gfxdecode, m_palette, gfx_videopin);

	PALETTE(config, m_palette, palette_device::MONOCHROME);

	/* sound hardware */
	SPEAKER(config, "mono").front_center();

	DISCRETE(config, m_discrete, videopin_discrete).add_route(ALL_OUTPUTS, "mono", 1.0);
}



/*************************************
 *
 *  ROM definitions
 *
 *************************************/

// This is PCB revision -01 (older version) with 16 PROMs (1024x4)
// A later undumped revision (still marked -01) only had 8 PROMs (2048x4)
ROM_START( videopin )
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD_NIB_LOW ( "34242-01.e0", 0x2000, 0x0400, CRC(c6a83795) SHA1(73a65cca7c1e337b336b7d515eafc2981e669be8) )
	ROM_LOAD_NIB_HIGH( "34237-01.k0", 0x2000, 0x0400, CRC(9b5ef087) SHA1(4ecf441742e7c39237cd544b0f0d9339943e1a2c) )
	ROM_LOAD_NIB_LOW ( "34243-01.d0", 0x2400, 0x0400, CRC(dc87d023) SHA1(1ecec121067a60b91b3912bd28737caaae463167) )
	ROM_LOAD_NIB_HIGH( "34238-01.j0", 0x2400, 0x0400, CRC(280d9e67) SHA1(229cc0448bb95f86fc7acbcb9594bc313f316580) )
	ROM_LOAD_NIB_LOW ( "34250-01.h1", 0x2800, 0x0400, CRC(26fdd5a3) SHA1(a5f1624b36f58fcdfc7c6c04784340bb08a89785) )
	ROM_LOAD_NIB_HIGH( "34249-01.h1", 0x2800, 0x0400, CRC(923b3609) SHA1(1b9fc60b27ff80b0ec26d897ea1817f466269506) )
	ROM_LOAD_NIB_LOW ( "34244-01.c0", 0x2c00, 0x0400, CRC(4c12a4b1) SHA1(4351887a8dada92cd24cfa5930456e7c5c251ceb) )
	ROM_LOAD_NIB_HIGH( "34240-01.h0", 0x2c00, 0x0400, CRC(d487eff5) SHA1(45bbf7e693f5471ea25e6ec71ce34708335d2d0b) )
	ROM_LOAD_NIB_LOW ( "34252-01.e1", 0x3000, 0x0400, CRC(4858d87a) SHA1(a50d7ef3d7a804defa25483768cf1a931dd799d5) )
	ROM_LOAD_NIB_HIGH( "34247-01.k1", 0x3000, 0x0400, CRC(d3083368) SHA1(c2083edb0f424dbf02caeaf786ab572326ae48d0) )
	ROM_LOAD_NIB_LOW ( "34246-01.a0", 0x3400, 0x0400, CRC(39ff2d49) SHA1(59221be088d783210516858a4272f7364e00e7b4) )
	ROM_LOAD_NIB_HIGH( "34239-01.h0", 0x3400, 0x0400, CRC(692de455) SHA1(ccbed14cdbeaf23961c356dfac98c6c7fb022486) )
	ROM_LOAD_NIB_LOW ( "34251-01.f1", 0x3800, 0x0400, CRC(5d416efc) SHA1(1debd835cc3e52f526fc0aab4955be7f3682b8c0) )
	ROM_LOAD_NIB_HIGH( "34248-01.j1", 0x3800, 0x0400, CRC(9f120e95) SHA1(d7434f437137690873cba66b408ec8e92b6509c1) )
	ROM_LOAD_NIB_LOW ( "34245-01.b0", 0x3c00, 0x0400, CRC(da02c194) SHA1(a4ec66c85f084286d13a9fc0b35ba5ad896bef44) )
	ROM_RELOAD(                       0xfc00, 0x0400 )
	ROM_LOAD_NIB_HIGH( "34241-01.f0", 0x3c00, 0x0400, CRC(5bfb83da) SHA1(9f392b0d4a972b6ae15ec12913a7e66761f4175d) )
	ROM_RELOAD(                       0xfc00, 0x0400 )

	ROM_REGION( 0x0200, "gfx1", 0 ) /* tiles */
	ROM_LOAD_NIB_LOW ( "34259-01.d5", 0x0000, 0x0200, CRC(6cd98c06) SHA1(48bf077b7abbd2f529a19bdf85700b93014f39f9) )
	ROM_LOAD_NIB_HIGH( "34258-01.c5", 0x0000, 0x0200, CRC(91a5f117) SHA1(03ac6b0b3da0ed5faf1ba6695d16918d12ceeff5) )

	ROM_REGION( 0x0020, "gfx2", 0 ) /* ball */
	ROM_LOAD( "34257-01.m1", 0x0000, 0x0020, CRC(50245866) SHA1(b0692bc8d44f127f6e7182a1ce75a785e22ac5b9) )

	ROM_REGION( 0x0100, "proms", 0 )
	ROM_LOAD( "9402-01.h4",  0x0000, 0x0100, CRC(b8094b4c) SHA1(82dc6799a19984f3b204ee3aeeb007e55afc8be3) ) /* sync */
ROM_END

// This is an even later revision (marked -02) with 4 EPROMs (4096x8 according to manual, while 2048x8 in reality)
ROM_START( videopina )
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD( "034253-01.m0", 0x2000, 0x0800, CRC(981b5986) SHA1(78ca6dc1b968529e23796884aa461e71bf7f9a48) )
	ROM_LOAD( "034254-01.h2", 0x2800, 0x0800, CRC(c3eebf23) SHA1(e9a0e2bf71a5131a8317ad40c1c56f84b5ae1643) )
	ROM_LOAD( "034255-01.j2", 0x3000, 0x0800, CRC(5565ae42) SHA1(4a9f376650684217d523d3378a5852aaa9fdfedc) )
	ROM_LOAD( "034256-01.k2", 0x3800, 0x0800, CRC(9f24428c) SHA1(df35225afebb4cc18a593ec665e94f677b3606ee) )
	ROM_COPY( "maincpu" , 0x3c00, 0xfc00, 0x400 )

	ROM_REGION( 0x0200, "gfx1", 0 ) // tiles
	ROM_LOAD_NIB_LOW ( "34259-01.d5", 0x0000, 0x0200, CRC(6cd98c06) SHA1(48bf077b7abbd2f529a19bdf85700b93014f39f9) )
	ROM_LOAD_NIB_HIGH( "34258-01.c5", 0x0000, 0x0200, CRC(91a5f117) SHA1(03ac6b0b3da0ed5faf1ba6695d16918d12ceeff5) )

	ROM_REGION( 0x0020, "gfx2", 0 ) // ball
	ROM_LOAD( "34257-01.m1", 0x0000, 0x0020, CRC(50245866) SHA1(b0692bc8d44f127f6e7182a1ce75a785e22ac5b9) )

	ROM_REGION( 0x0100, "proms", 0 )
	ROM_LOAD( "9402-01.h4",  0x0000, 0x0100, CRC(b8094b4c) SHA1(82dc6799a19984f3b204ee3aeeb007e55afc8be3) ) // sync
ROM_END

ROM_START( solarwar )
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD_NIB_LOW ( "36159-01.e0", 0x2000, 0x0400, CRC(0db9f0fc) SHA1(191429a25b43727694f75c0ae9cbff705fbc4d77) )
	ROM_LOAD_NIB_HIGH( "36154-01.k0", 0x2000, 0x0400, CRC(64629efc) SHA1(4da3870c35e693ed334502ea17ae023a0073ff85) )
	ROM_LOAD_NIB_LOW ( "36160-01.d0", 0x2400, 0x0400, CRC(63a25dee) SHA1(cff0f1c4d381eb99a30f2fe09ff6f42ca994a19f) )
	ROM_LOAD_NIB_HIGH( "36155-01.j0", 0x2400, 0x0400, CRC(5fa64f47) SHA1(64e37380be0df761ba81c516592fef87bba30b91) )
	ROM_LOAD_NIB_LOW ( "36167-01.h1", 0x2800, 0x0400, CRC(5a85bca8) SHA1(7af9895c2e567d569ed60305fa1245081e346fc1) )
	ROM_LOAD_NIB_HIGH( "36166-01.h1", 0x2800, 0x0400, CRC(6ce095a6) SHA1(e3bb534487d3cd0cecccff47c0742de8f951b46c) )
	ROM_LOAD_NIB_LOW ( "36161-01.c0", 0x2c00, 0x0400, CRC(a9e2e08f) SHA1(5539a86d4fb69735182762e21cf3cc26d16eff80) )
	ROM_LOAD_NIB_HIGH( "36157-01.h0", 0x2c00, 0x0400, CRC(30b6eb18) SHA1(ae819dd97c6a7e26981731e7706cbfa3699b6a0b) )
	ROM_LOAD_NIB_LOW ( "36169-01.e1", 0x3000, 0x0400, CRC(f702127c) SHA1(7fb83c616671e4ea9697282a04662ec035d5d8ed) )
	ROM_LOAD_NIB_HIGH( "36164-01.k1", 0x3000, 0x0400, CRC(3dcded96) SHA1(eacdf017b08a7c3305fd79430fbbf07292d0cfa0) )
	ROM_LOAD_NIB_LOW ( "36163-02.a0", 0x3400, 0x0400, CRC(3e176619) SHA1(9b6a9a5fa02b1d87bdaa43fad8971ff3317b132d) )
	ROM_LOAD_NIB_HIGH( "36156-02.h0", 0x3400, 0x0400, CRC(e51363fb) SHA1(c01b263dfd6d448a18ff855a93aa4e48afc6d725) )
	ROM_LOAD_NIB_LOW ( "36168-01.f1", 0x3800, 0x0400, CRC(5ccbcf7e) SHA1(10f8932265abe6e62e9f243c653d7fad770a2ff5) )
	ROM_LOAD_NIB_HIGH( "36165-01.j1", 0x3800, 0x0400, CRC(e2ee4f7d) SHA1(be2f602a5bcfe404509ac8d6914a03213573b0a6) )
	ROM_LOAD_NIB_LOW ( "36162-02.b0", 0x3c00, 0x0400, CRC(cec1baaa) SHA1(15c130b01a7b8b9aa07e01f7c84c4c26494f39d8) )
	ROM_RELOAD(                       0xfc00, 0x0400 )
	ROM_LOAD_NIB_HIGH( "36158-02.f0", 0x3c00, 0x0400, CRC(2606b87e) SHA1(ea72e36837eccf29cd5c82fe9a6a018a1a94730c) )
	ROM_RELOAD(                       0xfc00, 0x0400 )

	ROM_REGION( 0x0200, "gfx1", 0 ) /* tiles */
	ROM_LOAD_NIB_LOW ( "34259-01.d5", 0x0000, 0x0200, CRC(6cd98c06) SHA1(48bf077b7abbd2f529a19bdf85700b93014f39f9) )
	ROM_LOAD_NIB_HIGH( "34258-01.c5", 0x0000, 0x0200, CRC(91a5f117) SHA1(03ac6b0b3da0ed5faf1ba6695d16918d12ceeff5) )

	ROM_REGION( 0x0020, "gfx2", 0 ) /* ball */
	ROM_LOAD( "34257-01.m1", 0x0000, 0x0020, CRC(50245866) SHA1(b0692bc8d44f127f6e7182a1ce75a785e22ac5b9) )

	ROM_REGION( 0x0100, "proms", 0 )
	ROM_LOAD( "9402-01.h4",  0x0000, 0x0100, CRC(b8094b4c) SHA1(82dc6799a19984f3b204ee3aeeb007e55afc8be3) ) /* sync */
ROM_END


/*************************************
 *
 *  Game drivers
 *
 *************************************/

GAMEL( 1979, videopin,  0,        videopin, videopin, videopin_state, empty_init, ROT270, "Atari", "Video Pinball (16 PROMs version)", MACHINE_SUPPORTS_SAVE | MACHINE_REQUIRES_ARTWORK, layout_videopin )
GAMEL( 1979, videopina, videopin, videopin, videopin, videopin_state, empty_init, ROT270, "Atari", "Video Pinball (4 ROMs version)", MACHINE_SUPPORTS_SAVE | MACHINE_REQUIRES_ARTWORK, layout_videopin )
GAMEL( 1979, solarwar,  0,        videopin, solarwar, videopin_state, empty_init, ROT270, "Atari", "Solar War", MACHINE_SUPPORTS_SAVE | MACHINE_REQUIRES_ARTWORK, layout_videopin )
