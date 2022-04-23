// license:BSD-3-Clause
// copyright-holders:Barry Rodewald
/*
 * attache.c
 *
 *  Created on: 17/05/2013
 *
 *  Driver by Barry Rodewald
 *
 *
 *  Otrona Attache
 *
 *  CPU: Zilog Z80-A, 4MHz
 *  RAM: 64kB
 *  DMA: AMD 9517A (or compatible)
 *  RTC: Oki MSM5832, Z80-PIO
 *  Sound: GI AY-3-8912
 *  FDC: NEC D765A, 5.25" floppies
 *  Video: CRT5027, 320x240
 *  Serial: Z80-SIO, two RS-232C or RS-422/423 ports
 *
 *  Note:
 *  In terminal mode (when disk booting fails or no disk is inserted), press Ctrl+Linefeed (ctrl+pgdn by default)
 *  to enter monitor mode.  From here you can run a bunch of diagnostic tests.
 *
 *  G - Display Test Pattern
 *  H - Display RAM Test
 *  nnI - Input Test  (nn = port number)
 *  J - Jump
 *  K - Keyboard Test
 *  L - Loop Tests
 *  M - Map Test
 *  nnmmO - Output Test (nn = port number, mm = data to send)
 *  P - Format Diskette (P to format disk in Drive A, 1P for Drive B)
 *  Q - CMOS RAM Test
 *  nR - Main RAM Test (n = 16kB bank to test [0-3])
 *  bbpcS - Select Output Ports (first b = printer baud rate, second b = comm baud rate, p = printer port, c = comm port)
 *  T - Real Time Clock Test
 *  U - United Tests
 *  cchsV - Read a sector from disk (cc = cylinder, h = head [bit 0=drive, bit 2=side], s = sector)
 *  cchsW - Write a sector from disk
 *  nnnnmmmmX - I/O port transmit (nnnn = number of bytes to transmit, mmmm = start of data to transmit)
 *  nnnnY - I/O port receive (nnnn = address of data loaded)
 *  Z - Auto Disk Test (1Z for drive B)
 *
 *  For the 8086 board (will display an 'x' if the 8086 board is not installed):
 *  [ - 8086 RAM Test
 *  ] - SCC Test
 *  ( - GPIB Listener/Talker Test
 *  ) - GPIB Controller Test
 *
 *  The Attache 8:16 is an upgraded Attache adding an 8086 (+ optional 8087) board with its own 256kB of RAM,
 *  and optionally a GPIB controller (TMS9914A) and serial synchronous port (Z8530 SCC).  It also has modifications
 *  to the main Z80 board, specifically the display circuitry, adding a high-resolution display, and replacing
 *  the character ROM with a larger ROM containing an IBM character set.
 *  It effectively allows the Attache to run MS-DOS and use a 10MB hard disk.
 *
 *  TODO:
 *    - Get at least some of the system tests to pass
 *    - and probably lots more I've forgotten, too.
 *    - improve Z80-8086 comms on the 8:16, saving a file to the RAM disk under CP/M often ends in deadlock.
 *    - add Z8530 SCC and TMS9914A GPIB to the 8:16.  These are optional devices, so aren't strictly required at this stage.
 *    - connect dma and sio (channel 3)
 *    - MUSIC.COM freezes on polling serial port B
 *
 * Updates: 27 Mar 2022
 * Erik Petersen
 * 
 *    - Fixed memmap
 *    - Added dummy read/write mappings for Attache ports b8 & b9 to eliminate errors in debugger
 *    - Fixed am9517 driver to support active low trigger from FDC
 *    - Enabled floppy driver sounds
 *    - Added cursor rendering
 *    - Removed duplicate code
 *    - Fixed tms9927 driver which must recompute_parameters() when register 6 is written for scrolling to work after a clear screen
 *    - Enabled right SHIFT and CTRL keys; enabled CAPS LOCK key
 *    - Corrected graphics to render relative to the upscroll_offset
 *    - Corrected reverse and highlight video per Attache demo disk
 *    - Fully support 8:16 secondary board I/O and memory traps
 *    - Fixed issue with phantom key presses (e.g., : and ;) upon releasing SHIFT
 *    - Added key repeat
 */

#include "emu.h"

#include "cpu/z80/z80.h"
#include "machine/z80daisy.h"
#include "bus/rs232/rs232.h"
#include "imagedev/floppy.h"
#include "machine/am9517a.h"
#include "machine/msm5832.h"
#include "machine/nvram.h"
#include "machine/ram.h"
#include "machine/upd765.h"
#include "machine/z80ctc.h"
#include "machine/z80sio.h"
#include "machine/z80pio.h"
#include "machine/i8255.h"
#include "cpu/i86/i86.h"
#include "sound/ay8910.h"
#include "video/tms9927.h"
#include "emupal.h"
#include "screen.h"
#include "softlist_dev.h"
#include "speaker.h"


class attache_state : public driver_device
{
public:
	attache_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_rom(*this, "boot")
		, m_ram(*this, RAM_TAG)
		, m_char_rom(*this, "video")
		, m_rtc(*this, "rtc")
		, m_psg(*this, "psg")
		, m_fdc(*this, "fdc")
		, m_sio(*this, "sio")
		, m_pio(*this, "pio")
		, m_ctc(*this, "ctc")
		, m_crtc(*this, "crtc")
		, m_dma(*this, "dma")
		, m_palette(*this, "palette")
		, m_floppy0(*this, "fdc:0:525dd")
		, m_floppy1(*this, "fdc:1:525dd")
		, m_kb_rows(*this, "row%u", 0U)
		, m_kb_mod(*this, "modifiers")
		, m_membank1(*this, "bank1")
		, m_membank2(*this, "bank2")
		, m_membank3(*this, "bank3")
		, m_membank4(*this, "bank4")
		, m_membank5(*this, "bank5")
		, m_membank6(*this, "bank6")
		, m_membank7(*this, "bank7")
		, m_membank8(*this, "bank8")
		, m_nvram(*this, "nvram")
		, m_rom_active(true)
		, m_gfx_enabled(false)
		, m_kb_clock(true)
		, m_kb_empty(true)
        , m_lines(24)
	{ }

	void attache(machine_config &config);

	uint32_t screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);

	uint8_t pio_portA_r();
	uint8_t pio_portB_r();
	void pio_portA_w(uint8_t data);
	void pio_portB_w(uint8_t data);

	uint8_t dma_mem_r(offs_t offset);
	void dma_mem_w(offs_t offset, uint8_t data);

	uint8_t fdc_dma_r();
	void fdc_dma_w(uint8_t data);
    
	DECLARE_WRITE_LINE_MEMBER(hreq_w);
	DECLARE_WRITE_LINE_MEMBER(eop_w);
	DECLARE_WRITE_LINE_MEMBER(fdc_dack_w);

protected:
	// PIO port B operation select
	enum
	{
		PIO_SEL_8910_ADDR = 0,
		PIO_SEL_8910_DATA,
		PIO_SEL_5832_READ,
		PIO_SEL_5832_WRITE,
		PIO_SEL_5101_WRITE,
		PIO_SEL_5101_READ,
		PIO_SEL_LATCH,
		PIO_SEL_NOP
	};

	// Display controller operation select
	enum
	{
		DISP_GFX_0 = 0,
		DISP_GFX_1,
		DISP_GFX_2,
		DISP_GFX_3,
		DISP_GFX_4,
		DISP_CRTC,
		DISP_ATTR,
		DISP_CHAR
	};
    
    // Character attributes
    enum
    {
        CHAR_ATT_ALT   = 0x01,  // alternate character set
        CHAR_ATT_RW    = 0x02,  // reverse & bright
        CHAR_ATT_BKG   = 0x04,  // highlight (reverse)
        CHAR_ATT_BRT   = 0x08,  // bright
        CHAR_ATT_DBL   = 0x10,  // double-size
        CHAR_ATT_UL    = 0x20,  // underline
        CHAR_ATT_SUPER = 0x40,  // superscript
        CHAR_ATT_SUB   = 0x80,  // subscript
        CHAR_ATT_STRIKE= 0xC0,  // strike-through
    };

	// overrides
	virtual void driver_start() override;
	virtual void machine_start() override;
	virtual void machine_reset() override;

	uint8_t rom_r(offs_t offset);
	void rom_w(offs_t offset, uint8_t data);

	void display_command_w(uint8_t data);
	uint8_t display_data_r(offs_t offset);
	void display_data_w(offs_t offset, uint8_t data);
	uint8_t dma_mask_r();
	void dma_mask_w(uint8_t data);

	uint8_t memmap_r(offs_t offset);
	void memmap_w(offs_t offset, uint8_t data);

	uint8_t dummy_r();
	void dummy_w(uint8_t data);

	void operation_strobe(uint8_t data);
	void keyboard_clock_w(bool state);
	uint8_t keyboard_data_r();
	uint16_t get_key();

	void attache_io(address_map &map);
	void attache_io_common(address_map &map);
	void attache_map(address_map &map);

	required_device<z80_device> m_maincpu;
	required_memory_region m_rom;
	required_device<ram_device> m_ram;
	required_memory_region m_char_rom;
	required_device<msm5832_device> m_rtc;
	required_device<ay8912_device> m_psg;
	required_device<upd765a_device> m_fdc;
	required_device<z80sio_device> m_sio;
	required_device<z80pio_device> m_pio;
	required_device<z80ctc_device> m_ctc;
	required_device<tms9927_device> m_crtc;
	required_device<am9517a_device> m_dma;
	required_device<palette_device> m_palette;
	required_device<floppy_image_device> m_floppy0;
	required_device<floppy_image_device> m_floppy1;
	required_ioport_array<8> m_kb_rows;
	required_ioport m_kb_mod;
	required_memory_bank m_membank1;
	required_memory_bank m_membank2;
	required_memory_bank m_membank3;
	required_memory_bank m_membank4;
	required_memory_bank m_membank5;
	required_memory_bank m_membank6;
	required_memory_bank m_membank7;
	required_memory_bank m_membank8;
	required_device<nvram_device> m_nvram;

	bool m_rom_active;
	bool m_gfx_enabled;
	uint8_t m_pio_porta;
	uint8_t m_pio_portb;
	uint8_t m_pio_select;
	uint8_t m_pio_latch;
	uint8_t m_crtc_reg_select;
	uint8_t m_current_cmd;
	uint8_t m_char_ram[128*32];
	uint8_t m_attr_ram[128*32];
	uint8_t m_gfx_ram[128*32*5];
	uint8_t m_char_line;
	uint8_t m_attr_line;
	uint8_t m_gfx_line;
	uint8_t m_cmos_ram[64];
	uint8_t m_cmos_select;
	uint16_t m_kb_current_key;
	bool m_kb_clock;
	bool m_kb_empty;
	uint8_t m_kb_bitpos;
	uint8_t m_memmap;
    uint8_t m_lines;
};

class attache816_state : public attache_state
{
public:
	attache816_state(const machine_config &mconfig, device_type type, const char *tag)
		: attache_state(mconfig, type, tag)
		, m_extcpu(*this,"extcpu")
		, m_ppi(*this,"ppi")
        , m_pio2(*this, "pio2")
		, m_comms_val(0)
		, m_x86_irq_enable(0)
		, m_z80_rx_ready(false)
		, m_z80_tx_ready(false)
	{
        m_lines = 25; // override lines
    }

	void attache816(machine_config &config);

protected:
	void attache_io(address_map &map);

    // 8255 Port A is bi-directional interface with Z80 bus
    // 8255 Port A maps to Z80 port B9 (z80_comms_r, z80_comms_w)
    
    // Z80 Port B8 Read status (z80_comms_status_r)
    enum {
        RD_NRDY     = 0x01, // 1=no data ready for read (PC7 /OBF)
        WR_RDY      = 0x02, // 1=ready to accept data (PC5 IBF)
    };
    
    // Z80 Port B8 Write control (z80_comms_ctrl_w)
    enum {
        RESET_8086  = 0x01,
    };
    
    // 8255 Port B bits (m_x86_irq_enable)
    enum {
        INT_SINTR   = 0x01, // Secondary board interrupt enable
        INT_GPIB    = 0x02, // TMS9914A interrupt enable
        INT_SCC     = 0x04, // Z8530 interrupt enable
        INT_FPU     = 0x08, // 8087 interrupt enable
        ENB_WAIT    = 0x10, // WAIT logic enable
        ENB_MEDRES  = 0x20, // Graphics resolution 0=640x240, 1=320x240 (cut ground pin 49 to graphics card)
        //          = 0x40,
        //          = 0x80,
    };
    
    // 8255 Port C bits (Port A status)
    enum {
        DSR         = 0x01, // /DSR (Data set Ready) from Z8530
        //          = 0x02,
        //          = 0x04,
        INT_REQ     = 0x08, // Interrupt request to 8086 (not connected)
        STB         = 0x10, // /STB
        IBF         = 0x20, // IBF (Input Buffer Full)
        ACK         = 0x40, // /ACK
        OBF         = 0x80, // /OBF (Output Buffer Full)
    };
    
    // Z80 PIO on secondary board traps 8086 I/O and memory accesses, 
    // halts 8086, and raises interrupt to Z80.
    
    // Z80 PIO Port A is bi-directional interface (0x8c-0x8d - data/command)
    
    // Z80 PIO Port B is interrupt status (0x8e-0x8f - data/command)
    enum {
        MWR         = 0x01, // 0=memory write, 1=memory read
        AD0         = 0x02, // /BHE AD0
        BHE         = 0x04, //   0   0  Upper-byte on AD8-AD15, Lower-byte on AD0-AD7
                            //   0   1  Upper-byte only on AD8-AD15 from/to odd address
                            //   1   0  Lower-byte only on AD0-AD7 from/to even address
                            //   1   1  No data transferred
        U13_RD      = 0x08, // Output: enable low-byte latch output to Port A
        U16_RD      = 0x10, // Output: enable high-byte latch output to Port A
        INT_IOWR    = 0x20, // 0=I/O write interrupt
        INT_IORD    = 0x40, // 0=I/O read interrupt
        INT_MEMACC  = 0x80, // 0=memory access interrupt (0xb0000-0xbffff)
                            // 1=I/O access interrupt (outside 0x0100-0x01ff)
    };
        
private:
	void x86_comms_w(uint8_t data);
	uint8_t x86_comms_r();
	void x86_irq_enable(uint8_t data);
	void x86_iobf_enable_w(offs_t offset, uint8_t data);
	uint8_t z80_comms_r();
	void z80_comms_w(uint8_t data);
	uint8_t z80_comms_status_r();
	void z80_comms_ctrl_w(uint8_t data);
	uint8_t pio2_portA_r();
	uint8_t pio2_portB_r();

	virtual void machine_reset() override;

	void attache_x86_io(address_map &map);
	void attache_x86_map(address_map &map);

	void clear_8086_interrupt(uint8_t);
	void write_low_byte(uint8_t);
	void clock_8086_data(uint8_t);
	void write_high_byte(uint8_t);
	void set_8086_interrupt(uint8_t);
	void run_8086(uint8_t);

    void set_addr_decode_bits(offs_t offset, uint16_t mem_mask);
    uint16_t x86_trap_io_r(offs_t offset, uint16_t mem_mask);
    void x86_trap_io_w(offs_t offset, uint16_t data, uint16_t mem_mask);
    uint16_t x86_trap_io200_r(offs_t offset, uint16_t mem_mask);
    void x86_trap_io200_w(offs_t offset, uint16_t data, uint16_t mem_mask);
    uint16_t x86_dummy_io_r(offs_t offset, uint16_t mem_mask);
    void x86_dummy_io_w(offs_t offset, uint16_t data, uint16_t mem_mask);

    uint16_t x86_trap_mem_r(offs_t offset, uint16_t mem_mask);
    void x86_trap_mem_w(offs_t offset, uint16_t data, uint16_t mem_mask);
    
	required_device<cpu_device> m_extcpu;
	required_device<i8255_device> m_ppi;
	required_device<z80pio_device> m_pio2;

	uint8_t m_comms_val;
	uint8_t m_x86_irq_enable;
	bool m_z80_rx_ready;
	bool m_z80_tx_ready;
    uint8_t m_pio2_portB_inputs;
    uint16_t m_8086_latches;
    uint16_t m_8086_data_bus;
    bool m_halted;
};

// TODO
// The 8:16 has two graphics modes: medium- (320x200) and high-resolution (640x200),
// and two text modes: 40x25 and 80x25 available under MS/DOS. When operating CP/M, 
// the video supports only 80x24 text and 320x200 graphics.
// The 8:16 character ROM contains the complete Attache character set and the 
// complete IBM character set.
uint32_t attache_state::screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect)
{
	uint8_t const start = m_crtc->upscroll_offset();
    uint8_t const cols = 80;

	// Graphics output (if enabled)
	if(m_gfx_enabled)
	{
        //TODO if(!(m_x86_irq_enable & ENB_MEDRES)) logerror("Hi-res graphics not supported!\n");
        
		pen_t const *const pen = m_palette->pens();

		for(uint8_t y=0;y<m_lines;y++)
		{
            uint8_t const vy = (start + y) % m_lines;
            
			for(uint8_t x=0;x<cols;x++)
			{
				// graphics pixels use half the clock of text, so 4 graphics pixels per character
				for(uint8_t scan=0;scan<10;scan+=2)
				{
					uint8_t const data = m_gfx_ram[(128*32*(scan/2))+(vy*128+x)];
					bitmap.pix(y*10+scan,x*8)   = pen[BIT(data,7)];
					bitmap.pix(y*10+scan,x*8+1) = pen[BIT(data,7)];
					bitmap.pix(y*10+scan,x*8+2) = pen[BIT(data,6)];
					bitmap.pix(y*10+scan,x*8+3) = pen[BIT(data,6)];
					bitmap.pix(y*10+scan,x*8+4) = pen[BIT(data,5)];
					bitmap.pix(y*10+scan,x*8+5) = pen[BIT(data,5)];
					bitmap.pix(y*10+scan,x*8+6) = pen[BIT(data,4)];
					bitmap.pix(y*10+scan,x*8+7) = pen[BIT(data,4)];
					bitmap.pix(y*10+scan+1,x*8)   = pen[BIT(data,3)];
					bitmap.pix(y*10+scan+1,x*8+1) = pen[BIT(data,3)];
					bitmap.pix(y*10+scan+1,x*8+2) = pen[BIT(data,2)];
					bitmap.pix(y*10+scan+1,x*8+3) = pen[BIT(data,2)];
					bitmap.pix(y*10+scan+1,x*8+4) = pen[BIT(data,1)];
					bitmap.pix(y*10+scan+1,x*8+5) = pen[BIT(data,1)];
					bitmap.pix(y*10+scan+1,x*8+6) = pen[BIT(data,0)];
					bitmap.pix(y*10+scan+1,x*8+7) = pen[BIT(data,0)];
				}
			}
		}
	}
	else
		bitmap.fill(0);

	// Text output
	uint8_t dbl_mode = 0;  // determines which half of character to display when using double size attribute,
						   // as it can start on either odd or even character cells.

    // Get cursor coordinates
    rectangle cursor;
    if(m_crtc->cursor_bounds(cursor))
    {
        // The ROM & BIOS seem to advance the cursor position an extra character to the right
        cursor.min_x = (cursor.min_x/8) -1;
        cursor.max_x = (cursor.max_x/8) -1;
        cursor.min_y /= 10;
        cursor.max_y /= 10;
    }

	for(uint8_t y=0;y<m_lines;y++)  // lines
	{
		uint8_t const vy = (start + y) % m_lines;

		for(uint8_t x=0;x<cols;x++)  // columns
		{
			assert(((y*128)+x) >= 0 && ((y*128)+x) < std::size(m_char_ram));
			assert(((vy*128)+x) >= 0 && ((vy*128)+x) < std::size(m_char_ram));
			uint8_t ch = m_char_ram[(vy*128)+x];
			pen_t fg = m_palette->pen(
                (m_attr_ram[(vy*128)+x] & CHAR_ATT_BRT) ||
                (m_attr_ram[(vy*128)+x] & CHAR_ATT_RW) 
                ? 2 // brightness
                : 1); // normal intensity
			if(m_attr_ram[(vy*128)+x] & CHAR_ATT_DBL) // double-size
				dbl_mode++;
			else
				dbl_mode = 0;

			for(uint8_t scan=0;scan<10;scan++)  // 10 scanlines per line
			{
                size_t alt_offset = m_attr_ram[(vy*128)+x] & CHAR_ATT_ALT ? 4096 : 0;
				uint8_t data = m_char_rom->base()[(ch*16+scan) + alt_offset];
				if((m_attr_ram[(vy*128)+x] & CHAR_ATT_STRIKE) != CHAR_ATT_STRIKE)  // if not strikethrough
				{
					if(m_attr_ram[(vy*128)+x] & CHAR_ATT_SUPER)  // superscript
					{
						if(scan >= 5)
							data = 0;
						else
							data = m_char_rom->base()[ch*16+(scan*2)+1 + alt_offset];
					}
					if(m_attr_ram[(vy*128)+x] & CHAR_ATT_SUB)  // subscript
					{
						if(scan < 5)
							data = 0;
						else
							data = m_char_rom->base()[ch*16+((scan-5)*2)+1 + alt_offset];
					}
				}
				if((m_attr_ram[(vy*128)+x] & CHAR_ATT_UL) && scan == 9)  // underline
					data = 0xff;
				if((m_attr_ram[(vy*128)+x] & CHAR_ATT_STRIKE) == CHAR_ATT_STRIKE && scan == 4)  // strikethrough
					data = 0xff;
				if((m_attr_ram[(vy*128)+x] & CHAR_ATT_RW) ||  // reverse
                   (m_attr_ram[(vy*128)+x] & CHAR_ATT_BKG))
					data = ~data;
				if(m_attr_ram[(vy*128)+x] & CHAR_ATT_DBL) // double-size
				{
					uint8_t newdata = 0;
					if(dbl_mode & 1)
					{
						newdata = (data & 0x80) | ((data & 0x80) >> 1)
								| ((data & 0x40) >> 1) | ((data & 0x40) >> 2)
								| ((data & 0x20) >> 2) | ((data & 0x20) >> 3)
								| ((data & 0x10) >> 3) | ((data & 0x10) >> 4);
					}
					else
					{
						newdata = ((data & 0x08) << 4) | ((data & 0x08) << 3)
								| ((data & 0x04) << 3) | ((data & 0x04) << 2)
								| ((data & 0x02) << 2) | ((data & 0x02) << 1)
								| ((data & 0x01) << 1) | (data & 0x01);
					}
					data = newdata;
				}

                // If we are drawing in the area of the cursor, use reverse video
                if(x >= cursor.min_x && x <= cursor.max_x &&
                   y >= cursor.min_y && y <= cursor.max_y)
                    data = ~data;

				for(uint8_t bit=0;bit<8;bit++)  // 8 pixels per character
				{
					uint16_t const xpos = x*8+bit;
					uint16_t const ypos = y*10+scan;
                    
					if(BIT(data,7-bit))
						bitmap.pix(ypos,xpos) = fg;
				}
			}
		}
	}
    
	return 0;
}

uint8_t attache_state::rom_r(offs_t offset)
{
	if(m_rom_active)
		return m_rom->base()[offset];
	else
		return m_ram->pointer()[m_membank1->entry()*0x2000 + offset];
}

void attache_state::rom_w(offs_t offset, uint8_t data)
{
	m_ram->pointer()[m_membank1->entry()*0x2000 + offset] = data;
}

uint16_t attache_state::get_key()
{
	uint8_t row,bits,data;
	uint8_t res = 0;

	// scan input ports
	for(row=0;row<8;row++)
	{
		data = m_kb_rows[row]->read();
		for(bits=0;bits<8;bits++)
		{
			if(BIT(data,bits))
			{
				res = bits & 0x07;
				res |= ((row & 0x07) << 3);
                res |= 0x80; // no shift
				m_kb_empty = false;
				data = m_kb_mod->read();
                if(data & (0x01 | 0x04 | 0x08))
					res &= 0x7F;  // shift
				if(data & (0x02 | 0x10))
					res |= 0x40;  // ctrl
				//logerror("KB: hit row %i, bit %i, shift %i, ctrl %i\n",row,bits,res&0x80?0:1,res&0x40?1:0);
				return res;
			}
		}
	}
	// no key pressed
	m_kb_empty = true;
	return res;
}

uint8_t attache_state::keyboard_data_r()
{
    static attotime start;
    static s64 delay;
	uint16_t key;
	if(m_kb_bitpos == 1)  // start bit, if data is available
	{
		key = get_key();

        // just return if no key pressed
		if(m_kb_empty)
        {
            m_kb_current_key = key;
			return 0x00; // no key, no start bit
        }
              
        if((m_kb_current_key == 0) ||  // detect keypress
           ((m_kb_current_key & 0x3F) != (key & 0x3F)))  // allow key rollover
        {
            //logerror("KB: current %02x new %02x\n", m_kb_current_key, key);
            // return keypress
			m_kb_current_key = key;
            start = machine().time();
            delay = 1000;
        }
        else if(machine().time() - start > attotime::from_msec(delay))
        {
            // repeat the keypress 20 per second
            start = machine().time();
            delay = 50;
        }        
		else
        {
            // wait for key release
			return 0x00;
        }
        
		//logerror("KB: bit position %i, key %02x, empty %i\n",m_kb_bitpos,m_kb_current_key,m_kb_empty);

        return 0x40; // return start bit
	}
	else
	{
		//logerror("KB: bit position %i, key %02x, empty %i\n",m_kb_bitpos,m_kb_current_key,m_kb_empty);
		if(m_kb_current_key & (1<<(m_kb_bitpos-2)))
			return 0x00;
		else
			return 0x40;
	}
}

void attache_state::keyboard_clock_w(bool state)
{
	if(!state && m_kb_clock) // high to low transition - advance bit position
	{
		m_kb_bitpos++;
		if(m_kb_bitpos > 9)
			m_kb_bitpos = 1;
	}
	m_kb_clock = state;
}

// TODO: Figure out exactly how the HLD, RD, WR and CS lines on the RTC are hooked up
uint8_t attache_state::pio_portA_r()
{
	uint8_t ret = 0xff;
	uint8_t porta = m_pio_porta;

	switch(m_pio_select)
	{
	case PIO_SEL_8910_DATA:
		ret = m_psg->data_r();
		logerror("PSG: data read %02x\n",ret);
		break;
	case PIO_SEL_5832_WRITE:
		m_rtc->cs_w(1);
		m_rtc->write_w(0);
		m_rtc->read_w(1);
		m_rtc->address_w((porta & 0xf0) >> 4);
		ret = m_rtc->data_r();
		//logerror("RTC: read %02x from %02x (write)\n",ret,(porta & 0xf0) >> 4);
		break;
	case PIO_SEL_5832_READ:
		m_rtc->cs_w(1);
		m_rtc->write_w(0);
		m_rtc->read_w(1);
		m_rtc->address_w((porta & 0xf0) >> 4);
		ret = m_rtc->data_r();
		//logerror("RTC: read %02x from %02x\n",ret,(porta & 0xf0) >> 4);
		break;
	case PIO_SEL_5101_WRITE:
		m_cmos_select = (m_cmos_select & 0xf0) | ((porta & 0xf0) >> 4);
		ret = m_cmos_ram[m_cmos_select] & 0x0f;
		logerror("CMOS: read %02x from byte %02x (write)\n",ret, m_cmos_select);
		break;
	case PIO_SEL_5101_READ:
		m_cmos_select = (m_cmos_select & 0xf0) | ((porta & 0xf0) >> 4);
		ret = m_cmos_ram[m_cmos_select] & 0x0f;
		logerror("CMOS: read %02x from byte %02x\n",ret, m_cmos_select);
		break;
	case PIO_SEL_LATCH:
		ret = 0x00;  // Write-only?
		break;
	case PIO_SEL_NOP:
		logerror("PIO: NOP read\n");
		break;
	}
	//logerror("PIO: Port A read operation %i returning %02x\n",m_pio_select,ret);

	return ret;
}

uint8_t attache_state::pio_portB_r()
{
	uint8_t ret = m_pio_portb & 0xbf;
	ret |= keyboard_data_r();
	return ret;
}

void attache_state::operation_strobe(uint8_t data)
{
	//logerror("PIO: Port A write operation %i, data %02x\n",m_pio_select,data);
	switch(m_pio_select)
	{
	case PIO_SEL_8910_ADDR:
		m_psg->address_w(data);
		break;
	case PIO_SEL_8910_DATA:
		m_psg->data_w(data);
		break;
	case PIO_SEL_5832_WRITE:
		m_rtc->cs_w(1);
		m_rtc->read_w(0);
		m_rtc->address_w((data & 0xf0) >> 4);
		m_rtc->data_w(data & 0x0f);
		m_rtc->write_w(1);
		//logerror("RTC: write %01x to %01x\n",data & 0x0f,(data & 0xf0) >> 4);
		break;
	case PIO_SEL_5832_READ:
		m_rtc->cs_w(1);
		m_rtc->write_w(0);
		m_rtc->read_w(0);
		m_rtc->address_w((data & 0xf0) >> 4);
		//logerror("RTC: write %01x to %01x (read)\n",data & 0x0f,(data & 0xf0) >> 4);
		break;
	case PIO_SEL_5101_WRITE:
		m_cmos_select = (m_cmos_select & 0xf0) | ((data & 0xf0) >> 4);
		m_cmos_ram[m_cmos_select] = data & 0x0f;
		logerror("CMOS: write %01x to byte %02x\n",data & 0x0f, m_cmos_select);
		break;
	case PIO_SEL_5101_READ:
		m_cmos_select = (m_cmos_select & 0xf0) | ((data & 0xf0) >> 4);
		logerror("CMOS: write %01x to byte %02x (read)\n",data & 0x0f, m_cmos_select);
		break;
	case PIO_SEL_LATCH:
		m_pio_latch = data;
		m_rom_active = ~data & 0x04;
		m_floppy0->mon_w((data & 0x01) ? 0 : 1);
		m_floppy1->mon_w((data & 0x01) ? 0 : 1);
		m_gfx_enabled = data & 0x02;
		// TODO: display brightness
		break;
	case PIO_SEL_NOP:
		logerror("PIO: NOP write\n");
		break;
	default:
		logerror("PIO: Invalid write operation %i, data %02x\n",m_pio_select,data);
	}
}

void attache_state::pio_portA_w(uint8_t data)
{
	//  AO-7 = LATCH DATA OUT:
	//  LO = MOTOR ON
	//  L1 = GRAPHICS ENABLE
	//  L2 = /EPROM ENABLE
	//  L3-7 = DISPLAY BRIGHTNESS
	//  AO-7 = 8910 DATA I/O:
	//  AO-3 = 5832 DO-3 I/O
	//  A4-7 = 5832 AO-3 OUT
	//  AO-3 = 5101 DO-3 I/O
	//  A4-7 = 5101 AO-3 OUT
	m_pio_porta = data;
}

void attache_state::pio_portB_w(uint8_t data)
{
	//  BO-1 = 5101 A4-5
    //  B2   = 5101 READ (/WRITE)
	//  B2-4 = OPERATION SELECT
	//  0 = 8910 ADDR LOAD
	//  1 = 8910 DATA LOAD
	//  2 = 5832 READ
	//  3 = 5832 WRITE
	//  4 = 5101 WRITE
	//  5 = 5101 READ
	//  6 = LATCH LOAD
	//  7 = NO-OP
	//B5 = /'138 OPERATION STROBE
	//B6 = /KEYBOARD DATA IN
	//B7 = /KEYBOARD CLOCK OUT
	m_cmos_select = ((data & 0x03) << 4) | (m_cmos_select & 0x0f);
	if(!(data & 0x20) && (m_pio_portb & 0x20))
	{
		m_pio_select = (data & 0x1c) >> 2;
		operation_strobe(m_pio_porta);
	}
	m_pio_portb = data;
	keyboard_clock_w(data & 0x80);
}

// Display uses A8-A15 placed on the bus by the OUT instruction as an extra parameter
uint8_t attache_state::display_data_r(offs_t offset)
{
	uint8_t ret = 0xff;
	uint8_t param = (offset & 0xff00) >> 8;

	switch(m_current_cmd)
	{
	case DISP_GFX_0:
		ret = m_gfx_ram[(m_gfx_line*128)+(param & 0x7f)];
		break;
	case DISP_GFX_1:
		ret = m_gfx_ram[(m_gfx_line*128)+(param & 0x7f)+(128*32)];
		break;
	case DISP_GFX_2:
		ret = m_gfx_ram[(m_gfx_line*128)+(param & 0x7f)+(128*32*2)];
		break;
	case DISP_GFX_3:
		ret = m_gfx_ram[(m_gfx_line*128)+(param & 0x7f)+(128*32*3)];
		break;
	case DISP_GFX_4:
		ret = m_gfx_ram[(m_gfx_line*128)+(param & 0x7f)+(128*32*4)];
		break;
	case DISP_CRTC:
		ret = m_crtc->read(m_crtc_reg_select);
		break;
	case DISP_ATTR:
		ret = m_attr_ram[(m_attr_line*128)+(param & 0x7f)];
		break;
	case DISP_CHAR:
		ret = m_char_ram[(m_char_line*128)+(param & 0x7f)];
		break;
	default:
		logerror("Unimplemented display operation %02x\n",m_current_cmd);
	}

	return ret;
}

void attache_state::display_data_w(offs_t offset, uint8_t data)
{
	uint8_t param = (offset & 0xff00) >> 8;
	switch(m_current_cmd)
	{
	case DISP_GFX_0:
		m_gfx_ram[(m_gfx_line*128)+(param & 0x7f)] = data;
		break;
	case DISP_GFX_1:
		m_gfx_ram[(m_gfx_line*128)+(param & 0x7f)+(128*32)] = data;
		break;
	case DISP_GFX_2:
		m_gfx_ram[(m_gfx_line*128)+(param & 0x7f)+(128*32*2)] = data;
		break;
	case DISP_GFX_3:
		m_gfx_ram[(m_gfx_line*128)+(param & 0x7f)+(128*32*3)] = data;
		break;
	case DISP_GFX_4:
		m_gfx_ram[(m_gfx_line*128)+(param & 0x7f)+(128*32*4)] = data;
		break;
	case DISP_CRTC:
		m_crtc->write(m_crtc_reg_select, data);
		//logerror("CRTC: write reg %02x, data %02x\n",m_crtc_reg_select,data);
		break;
	case DISP_ATTR:
		m_attr_ram[(m_attr_line*128)+(param & 0x7f)] = data;
		break;
	case DISP_CHAR:
		m_char_ram[(m_char_line*128)+(param & 0x7f)] = data;
		break;
//  default:
//      logerror("Unimplemented display operation %02x data %02x param %02x\n",m_current_cmd,data,param);
	}
}

void attache_state::display_command_w(uint8_t data)
{
	uint8_t cmd = (data & 0xe0) >> 5;

	m_current_cmd = cmd;

	switch(cmd)
	{
	case DISP_GFX_0:
	case DISP_GFX_1:
	case DISP_GFX_2:
	case DISP_GFX_3:
	case DISP_GFX_4:
		m_gfx_line = data & 0x1f;
		break;
	case DISP_CRTC:
		// CRT5027/TMS9927 registers
		m_crtc_reg_select = data & 0x0f;
		break;
	case DISP_ATTR:
		// Attribute RAM
		m_attr_line = data & 0x1f;
		break;
	case DISP_CHAR:
		// Character RAM
		m_char_line = data & 0x1f;
		break;
	}
}

uint8_t attache_state::memmap_r(offs_t offset)
{
	return m_memmap;
}

// Memory mapper uses A8-A15 placed on the bus by the OUT instruction as an extra parameter
void attache_state::memmap_w(offs_t offset, uint8_t data)
{
    // A13-A15 selects a register in a 4-bit RAM device
    // The contents of the register select the system RAM bank
    // A0-A7 is port (FF), A13-A15 is address range, D0-D2 is bank select, D3 is bank enable
    // If the system bank is not enabled, then /ON_BRD_MEM is activated on the bus expansion connector
	uint8_t bank = data & 0x07;
    uint8_t enb = (data & 0x08) ? 1 : 0;
	memory_bank* banknum[8] = { m_membank1, m_membank2, m_membank3, m_membank4, m_membank5, m_membank6, m_membank7, m_membank8 };
	m_memmap = data;

	banknum[offset >> 13]->set_entry(bank);
    
	logerror("MEM: addr %04X-%04X -> RAM bank %i (%s)\n", offset, offset + 0x1fff, bank, (enb ? "enabled" : "disabled"));
}

uint8_t attache_state::dummy_r()
{
	return 0xFF;
}

void attache_state::dummy_w(uint8_t data)
{
}

uint8_t attache_state::dma_mask_r()
{
	return m_dma->read(0x0f);
}

void attache_state::dma_mask_w(uint8_t data)
{
	m_dma->write(0x0f,data);
}

uint8_t attache_state::fdc_dma_r()
{
	uint8_t ret = m_fdc->dma_r();
	return ret;
}

void attache_state::fdc_dma_w(uint8_t data)
{
	m_fdc->dma_w(data);
}

uint8_t attache_state::dma_mem_r(offs_t offset)
{
	return m_maincpu->space(AS_PROGRAM).read_byte(offset);
}

void attache_state::dma_mem_w(offs_t offset, uint8_t data)
{
	m_maincpu->space(AS_PROGRAM).write_byte(offset,data);
}

WRITE_LINE_MEMBER( attache_state::hreq_w )
{
	m_maincpu->set_input_line(INPUT_LINE_HALT, state ? ASSERT_LINE : CLEAR_LINE);
    
	m_dma->hack_w(state);
}

WRITE_LINE_MEMBER(attache_state::eop_w)
{
	m_fdc->tc_w(state);
}

WRITE_LINE_MEMBER( attache_state::fdc_dack_w )
{
}

/*
 * Z80 <-> 8086 communication
 */

// PPI Port A - 8086 -> Z80
void attache816_state::x86_comms_w(uint8_t data)
{
	m_comms_val = data;
	m_ppi->pc6_w(1);  // OBF
	m_z80_rx_ready = false;
}

// PPI Port A - 8086 <- Z80
uint8_t attache816_state::x86_comms_r()
{
	m_z80_tx_ready = false;
	m_ppi->pc4_w(1);  // IBF
	return m_comms_val;
}

// PPI Port B - IRQ enable
void attache816_state::x86_irq_enable(uint8_t data)
{
	m_x86_irq_enable = data;
}

void attache816_state::x86_iobf_enable_w(offs_t offset, uint8_t data)
{
	switch(offset)
	{
		case 0x00:
			m_ppi->pc6_w(0);  // OBF
			break;
		case 0x01:
			m_ppi->pc6_w(1);  // OBF
			break;
		case 0x04:
			m_ppi->pc4_w(0);  // IBF
			break;
		case 0x05:
			m_ppi->pc4_w(1);  // IBF
			break;
		default:
			logerror("Invalid x86 IRQ enable write offset %02x data %02x\n",offset,data);
	}
}

// Z80 Port B9 - Z80 <- 8086
uint8_t attache816_state::z80_comms_r()
{
	m_z80_rx_ready = true;
	m_ppi->pc6_w(0);  // OBF
	return m_comms_val;
}

// Z80 Port B9 - Z80 -> 8086
void attache816_state::z80_comms_w(uint8_t data)
{
	m_comms_val = data;
	m_z80_tx_ready = true;
	m_ppi->pc4_w(0);  // IBF
}

// Z80 Port B8 comms status
uint8_t attache816_state::z80_comms_status_r()
{
	uint8_t ret = 0xf0;  // low nibble always high?

	if(m_z80_rx_ready)
		ret |= RD_NRDY; // set if no data is ready
	if(m_z80_tx_ready)
		ret |= WR_RDY;  // set if ready to accept data

	return ret;
}

// Z80 Port B8 comms controller
void attache816_state::z80_comms_ctrl_w(uint8_t data)
{
	m_extcpu->set_input_line(INPUT_LINE_RESET,(data & RESET_8086) ? ASSERT_LINE : CLEAR_LINE);
}

// The daughter board traps 8086 memory accesses in the range 0xb0000-0xbffff
// and I/O ports outside 0x0100-0x01ff. The 8086 is halted and an interrupt
// is raised to the Z80 through the bus expansion PIO. The Z80 reads Port B
// to discover the interrupt reason.

// m_pio2 is the bi-directional interface that handles I/O and memory trap data transfers
//  8C-8FH PIO Ports A & B

uint8_t attache816_state::pio2_portA_r()
{
    uint8_t retval = 0;
    uint8_t portb = m_pio2->port_b_read(); // read PIO output latch

    if(!(portb & U13_RD)) // select low-byte of address/data to port a
        retval = m_8086_latches & 0xff;
    else if(!(portb & U16_RD)) // select high-byte of address/data to port a
        retval = m_8086_latches >> 8;

    //logerror("pio2_portA_r U13_RD=%d U16_RD=%d ret=%02x\n", 
    //    (portb & U13_RD) ? 1 : 0, (portb & U16_RD) ? 1 : 0, retval);
        
    return retval;
}


// It seems we have to remember the Z80 PIO inputs for it.
uint8_t attache816_state::pio2_portB_r()
{
    //logerror("pio2_portB_r portb=%02x\n", m_pio2_portB_inputs);
    return m_pio2_portB_inputs;
}


// Dummy writes by the Z80 to these ports trigger certain actions:
//  80-83H CLEAR INTERRUPT TO 8086
//  84-87H CLK Z-80A DATA INTO LOW BYTE LATCH
//  88-8BH CLK 8086 DATA INTO U13 and U16
//  90-93H CLK Z-80A DATA INTO HIGH BYTE LATCH
//  94-97H INTERRUPT THE 8086
//  98-9BH UNUSED
//  9C-9FH SET 8086 TO RUN STATE

void attache816_state::set_8086_interrupt(uint8_t data)
{
    //logerror("set_8086_interrupt\n");
    if(m_x86_irq_enable & INT_SINTR)
		m_extcpu->set_input_line_and_vector(0,1/*state*/,0x03); // I8086
}

void attache816_state::clear_8086_interrupt(uint8_t data)
{
    //logerror("clear_8086_interrupt\n");
    if(m_x86_irq_enable & INT_SINTR)
		m_extcpu->set_input_line_and_vector(0,0/*state*/,0x03); // I8086
}

void attache816_state::write_low_byte(uint8_t data) // actually 'odd' byte
{
    uint8_t porta = m_pio2->port_a_read(); // read PIO output latch
    //logerror("write_low_byte %02x\n", porta);
    m_8086_data_bus = (m_8086_data_bus & 0x00ff) | (porta << 8);
}

void attache816_state::write_high_byte(uint8_t data) // actually 'even' byte
{
    uint8_t porta = m_pio2->port_a_read(); // read PIO output latch
    //logerror("write_high_byte %02x\n", porta);
    m_8086_data_bus = (m_8086_data_bus & 0xff00) | porta;
}

void attache816_state::clock_8086_data(uint8_t data)
{
    //logerror("clock_8086_data\n");
    m_8086_latches = m_8086_data_bus;
}


// Copy Z80 response to read operations to 8086 and clear halt; reset Z80 interrupts
void attache816_state::run_8086(uint8_t data)
{
    //logerror("run_8086\n");
    
    m_halted = false;
    
    // reset interrupts
    m_pio2_portB_inputs |= (INT_IOWR | INT_IORD | INT_MEMACC);
    m_pio2->port_write(z80pio_device::PORT_B, m_pio2_portB_inputs);
   
    // clear the halt signal to 8086
    m_extcpu->set_input_line(INPUT_LINE_HALT, CLEAR_LINE);
}

void attache816_state::set_addr_decode_bits(offs_t offset, uint16_t mem_mask)
{
    m_extcpu->set_input_line(INPUT_LINE_HALT, ASSERT_LINE);
    m_halted = true;

    m_8086_latches = offset<<1;             // latch the address

    m_pio2_portB_inputs |= (AD0 | BHE);     // /BHE=1, AD0=1
    
    if(mem_mask == 0xff00)
    {
        m_pio2_portB_inputs &= ~BHE;        // /BHE=0, AD0=1
        m_8086_latches |= 1;                // high-byte is odd-numbered address
    }
    else if(mem_mask == 0x00ff)
    {
        m_pio2_portB_inputs &= ~AD0;        // /BHE=1, AD0=0
    }
    else if(mem_mask == 0xffff)
    {
        m_pio2_portB_inputs &= ~(AD0 | BHE); // /BHE=0, AD0=0
    }
    
    m_pio2->port_write(z80pio_device::PORT_B, m_pio2_portB_inputs);
}

// Set INT_IOWR as well as AD0 and BHE and halt 8086. 
// The Z80 reads the target address by setting the Port B output
// bits, U13 and U16, to select the upper- or lower-byte latch,
// and then reading Port A (pio2_portA_r) to get the specified
// address byte. The Z80 then writes to port 88-8b (clock_8086_data)
// to latch the 8086 data bus and repeats the same process to 
// read the data. The Z80 reads bits AD0 and BHE from Port B to 
// determine if the data is 8- or 16-bits.
void attache816_state::x86_trap_io200_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
    return x86_trap_io_w(offset + 0x100, data, mem_mask);
}

void attache816_state::x86_trap_io_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
    logerror("x86_trap_io_w portb=%02x write %04x to addr=%04x mem_mask=%04x\n", 
        m_pio2_portB_inputs, data, offset, mem_mask);

    set_addr_decode_bits(offset, mem_mask);     // set AD0 and BHE
           
    m_8086_data_bus = data;                     // remember the data
    m_pio2_portB_inputs &= ~INT_IOWR;           // Raise interrupt
    m_pio2->port_write(z80pio_device::PORT_B, m_pio2_portB_inputs);
}

// Set bits INT_IORD, AD0, and BHE2 and then halt the 8086.
// The Z80 reads the target address as described previously.
// The Z80 then writes the return data to Port A
// and latches it to the upper- or lower-byte latch by writing
// to ports 90-93 or 84-87 respectively. The contents of the
// latches are applied to the 8086 data bus so that the 
// processor receives them when the halt is cleared.
uint16_t attache816_state::x86_trap_io200_r(offs_t offset, uint16_t mem_mask)
{
    return x86_trap_io_r(offset + 0x100, mem_mask);
}

uint16_t attache816_state::x86_trap_io_r(offs_t offset, uint16_t mem_mask)
{
    logerror("x86_trap_io_r portb=%02x read addr=%04x mem_mask=%04x\n", 
        m_pio2_portB_inputs, offset, mem_mask);
        
    set_addr_decode_bits(offset, mem_mask);     // set AD0 and BHE

    m_pio2_portB_inputs &= ~INT_IORD;           // Raise interrupt
    m_pio2->port_write(z80pio_device::PORT_B, m_pio2_portB_inputs);
    
    // run_8086() terminates this while() loop and un-halts the 8086
    device_scheduler & m_scheduler = machine().scheduler();
    while(m_halted)
        m_scheduler.timeslice();
    
    return m_8086_data_bus;
}

// Graphics RAM read/write traps

void attache816_state::x86_trap_mem_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
    //logerror("x86_trap_mem_w portb=%02x write %04x to addr=%04x mem_mask=%04x\n", 
    //    m_pio2_portB_inputs, data, offset, mem_mask);    

    set_addr_decode_bits(offset, mem_mask);     // set AD0 and BHE

    m_8086_data_bus = data;                     // remember the data
    m_pio2_portB_inputs &= ~(INT_MEMACC | MWR); // Raise interrupt
    m_pio2->port_write(z80pio_device::PORT_B, m_pio2_portB_inputs);
}

uint16_t attache816_state::x86_trap_mem_r(offs_t offset, uint16_t mem_mask)
{
    //logerror("x86_trap_mem_r portb=%02x read addr=%04x mem_mask=%04x\n", 
    //    m_pio2_portB_inputs, offset, mem_mask);

    set_addr_decode_bits(offset, mem_mask);     // set AD0 and BHE

    m_pio2_portB_inputs &= ~INT_MEMACC;         // Raise interrupt
    m_pio2_portB_inputs |= MWR;                 // Read
    m_pio2->port_write(z80pio_device::PORT_B, m_pio2_portB_inputs);
        
    // run_8086() terminates this while() loop and un-halts the 8086
    device_scheduler & m_scheduler = machine().scheduler();
    while(m_halted)
        m_scheduler.timeslice();
            
    return m_8086_data_bus;
}


uint16_t attache816_state::x86_dummy_io_r(offs_t offset, uint16_t mem_mask)
{
    return 0xffff;
}

void attache816_state::x86_dummy_io_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{}

void attache_state::attache_map(address_map &map)
{
	map(0x0000, 0x1fff).bankrw("bank1");
	map(0x2000, 0x3fff).bankrw("bank2");
	map(0x4000, 0x5fff).bankrw("bank3");
	map(0x6000, 0x7fff).bankrw("bank4");
	map(0x8000, 0x9fff).bankrw("bank5");
	map(0xa000, 0xbfff).bankrw("bank6");
	map(0xc000, 0xdfff).bankrw("bank7");
	map(0xe000, 0xffff).bankrw("bank8");
}

void attache_state::attache_io_common(address_map &map)
{
	map(0xe0, 0xed).rw(m_dma, FUNC(am9517a_device::read), FUNC(am9517a_device::write)).mirror(0xff00);
	map(0xee, 0xee).w(FUNC(attache_state::display_command_w)).mirror(0xff00);
	map(0xef, 0xef).rw(FUNC(attache_state::dma_mask_r), FUNC(attache_state::dma_mask_w)).mirror(0xff00);
	map(0xf0, 0xf3).rw(m_sio, FUNC(z80sio_device::ba_cd_r), FUNC(z80sio_device::ba_cd_w)).mirror(0xff00);
	map(0xf4, 0xf7).rw(m_ctc, FUNC(z80ctc_device::read), FUNC(z80ctc_device::write)).mirror(0xff00);
	map(0xf8, 0xfb).rw(m_pio, FUNC(z80pio_device::read_alt), FUNC(z80pio_device::write_alt)).mirror(0xff00);
	map(0xfc, 0xfd).m(m_fdc, FUNC(upd765a_device::map)).mirror(0xff00);
	map(0xfe, 0xfe).rw(FUNC(attache_state::display_data_r), FUNC(attache_state::display_data_w)).select(0xff00);
	map(0xff, 0xff).rw(FUNC(attache_state::memmap_r), FUNC(attache_state::memmap_w)).select(0xff00);
}

void attache_state::attache_io(address_map &map)
{
    attache_io_common(map);
    
	map(0xb8, 0xb8).rw(FUNC(attache_state::dummy_r), FUNC(attache_state::dummy_w)).mirror(0xff00);
	map(0xb9, 0xb9).rw(FUNC(attache_state::dummy_r), FUNC(attache_state::dummy_w)).mirror(0xff00);
}

void attache816_state::attache_io(address_map &map)
{
    attache_io_common(map);
    
	map(0xb8, 0xb8).rw(FUNC(attache816_state::z80_comms_status_r), FUNC(attache816_state::z80_comms_ctrl_w)).mirror(0xff00);
	map(0xb9, 0xb9).rw(FUNC(attache816_state::z80_comms_r), FUNC(attache816_state::z80_comms_w)).mirror(0xff00);
    
	map(0x80, 0x83).w(FUNC(attache816_state::clear_8086_interrupt)).mirror(0xff00);
	map(0x84, 0x87).w(FUNC(attache816_state::write_low_byte)).mirror(0xff00);
	map(0x88, 0x8b).w(FUNC(attache816_state::clock_8086_data)).mirror(0xff00);
	map(0x8c, 0x8f).rw(m_pio2, FUNC(z80pio_device::read_alt), FUNC(z80pio_device::write_alt)).mirror(0xff00);
	map(0x90, 0x93).w(FUNC(attache816_state::write_high_byte)).mirror(0xff00);
	map(0x94, 0x97).w(FUNC(attache816_state::set_8086_interrupt)).mirror(0xff00);
	map(0x9c, 0x9f).w(FUNC(attache816_state::run_8086)).mirror(0xff00);
}


void attache816_state::attache_x86_map(address_map &map)
{
	map(0x00000, 0x3ffff).ram();
	map(0xb0000, 0xbffff).rw(FUNC(attache816_state::x86_trap_mem_r), FUNC(attache816_state::x86_trap_mem_w)).select(0);
	map(0xfe000, 0xfffff).rom().region("x86bios", 0x0000);
}

void attache816_state::attache_x86_io(address_map &map)
{
	map(0x100, 0x107).rw(m_ppi, FUNC(i8255_device::read), FUNC(i8255_device::write)).umask16(0x00ff);
	map(0x108, 0x10d).w(FUNC(attache816_state::x86_iobf_enable_w));
    
    // 0x140/2/4/6 - Z8530 SCC serial - map to silence Error Log
	map(0x140, 0x147).rw(FUNC(attache816_state::dummy_r), FUNC(attache816_state::dummy_w));

    // 0x180/2/4/6/8/a/c/e - GPIB (TMS9914A) - map to silence Error Log
	map(0x180, 0x18f).rw(FUNC(attache816_state::dummy_r), FUNC(attache816_state::dummy_w));
    
    // Trap all I/O reads and writes outside the range 0x0100-0x01ff
    map(0x0000, 0x00ff).rw(FUNC(attache816_state::x86_trap_io_r), FUNC(attache816_state::x86_trap_io_w)).mask(0x00ff);
    map(0x0200, 0xffff).rw(FUNC(attache816_state::x86_trap_io200_r), FUNC(attache816_state::x86_trap_io200_w)).mask(0xffff);
}

static INPUT_PORTS_START(attache)
	PORT_START("row0")
	PORT_BIT(0x01,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("BS") PORT_CODE(KEYCODE_BACKSPACE) PORT_CHAR(8)
	PORT_BIT(0x02,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("TAB") PORT_CODE(KEYCODE_TAB) PORT_CHAR(9)
	PORT_BIT(0x04,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("LF") PORT_CODE(KEYCODE_PGDN) PORT_CHAR(10)
	PORT_BIT(0x18,IP_ACTIVE_HIGH,IPT_UNUSED)
	PORT_BIT(0x20,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("Return") PORT_CODE(KEYCODE_ENTER) PORT_CHAR(13)
	PORT_BIT(0x40,IP_ACTIVE_HIGH,IPT_UNUSED)
	PORT_BIT(0x80,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("LOCK") PORT_CODE(KEYCODE_PGUP)

	PORT_START("row1")
	PORT_BIT(0x01,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("Space") PORT_CODE(KEYCODE_SPACE) PORT_CHAR(' ')
	PORT_BIT(0x06,IP_ACTIVE_HIGH,IPT_UNUSED)
	PORT_BIT(0x08,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("ESC") PORT_CODE(KEYCODE_ESC) PORT_CHAR(27)
	PORT_BIT(0x10,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("Left") PORT_CODE(KEYCODE_LEFT) PORT_CHAR(UCHAR_MAMEKEY(LEFT))
	PORT_BIT(0x20,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("Right") PORT_CODE(KEYCODE_RIGHT) PORT_CHAR(UCHAR_MAMEKEY(RIGHT))
	PORT_BIT(0x40,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("Up") PORT_CODE(KEYCODE_UP) PORT_CHAR(UCHAR_MAMEKEY(UP))
	PORT_BIT(0x80,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("Down") PORT_CODE(KEYCODE_DOWN) PORT_CHAR(UCHAR_MAMEKEY(DOWN))

	PORT_START("row2")
	PORT_BIT(0x01,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("0 ^") PORT_CODE(KEYCODE_0) PORT_CHAR('0') PORT_CHAR('^')
	PORT_BIT(0x02,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("1 !") PORT_CODE(KEYCODE_1) PORT_CHAR('1') PORT_CHAR('!')
	PORT_BIT(0x04,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("2 @") PORT_CODE(KEYCODE_2) PORT_CHAR('2') PORT_CHAR('@')
	PORT_BIT(0x08,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("3 #") PORT_CODE(KEYCODE_3) PORT_CHAR('3') PORT_CHAR('#')
	PORT_BIT(0x10,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("4 $") PORT_CODE(KEYCODE_4) PORT_CHAR('4') PORT_CHAR('$')
	PORT_BIT(0x20,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("5 %") PORT_CODE(KEYCODE_5) PORT_CHAR('5') PORT_CHAR('%')
	PORT_BIT(0x40,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("6 &") PORT_CODE(KEYCODE_6) PORT_CHAR('6') PORT_CHAR('&')
	PORT_BIT(0x80,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("7 *") PORT_CODE(KEYCODE_7) PORT_CHAR('7') PORT_CHAR('*')

	PORT_START("row3")
	PORT_BIT(0x01,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("8 (") PORT_CODE(KEYCODE_8) PORT_CHAR('8') PORT_CHAR('(')
	PORT_BIT(0x02,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("9 )") PORT_CODE(KEYCODE_9) PORT_CHAR('9') PORT_CHAR(')')
	PORT_BIT(0x04,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("' \"") PORT_CODE(KEYCODE_QUOTE) PORT_CHAR('\'') PORT_CHAR('"')
	PORT_BIT(0x08,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("; :") PORT_CODE(KEYCODE_COLON) PORT_CHAR(';') PORT_CHAR(':')
	PORT_BIT(0x10,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME(", <") PORT_CODE(KEYCODE_COMMA) PORT_CHAR(',') PORT_CHAR('<')
	PORT_BIT(0x20,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("= +") PORT_CODE(KEYCODE_EQUALS) PORT_CHAR('=') PORT_CHAR('+')
	PORT_BIT(0x40,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME(". >") PORT_CODE(KEYCODE_STOP) PORT_CHAR('.') PORT_CHAR('>')
	PORT_BIT(0x80,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("/ ?") PORT_CODE(KEYCODE_SLASH) PORT_CHAR('/') PORT_CHAR('?')

	PORT_START("row4")
	PORT_BIT(0x01,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("` ~") PORT_CODE(KEYCODE_TILDE) PORT_CHAR('`') PORT_CHAR('~')
	PORT_BIT(0x02,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("A") PORT_CODE(KEYCODE_A) PORT_CHAR('a') PORT_CHAR('A')
	PORT_BIT(0x04,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("B") PORT_CODE(KEYCODE_B) PORT_CHAR('b') PORT_CHAR('B')
	PORT_BIT(0x08,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("C") PORT_CODE(KEYCODE_C) PORT_CHAR('c') PORT_CHAR('C')
	PORT_BIT(0x10,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("D") PORT_CODE(KEYCODE_D) PORT_CHAR('d') PORT_CHAR('D')
	PORT_BIT(0x20,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("E") PORT_CODE(KEYCODE_E) PORT_CHAR('e') PORT_CHAR('E')
	PORT_BIT(0x40,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("F") PORT_CODE(KEYCODE_F) PORT_CHAR('f') PORT_CHAR('F')
	PORT_BIT(0x80,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("G") PORT_CODE(KEYCODE_G) PORT_CHAR('g') PORT_CHAR('G')

	PORT_START("row5")
	PORT_BIT(0x01,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("H") PORT_CODE(KEYCODE_H) PORT_CHAR('h') PORT_CHAR('H')
	PORT_BIT(0x02,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("I") PORT_CODE(KEYCODE_I) PORT_CHAR('i') PORT_CHAR('I')
	PORT_BIT(0x04,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("J") PORT_CODE(KEYCODE_J) PORT_CHAR('j') PORT_CHAR('J')
	PORT_BIT(0x08,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("K") PORT_CODE(KEYCODE_K) PORT_CHAR('k') PORT_CHAR('K')
	PORT_BIT(0x10,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("L") PORT_CODE(KEYCODE_L) PORT_CHAR('l') PORT_CHAR('L')
	PORT_BIT(0x20,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("M") PORT_CODE(KEYCODE_M) PORT_CHAR('m') PORT_CHAR('M')
	PORT_BIT(0x40,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("N") PORT_CODE(KEYCODE_N) PORT_CHAR('n') PORT_CHAR('N')
	PORT_BIT(0x80,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("O") PORT_CODE(KEYCODE_O) PORT_CHAR('o') PORT_CHAR('O')

	PORT_START("row6")
	PORT_BIT(0x01,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("P") PORT_CODE(KEYCODE_P) PORT_CHAR('p') PORT_CHAR('P')
	PORT_BIT(0x02,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("Q") PORT_CODE(KEYCODE_Q) PORT_CHAR('q') PORT_CHAR('Q')
	PORT_BIT(0x04,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("R") PORT_CODE(KEYCODE_R) PORT_CHAR('r') PORT_CHAR('R')
	PORT_BIT(0x08,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("S") PORT_CODE(KEYCODE_S) PORT_CHAR('s') PORT_CHAR('S')
	PORT_BIT(0x10,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("T") PORT_CODE(KEYCODE_T) PORT_CHAR('t') PORT_CHAR('T')
	PORT_BIT(0x20,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("U") PORT_CODE(KEYCODE_U) PORT_CHAR('u') PORT_CHAR('U')
	PORT_BIT(0x40,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("V") PORT_CODE(KEYCODE_V) PORT_CHAR('v') PORT_CHAR('V')
	PORT_BIT(0x80,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("W") PORT_CODE(KEYCODE_W) PORT_CHAR('w') PORT_CHAR('W')

	PORT_START("row7")
	PORT_BIT(0x01,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("X") PORT_CODE(KEYCODE_X) PORT_CHAR('x') PORT_CHAR('X')
	PORT_BIT(0x02,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("Y") PORT_CODE(KEYCODE_Y) PORT_CHAR('y') PORT_CHAR('Y')
	PORT_BIT(0x04,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("Z") PORT_CODE(KEYCODE_Z) PORT_CHAR('z') PORT_CHAR('Z')
	PORT_BIT(0x08,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("[ {") PORT_CODE(KEYCODE_OPENBRACE) PORT_CHAR('[') PORT_CHAR('{')
	PORT_BIT(0x10,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("\\ |") PORT_CODE(KEYCODE_BACKSLASH) PORT_CHAR('\\') PORT_CHAR('|')
	PORT_BIT(0x20,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("] }") PORT_CODE(KEYCODE_CLOSEBRACE) PORT_CHAR(']') PORT_CHAR('}')
	PORT_BIT(0x40,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("- _") PORT_CODE(KEYCODE_MINUS) PORT_CHAR('-') PORT_CHAR('_')
	PORT_BIT(0x80,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("DEL") PORT_CODE(KEYCODE_DEL) PORT_CHAR(UCHAR_MAMEKEY(DEL))

	PORT_START("modifiers")
	PORT_BIT(0x01,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("Shift") PORT_CODE(KEYCODE_LSHIFT) PORT_CHAR(UCHAR_SHIFT_1)
	PORT_BIT(0x02,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("Ctrl") PORT_CODE(KEYCODE_LCONTROL) PORT_CHAR(UCHAR_SHIFT_2)
	PORT_BIT(0x04,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("Shift") PORT_CODE(KEYCODE_RSHIFT) PORT_CHAR(UCHAR_SHIFT_1)
    PORT_BIT(0x08,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("CAPS LOCK") PORT_CODE(KEYCODE_CAPSLOCK) PORT_CHAR(UCHAR_MAMEKEY(CAPSLOCK))
	PORT_BIT(0x10,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("Ctrl") PORT_CODE(KEYCODE_RCONTROL) PORT_CHAR(UCHAR_SHIFT_2)
INPUT_PORTS_END

// IRQ daisy chain = CTC -> SIO -> Expansion
static const z80_daisy_config attache_daisy_chain[] =
{
	{ "ctc" },
	{ "sio" },
    // expansion bus
	{ nullptr }
};

static const z80_daisy_config attache_daisy_chain_expansion[] =
{
	{ "ctc" },
	{ "sio" },
    { "pio2" }, // expansion bus
	{ nullptr }
};

static void attache_floppies(device_slot_interface &device)
{
	device.option_add("525dd", FLOPPY_525_DD);
}

void attache_state::driver_start()
{
	uint8_t *RAM = m_ram->pointer();

	m_membank1->configure_entries(0, 8, &RAM[0x0000], 0x2000);
	m_membank2->configure_entries(0, 8, &RAM[0x0000], 0x2000);
	m_membank3->configure_entries(0, 8, &RAM[0x0000], 0x2000);
	m_membank4->configure_entries(0, 8, &RAM[0x0000], 0x2000);
	m_membank5->configure_entries(0, 8, &RAM[0x0000], 0x2000);
	m_membank6->configure_entries(0, 8, &RAM[0x0000], 0x2000);
	m_membank7->configure_entries(0, 8, &RAM[0x0000], 0x2000);
	m_membank8->configure_entries(0, 8, &RAM[0x0000], 0x2000);

	m_membank1->set_entry(0);
	m_membank2->set_entry(1);
	m_membank3->set_entry(2);
	m_membank4->set_entry(3);
	m_membank5->set_entry(4);
	m_membank6->set_entry(5);
	m_membank7->set_entry(6);
	m_membank8->set_entry(7);

	memset(RAM,0,65536);

	m_nvram->set_base(m_cmos_ram,64);

	m_maincpu->space(AS_PROGRAM).install_readwrite_handler(0x0000,0x0fff, read8sm_delegate(*this, FUNC(attache_state::rom_r)), write8sm_delegate(*this, FUNC(attache_state::rom_w)));

	save_pointer(m_char_ram,"Character RAM",128*32);
	save_pointer(m_attr_ram,"Attribute RAM",128*32);
	save_pointer(m_gfx_ram,"Graphics RAM",128*32*5);
	save_pointer(m_cmos_ram,"CMOS RAM",64);
}

void attache_state::machine_start()
{
	// initialise RAM
	memset(m_cmos_ram,0,64);
	memset(m_attr_ram,0,128*32);
	memset(m_char_ram,0,128*32);
	memset(m_gfx_ram,0,128*32*5);
}

void attache_state::machine_reset()
{
	m_kb_bitpos = 0;
}

void attache816_state::machine_reset()
{
	attache_state::machine_reset();
}
    
void attache_state::attache(machine_config &config)
{
	Z80(config, m_maincpu, 8_MHz_XTAL / 2);
	m_maincpu->set_addrmap(AS_PROGRAM, &attache_state::attache_map);
	m_maincpu->set_addrmap(AS_IO, &attache_state::attache_io);
	m_maincpu->set_daisy_config(attache_daisy_chain);

	config.set_maximum_quantum(attotime::from_hz(60));

	screen_device &screen(SCREEN(config, "screen", SCREEN_TYPE_RASTER, rgb_t::green()));
	screen.set_raw(12.324_MHz_XTAL, 784, 0, 640, 262, 0, 240);
	screen.set_screen_update(FUNC(attache_state::screen_update));

	PALETTE(config, m_palette, palette_device::MONOCHROME_HIGHLIGHT);

	SPEAKER(config, "mono").front_center();
	AY8912(config, m_psg, 8_MHz_XTAL / 4);
	m_psg->add_route(ALL_OUTPUTS, "mono", 0.25);

	MSM5832(config, m_rtc, 32.768_kHz_XTAL);

	Z80PIO(config, m_pio, 8_MHz_XTAL / 2);
	m_pio->in_pa_callback().set(FUNC(attache_state::pio_portA_r));
	m_pio->out_pa_callback().set(FUNC(attache_state::pio_portA_w));
	m_pio->in_pb_callback().set(FUNC(attache_state::pio_portB_r));
	m_pio->out_pb_callback().set(FUNC(attache_state::pio_portB_w));

	Z80SIO(config, m_sio, 8_MHz_XTAL / 2);
	m_sio->out_txda_callback().set("rs232a", FUNC(rs232_port_device::write_txd));
	m_sio->out_rtsa_callback().set("rs232a", FUNC(rs232_port_device::write_rts));
	m_sio->out_txdb_callback().set("rs232b", FUNC(rs232_port_device::write_txd));
	m_sio->out_rtsb_callback().set("rs232b", FUNC(rs232_port_device::write_rts));
	m_sio->out_int_callback().set_inputline(m_maincpu, INPUT_LINE_IRQ0);

	rs232_port_device &rs232a(RS232_PORT(config, "rs232a", default_rs232_devices, nullptr));
	rs232a.rxd_handler().set(m_sio, FUNC(z80sio_device::rxa_w));
	rs232a.cts_handler().set(m_sio, FUNC(z80sio_device::ctsa_w));

	rs232_port_device &rs232b(RS232_PORT(config, "rs232b", default_rs232_devices, nullptr));
	rs232b.rxd_handler().set(m_sio, FUNC(z80sio_device::rxb_w));
	rs232b.cts_handler().set(m_sio, FUNC(z80sio_device::ctsb_w));

	Z80CTC(config, m_ctc, 8_MHz_XTAL / 2);
	m_ctc->set_clk<0>(8_MHz_XTAL / 26); // 307.692 KHz
	m_ctc->set_clk<1>(8_MHz_XTAL / 26); // 307.692 KHz
	m_ctc->zc_callback<0>().set(m_sio, FUNC(z80sio_device::rxca_w));
	m_ctc->zc_callback<0>().append(m_sio, FUNC(z80sio_device::txca_w));
	m_ctc->zc_callback<1>().set(m_sio, FUNC(z80sio_device::rxtxcb_w));
	m_ctc->intr_callback().set_inputline(m_maincpu, INPUT_LINE_IRQ0);

	AM9517A(config, m_dma, 8_MHz_XTAL / 4);
	m_dma->out_hreq_callback().set(FUNC(attache_state::hreq_w));
	m_dma->out_eop_callback().set(FUNC(attache_state::eop_w));
	m_dma->in_memr_callback().set(FUNC(attache_state::dma_mem_r));
	m_dma->out_memw_callback().set(FUNC(attache_state::dma_mem_w));
	m_dma->in_ior_callback<0>().set(FUNC(attache_state::fdc_dma_r));
	m_dma->out_iow_callback<0>().set(FUNC(attache_state::fdc_dma_w));
	// m_dma->out_dack_callback<0>().set(FUNC(attache_state::fdc_dack_w));

    // channel 1 is SIOA
    // channel 2 is Expansion bus
    // channel 3 is SIOB

	UPD765A(config, m_fdc, 8_MHz_XTAL, true, true);
	m_fdc->intrq_wr_callback().set(m_ctc, FUNC(z80ctc_device::trg3));
	m_fdc->drq_wr_callback().set(m_dma, FUNC(am9517a_device::dreq0_w)).invert();
	FLOPPY_CONNECTOR(config, "fdc:0", attache_floppies, "525dd", floppy_image_device::default_mfm_floppy_formats).enable_sound(true);
	FLOPPY_CONNECTOR(config, "fdc:1", attache_floppies, "525dd", floppy_image_device::default_mfm_floppy_formats).enable_sound(true);

	TMS9927(config, m_crtc, 12.324_MHz_XTAL / 8);
	m_crtc->set_char_width(8);
	m_crtc->vsyn_callback().set(m_ctc, FUNC(z80ctc_device::trg2));
	m_crtc->set_screen("screen");

	NVRAM(config, "nvram", nvram_device::DEFAULT_ALL_0);

	RAM(config, RAM_TAG).set_default_size("64K");

	SOFTWARE_LIST(config, "disk_list").set_original("attache");
}

void attache816_state::attache816(machine_config &config)
{
    attache_state::attache(config);
	m_maincpu->set_addrmap(AS_IO, &attache816_state::attache_io);
	m_maincpu->set_daisy_config(attache_daisy_chain_expansion);

	I8086(config, m_extcpu, 24_MHz_XTAL / 3);
	m_extcpu->set_addrmap(AS_PROGRAM, &attache816_state::attache_x86_map);
	m_extcpu->set_addrmap(AS_IO, &attache816_state::attache_x86_io);
	config.set_perfect_quantum(m_extcpu);

	I8255A(config, m_ppi, 0);
	m_ppi->out_pa_callback().set(FUNC(attache816_state::x86_comms_w));
	m_ppi->in_pa_callback().set(FUNC(attache816_state::x86_comms_r));
	m_ppi->out_pb_callback().set(FUNC(attache816_state::x86_irq_enable));
    
	Z80PIO(config, m_pio2, 8_MHz_XTAL / 2);
	m_pio2->in_pa_callback().set(FUNC(attache816_state::pio2_portA_r));
	m_pio2->in_pb_callback().set(FUNC(attache816_state::pio2_portB_r));
    m_pio2->out_int_callback().set_inputline(m_maincpu, INPUT_LINE_IRQ0);
}

ROM_START( attache )
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_FILL(0x0000,0x10000,0x00)

	ROM_REGION(0x1000, "boot", 0)
	ROM_SYSTEM_BIOS(0, "u252revh", "Boot Rev.H")
	ROMX_LOAD("u252revh.bin", 0x0000, 0x1000, CRC(a06f0bdf) SHA1(d526cf23bfe0f8f9bcde812cd864a2a4cbc8b673), ROM_BIOS(0))
	ROM_SYSTEM_BIOS(1, "u252revg", "Boot Rev.G")
	ROMX_LOAD("u252revg.bin", 0x0000, 0x1000, CRC(113136b7) SHA1(845afd9ed2fd2b28c39921d8f2ba99e5295e0330), ROM_BIOS(1))
	ROM_SYSTEM_BIOS(2, "u252revf", "Boot Rev.F")
	ROMX_LOAD("u252revf.bin", 0x0000, 0x1000, CRC(b49eb3b2) SHA1(5b1b348301b2f76b1f250ba68bb8733fc15d18c2), ROM_BIOS(2))

	ROM_REGION(0x1000, "video", 0)
	ROM_LOAD("u416vid.bin",  0x0000, 0x1000, CRC(e376ec59) SHA1(7b9e9db575e77ce2f479eb9ae913528e4f0d125d) )

	ROM_REGION(0x100, "attr", 0)
	ROM_LOAD("u413.bin",  0x0000, 0x0100, CRC(5b60e622) SHA1(43450c747db1394466eabe5c26a61bf75a4f3b52) )

	ROM_REGION(0x200, "iosel", 0)
	ROM_LOAD("u110.bin",  0x0000, 0x0200, CRC(70dd255a) SHA1(36dcce07a2c14eefc069433459c422341bd47efb) )

	ROM_REGION(0x100, "floppy", 0)
	ROM_LOAD("u630.bin",  0x0000, 0x0100, CRC(f7a5c821) SHA1(fea07d9ac7e4e5f4f72aa7b2159deaedbd662ead) )

ROM_END

ROM_START( attache816 )
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_FILL(0x0000,0x10000,0x00)

	ROM_REGION(0x1000, "boot", 0)
	ROM_SYSTEM_BIOS(0, "u252revh", "Boot Rev.H")
	ROMX_LOAD("u252revh.bin", 0x0000, 0x1000, CRC(a06f0bdf) SHA1(d526cf23bfe0f8f9bcde812cd864a2a4cbc8b673), ROM_BIOS(0))
	ROM_SYSTEM_BIOS(1, "u252revg", "Boot Rev.G")
	ROMX_LOAD("u252revg.bin", 0x0000, 0x1000, CRC(113136b7) SHA1(845afd9ed2fd2b28c39921d8f2ba99e5295e0330), ROM_BIOS(1))
	ROM_SYSTEM_BIOS(2, "u252revf", "Boot Rev.F")
	ROMX_LOAD("u252revf.bin", 0x0000, 0x1000, CRC(b49eb3b2) SHA1(5b1b348301b2f76b1f250ba68bb8733fc15d18c2), ROM_BIOS(2))

	ROM_REGION(0x2000, "video", 0)
	ROM_LOAD("u416vid2.bin",  0x0000, 0x2000, CRC(0bdaed8d) SHA1(eee1e8505906e7c3587ecdf9dd9227a2a3b3cdd4) )

	ROM_REGION(0x100, "attr", 0)
	ROM_LOAD("u413.bin",  0x0000, 0x0100, CRC(5b60e622) SHA1(43450c747db1394466eabe5c26a61bf75a4f3b52) )

	ROM_REGION(0x200, "iosel", 0)
	ROM_LOAD("u110.bin",  0x0000, 0x0200, CRC(70dd255a) SHA1(36dcce07a2c14eefc069433459c422341bd47efb) )

	ROM_REGION(0x100, "floppy", 0)
	ROM_LOAD("u630.bin",  0x0000, 0x0100, CRC(f7a5c821) SHA1(fea07d9ac7e4e5f4f72aa7b2159deaedbd662ead) )

	// chip locations based on schematics
	ROM_REGION16_LE(0x2000, "x86bios", 0)
	ROM_LOAD16_BYTE("u4.bin",  0x0000, 0x1000, CRC(658c8f93) SHA1(ce4b388af5b73884194f548afa706964305462f7) )
	ROM_LOAD16_BYTE("u9.bin",  0x0001, 0x1000, CRC(cc4cd938) SHA1(6a1d316628641f9b4de5c8c46f9430ef5bd6120f) )

ROM_END

//    YEAR  NAME        PARENT   COMPAT  MACHINE     INPUT    CLASS             INIT        COMPANY   FULLNAME               FLAGS
COMP( 1982, attache,    0,       0,      attache,    attache, attache_state,    empty_init, "Otrona", "Attach\xC3\xA9",      0 )
COMP( 1983, attache816, attache, 0,      attache816, attache, attache816_state, empty_init, "Otrona", "Attach\xC3\xA9 8:16", MACHINE_IMPERFECT_GRAPHICS )
