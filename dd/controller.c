//
// dd/controller.c: DD controller.
//
// CEN64: Cycle-Accurate Nintendo 64 Simulator.
// Copyright (C) 2014, Tyler J. Stachecki.
//
// This file is subject to the terms and conditions defined in
// 'LICENSE', which is part of this source code package.
//

//
// Thanks go out to OzOnE and Luigiblood (Seru-kun) for reverse
// engineering, documenting, and assisting with the reversal of
// this device!
//

//
// LuigiBlood's notes:
// Thanks to Happy_ for figuring out a lot of the 64DD stuff.
//

//
// TODO: Currently, the DD IPL spams the controller with DD_CMD_NOOP.
// This is normal. Once you signify that a disk is present (using the
// DD_STATUS_DISK_PRES), the DD IPL attempts to start performing seeks.
//

#include "common.h"
#include "bus/address.h"
#include "bus/controller.h"
#include "dd/controller.h"
#include "vr4300/interface.h"
#include <time.h>

#ifdef DEBUG_MMIO_REGISTER_ACCESS
const char *dd_register_mnemonics[NUM_DD_REGISTERS] = {
#define X(reg) #reg,
#include "dd/registers.md"
#undef X
};
#endif

// ASIC_CMD_STATUS flags.
#define DD_CMD_NOOP           0x00000000U
#define DD_CMD_SEEK_READ      0x00010000U //Disk needed
#define DD_CMD_SEEK_WRITE     0x00020000U //Disk needed
#define DD_CMD_RECALIBRATE    0x00030000U // ??? Disk needed
#define DD_CMD_SLEEP          0x00040000U
#define DD_CMD_START          0x00050000U //Disk needed
#define DD_CMD_SET_STANDBY    0x00060000U
#define DD_CMD_SET_SLEEP      0x00070000U
#define DD_CMD_CLR_DSK_CHNG   0x00080000U
#define DD_CMD_CLR_RESET      0x00090000U
#define DD_CMD_READ_VERSION   0x000A0000U
#define DD_CMD_SET_DISK_TYPE  0x000B0000U //Disk needed
#define DD_CMD_REQUEST_STATUS 0x000C0000U
#define DD_CMD_STANDBY        0x000D0000U
#define DD_CMD_IDX_LOCK_RETRY 0x000E0000U // ???
#define DD_CMD_SET_YEAR_MONTH 0x000F0000U
#define DD_CMD_SET_DAY_HOUR   0x00100000U
#define DD_CMD_SET_MIN_SEC    0x00110000U
#define DD_CMD_GET_YEAR_MONTH 0x00120000U
#define DD_CMD_GET_DAY_HOUR   0x00130000U
#define DD_CMD_GET_MIN_SEC    0x00140000U
#define DD_CMD_FEATURE_INQ    0x001B0000U

#define DD_STATUS_DATA_RQ     0x40000000U
#define DD_STATUS_C2_XFER     0x10000000U
#define DD_STATUS_BM_ERR      0x08000000U
#define DD_STATUS_BM_INT      0x04000000U
#define DD_STATUS_MECHA_INT   0x02000000U
#define DD_STATUS_DISK_PRES   0x01000000U
#define DD_STATUS_BUSY_STATE  0x00800000U
#define DD_STATUS_RST_STATE   0x00400000U
#define DD_STATUS_MTR_N_SPIN  0x00100000U
#define DD_STATUS_HEAD_RTRCT  0x00080000U
#define DD_STATUS_WR_PR_ERR   0x00040000U
#define DD_STATUS_MECHA_ERR   0x00020000U
#define DD_STATUS_DISK_CHNG   0x00010000U

// ASIC_BM_STATUS_CTL flags.
#define DD_BM_STATUS_RUNNING  0x80000000U
#define DD_BM_STATUS_ERROR    0x04000000U
#define DD_BM_STATUS_MICRO    0x02000000U // ???
#define DD_BM_STATUS_BLOCK    0x01000000U
#define DD_BM_STATUS_C1CRR    0x00800000U
#define DD_BM_STATUS_C1DBL    0x00400000U
#define DD_BM_STATUS_C1SNG    0x00200000U
#define DD_BM_STATUS_C1ERR    0x00010000U // Typo ???

#define DD_BM_CTL_START       0x80000000U
#define DD_BM_CTL_MNGRMODE    0x40000000U
#define DD_BM_CTL_INTMASK     0x20000000U
#define DD_BM_CTL_RESET       0x10000000U
#define DD_BM_CTL_DIS_OR_CHK  0x08000000U // ???
#define DD_BM_CTL_DIS_C1_CRR  0x04000000U
#define DD_BM_CTL_BLK_TRANS   0x02000000U
#define DD_BM_CTL_MECHA_RST   0x01000000U

#define SECTORS_PER_BLOCK     85
#define BLOCKS_PER_TRACK      2

bool dd_reset_hold;		//RESET HOLD
bool dd_bm_mode_read;	//BM MODE 0 (false) = WRITE, MODE 1 (true) = READ
int	 LBA;				//GET LBA
int  CUR_BLOCK;			//Current Block
int  dd_zone;			//Current Zone
int  dd_track_offset;	//Offset to Track
bool dd_sector55;

const unsigned int ddZoneSecSize[16] = {232,216,208,192,176,160,144,128,
                                        216,208,192,176,160,144,128,112};

const unsigned int ddZoneTrackSize[16] = {158,158,149,149,149,149,149,114,
                                          158,158,149,149,149,149,149,114};

const unsigned int ddStartOffset[16] = 
	{0x0,0x5F15E0,0xB79D00,0x10801A0,0x1523720,0x1963D80,0x1D414C0,0x20BBCE0,
	 0x23196E0,0x28A1E00,0x2DF5DC0,0x3299340,0x36D99A0,0x3AB70E0,0x3E31900,0x4149200};

//-------------------------------------
void dd_clear_c2s(void *opaque)
{
  struct dd_controller *dd = (struct dd_controller *) opaque;

  dd->regs[DD_ASIC_CMD_STATUS] &= ~DD_STATUS_C2_XFER;
  dd->regs[DD_ASIC_CMD_STATUS] &= ~DD_STATUS_BM_INT;
  clear_dd_interrupt(dd->bus->vr4300);
}

void dd_clear_ds(void *opaque)
{
  struct dd_controller *dd = (struct dd_controller *) opaque;

  dd->regs[DD_ASIC_CMD_STATUS] &= ~DD_STATUS_DATA_RQ;
  dd->regs[DD_ASIC_CMD_STATUS] &= ~DD_STATUS_BM_INT;
  clear_dd_interrupt(dd->bus->vr4300);
}

void dd_write_sector(void *opaque)
{
	struct dd_controller *dd = (struct dd_controller *) opaque;

	//WRITE SECTOR
	int Cur_Sector = dd->regs[DD_ASIC_CUR_SECTOR] >> 16;
	if (Cur_Sector >= 0x5A)
		Cur_Sector -= 0x5A;

	int offset = dd_track_offset;
	offset += CUR_BLOCK * SECTORS_PER_BLOCK * ddZoneSecSize[dd_zone];
	offset += (Cur_Sector - 1) * ddZoneSecSize[dd_zone];

	//for (int i = 0; i <= (int)(dd->regs[DD_ASIC_HOST_SECBYTE] >> 16); i++)
        //dd->rom[offset + i] = dd->ds_buffer[i];
}

void dd_read_sector(void *opaque)
{
	struct dd_controller *dd = (struct dd_controller *) opaque;

	//READ SECTOR
	int Cur_Sector = dd->regs[DD_ASIC_CUR_SECTOR] >> 16;
	if (Cur_Sector >= 0x5A)
		Cur_Sector -= 0x5A;

	int offset = dd_track_offset;
	offset += CUR_BLOCK * SECTORS_PER_BLOCK * ddZoneSecSize[dd_zone];
	offset += Cur_Sector * ddZoneSecSize[dd_zone];

	for (int i = 0; i <= (int)(dd->regs[DD_ASIC_HOST_SECBYTE] >> 16); i++)
        dd->ds_buffer[i] = dd->rom[offset + i];
}

void dd_read_C2(void *opaque)
{
	struct dd_controller *dd = (struct dd_controller *) opaque;

	for (int i = 0; i < DD_C2S_BUFFER_LEN; i++)
      dd->c2s_buffer[i] = 0;
}

void dd_update_bm(void *opaque)
{
	struct dd_controller *dd = (struct dd_controller *) opaque;

  if (!(dd->regs[DD_ASIC_BM_STATUS_CTL] & DD_BM_STATUS_RUNNING))
		return;
	else
	{
		int Cur_Sector = dd->regs[DD_ASIC_CUR_SECTOR] >> 16;
		if (Cur_Sector >= 0x5A)
    {
      CUR_BLOCK = 1;
			Cur_Sector -= 0x5A;
    }

    if (!dd_bm_mode_read)		//WRITE MODE
		{
			printf("--DD_UPDATE_BM WRITE Block %d Sector %X\n", ((dd->regs[DD_ASIC_CUR_TK] & 0x0FFF0000U) >> 15) + CUR_BLOCK, Cur_Sector);

			if (Cur_Sector == 0)
			{
				Cur_Sector++;
				dd->regs[DD_ASIC_CMD_STATUS] |= DD_STATUS_DATA_RQ;
			}
			else if (Cur_Sector < SECTORS_PER_BLOCK)
			{
				dd_write_sector(opaque);
				Cur_Sector++;
				dd->regs[DD_ASIC_CMD_STATUS] |= DD_STATUS_DATA_RQ;
			}
			else if (Cur_Sector < SECTORS_PER_BLOCK + 1)
			{
				if (dd->regs[DD_ASIC_BM_STATUS_CTL] & DD_BM_STATUS_BLOCK)
				{
					dd_write_sector(opaque);
					//next block
					Cur_Sector = 1;
					CUR_BLOCK = 1 - CUR_BLOCK;
					dd->regs[DD_ASIC_BM_STATUS_CTL] &= ~DD_BM_STATUS_BLOCK;
					dd->regs[DD_ASIC_CMD_STATUS] |= DD_STATUS_DATA_RQ;
				}
				else
				{
					dd_write_sector(opaque);
					Cur_Sector++;
					dd->regs[DD_ASIC_BM_STATUS_CTL] &= ~DD_BM_STATUS_RUNNING;
				}
			}
		}
		else						//READ MODE
		{
			printf("--DD_UPDATE_BM READ Block %d Sector %X\n", ((dd->regs[DD_ASIC_CUR_TK] & 0x0FFF0000U) >> 15) + CUR_BLOCK, Cur_Sector);

			int Cur_Track = (dd->regs[DD_ASIC_CUR_TK] & 0x0FFF0000U) >> 16;

      dd->regs[DD_ASIC_CMD_STATUS] &= ~(DD_STATUS_DATA_RQ | DD_STATUS_C2_XFER);

      if (!dd_sector55 && Cur_Sector == 0x59)
      {
        dd_sector55 = true;
        Cur_Sector--;
      }

			if (Cur_Track == 6 && CUR_BLOCK == 0 && dd->ipl_rom != NULL)
			{
				dd->regs[DD_ASIC_CMD_STATUS] &= ~DD_STATUS_DATA_RQ;
				//dd->regs[DD_ASIC_BM_STATUS_CTL] |= DD_BM_STATUS_MICRO;
			}
			else if (Cur_Sector < SECTORS_PER_BLOCK)		//user sector
			{
				dd_read_sector(opaque);
				Cur_Sector++;
				dd->regs[DD_ASIC_CMD_STATUS] |= DD_STATUS_DATA_RQ;
			}
			else if (Cur_Sector < SECTORS_PER_BLOCK + 4)	//C2
			{
				Cur_Sector++;
				if (Cur_Sector == SECTORS_PER_BLOCK + 4)
				{
					dd_read_C2(opaque);
					dd->regs[DD_ASIC_CMD_STATUS] |= DD_STATUS_C2_XFER;
				}
			}
			else if (Cur_Sector == SECTORS_PER_BLOCK + 4)	//Gap
			{
        printf("SECTOR 0x59\n");
				if (dd->regs[DD_ASIC_BM_STATUS_CTL] & DD_BM_STATUS_BLOCK)
				{
					CUR_BLOCK = 1 - CUR_BLOCK;
					Cur_Sector = 0;
					dd->regs[DD_ASIC_BM_STATUS_CTL] &= ~DD_BM_STATUS_BLOCK;
				}
				else
					dd->regs[DD_ASIC_BM_STATUS_CTL] &= ~DD_BM_STATUS_RUNNING;
			}
		}

		dd->regs[DD_ASIC_CUR_SECTOR] = (Cur_Sector + (0x5A * CUR_BLOCK)) << 16;
		dd->regs[DD_ASIC_CMD_STATUS] |= DD_STATUS_BM_INT;
    signal_dd_interrupt(dd->bus->vr4300);
	}
}

void dd_set_zone_and_track_offset(void *opaque)
{
	struct dd_controller *dd = (struct dd_controller *) opaque;

	int head = ((dd->regs[DD_ASIC_CUR_TK] & 0x10000000U) >> 25);	//Head * 8
	int track = ((dd->regs[DD_ASIC_CUR_TK] & 0x0FFF0000U) >> 16);
	int tr_off;

	if(track >= 0x425)
	{
		dd_zone = 7 + head;
		tr_off = track - 0x425;
	}
	else if (track >= 0x390)
	{
		dd_zone = 6 + head;
		tr_off = track - 0x390;
	}
	else if (track >= 0x2FB)
	{
		dd_zone = 5 + head;
		tr_off = track - 0x2FB;
	}
	else if (track >= 0x266)
	{
		dd_zone = 4 + head;
		tr_off = track - 0x266;
	}
	else if (track >= 0x1D1)
	{
		dd_zone = 3 + head;
		tr_off = track - 0x1D1;
	}
	else if (track >= 0x13C)
	{
		dd_zone = 2 + head;
		tr_off = track - 0x13C;
	}
	else if (track >= 0x9E)
	{
		dd_zone = 1 + head;
		tr_off = track - 0x9E;
	}
	else
	{
		dd_zone = 0 + head;
		tr_off = track;
	}

	dd_track_offset = ddStartOffset[dd_zone] + tr_off*ddZoneSecSize[dd_zone]*SECTORS_PER_BLOCK*BLOCKS_PER_TRACK;
}
//------------------------------

// Initializes the DD.
int dd_init(struct dd_controller *dd, struct bus_controller *bus,
  const uint8_t *ddipl, const uint8_t *ddrom, size_t ddrom_size) {
  dd->bus = bus;
  dd->ipl_rom = ddipl;
  dd->rom = ddrom;
  dd->rom_size = ddrom_size;

  if (dd->ipl_rom != NULL)
    dd->regs[DD_ASIC_ID_REG] = 0x00030000U;
  else
    dd->regs[DD_ASIC_ID_REG] = 0x00040000U;

  dd->regs[DD_ASIC_CMD_STATUS] = DD_STATUS_MTR_N_SPIN | DD_STATUS_HEAD_RTRCT;

  //if (dd->rom_size == 0x3DEC800) //original dumps
  if (dd->rom_size > 0)
    dd->regs[DD_ASIC_CMD_STATUS] |= DD_STATUS_DISK_PRES;

  dd_reset_hold = false;
  dd_bm_mode_read = false;
  dd_sector55 = false;

  return 0;
}

// Reads a word from the DD MMIO register space.
int read_dd_regs(void *opaque, uint32_t address, uint32_t *word) {
  struct dd_controller *dd = (struct dd_controller *) opaque;
  unsigned offset = address - DD_REGS_BASE_ADDRESS;
  enum dd_register reg = (offset >> 2);

  *word = dd->regs[reg];
  debug_mmio_read(dd, dd_register_mnemonics[reg], *word);

  if (reg == DD_ASIC_CMD_STATUS)
  {
  	int Cur_Sector = dd->regs[DD_ASIC_CUR_SECTOR] >> 16;
	  if (Cur_Sector >= 0x5A)
		  Cur_Sector -= 0x5A;

    if ((dd->regs[DD_ASIC_CMD_STATUS] & DD_STATUS_BM_INT) && (SECTORS_PER_BLOCK < Cur_Sector))
    {
  	  dd->regs[DD_ASIC_CMD_STATUS] &= ~DD_STATUS_BM_INT;
  	  clear_dd_interrupt(dd->bus->vr4300);
  	  printf("DD_UPDATE_BM DD REG READ -");
  	  dd_update_bm(opaque);
    }
  }

  //*word = dd->regs[reg];

  //if (reg != DD_ASIC_CMD_STATUS)
    //debug_mmio_read(dd, dd_register_mnemonics[reg], *word);
  return 0;
}

// Writes a word to the DD MMIO register space.
int write_dd_regs(void *opaque, uint32_t address, uint32_t word, uint32_t dqm) {
  struct dd_controller *dd = (struct dd_controller *) opaque;
  unsigned offset = address - DD_REGS_BASE_ADDRESS;
  enum dd_register reg = (offset >> 2);

  debug_mmio_write(dd, dd_register_mnemonics[reg], word, dqm);

  //No matter what, the lower 16-bit is always ignored.
  word &= 0xFFFF0000U;

  // Command register written: do something.
  if (reg == DD_ASIC_CMD_STATUS) {

    time_t timer;
    struct tm * timeinfo;
    switch (word)
    {
    	case DD_CMD_GET_MIN_SEC:		// Get time [minute/second]:
    		//Get Time
    		time(&timer);
    		timeinfo = localtime(&timer);

    		//Put time in DATA as BCD
    		uint8_t min = (uint8_t)(((timeinfo->tm_min / 10) << 4) | (timeinfo->tm_min % 10));
    		uint8_t sec = (uint8_t)(((timeinfo->tm_sec / 10) << 4) | (timeinfo->tm_sec % 10));

    		dd->regs[DD_ASIC_DATA] = (min << 24) | (sec << 16);
    		break;

    	case DD_CMD_GET_DAY_HOUR:		// Get time [day/hour]:
			//Get Time
      		time(&timer);
      		timeinfo = localtime(&timer);

		    //Put time in DATA as BCD
      		uint8_t hour = (uint8_t)(((timeinfo->tm_hour / 10) << 4) | (timeinfo->tm_hour % 10));
      		uint8_t day = (uint8_t)(((timeinfo->tm_mday / 10) << 4) | (timeinfo->tm_mday % 10));

      		dd->regs[DD_ASIC_DATA] = (day << 24) | (hour << 16);
      		break;

      	case DD_CMD_GET_YEAR_MONTH:		// Get time [year/month]:
			//Get Time
      		time(&timer);
      		timeinfo = localtime(&timer);

      		//Put time in DATA as BCD
      		uint8_t year = (uint8_t)(((timeinfo->tm_year / 10) << 4) | (timeinfo->tm_year % 10));
      		uint8_t month = (uint8_t)((((timeinfo->tm_mon + 1) / 10) << 4) | ((timeinfo->tm_mon + 1) % 10));

      		dd->regs[DD_ASIC_DATA] = (year << 24) | (month << 16);
      		break;

      	case DD_CMD_CLR_DSK_CHNG:		//Clear Disk Change status bit
      		dd->regs[DD_ASIC_CMD_STATUS] &= ~DD_STATUS_DISK_CHNG;
      		break;

      	case DD_CMD_CLR_RESET:			//Clear Reset status bit
      		dd->regs[DD_ASIC_CMD_STATUS] &= ~DD_STATUS_RST_STATE;
      		break;

      	case DD_CMD_FEATURE_INQ:		//Feature Inquiry
      		dd->regs[DD_ASIC_DATA] = 0x00010000U;
      		break;

      	case DD_CMD_SLEEP:				//Sleep
      		dd->regs[DD_ASIC_CMD_STATUS] |= (DD_STATUS_MTR_N_SPIN | DD_STATUS_HEAD_RTRCT);
      		break;

      	case DD_CMD_STANDBY:			//Standby
      		dd->regs[DD_ASIC_CMD_STATUS] |= DD_STATUS_HEAD_RTRCT;
      		dd->regs[DD_ASIC_CMD_STATUS] &= ~DD_STATUS_MTR_N_SPIN;
      		break;

      	case DD_CMD_START:				//Start
      		dd->regs[DD_ASIC_CMD_STATUS] &= ~(DD_STATUS_MTR_N_SPIN | DD_STATUS_HEAD_RTRCT);
      		break;

      	case DD_CMD_SEEK_READ:			//SEEK READ
      		dd->regs[DD_ASIC_CUR_TK] = dd->regs[DD_ASIC_DATA] | 0x60000000U;
    		dd->regs[DD_ASIC_CMD_STATUS] &= ~(DD_STATUS_MTR_N_SPIN | DD_STATUS_HEAD_RTRCT);
    		dd_bm_mode_read = true;
    		dd_set_zone_and_track_offset(opaque);
    		printf("--READ\n");
    		break;

    	case DD_CMD_SEEK_WRITE:			//SEEK WRITE
      		dd->regs[DD_ASIC_CUR_TK] = dd->regs[DD_ASIC_DATA] | 0x60000000U;
    		dd->regs[DD_ASIC_CMD_STATUS] &= ~(DD_STATUS_MTR_N_SPIN | DD_STATUS_HEAD_RTRCT);
    		dd_bm_mode_read = false;
    		dd_set_zone_and_track_offset(opaque);
    		printf("--WRITE\n");
    		break;

    	case DD_CMD_RECALIBRATE:		//Recalibration
    		dd->regs[DD_ASIC_DATA] = 0;
    		break;

    	case DD_CMD_IDX_LOCK_RETRY:		//Index Lock Retry
    		dd->regs[DD_ASIC_CUR_TK] |= 0x60000000U;
    		break;
    }

    // Always signal an interrupt in response.
    dd->regs[DD_ASIC_CMD_STATUS] |= DD_STATUS_MECHA_INT;
    signal_dd_interrupt(dd->bus->vr4300);
  }

  // Buffer manager control request: handle it.
  else if (reg == DD_ASIC_BM_STATUS_CTL) {
    if (word & DD_BM_CTL_RESET)
      dd_reset_hold = true;

    if (!(word & DD_BM_CTL_RESET) && dd_reset_hold)
    {
      dd_reset_hold = false;
      dd->regs[DD_ASIC_BM_STATUS_CTL] = 0;
      dd->regs[DD_ASIC_CUR_SECTOR] = 0;
      dd->regs[DD_ASIC_CMD_STATUS] &= ~(DD_STATUS_BM_INT | DD_STATUS_BM_ERR | DD_STATUS_DATA_RQ | DD_STATUS_C2_XFER);
      CUR_BLOCK = 0;
	}

    if (word & DD_BM_CTL_MECHA_RST)
      dd->regs[DD_ASIC_CMD_STATUS] &= ~DD_STATUS_MECHA_INT;

    if (word & DD_BM_CTL_BLK_TRANS)
      dd->regs[DD_ASIC_BM_STATUS_CTL] |= DD_BM_STATUS_BLOCK;
    else
      dd->regs[DD_ASIC_BM_STATUS_CTL] &= ~DD_BM_STATUS_BLOCK;

    //SET SECTOR
    dd->regs[DD_ASIC_CUR_SECTOR] = word & 0x00FF0000U;
    if ((dd->regs[DD_ASIC_CUR_SECTOR] >> 16) < 0x5A)
    	CUR_BLOCK = 0;
    else
    	CUR_BLOCK = 1;

    if (!(dd->regs[DD_ASIC_CMD_STATUS] & DD_STATUS_BM_INT) && !(dd->regs[DD_ASIC_CMD_STATUS] & DD_STATUS_MECHA_INT))
      clear_dd_interrupt(dd->bus->vr4300);
    
    //START BM -----------------------
    if (word & DD_BM_CTL_START)
    {
      dd->regs[DD_ASIC_BM_STATUS_CTL] |= DD_BM_STATUS_RUNNING;
      dd_sector55 = false;
      printf("DD_UPDATE_BM START -");
      dd_update_bm(opaque);
    }
  }

  // This is done by the IPL and a lot of games. The only word
  // ever know to be written to this register is 0xAAAA0000.
  else if (reg == DD_ASIC_HARD_RESET) {
    assert(word == 0xAAAA0000 && "dd: Hard reset without magic word?");

    dd->regs[DD_ASIC_CMD_STATUS] |= DD_STATUS_RST_STATE;
  }

  else if ((reg == DD_ASIC_CUR_TK) |
    (reg == DD_ASIC_ERR_SECTOR) |
    (reg == DD_ASIC_CUR_SECTOR) |
    (reg == DD_ASIC_C1_S0) |
    (reg == DD_ASIC_C1_S2) |
    (reg == DD_ASIC_C1_S4) |
    (reg == DD_ASIC_C1_S6) |
    (reg == DD_ASIC_CUR_ADDR) |
    (reg == DD_ASIC_ID_REG) |
    (reg == DD_ASIC_TEST_REG))
  {
    // Do nothing. Not writable.
  }

  else {
    dd->regs[reg] &= ~dqm;
    dd->regs[reg] |= word;
  }

  return 0;
}

// Reads a word from the DD IPL ROM.
int read_dd_ipl_rom(void *opaque, uint32_t address, uint32_t *word) {
  uint32_t offset = address - DD_IPL_ROM_ADDRESS;
  struct dd_controller *dd = (struct dd_controller*) opaque;

  if (!dd->ipl_rom)
    memset(word, 0, sizeof(*word));

  else {
    memcpy(word, dd->ipl_rom + offset, sizeof(*word));
    *word = byteswap_32(*word);
  }

  //debug_mmio_read(dd, "DD_IPL_ROM", *word);
  return 0;
}

// Writes a word to the DD IPL ROM.
int write_dd_ipl_rom(void *opaque, uint32_t address, uint32_t word, uint32_t dqm) {
  assert(0 && "Attempt to write to DD IPL ROM.");
  return 0;
}

// Reads a word from the DD C2S buffer.
int read_dd_c2s_buffer(void *opaque, uint32_t address, uint32_t *word) {
  struct dd_controller *dd = (struct dd_controller *) opaque;
  unsigned offset = address - DD_C2S_BUFFER_ADDRESS;

  memcpy(word, dd->c2s_buffer + offset, sizeof(*word));
  *word = byteswap_32(*word);
/*
  if (offset == 0)
  {
  	dd->regs[DD_ASIC_CMD_STATUS] &= ~(DD_STATUS_BM_INT | DD_STATUS_BM_ERR | DD_STATUS_DATA_RQ | DD_STATUS_C2_XFER);
  	clear_dd_interrupt(dd->bus->vr4300);
  }

  if (offset == (((dd->regs[DD_ASIC_HOST_SECBYTE] >> 16) + 1) * 4) - 4)
  {
  	printf("DD_UPDATE_BM C2S BUFFER -");
	  dd_update_bm(opaque);
  }
*/
  if (offset == ((((dd->regs[DD_ASIC_HOST_SECBYTE] >> 16) + 1) * 0))
   || offset == ((((dd->regs[DD_ASIC_HOST_SECBYTE] >> 16) + 1) * 1))
   || offset == ((((dd->regs[DD_ASIC_HOST_SECBYTE] >> 16) + 1) * 2))
   || offset == ((((dd->regs[DD_ASIC_HOST_SECBYTE] >> 16) + 1) * 3)))
    debug_mmio_read(dd, "DD_C2S_BUFFER", *word);
  return 0;
}

// Writes a word to the DD C2S BUFFER.
int write_dd_c2s_buffer(void *opaque, uint32_t address, uint32_t word, uint32_t dqm) {
  struct dd_controller *dd = (struct dd_controller *) opaque;
  unsigned offset = address - DD_C2S_BUFFER_ADDRESS;

  debug_mmio_write(dd, "DD_C2S_BUFFER", word, dqm);
  return 0;
}

// Reads a word from the DD DS buffer.
int read_dd_ds_buffer(void *opaque, uint32_t address, uint32_t *word) {
  struct dd_controller *dd = (struct dd_controller *) opaque;
  unsigned offset = address - DD_DS_BUFFER_ADDRESS;

  memcpy(word, dd->ds_buffer + (address & 0xFC), sizeof(*word));
  *word = byteswap_32(*word);

  //(SECTORS_PER_BLOCK >= (dd->regs[DD_ASIC_CUR_SECTOR] >> 16))
/*
  if (offset == 0)
  {
  	dd->regs[DD_ASIC_CMD_STATUS] &= ~(DD_STATUS_BM_INT | DD_STATUS_BM_ERR | DD_STATUS_DATA_RQ | DD_STATUS_C2_XFER);
  	clear_dd_interrupt(dd->bus->vr4300);
  }

  if (offset == ((dd->regs[DD_ASIC_HOST_SECBYTE] >> 16) - 3))
  {
  	printf("DD_UPDATE_BM DS BUFFER -");
	  dd_update_bm(opaque);
  }
*/
  //debug_mmio_read(dd, "DD_DS_BUFFER", *word);
  return 0;
}

// Writes a word to the DD DS BUFFER.
int write_dd_ds_buffer(void *opaque, uint32_t address, uint32_t word, uint32_t dqm) {
  struct dd_controller *dd = (struct dd_controller *) opaque;
  unsigned offset = address - DD_DS_BUFFER_ADDRESS;

  for (int i = 0; i < 4; i++)
  	dd->ds_buffer[offset + i] = (uint8_t)((word >> (8 * i)) & 0xFF);

  debug_mmio_write(dd, "DD_DS_BUFFER", word, dqm);
  return 0;
}

// Reads a word from the DD MS RAM.
int read_dd_ms_ram(void *opaque, uint32_t address, uint32_t *word) {
  struct dd_controller *dd = (struct dd_controller *) opaque;
  unsigned offset = address - DD_MS_RAM_ADDRESS;

  memcpy(word, dd->ms_ram + offset, sizeof(*word));
  *word = byteswap_32(*word);

  debug_mmio_read(dd, "DD_MS_RAM", *word);
  return 0;
}

// Writes a word to the DD MS RAM.
int write_dd_ms_ram(void *opaque, uint32_t address, uint32_t word, uint32_t dqm) {
  struct dd_controller *dd = (struct dd_controller *) opaque;
  unsigned offset = address - DD_MS_RAM_ADDRESS;

  for (int i = 0; i < 4; i++)
  	dd->ms_ram[offset + i] = (uint8_t)((word >> (8 * i)) & 0xFF);

  //debug_mmio_write(dd, "DD_MS_RAM", word, dqm);
  return 0;
}

