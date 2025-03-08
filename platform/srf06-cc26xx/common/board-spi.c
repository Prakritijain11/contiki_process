/*
 * Copyright (c) 2014, Texas Instruments Incorporated - http://www.ti.com/
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*---------------------------------------------------------------------------*/
/**
 * \addtogroup sensortag-cc26xx-spi
 * @{
 *
 * \file
 * Board-specific SPI driver common to the Sensortag and LaunchPad
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "ti-lib.h"
#include "board-spi.h"
#include "board.h"

#include <stdbool.h>

typedef struct _SPI_PINS
{
  uint32_t   u32_periph;
  uint32_t   u32_base;
  uint32_t   u32_cs;
  uint32_t   u32_clk;
  uint32_t   u32_mosi;
  uint32_t   u32_miso;
}ST_SPI_PINS;

static const ST_SPI_PINS mast_spiPins[BOARD_MAX_SPI] =
{
  //  SPI peripheral       SPI Base            CS pin                       CLK pin                         MOSI pin                      MISO pin

  {  PRCM_PERIPH_SSI0,     SSI0_BASE,   BOARD_IOID_CBRAM_CS,    BOARD_IOID_SPI_CLK_CBRAM,      BOARD_IOID_SPI_MOSI_CBRAM,   BOARD_IOID_SPI_MISO_CBRAM },
  {  PRCM_PERIPH_SSI1,     SSI1_BASE,   BOARD_IOID_SENSOR_CS,   BOARD_IOID_SPI_CLK_SENSOR,     BOARD_IOID_SPI_MOSI,         BOARD_IOID_SPI_MISO },

};

/*---------------------------------------------------------------------------*/
static bool
accessible(EN_BOARD_SPI en_spi)
{
  bool b_res = false;

  if ( BOARD_MAX_SPI > en_spi )
  {
      /* First, check the PD */
      if(ti_lib_prcm_power_domain_status(PRCM_DOMAIN_SERIAL)
       != PRCM_DOMAIN_POWER_ON) {
      return false;
      }

      if ( BOARD_SPI0 == en_spi )
      {
        /* Then check the 'run mode' clock gate */
        if(!(HWREG(PRCM_BASE + PRCM_O_SSICLKGR) & PRCM_SSICLKGR_CLK_EN_SSI0)) {
          return false;
        }
      } 
      else
      {
        /* Then check the 'run mode' clock gate */
        if(!(HWREG(PRCM_BASE + PRCM_O_SSICLKGR) & PRCM_SSICLKGR_CLK_EN_SSI1)) {
          return false;
        }
      }
      b_res = true;
  }
  return b_res;
}
/*---------------------------------------------------------------------------*/

bool 
board_spi_write(const uint8_t *buf, size_t len, EN_BOARD_SPI en_spi)
{
  bool b_res;

  if ( BOARD_MAX_SPI > en_spi )
  {
    if(accessible(en_spi) == false) {
      return false;
    }

    while(len > 0) {
      uint32_t ul;
      ti_lib_ssi_data_put(mast_spiPins[en_spi].u32_base, *buf);
      ti_lib_rom_ssi_data_get(mast_spiPins[en_spi].u32_base, &ul);
      len--;
      buf++;
    }
    b_res = true;
  }
  else
  {
    b_res = false;
  }

  return b_res;
}

/*---------------------------------------------------------------------------*/

bool
board_spi_read(uint8_t *buf, size_t len, EN_BOARD_SPI en_spi)
{
  bool b_res;

  if ( BOARD_MAX_SPI > en_spi )
  {
    if(accessible(en_spi) == false) {
      return false;
    }

    while(len > 0) {
      uint32_t ul;

      if(!ti_lib_rom_ssi_data_put_non_blocking(mast_spiPins[en_spi].u32_base, 0)) {
        /* Error */
        return false;
      }
      ti_lib_rom_ssi_data_get(mast_spiPins[en_spi].u32_base, &ul);
      *buf = (uint8_t)ul;
      len--;
      buf++;
    }
    b_res = true;
  }
  else
  {
    b_res = false;
  }
  return b_res;
}

/*---------------------------------------------------------------------------*/

void
board_spi_flush(EN_BOARD_SPI en_spi)
{
  if ( BOARD_MAX_SPI > en_spi )
  {
    if(accessible(en_spi) == false) {
      return;
    }

    uint32_t ul;
    while(ti_lib_rom_ssi_data_get_non_blocking(mast_spiPins[en_spi].u32_base, &ul));
  }
}
/*---------------------------------------------------------------------------*/

void
board_spi_open(uint32_t bit_rate, EN_BOARD_SPI en_spi, uint32_t u32_mode)
{
  uint32_t buf;

  if ( BOARD_MAX_SPI > en_spi )
  {
      /* First, make sure the SERIAL PD is on */
      ti_lib_prcm_power_domain_on(PRCM_DOMAIN_SERIAL);
      while((ti_lib_prcm_power_domain_status(PRCM_DOMAIN_SERIAL)
          != PRCM_DOMAIN_POWER_ON));


    /* Enable clock in active mode */
    ti_lib_rom_prcm_peripheral_run_enable(mast_spiPins[en_spi].u32_periph);
    ti_lib_prcm_load_set();
    while(!ti_lib_prcm_load_get());

    /* SPI configuration */
    ti_lib_ssi_int_disable(mast_spiPins[en_spi].u32_base, SSI_RXOR | SSI_RXFF | SSI_RXTO | SSI_TXFF);
    ti_lib_ssi_int_clear(mast_spiPins[en_spi].u32_base, SSI_RXOR | SSI_RXTO);

    ti_lib_rom_ssi_config_set_exp_clk(mast_spiPins[en_spi].u32_base, ti_lib_sys_ctrl_clock_get(),
                      u32_mode,
                      SSI_MODE_MASTER, bit_rate, 8);
    ti_lib_rom_ioc_pin_type_ssi_master(mast_spiPins[en_spi].u32_base, mast_spiPins[en_spi].u32_miso,
                       mast_spiPins[en_spi].u32_mosi, IOID_UNUSED, mast_spiPins[en_spi].u32_clk);

    ti_lib_ssi_enable(mast_spiPins[en_spi].u32_base);



    /* Get rid of residual data from SSI port */
    while(ti_lib_ssi_data_get_non_blocking(mast_spiPins[en_spi].u32_base, &buf));
  }
}
/*---------------------------------------------------------------------------*/
void
board_spi_close( EN_BOARD_SPI en_spi )
{

  if ( BOARD_MAX_SPI > en_spi )
  {
  /* Power down SSI0 */
    ti_lib_rom_prcm_peripheral_run_disable(mast_spiPins[en_spi].u32_periph);
    ti_lib_prcm_load_set();
    while(!ti_lib_prcm_load_get());

      /* shut down the SERIAL PD  */
      PRCMPowerDomainOff(PRCM_DOMAIN_SERIAL);
      while((PRCMPowerDomainStatus(PRCM_DOMAIN_SERIAL)
        != PRCM_DOMAIN_POWER_OFF));

    /* Restore pins to a low-consumption state */
    ti_lib_ioc_pin_type_gpio_input(mast_spiPins[en_spi].u32_miso);
    ti_lib_ioc_io_port_pull_set(mast_spiPins[en_spi].u32_miso, IOC_IOPULL_DOWN);

    ti_lib_ioc_pin_type_gpio_input(mast_spiPins[en_spi].u32_mosi);
    ti_lib_ioc_io_port_pull_set(mast_spiPins[en_spi].u32_mosi, IOC_IOPULL_DOWN);

    ti_lib_ioc_pin_type_gpio_input(mast_spiPins[en_spi].u32_clk);
    ti_lib_ioc_io_port_pull_set(mast_spiPins[en_spi].u32_clk, IOC_IOPULL_DOWN);
  }
}


bool
board_spi_read_write(const uint8_t *txbuf, uint8_t *rxbuf, size_t len, EN_BOARD_SPI en_spi)
{
  uint8_t u8_loop = 0;
  bool b_ret;


  if ( BOARD_MAX_SPI > en_spi )
  {
    while(len > 0) {
      uint32_t ul;
      ti_lib_ssi_data_put(mast_spiPins[en_spi].u32_base, *txbuf);
      ti_lib_ssi_data_get(mast_spiPins[en_spi].u32_base, &ul);
      rxbuf[u8_loop] = ul;
      len--;
      txbuf++;
      u8_loop++;
    }
    b_ret = true;
  }
  else
  {
    b_ret = false;
  }

  return b_ret;
}
/*---------------------------------------------------------------------------*/
/** @} */
