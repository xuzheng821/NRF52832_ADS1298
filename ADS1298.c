#include "ADS1298.h"
#include "nrf_drv_spi.h"
#include "nrf_log.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"

#define SPI_INSTANCE 0                                               /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE); /**< SPI instance. */
static volatile bool spi_xfer_done;                                  /**< Flag used to indicate that SPI instance completed the transfer. */

#define ADS1298_SS_LOW nrf_gpio_pin_clear(ADS1298_SS_PIN)
#define ADS1298_SS_HIGH nrf_gpio_pin_set(ADS1298_SS_PIN)

void spi_event_handler(nrf_drv_spi_evt_t const *p_event, void *p_context)
{
    spi_xfer_done = true;
}

void ads1298_spi_init(void)
{
    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;

    spi_config.irq_priority = SPI_DEFAULT_CONFIG_IRQ_PRIORITY;
    spi_config.ss_pin = NRF_DRV_SPI_PIN_NOT_USED;
    spi_config.miso_pin = ADS1298_MISO_PIN;
    spi_config.mosi_pin = ADS1298_MOSI_PIN;
    spi_config.sck_pin = ADS1298_SCK_PIN;
    spi_config.mode = NRF_DRV_SPI_MODE_1;
    spi_config.frequency = NRF_DRV_SPI_FREQ_8M;
    spi_config.orc = 0xff;
		spi_config.bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));
	
		nrf_gpio_cfg_output(ADS1298_SS_PIN);
		ADS1298_SS_HIGH;
}

static void ads1298_spi_send(uint8_t send)
{
		spi_xfer_done = false;
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, &send, 1, NULL, 0));
		
		nrf_delay_us(2);
	
    while (!spi_xfer_done)
        __WFE();
}

static uint8_t ads1298_spi_recv(void)
{
		uint8_t ret = 0;
	
		spi_xfer_done = false;
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, NULL, 0, &ret, 1));
		
		nrf_delay_us(2);
	
    while (!spi_xfer_done)
        __WFE();
		
		return ret;
}

bool ads1298_read_register(uint8_t* val, uint8_t address, uint8_t num)
{

		ADS1298_SS_LOW;
		
		ads1298_spi_send(ADS129X_CMD_RREG | (address & 0x1F));
		ads1298_spi_send((num - 1) & 0x1F);
		
		for(int i = 0; i < num; i++)
			*(val + i) = ads1298_spi_recv();
	
		ADS1298_SS_HIGH;

    return true;
		
}

bool ads1298_write_register(uint8_t* val, uint8_t address, uint8_t num)
{
	
		ADS1298_SS_LOW;
		
		ads1298_spi_send(ADS129X_CMD_WREG | (address & 0x1F));
		ads1298_spi_send((num - 1) & 0x1F);
		
		for(int i = 0; i < num; i++)
			ads1298_spi_send(*(val + i));
	
		ADS1298_SS_HIGH;
		
		return true;
		
}

bool ads1298_write_command(uint8_t val)
{
	
    ADS1298_SS_LOW;
		
		ads1298_spi_send(val);
	
		ADS1298_SS_HIGH;
		
}
