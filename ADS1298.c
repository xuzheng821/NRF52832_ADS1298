#include "ADS1298.h"
#include "nrf_drv_spi.h"
#include "nrf_log.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_ppi.h"

#define SPI_INSTANCE 0                                               /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE); /**< SPI instance. */
static volatile bool spi_xfer_done;                                  /**< Flag used to indicate that SPI instance completed the transfer. */

#define ADS1298_SS_LOW		nrf_drv_gpiote_clr_task_trigger(ADS1298_SS_PIN)
#define ADS1298_SS_HIGH 	nrf_drv_gpiote_set_task_trigger(ADS1298_SS_PIN)

uint8_t recvbuf[27];

nrf_ppi_channel_t spi_start_transfer_channel;
nrf_ppi_channel_t spi_end_transfer_channel;

static void spi_event_handler(nrf_drv_spi_evt_t const *p_event, void *p_context)
{
    spi_xfer_done = true;
		NRF_SPIM0->RXD.PTR = (uint32_t)&recvbuf;
	
		NRF_LOG_HEXDUMP_INFO(recvbuf, 27);
	
}

static void drdy_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{

}

void ads1298_spi_init(void)
{
		ret_code_t ret;
	
    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;

    spi_config.irq_priority = SPI_DEFAULT_CONFIG_IRQ_PRIORITY;
    spi_config.ss_pin = NRF_DRV_SPI_PIN_NOT_USED;
    spi_config.miso_pin = ADS1298_MISO_PIN;
    spi_config.mosi_pin = ADS1298_MOSI_PIN;
    spi_config.sck_pin = ADS1298_SCK_PIN;
    spi_config.mode = NRF_DRV_SPI_MODE_1;
    spi_config.frequency = NRF_DRV_SPI_FREQ_1M;
    spi_config.orc = 0xff;
		spi_config.bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;
    
		ret = nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL);
		APP_ERROR_CHECK(ret);
	
		ret = nrf_drv_gpiote_init();
	
		nrf_drv_gpiote_out_config_t ss_pin_config = GPIOTE_CONFIG_OUT_TASK_TOGGLE(true);
		ret = nrf_drv_gpiote_out_init(ADS1298_SS_PIN, &ss_pin_config);
		APP_ERROR_CHECK(ret);
		nrf_drv_gpiote_out_task_enable(ADS1298_SS_PIN);
		
		nrf_drv_gpiote_in_config_t drdy_pin_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
		drdy_pin_config.pull = NRF_GPIO_PIN_PULLUP;
		ret = nrf_drv_gpiote_in_init(ADS1298_DRDY_PIN, &drdy_pin_config, drdy_pin_handler);
		APP_ERROR_CHECK(ret);
		nrf_drv_gpiote_in_event_enable(ADS1298_DRDY_PIN, true);
		
		
		
		ret = nrf_drv_ppi_init();
		APP_ERROR_CHECK(ret);
		ret = nrf_drv_ppi_channel_alloc(&spi_start_transfer_channel);
		APP_ERROR_CHECK(ret);
		ret = nrf_drv_ppi_channel_assign(spi_start_transfer_channel, nrf_drv_gpiote_in_event_addr_get(ADS1298_DRDY_PIN), nrf_drv_gpiote_clr_task_addr_get(ADS1298_SS_PIN));
		APP_ERROR_CHECK(ret);
		ret = nrf_drv_ppi_channel_fork_assign(spi_start_transfer_channel, nrf_drv_spi_start_task_get(&spi));
		APP_ERROR_CHECK(ret);
		
		ret = nrf_drv_ppi_channel_alloc(&spi_end_transfer_channel);
		APP_ERROR_CHECK(ret);
		ret = nrf_drv_ppi_channel_assign(spi_end_transfer_channel, nrf_drv_spi_end_event_get(&spi), nrf_drv_gpiote_set_task_addr_get(ADS1298_SS_PIN));
		APP_ERROR_CHECK(ret);
}

void ads1298_ppi_recv_start(void){

		ret_code_t ret;
	
		NRF_SPIM0->TXD.MAXCNT = 0;
    NRF_SPIM0->RXD.MAXCNT = 27;
    NRF_SPIM0->TXD.LIST =	0;
    //NRF_SPIM0->TXD.PTR = NULL;
    NRF_SPIM0->RXD.LIST =	1;
    NRF_SPIM0->RXD.PTR = (uint32_t)&recvbuf;
	
		ret = nrf_drv_ppi_channel_enable(spi_start_transfer_channel);
		APP_ERROR_CHECK(ret);
		ret = nrf_drv_ppi_channel_enable(spi_end_transfer_channel);
		APP_ERROR_CHECK(ret);

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

uint8_t ads1298_read_register(uint8_t reg)
{
		uint8_t val = 0;
		ads1298_read_multiple_register(reg, &val, 1);
		return val;
}

bool ads1298_read_multiple_register(uint8_t reg, uint8_t* val, uint8_t num)
{

		ADS1298_SS_LOW;
		
		ads1298_spi_send(ADS129X_CMD_RREG | (reg & 0x1F));
		ads1298_spi_send((num - 1) & 0x1F);
		
		for(int i = 0; i < num; i++)
			*(val + i) = ads1298_spi_recv();
	
		ADS1298_SS_HIGH;

    return true;
		
}

bool ads1298_write_register(uint8_t reg, uint8_t val)
{
	
		ads1298_write_multiple_register(reg, &val, 1);
		
}

bool ads1298_write_multiple_register(uint8_t reg, uint8_t* val, uint8_t num)
{
	
		ADS1298_SS_LOW;
		
		ads1298_spi_send(ADS129X_CMD_WREG | (reg & 0x1F));
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
	
		return true;
		
}
