#include "lsm6dso_platform.h"
#include "nrfx_twi.h"
#include "nrf_delay.h"
#include <string.h>

#define LSM6DSO_ADDR 0x6A 

extern const nrfx_twi_t m_twi;

int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
    // Create a buffer for the register address + data
    uint8_t data[len + 1];
    data[0] = reg;
    memcpy(&data[1], bufp, len);

    // In blocking mode (no handler), this returns NRF_SUCCESS only after completion
    ret_code_t err_code = nrfx_twi_tx(&m_twi, LSM6DSO_ADDR, data, len + 1, false);
    
    return (err_code == NRF_SUCCESS) ? 0 : -1;
}

int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
    // 1. Write the register address (use repeated start 'true')
    ret_code_t err_code = nrfx_twi_tx(&m_twi, LSM6DSO_ADDR, &reg, 1, true);
    if (err_code != NRF_SUCCESS) return -1;

    // 2. Read the data
    err_code = nrfx_twi_rx(&m_twi, LSM6DSO_ADDR, bufp, len);
    
    return (err_code == NRF_SUCCESS) ? 0 : -1;
}

void platform_delay_ms(void *handle, uint32_t ms)
{
    nrf_delay_ms(ms);
}