/**
 * \file
 * \brief ATCA Hardware abstraction layer for Linux using I2C.
 *
 * \copyright (c) 2015-2020 Microchip Technology Inc. and its subsidiaries.
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip software
 * and any derivatives exclusively with Microchip products. It is your
 * responsibility to comply with third party license terms applicable to your
 * use of third party software (including open source software) that may
 * accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
 * PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT,
 * SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE
 * OF ANY KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF
 * MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE
 * FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL
 * LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED
 * THE AMOUNT OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR
 * THIS SOFTWARE.
 */

#include <cryptoauthlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "atca_hal.h"

#define FTDIMPSSE_STATIC
#include "../../ftdi-mpsse/ftd2xx.h"
#include "../../ftdi-mpsse/libmpsse_i2c.h"

 /** \defgroup hal_ Hardware abstraction layer (hal_)
  *
  * \brief
  * These methods define the hardware abstraction layer for communicating with a CryptoAuth device
  *
    @{ */

typedef struct atca_i2c_host_s
{
    FT_HANDLE ftHandle;
    unsigned channel;
    int  ref_ct;
} atca_i2c_ftdi_host_t;


/** \brief HAL implementation of I2C init
 *
 * this implementation assumes I2C peripheral has been enabled by user. It only initialize an
 * I2C interface using given config.
 *
 *  \param[in] hal pointer to HAL specific data that is maintained by this HAL
 *  \param[in] cfg pointer to HAL specific configuration data that is used to initialize this HAL
 * \return ATCA_SUCCESS on success, otherwise an error code.
 */
ATCA_STATUS hal_i2c_init(ATCAIface iface, ATCAIfaceCfg* cfg)
{
    atca_i2c_ftdi_host_t* hal;
    FT_DEVICE_LIST_INFO_NODE devList;
    FT_STATUS status;

    if (iface == NULL || cfg == NULL)
        return ATCA_BAD_PARAM;

    if (iface->hal_data != NULL)
    {
        hal = (atca_i2c_ftdi_host_t*)iface->hal_data;

        // Assume the bus had already been initialized
        hal->ref_ct++;

        return ATCA_SUCCESS;
    }
    if ((iface->hal_data = malloc(sizeof(atca_i2c_ftdi_host_t))) == NULL)
        return ATCA_ALLOC_FAILURE;
    hal = (atca_i2c_ftdi_host_t*)iface->hal_data;

    hal->ref_ct = 1;                                 // buses are shared, this is the first instance
    hal->channel = (int)ATCA_IFACECFG_VALUE(cfg, atcai2c.bus); // 0-based logical bus number
    hal->ftHandle = NULL;

    Init_libMPSSE();
    status = I2C_GetChannelInfo(hal->channel, &devList);
    status = I2C_OpenChannel(hal->channel, &hal->ftHandle);
    if (status != FT_OK)
    {
        hal->ftHandle = NULL;
        return ATCA_NO_DEVICES;
    }
    return ATCA_SUCCESS;
}

/** \brief HAL implementation of I2C post init
 * \param[in] iface  instance
 * \return ATCA_SUCCESS on success, otherwise an error code.
 */
ATCA_STATUS hal_i2c_post_init(ATCAIface iface)
{
    (void)iface;
    return ATCA_SUCCESS;
}

/** \brief HAL implementation of I2C send
 * \param[in] iface         instance
 * \param[in] word_address  device transaction type
 * \param[in] txdata        pointer to space to bytes to send
 * \param[in] txlength      number of bytes to send
 * \return ATCA_SUCCESS on success, otherwise an error code.
 */
ATCA_STATUS hal_i2c_send(ATCAIface iface, uint8_t addr, uint8_t* txdata, int txlength)
{
    atca_i2c_ftdi_host_t* hal = (atca_i2c_ftdi_host_t*)atgetifacehaldat(iface);
    FT_STATUS status;
    DWORD xfer = 0;
    if (txlength <= 0)
        return ATCA_SUCCESS;


    if (hal == NULL || hal->ftHandle == NULL)
        return ATCA_NOT_INITIALIZED;
    status = I2C_DeviceWrite(hal->ftHandle, iface->mIfaceCFG->atcai2c.address >> 1, txlength, txdata, &xfer,
        I2C_TRANSFER_OPTIONS_START_BIT |
        I2C_TRANSFER_OPTIONS_STOP_BIT |
        I2C_TRANSFER_OPTIONS_BREAK_ON_NACK);

    if (status != FT_OK)
        return ATCA_COMM_FAIL;
    return ATCA_SUCCESS;
}

/** \brief HAL implementation of I2C receive function
 * \param[in]    iface          Device to interact with.
 * \param[in]    address        device address
 * \param[out]   rxdata         Data received will be returned here.
 * \param[in,out] rxlength      As input, the size of the rxdata buffer.
 *                              As output, the number of bytes received.
 * \return ATCA_SUCCESS on success, otherwise an error code.
 */
ATCA_STATUS hal_i2c_receive(ATCAIface iface, uint8_t addr, uint8_t* rxdata, uint16_t* rxlength)
{
    atca_i2c_ftdi_host_t* phal = (atca_i2c_ftdi_host_t*)atgetifacehaldat(iface);

    if (phal == NULL || phal->ftHandle == NULL)
        return ATCA_NOT_INITIALIZED;

    FT_STATUS status;
    DWORD xfer = 0;

    /* Repeated Start condition generated. */
    status = I2C_DeviceRead(phal->ftHandle, iface->mIfaceCFG->atcai2c.address >> 1, *rxlength, rxdata, &xfer,
        I2C_TRANSFER_OPTIONS_START_BIT |
        I2C_TRANSFER_OPTIONS_STOP_BIT |
        I2C_TRANSFER_OPTIONS_NACK_LAST_BYTE);
    *rxlength = (uint16_t)xfer;
    if (status == FT_OK)
        return ATCA_SUCCESS;
    return ATCA_COMM_FAIL;
}

/** \brief Perform control operations for the kit protocol
 * \param[in]     iface          Interface to interact with.
 * \param[in]     option         Control parameter identifier
 * \param[in]     param          Optional pointer to parameter value
 * \param[in]     paramlen       Length of the parameter
 * \return ATCA_SUCCESS on success, otherwise an error code.
 */
ATCA_STATUS hal_i2c_control(ATCAIface iface, uint8_t option, void* param, size_t paramlen)
{
    (void)option;
    (void)param;
    (void)paramlen;

    if ((NULL != iface) && (NULL != iface->mIfaceCFG))
    {
    /* This HAL does not support any of the control functions */
        return ATCA_UNIMPLEMENTED;
    }
    return ATCA_BAD_PARAM;
}

/** \brief manages reference count on given bus and releases resource if no more refences exist
 * \param[in] hal_data - opaque pointer to hal data structure - known only to the HAL implementation
 * \return ATCA_SUCCESS on success, otherwise an error code.
 */
ATCA_STATUS hal_i2c_release(void* hal_data)
{
    atca_i2c_ftdi_host_t* hal = (atca_i2c_ftdi_host_t*)hal_data;

    if (hal != NULL)
    {
        // if the use count for this bus has gone to 0 references, 
        // disable it.  protect against an unbracketed release
        if (hal->ref_ct > 0)
        {
            hal->ref_ct--;
        }
        if (hal->ref_ct == 0)
        {
            if (hal->ftHandle != NULL)
                I2C_CloseChannel(hal->ftHandle);
            Cleanup_libMPSSE();
            free(hal);
        }
    }
    return ATCA_SUCCESS;
}


/** @} */
