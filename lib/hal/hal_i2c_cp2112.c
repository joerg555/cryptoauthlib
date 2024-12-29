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
#include "SLABCP2112.h"

//
//#define DEBUG_BYTES
#ifdef DEBUG_BYTES
static DWORD g_tStart;
DWORD GetTick()
{
    if (g_tStart == 0)
        g_tStart = GetTickCount();
    return GetTickCount() - g_tStart;
}
#endif
 /** \defgroup hal_ Hardware abstraction layer (hal_)
  *
  * \brief
  * These methods define the hardware abstraction layer for communicating with a CryptoAuth device
  *
    @{ */

typedef struct atca_i2c_cp2112_host_s
{
    int  ref_ct;
    HID_SMBUS_DEVICE m_hidSmbus;
    unsigned long m_devnum;
    HID_SMBUS_S0	m_readstatus0;
} atca_i2c_cp2112_host_t;


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
    atca_i2c_cp2112_host_t* phal;

    if (iface == NULL || cfg == NULL)
        return ATCA_BAD_PARAM;

    if (iface->hal_data != NULL)
    {
        phal = (atca_i2c_cp2112_host_t*)iface->hal_data;

        // Assume the bus had already been initialized
        phal->ref_ct++;

        return ATCA_SUCCESS;
    }
    if ((iface->hal_data = malloc(sizeof(atca_i2c_cp2112_host_t))) == NULL)
        return ATCA_ALLOC_FAILURE;
    phal = (atca_i2c_cp2112_host_t*)iface->hal_data;

    phal->ref_ct = 1;                                 // buses are shared, this is the first instance
    phal->m_devnum = (int)ATCA_IFACECFG_VALUE(cfg, atcai2c.bus); // 0-based logical bus number
    phal->m_hidSmbus = NULL;
    phal->m_readstatus0 = 0;

    HID_SMBUS_STATUS status = HidSmbus_Open(&phal->m_hidSmbus, phal->m_devnum, 0, 0);
    if (status != HID_SMBUS_SUCCESS)
    {
        phal->m_hidSmbus = NULL;
        return ATCA_NO_DEVICES;
    }
    DWORD	m_bitRate = 0;
    BYTE	m_ackAddress = 0;
    BOOL	m_autoRespond = FALSE;
    WORD	m_writeTimeout = 0;
    WORD	m_readTimeout = 0;
    WORD	m_transferRetries = 0;
    BOOL	m_sclLowTimeout = FALSE;
    DWORD	m_responseTimeout = 0;
    status = HidSmbus_GetSmbusConfig(phal->m_hidSmbus, &m_bitRate, &m_ackAddress, &m_autoRespond, &m_writeTimeout, &m_readTimeout, &m_sclLowTimeout, &m_transferRetries);
    if (status == HID_SMBUS_SUCCESS)
    {
        // we need autorespond
        m_autoRespond = TRUE;
        status = HidSmbus_SetSmbusConfig(phal->m_hidSmbus, m_bitRate, m_ackAddress, m_autoRespond, m_writeTimeout, m_readTimeout, m_sclLowTimeout, m_transferRetries);
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
ATCA_STATUS hal_i2c_send(ATCAIface iface, uint8_t word_address, uint8_t* txdata, int txlength)
{
    atca_i2c_cp2112_host_t* phal = (atca_i2c_cp2112_host_t*)atgetifacehaldat(iface);
    HID_SMBUS_STATUS status;
    BOOL opened;
    uint8_t temp_buf[64];
    uint8_t device_address = ATCA_IFACECFG_I2C_ADDRESS(iface->mIfaceCFG);

    if (phal == NULL || phal->m_hidSmbus == NULL)
        return ATCA_NOT_INITIALIZED;
    if (txlength >= (sizeof(temp_buf) - 1))
        return ATCA_BAD_PARAM;

    if (device_address == 0)
        return ATCA_SUCCESS;
    temp_buf[0] = word_address;
    if (txlength > 1 && txdata)
        memcpy(temp_buf + 1, txdata, txlength);
    txlength++;
    // Make sure that the device is opened
    if (!(HidSmbus_IsOpened(phal->m_hidSmbus, &opened) == HID_SMBUS_SUCCESS && opened))
        return ATCA_COMM_FAIL;
    status = HidSmbus_CancelTransfer(phal->m_hidSmbus);
    status = HidSmbus_WriteRequest(phal->m_hidSmbus, device_address, temp_buf, txlength);
    if (status != 0)
        return ATCA_COMM_FAIL;
#ifdef DEBUG_BYTES
    printf("i2c_sen %4ld addr %02x, nlen %d :", GetTick(), device_address >> 1, txlength);
    for (int n = 0; n < txlength; n++)
        printf("%02x ", temp_buf[n]);
    printf("\n");
#endif
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
ATCA_STATUS hal_i2c_receive(ATCAIface iface, uint8_t device_address, uint8_t* rxdata, uint16_t* rxlength)
{
    atca_i2c_cp2112_host_t* phal = (atca_i2c_cp2112_host_t*)atgetifacehaldat(iface);

    HID_SMBUS_STATUS status;
    BOOL opened;
    BYTE rxbuf[HID_SMBUS_MAX_READ_RESPONSE_SIZE];

    if (phal == NULL || phal->m_hidSmbus == NULL)
        return ATCA_NOT_INITIALIZED;
    // Make sure that the device is opened
    if (!(HidSmbus_IsOpened(phal->m_hidSmbus, &opened) == HID_SMBUS_SUCCESS && opened))
        return ATCA_COMM_FAIL;
    status = HidSmbus_ReadRequest(phal->m_hidSmbus, device_address, *rxlength);
    //status = HidSmbus_AddressReadRequest(phal->m_hidSmbus, iface->mIfaceCFG->atcai2c.address, 1, 1, (BYTE *)addr);

    if (status == 0)
    {
        BYTE		nBytesRead = 0;
        uint8_t* rx = rxdata;
        unsigned rxcnt = 0;
        unsigned rxlen = *rxlength;
        for (int i = 0; i < 10; i++)
        {
            status = HidSmbus_GetReadResponse(phal->m_hidSmbus, &phal->m_readstatus0, rxbuf, sizeof(rxbuf), &nBytesRead);
            if (status != 0 || phal->m_readstatus0 == HID_SMBUS_S0_ERROR)
                break;
            memcpy(rx + rxcnt, rxbuf, nBytesRead);
            rxcnt += nBytesRead;
            if (rxcnt >= rxlen || phal->m_readstatus0 == HID_SMBUS_S0_COMPLETE)
            {
                *rxlength = rxcnt;
                phal->m_readstatus0 = HID_SMBUS_S0_IDLE;
#ifdef DEBUG_BYTES
                printf("i2c_rec %4ld addr %02x, nlen %d/%d :", GetTick(), device_address >> 1, *rxlength, rxcnt);
                for (int n = 0; n < *rxlength; n++)
                    printf("%02x ", rxdata[n]);
                printf("\n");
#endif
                return ATCA_SUCCESS;
            }
        }
    }
    *rxlength = 0;
    phal->m_readstatus0 = HID_SMBUS_S0_IDLE;
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
        if (option == ATCA_HAL_CHANGE_BAUD)
            return ATCA_SUCCESS;    // may be error if we say unimplemented
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
    atca_i2c_cp2112_host_t* phal = (atca_i2c_cp2112_host_t*)hal_data;

    if (phal != NULL)
    {
        // if the use count for this bus has gone to 0 references, 
        // disable it.  protect against an unbracketed release
        if (phal->ref_ct > 0)
        {
            phal->ref_ct--;
        }
        if (phal->ref_ct == 0)
        {
            if (phal->m_hidSmbus != NULL)
                HidSmbus_Close(phal->m_hidSmbus);
            //phal->m_hidSmbus = NULL;
            free(phal);
        }
    }
    return ATCA_SUCCESS;
}


/** @} */
