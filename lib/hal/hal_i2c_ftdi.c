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

#define FTDIMPSSE_STATIC
#include "../../ftdi-mpsse/ftdi_infra.h"
#include "../../ftdi-mpsse/ftdi_common.h"
#include "../../ftdi-mpsse/libmpsse_i2c.h"

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

// Debug interface

#define DEBUG_BYTES
#ifdef DEBUG_BYTES

#ifndef ATCA_HAL_SET_DEBUG
#define ATCA_HAL_SET_DEBUG  10
#endif

int hal_i2c_debug = 0;
#ifndef WIN32
#include <time.h>
unsigned long GetTickCount(void)
{
    unsigned long ticks;
    struct timespec now;
    // "Monotonic system-wide clock" Zeitstempel von System
    clock_gettime(CLOCK_MONOTONIC, &now);
    ticks = now.tv_sec * 1000l;
    ticks += now.tv_nsec / 1000000l;
    return ticks;
}
#endif

static void dumpbuffer(const char* txt, unsigned uaddr, const unsigned char* buf, unsigned nlen)
{
    if (hal_i2c_debug == 0)
        return;
    printf("%s adr %02x %4d: ", txt, uaddr, GetTickCount() % 10000);
    for (unsigned i = 0; i < nlen; i++)
        printf("%02x ", buf[i]);
    printf("\n");
}
#else
#define dumpbuffer(txt,adr,buf,nlen)
#endif
//#ifdef DEBUG_BYTES
//static DWORD g_tStart;
//DWORD GetTick()
//{
//    if (g_tStart == 0)
//        g_tStart = GetTickCount();
//    return GetTickCount() - g_tStart;
//}
//#endif
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
} atca_i2c_host_t;

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
    atca_i2c_host_t* phal;
    FT_DEVICE_LIST_INFO_NODE devList;
    FT_STATUS status;
    ChannelConfig channelConf;

    if (iface == NULL || cfg == NULL)
        return ATCA_BAD_PARAM;
    if (iface->hal_data != NULL)
    {
        phal = (atca_i2c_host_t*)iface->hal_data;

        // Assume the bus had already been initialized
        phal->ref_ct++;

        return ATCA_SUCCESS;
    }
    if ((iface->hal_data = malloc(sizeof(atca_i2c_host_t))) == NULL)
        return ATCA_ALLOC_FAILURE;
    phal = (atca_i2c_host_t*)iface->hal_data;

    phal->ref_ct = 1;                                 // buses are shared, this is the first instance
    phal->channel = (int)ATCA_IFACECFG_VALUE(cfg, atcai2c.bus); // 0-based logical bus number
    phal->ftHandle = NULL;
    //channelConf.ClockRate = I2C_CLOCK_FAST_MODE;/*i.e. 400000 KHz*/
    //channelConf.ClockRate = I2C_CLOCK_STANDARD_MODE; /*i.e. 100000 KHz*/
    //channelConf.ClockRate = ATCA_IFACECFG_I2C_BAUD(cfg);    // may be too fast
    channelConf.ClockRate = I2C_CLOCK_STANDARD_MODE; /*i.e. 100000 KHz*/
    channelConf.ClockRate = 20000; /*for test only*/
    channelConf.LatencyTimer = 64; //  255;
    //channelConf.Options = I2C_DISABLE_3PHASE_CLOCKING;
    channelConf.Options = I2C_ENABLE_DRIVE_ONLY_ZERO;
    //channelConf.Options = I2C_DISABLE_3PHASE_CLOCKING | I2C_ENABLE_DRIVE_ONLY_ZERO;
#ifdef INFRA_DEBUG_ENABLE
    currentDebugLevel = MSG_WARN;
#endif
    Init_libMPSSE();
    status = I2C_GetChannelInfo(phal->channel, &devList);
    status = I2C_OpenChannel(phal->channel, &phal->ftHandle);
    status = I2C_InitChannel(phal->ftHandle, &channelConf);
    if (status != FT_OK)
    {
        phal->ftHandle = NULL;
        fprintf(stderr, "Error FTDI %d\n", status);
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
ATCA_STATUS hal_i2c_send(ATCAIface iface, uint8_t word_address, uint8_t* txdata, int txlength)
{
    atca_i2c_host_t* phal = (atca_i2c_host_t*)atgetifacehaldat(iface);
    FT_STATUS status;
    DWORD xfer = 0;
    uint8_t temp_buf[256];
    uint8_t device_address = ATCA_IFACECFG_I2C_ADDRESS(iface->mIfaceCFG);
    uint32_t options;

    if (phal == NULL || phal->ftHandle == NULL)
        return ATCA_NOT_INITIALIZED;
    if (txlength >= (sizeof(temp_buf) - 1))
        return ATCA_BAD_PARAM;
    temp_buf[0] = word_address;
    if (txlength > 1 && txdata)
        memcpy(temp_buf + 1, txdata, txlength);
    txlength++;
    if (txlength > 64)
        options = 1;
    options = I2C_TRANSFER_OPTIONS_START_BIT | I2C_TRANSFER_OPTIONS_STOP_BIT; // | I2C_TRANSFER_OPTIONS_FAST_TRANSFER;
    if (device_address == 0)
        options |= I2C_TRANSFER_OPTIONS_NO_ADDRESS | I2C_TRANSFER_OPTIONS_FAST_TRANSFER;  // dummy device
    //if (txlength > 64)
    //    currentDebugLevel = MSG_DEBUG;
    status = I2C_DeviceWrite(phal->ftHandle, device_address >> 1, txlength, temp_buf, &xfer, options);

    dumpbuffer("i2c_sen", device_address >> 1, temp_buf, txlength);
    if (status != FT_OK)
    {
        fprintf(stderr, "Error Write %02x FTDI %d\n", device_address >> 1, status);
        return ATCA_COMM_FAIL;
    }
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
    atca_i2c_host_t* phal = (atca_i2c_host_t*)atgetifacehaldat(iface);

    if (phal == NULL || phal->ftHandle == NULL)
        return ATCA_NOT_INITIALIZED;

    FT_STATUS status;
    DWORD xfer = 0;
    uint8_t device_address = ATCA_IFACECFG_I2C_ADDRESS(iface->mIfaceCFG);

    /* Repeated Start condition generated. */
    status = I2C_DeviceRead(phal->ftHandle, device_address >> 1, *rxlength, rxdata, &xfer,
        I2C_TRANSFER_OPTIONS_START_BIT | I2C_TRANSFER_OPTIONS_STOP_BIT 
        | I2C_TRANSFER_OPTIONS_FAST_TRANSFER_BYTES
    );
    *rxlength = (uint16_t)xfer;
    if (status != FT_OK)
    {
        fprintf(stderr, "Error Read FTDI %d\n", status);
        return ATCA_COMM_FAIL;
    }
    dumpbuffer("i2c_rec", device_address >> 1, rxdata, *rxlength);
    return ATCA_SUCCESS;
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
#ifdef DEBUG_BYTES
    if (option == ATCA_HAL_SET_DEBUG)
    {
        hal_i2c_debug = 1; // enable debug output
        return ATCA_SUCCESS;
    }
#endif
    if (iface == NULL || iface->mIfaceCFG == NULL)
        return ATCA_BAD_PARAM;
    if (option == ATCA_HAL_CHANGE_BAUD)
        return ATCA_SUCCESS;    // may be error if we say unimplemented
    /* This HAL does not support any of the control functions */
    return ATCA_UNIMPLEMENTED;
}

/** \brief manages reference count on given bus and releases resource if no more refences exist
 * \param[in] hal_data - opaque pointer to hal data structure - known only to the HAL implementation
 * \return ATCA_SUCCESS on success, otherwise an error code.
 */
ATCA_STATUS hal_i2c_release(void* hal_data)
{
    atca_i2c_host_t* phal = (atca_i2c_host_t*)hal_data;

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
            if (phal->ftHandle != NULL)
                I2C_CloseChannel(phal->ftHandle);
            Cleanup_libMPSSE();
            free(phal);
        }
    }
    return ATCA_SUCCESS;
}


/** @} */
