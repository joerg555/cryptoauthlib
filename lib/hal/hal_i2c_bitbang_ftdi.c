/* --- setting states on the bus with the right timing: ---------------	*/
#include <stdio.h>
#define FTDIMPSSE_STATIC
#include "../../ftdi-mpsse/ftdi_infra.h"
#include "../../ftdi-mpsse/ftdi_common.h"

FT_STATUS JFT_Open(
    int deviceNumber,
    FT_HANDLE* pHandle
)
{
    return varFunctionPtrLst.p_FT_Open(deviceNumber, pHandle);
}

FT_STATUS JFT_Close(
    FT_HANDLE ftHandle
)
{
    return varFunctionPtrLst.p_FT_Close(ftHandle);
}


FT_STATUS WINAPI JFT_SetBitMode(
    FT_HANDLE ftHandle,
    UCHAR ucMask,
    UCHAR ucEnable
)
{
    return varFunctionPtrLst.p_FT_SetBitmode(ftHandle, ucMask, ucEnable);
}


FT_STATUS JFT_Read(
    FT_HANDLE ftHandle,
    LPVOID lpBuffer,
    DWORD dwBytesToRead,
    LPDWORD lpBytesReturned
)
{
    return varFunctionPtrLst.p_FT_Read(ftHandle, lpBuffer, dwBytesToRead, lpBytesReturned);
}

FT_STATUS JFT_Write(
    FT_HANDLE ftHandle,
    LPVOID lpBuffer,
    DWORD dwBytesToWrite,
    LPDWORD lpBytesWritten
)
{
    return varFunctionPtrLst.p_FT_Write(ftHandle, lpBuffer, dwBytesToWrite, lpBytesWritten);
}

struct i2c_algo_bit_data
{
    unsigned udelay;
    unsigned timeout;
    int retries;
};

#define jiffies GetTickCount()

unsigned time_after(unsigned st, unsigned tm)
{
    return (tm - st) > 0;
}

void udelay(unsigned us)
{

}

void cpu_relax()
{

}

void yield()
{

}

#ifndef ETIMEDOUT
#define ETIMEDOUT 1
#define ENODEV    2
#define EIO       3
#define EPROTO    4
#define ENXIO     5
#endif




#define I2C_SMBUS_BLOCK_MAX 32


#ifdef DEBUG
#define bit_dbg(level, dev, format, ...) \
	do { \
		if (i2c_debug >= level) \
			dev_dbg(dev, format, ##args); \
	} while (0)
#else
#define bit_dbg(level, dev, format, ...) \
	do {} while (0)
#endif /* DEBUG */

#define SCLMSK 0x01
#define SDAMSK 0x02


FT_HANDLE m_ftHandle;
DWORD m_direction = 0; 
DWORD m_gpiooutvalue = 0;
DWORD m_gpioinvalue = 0;
DWORD m_olddirection;
DWORD m_oldgpiovalue;

static void UpdateBitData()
{
    DWORD nbytes;
    FT_STATUS ftStatus;
    if (m_olddirection != m_direction)
    {
        m_olddirection = m_direction;
        ftStatus = JFT_SetBitMode(m_ftHandle, (UCHAR)m_direction, FT_BITMODE_SYNC_BITBANG);
        if (ftStatus != FT_OK)
            printf("Fehler FT_SetBitMode\n");
    }
    if (m_oldgpiovalue != m_gpiooutvalue)
    {
        m_oldgpiovalue = m_gpiooutvalue;
        ftStatus = JFT_Write(m_ftHandle, &m_gpiooutvalue, 1, &nbytes);
        if (ftStatus != FT_OK)
            printf("Fehler FT_Write\n");
    }
}

void GetBitData()
{
    DWORD nbytes;
    FT_STATUS ftStatus;
    ftStatus = JFT_Read(m_ftHandle, &m_gpioinvalue, 1, &nbytes);
    if (ftStatus != FT_OK)
        printf("Fehler JFT_Read\n");
}

int i2c_bitbang_init()
{
    FT_STATUS ftStatus;
    m_olddirection = 0xffff;
    m_olddirection = 0xffff;
    // Öffnen des FTDI-Geräts
    ftStatus = JFT_Open(0, &m_ftHandle);
    if (ftStatus != FT_OK)
    {
        printf("Fehler beim Öffnen des FTDI-Geräts\n");
        return -1;
    }
    UpdateBitData();
    return 0;
}

void i2c_bitbang_exit()
{
    if (m_ftHandle != 0)
        JFT_Close(m_ftHandle);
    m_ftHandle = 0;
}

void setsda(struct i2c_algo_bit_data* adap, int val)
{
    m_direction |= SDAMSK;
    if (val)
        m_gpiooutvalue |= SDAMSK;
    else
        m_gpiooutvalue &= ~SDAMSK;
    UpdateBitData();
}

void setscl(struct i2c_algo_bit_data* adap, int val)
{
    m_direction |= SCLMSK;
    if (val)
        m_gpiooutvalue |= SCLMSK;
    else
        m_gpiooutvalue &= ~SCLMSK;
    UpdateBitData();
}

int getsda(struct i2c_algo_bit_data* adap)
{
    m_direction &= ~SDAMSK;
    UpdateBitData();
    GetBitData();
    return (m_gpioinvalue & SDAMSK) != 0;
}

int getscl(struct i2c_algo_bit_data* adap)
{
    m_direction &= ~SCLMSK;
    UpdateBitData();
    GetBitData();
    return (m_gpioinvalue & SCLMSK) != 0;
}

static inline void sdalo(struct i2c_algo_bit_data* adap)
{
    setsda(adap, 0);
    udelay((adap->udelay + 1) / 2);
}

static inline void sdahi(struct i2c_algo_bit_data* adap)
{
    setsda(adap, 1);
    udelay((adap->udelay + 1) / 2);
}

static inline void scllo(struct i2c_algo_bit_data* adap)
{
    setscl(adap, 0);
    udelay(adap->udelay / 2);
}

/*
 * Raise scl line, and do checking for delays. This is necessary for slower
 * devices.
 */
static int sclhi(struct i2c_algo_bit_data* adap)
{
    unsigned long start;

    setscl(adap, 1);

    /* Not all adapters have scl sense line... */
    //if (!adap->getscl)
    //	goto done;

    start = jiffies;
    while (!getscl(adap)) {
        /* This hw knows how to read the clock line, so we wait
         * until it actually gets high.  This is safer as some
         * chips may hold it low ("clock stretching") while they
         * are processing data internally.
         */
        if (time_after(jiffies, start + adap->timeout)) {
            /* Test one last time, as we may have been preempted
             * between last check and timeout test.
             */
            if (getscl(adap))
                break;
            return -ETIMEDOUT;
        }
        cpu_relax();
    }
#ifdef DEBUG
    if (jiffies != start && i2c_debug >= 3)
        pr_debug("i2c-algo-bit: needed %ld jiffies for SCL to go high\n",
            jiffies - start);
#endif

    //done:
    udelay(adap->udelay);
    return 0;
}


/* --- other auxiliary functions --------------------------------------	*/
static void i2c_start(struct i2c_algo_bit_data* adap)
{
    /* assert: scl, sda are high */
    setsda(adap, 0);
    udelay(adap->udelay);
    scllo(adap);
}

static void i2c_repstart(struct i2c_algo_bit_data* adap)
{
    /* assert: scl is low */
    sdahi(adap);
    sclhi(adap);
    setsda(adap, 0);
    udelay(adap->udelay);
    scllo(adap);
}

static void i2c_stop(struct i2c_algo_bit_data* adap)
{
    /* assert: scl is low */
    sdalo(adap);
    sclhi(adap);
    setsda(adap, 1);
    udelay(adap->udelay);
}



/* send a byte without start cond., look for arbitration,
   check ackn. from slave */
   /* returns:
    * 1 if the device acknowledged
    * 0 if the device did not ack
    * -ETIMEDOUT if an error occurred (while raising the scl line)
    */
static int i2c_outb(struct i2c_algo_bit_data* adap, unsigned char c)
{
    int i;
    int sb;
    int ack;

    /* assert: scl is low */
    for (i = 7; i >= 0; i--) {
        sb = (c >> i) & 1;
        setsda(adap, sb);
        udelay((adap->udelay + 1) / 2);
        if (sclhi(adap) < 0) { /* timed out */
            bit_dbg(1, &adap->dev,
                "i2c_outb: 0x%02x, timeout at bit #%d\n",
                (int)c, i);
            return -ETIMEDOUT;
        }
        /* FIXME do arbitration here:
         * if (sb && !getsda(adap)) -> ouch! Get out of here.
         *
         * Report a unique code, so higher level code can retry
         * the whole (combined) message and *NOT* issue STOP.
         */
        scllo(adap);
    }
    sdahi(adap);
    if (sclhi(adap) < 0) { /* timeout */
        bit_dbg(1, &adap->dev,
            "i2c_outb: 0x%02x, timeout at ack\n", (int)c);
        return -ETIMEDOUT;
    }

    /* read ack: SDA should be pulled down by slave, or it may
     * NAK (usually to report problems with the data we wrote).
     */
    ack = !getsda(adap);    /* ack: sda is pulled low -> success */
    bit_dbg(2, &adap->dev, "i2c_outb: 0x%02x %s\n", (int)c,
        ack ? "A" : "NA");

    scllo(adap);
    return ack;
    /* assert: scl is low (sda undef) */
}


static int i2c_inb(struct i2c_algo_bit_data* adap)
{
    /* read byte via i2c port, without start/stop sequence	*/
    /* acknowledge is sent in i2c_read.			*/
    int i;
    unsigned char indata = 0;

    /* assert: scl is low */
    sdahi(adap);
    for (i = 0; i < 8; i++) {
        if (sclhi(adap) < 0) { /* timeout */
            bit_dbg(1, &adap->dev,
                "i2c_inb: timeout at bit #%d\n",
                7 - i);
            return -ETIMEDOUT;
        }
        indata *= 2;
        if (getsda(adap))
            indata |= 0x01;
        setscl(adap, 0);
        udelay(i == 7 ? adap->udelay / 2 : adap->udelay);
    }
    /* assert: scl is low */
    return indata;
}

/*
 * Sanity check for the adapter hardware - check the reaction of
 * the bus lines only if it seems to be idle.
 */
static int test_bus(struct i2c_algo_bit_data* adap)
{
    //const char* name = adap->name;
    int scl, sda;

    sda = getsda(adap);
    scl = getscl(adap);
    if (!scl || !sda) {
        printf("bus seems to be busy(scl = % d, sda = % d)\n", scl, sda);
        goto bailout;
    }

    sdalo(adap);
    sda = getsda(adap);
    scl = getscl(adap);
    if (sda) {
        printf("SDA stuck high!\n");
        goto bailout;
    }
    if (!scl) {
        printf("SCL unexpected low while pulling SDA low!\n");
        goto bailout;
    }

    sdahi(adap);
    sda = getsda(adap);
    scl = getscl(adap);
    if (!sda) {
        printf("SDA stuck low!\n");
        goto bailout;
    }
    if (!scl) {
        printf("SCL unexpected low while pulling SDA high!\n");
        goto bailout;
    }

    scllo(adap);
    sda = getsda(adap);
    scl = getscl(adap);
    if (scl) {
        printf("SCL stuck high!\n");
        goto bailout;
    }
    if (!sda) {
        printf("SDA unexpected low while pulling SCL low!\n");
        goto bailout;
    }

    sclhi(adap);
    sda = getsda(adap);
    scl = getscl(adap);
    if (!scl) {
        printf("SCL stuck low!\n");
        goto bailout;
    }
    if (!sda) {
        printf("SDA unexpected low while pulling SCL high!\n");
        goto bailout;
    }
    printf("Test OK\n");
    return 0;
bailout:
    sdahi(adap);
    sclhi(adap);

    return -ENODEV;
}

/* ----- Utility functions
 */

 /* try_address tries to contact a chip for a number of
  * times before it gives up.
  * return values:
  * 1 chip answered
  * 0 chip did not answer
  * -x transmission error
  */
static int try_address(struct i2c_algo_bit_data* adap,
    unsigned char addr, int retries)
{
    int i, ret = 0;

    for (i = 0; i <= retries; i++) {
        ret = i2c_outb(adap, addr);
        if (ret == 1 || i == retries)
            break;
        bit_dbg(3, &adap->dev, "emitting stop condition\n");
        i2c_stop(adap);
        udelay(adap->udelay);
        yield();
        bit_dbg(3, &adap->dev, "emitting start condition\n");
        i2c_start(adap);
    }
    if (i && ret)
        bit_dbg(1, &adap->dev,
            "Used %d tries to %s client at 0x%02x: %s\n", i + 1,
            addr & 1 ? "read from" : "write to", addr >> 1,
            ret == 1 ? "success" : "failed, timeout?");
    return ret;
}

struct i2c_msg {
    unsigned short addr;
    unsigned short flags;
#define I2C_M_RD		0x0001	/* guaranteed to be 0x0001! */
    //#define I2C_M_TEN		0x0010	/* use only if I2C_FUNC_10BIT_ADDR */
#define I2C_M_DMA_SAFE		0x0200	/* use only in kernel space */
#define I2C_M_RECV_LEN		0x0400	/* use only if I2C_FUNC_SMBUS_READ_BLOCK_DATA */
#define I2C_M_NO_RD_ACK		0x0800	/* use only if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_IGNORE_NAK	0x1000	/* use only if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_REV_DIR_ADDR	0x2000	/* use only if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_NOSTART		0x4000	/* use only if I2C_FUNC_NOSTART */
#define I2C_M_STOP		0x8000	/* use only if I2C_FUNC_PROTOCOL_MANGLING */
    unsigned short len;
    unsigned char* buf;
};

#define i2c_8bit_addr_from_msg(m)   (unsigned char)((m)->addr)

static int sendbytes(struct i2c_algo_bit_data* adap, struct i2c_msg* msg)
{
    const unsigned char* temp = msg->buf;
    int count = msg->len;
    unsigned short nak_ok = msg->flags & I2C_M_IGNORE_NAK;
    int retval;
    int wrcount = 0;

    while (count > 0) {
        retval = i2c_outb(adap, *temp);

        /* OK/ACK; or ignored NAK */
        if ((retval > 0) || (nak_ok && (retval == 0))) {
            count--;
            temp++;
            wrcount++;

            /* A slave NAKing the master means the slave didn't like
             * something about the data it saw.  For example, maybe
             * the SMBus PEC was wrong.
             */
        }
        else if (retval == 0) {
            printf("sendbytes: NAK bailout.\n");
            return -EIO;

            /* Timeout; or (someday) lost arbitration
             *
             * FIXME Lost ARB implies retrying the transaction from
             * the first message, after the "winning" master issues
             * its STOP.  As a rule, upper layer code has no reason
             * to know or care about this ... it is *NOT* an error.
             */
        }
        else {
            printf("sendbytes: error %d\n", retval);
            return retval;
        }
    }
    return wrcount;
}

static int acknak(struct i2c_algo_bit_data* adap , int is_ack)
{

    /* assert: sda is high */
    if (is_ack)		/* send ack */
        setsda(adap, 0);
    udelay((adap->udelay + 1) / 2);
    if (sclhi(adap) < 0) {	/* timeout */
        printf("readbytes: ack/nak timeout\n");
        return -ETIMEDOUT;
    }
    scllo(adap);
    return 0;
}

static int readbytes(struct i2c_algo_bit_data* adap , struct i2c_msg* msg)
{
    int inval;
    int rdcount = 0;	/* counts bytes read */
    unsigned char* temp = msg->buf;
    int count = msg->len;
    const unsigned flags = msg->flags;

    while (count > 0) {
        inval = i2c_inb(adap);
        if (inval >= 0) {
            *temp = inval;
            rdcount++;
        }
        else {   /* read timed out */
            break;
        }

        temp++;
        count--;

        /* Some SMBus transactions require that we receive the
           transaction length as the first read byte. */
        if (rdcount == 1 && (flags & I2C_M_RECV_LEN)) {
            if (inval <= 0 || inval > I2C_SMBUS_BLOCK_MAX) {
                if (!(flags & I2C_M_NO_RD_ACK))
                    acknak(adap, 0);
                printf("readbytes: invalid block length (%d)\n",
                    inval);
                return -EPROTO;
            }
            /* The original count value accounts for the extra
               bytes, that is, either 1 for a regular transaction,
               or 2 for a PEC transaction. */
            count += inval;
            msg->len += inval;
        }

        bit_dbg(2, &adap->dev, "readbytes: 0x%02x %s\n",
            inval,
            (flags & I2C_M_NO_RD_ACK)
            ? "(no ack/nak)"
            : (count ? "A" : "NA"));

        if (!(flags & I2C_M_NO_RD_ACK)) {
            inval = acknak(adap, count);
            if (inval < 0)
                return inval;
        }
    }
    return rdcount;
}

/* doAddress initiates the transfer by generating the start condition (in
 * try_address) and transmits the address in the necessary format to handle
 * reads, writes as well as 10bit-addresses.
 * returns:
 *  0 everything went okay, the chip ack'ed, or IGNORE_NAK flag was set
 * -x an error occurred (like: -ENXIO if the device did not answer, or
 *	-ETIMEDOUT, for example if the lines are stuck...)
 */
static int bit_doAddress(struct i2c_algo_bit_data* adap , struct i2c_msg* msg)
{
    unsigned short flags = msg->flags;
    unsigned short nak_ok = msg->flags & I2C_M_IGNORE_NAK;

    unsigned char addr;
    int ret, retries;

    retries = nak_ok ? 0 : adap->retries;

    //if (flags & I2C_M_TEN) {
    //    /* a ten bit address */
    //    addr = 0xf0 | ((msg->addr >> 7) & 0x06);
    //    bit_dbg(2, &adap->dev, "addr0: %d\n", addr);
    //    /* try extended address code...*/
    //    ret = try_address(adap, addr, retries);
    //    if ((ret != 1) && !nak_ok) {
    //        printf(
    //            "died at extended address code\n");
    //        return -ENXIO;
    //    }
    //    /* the remaining 8 bit address */
    //    ret = i2c_outb(adap, msg->addr & 0xff);
    //    if ((ret != 1) && !nak_ok) {
    //        /* the chip did not ack / xmission error occurred */
    //        printf( "died at 2nd address code\n");
    //        return -ENXIO;
    //    }
    //    if (flags & I2C_M_RD) {
    //        bit_dbg(3, &adap->dev,
    //            "emitting repeated start condition\n");
    //        i2c_repstart(adap);
    //        /* okay, now switch into reading mode */
    //        addr |= 0x01;
    //        ret = try_address(adap, addr, retries);
    //        if ((ret != 1) && !nak_ok) {
    //            printf(
    //                "died at repeated address code\n");
    //            return -EIO;
    //        }
    //    }
    //}
    //else 
    {		/* normal 7bit address	*/
        addr = i2c_8bit_addr_from_msg(msg);
        if (flags & I2C_M_REV_DIR_ADDR)
            addr ^= 1;
        ret = try_address(adap, addr, retries);
        if ((ret != 1) && !nak_ok)
            return -ENXIO;
    }

    return 0;
}

static int bit_xfer(struct i2c_algo_bit_data* adap ,
    struct i2c_msg msgs[], int num)
{
    struct i2c_msg* pmsg;
    int i, ret;
    unsigned short nak_ok;

    bit_dbg(3, &adap->dev, "emitting start condition\n");
    i2c_start(adap);
    for (i = 0; i < num; i++) {
        pmsg = &msgs[i];
        nak_ok = pmsg->flags & I2C_M_IGNORE_NAK;
        if (!(pmsg->flags & I2C_M_NOSTART)) {
            if (i) {
                if (msgs[i - 1].flags & I2C_M_STOP) {
                    bit_dbg(3, &adap->dev,
                        "emitting enforced stop/start condition\n");
                    i2c_stop(adap);
                    i2c_start(adap);
                }
                else {
                    bit_dbg(3, &adap->dev,
                        "emitting repeated start condition\n");
                    i2c_repstart(adap);
                }
            }
            ret = bit_doAddress(adap, pmsg);
            if ((ret != 0) && !nak_ok) {
                bit_dbg(1, &adap->dev,
                    "NAK from device addr 0x%02x msg #%d\n",
                    msgs[i].addr, i);
                goto bailout;
            }
        }
        if (pmsg->flags & I2C_M_RD) {
            /* read bytes into buffer*/
            ret = readbytes(adap, pmsg);
            if (ret >= 1)
                bit_dbg(2, &adap->dev, "read %d byte%s\n",
                    ret, ret == 1 ? "" : "s");
            if (ret < pmsg->len) {
                if (ret >= 0)
                    ret = -EIO;
                goto bailout;
            }
        }
        else {
            /* write bytes from buffer */
            ret = sendbytes(adap, pmsg);
            if (ret >= 1)
                bit_dbg(2, &adap->dev, "wrote %d byte%s\n",
                    ret, ret == 1 ? "" : "s");
            if (ret < pmsg->len) {
                if (ret >= 0)
                    ret = -EIO;
                goto bailout;
            }
        }
    }
    ret = i;

bailout:
    bit_dbg(3, &adap->dev, "emitting stop condition\n");
    i2c_stop(adap);
    return ret;
}

