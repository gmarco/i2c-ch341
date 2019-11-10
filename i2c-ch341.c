/*
 * Driver for the CH341 USB-I2C adapter
 *
 * Copyright (c) 2014 Marco Gittler
 *
 * Derived from:
 *  i2c-tiny-usb.c
 *  Copyright (C) 2006-2007 Till Harbaum (Till@Harbaum.org)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, version 2.
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/i2c.h>
#include <linux/mutex.h>

#define DRIVER_NAME     "i2c-ch341-u2c"

#define USB_VENDOR_ID_CH341     0x1a86
#define USB_DEVICE_ID_CH341_U2C 0x5512

#define DEFAULT_CONFIGURATION       0x01
#define DEFAULT_TIMEOUT             100    // 100mS for USB timeouts

#define                mCH341_PACKET_LENGTH        32
#define                mCH341_PKT_LEN_SHORT        8

#define                mCH341_ENDP_INTER_UP        0x81
#define                mCH341_ENDP_INTER_DOWN        0x01
#define                mCH341_ENDP_DATA_UP                0x82
#define                mCH341_ENDP_DATA_DOWN        0x02

#define                mCH341_VENDOR_READ                0xC0
#define                mCH341_VENDOR_WRITE                0x40

#define                mCH341_PARA_INIT                0xB1
#define                mCH341_I2C_STATUS                0x52
#define                mCH341_I2C_COMMAND                0x53

#define                mCH341_PARA_CMD_R0                0xAC
#define                mCH341_PARA_CMD_R1                0xAD
#define                mCH341_PARA_CMD_W0                0xA6
#define                mCH341_PARA_CMD_W1                0xA7
#define                mCH341_PARA_CMD_STS                0xA0

#define                mCH341A_CMD_SET_OUTPUT        0xA1
#define                mCH341A_CMD_IO_ADDR                0xA2
#define                mCH341A_CMD_PRINT_OUT        0xA3
#define                mCH341A_CMD_SPI_STREAM        0xA8
#define                mCH341A_CMD_SIO_STREAM        0xA9
#define                mCH341A_CMD_I2C_STREAM        0xAA
#define                mCH341A_CMD_UIO_STREAM        0xAB

#define                mCH341A_BUF_CLEAR                0xB2
#define                mCH341A_I2C_CMD_X                0x54
#define                mCH341A_DELAY_MS                0x5E
#define                mCH341A_GET_VER                        0x5F

#define                mCH341_EPP_IO_MAX                ( mCH341_PACKET_LENGTH - 1 )
#define                mCH341A_EPP_IO_MAX                0xFF

#define                mCH341A_CMD_IO_ADDR_W        0x00
#define                mCH341A_CMD_IO_ADDR_R        0x80

#define                mCH341A_CMD_I2C_STM_STA        0x74
#define                mCH341A_CMD_I2C_STM_STO        0x75
#define                mCH341A_CMD_I2C_STM_OUT        0x80
#define                mCH341A_CMD_I2C_STM_IN        0xC0
#define                mCH341A_CMD_I2C_STM_MAX        ( min( 0x3F, mCH341_PACKET_LENGTH ) )
#define                mCH341A_CMD_I2C_STM_SET        0x60
#define                mCH341A_CMD_I2C_STM_US        0x40
#define                mCH341A_CMD_I2C_STM_MS        0x50
#define                mCH341A_CMD_I2C_STM_DLY        0x0F
#define                mCH341A_CMD_I2C_STM_END        0x00

#define                mCH341A_CMD_UIO_STM_IN        0x00
#define                mCH341A_CMD_UIO_STM_DIR        0x40
#define                mCH341A_CMD_UIO_STM_OUT        0x80
#define                mCH341A_CMD_UIO_STM_US        0xC0
#define                mCH341A_CMD_UIO_STM_END        0x20

#define                mCH341_PARA_MODE_EPP        0x00
#define                mCH341_PARA_MODE_EPP17        0x00
#define                mCH341_PARA_MODE_EPP19        0x01
#define                mCH341_PARA_MODE_MEM        0x02


#define CH341_I2C_LOW_SPEED 0               // low speed - 20kHz
#define CH341_I2C_STANDARD_SPEED 1          // standard speed - 100kHz
#define CH341_I2C_FAST_SPEED 2              // fast speed - 400kHz
#define CH341_I2C_HIGH_SPEED 3              // high speed - 750kHz

#define RESP_OK         0x00
#define RESP_FAILED     0x01
#define RESP_BAD_MEMADDR    0x04
#define RESP_DATA_ERR       0x05
#define RESP_NOT_IMPLEMENTED    0x06
#define RESP_NACK       0x07
#define RESP_TIMEOUT        0x09

#define CH341_OUTBUF_LEN   32
#define CH341_INBUF_LEN 32 /* Maximum supported receive length */

static struct mutex ch341_lock;

struct i2c_ch341_u2c {
    u8 obuffer[CH341_OUTBUF_LEN];   /* output buffer */
    u8 ibuffer[CH341_INBUF_LEN];    /* input buffer */
    int ep_in, ep_out;              /* Endpoints    */
    struct usb_device *usb_dev; /* the usb device for this device */
    struct usb_interface *interface;/* the interface for this device */
    struct i2c_adapter adapter; /* i2c related things */
    int olen;           /* Output buffer length */
    int ilen;
};

static uint frequency = CH341_I2C_LOW_SPEED;   /* I2C clock frequency in Hz */

module_param(frequency, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(frequency, "I2C clock frequency (0=20kHz,1=100kHz,2=400kHz,3=750kHz");


static int ch341_usb_transfer(struct i2c_ch341_u2c *dev)
{
    int ret = 0;
    int actual;

	mutex_lock(&ch341_lock);
    ret = usb_bulk_msg(dev->usb_dev,
                       usb_sndbulkpipe(dev->usb_dev, dev->ep_out),
                       dev->obuffer, dev->olen, &actual,
                       DEFAULT_TIMEOUT);

	mutex_unlock(&ch341_lock);

	if (ret<0)
		return ret;

	if (dev->ilen==0)
		return actual;

	mutex_lock(&ch341_lock);

	memset(dev->ibuffer, 0, sizeof(dev->ibuffer));
	ret = usb_bulk_msg(dev->usb_dev,
						  usb_rcvbulkpipe(dev->usb_dev,
										  dev->ep_in),
						  dev->ibuffer,
						  sizeof(dev->ibuffer), &actual,
						  DEFAULT_TIMEOUT);
	mutex_unlock(&ch341_lock);
	if (dev->olen > 1 && dev->obuffer[0] == 0xaa && dev->obuffer[1] == 0x60) {
		return 0; // set speed should not fail
	}

	if (ret<0)
		return ret;

  return actual;
}


/* Send command (no data) */
static int ch341_usb_cmd_msg(struct i2c_ch341_u2c *dev, u8 *msg, u8 olen,u8 ilen)
{
    memcpy(dev->obuffer, msg, olen);
    dev->olen = olen;
  	dev->ilen = ilen;
    return ch341_usb_transfer(dev);
}
static int ch341_usb_cmd_check_presence(struct i2c_ch341_u2c *dev, u8 addr) {

  int msgsize = 0, ret=0;
  uint8_t msg[CH341_INBUF_LEN];


  msg[msgsize++] = mCH341A_CMD_I2C_STREAM;
  msg[msgsize++] = mCH341A_CMD_I2C_STM_STA;
  msg[msgsize++] = mCH341A_CMD_I2C_STM_OUT;
  msg[msgsize++] = addr |0x01; // use a read
  msg[msgsize++] = mCH341A_CMD_I2C_STM_STO;
  msg[msgsize++] = mCH341A_CMD_I2C_STM_END;

  ret= ch341_usb_cmd_msg(dev,msg,msgsize,1);

  if (ret <0)
	  return ret;

  if ( dev->ibuffer[0] & 0x80 )
		return -ETIMEDOUT;

  return 0;

}

static int ch341_usb_cmd_read_addr(struct i2c_ch341_u2c *dev, u8 addr, u8 *data, u8 datalen)
{
    u8 msg0[CH341_INBUF_LEN];
    int msgsize = 0, ret,i;

    msg0[msgsize++] =    mCH341A_CMD_I2C_STREAM;
    msg0[msgsize++] =    mCH341A_CMD_I2C_STM_STA;
    msg0[msgsize++] =    mCH341A_CMD_I2C_STM_OUT |0x01 ; // write addr byte
    msg0[msgsize++] = (addr) | 0x01 ;  ;
	if (datalen) {
		for (i=0;i< datalen ;i++){
			msg0[msgsize++] = mCH341A_CMD_I2C_STM_IN | 1;
		}
		msg0[msgsize++] = mCH341A_CMD_I2C_STM_IN ;

	}
    msg0[msgsize++] = mCH341A_CMD_I2C_STM_STO;
    msg0[msgsize++] = mCH341A_CMD_I2C_STM_END;

    ret = ch341_usb_cmd_msg(dev, msg0, msgsize, datalen);

    if (ret >= 0) {
        memcpy(data, dev->ibuffer , dev->ilen );
    }
    return ret;

}
static int ch341_usb_cmd_write_addr(struct i2c_ch341_u2c *dev, u8 addr, u8 *data, u8 datalen)
{
    u8 msg0[CH341_INBUF_LEN];

    int msgsize = 0, ret = -5;

    if (datalen){
		msg0[msgsize++] = mCH341A_CMD_I2C_STREAM;
		msg0[msgsize++] = mCH341A_CMD_I2C_STM_STA;
		msg0[msgsize++] = mCH341A_CMD_I2C_STM_OUT | ( datalen + 1 ); // addr byte + len
		msg0[msgsize++] =    addr & 0xfe;

		memcpy(msg0 + msgsize, data, datalen);
		msgsize += datalen;

		msg0[msgsize++] = mCH341A_CMD_I2C_STM_STO;
		msg0[msgsize++] = mCH341A_CMD_I2C_STM_END;

		ret = ch341_usb_cmd_msg(dev, msg0, msgsize,0);
		return ret;
	}
	return -EIO;

}


static int ch341_set_speed(struct i2c_ch341_u2c *dev, u8 speed)
{
    u8 msg[] = {
        mCH341A_CMD_I2C_STREAM,
        mCH341A_CMD_I2C_STM_SET | (speed & 0x03),
        mCH341A_CMD_I2C_STM_END
    };
    return ch341_usb_cmd_msg(dev, msg, 3,0);
}


static int ch341_init(struct i2c_ch341_u2c *dev)
{
    int speed, ret;
    int speeds[4]={20,100,400,750};

    //dev_info(&dev->interface->dev, "%s", __FUNCTION__);

    if (frequency >CH341_I2C_FAST_SPEED) {
        speed = CH341_I2C_FAST_SPEED;
    } else {
        speed = frequency & 0x03;
    }

    dev_info(&dev->interface->dev,
             "CH341 U2C at USB bus %03d address %03d speed %d kHz\n",
             dev->usb_dev->bus->busnum, dev->usb_dev->devnum, speeds[speed]);

    /* Set I2C speed */
    ret = ch341_set_speed(dev, speed);
    if (ret < 0) {
        return ret;
    }
    return ret;
}

/* i2c layer */

static int ch341_usb_xfer(struct i2c_adapter *adapter, struct i2c_msg *msgs,
                          int num)
{
  struct i2c_ch341_u2c *dev = i2c_get_adapdata(adapter);
  struct i2c_msg *pmsg;
  int i, ret;

  ret = ch341_usb_cmd_check_presence(dev, msgs[0].addr << 1 );

  if (ret< 0 )
	return ret;

  for (i = 0; i < num && msgs[0].len; i++) {
      pmsg = &msgs[i];

      if (pmsg->flags & I2C_M_RD) {
          ret = ch341_usb_cmd_read_addr(dev, pmsg->addr << 1 , pmsg->buf, pmsg->len);

          if (ret < 0) {
              goto abort;
          }
      } else {
          ret = ch341_usb_cmd_write_addr(dev, pmsg->addr << 1, pmsg->buf, pmsg->len);
          if (ret < 0) {
              goto abort;
          }
      }
  }
  ret = num;
abort:
  return ret ;

}

/*
 * Return list of supported functionality.
 */
static u32 ch341_usb_func(struct i2c_adapter *a)
{
    return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL ;

}

static const struct i2c_algorithm ch341_usb_algorithm = {
    .master_xfer = ch341_usb_xfer,
    .functionality = ch341_usb_func,
};

/* device layer */

static const struct usb_device_id ch341_u2c_table[] = {
    { USB_DEVICE(USB_VENDOR_ID_CH341, USB_DEVICE_ID_CH341_U2C) },
    { }
};

MODULE_DEVICE_TABLE(usb, ch341_u2c_table);

static void ch341_u2c_free(struct i2c_ch341_u2c *dev)
{
    usb_put_dev(dev->usb_dev);
    kfree(dev);
}

static int ch341_u2c_probe(struct usb_interface *interface,
                           const struct usb_device_id *id)
{
    struct usb_device *udev;
    struct usb_host_interface *hostif = interface->cur_altsetting;
    struct i2c_ch341_u2c *dev;
    const int ifnum = interface->altsetting[DEFAULT_CONFIGURATION].desc.bInterfaceNumber;
    char *speed;
    int ret;

    if (hostif->desc.bInterfaceNumber != 0
            || hostif->desc.bNumEndpoints < 2) {
        return -ENODEV;
    }

    /* allocate memory for our device state and initialize it */
    dev = kzalloc(sizeof(*dev), GFP_KERNEL);
    if (dev == NULL) {
        dev_err(&interface->dev, "no memory for device state\n");
        ret = -ENOMEM;
        goto error;
    }
    dev->ep_out = hostif->endpoint[1].desc.bEndpointAddress;
    //dev_info(&interface->dev, "ep_out=%x\n", dev->ep_out);
    dev->ep_in = hostif->endpoint[0].desc.bEndpointAddress;
    //dev_info(&interface->dev, "ep_in=%x\n", dev->ep_in);

    dev->usb_dev = usb_get_dev(interface_to_usbdev(interface));
    dev->interface = interface;

    udev = dev->usb_dev;
    switch (udev->speed) {
    case USB_SPEED_LOW:
        speed = "1.5";
        break;
    case USB_SPEED_UNKNOWN:
    case USB_SPEED_FULL:
        speed = "12";
        break;
    case USB_SPEED_HIGH:
        speed = "480";
        break;
    default:
        speed = "unknown";
    }
    printk(KERN_INFO DRIVER_NAME
           ": New device %s %s @ %s Mbps "
           "(%04x:%04x, interface %d, class %d, version %d.%02d)\n",
           udev->manufacturer ? udev->manufacturer : "",
           udev->product ? udev->product : "",
           speed,
           le16_to_cpu(udev->descriptor.idVendor),
           le16_to_cpu(udev->descriptor.idProduct),
           ifnum,
           interface->altsetting->desc.bInterfaceNumber,
           le16_to_cpu(udev->descriptor.bcdDevice % 0xff),
           le16_to_cpu(udev->descriptor.bcdDevice >> 8 % 0xff)
          );

    /* save our data pointer in this interface device */
    usb_set_intfdata(interface, dev);

    /* setup i2c adapter description */
    dev->adapter.owner = THIS_MODULE;
    dev->adapter.class = I2C_CLASS_HWMON;
    dev->adapter.algo = &ch341_usb_algorithm;
    i2c_set_adapdata(&dev->adapter, dev);
    snprintf(dev->adapter.name, sizeof(dev->adapter.name),
             DRIVER_NAME " at bus %03d device %03d",
             dev->usb_dev->bus->busnum, dev->usb_dev->devnum);

    dev->adapter.dev.parent = &dev->interface->dev;
    /* initialize ch341 i2c interface */
    ret = ch341_init(dev);

    if (ret < 0) {
        dev_err(&interface->dev, "failed to initialize adapter\n");
        goto error_free;
    }
    /* and finally attach to i2c layer */
    ret = i2c_add_adapter(&dev->adapter);
    if (ret < 0) {
        dev_err(&interface->dev, "failed to add I2C adapter\n");
        goto error_free;
    }

    dev_dbg(&interface->dev, "connected " DRIVER_NAME "\n");

    return 0;

error_free:
    usb_set_intfdata(interface, NULL);
    ch341_u2c_free(dev);
error:
    return ret;
}

static void ch341_u2c_disconnect(struct usb_interface *interface)
{
    struct i2c_ch341_u2c *dev = usb_get_intfdata(interface);

    i2c_del_adapter(&dev->adapter);
    usb_set_intfdata(interface, NULL);
    ch341_u2c_free(dev);

    dev_dbg(&interface->dev, "disconnected\n");
}

static int ch341_u2c_suspend(struct usb_interface *intf, pm_message_t message )
{
	return 0;
}

static int ch341_u2c_resume( struct usb_interface *intf )
{
	return 0;
}


static struct usb_driver ch341_u2c_driver = {
    .name = DRIVER_NAME,
    .probe = ch341_u2c_probe,
    .disconnect = ch341_u2c_disconnect,
    .id_table = ch341_u2c_table,
    .supports_autosuspend = 1,
    .suspend	= ch341_u2c_suspend,
    .resume		= ch341_u2c_resume,
};

module_usb_driver(ch341_u2c_driver);

MODULE_AUTHOR("Marco Gittler <git.marco@gmail.com>");
MODULE_DESCRIPTION(DRIVER_NAME " ch341driver");
MODULE_LICENSE("GPL");
