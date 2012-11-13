
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/switch.h>
#include <linux/device.h>
#include <linux/miscdevice.h>


MODULE_LICENSE("GPL");


static struct cdev c_dev;
static struct switch_dev sdev = {
	.name = "usb_audio",
};


//-----------------------------------------------------------------------------
//	Name	: usb_speaker_driver_write
//	Brief	: This function check the content in written this device file.
//			  If content is "0" or "4", these state passes switch event.
//			  It execute when device file is written something.
//	Param	: filp(cfile*)		information in the written device file
//			  buff(const char*)	content is in written device file
//			  count(size_t)		content length
//			  pos(loff_t*)		position of access file
//	Return : strlen(buff)		buff(argument) length
//-----------------------------------------------------------------------------
static int
usb_speaker_driver_write(struct file* filp, const char* buff, size_t count, loff_t* pos)
{
	printk(KERN_DEBUG "usb_speaker_driver_write\n");

	if (buff == NULL) {
		printk(KERN_ERR "write buffer is NULL\n");
		return -1;
	}

	if (strcmp(buff, "4") && strcmp(buff, "0")) {
		printk(KERN_ERR "invalid value(%s)\n", buff);
		return -1;
	}

	int state = buff[0] - '0';
	printk(KERN_DEBUG "write state : %d\n", state);

	switch_set_state(&sdev, state);
	printk(KERN_DEBUG "success usb_speaker_driver_write\n");

	return strlen(buff);
}



//-----------------------------------------------------------------------------
//	Name	: file_operations
//	Brief	: This struct regist this driver's function to kernel.
//	param	: owner	device driver's owner
//						(THIS_MODULE means default)
// 			  write	function is called when written device files.
//-----------------------------------------------------------------------------
static struct file_operations chardev_fops = {
	owner   : THIS_MODULE,
	write   : usb_speaker_driver_write,
};


//-----------------------------------------------------------------------------
//	Name	: usb_speaker_device
//	Brief	: This struct is for makes character device file.
//	param	: minor	device file's minor number
//						(MISC_DYNAMIC_MINOR means dynamic number)
// 			  name		device file's name
//			  fops		character device infomation
//-----------------------------------------------------------------------------
static struct miscdevice usb_speaker_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "usb_speaker",
	.fops = &chardev_fops,
};


//-----------------------------------------------------------------------------
//	Name	: usb_speaker_driver_init
//	Brief	: This function makes character device file, and
//			  prepores to regist switch devices driver.
//			  When this character devices driver(Kobject file) loads,
//			  this function called.
//	Param	: nothing
//	Return : 0			-> success
//			  other	-> failed
//-----------------------------------------------------------------------------
static int __init
usb_speaker_driver_init(void)
{
	printk(KERN_DEBUG "usb_speaker_driver_init\n");

	int err = 0;

	err = misc_register(&usb_speaker_device);
	if (err < 0) {
		printk(KERN_ERR "fail to misc_register(%d)\n", err);
		return -1;
	}

	err = switch_dev_register(&sdev);
	if (err < 0) {
		misc_deregister(&usb_speaker_device);
		printk(KERN_ERR "fail to switch_dev_register(%d)\n", err);

		return -1;
	}

	printk(KERN_DEBUG "success usb_speaker_driver_init\n");
	return 0;
}


//-----------------------------------------------------------------------------
//	Name	: usb_speaker_driver_cleanup
//	Brief	: This function unregist switch device driver, and
//			  removes character devices device.
//			  When no process of kernel module that are reference,
//			  this function called.
//	Param	: void
//	Return : void
//-----------------------------------------------------------------------------
static void __exit
usb_speaker_driver_cleanup(void)
{
	printk(KERN_DEBUG "usb_speaker_driver_cleanup\n");

	switch_dev_unregister(&sdev);
	misc_deregister(&usb_speaker_device);

	printk(KERN_DEBUG "success usb_speaker_driver_cleanup\n");
	return;
}

module_init(usb_speaker_driver_init);
module_exit(usb_speaker_driver_cleanup);

