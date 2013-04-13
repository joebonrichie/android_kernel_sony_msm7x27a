/*************************************************************************************
 *
 * FIH Project
 *
 * General Description
 * 
 * Linux kernel: Definitions of SW related information for SEMC.
 *	
 * Copyright(C) 2011-2012 Foxconn International Holdings, Ltd. All rights reserved.
 * Copyright (C) 2011, Foxconn Corporation (BokeeLi@fih-foxconn.com)
 *
 */
/*************************************************************************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/sysdev.h>
#include <linux/fih_sw_info.h>
#include <asm/setup.h>

// For HW ID
//#include <linux/mfd/88pm860x.h>
// For SW_ID SW_INFO sync
#include <linux/mtd/mtd.h>
//#include <mach/Testflag.h>

#define ID2NAME(ID,NAME)  {(unsigned int)(ID), (char*)(NAME)}

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a) (sizeof(a)/sizeof(a[0]))
#endif

/* FIH-SW3-KERNEL-DL-Store_log_for_MTBF-00+[ */
#define DIAG_BUFFER_LEN		0xC8
#define STORE_FATAL_ERROR_REASON_ADDR (MTD_RAM_CONSOLE_ADDR + 0xa0000)
#define STORE_FATAL_ERROR_REASON STORE_FATAL_ERROR_REASON_ADDR
/* FIH-SW3-KERNEL-DL-Store_log_for_MTBF-00+] */

typedef struct 
{
	unsigned int flash_id;
	char *flash_name;
}flash_id_number_name_map;

typedef struct
{
	unsigned int hw_id_number;
	char *hw_id_name;
}hw_id_number_name_map;

//	ID2NAME(0xBC2C, "Micron"),
flash_id_number_name_map flash_id_map_table[] = 
{
	ID2NAME(0x00000000, "ONFI"),
	ID2NAME(0x1500aaec, "Sams"),
	ID2NAME(0x5500baec, "Sams"),
	ID2NAME(0x1500aa98, "Tosh"),
	ID2NAME(0x5500ba98, "Tosh"),
	ID2NAME(0xd580b12c, "Micr"),
	ID2NAME(0x5590bc2c, "Micr"),
	ID2NAME(0x1580aa2c, "Micr"),
	ID2NAME(0x1590aa2c, "Micr"),
	ID2NAME(0x1590ac2c, "Micr"),
	ID2NAME(0x5580baad, "Hynx"),
	ID2NAME(0x5510baad, "Hynx"),
	ID2NAME(0x004000ec, "Sams"),
	ID2NAME(0x005c00ec, "Sams"),
	ID2NAME(0x005800ec, "Sams"),
	ID2NAME(0x6600bcec, "Sams"),
	ID2NAME(0x5580ba2c, "Hynx"),
	ID2NAME(0x6600b3ec, "Sams"),
	ID2NAME(0x6601b3ec, "Sams"),
	ID2NAME(0xffffffff, "unknown flash")
};

/*
	export fih_get_product_id(),fih_get_product_phase() and fih_get_band_id() in .\LINUX\kernel\arch\arm\mach-msm\smd.c
*/
extern unsigned int fih_get_product_id(void);
extern unsigned int fih_get_product_phase(void);
extern unsigned int fih_get_band_id(void);

hw_id_number_name_map hw_id_map_table[] = 
{
	ID2NAME(Phase_EVB, "EVB"),
	ID2NAME(Phase_DP, "DP"),
	ID2NAME(Phase_SP, "SP"),
	ID2NAME(Phase_SP2, "SP2"), /* MTD-BSP-VT-HWID-02+ */
	ID2NAME(Phase_SP3, "SP3"),
	ID2NAME(Phase_PreAP, "Pre AP"),
	ID2NAME(Phase_AP, "AP"),
	ID2NAME(Phase_AP2, "AP2"),
	ID2NAME(Phase_TP, "TP"),  /* MTD-BSP-VT-HWID-01+ */
	ID2NAME(Phase_TP2, "TP2"), 
	ID2NAME(Phase_TP3, "TP3"), 
	ID2NAME(Phase_PQ, "PQ"),
	ID2NAME(Phase_MP, "MP"),
	ID2NAME(PHASE_MAX, "unknown hw id")
};

hw_id_number_name_map band_id_map_table[] = 
{
	ID2NAME(BAND_18, "18"),
	ID2NAME(BAND_125, "125"),
	ID2NAME(BAND_MAX, "unknown band id")
};

hw_id_number_name_map project_id_map_table[] = 
{
	ID2NAME(Project_GUA, "GUA"),
	ID2NAME(Project_BMB, "BMB"),
	//MTD-BSP-REXER-HWID-01+[
	ID2NAME(Project_TAP, "TAP"),
	ID2NAME(Project_MES, "MES"),
	//MTD-BSP-REXER-HWID-01+]
	/* MTD-BSP-VT-HWID-01+[ */
	ID2NAME(Project_JLO, "JLO"),
	/* MTD-BSP-VT-HWID-01+[ */
	ID2NAME(PROJECT_MAX, "unknown project id")
};

static struct kobject *sw_info_kobj = NULL;

#define sw_info_attr(_name) \
static struct kobj_attribute _name##_attr = {	\
	.attr	= {				\
		.name = __stringify(_name),	\
		.mode = 0644,			\
	},					\
	.show	= _name##_show,			\
	.store	= _name##_store,		\
}

#define sw_info_func_init(type,name,initval)   \
	static type name = (initval);       \
	type get_##name(void)               \
	{                                   \
	    return name;                    \
	}                                   \
	void set_##name(type __##name)      \
	{                                   \
	    name = __##name;                \
	}                                   \
	EXPORT_SYMBOL(set_##name);          \
	EXPORT_SYMBOL(get_##name);



/* FIH-SW3-KERNEL-EL-fix_coverity-issues-02*[ */
static ssize_t fih_complete_sw_version_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char *s = buf;

	/* PartNumber_R1AAxxx --> ex: 1248-5695_R1AA001_DEV */
	s += snprintf(s, PAGE_SIZE - ((size_t)(s-buf)), SEMC_PartNumber);
	s += snprintf(s, PAGE_SIZE - ((size_t)(s-buf)), "_");
	s += snprintf(s, PAGE_SIZE - ((size_t)(s-buf)), SEMC_SwRevison);
	if(strlen(SEMC_ReleaseFlag) != 0)
	{
		s += snprintf(s, PAGE_SIZE - ((size_t)(s-buf)), "_");
		s += snprintf(s, PAGE_SIZE - ((size_t)(s-buf)), SEMC_ReleaseFlag);
	}
	s += snprintf(s, PAGE_SIZE - ((size_t)(s-buf)), "\n");
	
	return (s - buf);
}
/* FIH-SW3-KERNEL-EL-fix_coverity-issues-02*] */

static ssize_t fih_complete_sw_version_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	return n;
}

sw_info_attr(fih_complete_sw_version);


/* FIH-SW3-KERNEL-EL-fix_coverity-issues-02*[ */
static ssize_t fih_sw_version_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char *s = buf;

	s += snprintf(s, PAGE_SIZE - ((size_t)(s-buf)), SEMC_SwRevison);
	s += snprintf(s, PAGE_SIZE - ((size_t)(s-buf)), "\n");
	
	return (s - buf);
}
/* FIH-SW3-KERNEL-EL-fix_coverity-issues-02*] */

static ssize_t fih_sw_version_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	return n;
}

sw_info_attr(fih_sw_version);

/* FIH-SW3-KERNEL-EL-fix_coverity-issues-02*[ */
static ssize_t fih_part_number_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char *s = buf;

	s += snprintf(s, PAGE_SIZE - ((size_t)(s-buf)), SEMC_PartNumber);
	s += snprintf(s, PAGE_SIZE - ((size_t)(s-buf)), "\n");
	
	return (s - buf);
}
/* FIH-SW3-KERNEL-EL-fix_coverity-issues-02*] */
static ssize_t fih_part_number_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	return n;
}

sw_info_attr(fih_part_number);

/* FIH-SW3-KERNEL-EL-fix_coverity-issues-02*[ */
static ssize_t fih_hw_id_version_number_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char *s = buf;

	unsigned int hw_id;
	hw_id = fih_get_product_phase();

	s += snprintf(s, PAGE_SIZE - ((size_t)(s-buf)), "%d", hw_id);
	s += snprintf(s, PAGE_SIZE - ((size_t)(s-buf)), "\n");

	return (s - buf);
}
/* FIH-SW3-KERNEL-EL-fix_coverity-issues-02*] */

static ssize_t fih_hw_id_version_number_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	return n;
}

sw_info_attr(fih_hw_id_version_number);


static ssize_t fih_hw_id_version_name_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char *s = buf;
	unsigned int hw_id;
	int i;
	
	hw_id = fih_get_product_phase();

/* FIH-SW3-KERNEL-EL-fix_coverity-issues-00*[ */
	for( i = 0 ; i < sizeof(hw_id_map_table)/sizeof(hw_id_number_name_map); i++)
	{
		if(hw_id_map_table[i].hw_id_number == hw_id)
		{
/* FIH-SW3-KERNEL-EL-fix_coverity-issues-02*[ */			
			s += snprintf(s, PAGE_SIZE - ((size_t)(s-buf)), "%s", hw_id_map_table[i].hw_id_name);
			s += snprintf(s, PAGE_SIZE - ((size_t)(s-buf)), "\r\n");
/* FIH-SW3-KERNEL-EL-fix_coverity-issues-02*] */
			break;
		}
	}
/* FIH-SW3-KERNEL-EL-fix_coverity-issues-00*] */

	return (s - buf);
}

static ssize_t fih_hw_id_version_name_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	return n;
}

sw_info_attr(fih_hw_id_version_name);


static ssize_t fih_flash_id_version_number_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char *s = buf;

	unsigned int flash_id =0 ;
/* FIH-SW3-KERNEL-EL-fix_coverity-issues-02*[ */
	s += snprintf(s, PAGE_SIZE - ((size_t)(s-buf)), "%X", flash_id);
	s += snprintf(s, PAGE_SIZE - ((size_t)(s-buf)), "\n");
/* FIH-SW3-KERNEL-EL-fix_coverity-issues-02*] */
	return (s - buf);
}

static ssize_t fih_flash_id_version_number_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	return n;
}

sw_info_attr(fih_flash_id_version_number);

static ssize_t fih_flash_id_version_name_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char *s = buf;
	unsigned int flash_id = 0;
	int idx;
	int array_max_idx = ARRAY_SIZE(flash_id_map_table) - 1;

	for (idx = 0; idx <= array_max_idx; idx ++)
	{
		if(flash_id_map_table[idx].flash_id == flash_id)
		{
			break;
		}
	}

	if (idx > array_max_idx)
		idx = array_max_idx;
/* FIH-SW3-KERNEL-EL-fix_coverity-issues-02*[ */
	s += snprintf(s, PAGE_SIZE - ((size_t)(s-buf)), "%s", flash_id_map_table[idx].flash_name);
	s += snprintf(s, PAGE_SIZE - ((size_t)(s-buf)), "\n");
/* FIH-SW3-KERNEL-EL-fix_coverity-issues-02*] */
	return (s - buf);
}

static ssize_t fih_flash_id_version_name_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	return n;
}

sw_info_attr(fih_flash_id_version_name);


static ssize_t fih_blob_version_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char *s = buf;
/* FIH-SW3-KERNEL-EL-fix_coverity-issues-02*[ */
	s += snprintf(s, PAGE_SIZE - ((size_t)(s-buf)), "%s", SEMC_BlobVersion);
	s += snprintf(s, PAGE_SIZE - ((size_t)(s-buf)), "\n");
/* FIH-SW3-KERNEL-EL-fix_coverity-issues-02*] */	

	return (s - buf);
}

static ssize_t fih_blob_version_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	return n;
}

sw_info_attr(fih_blob_version);


static ssize_t fih_bp_version_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char *s = buf;
	
/* FIH-SW3-KERNEL-EL-fix_coverity-issues-02*[ */
	s += snprintf(s, PAGE_SIZE - ((size_t)(s-buf)), "%s", SEMC_BpVersion);
	s += snprintf(s, PAGE_SIZE - ((size_t)(s-buf)), "\n");
/* FIH-SW3-KERNEL-EL-fix_coverity-issues-02*] */	

	return (s - buf);
}

static ssize_t fih_bp_version_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	return n;
}

sw_info_attr(fih_bp_version);


static ssize_t fih_svn_version_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char *s = buf;

/* FIH-SW3-KERNEL-EL-fix_coverity-issues-02*[ */
	s += snprintf(s, PAGE_SIZE - ((size_t)(s-buf)), "%s", SEMC_SvnVersion);
	s += snprintf(s, PAGE_SIZE - ((size_t)(s-buf)), "\n");
/* FIH-SW3-KERNEL-EL-fix_coverity-issues-02*] */

	return (s - buf);
}

static ssize_t fih_svn_version_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	return n;
}

sw_info_attr(fih_svn_version);

/* FIH-SW3-KERNEL-EL-get_last_alog_buffer_virt_addr-01+ */
void * get_alog_buffer_virt_addr(void);

static struct attribute * g[] = {
	&fih_complete_sw_version_attr.attr,
	&fih_sw_version_attr.attr,
	&fih_part_number_attr.attr,
	&fih_hw_id_version_number_attr.attr,
	&fih_hw_id_version_name_attr.attr,
	&fih_flash_id_version_number_attr.attr,
	&fih_flash_id_version_name_attr.attr,
	&fih_blob_version_attr.attr,
	&fih_bp_version_attr.attr,
	&fih_svn_version_attr.attr,
	NULL,
};


static struct attribute_group attr_group = {
	.attrs = g,
};


static int sw_info_sync(void)
{
	return 0;
}

static int __init sw_info_init(void)
{
	int ret = -ENOMEM;

	sw_info_kobj = kobject_create_and_add("fih_sw_info", NULL);
	if (sw_info_kobj == NULL) {
		printk("sw_info_init: subsystem_register failed\n");
		goto fail;
	}

	ret = sysfs_create_group(sw_info_kobj, &attr_group);
	if (ret) {
		printk("sw_info_init: subsystem_register failed\n");
		goto sys_fail;
	}

	sw_info_sync();

	return ret;

sys_fail:
	kobject_del(sw_info_kobj);
fail:
	return ret;

}

static void __exit sw_info_exit(void)
{
	if (sw_info_kobj) {
		sysfs_remove_group(sw_info_kobj, &attr_group);
		kobject_del(sw_info_kobj);
	}
}

late_initcall(sw_info_init);
module_exit(sw_info_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Eric Liu <huaruiliu@fihspec.com>");
MODULE_DESCRIPTION("SW information collector");
