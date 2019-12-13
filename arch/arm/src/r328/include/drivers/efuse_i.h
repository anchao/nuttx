/*
**********************************************************************************************************************
*
*						           the Embedded Secure Bootloader System
*
*
*						       Copyright(C), 2006-2014, Allwinnertech Co., Ltd.
*                                           All Rights Reserved
*
* File    :
*
* By      :
*
* Version : V2.00
*
* Date	  :
*
* Descript:
**********************************************************************************************************************
*/

#ifndef __SUNXI_SID_H__
#define __SUNXI_SID_H__

#define SUNXI_SID_BASE			0x03006000
#define SID_PRCTL				(SUNXI_SID_BASE + 0x40)
#define SID_PRKEY				(SUNXI_SID_BASE + 0x50)
#define SID_RDKEY				(SUNXI_SID_BASE + 0x60)
#define SJTAG_AT0				(SUNXI_SID_BASE + 0x80)
#define SJTAG_AT1				(SUNXI_SID_BASE + 0x84)
#define SJTAG_S					(SUNXI_SID_BASE + 0x88)

#define EFUSE_SRAM              (SUNXI_SID_BASE + 0x200)
#define EFUSE_SECURE_MODE       (SUNXI_SID_BASE + 0xA0)

#define EFUSE_CHIPD             (0x00) /* 0x0-0xf, 128bits */
#define EFUSE_BROM_CONFIG       (0x10) /* 16 bit config, 16 bits try */
#define EFUSE_THERMAL_SENSOR    (0x14) /* 0x14-0x1b, 64bits */
#define EFUSE_TF_ZONE           (0x1C) /* 0x1c-0x2b, 128bits */
#define EFUSE_OEM_PROGRAM       (0x2C) /* 0x2c-0x2f, 32bits */

/* write protect */
#define EFUSE_WRITE_PROTECT     (0x30) /* 0x30-0x33, 32bits */
/* read  protect */
#define EFUSE_READ_PROTECT      (0x34) /* 0x34-0x37, 32bits */
/* jtag security */
#define EFUSE_LCJS              (0x38)


/* jtag attribute */
#define EFUSE_ATTR              (0x3C)

#define EFUSE_ROTPK             (0x40) /* 0x40-0x5f, 256bits */
#define EFUSE_SSK               (0x60) /* 0x60-0x6f, 128bits */
#define EFUSE_NV1               (0x70) /* 0x70-0x73, 32 bits */
#define EFUSE_NV2               (0x74) /* 0x74-0x77,224bits */
#define EFUSE_OEM_PROGRAM_SECURE (0x78)/* 64 bits */


#define SID_OP_LOCK  (0xAC)

#define SECURE_BIT_OFFSET 11
/*It can not be seen.*/
#define EFUSE_PRIVATE (0)
/*After burned ,cpu can not access.*/
#define EFUSE_NACCESS (1)

#define EFUSE_RO (2)	/* burn read_protect bit disable */
#define EFUSE_RW (3)	/* burn read/write_protect bit disable */

#define EFUSE_ACL_SET_BRUN_BIT      (1<<29)
#define EFUSE_ACL_SET_RD_FORBID_BIT (1<<30)
#define EFUSE_BRUN_RD_OFFSET_MASK    (0xFFFFFF)

#define EFUSE_DEF_ITEM(name,offset,size_bits,rd_offset,burn_offset,acl) \
{name,offset,size_bits,rd_offset,burn_offset,acl}

typedef enum efuse_err
{
	EFUSE_ERR_ARG = -1,
	EFUSE_ERR_KEY_NAME_WRONG = -2,
	EFUSE_ERR_KEY_SIZE_TOO_BIG = -3,
	EFUSE_ERR_PRIVATE = -4,
	EFUSE_ERR_ALREADY_BURNED = -5,
	EFUSE_ERR_READ_FORBID = -6,
	EFUSE_ERR_BURN_TIMING = -7,
	EFUSE_ERR_NO_ACCESS = -8,
	EFUSE_ERR_INVALID_ROTPK = -9,
}
efuse_err_e;

/* internal struct */
typedef struct efuse_key_map_new{
	#define SUNXI_KEY_NAME_LEN	64
	char name[SUNXI_KEY_NAME_LEN];	/* key_name */
	int offset;	/* key_addr offset */
	int size;	 /* unit: bit */
	int rd_fbd_offset;	/* key can read or not */
	int burned_flg_offset;	/* key has burned or not */
	int sw_rule;
}efuse_key_map_new_t;

efuse_key_map_new_t *efuse_search_key_by_name(const char *key_name);
int efuse_acl_ck(efuse_key_map_new_t *key_map,int burn);
void efuse_set_cfg_flg(int efuse_cfg_base,int bit_offset);
int efuse_uni_burn_key(unsigned int key_index, unsigned int key_value);
unsigned int efuse_sram_read_key(unsigned int key_index);
int efuse_set_security_mode(void);
int efuse_get_security_mode(void);

#endif    /*  #ifndef __EFUSE_H__  */
