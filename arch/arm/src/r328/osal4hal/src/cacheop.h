/*
 * ===========================================================================================
 *
 *       Filename:  cacheop.h
 *
 *    Description:  cacheop for arch
 *
 *        Version:  Melis3.0 
 *         Create:  2019-12-16 15:47:02
 *       Revision:  none
 *       Compiler:  GCC:version 7.2.1 20170904 (release),ARM/embedded-7-branch revision 255204
 *
 *         Author:  caozilong@allwinnertech.com
 *   Organization:  BU1-PSW
 *  Last Modified:  2019-12-16 15:49:41
 *
 * ===========================================================================================
 */

#ifndef __CACHE_OP__
#define __CACHE_OP__

void awos_arch_mems_clean_dcache_region(unsigned long vaddr_start, unsigned long size);
void awos_arch_mems_clean_flush_dcache_region(unsigned long vaddr_start, unsigned long size);
void awos_arch_mems_flush_dcache_region(unsigned long vaddr_start, unsigned long size);
void awos_arch_clean_dcache(void);
void awos_arch_flush_icache_all(void);

#endif
