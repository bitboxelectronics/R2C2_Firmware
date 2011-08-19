//-----------------------------------------------------------------------------
// Software that is described herein is for illustrative purposes only
// which provides customers with programming information regarding the
// products. This software is supplied "AS IS" without any warranties.
// NXP Semiconductors assumes no responsibility or liability for the
// use of the software, conveys no license or title under any patent,
// copyright, or mask work right to the product. NXP Semiconductors
// reserves the right to make changes in the software without
// notification. NXP Semiconductors also make no representation or
// warranty that such application will be suitable for the specified
// use without further testing or modification.
//-----------------------------------------------------------------------------

/***********************************************************************
 * Code Red Technologies - Minor modifications to original NXP AN10866
 * example code for use in RDB1768 secondary USB bootloader based on
 * LPCUSB USB stack.
 *
 * diskimage.c - definition of boot sector and root directory for FAT12
 *               tables used for accessing LPC1768 flash by RDB1768 USB
 *               bootloader.
 *
 * *********************************************************************/


#include "disk.h"

/* FAT12 Boot sector constants */
const unsigned char BootSect[] = {
0xEB,0x3C,0x90, /*  Instructions to jump to boot code */
0x4D,0x53,0x44,0x4F,0x53,0x35,0x2E,0x30, /*  Name string (MSDOS5.0) */
0x00,0x02, /* Bytes/sector (0x0200 = 512) */
BLOCKS_PER_CLUSTER, /* Sectors/cluster (64) -- 64*512bytes = 32kbytes each cluster */
0x01,0x00, /* Size of reserved area (1 sector) */
0x01, /* Number of FATs (1) */
0x10,0x00, /* Max. number of root directory entries (0x0010 = 16) */
0x80,0x43, /* Total number of sectors (0x0384 = 900) */
0xF8, /* Media type (Any Hard Drive) */
0x02,0x00, /*  FAT size (0x0002 = 2 sectors) */
0x01,0x00, /* Sectors/track (0x0001 = 1) */
0x01,0x00, /* Number of heads (0x0001 = 1) */
0x00,0x00, /* Number of sector before partition (0) */
};

/* FAT12 Root directory entry constants */
const unsigned char RootDirEntry[DIR_ENTRY] = {
'B', 'o', 'o', 't', 'l', 'o', ' ', 'v', '1', '.', '0',
0x28, /* Archive and volume label */
0x00, /* Reserved */
0x00,
0x00,0x00,
0x00,0x00,
0x00,0x00,
0x00,0x00, /* 0 on FAT12 */
0x00,0x00,
0x00,0x00,
0x00,0x00, /* 0 on FAT12 */
0x00,0x00,0x00,0x00,
'F', 'I', 'R', 'M', 'W', 'A', 'R', 'E', 'B', 'I', 'N',0x20,0x18,0xbc,0x41,0x97,
0x37,0x38,0x37,0x38,0x00,0x00,0x3d,0x6e,0x2b,0x38,0x02,0x00,0x00,0xD0,0x07,0x00,
 };

/* RAM to store the file allocation table */
unsigned char  Fat_RootDir[FAT_SIZE + ROOT_DIR_SIZE];



