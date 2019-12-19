#ifndef _DISK_PART_EFI_H
#define _DISK_PART_EFI_H

#ifndef CONFIG_PARTITIONS_TABLE_SIZE
#define CONFIG_PARTITIONS_TABLE_SIZE 16
#endif
#ifndef CONFIG_PARTITIONS_LOGICAL_OFFSET
#define CONFIG_PARTITIONS_LOGICAL_OFFSET  128
#endif

#ifdef __cplusplus
extern "C" {
#endif

#ifndef __packed
#define __packed __attribute__((packed))
#endif

typedef struct
{
    uint8_t b[16];
} efi_guid_t;

#define SUNXI_MBR_SIZE  (16 * 1024)
#define GPT_HEADER_SIGNATURE 0x5452415020494645ULL

typedef uint16_t efi_char16_t;

typedef struct _gpt_header
{
    uint64_t signature;
    uint32_t revision;
    uint32_t header_size;
    uint32_t header_crc32;
    uint32_t reserved1;
    uint64_t my_lba;
    uint64_t alternate_lba;
    uint64_t first_usable_lba;
    uint64_t last_usable_lba;
    efi_guid_t disk_guid;
    uint64_t partition_entry_lba;
    uint32_t num_partition_entries;
    uint32_t sizeof_partition_entry;
    uint32_t partition_entry_array_crc32;
} __packed gpt_header;

typedef union _gpt_entry_attributes
{
    struct
    {
        uint64_t required_to_function: 1;
        uint64_t no_block_io_protocol: 1;
        uint64_t legacy_bios_bootable: 1;
        /* uint64_t reserved:45; */
        uint64_t reserved: 27;
        uint64_t user_type: 16;
        uint64_t ro: 1;
        uint64_t keydata: 1;
        uint64_t type_guid_specific: 16;
    } fields;
    unsigned long long raw;
} __packed gpt_entry_attributes;

#define PARTNAME_SZ    (72 / sizeof(efi_char16_t))
typedef struct _gpt_entry
{
    efi_guid_t partition_type_guid;
    efi_guid_t unique_partition_guid;
    uint64_t starting_lba;
    uint64_t ending_lba;
    gpt_entry_attributes attributes;
    efi_char16_t partition_name[PARTNAME_SZ];
} __packed gpt_entry;

#define  GPT_ENTRY_OFFSET        1024
#define  GPT_HEAD_OFFSET         512

#define GPT_TABLE_SIZE (CONFIG_PARTITIONS_TABLE_SIZE * 1024)
#define GPT_START_MAPPING (CONFIG_PARTITIONS_LOGICAL_OFFSET * 1024)
#define GPT_ADDRESS (GPT_START_MAPPING - GPT_TABLE_SIZE)

struct gpt_part
{
    char name[PARTNAME_SZ];
    uint32_t off_sects;
    uint32_t sects;
    int index;

    gpt_header *header;
    gpt_entry *entry;
};
struct gpt_part *first_gpt_part(void *gpt_buf);
struct gpt_part *next_gpt_part(struct gpt_part *part);

#define foreach_gpt_part(gpt_buf, gpt_part)                 \
    for (gpt_part = first_gpt_part(gpt_buf);                \
         gpt_part; gpt_part = next_gpt_part(gpt_part))

int gpt_part_cnt(void *gpt_buf);
int show_gpt_part(void *gpt_buf);

#ifdef __cplusplus
}
#endif

#endif    /* _DISK_PART_EFI_H */
