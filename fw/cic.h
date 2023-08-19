#pragma once

enum CicType {
    CicType6101 = 0,
    CicType6102,
    CicType6103,
    CicType6105,
    CicType6106,
    CicType7102,
    _CicTypeMax_
};

void cic_run(void);
void select_cic(enum CicType type);
void reset_cic();

const char* cic_get_name(enum CicType type);
enum CicType cic_easy_detect(uint32_t keydata);

#define N64_BOOTCODE_OFFSET 0x40
#define CIC_DETECT_OFFSET (N64_BOOTCODE_OFFSET + 0x550)