#ifndef SRECORDER_SAHARA_H
#define SRECORDER_SAHARA_H


#ifdef __cplusplus
extern "C" {
#endif
#define SAHARA_BOOT_LOG_ADDR  0x8cb00000
#define SAHARA_BOOT_LOG_SIZE  0x00050000
#define SAHARA_BOOT_LOG_SIZE_MAX  0x00050000

struct sahara_boot_log
{
    uint32_t sbl_log_addr;
    uint32_t lk_log_addr;
    uint32_t kernel_log_addr;
    uint32_t sbl_log_size;
    uint32_t lk_log_size;
    uint32_t kernel_log_size;
};

void srecorder_save_kernel_log_addr(void);

#ifdef __cplusplus
}
#endif
#endif /* SRECORDER_STACK_H */
