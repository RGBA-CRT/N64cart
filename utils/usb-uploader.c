/**
 * Copyright (c) 2022 sashz /pdaXrom.org/
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libusb.h>
#include "utils.h"

#define RETRY_MAX	50 

static int bulk_transfer(struct libusb_device_handle *devh, unsigned char endpoint,
                  unsigned char *data, int length, int *transferred, unsigned int timeout)
{
    int ret;
    int try = 0;
    do {
        ret = libusb_bulk_transfer(devh, endpoint, data, length, transferred, timeout);
        if (ret == LIBUSB_ERROR_PIPE) {
            //fprintf(stderr, "usb stalled, retry\n");
            libusb_clear_halt(devh, endpoint);
        }
        try++;
    } while ((ret == LIBUSB_ERROR_PIPE) && (try < RETRY_MAX));
    return ret;
}

static void usage(char *app)
{
    printf("Usage:\n");
    printf("  %s info\n", app);
    printf("  %s rom <page> <rom file>\n", app);
    printf("  %s picture <jpeg file>\n\n", app);
}

static int rom_write(libusb_device_handle *dev_handle, int page, const char* filename, long rom_offset, int rom_type) {
        uint32_t type;
        int err = 0;
        int actual;
        int ret;

        FILE *inf = fopen(filename, "rb");
        if (!inf) {
            fprintf(stderr, "Cannot open file %s\n", filename);
            goto exit;
        }

        
        fread(&type, 1, 4, inf);
        fprintf(stderr, "ROM type ");
        if (type == 0x40123780) {
            type = 0;
            fprintf(stderr, "Z64\n");
        } else if (type == 0x80371240) {
            type = 1;
            fprintf(stderr, "N64\n");
        } else if (type == 0x12408037) {
            type = 2;
            fprintf(stderr, "V64\n");
        } else {
            fprintf(stderr, "Unknown\n\nError!\n");
            goto exit;
        }

        fseek(inf, 0, SEEK_END);
        int size = ftell(inf);
        fseek(inf, rom_offset, SEEK_SET);

        fprintf(stderr, "ROM size %d\n", size);
        if(size <= 0) {
                fprintf(stderr, "ROM size is zero.\n");
                goto exit;
        }
        if(rom_offset > size) {
            fprintf(stderr, "rom offset too big.\n");
            goto exit;
        }

        struct data_header *header = alloca(sizeof(struct data_header));
        struct data_header *header_reply = alloca(sizeof(struct data_header));

        // todo get from device
        #define FLASH_PAGE_SIZE_MAX (16*1024*1024)
        #define FLASH_PAGE_MAX 4
        int write_pages = (page + (size+FLASH_PAGE_SIZE_MAX-1)/FLASH_PAGE_SIZE_MAX - 1) ;
        // int start_page = 
        // if(size % FLASH_PAGE_SIZE_MAX) write_pages++;
        
        fprintf(stderr, "Write to page %d-%d\n", page, write_pages % FLASH_PAGE_MAX);

        int remain_size = size;
        // for(int cur_page = page + (rom_offset/FLASH_PAGE_SIZE_MAX); cur_page!=(write_pages+1); cur_page++){
        for(int i= (rom_offset/FLASH_PAGE_SIZE_MAX); i<(write_pages-page+1); i++){
            //  (cur_page == write_pages) ? (size - FLASH_PAGE_SIZE_MAX*(write_pages-page)) : FLASH_PAGE_SIZE_MAX;
            int current_page_write_size = (remain_size<FLASH_PAGE_SIZE_MAX) ? remain_size : FLASH_PAGE_SIZE_MAX;
            
            printf("write page %d ::: %d bytes\n", i+page, remain_size);
            header->type = DATA_WRITE;
            header->address = rom_offset % FLASH_PAGE_SIZE_MAX;
            header->length = current_page_write_size;
            header->pages = (i+page)% FLASH_PAGE_MAX;
            printf("[cmd] DATA_WRITE, addr=%x len=%x, page=%d\n", header->address, header->length, header->pages);

            bulk_transfer(dev_handle, 0x01, (void *)header, sizeof(struct data_header), &actual, 5000);
            if (actual != sizeof(struct data_header)) {
                fprintf(stderr, "Header error transfer\n");
                goto exit;
            }

            bulk_transfer(dev_handle, 0x82, (void *)header_reply, sizeof(struct data_header), &actual, 5000);
            if (actual != sizeof(struct data_header)) {
                fprintf(stderr, "Header reply error transfer\n");
                goto exit;
            } else if (header_reply->type != DATA_REPLY) {
                fprintf(stderr, "Wrong header reply\n");
                goto exit;
            } else if (header_reply->pages != header->pages) {
                fprintf(stderr, "Wrong page\n");
                goto exit;
            } else if (header_reply->length != header->length) {
                fprintf(stderr, "Wrong ROM size\n");
                goto exit;
            } else {
                uint8_t buf[64];
                uint8_t buf_in[64];

                while (current_page_write_size) {
                    int r = fread(buf, 1, 32, inf);
                    if (r == 32) {
                        if (type) {
                            for (int i = 0; i < 32; i += 4) {
                                uint8_t tmp;
                                if (type == 1) {
                                    tmp = buf[i + 0];
                                    buf[i + 0] = buf[i + 3];
                                    buf[i + 3] = tmp;
                                    tmp = buf[i + 2];
                                    buf[i + 2] = buf[i + 1];
                                    buf[i + 1] = tmp;
                                } else {
                                    tmp = buf[i + 0];
                                    buf[i + 0] = buf[i + 1];
                                    buf[i + 1] = tmp;
                                    tmp = buf[i + 2];
                                    buf[i + 2] = buf[i + 3];
                                    buf[i + 3] = tmp;
                                }
                            }
                        }

                        ret = bulk_transfer(dev_handle, 0x01, buf, 32, &actual, 5000);
                        if (ret) {
                            fprintf(stderr, "data transfer error - libusb error %d\n", ret);
                            break;
                        }
                        if (actual != 32) {
                            fprintf(stderr, "\nData transfer error (%d of 32)\n", actual);
                            break;
                        }

                        bulk_transfer(dev_handle, 0x82, buf_in, sizeof(buf_in), &actual, 5000);
                        if (actual != 32) {
                            fprintf(stderr, "\nData receive error\n");
                            break;
                        }

                        if (memcmp(buf, buf_in, 32)) {
                            fprintf(stderr, "\nDevice received wrong data\n");
                            break;
                        }

                        current_page_write_size -= 32;

                        if ((header->length - current_page_write_size) % 1024 == 0) {
                            printf("Send %d bytes of %d\r", header->length - current_page_write_size, header->length);
                        }
                    } else {
                        fprintf(stderr, "\nError - unaligned ROM file (align 64)\n");
                        break;
                    }
                }

                if (current_page_write_size == 0) {
                    printf("\n");
                    err = 0;
                }
            }
            remain_size -= current_page_write_size;

        }
    exit: 
    return err;
}

int main(int argc, char **argv)
{
    libusb_context *ctx = NULL;
    uint32_t r;
    libusb_device_handle *dev_handle;
    uint8_t data[512];
    int actual;
    int err = 1;

    printf("N64cart USB utility by pdaXrom.org, 2022\n\n");

    if (argc < 2) {
        usage(argv[0]);

        return 0;
    }

    memset(data, 0xFF, sizeof(data));

    r = libusb_init(&ctx);

    if (r < 0) {
        fprintf(stderr, "Error init libusb\n");
        return 1;
    }

    libusb_set_debug(ctx, 3);

    dev_handle = libusb_open_device_with_vid_pid(ctx, 0x1209, 0x6800);

    if (dev_handle == NULL) {
        fprintf(stderr, "Cannot open device, ensure N64cart is attached\n");
        return 1;
    }

    if (libusb_kernel_driver_active(dev_handle, 0) == 1) {
        fprintf(stderr, "Kernel has hold of this device, detaching kernel driver\n");
        libusb_detach_kernel_driver(dev_handle, 0);
    }

    libusb_claim_interface(dev_handle, 0);

    if (!strcmp(argv[1], "info")) {
        struct data_header header;
        header.type = DATA_INFO;

        bulk_transfer(dev_handle, 0x01, (void *)&header, sizeof(struct data_header), &actual, 5000);
        if (actual != sizeof(struct data_header)) {
            fprintf(stderr, "Header error transfer\n");
            goto exit;
        }

        bulk_transfer(dev_handle, 0x82, (void *)&header, sizeof(struct data_header), &actual, 5000);
        if (actual != sizeof(struct data_header)) {
            fprintf(stderr, "Header reply error transfer\n");
            goto exit;
        } else if (header.type != DATA_REPLY) {
            fprintf(stderr, "Wrong header reply\n");
            goto exit;
        } else {
            printf("Page 0\n");
            printf(" Address %08X\n", header.address);
            printf(" Size    %d\n", header.length - header.address);
            for (int i = 1; i < header.pages; i++) {
                printf("Page %d\n", i);
                printf(" Address %08X\n", 0);
                printf(" Size    %d\n", header.length);
            }
        }

        err = 0;
    } else if (!strcmp(argv[1], "picture")) {
        int ret;

        FILE *inf = fopen(argv[2], "rb");
        if (!inf) {
            fprintf(stderr, "Cannot open file %s\n", argv[1]);
            goto exit;
        }

        fseek(inf, 0, SEEK_END);
        int size = ftell(inf);
        fseek(inf, 0, SEEK_SET);

        fprintf(stderr, "Picture size %d\n", size);

        int data_size = (size + 4095) & ~4095;

        fprintf(stderr, "Picture transfer size %d\n", data_size);

        struct data_header *header = alloca(sizeof(struct data_header));
        struct data_header *header_reply = alloca(sizeof(struct data_header));

        header->type = DATA_PICTURE;
        header->address = 0;
        header->length = size;

        bulk_transfer(dev_handle, 0x01, (void *)header, sizeof(struct data_header), &actual, 5000);
        if (actual != sizeof(struct data_header)) {
            fprintf(stderr, "Header error transfer\n");
            goto exit;
        }

        bulk_transfer(dev_handle, 0x82, (void *)header_reply, sizeof(struct data_header), &actual, 5000);
        if (actual != sizeof(struct data_header)) {
            fprintf(stderr, "Header reply error transfer\n");
            goto exit;
        } else if (header_reply->type != DATA_REPLY) {
            fprintf(stderr, "Wrong header reply\n");
            goto exit;
        } else if (header_reply->length != header->length) {
            fprintf(stderr, "Wrong picture size\n");
            goto exit;
        } else {
            uint8_t buf[64];
            uint8_t buf_in[64];
            int saved_size = data_size;

            while (data_size) {
                if (size > 0) {
                    int r = fread(buf, 1, 32, inf);
                    if (r > 0) {
                        size -= r;
                    } else {
                        fprintf(stderr, "Read file error!\n");
                        break;
                    }
                } else {
                    memset(buf, 0xff, sizeof(buf));
                }

                ret = bulk_transfer(dev_handle, 0x01, buf, 32, &actual, 5000);
                if (ret) {
                    fprintf(stderr, "data transfer error - libusb error %d\n", ret);
                    break;
                }
                if (actual != 32) {
                    fprintf(stderr, "\nData transfer error (%d of 32)\n", actual);
                    break;
                }

                bulk_transfer(dev_handle, 0x82, buf_in, sizeof(buf_in), &actual, 5000);
                if (actual != 32) {
                    fprintf(stderr, "\nData receive error\n");
                    break;
                }

                if (memcmp(buf, buf_in, 32)) {
                    fprintf(stderr, "\nDevice received wrong data\n");
                    break;
                }

                data_size -= 32;

                if ((saved_size - data_size) % 1024 == 0) {
                    printf("Send %d bytes of %d\r", saved_size - data_size, saved_size);
                }
            }

            if (size == 0) {
                printf("\n");
                err = 0;
            }

        }
    } else if (!strcmp(argv[1], "rom")) {
        int ret;
        int page = atoi(argv[2]);

        ret = rom_write(dev_handle, page, argv[3], 0, -1);

    } else if (!strcmp(argv[1], "romr")) {
        int ret;
        int page = atoi(argv[2]);
        long offset = strtol(argv[4],NULL,0);
        printf("rom resume: offset=%lx\n", offset);
        ret = rom_write(dev_handle, page, argv[3], offset, -1);
    } else {
        fprintf(stderr, "Unknown command\n\n");
        usage(argv[0]);
    }

 exit:

    libusb_release_interface(dev_handle, 0);

    return err;
}
