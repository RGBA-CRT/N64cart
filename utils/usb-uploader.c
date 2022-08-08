#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libusb-1.0/libusb.h>
#include "utils.h"

int main(int argc, char **argv)
{
    libusb_context *ctx = NULL;
    uint r;
    libusb_device_handle *dev_handle;
    uint8_t data[512];
    int actual;
    int err = 1;

    printf("N64cart USB utility\n\n");

    memset(data, 0xFF, sizeof(data));

    r = libusb_init(&ctx);

    if (r < 0) {
	fprintf(stderr, "Error init libusb\n");
	return 1;
    }

    libusb_set_debug(ctx, 3);

    dev_handle = libusb_open_device_with_vid_pid(ctx, 0x0000, 0x0001);

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

	libusb_bulk_transfer(dev_handle, 0x01, (void *)&header, sizeof(struct data_header), &actual, 5000);
	if (actual != sizeof(struct data_header)) {
	    fprintf(stderr, "Header error transfer\n");
	    goto exit;
	}

	libusb_bulk_transfer(dev_handle, 0x82, (void *)&header, sizeof(struct data_header), &actual, 5000);
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
    } else if (!strcmp(argv[1], "write")) {
	int page = atoi(argv[2]);

	FILE *inf = fopen(argv[3], "rb");
	if (!inf) {
	    fprintf(stderr, "Cannot open file %s\n", argv[1]);
	    goto exit;
	}

	fseek(inf, 0, SEEK_END);
	int size = ftell(inf);
	fseek(inf, 0, SEEK_SET);

	fprintf(stderr, "ROM size %d\n", size);
	fprintf(stderr, "Write to page %d\n", page);

	struct data_header *header = alloca(sizeof(struct data_header));
	struct data_header *header_reply = alloca(sizeof(struct data_header));

	header->type = DATA_WRITE;
	header->address = 0;
	header->length = size;
	header->pages = page;

	libusb_bulk_transfer(dev_handle, 0x01, (void *)header, sizeof(struct data_header), &actual, 5000);
	if (actual != sizeof(struct data_header)) {
	    fprintf(stderr, "Header error transfer\n");
	    goto exit;
	}

	libusb_bulk_transfer(dev_handle, 0x82, (void *)header_reply, sizeof(struct data_header), &actual, 5000);
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

	    while (size) {
		int r = fread(buf, 1, 64, inf);
		if (r == 64) {
		    libusb_bulk_transfer(dev_handle, 0x01, buf, 64, &actual, 5000);
		    if (actual != 64) {
			fprintf(stderr, "\nData transfer error\n");
			break;
		    }

		    libusb_bulk_transfer(dev_handle, 0x82, buf_in, sizeof(buf_in), &actual, 5000);
		    if (actual != 64) {
			fprintf(stderr, "\nData receive error\n");
			break;
		    }

		    if (memcmp(buf, buf_in, 64)) {
			fprintf(stderr, "\nDevice received wrong data\n");
			break;
		    }

		    size -= 64;

		    if ((header->length - size) % 1024 == 0) {
			printf("Send %d bytes of %d\r", header->length - size, header->length);
		    }
		} else {
		    fprintf(stderr, "\nError - unaligned ROM file (align 64)\n");
		    break;
		}
	    }

	    if (size == 0) {
		printf("\n");
		err = 0;
	    }

	}

    } else {
	fprintf(stderr, "Unknown command\n");
    }

 exit:

    libusb_release_interface(dev_handle, 0);

    return err;
}
