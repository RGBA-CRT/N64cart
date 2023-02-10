#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdbool.h>
#include "romfs.h"

static uint8_t memory[64 * 1024 * 1024];

void save_romfs(char *name, uint8_t *mem, size_t len)
{
    FILE *out = fopen(name, "wb");
    if (out) {
	if (fwrite(mem, 1, len, out) != len) {
	    fprintf(stderr, "Error write file!\n");
	}
	fclose(out);
    }
}

bool load_romfs(char *name, uint8_t *mem, size_t len)
{
    bool ret = true;
    FILE *out = fopen(name, "rb");
    if (out) {
	if (fread(mem, 1, len, out) != len) {
	    ret = false;
	}
	fclose(out);
	return ret;
    }

    return false;
}

int main(int argc, char *argv[])
{
    if (argc <= 1) {
	fprintf(stderr, "No rom file defined!");
	return -1;
    }

    if (!load_romfs(argv[1], memory, sizeof(memory))) {
	fprintf(stderr, "Cannot read %s\n", argv[1]);
    }

    if (!romfs_start(memory, 65536, sizeof(memory))) {
	printf("Cannot start romfs!\n");
	goto err;
    }

    if (argc > 2) {
	if (!strcmp(argv[2], "format")) {
	    romfs_format();
	} else if (!strcmp(argv[2], "list")) {
	    romfs_file file;
	    if (romfs_list(&file, true)) {
		bool next_file;
		do {
		    printf("%s\t%d\t%0X %4X\n", file.entry.name, file.entry.size, file.entry.mode, file.entry.type);
		    next_file = romfs_list(&file, false);
		} while (next_file);
	    }
	} else if (!strcmp(argv[2], "delete")) {
	    if (!romfs_delete(argv[3])) {
		fprintf(stderr, "Error: [%s] file not found!\n", argv[3]);
	    }
	} else {
	    fprintf(stderr, "Error: Unknown command '%s'\n", argv[2]);
	}
    }

    save_romfs(argv[1], memory, sizeof(memory));

 err:

    return 0;
}
