/**
 * Helper functions useful for testing purposes
 */
#ifndef TEST_HELPERS_H
#define	TEST_HELPERS_H
/*
 * Prints out an array in hex
 * @param mem
 * @param length
 */
void print_hex_memory(void *mem, uint16_t length) {
  int i;
  unsigned char *p = (unsigned char *)mem;
  for (i=0;i<length;i++) {
    printf("0x%02x ", p[i]);
  }
  printf("\n");
}

#endif

