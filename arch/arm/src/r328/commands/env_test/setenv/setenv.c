#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <nuttx/config.h>
#include <env/env.h>

static void usage(void)
{
  printf("Usage: setenv <name> <value>\n"
         "Examples:\n"
         "  setenv foo bar        /*set variable foo equal bar*/\n");
}

int main(int argc, char *argv[])
{
  if (argc != 3) {
    usage();
    return -1;
  }

  if (fw_env_open()) {
    printf("open env failed\n");
    return -1;
  }

  fw_setenv(argv[1], argv[2]);

  fw_env_close();
  return 0;
}
