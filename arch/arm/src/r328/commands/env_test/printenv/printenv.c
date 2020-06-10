#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <nuttx/config.h>
#include <env/env.h>

int main(int argc, char *argv[])
{
  if (fw_env_open()) {
    printf("open env failed\n");
    return -1;
  }

  if (argc == 1) {
    fw_printenv(NULL);
  }
  else if (argc == 2){
    fw_printenv(argv[1]);
  }
  else {
    printf("error:too many parameters\n");
  }

  fw_env_close();
  return 0;
}
