#ifndef __ENV_H__
#define __ENV_H__

#ifdef __cplusplus
extern "C" {
#endif

#define CUR_ENVSIZE           (CONFIG_COMPONENTS_ENV_SIZE)

enum flag_scheme {
	FLAG_NONE,
	FLAG_BOOLEAN,
	FLAG_INCREMENTAL,
};

struct environment {
	void *image;
	uint32_t *crc;
	unsigned char *flags;
	char *data;
	enum flag_scheme flag_scheme;
};

int fw_env_open(void);
int fw_env_close(void);
int fw_env_flush(void);
int fw_env_write(char *name, char *value);
char *fw_getenv(char *name);
int fw_printenv(char *arg);
int fw_setenv(char *arg, char *value);

#ifdef __cplusplus
}
#endif

#endif
