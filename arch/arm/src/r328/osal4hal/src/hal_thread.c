#include <stdio.h>
#include <stdlib.h>
#include <nuttx/kthread.h>
#include <hal_thread.h>
#include <hal_mem.h>

typedef struct kthread_arg {
	hal_kthread_fn_t entry;
	void *data;
	char *argv[2];
} kthread_arg_t;

static int kthread_entry_wrapper(int argc, FAR char *argv[])
{
	kthread_arg_t *arg;

	if (!argv)
		return -1;
	arg = (kthread_arg_t *)atoi(argv[1]);
	if (!arg)
		return -1;
	arg->entry(arg->data);
	hal_free(arg->argv[0]);
	hal_free(arg);
	return 0;
}

int hal_kthread_create(hal_kthread_t *thread, hal_kthread_fn_t entry,
			void *data, const char *name,
			int priority, int stack_size)
{
	pid_t pid;
	kthread_arg_t *arg;
	char *pointer_str;
	int len = 16;

	if (!thread || !entry)
		return -1;

	arg = hal_zalloc(sizeof(kthread_arg_t));
	if (!arg)
		return -1;
	pointer_str = hal_zalloc(len);
	if (!pointer_str) {
		hal_free(arg);
		return -1;
	}
	arg->entry = entry;
	arg->data = data;
	snprintf(pointer_str, len, "%lu", (_uintptr_t)arg);
	arg->argv[0] = pointer_str;
	arg->argv[1] = NULL;

	pid = kthread_create(name, priority, stack_size, kthread_entry_wrapper, arg->argv);

	*thread = pid;
	if (pid < 0) {
		hal_free(pointer_str);
		hal_free(arg);
		return pid;
	}
	return 0;
}

void hal_kthread_delete(hal_kthread_t *thread)
{
	pid_t *pid;
	pid  = (pid_t *)thread;
	kthread_delete(*pid);
}
