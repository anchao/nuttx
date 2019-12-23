#include <nuttx/config.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <semaphore.h>
#include <nuttx/arch.h>

/* msg queue 
 * nxmq_send
 * nxmq_timedsend
 * nxmq_receive
 * nxmq_timedreceive
 * nxmq_free_msgq
 * nxmq_alloc_msgq
 * nxmq_create_des
 * nxmq_close_group
 * nxmq_desclose_group
 *
 * mq_open
 * mq_close
 * mq_unlink
 * mq_notify
 * */
static void *nx_msg_queue_receive_thread(void *arg)
{
	int size;
	int msg;
	unsigned int prio;
	struct timespec tp;
	mqd_t mq = (mqd_t)arg;

	while (1) {
		size = nxmq_receive(mq, (char *)&msg, sizeof(msg), &prio);
		printf("nxmq_receive return %d\n", size);
		if (size == 0)
			break;
		printf("msg value:%d\n", msg);
		if (msg == 3)
			break;
	}

	clock_gettime(CLOCK_REALTIME, &tp);
	tp.tv_sec += 1;
	printf("before nxmq_timedreceive\n");
	msg = 0;
	size = nxmq_timedreceive(mq, (char *)&msg, sizeof(msg),
				&prio, (const struct timespec *)&tp);
	printf("recv msg=%d, size=%d\n", msg, size);
	printf("after nxmq_timedreceive\n");
	pthread_exit(NULL);
}

static void nx_msg_queue_test(void)
{
	struct mq_attr attr;	
	int msg_test;
	pthread_t tid;
	mqd_t mq;

	attr.mq_maxmsg = 16;
	attr.mq_msgsize = sizeof(msg_test);
	attr.mq_curmsgs = 0;
	attr.mq_flags = 0;
	mq = mq_open("/tmp/1234", O_RDWR | O_CREAT, 0644, &attr);
	if (mq == NULL) {
		printf("allocate msg queue failed\n");
		return;
	}

	pthread_create(&tid, NULL, nx_msg_queue_receive_thread, (void *)mq);
	usleep(500000);

	msg_test = 1;
	nxmq_send(mq, (const char *)&msg_test, sizeof(msg_test), 100);
	msg_test = 2;
	nxmq_send(mq, (const char *)&msg_test, sizeof(msg_test), 100);
	msg_test = 3;
	nxmq_send(mq, (const char *)&msg_test, sizeof(msg_test), 100);
	pthread_join(tid, NULL);

	mq_close(mq);
	printf("msg queue test finish...\n");
	return;
}

static void *sem_thread(void *arg)
{
	sem_t *sem = (sem_t *)arg;

	sleep(1);
	printf("sem post...\n");
	sem_post(sem);
	sleep(2);
	printf("sem post...\n");
	sem_post(sem);
	sleep(3);
	printf("sem post...\n");
	sem_post(sem);

	sleep(4);
	printf("exit\n");
	pthread_exit(NULL);
}

static void sem_test(void)
{
	int ret;
	sem_t sem;
	pthread_t tid;
	int test_count = 3;

	ret = sem_init(&sem, 0, 1);
	if (ret != 0) {
		printf("sem init failed\n");
		return;
	}
	pthread_create(&tid, NULL, sem_thread, (void *)&sem);
	while (1) {
#if 1
		struct timespec tp;
		clock_gettime(CLOCK_REALTIME, &tp);
		tp.tv_sec += 2;
		tp.tv_nsec += 500000000;
		printf("set timeout : %lu, %lu\n", tp.tv_sec, tp.tv_nsec);
		ret = sem_timedwait(&sem, &tp);
		printf("sem_timedwait return %d\n", ret);
#else
		ret = sem_wait(&sem);
#endif
		test_count--;
		if (!test_count)
			break;
		if (ret == -ETIMEDOUT) {
			printf("timeout...\n");
			continue;
		}
	}
	pthread_join(tid, NULL);
	sem_destroy(&sem);
	return ;
}

int main(int argc, FAR char *argv[])
{
	int c;
	printf("cpu id=%d\n", up_cpu_index());
	while ((c = getopt(argc, argv, "ms")) != -1) {
		switch (c) {
		case 'm':
			nx_msg_queue_test();
			break;
		case 's':
			sem_test();
			break;
		default:
			break;
		}
	}
	return 0;
}
