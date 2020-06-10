#include "msgqueue.h"

#ifdef CONFIG_OS_NUTTX
#include <fcntl.h>
#include <mqueue.h>
#define QUE_PRI 1

int nuttx_xr_drv_msgqueue_init(xr_drv_msgqueue_t *queue_hd,uint32_t queueLen)
{
	struct mq_attr attr;
	char mqname[16];

	/* Create a message queue for the worker thread */
	snprintf(queue_hd->mqname, sizeof(mqname), "/tmp/%X", (unsigned int)queue_hd);
	attr.mq_maxmsg  = queueLen;
	attr.mq_msgsize = sizeof(void*);
	attr.mq_curmsgs = 0;
	attr.mq_flags   = 0;

	queue_hd->queue = mq_open(queue_hd->mqname, O_RDWR | O_CREAT, 0644, &attr);
	if(queue_hd->queue == NULL){
		/* Error creating message queue! */
		return -1;
    }
	return 0;
}

int nuttx_xr_drv_msgqueue_send(xr_drv_msgqueue_t *queue_hd,void *msg)
{
	nxmq_send(queue_hd->queue,(const char *)&msg,sizeof(void*),QUE_PRI);
	return 0;
}

int nuttx_xr_drv_msgqueue_receive(xr_drv_msgqueue_t *queue_hd,void *msg,uint32_t timeout)
{
	//	nxmq_timedreceive  TODO:timeout
	ssize_t msgsize;
	unsigned int priority;
	msgsize = nxmq_receive(queue_hd->queue,&msg,&priority);
	if(msgsize < 0) {
		return -1;
	}
	return (int)msgsize;
}

int nuttx_xr_drv_msgqueue_destroy(xr_drv_msgqueue_t *queue_hd)
{
	mq_close(queue_hd->queue);
	mq_unlink(queue_hd->mqname);
	queue_hd->queue = NULL;
	return 0;
}
#endif
