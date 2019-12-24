#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <stdbool.h>
#include <string.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/random.h>
#include <nuttx/fs/fs.h>
#include <nuttx/config.h>
#include "sunxi_input.h"

#define input_err(fmt, args...)  printf("%s()%d - "fmt, __func__, __LINE__, ##args)

#ifdef DEBUG
#define input_info(fmt, args...)  printf("%s()%d - "fmt, __func__, __LINE__, ##args)
#else
#define input_info(fmt, args...)
#endif

static INPUT_LIST_HEAD(input_dev_list);
static INPUT_LIST_HEAD(evdev_list);

//static unsigned evdev_cnt = 0;

static inline int is_event_support(unsigned int code,
					unsigned long *bm, unsigned int max)
{
	return code <= max && input_test_bit(code, bm);
}

static struct sunxi_evdev *find_evdev_by_name(const char *name)
{
	struct sunxi_evdev *evdev = NULL;

	input_list_for_each_entry(evdev, &evdev_list, node, struct sunxi_evdev) {
		if(!strcmp(evdev->name, name))
			return evdev;
	}

	return NULL;
}

#if 0
static struct sunxi_evdev *find_evdev_by_fd(int fd)
{
	struct sunxi_evdev *evdev = NULL;

	input_list_for_each_entry(evdev, &evdev_list, node, struct sunxi_evdev) {
		if(evdev->fd == fd)
			return evdev;
	}

	return NULL;
}
#endif


static void evdev_pass_event(struct sunxi_evdev *evdev, struct sunxi_input_event *event)
{
	int ret;

	evdev->buffer[evdev->head++] = *event;
	evdev->head &= EVENT_BUFFER_SIZE - 1;

	if(evdev->head == evdev->tail) {
		evdev->tail = (evdev->head - 1) & (EVENT_BUFFER_SIZE - 1);
		evdev->packet_head = evdev->tail;
	}

	if (event->type == EV_SYN && event->code == SYN_REPORT) {
		evdev->packet_head = evdev->head;
		ret = sem_post(&evdev->sem);
		if (ret < 0) {
			input_err(" evdev give semaphore err\n");
		}
	}
}

static void input_pass_event(struct sunxi_input_dev *dev, struct sunxi_input_event *event)
{
	struct sunxi_evdev *evdev = NULL;

	//report input event to all evdev (all task that read).
	input_list_for_each_entry(evdev, &evdev_list, node, struct sunxi_evdev) {
		if(!strcmp(evdev->name, dev->name))
		{
			evdev_pass_event(evdev, event);
		}
	}

}

static void input_handle_event(struct sunxi_input_dev *dev,
				unsigned int type, unsigned int code, unsigned int value)
{
	bool report = false;
	struct sunxi_input_event event;

	switch (type) {
		case EV_SYN :
			report = true;
			break;
		case EV_KEY :
			if(is_event_support(code, dev->keybit, KEY_MAX))
				report = true;
			break;
		case EV_ABS :
			if(is_event_support(code, dev->absbit, ABS_MAX))
				report = true;
			break;
		case EV_REL :
			if(is_event_support(code, dev->relbit, REL_MAX))
				report = true;
			break;
		case EV_MSC :
			if(is_event_support(code, dev->mscbit, MSC_MAX))
				report = true;
			break;
		default :
			break;
	}

	if (report) {
		event.type = type;
		event.code = code;
		event.value = value;
		input_pass_event(dev, &event);
	}

}

void sunxi_input_event(struct sunxi_input_dev *dev,
			unsigned int type, unsigned int code, unsigned int value)
{
	if(is_event_support(type, dev->evbit, EV_MAX)) {
		//spin_lock();
		input_handle_event(dev, type, code, value);
		//spin_unlock();
	}

}

static int sunxi_fetch_next_event(struct sunxi_evdev *evdev, struct sunxi_input_event *event)
{
	int have_event;
	//uint32_t level;

	//spin_lock_irq(level);

	have_event = evdev->packet_head != evdev->tail;
	if (have_event) {
		*event = evdev->buffer[evdev->tail++];
		evdev->tail &= EVENT_BUFFER_SIZE - 1;
	}

	//spin_unlock_irq(level);

	return have_event;
}


/*------------------------------DRIVER API-------------------------*/
void input_set_capability(struct sunxi_input_dev *dev, unsigned int type, unsigned int code)
{
	switch (type) {
		case EV_KEY:
			input_set_bit(code, dev->keybit);
			break;

		case EV_REL:
			input_set_bit(code, dev->relbit);
			break;

		case EV_ABS:
			input_set_bit(code, dev->absbit);
			break;

		case EV_MSC:
			input_set_bit(code, dev->mscbit);
			break;

		default:
			input_err("input_set_capability: unknown type %u (code %u)\n",
					type, code);
			return;
	}

	input_set_bit(type, dev->evbit);
}

static struct sunxi_input_dev *find_input_dev_by_name(const char *name)
{
	struct sunxi_input_dev *dev;

	input_list_for_each_entry(dev, &input_dev_list, node, struct sunxi_input_dev) {
		if(!strcmp(dev->name, name))
			return dev;
	}

	return NULL;
}


struct sunxi_input_dev *sunxi_input_allocate_device()
{
	struct sunxi_input_dev *dev;

	dev = malloc(sizeof(struct sunxi_input_dev));
	if (dev)
		memset(dev, 0 , sizeof(struct sunxi_input_dev));

	return dev;
}

/*---------------------------User File Operations-------------------------*/
static int sunxi_input_open(FAR struct file *filep)
{
	int ret;
	FAR struct inode *inode;
	struct sunxi_input_dev *dev = NULL;
	struct sunxi_evdev *evdev;

	/* Get our private data structure */
	DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
	inode = filep->f_inode;

	dev = (FAR struct sunxi_input_dev *)inode->i_private;
	DEBUGASSERT(dev);

	input_info("open dev : %s\n", dev->name);
	//Note: nuttx return the same fd, when open file two times.
	evdev = find_evdev_by_name(dev->name);
	if(evdev) {
		input_err("%s has open by some task, can not open again\n", dev->name);
		return -1;
	}

#if 0
	dev = find_input_dev_by_name(name);
	if (NULL == dev) {
		input_err("input dev %s is not exist\n", name);
		return -1;
	}
#endif

	evdev = malloc(sizeof(struct sunxi_evdev));
	if (evdev) {
		memset(evdev, 0 ,sizeof(struct sunxi_evdev));
		evdev->name = dev->name;
		ret = sem_init(&evdev->sem, 0, 0);
		if (ret < 0) {
			input_err("creating semaphore_tx failed.\n");
			return -1;
		}
		input_list_add_tail(&evdev->node, &evdev_list);
		//return evdev->fd;
	}

	return OK;
}

static ssize_t sunxi_input_read(FAR struct file *filep, FAR char *buffer, size_t buflen)
{
	int ret;
	FAR struct inode *inode;
	struct sunxi_input_dev *dev = NULL;
	struct sunxi_evdev *evdev = NULL;
	struct sunxi_input_event event;
	unsigned int count = 0;

	/* Get our private data structure */
	DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
	inode = filep->f_inode;

	dev = (FAR struct sunxi_input_dev *)inode->i_private;
	DEBUGASSERT(dev);
	input_info("read dev : %s\n", dev->name);

	evdev = find_evdev_by_name(dev->name);
	if(NULL == evdev) {
		input_err("input read fd err\n");
		return -1;
	}

	if (evdev->packet_head == evdev->tail) {
		ret = sem_wait(&evdev->sem);
		if (ret < 0) {
			input_err("input take semaphore err\n");
			return -1;
		}
	}

	while(count + sizeof(struct sunxi_input_event) <= buflen
			&& sunxi_fetch_next_event(evdev, &event)) {
		memcpy(buffer + count, &event, sizeof(struct sunxi_input_event));
		count += sizeof(struct sunxi_input_event);
	}

	return count;
}

static int sunxi_input_close(FAR struct file *filep)
{
	FAR struct inode *inode;
	struct sunxi_input_dev *dev = NULL;
	struct sunxi_evdev *evdev;

	/* Get our private data structure */
	DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
	inode = filep->f_inode;

	dev = (FAR struct sunxi_input_dev *)inode->i_private;
	DEBUGASSERT(dev);

	evdev = find_evdev_by_name(dev->name);
	if(!evdev) {
		input_err("%s is not open\n", dev->name);
		return -1;
	}

	list_del(&evdev->node);
	free(evdev);

	return 0;
}
static const struct file_operations input_fops =
{
	sunxi_input_open,  /* open */
	sunxi_input_close, /* close */
	sunxi_input_read,  /* read */
	//not support,     /* write */
};

int sunxi_input_register_device(struct sunxi_input_dev *dev)
{
	int ret;
	char input_name[256];

	if (NULL == dev->name) {
		input_err("ERROR : input device must have a name\n");
		return -1;
	}

	sprintf(input_name, "/dev/input/%s", dev->name);
	ret = register_driver(input_name, &input_fops, 0666, dev);
	if (ret < 0) {
		input_err("ERROR: register_driver failed: %d\n", ret);
		return -1;
	}

	/* Every input device generates EV_SYN/SYN_REPORT events. */
	input_set_bit(EV_SYN, dev->evbit);

	input_list_add_tail(&dev->node, &input_dev_list);

	return 0;
}

