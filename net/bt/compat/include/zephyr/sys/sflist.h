/* SPDX-License-Identifier: Apache-2.0 */

#ifndef ZEPHYR_INCLUDE_SYS_SFLIST_H_
#define ZEPHYR_INCLUDE_SYS_SFLIST_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <nuttx/list.h>

#include <zephyr/sys/__assert.h>
#include <zephyr/toolchain.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct list_node sys_sfnode_t;
typedef struct list_node sys_sflist_t;

#define SYS_SFLIST_STATIC_INIT(ptr_to_list) LIST_INITIAL_VALUE(*(ptr_to_list))

#define SYS_SFLIST_FOR_EACH_NODE(__sl, __sn) \
	for ((__sn) = sys_sflist_peek_head(__sl); (__sn) != NULL; \
	     (__sn) = sys_sflist_peek_next(__sn))

#define SYS_SFLIST_ITERATE_FROM_NODE(__sl, __sn) \
	for ((__sn) = (__sn) ? sys_sflist_peek_next_no_check(__sn) : \
				 sys_sflist_peek_head(__sl); \
	     (__sn) != NULL; (__sn) = sys_sflist_peek_next(__sn))

#define SYS_SFLIST_FOR_EACH_NODE_SAFE(__sl, __sn, __sns) \
	for ((__sn) = sys_sflist_peek_head(__sl), \
	     (__sns) = sys_sflist_peek_next(__sn); \
	     (__sn) != NULL; \
	     (__sn) = (__sns), (__sns) = sys_sflist_peek_next(__sn))

#define SYS_SFLIST_CONTAINER(__ln, __cn, __n) \
	((__cn) = ((__ln) != NULL) ? CONTAINER_OF(__ln, __typeof__(*(__cn)), __n) : NULL)

#define SYS_SFLIST_PEEK_HEAD_CONTAINER(__sl, __cn, __n) \
	SYS_SFLIST_CONTAINER(sys_sflist_peek_head(__sl), __cn, __n)

#define SYS_SFLIST_PEEK_TAIL_CONTAINER(__sl, __cn, __n) \
	SYS_SFLIST_CONTAINER(sys_sflist_peek_tail(__sl), __cn, __n)

#define SYS_SFLIST_PEEK_NEXT_CONTAINER(__cn, __n) \
	SYS_SFLIST_CONTAINER(sys_sflist_peek_next(&((__cn)->__n)), __cn, __n)

#define SYS_SFLIST_FOR_EACH_CONTAINER(__sl, __cn, __n) \
	for ((__cn) = NULL, SYS_SFLIST_PEEK_HEAD_CONTAINER(__sl, __cn, __n); \
	     (__cn) != NULL; \
	     SYS_SFLIST_PEEK_NEXT_CONTAINER(__cn, __n))

#define SYS_SFLIST_FOR_EACH_CONTAINER_SAFE(__sl, __cn, __cns, __n) \
	for ((__cn) = NULL, SYS_SFLIST_PEEK_HEAD_CONTAINER(__sl, __cn, __n), \
	     (__cns) = NULL, SYS_SFLIST_PEEK_NEXT_CONTAINER(__cn, __n); \
	     (__cn) != NULL; (__cn) = (__cns), \
	     (__cns) = NULL, SYS_SFLIST_PEEK_NEXT_CONTAINER(__cn, __n))

static inline void sys_sflist_init(sys_sflist_t *list)
{
	list_initialize(list);
}

/* NuttX list_node keeps link pointers directly.  The Bluetooth compatibility
 * layer only uses sys_sflist_t for k_fifo's reserved link field, so flagged
 * bits are intentionally constrained to zero while preserving the public API.
 */

#define SYS_SFLIST_FLAGS_MASK 0
#define SYS_SFLIST_FLAG_BITS  0

static inline sys_sfnode_t *z_sfnode_next_peek(const sys_sfnode_t *node)
{
	return node->next;
}

static inline uint8_t sys_sfnode_flags_get(const sys_sfnode_t *node)
{
	ARG_UNUSED(node);
	return 0;
}

static inline void z_sfnode_next_set(sys_sfnode_t *parent,
				       sys_sfnode_t *child)
{
	parent->next = child;
	if (child != NULL) {
		child->prev = parent;
	}
}

static inline void z_sflist_head_set(sys_sflist_t *list, sys_sfnode_t *node)
{
	list->next = node ? node : list;
}

static inline void z_sflist_tail_set(sys_sflist_t *list, sys_sfnode_t *node)
{
	list->prev = node ? node : list;
}

static inline sys_sfnode_t *sys_sflist_peek_head(const sys_sflist_t *list)
{
	return list_peek_head(list);
}

static inline sys_sfnode_t *sys_sflist_peek_tail(const sys_sflist_t *list)
{
	return list_peek_tail(list);
}

static inline void sys_sfnode_init(sys_sfnode_t *node, uint8_t flags)
{
	__ASSERT(flags == 0, "flags unsupported");
	list_clear_node(node);
}

static inline void sys_sfnode_flags_set(sys_sfnode_t *node, uint8_t flags)
{
	ARG_UNUSED(node);
	__ASSERT(flags == 0, "flags unsupported");
}

static inline bool sys_sflist_is_empty(const sys_sflist_t *list)
{
	return list_is_empty(list);
}

static inline sys_sfnode_t *sys_sflist_peek_next_no_check(const sys_sfnode_t *node)
{
	return list_next(node->prev, node);
}

static inline sys_sfnode_t *sys_sflist_peek_next(const sys_sfnode_t *node)
{
	return (node != NULL) ? sys_sflist_peek_next_no_check(node) : NULL;
}

static inline void sys_sflist_prepend(sys_sflist_t *list,
				      sys_sfnode_t *node)
{
	list_add_head(list, node);
}

static inline void sys_sflist_append(sys_sflist_t *list,
				     sys_sfnode_t *node)
{
	list_add_tail(list, node);
}

static inline void sys_sflist_append_list(sys_sflist_t *list,
					  void *head, void *tail)
{
	sys_sfnode_t *node = head;
	sys_sfnode_t *end = tail;

	while (node != NULL) {
		sys_sfnode_t *next = (node == end) ? NULL : node->next;

		list_add_tail(list, node);
		if (node == end) {
			break;
		}

		node = next;
	}
}

static inline void sys_sflist_merge_sflist(sys_sflist_t *list,
					   sys_sflist_t *list_to_append)
{
	while (!sys_sflist_is_empty(list_to_append)) {
		sys_sflist_append(list, list_remove_head(list_to_append));
	}
}

static inline void sys_sflist_insert(sys_sflist_t *list,
				     sys_sfnode_t *prev,
				     sys_sfnode_t *node)
{
	if (prev == NULL) {
		sys_sflist_prepend(list, node);
	} else {
		list_add_after(prev, node);
	}
}

static inline sys_sfnode_t *sys_sflist_get_not_empty(sys_sflist_t *list)
{
	return list_remove_head(list);
}

static inline sys_sfnode_t *sys_sflist_get(sys_sflist_t *list)
{
	return sys_sflist_is_empty(list) ? NULL : sys_sflist_get_not_empty(list);
}

static inline void sys_sflist_remove(sys_sflist_t *list,
				     sys_sfnode_t *prev_node,
				     sys_sfnode_t *node)
{
	ARG_UNUSED(list);
	ARG_UNUSED(prev_node);
	list_delete(node);
}

static inline bool sys_sflist_find(const sys_sflist_t *list,
				  const sys_sfnode_t *node,
				  sys_sfnode_t **prev)
{
	sys_sfnode_t *cur;
	sys_sfnode_t *last = NULL;

	SYS_SFLIST_FOR_EACH_NODE((sys_sflist_t *)list, cur) {
		if (cur == node) {
			if (prev != NULL) {
				*prev = last;
			}

			return true;
		}

		last = cur;
	}

	if (prev != NULL) {
		*prev = last;
	}

	return false;
}

static inline bool sys_sflist_find_and_remove(sys_sflist_t *list,
					      sys_sfnode_t *node)
{
	if (!sys_sflist_find(list, node, NULL)) {
		return false;
	}

	list_delete(node);
	return true;
}

static inline size_t sys_sflist_len(const sys_sflist_t *list)
{
	return list_length((sys_sflist_t *)list);
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_SYS_SFLIST_H_ */
