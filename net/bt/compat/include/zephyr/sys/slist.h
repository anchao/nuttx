/* SPDX-License-Identifier: Apache-2.0 */

#ifndef ZEPHYR_INCLUDE_SYS_SLIST_H_
#define ZEPHYR_INCLUDE_SYS_SLIST_H_

#include <stdbool.h>
#include <stddef.h>

#include <nuttx/list.h>

#include <zephyr/toolchain.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct list_node sys_snode_t;
typedef struct list_node sys_slist_t;

#define SYS_SLIST_STATIC_INIT(ptr_to_list) LIST_INITIAL_VALUE(*(ptr_to_list))

#define SYS_SLIST_FOR_EACH_NODE(__sl, __sn) \
	for ((__sn) = sys_slist_peek_head(__sl); (__sn) != NULL; \
	     (__sn) = sys_slist_peek_next(__sn))

#define SYS_SLIST_ITERATE_FROM_NODE(__sl, __sn) \
	for ((__sn) = (__sn) ? sys_slist_peek_next_no_check(__sn) : \
				 sys_slist_peek_head(__sl); \
	     (__sn) != NULL; (__sn) = sys_slist_peek_next(__sn))

#define SYS_SLIST_FOR_EACH_NODE_SAFE(__sl, __sn, __sns) \
	for ((__sn) = sys_slist_peek_head(__sl), \
	     (__sns) = sys_slist_peek_next(__sn); \
	     (__sn) != NULL; \
	     (__sn) = (__sns), (__sns) = sys_slist_peek_next(__sn))

#define SYS_SLIST_CONTAINER(__ln, __cn, __n) \
	((__cn) = ((__ln) != NULL) ? CONTAINER_OF(__ln, __typeof__(*(__cn)), __n) : NULL)

#define SYS_SLIST_PEEK_HEAD_CONTAINER(__sl, __cn, __n) \
	SYS_SLIST_CONTAINER(sys_slist_peek_head(__sl), __cn, __n)

#define SYS_SLIST_PEEK_TAIL_CONTAINER(__sl, __cn, __n) \
	SYS_SLIST_CONTAINER(sys_slist_peek_tail(__sl), __cn, __n)

#define SYS_SLIST_PEEK_NEXT_CONTAINER(__cn, __n) \
	SYS_SLIST_CONTAINER(sys_slist_peek_next(&((__cn)->__n)), __cn, __n)

#define SYS_SLIST_FOR_EACH_CONTAINER(__sl, __cn, __n) \
	for ((__cn) = NULL, SYS_SLIST_PEEK_HEAD_CONTAINER(__sl, __cn, __n); \
	     (__cn) != NULL; \
	     SYS_SLIST_PEEK_NEXT_CONTAINER(__cn, __n))

#define SYS_SLIST_FOR_EACH_CONTAINER_SAFE(__sl, __cn, __cns, __n) \
	for ((__cn) = NULL, SYS_SLIST_PEEK_HEAD_CONTAINER(__sl, __cn, __n), \
	     (__cns) = NULL, SYS_SLIST_PEEK_NEXT_CONTAINER(__cn, __n); \
	     (__cn) != NULL; (__cn) = (__cns), \
	     (__cns) = NULL, SYS_SLIST_PEEK_NEXT_CONTAINER(__cn, __n))

static inline void sys_slist_init(sys_slist_t *list)
{
	list_initialize(list);
}

static inline sys_snode_t *z_snode_next_peek(const sys_snode_t *node)
{
	return node->next;
}

static inline void z_snode_next_set(sys_snode_t *parent, sys_snode_t *child)
{
	parent->next = child;
	if (child != NULL) {
		child->prev = parent;
	}
}

static inline void z_slist_head_set(sys_slist_t *list, sys_snode_t *node)
{
	list->next = node ? node : list;
}

static inline void z_slist_tail_set(sys_slist_t *list, sys_snode_t *node)
{
	list->prev = node ? node : list;
}

static inline sys_snode_t *sys_slist_peek_head(const sys_slist_t *list)
{
	return list_peek_head(list);
}

static inline sys_snode_t *sys_slist_peek_tail(const sys_slist_t *list)
{
	return list_peek_tail(list);
}

static inline bool sys_slist_is_empty(const sys_slist_t *list)
{
	return list_is_empty(list);
}

static inline sys_snode_t *sys_slist_peek_next_no_check(const sys_snode_t *node)
{
	return list_next(node->prev, node);
}

static inline sys_snode_t *sys_slist_peek_next(const sys_snode_t *node)
{
	return (node != NULL) ? sys_slist_peek_next_no_check(node) : NULL;
}

static inline void sys_slist_prepend(sys_slist_t *list, sys_snode_t *node)
{
	list_add_head(list, node);
}

static inline void sys_slist_append(sys_slist_t *list, sys_snode_t *node)
{
	list_add_tail(list, node);
}

static inline void sys_slist_append_list(sys_slist_t *list, void *head, void *tail)
{
	sys_snode_t *node = head;
	sys_snode_t *end = tail;

	while (node != NULL) {
		sys_snode_t *next = (node == end) ? NULL : node->next;

		list_add_tail(list, node);
		if (node == end) {
			break;
		}

		node = next;
	}
}

static inline void sys_slist_merge_slist(sys_slist_t *list,
						 sys_slist_t *list_to_append)
{
	while (!sys_slist_is_empty(list_to_append)) {
		sys_slist_append(list, list_remove_head(list_to_append));
	}
}

static inline void sys_slist_insert(sys_slist_t *list, sys_snode_t *prev,
				    sys_snode_t *node)
{
	if (prev == NULL) {
		sys_slist_prepend(list, node);
	} else {
		list_add_after(prev, node);
	}
}

static inline sys_snode_t *sys_slist_get_not_empty(sys_slist_t *list)
{
	return list_remove_head(list);
}

static inline sys_snode_t *sys_slist_get(sys_slist_t *list)
{
	return sys_slist_is_empty(list) ? NULL : sys_slist_get_not_empty(list);
}

static inline void sys_slist_remove(sys_slist_t *list, sys_snode_t *prev_node,
				    sys_snode_t *node)
{
	ARG_UNUSED(list);
	ARG_UNUSED(prev_node);
	list_delete(node);
}

static inline bool sys_slist_find(const sys_slist_t *list, const sys_snode_t *node,
				  sys_snode_t **prev)
{
	sys_snode_t *cur;
	sys_snode_t *last = NULL;

	SYS_SLIST_FOR_EACH_NODE((sys_slist_t *)list, cur) {
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

static inline bool sys_slist_find_and_remove(sys_slist_t *list,
					     sys_snode_t *node)
{
	if (!sys_slist_find(list, node, NULL)) {
		return false;
	}

	list_delete(node);
	return true;
}

static inline size_t sys_slist_len(const sys_slist_t *list)
{
	return list_length((sys_slist_t *)list);
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_SYS_SLIST_H_ */
