/* SPDX-License-Identifier: Apache-2.0 */

#ifndef ZEPHYR_INCLUDE_SYS_DLIST_H_
#define ZEPHYR_INCLUDE_SYS_DLIST_H_

#include <stdbool.h>
#include <stddef.h>

#include <nuttx/list.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct list_node sys_dlist_t;
typedef struct list_node sys_dnode_t;

#define SYS_DLIST_STATIC_INIT(ptr_to_list) LIST_INITIAL_VALUE(*(ptr_to_list))

#define SYS_DLIST_FOR_EACH_NODE(__dl, __dn) \
	for ((__dn) = sys_dlist_peek_head(__dl); (__dn) != NULL; \
	     (__dn) = sys_dlist_peek_next((__dl), (__dn)))

#define SYS_DLIST_ITERATE_FROM_NODE(__dl, __dn) \
	for ((__dn) = (__dn) ? sys_dlist_peek_next_no_check((__dl), (__dn)) : \
				 sys_dlist_peek_head(__dl); \
	     (__dn) != NULL; \
	     (__dn) = sys_dlist_peek_next((__dl), (__dn)))

#define SYS_DLIST_FOR_EACH_NODE_SAFE(__dl, __dn, __dns) \
	for ((__dn) = sys_dlist_peek_head(__dl), \
	     (__dns) = sys_dlist_peek_next((__dl), (__dn)); \
	     (__dn) != NULL; \
	     (__dn) = (__dns), (__dns) = sys_dlist_peek_next((__dl), (__dn)))

#define SYS_DLIST_CONTAINER(__dn, __cn, __n) \
	(((__dn) != NULL) ? CONTAINER_OF(__dn, __typeof__(*(__cn)), __n) : NULL)

#define SYS_DLIST_PEEK_HEAD_CONTAINER(__dl, __cn, __n) \
	SYS_DLIST_CONTAINER(sys_dlist_peek_head(__dl), __cn, __n)

#define SYS_DLIST_PEEK_NEXT_CONTAINER(__dl, __cn, __n) \
	(((__cn) != NULL) ? \
	 SYS_DLIST_CONTAINER(sys_dlist_peek_next((__dl), &((__cn)->__n)), \
			      __cn, __n) : NULL)

#define SYS_DLIST_FOR_EACH_CONTAINER(__dl, __cn, __n) \
	for ((__cn) = SYS_DLIST_PEEK_HEAD_CONTAINER(__dl, __cn, __n); \
	     (__cn) != NULL; \
	     (__cn) = SYS_DLIST_PEEK_NEXT_CONTAINER(__dl, __cn, __n))

#define SYS_DLIST_FOR_EACH_CONTAINER_SAFE(__dl, __cn, __cns, __n) \
	for ((__cn) = SYS_DLIST_PEEK_HEAD_CONTAINER(__dl, __cn, __n), \
	     (__cns) = SYS_DLIST_PEEK_NEXT_CONTAINER(__dl, __cn, __n); \
	     (__cn) != NULL; \
	     (__cn) = (__cns), \
	     (__cns) = SYS_DLIST_PEEK_NEXT_CONTAINER(__dl, __cn, __n))

static inline void sys_dlist_init(sys_dlist_t *list)
{
	list_initialize(list);
}

static inline void sys_dnode_init(sys_dnode_t *node)
{
	list_clear_node(node);
}

static inline bool sys_dnode_is_linked(const sys_dnode_t *node)
{
	return list_in_list(node);
}

static inline bool sys_dlist_is_head(const sys_dlist_t *list,
					     const sys_dnode_t *node)
{
	return list_is_head(list, node);
}

static inline bool sys_dlist_is_tail(const sys_dlist_t *list,
					     const sys_dnode_t *node)
{
	return list_is_tail(list, node);
}

static inline bool sys_dlist_is_empty(const sys_dlist_t *list)
{
	return list_is_empty(list);
}

static inline bool sys_dlist_has_multiple_nodes(const sys_dlist_t *list)
{
	return !list_is_empty(list) && !list_is_singular(list);
}

static inline sys_dnode_t *sys_dlist_peek_head(const sys_dlist_t *list)
{
	return list_peek_head(list);
}

static inline sys_dnode_t *sys_dlist_peek_head_not_empty(const sys_dlist_t *list)
{
	return list->next;
}

static inline sys_dnode_t *sys_dlist_peek_next_no_check(const sys_dlist_t *list,
							const sys_dnode_t *node)
{
	return list_next(list, node);
}

static inline sys_dnode_t *sys_dlist_peek_next(const sys_dlist_t *list,
					       const sys_dnode_t *node)
{
	return (node != NULL) ? sys_dlist_peek_next_no_check(list, node) : NULL;
}

static inline sys_dnode_t *sys_dlist_peek_prev_no_check(const sys_dlist_t *list,
							const sys_dnode_t *node)
{
	return list_prev(list, node);
}

static inline sys_dnode_t *sys_dlist_peek_prev(const sys_dlist_t *list,
					       const sys_dnode_t *node)
{
	return (node != NULL) ? sys_dlist_peek_prev_no_check(list, node) : NULL;
}

static inline sys_dnode_t *sys_dlist_peek_tail(const sys_dlist_t *list)
{
	return list_peek_tail(list);
}

static inline void sys_dlist_append(sys_dlist_t *list, sys_dnode_t *node)
{
	list_add_tail(list, node);
}

static inline void sys_dlist_prepend(sys_dlist_t *list, sys_dnode_t *node)
{
	list_add_head(list, node);
}

static inline void sys_dlist_insert(sys_dnode_t *successor, sys_dnode_t *node)
{
	list_add_before(successor, node);
}

static inline void sys_dlist_insert_at(sys_dlist_t *list, sys_dnode_t *node,
				       int (*cond)(sys_dnode_t *node, void *data),
				       void *data)
{
	sys_dnode_t *pos;

	SYS_DLIST_FOR_EACH_NODE(list, pos) {
		if (cond(pos, data) != 0) {
			sys_dlist_insert(pos, node);
			return;
		}
	}

	sys_dlist_append(list, node);
}

static inline void sys_dlist_dequeue(sys_dnode_t *node)
{
	list_delete_fast(node);
}

static inline void sys_dlist_remove(sys_dnode_t *node)
{
	list_delete(node);
}

static inline void sys_dlist_range_prepend(sys_dlist_t *dest,
					   sys_dnode_t *start,
					   sys_dnode_t *last)
{
	sys_dnode_t *head = dest->next;
	sys_dnode_t *prev = start->prev;
	sys_dnode_t *next = last->next;

	prev->next = next;
	next->prev = prev;
	last->next = head;
	start->prev = dest;
	head->prev = last;
	dest->next = start;
}

static inline void sys_dlist_range_append(sys_dlist_t *dest,
					  sys_dnode_t *start,
					  sys_dnode_t *last)
{
	sys_dnode_t *tail = dest->prev;
	sys_dnode_t *prev = start->prev;
	sys_dnode_t *next = last->next;

	prev->next = next;
	next->prev = prev;
	last->next = dest;
	start->prev = tail;
	tail->next = start;
	dest->prev = last;
}

static inline sys_dnode_t *sys_dlist_get(sys_dlist_t *list)
{
	return list_remove_head(list);
}

static inline size_t sys_dlist_len(const sys_dlist_t *list)
{
	return list_length((sys_dlist_t *)list);
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_SYS_DLIST_H_ */
