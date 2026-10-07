/* SPDX-License-Identifier: Apache-2.0 */
#ifndef ZEPHYR_INCLUDE_SYS_LIST_GEN_H_
#define ZEPHYR_INCLUDE_SYS_LIST_GEN_H_

#include <stddef.h>
#include <stdbool.h>
#include <zephyr/toolchain.h>
#include <zephyr/sys/util.h>

/* Generic singly-linked list operations shared between slist and sflist.
 * Uses function-name token pasting to generate type-specific functions.
 */

#define _Z_GENLIST_ID(t, f) _CONCAT(_CONCAT(sys_, t), _CONCAT(_, f))
#define Z_GENLIST_ID(t, f)  _Z_GENLIST_ID(t, f)

/* Container */
#define Z_GENLIST_CONTAINER(__ln, __cn, __n) \
    ((__cn) = ((__ln) != NULL) ? CONTAINER_OF(__ln, __typeof__(*(__cn)), __n) : NULL)

/* Peek head container */
#define Z_GENLIST_PEEK_HEAD_CONTAINER(t, __sl, __cn, __n) \
    Z_GENLIST_CONTAINER(Z_GENLIST_ID(t, peek_head)(__sl), __cn, __n)

/* Peek tail container */
#define Z_GENLIST_PEEK_TAIL_CONTAINER(t, __sl, __cn, __n) \
    Z_GENLIST_CONTAINER(Z_GENLIST_ID(t, peek_tail)(__sl), __cn, __n)

/* Peek next container */
#define Z_GENLIST_PEEK_NEXT_CONTAINER(t, __cn, __n) \
    Z_GENLIST_CONTAINER(Z_GENLIST_ID(t, peek_next)(&((__cn)->__n)), __cn, __n)

/* Iteration */
#define Z_GENLIST_FOR_EACH_NODE(t, __sl, __sn) \
    for (__sn = Z_GENLIST_ID(t, peek_head)(__sl); \
         __sn != NULL; \
         __sn = Z_GENLIST_ID(t, peek_next)(__sn))

#define Z_GENLIST_ITERATE_FROM_NODE(t, __sl, __sn) \
    for (__sn = (__sn) ? Z_GENLIST_ID(t, peek_next_no_check)(__sn) : Z_GENLIST_ID(t, peek_head)(__sl); \
         __sn != NULL; \
         __sn = Z_GENLIST_ID(t, peek_next)(__sn))

#define Z_GENLIST_FOR_EACH_NODE_SAFE(t, __sl, __sn, __sns) \
    for (__sn = Z_GENLIST_ID(t, peek_head)(__sl), \
         __sns = Z_GENLIST_ID(t, peek_next)(__sn); \
         __sn != NULL; __sn = __sns, \
         __sns = Z_GENLIST_ID(t, peek_next)(__sn))

#define Z_GENLIST_FOR_EACH_CONTAINER(t, __sl, __cn, __n) \
    for (__cn = NULL, Z_GENLIST_PEEK_HEAD_CONTAINER(t, __sl, __cn, __n); \
         __cn != NULL; \
         Z_GENLIST_PEEK_NEXT_CONTAINER(t, __cn, __n))

#define Z_GENLIST_FOR_EACH_CONTAINER_SAFE(t, __sl, __cn, __cns, __n) \
    for (__cn = NULL, Z_GENLIST_PEEK_HEAD_CONTAINER(t, __sl, __cn, __n), \
         __cns = NULL, Z_GENLIST_PEEK_NEXT_CONTAINER(t, __cn, __n); \
         __cn != NULL; __cn = __cns, \
         __cns = NULL, Z_GENLIST_PEEK_NEXT_CONTAINER(t, __cn, __n))

/* Derived, inline function generators */

/* is_empty */
#define Z_GENLIST_IS_EMPTY(t) \
    static inline bool Z_GENLIST_ID(t, is_empty)(const sys_##t##_t *list) \
    { return list->head == NULL; }

/* peek_next_no_check */
#define Z_GENLIST_PEEK_NEXT_NO_CHECK(t, nodetype) \
    static inline sys_##nodetype##_t *Z_GENLIST_ID(t, peek_next_no_check)(const sys_##nodetype##_t *node) \
    { return z_##nodetype##_next_peek(node); }

/* peek_next */
#define Z_GENLIST_PEEK_NEXT(t, nodetype) \
    static inline sys_##nodetype##_t *Z_GENLIST_ID(t, peek_next)(const sys_##nodetype##_t *node) \
    { return (node != NULL) ? Z_GENLIST_ID(t, peek_next_no_check)(node) : NULL; }

/* prepend */
#define Z_GENLIST_PREPEND(t, nodetype) \
    static inline void Z_GENLIST_ID(t, prepend)(sys_##t##_t *list, sys_##nodetype##_t *node) \
    { \
        z_##nodetype##_next_set(node, list->head); \
        if (list->head == NULL) z_##t##_tail_set(list, node); \
        z_##t##_head_set(list, node); \
    }

/* append */
#define Z_GENLIST_APPEND(t, nodetype) \
    static inline void Z_GENLIST_ID(t, append)(sys_##t##_t *list, sys_##nodetype##_t *node) \
    { \
        z_##nodetype##_next_set(node, NULL); \
        if (list->tail != NULL) \
            z_##nodetype##_next_set(list->tail, node); \
        else \
            z_##t##_head_set(list, node); \
        z_##t##_tail_set(list, node); \
    }

/* append_list */
#define Z_GENLIST_APPEND_LIST(t, nodetype) \
    static inline void Z_GENLIST_ID(t, append_list)(sys_##t##_t *list, void *head, void *tail) \
    { \
        if (list->tail != NULL) \
            z_##nodetype##_next_set(list->tail, (sys_##nodetype##_t *)head); \
        else \
            z_##t##_head_set(list, (sys_##nodetype##_t *)head); \
        z_##t##_tail_set(list, (sys_##nodetype##_t *)tail); \
    }

/* merge_list */
#define Z_GENLIST_MERGE_LIST(t, nodetype) \
    static inline void Z_GENLIST_ID(t, merge_##t)(sys_##t##_t *list, sys_##t##_t *list_to_append) \
    { \
        if (!Z_GENLIST_ID(t, is_empty)(list_to_append)) { \
            if (list->tail != NULL) { \
                z_##nodetype##_next_set(list->tail, list_to_append->head); \
            } else { \
                z_##t##_head_set(list, list_to_append->head); \
            } \
            z_##t##_tail_set(list, list_to_append->tail); \
            Z_GENLIST_ID(t, init)(list_to_append); \
        } \
    }

/* insert */
#define Z_GENLIST_INSERT(t, nodetype) \
    static inline void Z_GENLIST_ID(t, insert)(sys_##t##_t *list, sys_##nodetype##_t *prev, sys_##nodetype##_t *node) \
    { \
        if (prev == NULL) { \
            Z_GENLIST_ID(t, prepend)(list, node); \
        } else { \
            z_##nodetype##_next_set(node, z_##nodetype##_next_peek(prev)); \
            z_##nodetype##_next_set(prev, node); \
            if (list->tail == prev) \
                z_##t##_tail_set(list, node); \
        } \
    }

/* get_not_empty */
#define Z_GENLIST_GET_NOT_EMPTY(t, nodetype) \
    static inline sys_##nodetype##_t *Z_GENLIST_ID(t, get_not_empty)(sys_##t##_t *list) \
    { \
        sys_##nodetype##_t *node = list->head; \
        list->head = z_##nodetype##_next_peek(node); \
        if (list->head == NULL) list->tail = NULL; \
        return node; \
    }

/* get */
#define Z_GENLIST_GET(t, nodetype) \
    static inline sys_##nodetype##_t *Z_GENLIST_ID(t, get)(sys_##t##_t *list) \
    { \
        return Z_GENLIST_ID(t, is_empty)(list) ? NULL : Z_GENLIST_ID(t, get_not_empty)(list); \
    }

/* remove */
#define Z_GENLIST_REMOVE(t, nodetype) \
    static inline void Z_GENLIST_ID(t, remove)(sys_##t##_t *list, sys_##nodetype##_t *prev_node, sys_##nodetype##_t *node) \
    { \
        if (prev_node == NULL) { \
            list->head = z_##nodetype##_next_peek(node); \
            if (list->head == NULL) list->tail = NULL; \
        } else { \
            z_##nodetype##_next_set(prev_node, z_##nodetype##_next_peek(node)); \
            if (list->tail == node) list->tail = prev_node; \
        } \
    }

/* find_and_remove */
#define Z_GENLIST_FIND_AND_REMOVE(t, nodetype) \
    static inline bool Z_GENLIST_ID(t, find_and_remove)(sys_##t##_t *list, sys_##nodetype##_t *node) \
    { \
        sys_##nodetype##_t *prev = NULL; \
        sys_##nodetype##_t *cur; \
        Z_GENLIST_FOR_EACH_NODE(t, list, cur) { \
            if (cur == node) { \
                Z_GENLIST_ID(t, remove)(list, prev, node); \
                return true; \
            } \
            prev = cur; \
        } \
        return false; \
    }

/* find */
#define Z_GENLIST_FIND(t, nodetype) \
    static inline bool Z_GENLIST_ID(t, find)(const sys_##t##_t *list, const sys_##nodetype##_t *node, sys_##nodetype##_t **prev) \
    { \
        sys_##nodetype##_t *cur; \
        *prev = NULL; \
        Z_GENLIST_FOR_EACH_NODE(t, (sys_##t##_t *)list, cur) { \
            if (cur == (sys_##nodetype##_t *)node) \
                return true; \
            *prev = cur; \
        } \
        return false; \
    }

/* len */
#define Z_GENLIST_LEN(t, nodetype) \
    static inline size_t Z_GENLIST_ID(t, len)(const sys_##t##_t *list) \
    { \
        size_t count = 0; \
        sys_##nodetype##_t *node; \
        Z_GENLIST_FOR_EACH_NODE(t, (sys_##t##_t *)list, node) { count++; } \
        return count; \
    }

#endif /* ZEPHYR_INCLUDE_SYS_LIST_GEN_H_ */