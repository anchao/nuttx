/****************************************************************************
 * net/rpmsg/rpmsg_sockif.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <debug.h>
#include <poll.h>
#include <stdio.h>
#include <string.h>
#include <sys/param.h>
#include <sys/socket.h>

#include <nuttx/kmalloc.h>
#include <nuttx/circbuf.h>
#include <nuttx/crc32.h>
#include <nuttx/rpmsg/rpmsg.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/fs/ioctl.h>

#include <netinet/in.h>
#include <netpacket/rpmsg.h>

#include "socket/socket.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RPMSG_SOCKET_CMD_SYNC           1
#define RPMSG_SOCKET_CMD_DATA           2
#define RPMSG_SOCKET_CMD_SHUTDOWN       3
#define RPMSG_SOCKET_NAME_PREFIX        "s:"
#define RPMSG_SOCKET_NAME_PREFIX_LEN    2
#define RPMSG_SOCKET_NAME_LEN           10

/****************************************************************************
 * Private Types
 ****************************************************************************/

begin_packed_struct struct rpmsg_socket_sync_s
{
  uint32_t                       cmd;
  uint32_t                       size;
  uint32_t                       pid;
  uint32_t                       uid;
  uint32_t                       gid;
} end_packed_struct;

begin_packed_struct struct rpmsg_socket_data_s
{
  uint32_t                       cmd;
  uint32_t                       pos;

  /* Act data len, don't include len itself when SOCK_DGRAM */

  uint32_t                       len;
  uint8_t                        data[0];
} end_packed_struct;

begin_packed_struct struct rpmsg_socket_shutdown_s
{
  uint32_t                       cmd;
  uint32_t                       how;
} end_packed_struct;

struct rpmsg_socket_conn_s
{
  /* Common prologue of all connection structures. */

  struct socket_conn_s           sconn;

  bool                           unbind;
  struct rpmsg_endpoint          ept;

  struct sockaddr_rpmsg          rpaddr;
  char                           nameid[RPMSG_SOCKET_NAME_LEN];
  uint16_t                       crefs;
  uint32_t                       how;

  FAR struct pollfd              *fds[CONFIG_NET_RPMSG_NPOLLWAITERS];
  mutex_t                        polllock;

  sem_t                          sendsem;
  mutex_t                        sendlock;

  sem_t                          recvsem;
  mutex_t                        recvlock;

  FAR void                       *recvdata;
  uint32_t                       recvlen;
  FAR struct circbuf_s           recvbuf;

  FAR struct rpmsg_socket_conn_s *next;

  /* server listen-scoket listening: backlog > 0;
   * others: backlog = 0;
   */

  int                            backlog;

  /* The remote connection's credentials */

  struct ucred                   cred;

  /* Recvbuf size */

  uint32_t                       recvsize;

  /* Flow control, descript send side */

  uint32_t                       sendsize;
  uint32_t                       sendpos;
  uint32_t                       ackpos;

  /* Flow control, descript recv side */

  uint32_t                       recvpos;
  uint32_t                       lastpos;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int        rpmsg_socket_setup(FAR struct socket *psock);
static sockcaps_t rpmsg_socket_sockcaps(FAR struct socket *psock);
static void       rpmsg_socket_addref(FAR struct socket *psock);
static int        rpmsg_socket_bind(FAR struct socket *psock,
                                    FAR const struct sockaddr *addr,
                                    socklen_t addrlen);
static int        rpmsg_socket_getsockname(FAR struct socket *psock,
                                           FAR struct sockaddr *addr,
                                           FAR socklen_t *addrlen);
static int        rpmsg_socket_getpeername(FAR struct socket *psock,
                                           FAR struct sockaddr *addr,
                                           FAR socklen_t *addrlen);
static int        rpmsg_socket_listen(FAR struct socket *psock, int backlog);
static int        rpmsg_socket_connect(FAR struct socket *psock,
                                       FAR const struct sockaddr *addr,
                                       socklen_t addrlen);
static int        rpmsg_socket_accept(FAR struct socket *psock,
                                      FAR struct sockaddr *addr,
                                      FAR socklen_t *addrlen,
                                      FAR struct socket *newsock, int flags);
static int        rpmsg_socket_poll(FAR struct socket *psock,
                                    FAR struct pollfd *fds, bool setup);
static ssize_t    rpmsg_socket_sendmsg(FAR struct socket *psock,
                                       FAR struct msghdr *msg, int flags);
static ssize_t    rpmsg_socket_recvmsg(FAR struct socket *psock,
                                       FAR struct msghdr *msg, int flags);
static int        rpmsg_socket_close(FAR struct socket *psock);
static int        rpmsg_socket_ioctl(FAR struct socket *psock,
                                     int cmd, unsigned long arg);
static int        rpmsg_socket_shutdown(FAR struct socket *psock, int how);
#ifdef CONFIG_NET_SOCKOPTS
static int        rpmsg_socket_getsockopt(FAR struct socket *psock,
                                          int level, int option,
                                          FAR void *value,
                                          FAR socklen_t *value_len);
static int        rpmsg_socket_setsockopt(FAR struct socket *psock,
                                          int level, int option,
                                          FAR const void *value,
                                          socklen_t value_len);
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct sock_intf_s g_rpmsg_sockif =
{
  rpmsg_socket_setup,       /* si_setup */
  rpmsg_socket_sockcaps,    /* si_sockcaps */
  rpmsg_socket_addref,      /* si_addref */
  rpmsg_socket_bind,        /* si_bind */
  rpmsg_socket_getsockname, /* si_getsockname */
  rpmsg_socket_getpeername, /* si_getpeername */
  rpmsg_socket_listen,      /* si_listen */
  rpmsg_socket_connect,     /* si_connect */
  rpmsg_socket_accept,      /* si_accept */
  rpmsg_socket_poll,        /* si_poll */
  rpmsg_socket_sendmsg,     /* si_sendmsg */
  rpmsg_socket_recvmsg,     /* si_recvmsg */
  rpmsg_socket_close,       /* si_close */
  rpmsg_socket_ioctl,       /* si_ioctl */
  NULL,                     /* si_socketpair */
  rpmsg_socket_shutdown     /* si_shutdown */
#ifdef CONFIG_NET_SOCKOPTS
  , rpmsg_socket_getsockopt /* si_getsockopt */
  , rpmsg_socket_setsockopt /* si_setsockopt */
#endif
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint32_t g_rpmsg_id;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void rpmsg_socket_post(FAR sem_t *sem)
{
  int semcount;

  nxsem_get_value(sem, &semcount);
  if (semcount < 1)
    {
      nxsem_post(sem);
    }
}

static inline void
rpmsg_socket_poll_notify(FAR struct rpmsg_socket_conn_s *conn,
                         pollevent_t eventset)
{
  nxmutex_lock(&conn->polllock);
  poll_notify(conn->fds, CONFIG_NET_RPMSG_NPOLLWAITERS, eventset);
  nxmutex_unlock(&conn->polllock);
}

static FAR struct rpmsg_socket_conn_s *rpmsg_socket_alloc(void)
{
  FAR struct rpmsg_socket_conn_s *conn;
  int ret;

  conn = kmm_zalloc(sizeof(struct rpmsg_socket_conn_s));
  if (!conn)
    {
      return NULL;
    }

  ret = circbuf_init(&conn->recvbuf, NULL, 0);
  if (ret < 0)
    {
      kmm_free(conn);
      return NULL;
    }

  nxmutex_init(&conn->polllock);
  nxmutex_init(&conn->sendlock);
  nxmutex_init(&conn->recvlock);
  nxsem_init(&conn->sendsem, 0, 0);
  nxsem_init(&conn->recvsem, 0, 0);

  conn->recvsize = CONFIG_NET_RPMSG_RXBUF_SIZE;
  conn->crefs = 1;
  return conn;
}

static void rpmsg_socket_free(FAR struct rpmsg_socket_conn_s *conn)
{
  circbuf_uninit(&conn->recvbuf);

  nxmutex_destroy(&conn->polllock);
  nxmutex_destroy(&conn->recvlock);
  nxmutex_destroy(&conn->sendlock);
  nxsem_destroy(&conn->sendsem);
  nxsem_destroy(&conn->recvsem);

  kmm_free(conn);
}

static int rpmsg_socket_wakeup(FAR struct rpmsg_socket_conn_s *conn)
{
  struct rpmsg_socket_data_s msg;
  uint32_t space;
  int ret = 0;

  if (!conn->ept.rdev || conn->unbind)
    {
      return ret;
    }

  nxmutex_lock(&conn->recvlock);
  space = conn->recvpos - conn->lastpos;

  if (space > circbuf_size(&conn->recvbuf) / 2)
    {
      conn->lastpos = conn->recvpos;
      msg.cmd = RPMSG_SOCKET_CMD_DATA;
      msg.pos = conn->recvpos;
      msg.len = 0;
      ret = 1;
    }

  nxmutex_unlock(&conn->recvlock);
  return ret ? rpmsg_send(&conn->ept, &msg, sizeof(msg)) : 0;
}

static inline uint32_t
rpmsg_socket_get_space(FAR struct rpmsg_socket_conn_s *conn)
{
  return conn->sendsize - (conn->sendpos - conn->ackpos);
}

static int rpmsg_socket_ept_cb(FAR struct rpmsg_endpoint *ept,
                               FAR void *data, size_t len, uint32_t src,
                               FAR void *priv)
{
  FAR struct rpmsg_socket_conn_s *conn = ept->priv;
  FAR struct rpmsg_socket_sync_s *head = data;

  if (head->cmd == RPMSG_SOCKET_CMD_SYNC)
    {
      nxmutex_lock(&conn->sendlock);
      conn->sendsize = head->size;
      conn->cred.pid = head->pid;
      conn->cred.uid = head->uid;
      conn->cred.gid = head->gid;

      conn->sconn.s_flags |= _SF_CONNECTED;
      _SO_CONN_SETERRNO(conn, OK);

      rpmsg_socket_post(&conn->sendsem);
      rpmsg_socket_poll_notify(conn, POLLOUT);
      nxmutex_unlock(&conn->sendlock);
    }
  else if (head->cmd == RPMSG_SOCKET_CMD_DATA)
    {
      FAR struct rpmsg_socket_data_s *msg = data;
      FAR uint8_t *buf = msg->data;

      nxmutex_lock(&conn->sendlock);
      conn->ackpos = msg->pos;
      if (rpmsg_socket_get_space(conn) > 0)
        {
          rpmsg_socket_post(&conn->sendsem);
          rpmsg_socket_poll_notify(conn, POLLOUT);
        }

      nxmutex_unlock(&conn->sendlock);
      if (len > sizeof(*msg))
        {
          len -= sizeof(*msg);
          DEBUGASSERT(len == msg->len || len == msg->len + sizeof(uint32_t));

          nxmutex_lock(&conn->recvlock);
          if (conn->recvdata)
            {
              conn->recvlen = MIN(conn->recvlen, msg->len);
              if (len == msg->len)
                {
                  /* SOCK_STREAM */

                  conn->recvpos += conn->recvlen;
                  memcpy(conn->recvdata, buf, conn->recvlen);
                  buf += conn->recvlen;
                  len -= conn->recvlen;
                }
              else
                {
                  /* SOCK_DGRAM */

                  conn->recvpos += len;
                  memcpy(conn->recvdata, buf + sizeof(uint32_t),
                         conn->recvlen);
                  len = 0;
                }

              conn->recvdata = NULL;
              rpmsg_socket_post(&conn->recvsem);
            }

          if (len > 0)
            {
              ssize_t written;

              written = circbuf_write(&conn->recvbuf, buf, len);
              if (written != len)
                {
                  nerr("circbuf_write overflow, %zu, %zu\n", written, len);
                }

              rpmsg_socket_poll_notify(conn, POLLIN);
            }

          nxmutex_unlock(&conn->recvlock);
        }
    }
  else if (head->cmd == RPMSG_SOCKET_CMD_SHUTDOWN)
    {
      FAR struct rpmsg_socket_shutdown_s *msg = data;

      if (msg->how & SHUT_WR)
        {
          nxmutex_lock(&conn->recvlock);
          conn->how |= SHUT_RD;
          rpmsg_socket_post(&conn->recvsem);
          rpmsg_socket_poll_notify(conn, POLLIN | POLLHUP);
          nxmutex_unlock(&conn->recvlock);
        }

      if (msg->how & SHUT_RD)
        {
          nxmutex_lock(&conn->sendlock);
          conn->how |= SHUT_WR;
          rpmsg_socket_post(&conn->sendsem);
          rpmsg_socket_poll_notify(conn, POLLOUT | POLLHUP);
          nxmutex_unlock(&conn->sendlock);
        }
    }

  return 0;
}

static inline void rpmsg_socket_destroy_ept(
                    FAR struct rpmsg_socket_conn_s *conn)
{
  if (!conn)
    {
      return;
    }

  nxmutex_lock(&conn->recvlock);
  nxmutex_lock(&conn->sendlock);

  if (conn->ept.rdev)
    {
      rpmsg_socket_post(&conn->sendsem);
      rpmsg_socket_post(&conn->recvsem);
      rpmsg_socket_poll_notify(conn, POLLIN | POLLOUT);
    }

  nxmutex_unlock(&conn->sendlock);
  nxmutex_unlock(&conn->recvlock);
  rpmsg_destroy_ept(&conn->ept);
}

static void rpmsg_socket_ns_bound(struct rpmsg_endpoint *ept)
{
  FAR struct rpmsg_socket_conn_s *conn = ept->priv;
  struct rpmsg_socket_sync_s msg;

  msg.cmd  = RPMSG_SOCKET_CMD_SYNC;
  msg.size = circbuf_size(&conn->recvbuf);
  msg.pid  = nxsched_getpid();
  msg.uid  = getuid();
  msg.gid  = getgid();

  rpmsg_send(&conn->ept, &msg, sizeof(msg));
}

static void rpmsg_socket_ns_unbind(FAR struct rpmsg_endpoint *ept)
{
  FAR struct rpmsg_socket_conn_s *conn = ept->priv;

  nxmutex_lock(&conn->recvlock);

  conn->unbind = true;
  rpmsg_socket_post(&conn->sendsem);
  rpmsg_socket_post(&conn->recvsem);
  rpmsg_socket_poll_notify(conn, POLLIN | POLLOUT);

  nxmutex_unlock(&conn->recvlock);
}

static void rpmsg_socket_ept_release(FAR struct rpmsg_endpoint *ept)
{
  FAR struct rpmsg_socket_conn_s *conn = ept->priv;

  /* Release conn only when close is called */

  if (!conn->crefs)
    {
      rpmsg_socket_free(conn);
    }
}

static void rpmsg_socket_format_name(FAR struct rpmsg_socket_conn_s *conn,
                                     FAR char *namebuf)
{
  if (strlen(conn->rpaddr.rp_name) > RPMSG_SOCKET_NAME_LEN)
    {
      uint32_t crc = crc32((const uint8_t *)conn->rpaddr.rp_name,
                           strlen(conn->rpaddr.rp_name));
      snprintf(namebuf, RPMSG_NAME_SIZE, "%s%.2s%08" PRIx32 "%s",
               RPMSG_SOCKET_NAME_PREFIX, conn->rpaddr.rp_name,
               crc, conn->nameid);
    }
  else
    {
      snprintf(namebuf, RPMSG_NAME_SIZE, "%s%.*s%s",
               RPMSG_SOCKET_NAME_PREFIX, RPMSG_SOCKET_NAME_LEN,
               conn->rpaddr.rp_name, conn->nameid);
    }
}

static void rpmsg_socket_device_created(FAR struct rpmsg_device *rdev,
                                        FAR void *priv)
{
  FAR struct rpmsg_socket_conn_s *conn = priv;
  char namebuf[RPMSG_NAME_SIZE];

  if (conn->ept.rdev)
    {
      return;
    }

  if (strcmp(conn->rpaddr.rp_cpu, rpmsg_get_cpuname(rdev)) == 0)
    {
      conn->ept.priv = conn;
      conn->ept.ns_bound_cb = rpmsg_socket_ns_bound;
      conn->ept.release_cb = rpmsg_socket_ept_release;
      rpmsg_socket_format_name(conn, namebuf);

      rpmsg_create_ept(&conn->ept, rdev, namebuf,
                       RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
                       rpmsg_socket_ept_cb, rpmsg_socket_ns_unbind);
    }
}

static bool rpmsg_socket_ns_match(FAR struct rpmsg_device *rdev,
                                  FAR void *priv, FAR const char *name,
                                  uint32_t dest)
{
  FAR struct rpmsg_socket_conn_s *server = priv;
  char namebuf[RPMSG_NAME_SIZE];

  rpmsg_socket_format_name(server, namebuf);
  if (strncmp(name, namebuf, strlen(namebuf)))
    {
      return false;
    }

  if (server->rpaddr.rp_cpu[0] &&
      strcmp(server->rpaddr.rp_cpu, rpmsg_get_cpuname(rdev)))
    {
      /* Bind specific CPU, then only listen that CPU */

      return false;
    }

  return true;
}

static void rpmsg_socket_ns_bind(FAR struct rpmsg_device *rdev,
                                 FAR void *priv, FAR const char *name,
                                 uint32_t dest)
{
  FAR struct rpmsg_socket_conn_s *server = priv;
  FAR struct rpmsg_socket_conn_s *tmp;
  FAR struct rpmsg_socket_conn_s *new;
  int cnt = 0;
  int ret;

  new = rpmsg_socket_alloc();
  if (!new)
    {
      return;
    }

  ret = circbuf_resize(&new->recvbuf, server->recvsize);
  if (ret < 0)
    {
      rpmsg_socket_free(new);
      return;
    }

  new->ept.priv = new;
  new->ept.release_cb = rpmsg_socket_ept_release;
  ret = rpmsg_create_ept(&new->ept, rdev, name,
                         RPMSG_ADDR_ANY, dest,
                         rpmsg_socket_ept_cb, rpmsg_socket_ns_unbind);
  if (ret < 0)
    {
      rpmsg_socket_free(new);
      return;
    }

  new->rpaddr.rp_family = AF_RPMSG;
  strlcpy(new->rpaddr.rp_cpu, rpmsg_get_cpuname(rdev),
          sizeof(new->rpaddr.rp_cpu));
  strlcpy(new->rpaddr.rp_name, name + RPMSG_SOCKET_NAME_PREFIX_LEN,
          sizeof(new->rpaddr.rp_name));

  rpmsg_socket_ns_bound(&new->ept);

  nxmutex_lock(&server->recvlock);
  for (tmp = server; tmp->next; tmp = tmp->next)
    {
      if (++cnt >= server->backlog)
        {
          /* Reject the connection */

          nxmutex_unlock(&server->recvlock);
          rpmsg_destroy_ept(&new->ept);
          rpmsg_socket_free(new);
          return;
        }
    }

  tmp->next = new;

  rpmsg_socket_post(&server->recvsem);
  rpmsg_socket_poll_notify(server, POLLIN);
  nxmutex_unlock(&server->recvlock);
}

static int rpmsg_socket_getaddr(FAR struct rpmsg_socket_conn_s *conn,
                                FAR struct sockaddr *addr,
                                FAR socklen_t *addrlen)
{
  if (!addr || *addrlen < sizeof(struct sockaddr_rpmsg))
    {
      return -EINVAL;
    }

  memcpy(addr, &conn->rpaddr, sizeof(struct sockaddr_rpmsg));
  *addrlen = sizeof(struct sockaddr_rpmsg);

  return 0;
}

static int rpmsg_socket_setaddr(FAR struct rpmsg_socket_conn_s *conn,
                                FAR const struct sockaddr *addr,
                                socklen_t addrlen, bool suffix)
{
  FAR struct sockaddr_rpmsg *rpaddr = (FAR struct sockaddr_rpmsg *)addr;

  if (rpaddr->rp_family != AF_RPMSG ||
      addrlen < sizeof(struct sockaddr_rpmsg))
    {
      return -EINVAL;
    }

  memcpy(&conn->rpaddr, rpaddr, sizeof(struct sockaddr_rpmsg));

  if (suffix)
    {
      snprintf(conn->nameid, sizeof(conn->nameid), ":%" PRIx32,
               g_rpmsg_id++);
    }
  else
    {
      conn->nameid[0] = 0;
    }

  return 0;
}

static int rpmsg_socket_setup(FAR struct socket *psock)
{
  FAR struct rpmsg_socket_conn_s *conn;

  conn = rpmsg_socket_alloc();
  if (!conn)
    {
      return -ENOMEM;
    }

  psock->s_conn = conn;
  return 0;
}

static sockcaps_t rpmsg_socket_sockcaps(FAR struct socket *psock)
{
  return SOCKCAP_NONBLOCKING;
}

static void rpmsg_socket_addref(FAR struct socket *psock)
{
  FAR struct rpmsg_socket_conn_s *conn = psock->s_conn;

  conn->crefs++;
}

static int rpmsg_socket_bind(FAR struct socket *psock,
                            FAR const struct sockaddr *addr,
                            socklen_t addrlen)
{
  return rpmsg_socket_setaddr(psock->s_conn, addr, addrlen, false);
}

static int rpmsg_socket_getsockname(FAR struct socket *psock,
                                    FAR struct sockaddr *addr,
                                    FAR socklen_t *addrlen)
{
  int ret;

  ret = rpmsg_socket_getaddr(psock->s_conn, addr, addrlen);
  if (ret >= 0)
    {
      strlcpy(((struct sockaddr_rpmsg *)addr)->rp_cpu,
              CONFIG_RPMSG_LOCAL_CPUNAME, RPMSG_SOCKET_CPU_SIZE);
    }

  return ret;
}

static int rpmsg_socket_getpeername(FAR struct socket *psock,
                                    FAR struct sockaddr *addr,
                                    FAR socklen_t *addrlen)
{
  return rpmsg_socket_getaddr(psock->s_conn, addr, addrlen);
}

static int rpmsg_socket_listen(FAR struct socket *psock, int backlog)
{
  FAR struct rpmsg_socket_conn_s *server = psock->s_conn;

  if (psock->s_type != SOCK_STREAM)
    {
      return -ENOSYS;
    }

  if (!_SS_ISBOUND(server->sconn.s_flags) || backlog <= 0)
    {
      return -EINVAL;
    }

  server->backlog = backlog;
  return rpmsg_register_callback(server,
                                 NULL,
                                 NULL,
                                 rpmsg_socket_ns_match,
                                 rpmsg_socket_ns_bind);
}

static int rpmsg_socket_connect_internal(FAR struct socket *psock)
{
  FAR struct rpmsg_socket_conn_s *conn = psock->s_conn;
  int ret;

  ret = circbuf_resize(&conn->recvbuf, conn->recvsize);
  if (ret < 0)
    {
      return ret;
    }

  ret = rpmsg_register_callback(conn,
                                rpmsg_socket_device_created,
                                NULL,
                                NULL,
                                NULL);
  if (ret < 0)
    {
      return ret;
    }

  nxmutex_lock(&conn->sendlock);
  if (conn->sendsize == 0)
    {
      nxmutex_unlock(&conn->sendlock);
      if (_SS_ISNONBLOCK(conn->sconn.s_flags))
        {
          return -EINPROGRESS;
        }

      ret = net_sem_timedwait(&conn->sendsem,
                              _SO_TIMEOUT(conn->sconn.s_sndtimeo));

      if (ret < 0)
        {
          rpmsg_unregister_callback(conn,
                                    rpmsg_socket_device_created,
                                    NULL,
                                    NULL,
                                    NULL);
          rpmsg_socket_destroy_ept(conn);
        }
    }
  else
    {
      nxmutex_unlock(&conn->sendlock);
    }

  return ret;
}

static int rpmsg_socket_connect(FAR struct socket *psock,
                                FAR const struct sockaddr *addr,
                                socklen_t addrlen)
{
  FAR struct rpmsg_socket_conn_s *conn = psock->s_conn;
  int ret;

  if (_SS_ISCONNECTED(conn->sconn.s_flags))
    {
      return -EISCONN;
    }

  ret = rpmsg_socket_setaddr(conn, addr, addrlen,
                             psock->s_type == SOCK_STREAM);
  if (ret < 0)
    {
      return ret;
    }

  return rpmsg_socket_connect_internal(psock);
}

static int rpmsg_socket_accept(FAR struct socket *psock,
                               FAR struct sockaddr *addr,
                               FAR socklen_t *addrlen,
                               FAR struct socket *newsock,
                               int flags)
{
  FAR struct rpmsg_socket_conn_s *server = psock->s_conn;
  int ret = 0;

  if (!_SS_ISLISTENING(server->sconn.s_flags))
    {
      return -EINVAL;
    }

  while (1)
    {
      FAR struct rpmsg_socket_conn_s *conn = NULL;

      nxmutex_lock(&server->recvlock);

      if (server->next)
        {
          conn = server->next;
          server->next = conn->next;
          conn->next = NULL;
        }

      nxmutex_unlock(&server->recvlock);

      if (conn)
        {
          if (conn->sendsize == 0)
            {
              net_sem_wait(&conn->sendsem);
            }

          newsock->s_domain = psock->s_domain;
          newsock->s_sockif = psock->s_sockif;
          newsock->s_type   = SOCK_STREAM;
          newsock->s_conn   = conn;

          rpmsg_socket_getaddr(conn, addr, addrlen);
          break;
        }
      else
        {
          if (_SS_ISNONBLOCK(server->sconn.s_flags))
            {
              ret = -EAGAIN;
              break;
            }
          else
            {
              ret = net_sem_wait(&server->recvsem);
              if (ret < 0)
                {
                  break;
                }
            }
        }
    }

  return ret;
}

static int rpmsg_socket_poll(FAR struct socket *psock,
                             FAR struct pollfd *fds, bool setup)
{
  FAR struct rpmsg_socket_conn_s *conn = psock->s_conn;
  pollevent_t eventset = 0;
  int i;

  if (setup)
    {
      nxmutex_lock(&conn->polllock);
      for (i = 0; i < CONFIG_NET_RPMSG_NPOLLWAITERS; i++)
        {
          /* Find an available slot */

          if (!conn->fds[i])
            {
              /* Bind the poll structure and this slot */

              conn->fds[i] = fds;
              fds->priv    = &conn->fds[i];
              break;
            }
        }

      nxmutex_unlock(&conn->polllock);
      if (i >= CONFIG_NET_RPMSG_NPOLLWAITERS)
        {
          fds->priv = NULL;
          return -EBUSY;
        }

      /* Immediately notify on any of the requested events */

      if (_SS_ISLISTENING(conn->sconn.s_flags))
        {
          nxmutex_lock(&conn->recvlock);
          if (conn->next)
            {
              eventset |= POLLIN;
            }

          nxmutex_unlock(&conn->recvlock);
        }
      else if (_SS_ISCONNECTED(conn->sconn.s_flags))
        {
          if (!conn->ept.rdev || conn->unbind)
            {
              eventset |= POLLHUP;
            }

          nxmutex_lock(&conn->sendlock);
          if (rpmsg_socket_get_space(conn) > 0)
            {
              eventset |= POLLOUT;
            }

          nxmutex_unlock(&conn->sendlock);

          nxmutex_lock(&conn->recvlock);
          if (!circbuf_is_empty(&conn->recvbuf))
            {
              eventset |= POLLIN;
            }

          nxmutex_unlock(&conn->recvlock);
        }
      else /* !_SS_ISCONNECTED(conn->sconn.s_flags) */
        {
          if ((!conn->ept.rdev || conn->unbind) &&
              !_SS_INITD(conn->sconn.s_flags))
            {
              eventset |= POLLHUP;
            }
        }

      poll_notify(&fds, 1, eventset);
    }
  else
    {
      nxmutex_lock(&conn->polllock);
      if (fds->priv != NULL)
        {
          for (i = 0; i < CONFIG_NET_RPMSG_NPOLLWAITERS; i++)
            {
              if (fds == conn->fds[i])
                {
                  conn->fds[i] = NULL;
                  fds->priv = NULL;
                  break;
                }
            }
        }

      nxmutex_unlock(&conn->polllock);
    }

  return OK;
}

static uint32_t rpmsg_socket_get_iovlen(FAR const struct iovec *buf,
                                        size_t iovcnt)
{
  uint32_t len = 0;
  while (iovcnt--)
    {
      len += (buf++)->iov_len;
    }

  return len;
}

static ssize_t rpmsg_socket_send_continuous(FAR struct socket *psock,
                                            FAR const struct iovec *buf,
                                            size_t iovcnt, bool nonblock)
{
  FAR struct rpmsg_socket_conn_s *conn = psock->s_conn;
  uint32_t len = rpmsg_socket_get_iovlen(buf, iovcnt);
  uint32_t written = 0;
  uint32_t offset = 0;
  int ret = 0;

  while (written < len)
    {
      FAR struct rpmsg_socket_data_s *msg;
      uint32_t block_written = 0;
      uint32_t ipcsize;
      uint32_t block;

      nxmutex_lock(&conn->sendlock);
      block = MIN(len - written, rpmsg_socket_get_space(conn));
      nxmutex_unlock(&conn->sendlock);

      if (block == 0)
        {
          if (nonblock)
            {
              ret = -EAGAIN;
              break;
            }

          ret = net_sem_timedwait(&conn->sendsem,
                                  _SO_TIMEOUT(conn->sconn.s_sndtimeo));
          if (!conn->ept.rdev || conn->unbind)
            {
              ret = -ECONNRESET;
            }

          if (ret < 0)
            {
              break;
            }

          continue;
        }

      msg = rpmsg_get_tx_payload_buffer(&conn->ept, &ipcsize, true);
      if (!msg)
        {
          ret = -EINVAL;
          break;
        }

      nxmutex_lock(&conn->sendlock);
      block = MIN(len - written, rpmsg_socket_get_space(conn));
      block = MIN(block, ipcsize - sizeof(*msg));

      msg->cmd = RPMSG_SOCKET_CMD_DATA;
      msg->pos = conn->recvpos;
      msg->len = block;
      while (block_written < block)
        {
          uint32_t chunk = MIN(block - block_written, buf->iov_len - offset);
          memcpy(msg->data + block_written,
                 (FAR const uint8_t *)buf->iov_base + offset, chunk);
          offset += chunk;
          if (offset == buf->iov_len)
            {
              buf++;
              offset = 0;
            }

          block_written += chunk;
        }

      conn->lastpos  = conn->recvpos;
      conn->sendpos += msg->len;

      ret = rpmsg_sendto_nocopy(&conn->ept, msg, block + sizeof(*msg),
                                conn->ept.dest_addr);
      nxmutex_unlock(&conn->sendlock);
      if (ret < 0)
        {
          rpmsg_release_tx_buffer(&conn->ept, msg);
          break;
        }

      written += block;
    }

  return written ? written : ret;
}

static ssize_t rpmsg_socket_send_single(FAR struct socket *psock,
                                        FAR const struct iovec *buf,
                                        size_t iovcnt, bool nonblock)
{
  FAR struct rpmsg_socket_conn_s *conn = psock->s_conn;
  FAR struct rpmsg_socket_data_s *msg;
  uint32_t len = rpmsg_socket_get_iovlen(buf, iovcnt);
  uint32_t total = len + sizeof(uint32_t);
  uint32_t written = 0;
  uint32_t ipcsize;
  uint32_t space;
  FAR uint8_t *msgpos;
  int ret;

  if (total > conn->sendsize)
    {
      return -EFBIG;
    }

  while (1)
    {
      nxmutex_lock(&conn->sendlock);
      space = rpmsg_socket_get_space(conn);
      nxmutex_unlock(&conn->sendlock);

      if (space >= total)
        {
          break;
        }

      if (nonblock)
        {
          return -EAGAIN;
        }

      ret = net_sem_timedwait(&conn->sendsem,
                              _SO_TIMEOUT(conn->sconn.s_sndtimeo));
      if (!conn->ept.rdev || conn->unbind)
        {
          ret = -ECONNRESET;
        }

      if (ret < 0)
        {
          return ret;
        }
    }

  msg = rpmsg_get_tx_payload_buffer(&conn->ept, &ipcsize, true);
  if (!msg)
    {
      return -EINVAL;
    }

  nxmutex_lock(&conn->sendlock);
  total = MIN(total, rpmsg_socket_get_space(conn));
  total = MIN(total, ipcsize - sizeof(*msg));
  len = total - sizeof(uint32_t);

  /* SOCK_DGRAM need write len to buffer */

  msg->cmd = RPMSG_SOCKET_CMD_DATA;
  msg->pos = conn->recvpos;
  msg->len = len;
  memcpy(msg->data, &len, sizeof(uint32_t));
  msgpos = msg->data + sizeof(uint32_t);
  while (written < len)
    {
      if (len - written < buf->iov_len)
        {
          memcpy(msgpos, buf->iov_base, len - written);
          written = len;
        }
      else
        {
          memcpy(msgpos, buf->iov_base, buf->iov_len);
          written += buf->iov_len;
          msgpos  += buf->iov_len;
          buf++;
        }
    }

  conn->lastpos  = conn->recvpos;
  conn->sendpos += total;

  ret = rpmsg_sendto_nocopy(&conn->ept, msg, total + sizeof(*msg),
                            conn->ept.dest_addr);
  nxmutex_unlock(&conn->sendlock);
  if (ret < 0)
    {
      rpmsg_release_tx_buffer(&conn->ept, msg);
    }

  return ret > 0 ? len : ret;
}

static ssize_t rpmsg_socket_sendmsg(FAR struct socket *psock,
                                    FAR struct msghdr *msg, int flags)
{
  FAR struct rpmsg_socket_conn_s *conn = psock->s_conn;
  FAR const struct iovec *buf = msg->msg_iov;
  size_t len = msg->msg_iovlen;
  FAR const struct sockaddr *to = msg->msg_name;
  socklen_t tolen = msg->msg_namelen;
  bool nonblock;
  ssize_t ret;

  if (!_SS_ISCONNECTED(conn->sconn.s_flags))
    {
      if (to == NULL)
        {
          return -ENOTCONN;
        }

      ret = rpmsg_socket_connect(psock, to, tolen);
      if (ret < 0)
        {
          return ret;
        }
    }

  if (!conn->ept.rdev || conn->unbind)
    {
      /* return ECONNRESET if lower IPC closed */

      return -ECONNRESET;
    }

  if (conn->how & SHUT_WR)
    {
      return -EPIPE;
    }

  nonblock = _SS_ISNONBLOCK(conn->sconn.s_flags) ||
             (flags & MSG_DONTWAIT) != 0;

  if (psock->s_type == SOCK_STREAM)
    {
      return rpmsg_socket_send_continuous(psock, buf, len, nonblock);
    }
  else
    {
      return rpmsg_socket_send_single(psock, buf, len, nonblock);
    }
}

static ssize_t rpmsg_socket_recvmsg(FAR struct socket *psock,
                                    FAR struct msghdr *msg, int flags)
{
  FAR struct rpmsg_socket_conn_s *conn = psock->s_conn;
  FAR struct sockaddr *from = msg->msg_name;
  FAR socklen_t *fromlen = &msg->msg_namelen;
  FAR void *buf = msg->msg_iov->iov_base;
  size_t len = msg->msg_iov->iov_len;
  ssize_t ret;

  if (msg->msg_iovlen != 1)
    {
      return -ENOTSUP;
    }

  if (psock->s_type != SOCK_STREAM &&
      _SS_ISBOUND(conn->sconn.s_flags) &&
      !_SS_ISCONNECTED(conn->sconn.s_flags))
    {
      ret = rpmsg_socket_connect_internal(psock);
      if (ret < 0)
        {
          return ret;
        }
    }

  if (!_SS_ISCONNECTED(conn->sconn.s_flags))
    {
      return -EISCONN;
    }

  if (conn->how & SHUT_RD)
    {
      return 0;
    }

  nxmutex_lock(&conn->recvlock);
  if (psock->s_type != SOCK_STREAM)
    {
      uint32_t datalen;

      ret = circbuf_read(&conn->recvbuf, &datalen, sizeof(uint32_t));
      if (ret > 0)
        {
          ret = circbuf_read(&conn->recvbuf, buf, MIN(datalen, len));
          if (ret > 0 && datalen > ret)
            {
              circbuf_skip(&conn->recvbuf, datalen - ret);
            }

          conn->recvpos += datalen + sizeof(uint32_t);
        }
    }
  else
    {
      ret = circbuf_read(&conn->recvbuf, buf, len);
      conn->recvpos += ret > 0 ? ret : 0;
    }

  if (ret > 0)
    {
      goto out;
    }

  if (!conn->ept.rdev || conn->unbind)
    {
      /* return EOF if lower IPC closed */

      ret = 0;
      goto out;
    }

  if (_SS_ISNONBLOCK(conn->sconn.s_flags) || (flags & MSG_DONTWAIT) != 0)
    {
      ret = -EAGAIN;
      goto out;
    }

  conn->recvdata = buf;
  conn->recvlen  = len;

  nxsem_reset(&conn->recvsem, 0);
  nxmutex_unlock(&conn->recvlock);

  ret = net_sem_timedwait(&conn->recvsem,
                          _SO_TIMEOUT(conn->sconn.s_rcvtimeo));
  if (!conn->ept.rdev || conn->unbind)
    {
      ret = 0;
    }

  nxmutex_lock(&conn->recvlock);
  if (!conn->recvdata)
    {
      ret = conn->recvlen;
    }
  else
    {
      conn->recvdata = NULL;
    }

out:
  nxmutex_unlock(&conn->recvlock);
  if (ret > 0)
    {
      rpmsg_socket_wakeup(conn);
      rpmsg_socket_getaddr(conn, from, fromlen);
    }

  return ret;
}

static int rpmsg_socket_close(FAR struct socket *psock)
{
  FAR struct rpmsg_socket_conn_s *conn = psock->s_conn;

  if (--conn->crefs)
    {
      return 0;
    }

  if (conn->backlog > 0)
    {
      rpmsg_unregister_callback(conn,
                                NULL,
                                NULL,
                                rpmsg_socket_ns_match,
                                rpmsg_socket_ns_bind);
    }
  else
    {
      rpmsg_unregister_callback(conn,
                                rpmsg_socket_device_created,
                                NULL,
                                NULL,
                                NULL);
    }

  if (conn->ept.rdev)
    {
      rpmsg_socket_destroy_ept(conn);
    }
  else
    {
      rpmsg_socket_free(conn);
    }

  return 0;
}

static void rpmsg_socket_path(FAR struct rpmsg_socket_conn_s *conn,
                              FAR char *buf, size_t len)
{
  if (conn->backlog) /* Server */
    {
      snprintf(buf, len,
               "rpmsg:[%s:[%s%s]<->%s]",
               CONFIG_RPMSG_LOCAL_CPUNAME, conn->rpaddr.rp_name,
               conn->nameid, conn->rpaddr.rp_cpu);
    }
  else /* Client */
    {
      snprintf(buf, len,
               "rpmsg:[%s<->%s:[%s%s]]",
               CONFIG_RPMSG_LOCAL_CPUNAME, conn->rpaddr.rp_cpu,
               conn->rpaddr.rp_name, conn->nameid);
    }
}

static int rpmsg_socket_ioctl(FAR struct socket *psock,
                              int cmd, unsigned long arg)
{
  FAR struct rpmsg_socket_conn_s *conn = psock->s_conn;
  int ret = OK;

  switch (cmd)
    {
      case FIONREAD:
        *(FAR int *)((uintptr_t)arg) = circbuf_used(&conn->recvbuf);
        break;

      case FIONSPACE:
        *(FAR int *)((uintptr_t)arg) = rpmsg_socket_get_space(conn);
        break;

      case FIOC_FILEPATH:
        rpmsg_socket_path(conn, (FAR char *)(uintptr_t)arg, PATH_MAX);
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}

static int rpmsg_socket_shutdown(FAR struct socket *psock, int how)
{
  FAR struct rpmsg_socket_conn_s *conn = psock->s_conn;
  struct rpmsg_socket_shutdown_s msg;
  int ret;

  if (!conn->ept.rdev || conn->unbind)
    {
      return -ENOTCONN;
    }

  conn->how |= how;

  msg.cmd = RPMSG_SOCKET_CMD_SHUTDOWN;
  msg.how = how;

  ret = rpmsg_send(&conn->ept, &msg, sizeof(msg));
  if (ret < 0)
      return ret;

  return OK;
}

#ifdef CONFIG_NET_SOCKOPTS
static int rpmsg_socket_getsockopt(FAR struct socket *psock, int level,
                                   int option, FAR void *value,
                                   FAR socklen_t *value_len)
{
  FAR struct rpmsg_socket_conn_s *conn = psock->s_conn;

  if (level == SOL_SOCKET)
    {
      switch (option)
        {
          case SO_PEERCRED:
            {
              if (!value || *value_len != sizeof(struct ucred))
                {
                  return -EINVAL;
                }

              memcpy(value, &conn->cred, sizeof(struct ucred));
              return OK;
            }

          case SO_RCVBUF:
            {
              if (*value_len != sizeof(int))
                {
                  return -EINVAL;
                }

              *(FAR int *)value = conn->recvsize;
              return OK;
            }

          case SO_SNDBUF:
            {
              if (*value_len != sizeof(int))
                {
                  return -EINVAL;
                }

              *(FAR int *)value = conn->sendsize;
              return OK;
            }
        }
    }

  return -ENOPROTOOPT;
}

static int rpmsg_socket_setsockopt(FAR struct socket *psock, int level,
                                   int option, FAR const void *value,
                                   socklen_t value_len)
{
  FAR struct rpmsg_socket_conn_s *conn = psock->s_conn;

  if (level == SOL_SOCKET && option == SO_RCVBUF)
    {
      if (value_len < sizeof(int))
        {
          return -EINVAL;
        }

      if (_SS_ISCONNECTED(conn->sconn.s_flags))
        {
          return -EISCONN;
        }

      conn->recvsize = *(FAR const int *)value;
      return OK;
    }

  return -ENOPROTOOPT;
}

#endif
