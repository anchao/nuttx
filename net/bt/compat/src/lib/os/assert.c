/* SPDX-License-Identifier: Apache-2.0 */
#include <assert.h>
#include <nuttx/compiler.h>

void __attribute__((weak)) assert_post_action(const char *file, unsigned int line)
{
    PANIC();
}