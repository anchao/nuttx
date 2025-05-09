/****************************************************************************
 * crypto/sk.h
 *
 * SPDX-License-Identifier: SSLeay-standalone
 * SPDX-FileCopyrightText: Copyright (C) 1995 Eric Young (eay@mincom.oz.au)
 * SPDX-FileCopyrightText: Eric Young (eay@mincom.oz.au).
 *
 * This file is part of an SSL implementation written
 * by Eric Young (eay@mincom.oz.au).
 * The implementation was written so as to conform with Netscapes SSL
 * specification.  This library and applications are
 * FREE FOR COMMERCIAL AND NON-COMMERCIAL USE
 * as long as the following conditions are aheared to.
 *
 * Copyright remains Eric Young's, and as such any Copyright notices in
 * the code are not to be removed.  If this code is used in a product,
 * Eric Young should be given attribution as the author of the parts used.
 * This can be in the form of a textual message at program startup or
 * in documentation (online or textual) provided with the package.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *    This product includes software developed
 *    by Eric Young (eay@mincom.oz.au)
 *
 * THIS SOFTWARE IS PROVIDED BY ERIC YOUNG ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAl,  SPECIAl,
 * EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * The licence and distribution terms for any publically
 * available version or derivative of this code cannot be changed.
 * i.e. this code cannot simply be
 * copied and put under another distribution licence
 * [including the GNU Public Licence.]
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>

static const uint32_t des_skb[8][64] =
{
  {
    /* for C bits (numbered as per FIPS 46) 1 2 3 4 5 6 */

    0x00000000l, 0x00000010l, 0x20000000l, 0x20000010l,
    0x00010000l, 0x00010010l, 0x20010000l, 0x20010010l,
    0x00000800l, 0x00000810l, 0x20000800l, 0x20000810l,
    0x00010800l, 0x00010810l, 0x20010800l, 0x20010810l,
    0x00000020l, 0x00000030l, 0x20000020l, 0x20000030l,
    0x00010020l, 0x00010030l, 0x20010020l, 0x20010030l,
    0x00000820l, 0x00000830l, 0x20000820l, 0x20000830l,
    0x00010820l, 0x00010830l, 0x20010820l, 0x20010830l,
    0x00080000l, 0x00080010l, 0x20080000l, 0x20080010l,
    0x00090000l, 0x00090010l, 0x20090000l, 0x20090010l,
    0x00080800l, 0x00080810l, 0x20080800l, 0x20080810l,
    0x00090800l, 0x00090810l, 0x20090800l, 0x20090810l,
    0x00080020l, 0x00080030l, 0x20080020l, 0x20080030l,
    0x00090020l, 0x00090030l, 0x20090020l, 0x20090030l,
    0x00080820l, 0x00080830l, 0x20080820l, 0x20080830l,
    0x00090820l, 0x00090830l, 0x20090820l, 0x20090830l,
  },
  {
    /* for C bits (numbered as per FIPS 46) 7 8 10 11 12 13 */

    0x00000000l, 0x02000000l, 0x00002000l, 0x02002000l,
    0x00200000l, 0x02200000l, 0x00202000l, 0x02202000l,
    0x00000004l, 0x02000004l, 0x00002004l, 0x02002004l,
    0x00200004l, 0x02200004l, 0x00202004l, 0x02202004l,
    0x00000400l, 0x02000400l, 0x00002400l, 0x02002400l,
    0x00200400l, 0x02200400l, 0x00202400l, 0x02202400l,
    0x00000404l, 0x02000404l, 0x00002404l, 0x02002404l,
    0x00200404l, 0x02200404l, 0x00202404l, 0x02202404l,
    0x10000000l, 0x12000000l, 0x10002000l, 0x12002000l,
    0x10200000l, 0x12200000l, 0x10202000l, 0x12202000l,
    0x10000004l, 0x12000004l, 0x10002004l, 0x12002004l,
    0x10200004l, 0x12200004l, 0x10202004l, 0x12202004l,
    0x10000400l, 0x12000400l, 0x10002400l, 0x12002400l,
    0x10200400l, 0x12200400l, 0x10202400l, 0x12202400l,
    0x10000404l, 0x12000404l, 0x10002404l, 0x12002404l,
    0x10200404l, 0x12200404l, 0x10202404l, 0x12202404l,
  },
  {
    /* for C bits (numbered as per FIPS 46) 14 15 16 17 19 20 */

    0x00000000l, 0x00000001l, 0x00040000l, 0x00040001l,
    0x01000000l, 0x01000001l, 0x01040000l, 0x01040001l,
    0x00000002l, 0x00000003l, 0x00040002l, 0x00040003l,
    0x01000002l, 0x01000003l, 0x01040002l, 0x01040003l,
    0x00000200l, 0x00000201l, 0x00040200l, 0x00040201l,
    0x01000200l, 0x01000201l, 0x01040200l, 0x01040201l,
    0x00000202l, 0x00000203l, 0x00040202l, 0x00040203l,
    0x01000202l, 0x01000203l, 0x01040202l, 0x01040203l,
    0x08000000l, 0x08000001l, 0x08040000l, 0x08040001l,
    0x09000000l, 0x09000001l, 0x09040000l, 0x09040001l,
    0x08000002l, 0x08000003l, 0x08040002l, 0x08040003l,
    0x09000002l, 0x09000003l, 0x09040002l, 0x09040003l,
    0x08000200l, 0x08000201l, 0x08040200l, 0x08040201l,
    0x09000200l, 0x09000201l, 0x09040200l, 0x09040201l,
    0x08000202l, 0x08000203l, 0x08040202l, 0x08040203l,
    0x09000202l, 0x09000203l, 0x09040202l, 0x09040203l,
  },
  {
    /* for C bits (numbered as per FIPS 46) 21 23 24 26 27 28 */

    0x00000000l, 0x00100000l, 0x00000100l, 0x00100100l,
    0x00000008l, 0x00100008l, 0x00000108l, 0x00100108l,
    0x00001000l, 0x00101000l, 0x00001100l, 0x00101100l,
    0x00001008l, 0x00101008l, 0x00001108l, 0x00101108l,
    0x04000000l, 0x04100000l, 0x04000100l, 0x04100100l,
    0x04000008l, 0x04100008l, 0x04000108l, 0x04100108l,
    0x04001000l, 0x04101000l, 0x04001100l, 0x04101100l,
    0x04001008l, 0x04101008l, 0x04001108l, 0x04101108l,
    0x00020000l, 0x00120000l, 0x00020100l, 0x00120100l,
    0x00020008l, 0x00120008l, 0x00020108l, 0x00120108l,
    0x00021000l, 0x00121000l, 0x00021100l, 0x00121100l,
    0x00021008l, 0x00121008l, 0x00021108l, 0x00121108l,
    0x04020000l, 0x04120000l, 0x04020100l, 0x04120100l,
    0x04020008l, 0x04120008l, 0x04020108l, 0x04120108l,
    0x04021000l, 0x04121000l, 0x04021100l, 0x04121100l,
    0x04021008l, 0x04121008l, 0x04021108l, 0x04121108l,
  },
  {
    /* for D bits (numbered as per FIPS 46) 1 2 3 4 5 6 */

    0x00000000l, 0x10000000l, 0x00010000l, 0x10010000l,
    0x00000004l, 0x10000004l, 0x00010004l, 0x10010004l,
    0x20000000l, 0x30000000l, 0x20010000l, 0x30010000l,
    0x20000004l, 0x30000004l, 0x20010004l, 0x30010004l,
    0x00100000l, 0x10100000l, 0x00110000l, 0x10110000l,
    0x00100004l, 0x10100004l, 0x00110004l, 0x10110004l,
    0x20100000l, 0x30100000l, 0x20110000l, 0x30110000l,
    0x20100004l, 0x30100004l, 0x20110004l, 0x30110004l,
    0x00001000l, 0x10001000l, 0x00011000l, 0x10011000l,
    0x00001004l, 0x10001004l, 0x00011004l, 0x10011004l,
    0x20001000l, 0x30001000l, 0x20011000l, 0x30011000l,
    0x20001004l, 0x30001004l, 0x20011004l, 0x30011004l,
    0x00101000l, 0x10101000l, 0x00111000l, 0x10111000l,
    0x00101004l, 0x10101004l, 0x00111004l, 0x10111004l,
    0x20101000l, 0x30101000l, 0x20111000l, 0x30111000l,
    0x20101004l, 0x30101004l, 0x20111004l, 0x30111004l,
  },
  {
    /* for D bits (numbered as per FIPS 46) 8 9 11 12 13 14 */

    0x00000000l, 0x08000000l, 0x00000008l, 0x08000008l,
    0x00000400l, 0x08000400l, 0x00000408l, 0x08000408l,
    0x00020000l, 0x08020000l, 0x00020008l, 0x08020008l,
    0x00020400l, 0x08020400l, 0x00020408l, 0x08020408l,
    0x00000001l, 0x08000001l, 0x00000009l, 0x08000009l,
    0x00000401l, 0x08000401l, 0x00000409l, 0x08000409l,
    0x00020001l, 0x08020001l, 0x00020009l, 0x08020009l,
    0x00020401l, 0x08020401l, 0x00020409l, 0x08020409l,
    0x02000000l, 0x0a000000l, 0x02000008l, 0x0a000008l,
    0x02000400l, 0x0a000400l, 0x02000408l, 0x0a000408l,
    0x02020000l, 0x0a020000l, 0x02020008l, 0x0a020008l,
    0x02020400l, 0x0a020400l, 0x02020408l, 0x0a020408l,
    0x02000001l, 0x0a000001l, 0x02000009l, 0x0a000009l,
    0x02000401l, 0x0a000401l, 0x02000409l, 0x0a000409l,
    0x02020001l, 0x0a020001l, 0x02020009l, 0x0a020009l,
    0x02020401l, 0x0a020401l, 0x02020409l, 0x0a020409l,
  },
  {
    /* for D bits (numbered as per FIPS 46) 16 17 18 19 20 21 */

    0x00000000l, 0x00000100l, 0x00080000l, 0x00080100l,
    0x01000000l, 0x01000100l, 0x01080000l, 0x01080100l,
    0x00000010l, 0x00000110l, 0x00080010l, 0x00080110l,
    0x01000010l, 0x01000110l, 0x01080010l, 0x01080110l,
    0x00200000l, 0x00200100l, 0x00280000l, 0x00280100l,
    0x01200000l, 0x01200100l, 0x01280000l, 0x01280100l,
    0x00200010l, 0x00200110l, 0x00280010l, 0x00280110l,
    0x01200010l, 0x01200110l, 0x01280010l, 0x01280110l,
    0x00000200l, 0x00000300l, 0x00080200l, 0x00080300l,
    0x01000200l, 0x01000300l, 0x01080200l, 0x01080300l,
    0x00000210l, 0x00000310l, 0x00080210l, 0x00080310l,
    0x01000210l, 0x01000310l, 0x01080210l, 0x01080310l,
    0x00200200l, 0x00200300l, 0x00280200l, 0x00280300l,
    0x01200200l, 0x01200300l, 0x01280200l, 0x01280300l,
    0x00200210l, 0x00200310l, 0x00280210l, 0x00280310l,
    0x01200210l, 0x01200310l, 0x01280210l, 0x01280310l,
  },
  {
    /* for D bits (numbered as per FIPS 46) 22 23 24 25 27 28 */

    0x00000000l, 0x04000000l, 0x00040000l, 0x04040000l,
    0x00000002l, 0x04000002l, 0x00040002l, 0x04040002l,
    0x00002000l, 0x04002000l, 0x00042000l, 0x04042000l,
    0x00002002l, 0x04002002l, 0x00042002l, 0x04042002l,
    0x00000020l, 0x04000020l, 0x00040020l, 0x04040020l,
    0x00000022l, 0x04000022l, 0x00040022l, 0x04040022l,
    0x00002020l, 0x04002020l, 0x00042020l, 0x04042020l,
    0x00002022l, 0x04002022l, 0x00042022l, 0x04042022l,
    0x00000800l, 0x04000800l, 0x00040800l, 0x04040800l,
    0x00000802l, 0x04000802l, 0x00040802l, 0x04040802l,
    0x00002800l, 0x04002800l, 0x00042800l, 0x04042800l,
    0x00002802l, 0x04002802l, 0x00042802l, 0x04042802l,
    0x00000820l, 0x04000820l, 0x00040820l, 0x04040820l,
    0x00000822l, 0x04000822l, 0x00040822l, 0x04040822l,
    0x00002820l, 0x04002820l, 0x00042820l, 0x04042820l,
    0x00002822l, 0x04002822l, 0x00042822l, 0x04042822l,
  }
};
