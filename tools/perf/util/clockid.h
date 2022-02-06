/* SPDX-License-Identifier: GPL-2.0 */

#ifndef __PERF_CLOCKID_H
#define __PERF_CLOCKID_H

struct option;
int parse_clockid(const struct option *opt, const char *str, int unset);

const char *clockid_name(clockid_t clk_id);

#ifndef CLOCK_PERF_HW_CLOCK
#define CLOCK_PERF_HW_CLOCK		0x10000000
#endif

#ifndef CLOCK_PERF_HW_CLOCK_NS
#define CLOCK_PERF_HW_CLOCK_NS		0x10000001
#endif

#endif
