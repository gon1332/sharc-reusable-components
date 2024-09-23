/**
 * Copyright (c) 2021 - Analog Devices Inc. All Rights Reserved.
 * This software is proprietary and confidential to Analog Devices, Inc.
 * and its licensors.
 *
 * This software is subject to the terms and conditions of the license set
 * forth in the project LICENSE file. Downloading, reproducing, distributing or
 * otherwise using the software constitutes acceptance of the license. The
 * software may not be used except as expressly authorized under the license.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "shell.h"
#include "shell_printf.h"

#ifdef printf
#undef printf
#endif

#define printf(...) shell_printf(ctx, __VA_ARGS__)

/***********************************************************************
 * NOTE: New commands added here must also be added to the appropriate
 *       locations in shell.c
 **********************************************************************/

/***********************************************************************
 * CMD: hello
 **********************************************************************/
const char shell_help_hello[] = "<arg1> <arg2> ... <argX>\n"
  "  argX - Arguments to echo back\n";
const char shell_help_summary_hello[] = "Echoes back command line arguments";

void shell_hello(SHELL_CONTEXT *ctx, int argc, char **argv)
{
    int i;
    for (i = 0; i < argc; i++) {
        printf("Arg %d: %s\n", i, argv[i]);
    }
}
