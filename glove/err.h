/*
 * Smart Glove Demo v1.0
 *
 * Copyright (C) 2025 Soothsayer Systems Ltd. All rights reserved.
 *
 * Author: Tom Coates <tom@soothsys.com>
 */

#ifndef __ERR_H
#define __ERR_H

#ifndef ERR_MODULE_NAME
	#error "ERR_MODULE_NAME is not defined"
#endif /* ERR_MODULE_NAME */

#define ERROR(x, ...)		err_print(false, ERR_MODULE_NAME, (x), ##__VA_ARGS__)
#define ERROR_HALT(x, ...)	err_print(true, ERR_MODULE_NAME, (x), ##__VA_ARGS__)

void err_init(void);
void err_print(bool halt, const char *module, const char *message, ...);

#endif /* __ERR_H */
