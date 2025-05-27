/*
 * cli.h
 *
 *  Created on: May 27, 2025
 *      Author: horvz
 */

#ifndef INC_CLI_H_
#define INC_CLI_H_


#include <stdint.h>
#include <stdbool.h>

// cli_printf függvény
void cli_printf(const char *fmt, ...);

// Parancs feldolgozás
bool CLI_ProcessCommand(const char* input);

#endif /* INC_CLI_H_ */
