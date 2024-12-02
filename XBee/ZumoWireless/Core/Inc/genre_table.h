/*
 * genre_table.h
 *
 *  Created on: Nov 26, 2024
 *      Author: zender
 */

#include "dance_library.h"
#include <stdio.h>
#include <string.h>

#ifndef INC_GENRE_TABLE_H_
#define INC_GENRE_TABLE_H_

struct table_entry genre_table[3];

void create_table() {
	// Rock
	struct table_entry rock_entry;
	strcpy(rock_entry.genre, "JAZZ");
	rock_entry.fun_ptr = &spin_cw;
	genre_table[0] = rock_entry;

	// Pop
	struct table_entry pop_entry;
	strcpy(pop_entry.genre, "ROCK");
	pop_entry.fun_ptr = &spin_ccw;
	genre_table[1] = pop_entry;

	// Country
	struct table_entry country_entry;
	strcpy(country_entry.genre, "GROOVY");
	country_entry.fun_ptr = &straight_forward;
	genre_table[2] = country_entry;
}

#endif /* INC_GENRE_TABLE_H_ */
