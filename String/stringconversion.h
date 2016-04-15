/*
 * stringconversion.h
 *
 *  Created on: Dec 19, 2013
 *      Author: jan
 */

#ifndef STRINGCONVERSION_H_
#define STRINGCONVERSION_H_

#include <stdint.h>

#define FLOAT_PRECISION		10000

char* ftoa(float num, char* out);

char* StringCopy(char *new, char *old);

/*
 * Compares two strings. Returns 1 if they have the same content,
 * returns 0 otherwise. If the strings are not of equal length, the
 * functions aborts at the end of the shorter string and returns the
 * result so far. E.g. comparing "foo" and "foobar" would return 1
 */
uint8_t StringEquals(char *l, char *r);

/* Custom reverse() function for which the length of the string is
 * known in advance (avoids a call to strlen() and including
 * string.h).
 *
 * @args: a null-terminated string
 * @return: nothing
 * @result: the characters in str are reversed
 */
void reverse(char* str, int length);
/* Returns the string representation of integer n. Assumes 32-bit
 * int, and 8-bit bytes (i.e. sizeof(char) = 1, sizeof(int) = 4).
 * Assumes char *out is big enough to hold the string
 * representation of n.
 *
 * @args: int n to convert, char* out for the result
 * @result the string representation of n is stored in out
 * @return 0 on success, -1 on error
 */
char* itoASCII(int32_t n, char* out);

#endif /* STRINGCONVERSION_H_ */
