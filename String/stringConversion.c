#include "stringconversion.h"

char* ftoa(float num, char* out) {
	int32_t l;
	//unsigned long precision = FLOAT_PRECISION;

	l = num;
	if ((num > 0 ? num : -num) >= 1) {
		out = itoASCII(l, out);
	} else {
		if (num < 0) {
			*out++ = '-';
		}
		*out++ = '0';
	}
	*out++ = '.';
	l = (num - l) * FLOAT_PRECISION;
	if (l < 0)
		l = -l;
	uint32_t prec = FLOAT_PRECISION / 10;
	while (l < prec) {
		prec /= 10;
		*out++ = '0';
	}
	return itoASCII(l, out);
}

char* StringCopy(char *new, char *old) {
	while (*old) {
		*new++ = *old++;
	}
	return new;
}

/*
 * Compares two strings. Returns 1 if they have the same content,
 * returns 0 otherwise. If the strings are not of equal length, the
 * functions aborts at the end of the shorter string and returns the
 * result so far. E.g. comparing "foo" and "foobar" would return 1
 */
uint8_t StringEquals(char *l, char *r) {
	while (*l && *r) {
		if (*l++ != *r++)
			return 0;
	}
	return 1;
}

/* Custom reverse() function for which the length of the string is
 * known in advance (avoids a call to strlen() and including
 * string.h).
 *
 * @args: a null-terminated string
 * @return: nothing
 * @result: the characters in str are reversed
 */
void reverse(char* str, int length) {
	uint8_t i = 0, j = length - 1;
	char tmp;
	while (i < j) {
		tmp = str[i];
		str[i] = str[j];
		str[j] = tmp;
		i++;
		j--;
	}
}

/* Returns the string representation of integer n. Assumes 32-bit
 * int, and 8-bit bytes (i.e. sizeof(char) = 1, sizeof(int) = 4).
 * Assumes char *out is big enough to hold the string
 * representation of n.
 *
 * @args: int n to convert, char* out for the result
 * @result the string representation of n is stored in out
 * @return 0 on success, -1 on error
 */
char* itoASCII(int32_t n, char* out) {
// if negative, need 1 char for the sign
	uint8_t sign = n < 0 ? 1 : 0;
	uint8_t i = 0;
	if (n == 0) {
		out[i++] = '0';
	} else if (n < 0) {
		out[i++] = '-';
		n = -n;
	}
	while (n > 0) {
		out[i++] = '0' + n % 10;
		n /= 10;
	}
	out[i] = '\0';
	reverse(out + sign, i - sign);
	return &out[i];
}

