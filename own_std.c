#include <inttypes.h>

int o_strnlen(const char* str, int n)
{
	int i = 0;
	while(*(str++) != 0)
		if(++i == n) break;
	return i;
}

int o_pow(int b, int e)
{
	if(e<1) return 1;
	while(--e)
	{
		b*=b;
	}
	return b;
}

char* o_utoa16(uint16_t val, char* str)
{
	static const uint16_t div[5] = {10000,1000,100,10,1};
	int outp_ena = 0;
	int i;
	for(i = 0; i < 5; i++)
	{
		int c = val/div[i];
		val -= c*div[i];
		if(c || i == 4)
			outp_ena = 1;
		if(outp_ena)
			*str++ = (char)c+'0';
	}
	*str = 0;
	return str;
}

char* o_itoa16(int16_t val, char* str)
{
	if(val < 0)
	{
		*str++ = '-';
		val *= -1;
	}

	return o_utoa16(val, str);
}

char* o_str_append(char* str1, char* str2)
{
	while(*str2)
		*str1++ = *str2++;
	*str1 = 0;
	return str1;
}

char* o_str_cmp(char* str1, char* str2)
{
	while(*str2 != 0)
	{
		if(*str1 != *str2)
			return 0;
		str1++;
		str2++;
	}
	return str1;
}

char* o_atoi_append(char* str, int* val_out)
{
	char* first;
	char* last;
	char* pnt;
	int val;
	int multiplier;

	first = str;
	while(*first < '0' || *first > '9')
		first++;

	last = first;
	while(*last >= '0' && *last <= '9')
		last++;

	if(first == last) return 0;

	pnt = last-1;
	multiplier = 1;
	val = 0;

	while(pnt >= first)
	{
		val += ((*pnt)-'0')*multiplier;
		multiplier *= 10;
		pnt--;
	}
	if(pnt >= str && *pnt == '-')
		val *= -1;

	*val_out = val;
	return last;
}
