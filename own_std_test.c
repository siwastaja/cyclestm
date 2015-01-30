#include <stdio.h>
#include <malloc.h>
#include "own_std.h"

int main()
{
	printf("len = %u, %u\n", o_strnlen("kakka", 10), o_strnlen("kakka", 4));
	printf("5^0 = %u, 10^1 = %u, 10^2 = %u\n", o_pow(5,0), o_pow(10,1), o_pow(10,2));

	char kak[5][20];

	o_utoa16(0, kak[0]);
	o_utoa16(5, kak[1]);
	o_utoa16(10, kak[2]);
	o_utoa16(999, kak[3]);
	o_utoa16(65535, kak[4]);

	printf("utoa 0 = |%s| utoa 5 = |%s| utoa 10 = |%s| utoa 999 = |%s| utoa 65535 = |%s|\n",
		kak[0],kak[1],kak[2],kak[3],kak[4]);

	o_itoa16(0, kak[0]);
	o_itoa16(32767, kak[1]);
	o_itoa16(-1, kak[2]);
	o_itoa16(-999, kak[3]);
	o_itoa16(-32768, kak[4]);

	printf("itoa 0 = |%s| itoa 32767 = |%s| itoa -1 = |%s| itoa -999 = |%s| itoa -32768 = |%s|\n",
		kak[0],kak[1],kak[2],kak[3],kak[4]);


	char* muovi = malloc(100);
	char* muoviorig = muovi;

	muovi = o_str_append(muovi, "Johannes = ");
	muovi = o_itoa16(-999, muovi);
	muovi = o_str_append(muovi, " vihannesta.\n");

	printf("%s", muoviorig);

	printf("atoi: |0| = %u, |5| = %u, |15| = %u, |999| = %u, |999999| = %u\n",
		o_atoi("0"), o_atoi("5"), o_atoi("15"), o_atoi("999"), o_atoi("999999"));

	printf("atoi: | 55555 | = %u, |kak5kuk| = %u, |.55555.| = %u, | 5| = %u, |55555 | = %u\n",
		o_atoi(" 55555 "), o_atoi("kak5kuk"), o_atoi(".55555."), o_atoi(" 5"), o_atoi("55555 "));

	printf("atoi: |-0| = %d, |-5| = %d, |-15| = %d, |-999| = %d, |-999999| = %d\n",
		o_atoi("-0"), o_atoi("-5"), o_atoi("-15"), o_atoi("-999"), o_atoi("-999999"));

	printf("atoi: | -55555 | = %d, |kak-5kuk| = %d, |.-55555.| = %d, | -5| = %d, |-55555 | = %d\n",
		o_atoi(" -55555 "), o_atoi("kak-5kuk"), o_atoi(".-55555."), o_atoi(" -5"), o_atoi("-55555 "));

	return 0;
}
