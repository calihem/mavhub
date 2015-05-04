#ifndef _TOOLBOX_H_
#define _TOOLBOX_H_

// defined in /usr/include/glib-2.0/glib/gmacros.h
#ifndef ABS
#define ABS(x) ((x)<0?-(x):(x))
#endif

#define SGN(x) ((x)==0?0:((x)>0?1:-1))
#define SGN2(x) ((x > 0) - (x < 0))

#define POWER2(x) ((x)*(x))

/**
	fast 32 bit integer square root
*/
#define ITERATE(N) \
    tries = root + (1 << (N)); \
    if (n >= tries << (N))   \
    {   n -= tries << (N);   \
        root |= 2 << (N); \
    }
inline uint32_t isqrt (uint32_t n) {
	uint32_t root = 0, tries;
	ITERATE(15); ITERATE(14); ITERATE(13); ITERATE(12);
	ITERATE(11); ITERATE(10); ITERATE( 9); ITERATE( 8);
	ITERATE( 7); ITERATE( 6); ITERATE( 5); ITERATE( 4);
	ITERATE( 3); ITERATE( 2); ITERATE( 1); ITERATE( 0);

	return root >> 1;
}

#endif
