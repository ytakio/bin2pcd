#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	struct {
		float x,y,z;
	};
	struct {
		uint8_t r,g,b,a;
	};
} point_data_t;

#ifdef __cplusplus
}
#endif
