#include "unittest.h"

extern int nanopb_simple_test();
extern int nvs_unittest();

void unittest() {

#ifdef CONFIG_NVS_UNITTEST
    nvs_unittest();
#endif

#ifdef CONFIG_NANOPB_UNITTEST
    nanopb_simple_test();
#endif
}
