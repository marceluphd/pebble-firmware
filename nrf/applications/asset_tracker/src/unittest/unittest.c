#include "unittest.h"

extern int nanopb_simple_test();
extern int nvs_unittest();


void unittest() {

    nvs_unittest();
    nanopb_simple_test();
}
