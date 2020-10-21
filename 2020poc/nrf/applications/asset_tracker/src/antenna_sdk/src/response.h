#ifndef _IOTEX_EMB_RESPONSE_H_
#define _IOTEX_EMB_RESPONSE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "parse.h"
struct iotex_st_action_info;

int res_get_data(const char *request, json_parse_rule *rules);
int res_get_hash(const void *action, uint8_t id, char *hash, size_t max_size, char **error);
int res_get_actions(const char *request, struct iotex_st_action_info *actions, size_t max_size);

#ifdef __cplusplus
}
#endif

#endif /* _IOTEX_EMB_RESPONSE_H_ */
