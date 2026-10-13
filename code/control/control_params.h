#ifndef MCAR_CONTROL_PARAMS_H
#define MCAR_CONTROL_PARAMS_H
#include "control.h"
/* 参数只存在RAM中；重新上电从config.h装载。所有写操作在主循环执行。 */
uint8 control_param_count(void);
const char *control_param_get(uint8 index, float *value);
uint8 control_param_set(const char *name, float value, const char **error);
#endif
