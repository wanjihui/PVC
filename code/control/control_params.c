#include "control_params.h"
#include <string.h>

typedef struct {
    const char *name;
    float minimum, maximum;
    void *value;
    uint8 kind; /* 0=float; 1=uint8; 2=uint32停车; 3=uint32在线; 4=uint8停车 */
} control_param_t;

static const control_param_t params[] = {
    {"cruise_cms",0,200,&ctl_speed_target,0},
    {"ramp_ms",100,2000,(void *)&ctl_ramp_ms,2},
    {"preview_row",20,100,&ctl_preview_row,1},
    {"deadband_px",0,10,&ctl_deadband,0},
    {"omega_kp1",0,3,&ctl_omega_kp1,0},
    {"omega_kp2",0,0.03f,&ctl_omega_kp2,0},
    {"omega_kd",0,2,&ctl_omega_kd,0},
    {"omega_max_dps",30,360,&ctl_omega_max,0},
    {"gyro_kd",0,1,&ctl_gyro_kd,0},
    {"lateral_gain",0,1,&ctl_lateral_gain,0},
    {"kp_FL",0,0.5f,&speed_kp[0],0}, {"kp_FR",0,0.5f,&speed_kp[1],0},
    {"kp_RL",0,0.5f,&speed_kp[2],0}, {"kp_RR",0,0.5f,&speed_kp[3],0},
    {"ki_FL",0,0.01f,&speed_ki[0],0}, {"ki_FR",0,0.01f,&speed_ki[1],0},
    {"ki_RL",0,0.01f,&speed_ki[2],0}, {"ki_RR",0,0.01f,&speed_ki[3],0},
    {"kd_FL",0,0.1f,&speed_kd[0],0}, {"kd_FR",0,0.1f,&speed_kd[1],0},
    {"kd_RL",0,0.1f,&speed_kd[2],0}, {"kd_RR",0,0.1f,&speed_kd[3],0},
    {"ff",0,0.1f,&speed_ff,0},
    {"curve_enable",0,1,&ctl_curve_enable,4},
    {"curve_normal_cms",5,200,&ctl_curve_normal_cms,0},
    {"curve_tight_cms",5,200,&ctl_curve_tight_cms,0},
    {"curve_enter_dps",1,720,&ctl_curve_enter_dps,0},
    {"curve_tight_dps",1,1440,&ctl_curve_tight_dps,0},
    {"curve_confirm_frames",1,10,&ctl_curve_confirm_frames,1},
    {"curve_exit_dps",0,720,&ctl_curve_exit_dps,0},
    {"curve_exit_err_px",0,100,&ctl_curve_exit_err_px,0},
    {"curve_exit_gyro_dps",0,360,&ctl_curve_exit_gyro_dps,0},
    {"curve_exit_stable_ms",20,2000,(void *)&ctl_curve_exit_stable_ms,3},
    {"curve_frame_max_age_ms",20,500,(void *)&ctl_curve_frame_max_age_ms,3},
    {"curve_decel_cms2",1,2000,&ctl_curve_decel_cms2,0},
    {"curve_accel_cms2",1,1000,&ctl_curve_accel_cms2,0}
};

/* 校验候选组合，不允许紧弯比普通弯快或破坏进入/退出滞回。 */
static uint8 curve_values_valid(const char *name, float value)
{
    float normal = !strcmp(name,"curve_normal_cms") ? value : ctl_curve_normal_cms;
    float tight = !strcmp(name,"curve_tight_cms") ? value : ctl_curve_tight_cms;
    float enter = !strcmp(name,"curve_enter_dps") ? value : ctl_curve_enter_dps;
    float tight_dps = !strcmp(name,"curve_tight_dps") ? value : ctl_curve_tight_dps;
    float exit_dps = !strcmp(name,"curve_exit_dps") ? value : ctl_curve_exit_dps;
    return tight <= normal && exit_dps < enter && enter < tight_dps;
}

uint8 control_param_count(void) { return (uint8)(sizeof(params)/sizeof(params[0])); }
const char *control_param_get(uint8 index, float *value)
{
    uint32 irq;
    const control_param_t *p;
    if (index >= control_param_count() || value == NULL) return NULL;
    p = &params[index];
    irq = interrupt_global_disable();
    if (p->kind == 1 || p->kind == 4) *value = (float)*(uint8 *)p->value;
    else if (p->kind == 2 || p->kind == 3) *value = (float)*(volatile uint32 *)p->value;
    else *value = *(float *)p->value;
    interrupt_global_enable(irq);
    return p->name;
}

uint8 control_param_set(const char *name, float value, const char **error)
{
    uint8 i;
    uint32 irq;
    const control_param_t *p;
    *error = "UNKNOWN_KEY";
    for (i = 0; i < control_param_count(); i++)
    {
        p = &params[i];
        if (strcmp(name,p->name)) continue;
        /* 包含NaN/Inf检查：NaN不会通过两个比较；范围有限。 */
        if (!(value >= p->minimum && value <= p->maximum)) { *error="RANGE"; return 1; }
        if (p->kind && value != (float)(uint32)value) { *error="INTEGER"; return 1; }
        if ((p->kind == 2 || p->kind == 3) && ((uint32)value % SPEED_LOOP_PERIOD_MS)) { *error="STEP_5MS"; return 1; }
        irq = interrupt_global_disable();
        if ((p->kind == 2 || p->kind == 4) && car_state != CAR_IDLE)
        { interrupt_global_enable(irq); *error="STOP_FIRST"; return 1; }
        if (!strncmp(name,"curve_",6) && !curve_values_valid(name,value))
        { interrupt_global_enable(irq); *error="CURVE_ORDER"; return 1; }
        if (p->kind == 1 || p->kind == 4) { *(uint8 *)p->value=(uint8)value; asc_far=ctl_preview_row; }
        else if (p->kind == 2 || p->kind == 3) *(volatile uint32 *)p->value=(uint32)value;
        else *(float *)p->value=value;
        if (!strncmp(name,"curve_",6)) control_curve_params_changed();
        /* 更新增益不清累计输出/误差历史，也不改变K1/K2运行状态。 */
        Speed_PID_Sync_Params();
        interrupt_global_enable(irq);
        *error = "";
        return 0;
    }
    return 1;
}
