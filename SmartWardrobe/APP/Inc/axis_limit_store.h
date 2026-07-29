#ifndef AXIS_LIMIT_STORE_H
#define AXIS_LIMIT_STORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "main.h"

/* Flash 数据块魔数和当前版本号，用于判断保存区是否属于本项目配置。 */
#define AXIS_LIMIT_STORE_MAGIC 0x41584C4DU
#define AXIS_LIMIT_STORE_VERSION 5U

/* STM32F407VETx 512KB Flash：sector 7 范围为 0x08060000~0x0807FFFF。 */
#define AXIS_LIMIT_STORE_ADDR 0x08060000UL
#define AXIS_LIMIT_STORE_SECTOR FLASH_SECTOR_7

/* 三轴规划参数默认值，rpm 使用 rpm * 10 保存。 */
#define AXIS_PLAN_DEFAULT_RPM_X10 1200U
#define AXIS_PLAN_DEFAULT_ENC_ERR 300U
#define AXIS_PLAN_DEFAULT_POS_TOL 20U

/* 旧接口：只加载/保存 XYZ 限位。 */
int AxisLimitStore_Load(uint32_t *x_max, uint32_t *y_max, uint32_t *z_max);
int AxisLimitStore_Save(uint32_t x_max, uint32_t y_max, uint32_t z_max);
/* 当前完整接口：加载/保存 XYZ 限位和保护开关，并能标记旧版本迁移。 */
int AxisLimitStore_LoadAll(uint32_t *x_max,
                           uint32_t *y_max,
                           uint32_t *z_max,
                           bool *protect_enabled,
                           bool *migrated_legacy);
int AxisLimitStore_SaveAll(uint32_t x_max, uint32_t y_max, uint32_t z_max, bool protect_enabled);
/* 单独读写保护开关。 */
int AxisLimitStore_LoadProtect(bool *protect_enabled);
int AxisLimitStore_SaveProtect(bool protect_enabled);
/* 读写 M2 旋转角度，单位为角度 * 100。 */
int AxisLimitStore_LoadM3Angle(int32_t *angle_x100);
int AxisLimitStore_SaveM3Angle(int32_t angle_x100);
/* 读写 M2 旋转轴最大允许步数。 */
int AxisLimitStore_LoadM3RotMax(uint32_t *rot_max);
int AxisLimitStore_SaveM3RotMax(uint32_t rot_max);
/* 读写三轴规划参数。 */
int AxisLimitStore_LoadPlanner(uint32_t *move_rpm_x10, uint32_t *enc_err_max, uint32_t *pos_tol);
int AxisLimitStore_SavePlanner(uint32_t move_rpm_x10, uint32_t enc_err_max, uint32_t pos_tol);

#ifdef __cplusplus
}
#endif

#endif
