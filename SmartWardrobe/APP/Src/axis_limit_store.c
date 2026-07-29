#include "axis_limit_store.h"

#include <stddef.h>

/*
 * Flash 持久化模块：
 * 用于保存 XYZ 限位、限位保护开关、M2 旋转角度、M2 旋转最大步数和三轴规划参数。
 * 所有字段统一保存在 AXIS_LIMIT_STORE_ADDR 指向的 Flash 扇区中，并通过 CRC32 校验完整性。
 */
typedef struct {
    uint32_t magic;
    uint32_t version;
} AxisLimitStoreHeader_t;

/* 旧版本 V2：没有 protect_enabled、M2 角度、M2 最大旋转步数等字段。 */
typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t x_max;
    uint32_t y_max;
    uint32_t z_max;
    uint32_t move_rpm_x10;
    uint32_t enc_err_max;
    uint32_t pos_tol;
    uint32_t crc32;
} AxisLimitStoreBlobV2_t;

/* 旧版本 V3：增加 protect_enabled，但没有 M2 旋转角度和 M2 最大旋转步数字段。 */
typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t x_max;
    uint32_t y_max;
    uint32_t z_max;
    uint32_t protect_enabled;
    uint32_t move_rpm_x10;
    uint32_t enc_err_max;
    uint32_t pos_tol;
    uint32_t crc32;
} AxisLimitStoreBlobV3_t;

/* 旧版本 V4：增加 M2 角度，但没有 M2 最大旋转步数字段。 */
typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t x_max;
    uint32_t y_max;
    uint32_t z_max;
    uint32_t protect_enabled;
    int32_t m3_angle_x100;
    uint32_t move_rpm_x10;
    uint32_t enc_err_max;
    uint32_t pos_tol;
    uint32_t crc32;
} AxisLimitStoreBlobV4_t;

/* 当前版本数据结构，必须和 axis_store_write_blob() 写入大小保持一致。 */
typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t x_max;
    uint32_t y_max;
    uint32_t z_max;
    uint32_t protect_enabled;
    int32_t m3_angle_x100;
    uint32_t m3_rot_max;
    uint32_t move_rpm_x10;
    uint32_t enc_err_max;
    uint32_t pos_tol;
    uint32_t crc32;
} AxisLimitStoreBlob_t;

static uint32_t axis_limit_crc32(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFUL;
    uint32_t i;
    uint32_t b;

    /* 标准 CRC32 多项式 0xEDB88320，用于判断 Flash 中参数是否损坏。 */
    if (data == 0) {
        return 0U;
    }

    for (i = 0U; i < len; i++) {
        crc ^= (uint32_t)data[i];
        for (b = 0U; b < 8U; b++) {
            if ((crc & 1UL) != 0U) {
                crc = (crc >> 1U) ^ 0xEDB88320UL;
            } else {
                crc >>= 1U;
            }
        }
    }

    return crc ^ 0xFFFFFFFFUL;
}

static void axis_store_apply_defaults(AxisLimitStoreBlob_t *blob)
{
    /* 初始化默认数据块。限位值默认为 0，表示还没有完成有效配置。 */
    if (blob == 0) {
        return;
    }

    blob->magic = AXIS_LIMIT_STORE_MAGIC;
    blob->version = AXIS_LIMIT_STORE_VERSION;
    blob->x_max = 0U;
    blob->y_max = 0U;
    blob->z_max = 0U;
    blob->protect_enabled = 0U;
    blob->m3_angle_x100 = 0;
    blob->m3_rot_max = 0U;
    blob->move_rpm_x10 = AXIS_PLAN_DEFAULT_RPM_X10;
    blob->enc_err_max = AXIS_PLAN_DEFAULT_ENC_ERR;
    blob->pos_tol = AXIS_PLAN_DEFAULT_POS_TOL;
    blob->crc32 = 0U;
}

static int axis_store_load_blob(AxisLimitStoreBlob_t *out, uint8_t *legacy_migrated)
{
    const AxisLimitStoreHeader_t *hdr;
    uint32_t crc_calc;
    uint32_t crc_ref;

    /*
     * 从 Flash 中读取配置数据。
     * 如果发现旧版本 V2/V3/V4，则转换成当前结构返回，并通过 legacy_migrated 标记需要写回升级。
     */
    if (out == 0) {
        return -1;
    }

    hdr = (const AxisLimitStoreHeader_t *)AXIS_LIMIT_STORE_ADDR;
    if (hdr->magic != AXIS_LIMIT_STORE_MAGIC) {
        return -2;
    }

    /* 当前版本：直接校验 CRC 后复制完整结构。 */
    if (hdr->version == AXIS_LIMIT_STORE_VERSION) {
        const AxisLimitStoreBlob_t *blob = (const AxisLimitStoreBlob_t *)AXIS_LIMIT_STORE_ADDR;

        crc_ref = blob->crc32;
        crc_calc = axis_limit_crc32((const uint8_t *)blob, (uint32_t)offsetof(AxisLimitStoreBlob_t, crc32));
        if (crc_calc != crc_ref) {
            return -3;
        }

        *out = *blob;
        if (legacy_migrated != 0) {
            *legacy_migrated = 0U;
        }
        return 0;
    }

    /* V4 兼容读取：缺少 m3_rot_max，所以用 0 表示未配置。 */
    if (hdr->version == 4U) {
        const AxisLimitStoreBlobV4_t *blob_v4 = (const AxisLimitStoreBlobV4_t *)AXIS_LIMIT_STORE_ADDR;

        crc_ref = blob_v4->crc32;
        crc_calc = axis_limit_crc32((const uint8_t *)blob_v4, (uint32_t)offsetof(AxisLimitStoreBlobV4_t, crc32));
        if (crc_calc != crc_ref) {
            return -3;
        }

        axis_store_apply_defaults(out);
        out->x_max = blob_v4->x_max;
        out->y_max = blob_v4->y_max;
        out->z_max = blob_v4->z_max;
        out->protect_enabled = (blob_v4->protect_enabled != 0U) ? 1U : 0U;
        out->m3_angle_x100 = blob_v4->m3_angle_x100;
        out->m3_rot_max = 0U;
        out->move_rpm_x10 = blob_v4->move_rpm_x10;
        out->enc_err_max = blob_v4->enc_err_max;
        out->pos_tol = blob_v4->pos_tol;
        if (legacy_migrated != 0) {
            *legacy_migrated = 1U;
        }
        return 0;
    }

    /* V3 兼容读取：缺少 M2 角度和 M2 旋转最大步数。 */
    if (hdr->version == 3U) {
        const AxisLimitStoreBlobV3_t *blob_v3 = (const AxisLimitStoreBlobV3_t *)AXIS_LIMIT_STORE_ADDR;

        crc_ref = blob_v3->crc32;
        crc_calc = axis_limit_crc32((const uint8_t *)blob_v3, (uint32_t)offsetof(AxisLimitStoreBlobV3_t, crc32));
        if (crc_calc != crc_ref) {
            return -3;
        }

        axis_store_apply_defaults(out);
        out->x_max = blob_v3->x_max;
        out->y_max = blob_v3->y_max;
        out->z_max = blob_v3->z_max;
        out->protect_enabled = (blob_v3->protect_enabled != 0U) ? 1U : 0U;
        out->move_rpm_x10 = blob_v3->move_rpm_x10;
        out->enc_err_max = blob_v3->enc_err_max;
        out->pos_tol = blob_v3->pos_tol;
        out->m3_angle_x100 = 0;
        out->m3_rot_max = 0U;
        if (legacy_migrated != 0) {
            *legacy_migrated = 1U;
        }
        return 0;
    }

    /* V2 兼容读取：没有保护开关，迁移后默认保护关闭。 */
    if (hdr->version == 2U) {
        const AxisLimitStoreBlobV2_t *blob_v2 = (const AxisLimitStoreBlobV2_t *)AXIS_LIMIT_STORE_ADDR;

        crc_ref = blob_v2->crc32;
        crc_calc = axis_limit_crc32((const uint8_t *)blob_v2, (uint32_t)offsetof(AxisLimitStoreBlobV2_t, crc32));
        if (crc_calc != crc_ref) {
            return -3;
        }

        axis_store_apply_defaults(out);
        out->x_max = blob_v2->x_max;
        out->y_max = blob_v2->y_max;
        out->z_max = blob_v2->z_max;
        out->move_rpm_x10 = blob_v2->move_rpm_x10;
        out->enc_err_max = blob_v2->enc_err_max;
        out->pos_tol = blob_v2->pos_tol;
        out->protect_enabled = 0U;
        out->m3_angle_x100 = 0;
        out->m3_rot_max = 0U;
        if (legacy_migrated != 0) {
            *legacy_migrated = 1U;
        }
        return 0;
    }

    return -2;
}

static int axis_store_write_blob(AxisLimitStoreBlob_t *blob)
{
    FLASH_EraseInitTypeDef erase_cfg;
    uint32_t erase_error = 0U;
    const uint32_t *words;
    uint32_t i;
    uint32_t addr;

    /*
     * 写 Flash 的完整流程：
     * 1. 补齐 magic/version/protect_enabled/CRC；
     * 2. 解锁 Flash；
     * 3. 擦除整个保存扇区；
     * 4. 按 word 写入当前结构；
     * 5. 重新上锁。
     */
    if (blob == 0) {
        return -1;
    }

    blob->magic = AXIS_LIMIT_STORE_MAGIC;
    blob->version = AXIS_LIMIT_STORE_VERSION;
    blob->protect_enabled = (blob->protect_enabled != 0U) ? 1U : 0U;
    blob->crc32 = axis_limit_crc32((const uint8_t *)blob, (uint32_t)offsetof(AxisLimitStoreBlob_t, crc32));

    if (HAL_FLASH_Unlock() != HAL_OK) {
        return -2;
    }

    erase_cfg.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase_cfg.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    erase_cfg.Sector = AXIS_LIMIT_STORE_SECTOR;
    erase_cfg.NbSectors = 1U;
    if (HAL_FLASHEx_Erase(&erase_cfg, &erase_error) != HAL_OK) {
        (void)HAL_FLASH_Lock();
        return -3;
    }

    words = (const uint32_t *)blob;
    addr = AXIS_LIMIT_STORE_ADDR;
    for (i = 0U; i < (sizeof(AxisLimitStoreBlob_t) / sizeof(uint32_t)); i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, words[i]) != HAL_OK) {
            (void)HAL_FLASH_Lock();
            return -4;
        }
        addr += sizeof(uint32_t);
    }

    if (HAL_FLASH_Lock() != HAL_OK) {
        return -5;
    }

    return 0;
}

int AxisLimitStore_LoadAll(uint32_t *x_max,
                           uint32_t *y_max,
                           uint32_t *z_max,
                           bool *protect_enabled,
                           bool *migrated_legacy)
{
    AxisLimitStoreBlob_t blob;
    uint8_t legacy_migrated = 0U;
    int rc;

    /* 一次性读取 XYZ 限位和保护开关，是 AxisLimit_Init() 的主要入口。 */
    if ((x_max == 0) || (y_max == 0) || (z_max == 0) || (protect_enabled == 0)) {
        return -1;
    }

    rc = axis_store_load_blob(&blob, &legacy_migrated);
    if (rc != 0) {
        return rc;
    }

    *x_max = blob.x_max;
    *y_max = blob.y_max;
    *z_max = blob.z_max;
    *protect_enabled = (blob.protect_enabled != 0U) ? true : false;
    if (migrated_legacy != 0) {
        *migrated_legacy = (legacy_migrated != 0U) ? true : false;
    }
    return 0;
}

int AxisLimitStore_SaveAll(uint32_t x_max, uint32_t y_max, uint32_t z_max, bool protect_enabled)
{
    AxisLimitStoreBlob_t blob;
    bool verify_protect = false;
    uint32_t verify_x = 0U;
    uint32_t verify_y = 0U;
    uint32_t verify_z = 0U;
    int rc;

    /*
     * 保存 XYZ 限位和保护开关。
     * 先尽量读取已有数据，保留 M2 角度、M2 旋转限制和规划参数等其他字段。
     */
    rc = axis_store_load_blob(&blob, 0);
    if (rc != 0) {
        axis_store_apply_defaults(&blob);
    }

    blob.x_max = x_max;
    blob.y_max = y_max;
    blob.z_max = z_max;
    blob.protect_enabled = protect_enabled ? 1U : 0U;

    rc = axis_store_write_blob(&blob);
    if (rc != 0) {
        return rc;
    }

    /* 写入后立即读回校验，避免 Flash 写入失败但上层误以为成功。 */
    rc = AxisLimitStore_LoadAll(&verify_x, &verify_y, &verify_z, &verify_protect, 0);
    if (rc != 0) {
        return -6;
    }
    if ((verify_x != x_max) || (verify_y != y_max) || (verify_z != z_max) ||
        (verify_protect != protect_enabled)) {
        return -7;
    }

    return 0;
}

int AxisLimitStore_Load(uint32_t *x_max, uint32_t *y_max, uint32_t *z_max)
{
    /* 旧接口兼容：只读取 XYZ 限位，不关心保护开关。 */
    bool protect_enabled;
    return AxisLimitStore_LoadAll(x_max, y_max, z_max, &protect_enabled, 0);
}

int AxisLimitStore_Save(uint32_t x_max, uint32_t y_max, uint32_t z_max)
{
    /* 旧接口兼容：保存 XYZ 限位时尽量保留原来的保护开关状态。 */
    bool protect_enabled = false;

    (void)AxisLimitStore_LoadProtect(&protect_enabled);
    return AxisLimitStore_SaveAll(x_max, y_max, z_max, protect_enabled);
}

int AxisLimitStore_LoadProtect(bool *protect_enabled)
{
    bool migrated_legacy;
    uint32_t x_max;
    uint32_t y_max;
    uint32_t z_max;

    /* 单独读取保护开关，内部仍然复用完整数据块读取。 */
    if (protect_enabled == 0) {
        return -1;
    }

    return AxisLimitStore_LoadAll(&x_max, &y_max, &z_max, protect_enabled, &migrated_legacy);
}

int AxisLimitStore_SaveProtect(bool protect_enabled)
{
    bool stored_protect = false;
    uint32_t x_max = 0U;
    uint32_t y_max = 0U;
    uint32_t z_max = 0U;

    /*
     * 单独保存保护开关。
     * 如果是新板子还没有任何 Flash 数据，则保留 0 限位，只写入保护状态。
     */
    if (AxisLimitStore_LoadAll(&x_max, &y_max, &z_max, &stored_protect, 0) != 0) {
        /* 新板子场景：保留 0 限位，只保存保护状态。 */
        x_max = 0U;
        y_max = 0U;
        z_max = 0U;
    }

    return AxisLimitStore_SaveAll(x_max, y_max, z_max, protect_enabled);
}

int AxisLimitStore_LoadM3Angle(int32_t *angle_x100)
{
    AxisLimitStoreBlob_t blob;
    int rc;

    /* 读取 M2 旋转轴保存角度，单位是角度 * 100。 */
    if (angle_x100 == 0) {
        return -1;
    }

    rc = axis_store_load_blob(&blob, 0);
    if (rc != 0) {
        return rc;
    }

    *angle_x100 = blob.m3_angle_x100;
    return 0;
}

int AxisLimitStore_SaveM3Angle(int32_t angle_x100)
{
    AxisLimitStoreBlob_t blob;
    int32_t verify_angle_x100;
    int rc;

    /* 保存 M2 旋转轴角度，保留同一 Flash 数据块里的其他配置项。 */
    rc = axis_store_load_blob(&blob, 0);
    if (rc != 0) {
        axis_store_apply_defaults(&blob);
    }

    blob.m3_angle_x100 = angle_x100;

    rc = axis_store_write_blob(&blob);
    if (rc != 0) {
        return rc;
    }

    /* 写入后读回校验，确认角度字段真正保存成功。 */
    rc = AxisLimitStore_LoadM3Angle(&verify_angle_x100);
    if (rc != 0) {
        return -6;
    }
    if (verify_angle_x100 != angle_x100) {
        return -7;
    }

    return 0;
}

int AxisLimitStore_LoadM3RotMax(uint32_t *rot_max)
{
    AxisLimitStoreBlob_t blob;
    int rc;

    /* 读取 M2 旋转轴最大允许步数，0 表示未配置。 */
    if (rot_max == 0) {
        return -1;
    }

    rc = axis_store_load_blob(&blob, 0);
    if (rc != 0) {
        return rc;
    }

    *rot_max = blob.m3_rot_max;
    return 0;
}

int AxisLimitStore_SaveM3RotMax(uint32_t rot_max)
{
    AxisLimitStoreBlob_t blob;
    uint32_t verify_rot_max;
    int rc;

    /* 保存 M2 旋转轴最大允许步数，保留其他配置项。 */
    rc = axis_store_load_blob(&blob, 0);
    if (rc != 0) {
        axis_store_apply_defaults(&blob);
    }

    blob.m3_rot_max = rot_max;

    rc = axis_store_write_blob(&blob);
    if (rc != 0) {
        return rc;
    }

    /* 写入后读回校验，确认旋转限位字段真正保存成功。 */
    rc = AxisLimitStore_LoadM3RotMax(&verify_rot_max);
    if (rc != 0) {
        return -6;
    }
    if (verify_rot_max != rot_max) {
        return -7;
    }

    return 0;
}

int AxisLimitStore_LoadPlanner(uint32_t *move_rpm_x10, uint32_t *enc_err_max, uint32_t *pos_tol)
{
    AxisLimitStoreBlob_t blob;
    int rc;

    /* 读取三轴规划参数，rpm 使用 rpm * 10 的整数形式保存。 */
    if ((move_rpm_x10 == 0) || (enc_err_max == 0) || (pos_tol == 0)) {
        return -1;
    }

    rc = axis_store_load_blob(&blob, 0);
    if (rc != 0) {
        return rc;
    }

    *move_rpm_x10 = blob.move_rpm_x10;
    *enc_err_max = blob.enc_err_max;
    *pos_tol = blob.pos_tol;
    return 0;
}

int AxisLimitStore_SavePlanner(uint32_t move_rpm_x10, uint32_t enc_err_max, uint32_t pos_tol)
{
    AxisLimitStoreBlob_t blob;
    uint32_t verify_rpm_x10;
    uint32_t verify_enc_err;
    uint32_t verify_pos_tol;
    int rc;

    /* 保存三轴规划参数，保留限位和 M2 相关字段。 */
    rc = axis_store_load_blob(&blob, 0);
    if (rc != 0) {
        axis_store_apply_defaults(&blob);
    }

    blob.move_rpm_x10 = move_rpm_x10;
    blob.enc_err_max = enc_err_max;
    blob.pos_tol = pos_tol;

    rc = axis_store_write_blob(&blob);
    if (rc != 0) {
        return rc;
    }

    /* 写入后读回校验，确认规划参数真正保存成功。 */
    rc = AxisLimitStore_LoadPlanner(&verify_rpm_x10, &verify_enc_err, &verify_pos_tol);
    if (rc != 0) {
        return -6;
    }
    if ((verify_rpm_x10 != move_rpm_x10) || (verify_enc_err != enc_err_max) || (verify_pos_tol != pos_tol)) {
        return -7;
    }

    return 0;
}
