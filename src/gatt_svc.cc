#include "rel_common.h"
#include "nimble_handler.h"
#include "gatt_svc.h"
#include "model_inference.h"

static const char* TAG = "BLE_GATT";

// pipeline service
static const ble_uuid128_t pipeline_svc_uuid = BLE_UUID128_INIT(0x4f, 0xaf, 0xc2, 0x01, 0x1f, 0xb5, 0x45, 0x9e, 
    0x8f, 0xcc, 0xc5, 0xc9, 0xc3, 0x31, 0x91, 0x4b);
static const ble_uuid128_t pipeline_chr_uuid = BLE_UUID128_INIT(0xbe, 0xb5, 0x48, 0x3e, 0x36, 0xe1, 0x46, 0x88, 
    0xb7, 0xf5, 0xea, 0x07, 0x36, 0x1b, 0x26, 0xa8);
static uint16_t pipeline_chr_val_handle;

static int pipeline_chr_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int send_notification(uint16_t conn_handle, uint16_t handle, uint8_t* data, uint16_t length);
void gatt_svr_subscribe_cb(struct ble_gap_event *event);

// gatt service table
static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &pipeline_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                .uuid = &pipeline_chr_uuid.u,
                .access_cb = pipeline_chr_access,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &pipeline_chr_val_handle
            },
            {0}
        },
    },
    {
        0,
    },
};

/*
 *  GATT server initialization
 *      1. Initialize GATT service
 *      2. Update NimBLE host GATT services counter
 *      3. Add GATT services to server
 */
int gatt_svc_init(void) {
    /* Local variables */
    int rc;

    /* 1. GATT service initialization */
    ble_svc_gatt_init();

    /* 2. Update GATT services counter */
    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    /* 3. Add GATT services */
    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    return 0;
}

static int pipeline_chr_access(uint16_t conn_handle, uint16_t attr_handle,
    struct ble_gatt_access_ctxt *ctxt, void *arg) {
    int rc;
    uint8_t command[1];
    uint16_t len;

    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_WRITE_CHR:
            len = OS_MBUF_PKTLEN(ctxt->om);
            if (len != 1) {
                ESP_LOGE(TAG, "유효하지 않은 명령 길이: %d", len);
                return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            }

            rc = ble_hs_mbuf_to_flat(ctxt->om, command, sizeof(command), &len);
            if (rc != 0) {
                ESP_LOGE(TAG, "mbuf에서 데이터 복사 실패");
                return BLE_ATT_ERR_UNLIKELY;
            }

            ESP_LOGI(TAG, "받은 명령: %c", command[0]);

            if (command[0] == 'r') {
                ESP_LOGI(TAG, "'r' 명령 수신, 처리 중...");
                vTaskDelay(pdMS_TO_TICKS(100));
                rc = send_notification(conn_handle, pipeline_chr_val_handle, (uint8_t*)"w", strlen("w"));
                vTaskDelay(pdMS_TO_TICKS(100));
                const char* result = pipeline();
                rc = send_notification(conn_handle, pipeline_chr_val_handle, (uint8_t*)result, strlen(result));
                vTaskDelay(pdMS_TO_TICKS(100));
                rc = send_notification(conn_handle, pipeline_chr_val_handle, (uint8_t*)"EOF", strlen("EOF"));
                vTaskDelay(pdMS_TO_TICKS(100));
                if (rc != 0) {
                    ESP_LOGE(TAG, "파이프라인 결과 전송 실패: %d", rc);
                    return BLE_ATT_ERR_UNLIKELY;
                }
            } else {
                ESP_LOGE(TAG, "알 수 없는 명령: %c", command[0]);
                return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            }
            return 0;

        case BLE_GATT_ACCESS_OP_READ_CHR:
            // 읽기 작업 시 기본 응답 전송
            rc = os_mbuf_append(ctxt->om, "Ready", 5);
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;

        default:
            ESP_LOGE(TAG, "예상치 못한 접근 작업: %d", ctxt->op);
            return BLE_ATT_ERR_UNLIKELY;
    }
}


void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg) {
    char buf[BLE_UUID_STR_LEN];
    ESP_LOGI(TAG, "GATT 서비스 등록 요청, 작업: %d", ctxt->op);

    switch (ctxt->op) {
        case BLE_GATT_REGISTER_OP_SVC:
            ESP_LOGI(TAG, "서비스 등록: %s, 핸들=%d",
                     ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
                     ctxt->svc.handle);
            break;
        case BLE_GATT_REGISTER_OP_CHR:
            ESP_LOGI(TAG, "특성 등록: %s, def_handle=%d, val_handle=%d",
                     ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
                     ctxt->chr.def_handle, ctxt->chr.val_handle);
            break;
        case BLE_GATT_REGISTER_OP_DSC:
            ESP_LOGI(TAG, "디스크립터 등록: %s, 핸들=%d",
                     ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf),
                     ctxt->dsc.handle);
            break;
        default:
            ESP_LOGW(TAG, "알 수 없는 등록 작업: %d", ctxt->op);
            break;
    }
}

static int send_notification(uint16_t conn_handle, uint16_t handle, uint8_t* data, uint16_t length)
{
    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, length);
    if (!om) {
        return BLE_HS_ENOMEM;
    }

    int rc = ble_gatts_notify_custom(conn_handle, handle, om);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to send notification: %d", rc);
    }

    return rc;
}

void gatt_svr_subscribe_cb(struct ble_gap_event *event) {
    /* Check connection handle */
    if (event->subscribe.conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        ESP_LOGI(TAG, "subscribe event; conn_handle=%d attr_handle=%d",
                 event->subscribe.conn_handle, event->subscribe.attr_handle);
    } else {
        ESP_LOGI(TAG, "subscribe by nimble stack; attr_handle=%d",
                 event->subscribe.attr_handle);
    }
}
