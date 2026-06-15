#include <stdio.h>
#include "ble_peer.h"
#include "bsp/device.h"
#include "bsp/display.h"
#include "bsp/input.h"
#include "bsp/led.h"
#include "bsp/power.h"
#include "console/console.h"
#include "custom_certificates.h"
#include "driver/gpio.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_types.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "hal/lcd_types.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs_flash.h"
#include "pax_fonts.h"
#include "pax_gfx.h"
#include "pax_text.h"
#include "portmacro.h"
#include "services/gap/ble_svc_gap.h"
#include "wifi_connection.h"
#include "wifi_remote.h"
#include "esp_hosted.h"

#define BLACK 0xFF000000
#define WHITE 0xFFFFFFFF
#define RED   0xFFFF0000

#define BLECENT_SVC_CATPRINTER_UUID_ADVERTISED 0xaf30
#define BLECENT_SVC_CATPRINTER_UUID            0xae30
#define BLECENT_CHR_TX_UUID                    0xae01
#define BLECENT_CHR_RX_UUID                    0xae02

// Constants
static char const TAG[] = "main";

// Global variables
static size_t                       display_h_res        = 0;
static size_t                       display_v_res        = 0;
static lcd_color_rgb_pixel_format_t display_color_format = LCD_COLOR_PIXEL_FORMAT_RGB565;
static lcd_rgb_data_endian_t        display_data_endian  = LCD_RGB_DATA_ENDIAN_LITTLE;
static pax_buf_t                    fb                   = {0};
static QueueHandle_t                input_event_queue    = NULL;

static void blecent_on_disc_complete(const struct peer* peer, int status, void* arg);

static void blecent_scan(void);
void        ble_store_config_init(void);

static int  check_if_catprinter(const struct ble_gap_disc_desc* disc);
static void connect_to_device(const ble_addr_t* addr);
static int  blecent_subscribe(uint16_t conn_handle);

static void write_task(void* arg);

static void blit(void) {
    bsp_display_blit(0, 0, display_h_res, display_v_res, pax_buf_get_pixels(&fb));
}

static void blecent_on_reset(int reason) {
    ESP_LOGE(TAG, "BLE resetting state: reason=%d\n", reason);
}

void print_uuid(const ble_uuid_t* uuid) {
    char buf[BLE_UUID_STR_LEN];
    ESP_LOGI(TAG, "%s", ble_uuid_to_str(uuid, buf));
}

void print_bytes(const uint8_t* bytes, int len) {
    int i;
    for (i = 0; i < len; i++) {
        printf("%s0x%02x", i != 0 ? ":" : "", bytes[i]);
    }
    printf("\r\n");
}

char* addr_str(const void* addr) {
    static char    buf[6 * 2 + 5 + 1];
    const uint8_t* u8p;
    u8p = addr;
    sprintf(buf, "%02x:%02x:%02x:%02x:%02x:%02x", u8p[5], u8p[4], u8p[3], u8p[2], u8p[1], u8p[0]);
    return buf;
}

void print_conn_desc(const struct ble_gap_conn_desc* desc) {
    printf("handle=%d our_ota_addr_type=%d our_ota_addr=%s ", desc->conn_handle, desc->our_ota_addr.type,
           addr_str(desc->our_ota_addr.val));
    printf("our_id_addr_type=%d our_id_addr=%s ", desc->our_id_addr.type, addr_str(desc->our_id_addr.val));
    printf("peer_ota_addr_type=%d peer_ota_addr=%s ", desc->peer_ota_addr.type, addr_str(desc->peer_ota_addr.val));
    printf("peer_id_addr_type=%d peer_id_addr=%s ", desc->peer_id_addr.type, addr_str(desc->peer_id_addr.val));
    printf(
        "conn_itvl=%d conn_latency=%d supervision_timeout=%d "
        "encrypted=%d authenticated=%d bonded=%d",
        desc->conn_itvl, desc->conn_latency, desc->supervision_timeout, desc->sec_state.encrypted,
        desc->sec_state.authenticated, desc->sec_state.bonded);
    printf("\r\n");
}

void print_mbuf(const struct os_mbuf* om) {
    int colon, i;

    colon = 0;
    while (om != NULL) {
        if (colon) {
            printf(":");
        } else {
            colon = 1;
        }
        for (i = 0; i < om->om_len; i++) {
            printf("%s0x%02x", i != 0 ? ":" : "", om->om_data[i]);
        }
        om = SLIST_NEXT(om, om_next);
    }
    printf("\r\n");
}

////////

static uint16_t write_conn_handle = 0;

static int blecent_gap_event(struct ble_gap_event* event, void* arg) {
    struct ble_gap_conn_desc desc;
    struct ble_hs_adv_fields fields;
    int                      rc;

    switch (event->type) {
        case BLE_GAP_EVENT_DISC:
            // New device discovered
            rc = ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data);
            if (rc != 0) return 0;
            if (check_if_catprinter(&event->disc)) {
                ESP_LOGI(TAG, "Found cat printer!  Connecting...");
                connect_to_device(&event->disc.addr);
            } else {
                ESP_LOGI(TAG, "Found other BLE device.");
            }
            return 0;
        case BLE_GAP_EVENT_CONNECT:
            // A new connection was established or a connection attempt failed
            if (event->connect.status == 0) {
                // Connection successfully established
                rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
                if (rc != 0) return 0;
                ESP_LOGI(TAG, "Connected to BLE device %s", addr_str(desc.peer_ota_addr.val));
                // Remember peer
                rc = peer_add(event->connect.conn_handle);
                if (rc != 0) {
                    ESP_LOGE(TAG, "Failed to add peer; rc=%d\n", rc);
                    return 0;
                }
                // Set MTU to 23 bytes
                rc = ble_att_set_preferred_mtu(23);
                if (rc != 0) {
                    ESP_LOGE(TAG, "Failed to set preferred MTU; rc = %d", rc);
                }
                // Negotiate MTU
                rc = ble_gattc_exchange_mtu(event->connect.conn_handle, NULL, NULL);
                if (rc != 0) {
                    ESP_LOGE(TAG, "Failed to negotiate MTU; rc = %d", rc);
                }
                // Perform service discovery
                rc = peer_disc_all(event->connect.conn_handle, blecent_on_disc_complete, NULL);
                if (rc != 0) {
                    ESP_LOGE(TAG, "Failed to discover services; rc=%d\n", rc);
                    return 0;
                }
            } else {
                // Connection attempt failed
                ESP_LOGE(TAG, "Error: Connection failed; status=%d\n", event->connect.status);
            }
            return 0;
        case BLE_GAP_EVENT_DISCONNECT:
            // Connection terminated
            ESP_LOGI(TAG, "Disconnected from BLE device %s (reason=%d)", addr_str(desc.peer_ota_addr.val),
                     event->disconnect.reason);
            // Forget about peer
            peer_delete(event->disconnect.conn.conn_handle);
            return 0;
        case BLE_GAP_EVENT_DISC_COMPLETE:
            ESP_LOGI(TAG, "Discovery complete; reason=%d\n", event->disc_complete.reason);
            return 0;
        case BLE_GAP_EVENT_ENC_CHANGE:
            rc = ble_gap_conn_find(event->enc_change.conn_handle, &desc);
            if (rc != 0) return rc;
            ESP_LOGI(TAG, "BLE encryption change event for device %s (status=%d)", addr_str(desc.peer_ota_addr.val),
                     event->enc_change.status);
            return 0;
        case BLE_GAP_EVENT_NOTIFY_RX:
            // Peer sent us a notification or indication
            ESP_LOGI(TAG, "BLE received %s (conn_handle=%d attr_handle=%d attr_len=%d)\n",
                     event->notify_rx.indication ? "indication" : "notification", event->notify_rx.conn_handle,
                     event->notify_rx.attr_handle, OS_MBUF_PKTLEN(event->notify_rx.om));
            /* Attribute data is contained in event->notify_rx.om. Use
             * `os_mbuf_copydata` to copy the data received in notification mbuf */

            uint8_t  notif_data[100];
            uint16_t notif_len;
            int      offset = 0;
            notif_len       = OS_MBUF_PKTLEN(event->notify_rx.om);
            if (notif_len > sizeof(notif_data)) {
                notif_len = sizeof(notif_data);
            }
            os_mbuf_copydata(event->notify_rx.om, offset, notif_len, notif_data);

            printf("Notification data: ");
            for (uint8_t i = 0; i < notif_len; i++) {
                printf("0x%02x ", notif_data[i]);
            }
            printf("\n");

            return 0;
        case BLE_GAP_EVENT_MTU:
            ESP_LOGI(TAG, "mtu update event; conn_handle=%d cid=%d mtu=%d\n", event->mtu.conn_handle,
                     event->mtu.channel_id, event->mtu.value);
            struct peer* peer = peer_find(event->mtu.conn_handle);
            if (peer != NULL) {
                peer->mtu = event->mtu.value;
            } else {
                ESP_LOGE(TAG, "No peer found for connection, can't set MTU");
            }
            return 0;
        case BLE_GAP_EVENT_REPEAT_PAIRING:
            /* We already have a bond with the peer, but it is attempting to
             * establish a new secure link.  This app sacrifices security for
             * convenience: just throw away the old bond and accept the new link.
             */
            // Delete the old bond
            rc = ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc);
            if (rc != 0) return rc;
            ble_store_util_delete_peer(&desc.peer_id_addr);
            return BLE_GAP_REPEAT_PAIRING_RETRY;
        default:
            return 0;
    }
}

static int blecent_on_subscribe(uint16_t conn_handle, const struct ble_gatt_error* error, struct ble_gatt_attr* attr,
                                void* arg) {
    ESP_LOGI(TAG, "Subscribe complete; status=%d conn_handle=%d attr_handle=%d\n",
             error->status, conn_handle, attr->handle);

    write_conn_handle = conn_handle;
    xTaskCreate(write_task, "ble_write", 4096, NULL, 5, NULL);

    return 0;
}

static int blecent_subscribe(uint16_t conn_handle) {
    const struct peer_dsc* dsc;
    uint8_t                value[2];
    int                    rc;
    const struct peer*     peer = peer_find(conn_handle);

    dsc = peer_dsc_find_uuid(peer, BLE_UUID16_DECLARE(BLECENT_SVC_CATPRINTER_UUID),
                             BLE_UUID16_DECLARE(BLECENT_CHR_RX_UUID), BLE_UUID16_DECLARE(BLE_GATT_DSC_CLT_CFG_UUID16));
    if (dsc == NULL) {
        ESP_LOGE(TAG, "Error: Peer lacks a CCCD for the RX characteristic\n");
        goto err;
    }

    value[0] = 1;
    value[1] = 0;
    rc       = ble_gattc_write_flat(conn_handle, dsc->dsc.handle, value, sizeof value, blecent_on_subscribe, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG,
                 "Error: Failed to subscribe to characteristic; "
                 "rc=%d\n",
                 rc);
        goto err;
    }

    return 0;
err:
    /* Terminate the connection. */
    return ble_gap_terminate(peer->conn_handle, BLE_ERR_REM_USER_CONN_TERM);
}

static void ble_write_raw(uint16_t conn_handle, uint16_t val_handle, const uint8_t* data, size_t len) {
    const struct peer* peer      = peer_find(conn_handle);
    uint8_t            max_chunk = (peer && peer->mtu > 3) ? peer->mtu - 3 : 20;
    size_t             pos       = 0;
    while (pos < len) {
        size_t chunk = len - pos;
        if (chunk > max_chunk) chunk = max_chunk;
        int rc = ble_gattc_write_no_rsp_flat(conn_handle, val_handle, &data[pos], chunk);
        if (rc == BLE_HS_ENOMEM) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        if (rc != 0) {
            ESP_LOGE(TAG, "BLE write failed: rc=%d", rc);
            return;
        }
        pos += chunk;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static uint8_t crc8(const uint8_t* data, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
        }
    }
    return crc;
}

static void write_packet(uint16_t conn_handle, uint16_t val_handle, uint8_t cmd, const uint8_t* payload, uint8_t payload_len) {
    // Packet: 51 78 <cmd> 00 <len_lo> <len_hi> <payload...> <crc8> ff
    uint8_t buf[8 + payload_len];
    buf[0] = 0x51;
    buf[1] = 0x78;
    buf[2] = cmd;
    buf[3] = 0x00;
    buf[4] = payload_len;
    buf[5] = 0x00;
    memcpy(&buf[6], payload, payload_len);
    buf[6 + payload_len] = crc8(payload, payload_len);
    buf[7 + payload_len] = 0xff;
    ble_write_raw(conn_handle, val_handle, buf, sizeof(buf));
}

// cmd 0xa3 — query device status (CMD_GET_DEV_STATE)
static void write_get_device_state(uint16_t conn_handle, uint16_t val_handle) {
    uint8_t payload[] = {0x00};
    write_packet(conn_handle, val_handle, 0xa3, payload, sizeof(payload));
}

// cmd 0xa4 — set print quality / 200 DPI mode (CMD_SET_QUALITY_200_DPI, default 0x32)
static void write_set_quality(uint16_t conn_handle, uint16_t val_handle, uint8_t quality) {
    write_packet(conn_handle, val_handle, 0xa4, &quality, 1);
}

// cmd 0xa8 — query device info (CMD_GET_DEV_INFO)
static void write_get_device_info(uint16_t conn_handle, uint16_t val_handle) {
    uint8_t payload[] = {0x00};
    write_packet(conn_handle, val_handle, 0xa8, payload, sizeof(payload));
}

// cmd 0xaf — set print energy as 16-bit little-endian value (cmd_set_energy in reference)
static void write_set_energy(uint16_t conn_handle, uint16_t val_handle, uint16_t energy) {
    uint8_t payload[] = {energy & 0xff, energy >> 8};
    write_packet(conn_handle, val_handle, 0xaf, payload, sizeof(payload));
}

// cmd 0xbe — set print mode: 0x00 = image (CMD_PRINT_IMG), 0x01 = text (CMD_PRINT_TEXT)
static void write_set_print_mode(uint16_t conn_handle, uint16_t val_handle, uint8_t mode) {
    write_packet(conn_handle, val_handle, 0xbe, &mode, 1);
}

static void write_print_img_mode(uint16_t conn_handle, uint16_t val_handle) {
    write_set_print_mode(conn_handle, val_handle, 0x00);
}

static void write_print_text_mode(uint16_t conn_handle, uint16_t val_handle) {
    write_set_print_mode(conn_handle, val_handle, 0x01);
}

// cmd 0xa6 — draw lattice border line (11-byte pattern)
static void write_draw_lattice(uint16_t conn_handle, uint16_t val_handle, const uint8_t data[11]) {
    write_packet(conn_handle, val_handle, 0xa6, data, 11);
}

// Convenience wrappers for the two fixed lattice patterns used before/after an image
static void write_lattice_start(uint16_t conn_handle, uint16_t val_handle) {
    static const uint8_t data[] = {0xaa, 0x55, 0x17, 0x38, 0x44, 0x5f, 0x5f, 0x5f, 0x44, 0x38, 0x2c};
    write_draw_lattice(conn_handle, val_handle, data);
}

static void write_lattice_end(uint16_t conn_handle, uint16_t val_handle) {
    static const uint8_t data[] = {0xaa, 0x55, 0x17, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17};
    write_draw_lattice(conn_handle, val_handle, data);
}

// cmd 0xbd — feed paper to tear-off position (cmd_feed_paper in reference, default 0x19 = 25 lines)
static void write_feed_to_tear(uint16_t conn_handle, uint16_t val_handle, uint8_t lines) {
    write_packet(conn_handle, val_handle, 0xbd, &lines, 1);
}

// cmd 0xa1 — set paper / feed N lines (16-bit little-endian, default 0x0030 = 48 lines)
static void write_feed_paper(uint16_t conn_handle, uint16_t val_handle, uint16_t lines) {
    uint8_t payload[] = {lines & 0xff, lines >> 8};
    write_packet(conn_handle, val_handle, 0xa1, payload, sizeof(payload));
}

// cmd 0xbf — print one RLE-compressed bitmap line (384 pixels, alternating white/black runs)
static void write_print_rle_line(uint16_t conn_handle, uint16_t val_handle, const uint8_t* rle_data, uint8_t len) {
    write_packet(conn_handle, val_handle, 0xbf, rle_data, len);
}

// cmd 0xa2 — print one raw bitmap line (exactly 48 bytes = 384 pixels, MSB first, 1=black)
static void write_print_raw_line(uint16_t conn_handle, uint16_t val_handle, const uint8_t bitmap[48]) {
    write_packet(conn_handle, val_handle, 0xa2, bitmap, 48);
}


static uint8_t rle_encode_line(const uint8_t bitmap[48], uint8_t out[128]) {
    uint8_t len = 0;
    int     px  = 0;
    while (px < 384) {
        int color = (bitmap[px / 8] >> (px % 8)) & 1;
        int count = 0;
        while (px < 384 && count < 127) {
            if (((bitmap[px / 8] >> (px % 8)) & 1) != color) break;
            count++;
            px++;
        }
        out[len++] = color ? (uint8_t)count : (uint8_t)(0x80 | count);
    }
    return len;
}

uint8_t image_data[256][48] = {0};

static void write_task(void* arg) {
    uint16_t           conn_handle = write_conn_handle;
    const struct peer* peer        = peer_find(conn_handle);
    if (peer == NULL) {
        ESP_LOGE(TAG, "Write task: no peer found");
        vTaskDelete(NULL);
        return;
    }

    const struct peer_chr* chr = peer_chr_find_uuid(peer, BLE_UUID16_DECLARE(BLECENT_SVC_CATPRINTER_UUID),
                                                    BLE_UUID16_DECLARE(BLECENT_CHR_TX_UUID));
    if (chr == NULL) {
        ESP_LOGE(TAG, "Write task: TX characteristic not found");
        vTaskDelete(NULL);
        return;
    }

    uint16_t val_handle = chr->chr.val_handle;

    write_get_device_state(conn_handle, val_handle);
    write_set_quality(conn_handle, val_handle, 0x32);
    write_set_energy(conn_handle, val_handle, 0xffff);
    write_set_print_mode(conn_handle, val_handle, 0x01);
    write_lattice_start(conn_handle, val_handle);
    for (int row = 0; row < 256; row++) {
        uint8_t rle_buf[128];
        uint8_t rle_len = rle_encode_line(image_data[row], rle_buf);
        write_print_rle_line(conn_handle, val_handle, rle_buf, rle_len);
    }
    write_feed_to_tear(conn_handle, val_handle, 0x19);
    write_feed_paper(conn_handle, val_handle, 0x0030);
    write_feed_paper(conn_handle, val_handle, 0x0030);
    write_feed_paper(conn_handle, val_handle, 0x0030);
    write_lattice_end(conn_handle, val_handle);
    write_get_device_state(conn_handle, val_handle);

    vTaskDelete(NULL);
}

static void blecent_on_disc_complete(const struct peer* peer, int status, void* arg) {
    if (status != 0) {
        /* Service discovery failed.  Terminate the connection. */
        ESP_LOGE(TAG,
                 "Error: Service discovery failed; status=%d "
                 "conn_handle=%d\n",
                 status, peer->conn_handle);
        ble_gap_terminate(peer->conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        return;
    }

    /* Service discovery has completed successfully.  Now we have a complete
     * list of services, characteristics, and descriptors that the peer
     * supports.
     */
    ESP_LOGI(TAG,
             "Service discovery complete; status=%d "
             "conn_handle=%d\n",
             status, peer->conn_handle);

    peer_list_all(peer);

    // Ready to perform read, write, and subscribe operations

    // Subscribe to RX characteristic
    int rc = blecent_subscribe(peer->conn_handle);
    if (rc != 0) return;

    // blecent_read_write_subscribe(peer);
}

#define CONFIG_EXAMPLE_PEER_ADDR "ADDR_ANY"

static int check_if_catprinter(const struct ble_gap_disc_desc* disc) {
    struct ble_hs_adv_fields fields;
    int                      rc;
    int                      i;
#if CONFIG_EXAMPLE_USE_CI_ADDRESS
    uint32_t* addr_offset;
#endif  // CONFIG_EXAMPLE_USE_CI_ADDRESS
    uint8_t  test_addr[6];
    uint32_t peer_addr[6];

    memset(peer_addr, 0x0, sizeof peer_addr);

    /* The device has to be advertising connectability. */
    if (disc->event_type != BLE_HCI_ADV_RPT_EVTYPE_ADV_IND && disc->event_type != BLE_HCI_ADV_RPT_EVTYPE_DIR_IND) {
        return 0;
    }

    rc = ble_hs_adv_parse_fields(&fields, disc->data, disc->length_data);
    if (rc != 0) {
        return 0;
    }

    if (strlen(CONFIG_EXAMPLE_PEER_ADDR) && (strncmp(CONFIG_EXAMPLE_PEER_ADDR, "ADDR_ANY", strlen("ADDR_ANY")) != 0)) {
        ESP_LOGI(TAG, "Peer address from menuconfig: %s", CONFIG_EXAMPLE_PEER_ADDR);
        sscanf(CONFIG_EXAMPLE_PEER_ADDR, "%lx:%lx:%lx:%lx:%lx:%lx", &peer_addr[5], &peer_addr[4], &peer_addr[3],
               &peer_addr[2], &peer_addr[1], &peer_addr[0]);
        printf("peer-->  %lx %lx %lx %lx %lx %lx \n", peer_addr[5], peer_addr[4], peer_addr[3], peer_addr[2],
               peer_addr[1], peer_addr[0]);
        /* Conversion */
        for (int i = 0; i < 6; i++) {
            test_addr[i] = (uint8_t)peer_addr[i];
        }
        if (memcmp(test_addr, disc->addr.val, sizeof(disc->addr.val)) != 0) {
            printf("Address does not match\n");
            return 0;
        }
    }

    for (i = 0; i < fields.num_uuids16; i++) {
        // Search for the catprinter service UUID, for some reason the advertised UUID doesn't match the actual UUID so
        // we check for both
        if (ble_uuid_u16(&fields.uuids16[i].u) == BLECENT_SVC_CATPRINTER_UUID ||
            ble_uuid_u16(&fields.uuids16[i].u) == BLECENT_SVC_CATPRINTER_UUID_ADVERTISED) {
            return 1;
        }
    }

    return 0;
}

static void connect_to_device(const ble_addr_t* addr) {
    uint8_t own_addr_type;
    int     rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error determining address type; rc=%d\n", rc);
        return;
    }

    rc = ble_gap_disc_cancel();
    if (rc != 0) {
        MODLOG_DFLT(DEBUG, "Failed to cancel scan; rc=%d\n", rc);
        return;
    }

    rc = ble_gap_connect(own_addr_type, addr, 30000, NULL, blecent_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG,
                 "Error: Failed to connect to BLE device; addr_type=%d "
                 "addr=%s; rc=%d\n",
                 addr->type, addr_str(addr->val), rc);
        return;
    }
}

static void blecent_scan(void) {
    uint8_t                    own_addr_type;
    struct ble_gap_disc_params disc_params = {0};
    int                        rc;

    /* Figure out address to use while advertising (no privacy for now) */
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "error determining address type; rc=%d\n", rc);
        return;
    }

    /* Tell the controller to filter duplicates; we don't want to process
     * repeated advertisements from the same device.
     */
    disc_params.filter_duplicates = 1;

    /**
     * Perform a passive scan.  I.e., don't send follow-up scan requests to
     * each advertiser.
     */
    disc_params.passive = 1;

    /* Use defaults for the rest of the parameters. */
    disc_params.itvl          = 0;
    disc_params.window        = 0;
    disc_params.filter_policy = 0;
    disc_params.limited       = 0;

    rc = ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &disc_params, blecent_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error initiating GAP discovery procedure; rc=%d\n", rc);
    }
}

static void blecent_on_sync(void) {
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "BLE util ensure addr failed: rc=%d\n", rc);
        return;
    }
    ESP_LOGI(TAG, "BLE scanning...");
    blecent_scan();
}

void blecent_host_task(void* param) {
    ESP_LOGI(TAG, "BLE Host Task Started");
    /* This function will return only when nimble_port_stop() is executed */
    nimble_port_run();

    nimble_port_freertos_deinit();
}

void app_main(void) {
    // Start the GPIO interrupt service
    gpio_install_isr_service(0);

    // Initialize the Non Volatile Storage partition
    esp_err_t res = nvs_flash_init();
    if (res == ESP_ERR_NVS_NO_FREE_PAGES || res == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        res = nvs_flash_erase();
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "Failed to erase NVS flash: %d", res);
            return;
        }
        res = nvs_flash_init();
    }
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize NVS flash: %d", res);
        return;
    }

    // Initialize the Board Support Package
    const bsp_configuration_t bsp_configuration = {
        .display =
            {
                .requested_color_format = LCD_COLOR_PIXEL_FORMAT_RGB888,
                .num_fbs                = 1,
            },
    };
    res = bsp_device_initialize(&bsp_configuration);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BSP: %d", res);
        return;
    }

    // Get display parameters and rotation
    res = bsp_display_get_parameters(&display_h_res, &display_v_res, &display_color_format, &display_data_endian);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get display parameters: %d", res);
        return;
    }

    bsp_display_rotation_t display_rotation = bsp_display_get_default_rotation();

    // Convert ESP-IDF color format into PAX buffer type
    pax_buf_type_t format = PAX_BUF_24_888RGB;
    switch (display_color_format) {
        case LCD_COLOR_PIXEL_FORMAT_RGB565:
            format = PAX_BUF_16_565RGB;
            break;
        case LCD_COLOR_PIXEL_FORMAT_RGB888:
            format = PAX_BUF_24_888RGB;
            break;
        default:
            break;
    }

    // Convert BSP display rotation format into PAX orientation type
    pax_orientation_t orientation = PAX_O_UPRIGHT;
    switch (display_rotation) {
        case BSP_DISPLAY_ROTATION_90:
            orientation = PAX_O_ROT_CCW;
            break;
        case BSP_DISPLAY_ROTATION_180:
            orientation = PAX_O_ROT_HALF;
            break;
        case BSP_DISPLAY_ROTATION_270:
            orientation = PAX_O_ROT_CW;
            break;
        case BSP_DISPLAY_ROTATION_0:
        default:
            orientation = PAX_O_UPRIGHT;
            break;
    }

    // Initialize graphics stack
    pax_buf_init(&fb, NULL, display_h_res, display_v_res, format);
    pax_buf_reversed(&fb, display_data_endian == LCD_RGB_DATA_ENDIAN_BIG);
    pax_buf_set_orientation(&fb, orientation);

    // Get input event queue from BSP
    res = bsp_input_get_queue(&input_event_queue);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get input event queue: %d", res);
        return;
    }

    // Start WiFi stack (if your app does not require WiFi or BLE you can remove this section)
    pax_background(&fb, WHITE);
    pax_draw_text(&fb, BLACK, pax_font_sky_mono, 16, 0, 0, "Connecting to radio...");
    blit();

    if (wifi_remote_initialize() != ESP_OK) {
        pax_background(&fb, RED);
        pax_draw_text(&fb, WHITE, pax_font_sky_mono, 16, 0, 0, "Radio unavailable");
        blit();
        return;
    }

    wifi_connection_init_stack();

    if (ESP_OK != esp_hosted_bt_controller_init()) {
        ESP_LOGW("INFO", "failed to init bt controller");
    }

    // enable bt controller
    if (ESP_OK != esp_hosted_bt_controller_enable()) {
        ESP_LOGW("INFO", "failed to enable bt controller");
    }

    res = nimble_port_init();
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init nimble %d ", res);
        return;
    }

    // Configure the host.
    struct ble_hs_cfg;
    ble_hs_cfg.reset_cb        = blecent_on_reset;
    ble_hs_cfg.sync_cb         = blecent_on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    // Initialize data structures to track connected peers.
    int rc = peer_init(MYNEWT_VAL(BLE_MAX_CONNECTIONS), 64, 64, 64);
    assert(rc == 0);

    // Set the default device name.
    rc = ble_svc_gap_device_name_set("nimble-blecent");
    assert(rc == 0);

    // XXX Need to have template for store
    ble_store_config_init();

    nimble_port_freertos_init(blecent_host_task);

#if MYNEWT_VAL(BLE_EATT_CHAN_NUM) > 0
    bearers = 0;
    for (int i = 0; i < MYNEWT_VAL(BLE_EATT_CHAN_NUM); i++) {
        cids[i] = 0;
    }
#endif

    pax_buf_t printer_fb = {0};
    pax_buf_init(&printer_fb, (uint8_t*)image_data, 48*8, 256, PAX_BUF_1_GREY);
    // Synthwave scene: black sky, striped sun on horizon, perspective grid floor
    pax_background(&printer_fb, BLACK);

    // Grid floor: radiating lines from vanishing point (192, 128)
    for (int i = 0; i <= 12; i++) {
        float x = i * (384.0f / 12.0f);
        pax_simple_line(&printer_fb, WHITE, 192, 128, x, 255);
    }
    // Horizontal floor lines with quadratic (perspective) spacing
    for (int i = 1; i <= 6; i++) {
        float t = (float)i / 7.0f;
        float y = 128.0f + (256.0f - 128.0f) * t * t;
        pax_simple_line(&printer_fb, WHITE, 0, y, 383, y);
    }
    // Horizon line
    pax_simple_line(&printer_fb, WHITE, 0, 128, 383, 128);

    // Sun: white circle sitting on horizon, drawn over grid
    pax_simple_circle(&printer_fb, WHITE, 192, 90, 50);
    // Horizontal stripe bands through lower sun (synthwave style)
    for (int i = 0; i < 7; i++) {
        float y = 95.0f + i * 7.0f;
        pax_simple_rect(&printer_fb, BLACK, 140, y, 104, 3.5f);
    }

    pax_draw_text(&printer_fb, 0xFFFFFFFF, pax_font_marker, 48, 0, 0, "Hello world!");
    

    // Main section of the app

    // This example shows how to read from the BSP event queue to read input events

    // If you want to run something at an interval in this same main thread you can replace portMAX_DELAY with an amount
    // of ticks to wait, for example pdMS_TO_TICKS(1000)

    pax_background(&fb, WHITE);
    pax_draw_text(&fb, BLACK, pax_font_sky_mono, 16, 0, 0, "Welcome! Press any key to trigger an event.");
    blit();
    

    while (1) {
        bsp_input_event_t event;
        if (xQueueReceive(input_event_queue, &event, portMAX_DELAY) == pdTRUE) {
            switch (event.type) {
                case INPUT_EVENT_TYPE_KEYBOARD: {
                    if (event.args_keyboard.ascii != '\b' ||
                        event.args_keyboard.ascii != '\t') {  // Ignore backspace & tab keyboard events
                        ESP_LOGI(TAG, "Keyboard event %c (%02x) %s", event.args_keyboard.ascii,
                                 (uint8_t)event.args_keyboard.ascii, event.args_keyboard.utf8);
                        pax_simple_rect(&fb, WHITE, 0, 0, pax_buf_get_width(&fb), 72);
                        pax_draw_text(&fb, BLACK, pax_font_sky_mono, 16, 0, 0, "Keyboard event");
                        char text[64];
                        snprintf(text, sizeof(text), "ASCII:     %c (0x%02x)", event.args_keyboard.ascii,
                                 (uint8_t)event.args_keyboard.ascii);
                        pax_draw_text(&fb, BLACK, pax_font_sky_mono, 16, 0, 18, text);
                        snprintf(text, sizeof(text), "UTF-8:     %s", event.args_keyboard.utf8);
                        pax_draw_text(&fb, BLACK, pax_font_sky_mono, 16, 0, 36, text);
                        snprintf(text, sizeof(text), "Modifiers: 0x%0" PRIX32, event.args_keyboard.modifiers);
                        pax_draw_text(&fb, BLACK, pax_font_sky_mono, 16, 0, 54, text);
                        blit();
                    }
                    break;
                }
                case INPUT_EVENT_TYPE_NAVIGATION: {
                    ESP_LOGI(TAG, "Navigation event %0" PRIX32 ": %s", (uint32_t)event.args_navigation.key,
                             event.args_navigation.state ? "pressed" : "released");

                    if (event.args_navigation.key == BSP_INPUT_NAVIGATION_KEY_F1) {
                        bsp_device_restart_to_launcher();
                    }
                    if (event.args_navigation.key == BSP_INPUT_NAVIGATION_KEY_F2) {
                        bsp_input_set_backlight_brightness(0);
                    }
                    if (event.args_navigation.key == BSP_INPUT_NAVIGATION_KEY_F3) {
                        bsp_input_set_backlight_brightness(100);
                    }

                    pax_simple_rect(&fb, WHITE, 0, 100, pax_buf_get_width(&fb), 72);
                    pax_draw_text(&fb, BLACK, pax_font_sky_mono, 16, 0, 100 + 0, "Navigation event");
                    char text[64];
                    snprintf(text, sizeof(text), "Key:       0x%0" PRIX32, (uint32_t)event.args_navigation.key);
                    pax_draw_text(&fb, BLACK, pax_font_sky_mono, 16, 0, 100 + 18, text);
                    snprintf(text, sizeof(text), "State:     %s", event.args_navigation.state ? "pressed" : "released");
                    pax_draw_text(&fb, BLACK, pax_font_sky_mono, 16, 0, 100 + 36, text);
                    snprintf(text, sizeof(text), "Modifiers: 0x%0" PRIX32, event.args_navigation.modifiers);
                    pax_draw_text(&fb, BLACK, pax_font_sky_mono, 16, 0, 100 + 54, text);
                    blit();
                    break;
                }
                case INPUT_EVENT_TYPE_ACTION: {
                    ESP_LOGI(TAG, "Action event 0x%0" PRIX32 ": %s", (uint32_t)event.args_action.type,
                             event.args_action.state ? "yes" : "no");
                    pax_simple_rect(&fb, WHITE, 0, 200 + 0, pax_buf_get_width(&fb), 72);
                    pax_draw_text(&fb, BLACK, pax_font_sky_mono, 16, 0, 200 + 0, "Action event");
                    char text[64];
                    snprintf(text, sizeof(text), "Type:      0x%0" PRIX32, (uint32_t)event.args_action.type);
                    pax_draw_text(&fb, BLACK, pax_font_sky_mono, 16, 0, 200 + 36, text);
                    snprintf(text, sizeof(text), "State:     %s", event.args_action.state ? "yes" : "no");
                    pax_draw_text(&fb, BLACK, pax_font_sky_mono, 16, 0, 200 + 54, text);
                    blit();
                    break;
                }
                case INPUT_EVENT_TYPE_SCANCODE: {
                    ESP_LOGI(TAG, "Scancode event 0x%0" PRIX32, (uint32_t)event.args_scancode.scancode);
                    pax_simple_rect(&fb, WHITE, 0, 300 + 0, pax_buf_get_width(&fb), 72);
                    pax_draw_text(&fb, BLACK, pax_font_sky_mono, 16, 0, 300 + 0, "Scancode event");
                    char text[64];
                    snprintf(text, sizeof(text), "Scancode:  0x%0" PRIX32, (uint32_t)event.args_scancode.scancode);
                    pax_draw_text(&fb, BLACK, pax_font_sky_mono, 16, 0, 300 + 36, text);
                    blit();
                    break;
                }
                default:
                    break;
            }
        }
    }
}
