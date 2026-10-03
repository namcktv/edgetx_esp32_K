/**
 * @file sh1107.cpp
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "edgetx.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_driver.h"
#include "lvgl.h"
#include "telemetry/frsky.h"

extern i2c_master_bus_handle_t toplcd_i2c_bus_handle;
static i2c_master_dev_handle_t toplcd_handle = NULL;

/*********************
 *      DEFINES
 *********************/
#define TAG "SH1107"

#define SH1107_ADDR 0x3C

#define OLED_W 128
#define OLED_H 64

#define MAX_CHAR_IN_STR 16
#define FLYSKY_EXT_VOLTAGE_ID 0x0003
#define FLYSKY_RX_BVD_ID 0x0103
#define FLYSKY_RX_VOLTAGE_ID 0x1000

/*The LCD needs a bunch of command/argument values to be initialized. They are stored in this struct. */
typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

typedef struct {
    lv_font_glyph_dsc_t dsc;
    const uint8_t *bitmap;
} toplcd_glyph_t;

/**********************
 *  STATIC PROTOTYPES
 **********************/
static esp_err_t sh1107_send_cmd(uint8_t cmd);
static void sh1107_send_data(void *data, uint16_t length);
static void advance_oled_cursor(uint8_t *&col, uint8_t &bit);
static uint8_t get_glyph_pixel(const toplcd_glyph_t &glyph, int local_x, int local_y);
static uint32_t get_str_width(const char *str, LcdFlags flags);
static int findTopLcdRxBatterySensor();
static void formatTopLcdVoltage(char *buf, size_t len, int32_t value, uint8_t prec);

static bool top_lcd_exists = true;

static uint8_t oled_buf[OLED_H][(OLED_W / 8) + 1] EXT_RAM_BSS_ATTR;

/**********************
 *   STATIC FUNCTIONS
 **********************/

static void reset_oled_buf(void) {
    for (int i = 0; i < OLED_H; i++) {
        memset(oled_buf[i], 0, (OLED_W / 8) + 1);
        oled_buf[i][0] = 0x40;
    }
}

static esp_err_t sh1107_init(void) {
    esp_err_t err = ESP_OK;
    i2c_device_config_t i2c_dev_conf = {
        .device_address = SH1107_ADDR,
        .scl_speed_hz = 400000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(toplcd_i2c_bus_handle,
            &i2c_dev_conf, &toplcd_handle));

    // SSD1306-compatible init sequence for a 128x64 display
    lcd_init_cmd_t init_cmds[] = {
        { 0xAE, { 0 }, 0 }, // Display OFF
        { 0xD5, { 0 }, 0 }, // Set display clock divide ratio/oscillator frequency
        { 0x80, { 0 }, 0 }, // Default clock divide ratio
        { 0xA8, { 0 }, 0 }, // Set multiplex ratio
        { 0x3F, { 0 }, 0 }, // 1/64 duty
        { 0xD3, { 0 }, 0 }, // Set display offset
        { 0x00, { 0 }, 0 }, // No offset
        { 0x40, { 0 }, 0 }, // Set display start line to 0
        { 0x81, { 0 }, 0 }, // Set contrast control
        { 0xFF, { 0 }, 0 }, // Maximum contrast
        { 0x8D, { 0 }, 0 }, // Charge pump setting
        { 0x14, { 0 }, 0 }, // Enable internal charge pump
        { 0x20, { 0 }, 0 }, // Set memory addressing mode
        { 0x00, { 0 }, 0 }, // Horizontal addressing mode
        { 0xA1, { 0 }, 0 }, // Segment remap, column 127 mapped to SEG0
        { 0xC8, { 0 }, 0 }, // COM output scan direction remapped
        { 0xDA, { 0 }, 0 }, // Set COM pins hardware config
        { 0x12, { 0 }, 0 },
        { 0xD9, { 0 }, 0 }, // Set pre-charge period
        { 0xF1, { 0 }, 0 },
        { 0xDB, { 0 }, 0 }, // Set VCOMH deselect level
        { 0x20, { 0 }, 0 },
        { 0xA4, { 0 }, 0 }, // Display follows RAM content
        { 0xA6, { 0 }, 0 }, // Normal display
        { 0x2E, { 0 }, 0 }, // Deactivate scroll
        { 0xAF, { 0 }, 0 }, // Turn display ON
        { 0, { 0 }, 0xff },
    };

    // Send all the commands
    uint16_t cmd = 0;
    while (init_cmds[cmd].databytes != 0xff) {
        err = sh1107_send_cmd(init_cmds[cmd].cmd);
        if (ESP_OK != err) {
            break;
        }
        sh1107_send_data(init_cmds[cmd].data, init_cmds[cmd].databytes & 0x1F);
        if (init_cmds[cmd].databytes & 0x80) {
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        cmd++;
    }
    return err;
}

static void sh1107_flush(void) {
    const int pages = OLED_H / 8;
    const int bytes_per_page = OLED_W;

    uint8_t page_buf[OLED_W + 1];

    for (int page = 0; page < pages; page++) {
        // SSD1306 page addressing mode
        sh1107_send_cmd(0xB0 | page);  // Set page address
        sh1107_send_cmd(0x00);         // Set lower column address
        sh1107_send_cmd(0x10);         // Set higher column address

        page_buf[0] = 0x40; // Control byte: Co=0, D/C#=1
        memset(&page_buf[1], 0, bytes_per_page);

        for (int x = 0; x < OLED_W; x++) {
            uint8_t column = 0;

            for (int bit = 0; bit < 8; bit++) {
                int y = page * 8 + bit;
                uint8_t row_byte = oled_buf[y][x / 8 + 1];

                if (row_byte & (1U << (x % 8))) {
                    column |= (1U << bit);
                }
            }

            page_buf[x + 1] = column;
        }

        i2c_register_write_buf(toplcd_handle, page_buf, sizeof(page_buf));
    }
}

static esp_err_t sh1107_send_cmd(uint8_t cmd) {
    return i2c_register_write_byte(toplcd_handle, 0, cmd);
}

static void sh1107_send_data(void *data, uint16_t length) {
    if (0 != length) {
        uint8_t buf[OLED_W / 8 + 1] = {0x40};
        memcpy(&buf[1], data, length);
        i2c_register_write_buf(toplcd_handle, (uint8_t *)buf, length + 1);
    }
}

static void advance_oled_cursor(uint8_t *&col, uint8_t &bit)
{
    if (bit == 0x80) {
        col++;
        bit = 0x01;
    }
    else {
        bit <<= 1;
    }
}

static uint8_t get_glyph_pixel(const toplcd_glyph_t &glyph, int local_x, int local_y)
{
    int pixel_index = local_y * glyph.dsc.box_w + local_x;
    uint8_t packed = glyph.bitmap[pixel_index / 2];
    return (pixel_index & 1) ? (packed & 0x0F) : ((packed >> 4) & 0x0F);
}

static uint32_t get_str_width(const char *str, LcdFlags flags)
{
    const lv_font_t *font = getFont(flags);
    uint32_t width = 0;

    for (int i = 0; i < MAX_CHAR_IN_STR && str[i] != '\0'; i++) {
        lv_font_glyph_dsc_t glyph = {0};
        lv_font_get_glyph_dsc(font, &glyph, str[i], str[i + 1]);
        width += glyph.adv_w;
    }

    return width;
}

static void draw_str(uint32_t start_x, uint32_t start_y, const char *str, LcdFlags flags) {
    if (start_x >= OLED_W || start_y >= OLED_H) {
        return;
    }

    const lv_font_t *font = getFont(flags);
    toplcd_glyph_t glyphs[MAX_CHAR_IN_STR] = {0};
    int glyph_count = 0;
    int top = 0;
    int bottom = 0;

    for (int i = 0; i < MAX_CHAR_IN_STR; i++) {
        if (str[i] == '\0') {
            break;
        }

        lv_font_get_glyph_dsc(font, &glyphs[glyph_count].dsc, str[i], str[i + 1]);
        glyphs[glyph_count].bitmap = lv_font_get_glyph_bitmap(font, str[i]);

        if (glyphs[glyph_count].dsc.ofs_y - glyphs[glyph_count].dsc.box_h < top) {
            top = glyphs[glyph_count].dsc.ofs_y - glyphs[glyph_count].dsc.box_h;
        }
        if (-glyphs[glyph_count].dsc.ofs_y > bottom) {
            bottom = -glyphs[glyph_count].dsc.ofs_y;
        }

        glyph_count++;
    }

    for (int y = top; y < bottom; y++) {
        int row = static_cast<int>(start_y) + y;
        if (row < 0 || row >= OLED_H) {
            continue;
        }

        uint8_t *col = &oled_buf[row][start_x / 8 + 1];
        uint8_t bit = (1 << (start_x % 8));
        int current_x = start_x;

        for (int i = 0; i < glyph_count; i++) {
            const toplcd_glyph_t &glyph = glyphs[i];
            int glyph_top = -glyph.dsc.ofs_y - glyph.dsc.box_h;
            int glyph_bottom = -glyph.dsc.ofs_y;

            for (int x = 0; x < glyph.dsc.adv_w && current_x < OLED_W; x++) {
                bool pixel_on = false;

                if (y >= glyph_top && y < glyph_bottom &&
                    x >= glyph.dsc.ofs_x && x < glyph.dsc.ofs_x + glyph.dsc.box_w &&
                    glyph.bitmap != nullptr) {
                    int local_x = x - glyph.dsc.ofs_x;
                    int local_y = y - glyph_top;
                    pixel_on = (get_glyph_pixel(glyph, local_x, local_y) & 0x08) != 0;
                }

                if (pixel_on) {
                    *col |= bit;
                }
                else {
                    *col &= ~bit;
                }

                advance_oled_cursor(col, bit);
                current_x++;
            }
        }
    }
}

static int findTopLcdRxBatterySensor()
{
    int fallback = -1;
    int receiverRail = -1;

    for (int idx = 0; idx < MAX_TELEMETRY_SENSORS; ++idx) {
        const TelemetrySensor &sensor = g_model.telemetrySensors[idx];
        if (!sensor.isAvailable() || sensor.unit != UNIT_VOLTS) {
            continue;
        }

        if (fallback < 0) {
            fallback = idx;
        }

        // On FlySky setups with a flight pack wired to the receiver, A3 is the
        // useful pack voltage while A1/BVD can just be the receiver rail.
        if (sensor.id == FLYSKY_EXT_VOLTAGE_ID) {
            return idx;
        }

        if (receiverRail < 0 &&
            (sensor.id == FLYSKY_RX_VOLTAGE_ID || sensor.id == FLYSKY_RX_BVD_ID)) {
            receiverRail = idx;
        }
    }

    if (receiverRail >= 0) {
        return receiverRail;
    }

    return fallback;
}

static void formatTopLcdVoltage(char *buf, size_t len, int32_t value, uint8_t prec)
{
    if (prec == 0) {
        snprintf(buf, len, "%d", value);
    }
    else if (prec == 1) {
        snprintf(buf, len, "%d.%01d", value / 10, abs(value % 10));
    }
    else {
        snprintf(buf, len, "%d.%02d", value / 100, abs(value % 100));
    }
}

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void toplcdInit()
{
    reset_oled_buf();
    if (ESP_OK != sh1107_init()) {
        TRACE("SH1107 Top LCD not detected");
        top_lcd_exists = false;
    } else {
        TRACE("SH1107 Top LCD initialized");
        draw_str(0, 40, "EdgeTX", FONT(XL));
        sh1107_flush();
    }
}

void toplcdRefresh()
{
    reset_oled_buf();
    
    for (uint8_t idx = 0; idx < MAX_TELEMETRY_SENSORS; idx++) {
        if (g_model.telemetrySensors[idx].isAvailable()) {
            TelemetryItem &telemetryItem = telemetryItems[idx];
            
            if (strcmp(g_model.telemetrySensors[idx].label, "RxBt") == 0) {
                const TelemetrySensor &sensor = g_model.telemetrySensors[idx];
                if (telemetryItem.isAvailable()) {
                    char buf[20] = {0};
                    formatTopLcdVoltage(buf, sizeof(buf), telemetryItem.value, sensor.prec);
                    draw_str(0, 22, "RXBAT", FONT(STD));
                    draw_str(0, 50, buf, FONT(XL));
                }
            }
            
            if (strcmp(g_model.telemetrySensors[idx].label, "RQly") == 0) {
                if (telemetryItem.isAvailable()) {
                    char buf[20] = {0};
                    snprintf(buf, sizeof(buf), "%d", (int)telemetryItem.value);
                    draw_str(OLED_W - get_str_width("RSSI", FONT(STD)), 22, "RSSI", FONT(STD));
                    draw_str(OLED_W - get_str_width(buf, FONT(XL)), 50, buf, FONT(XL));
                }
            }
        }
    }

    if (top_lcd_exists) {
        sh1107_flush();
    }
}
