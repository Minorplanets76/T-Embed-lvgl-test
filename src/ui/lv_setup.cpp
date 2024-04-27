#include <lvgl.h>          // https://github.com/lvgl/lvgl
#include <TFT_eSPI.h>      // https://github.com/Bodmer/TFT_eSPI
#include <RotaryEncoder.h> // https://github.com/mathertel/RotaryEncoder
#include <OneButton.h>     // https://github.com/mathertel/OneButton
#include <APA102.h>        // https://github.com/pololu/apa102-arduino
#include "../pin_config.h"

/* Screen dimensions */
#define LV_SCREEN_WIDTH 170
#define LV_SCREEN_HEIGHT 320
#define LV_BUF_SIZE (LV_SCREEN_WIDTH * LV_SCREEN_HEIGHT)

/* Input events bit masks */
#define LV_BUTTON bit(0)
#define LV_ENCODER_CW bit(1)
#define LV_ENCODER_CCW bit(2)
#define LV_ENCODER_LED_CW bit(3)
#define LV_ENCODER_LED_CCW bit(4)



EventGroupHandle_t lv_input_event;
RotaryEncoder encoder(PIN_ENCODE_A, PIN_ENCODE_B, RotaryEncoder::LatchMode::TWO03);
OneButton button(PIN_ENCODE_BTN, true);
TFT_eSPI tft = TFT_eSPI(LV_SCREEN_WIDTH, LV_SCREEN_HEIGHT);


typedef struct {
  uint8_t cmd;
  uint8_t data[14];
  uint8_t len;
} lcd_cmd_t;

lcd_cmd_t lcd_st7789v[] = {
  {0x11, {0}, 0 | 0x80},
  {0x3A, {0X05}, 1},
  {0xB2, {0X0B, 0X0B, 0X00, 0X33, 0X33}, 5},
  {0xB7, {0X75}, 1},
  {0xBB, {0X28}, 1},
  {0xC0, {0X2C}, 1},
  {0xC2, {0X01}, 1},
  {0xC3, {0X1F}, 1},
  {0xC6, {0X13}, 1},
  {0xD0, {0XA7}, 1},
  {0xD0, {0XA4, 0XA1}, 2},
  {0xD6, {0XA1}, 1},
  {0xE0, {0XF0, 0X05, 0X0A, 0X06, 0X06, 0X03, 0X2B, 0X32, 0X43, 0X36, 0X11, 0X10, 0X2B, 0X32}, 14},
  {0xE1, {0XF0, 0X08, 0X0C, 0X0B, 0X09, 0X24, 0X2B, 0X22, 0X43, 0X38, 0X15, 0X16, 0X2F, 0X37}, 14},
};

/* Display rendering callback */
static void lv_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);
    tft.setAddrWindow(area->x1, area->y1, w, h);
    tft.pushColors((uint16_t *)&color_p->full, w * h);
    lv_disp_flush_ready(disp);
}

/* Read input events callback */
static void lv_encoder_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data)
{
    EventGroupHandle_t *lv_input_event = (EventGroupHandle_t *)indev_drv->user_data;
    EventBits_t bit = xEventGroupGetBits(lv_input_event);
    data->state = LV_INDEV_STATE_RELEASED;
    if (bit & LV_BUTTON)
    {
        // Serial.println("Button pressed");
        xEventGroupClearBits(lv_input_event, LV_BUTTON);
        data->state = LV_INDEV_STATE_PR;
    }
    else if (bit & LV_ENCODER_CW)
    {
        // Serial.println("Encoder CW");
        xEventGroupClearBits(lv_input_event, LV_ENCODER_CW);
        data->enc_diff = 1;
    }
    else if (bit & LV_ENCODER_CCW)
    {
        // Serial.println("Encoder CCW");
        xEventGroupClearBits(lv_input_event, LV_ENCODER_CCW);
        data->enc_diff = -1;
    }
}

/* Read encoder direction */
static void encoder_read()
{
    RotaryEncoder::Direction dir = encoder.getDirection();
    if (dir != RotaryEncoder::Direction::NOROTATION)
    {
        if (dir != RotaryEncoder::Direction::CLOCKWISE)
        {
            xEventGroupSetBits(lv_input_event, LV_ENCODER_CW);
            // xEventGroupSetBits(lv_input_event, LV_ENCODER_LED_CW);
        }
        else
        {
            xEventGroupSetBits(lv_input_event, LV_ENCODER_CCW);
            // xEventGroupSetBits(lv_input_event, LV_ENCODER_LED_CCW);
        }
    }
}

/* Setup display, rotary encoder and button */
void lv_begin()
{
    /* Serial init */
    Serial.begin(115200);

    /* Enable display */
    pinMode(PIN_LCD_POWER_ON, OUTPUT);
    digitalWrite(PIN_LCD_POWER_ON, HIGH); // Turn on the display enable pin

    /* Initialize rotary encoder button*/
    lv_input_event = xEventGroupCreate();
    button.attachClick(
        [](void *param)
        {
            EventGroupHandle_t *lv_input_event = (EventGroupHandle_t *)param;
            xEventGroupSetBits(lv_input_event, LV_BUTTON);
            // xEventGroupSetBits(global_event_group, WAV_RING_1);
        },
        lv_input_event);

    /* Initialize tft */
    tft.begin();
//     for (uint8_t i = 0; i < (sizeof(lcd_st7789v) / sizeof(lcd_cmd_t)); i++) {
//         tft.writecommand(lcd_st7789v[i].cmd);
//     for (int j = 0; j < lcd_st7789v[i].len & 0x7f; j++) {
//         tft.writedata(lcd_st7789v[i].data[j]);
//     }
//     if (lcd_st7789v[i].len & 0x80) {
//         delay(120);
//     }
//   } 
   
    tft.fillScreen(TFT_WHITE);
    tft.setRotation(2); /* portrait */
    tft.fillScreen(TFT_WHITE);
    
    /* Initialize encoder */

    /* Call before other lv functions */
    lv_init();

    /* Initialize the display buffer */
    static lv_disp_draw_buf_t draw_buf;
    static lv_color_t *buf1, *buf2;
    buf1 = (lv_color_t *)heap_caps_malloc(LV_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
    assert(buf1);
    buf2 = (lv_color_t *)heap_caps_malloc(LV_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
    assert(buf2);
    lv_disp_draw_buf_init(&draw_buf, buf1, buf2, LV_BUF_SIZE);

    /* Initialize the display */
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LV_SCREEN_WIDTH;
    disp_drv.ver_res = LV_SCREEN_HEIGHT;
    disp_drv.flush_cb = lv_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    /* Initialize the rotary encoder */
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_ENCODER;
    indev_drv.read_cb = lv_encoder_read;
    indev_drv.user_data = lv_input_event;

    /* Initialize the rotary input group */
    static lv_indev_t *lv_encoder_indev = lv_indev_drv_register(&indev_drv);
    lv_group_t *g = lv_group_create();
    lv_indev_set_group(lv_encoder_indev, g);
    lv_group_set_default(g);

    Serial.printf("Lvgl v%d.%d.%d initialized\n", lv_version_major(), lv_version_minor(), lv_version_patch());
}

/* Handles updating the display and encoder/button events */
void lv_handler()
{
    lv_task_handler();
    delay(5);
    button.tick();
    encoder.tick();
    encoder_read();
}