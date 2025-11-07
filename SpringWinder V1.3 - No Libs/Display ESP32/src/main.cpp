/*
Display ESP32
*/
#include <Arduino.h>
#include <TAMC_GT911.h>
#include <SPI.h>
#include "lvgl.h"
#include "ui.h"
#include <TFT_eSPI.h>



// Screen dimensions
static const uint16_t screenWidth  = 480;    //320
static const uint16_t screenHeight = 320;    //480

enum { SCREENBUFFER_SIZE_PIXELS = screenWidth * screenHeight / 20 };
static lv_color_t buf[SCREENBUFFER_SIZE_PIXELS];

// Display
TFT_eSPI tft = TFT_eSPI(screenWidth, screenHeight);

// Touch screen config
#define TOUCH_SDA 33
#define TOUCH_SCL 32
#define TOUCH_INT 21
#define TOUCH_RST 25

TAMC_GT911 ts = TAMC_GT911(TOUCH_SDA, TOUCH_SCL, TOUCH_INT, TOUCH_RST, screenWidth, screenHeight);

//Communication
constexpr int LINK_RX_PIN = 3;   // receives from display TX
constexpr int LINK_TX_PIN = 1;   // optional, for replies
#define LINK_UART Serial2       //Comment out to use serial

/* Display flush for LVGL */
void my_disp_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *pixelmap) 
{
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);

    tft.startWrite();
    tft.setAddrWindow(area->x1, area->y1, w, h);
    tft.pushColors((uint16_t*)pixelmap, w * h, true);
    tft.endWrite();

    lv_disp_flush_ready(disp);
}

/* Touch read function */ 
void my_touch_read(lv_indev_t * indev, lv_indev_data_t * data) 
{
    ts.read();  // Update touch state (always call this first)
    if (ts.isTouched) {  // Refreshes internal state
        data->point.x = ts.points[0].x;
        data->point.y = ts.points[0].y;
        Serial.printf("Touch at x=%d y=%d\n", ts.points[0].x, ts.points[0].y); // Nice for Debugging Touch Points
        data->state = LV_INDEV_STATE_PR;
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
}

// LVGL tick callback
static uint32_t my_tick_get_cb(void) {
    return millis();
}


// Function to get float value from LVGL object (e.g. spinbox or text area)
float get_lvgl_value(lv_obj_t* obj) {
    const char* txt = lv_textarea_get_text(obj);  // or lv_spinbox_get_value() if it's a spinbox
    return atof(txt);  // Convert string to floa
}


// void StartSamplingLC(lv_event_t * e)
// {
// 	float samplerate_Hz = get_lvgl_value(ui_SamplingOptions);// Your code here
//     float samplerate_ms = 1000/samplerate_Hz;
//     load_cell_timer = lv_timer_create(update_load_cell_label, samplerate_ms, NULL); // 200 ms
// }

// void StopSamplingLC(lv_event_t * e)
// {
// 	// Your code here
// }


void MoveFixtureUp(lv_event_t * e)
{
	//Serial.println("Up");
    Serial2.println("go mm .05");
}

void MoveFixtureDown(lv_event_t * e)
{
	//Serial.println("Down");
    Serial2.println("go mm -.05");
}

void SetWindCount()
{     
 	int NumberOfWindes = lv_roller_get_selected(ui_NumberOfWindes) + 1;
    //Serial.println(NumberOfWindes);
    Serial2.print("go mm -");
    Serial2.println(NumberOfWindes);
}


void StartWinding(lv_event_t * e)
{
	SetWindCount();
    //Serial.println("Start");
}

void StopWinding(lv_event_t * e)
{
	//Serial.println("Stop");
    Serial2.println("Stop");
}



void setup() {
    Serial.begin(115200);
    lv_init();

    ts.begin(GT911_ADDR1);
    ts.setRotation(2);     

    tft.begin();
    tft.setRotation(1);   //2
    
    LINK_UART.begin(115200, SERIAL_8N1, LINK_RX_PIN, LINK_TX_PIN);  //Comment out to use serial

    //lv_timer_create(lc_timer_cb, 400, NULL); // every 200ms

    // Set up LVGL display
    lv_display_t* disp = lv_display_create(screenWidth, screenHeight);
    lv_display_set_buffers(disp, buf, NULL, SCREENBUFFER_SIZE_PIXELS * sizeof(lv_color_t), LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_flush_cb(disp, my_disp_flush);

    // Set up LVGL touch
    lv_indev_t* touch_indev = lv_indev_create();
    lv_indev_set_type(touch_indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(touch_indev, my_touch_read);

    lv_tick_set_cb(my_tick_get_cb);
    ui_init();

    Serial.println("Setup done");
    
    
}

int loopCount = 0;

void loop() {
    
    lv_timer_handler();  // LVGL handler
    delay(10);
    //LogLCReading();
    
}
