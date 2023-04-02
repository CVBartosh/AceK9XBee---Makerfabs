#define LCD_HRES 480
#define LCD_VRES 320
#define LVGL_LCD_BUF_SIZE (16*1024)
#include <Arduino.h>
#include <lcd_controller.h>
#include <lvgl.h>
#include <ui.h>
static lv_disp_draw_buf_t disp_buf;  // contains internal graphic buffer(s) called draw buffer(s)
static lv_disp_drv_t disp_drv;       // contains callback functions
static lv_color_t *lv_disp_buf;
static lv_color_t *lv_disp_buf2;
static bool is_initialized_lvgl = false;

static bool lcd_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t* edata, void* user_ctx) {
    lv_disp_flush_ready(&disp_drv);
    return true;
}
static void lvgl_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map) {
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    // copy a buffer's content to a specific area of the display
    lcd_flush(offsetx1, offsety1, offsetx2 , offsety2 , color_map);
}


//================================================== MY STUFF =========================================
const byte numChars = 32;
char receivedChars[numChars];   // an array to store the received data

boolean newData = false;

//================================================== MY STUFF =========================================*/


void setup() {

    Serial1.begin(9600, SERIAL_8N1, 18, 17);

    Serial.begin(115200);
    Serial.println("Booted");
    Serial.println();
    lv_init();
    lv_disp_buf = (lv_color_t *)heap_caps_malloc(LVGL_LCD_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    lv_disp_buf2 = (lv_color_t *)heap_caps_malloc(LVGL_LCD_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    lv_disp_draw_buf_init(&disp_buf, lv_disp_buf, lv_disp_buf2, LVGL_LCD_BUF_SIZE);
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_HRES;
    disp_drv.ver_res = LCD_VRES;
    disp_drv.flush_cb = lvgl_flush;
    disp_drv.draw_buf = &disp_buf;
    lcd_color_trans_done_register_cb(lcd_flush_ready, &disp_drv);
    lcd_init(32*1024);  
    lv_disp_drv_register(&disp_drv);
    ui_init();
    is_initialized_lvgl = true;
    
    // your setup code here:
    
    
}
static uint32_t ts = 0;
static int counter = 0;
void loop() {
    // your loop code here 
    // currently increments a label every second
    if(millis()>ts+1000) {
        ts=millis();

        // Serial TX "+++"
        Serial1.print("+++");
        lv_label_set_text(ui_Label1, "Sending: +++");
        //delay(5000);

        Serial1.print("ATVR\r");
        lv_label_set_text(ui_Label1, "Sending: ATVR\r");

        //delay(5000);

        char sz[128]; 
        char rc;
        int counter = 0;
        
        while (Serial.available() > 0) {
            
            rc = Serial1.read();
            sz[counter] = rc;
            counter++;
        }
        sz[counter] = '\0';

        String str = "Received: " + String(sz);
        char buff[str.length() + 1];
        
        str.toCharArray(buff,str.length()+1);

        lv_label_set_text(ui_Label1, buff);
    }
    lv_timer_handler();
    delay(3);
}

void ReceiveData();

void ReceiveData(){
    

    
}