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

void SendString(String str);

//================================================== MY STUFF =========================================*/
static volatile int old_ticks = 0;
static volatile int ticks = 0;
void setup() {

    attachInterrupt(44,[](){++ticks;},RISING);

    USBSerial.begin(115200);
    USBSerial.println("Booted");
    USBSerial.println();
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
    Serial1.begin(9600, SERIAL_8N1, 18, 17);
    while(Serial1.available()) {
        Serial1.read();
    }
}
static uint32_t ts = 0;
static int counter = 0;
void loop() {
    /*if(old_ticks!=ticks) {
        USBSerial.println("#");
        old_ticks  =ticks;
    }*/
    // your loop code here 
    // currently increments a label every second
    /*if(Serial1.available()) {
        USBSerial.println("#");
    }
    else*/
    if(millis()>ts+5000) {
        
        ts=millis();

        delay(1000);

        // Serial1 TX "+++"
        USBSerial.println("Sending +++");
        Serial1.print("+++");
        
        //delay(3000);
        
        //USBSerial.println("Sending: ATVR\r");
        //Serial1.print("ATVR\r");
        //delay(1000);
        USBSerial.println("Waiting for response.");
        int i;
        for(i=0;i<1000 && !Serial1.available();++i) {delay(1);}

        if(i<1000) {
            USBSerial.println("Serial1 Data Received");
                //lv_label_set_text(ui_Label1, "Serial1 Data Received");
            String str = Serial1.readStringUntil('\n');
            lv_textarea_set_text(ui_TextArea1, str.c_str());
            USBSerial.print("Received: ");
            USBSerial.println(str.c_str());
        } else {
            USBSerial.println("Timeout.");
        }
       
        
    }
    





    lv_timer_handler();
    delay(3);
}



void SendString(String str){
    
    USBSerial.println("Sending: " + str);
    lv_textarea_set_text(ui_TextArea2,str.c_str());
    Serial1.print(str);

    //delay(3000);

    if (Serial1.available() > 0)
        {
            String RX = Serial1.readString();
            USBSerial.println("Serial1 Data Received: " + RX);
            lv_textarea_set_text(ui_TextArea1,RX.c_str());
        }  



    
}