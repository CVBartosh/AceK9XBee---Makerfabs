// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.2.3
// LVGL version: 8.2.0
// Project name: SquareLine_Project

#include "ui.h"
#include "ui_helpers.h"

///////////////////// VARIABLES ////////////////////
lv_obj_t * ui_Screen1;
void ui_event_btnSpashScreenBackground(lv_event_t * e);
lv_obj_t * ui_btnSpashScreenBackground;
lv_obj_t * ui_Screen2;
lv_obj_t * ui_ImgButton1;
lv_obj_t * ui_ImgButton2;
lv_obj_t * ui_ImgButton3;
lv_obj_t * ui_ImgButton4;
lv_obj_t * ui_ImgButton5;
void ui_event_ImgButton6(lv_event_t * e);
lv_obj_t * ui_ImgButton6;
lv_obj_t * ui_ImgButton7;
lv_obj_t * ui_ImgButton8;
lv_obj_t * ui_LabelTemp1;
lv_obj_t * ui_LabelTemp2;
lv_obj_t * ui_LabelTempAvg;
lv_obj_t * ui_Screen3;
lv_obj_t * ui_ImgButton13;
void ui_event_ImgButton14(lv_event_t * e);
lv_obj_t * ui_ImgButton14;
void ui_event_BtnEngineStatus(lv_event_t * e);
lv_obj_t * ui_BtnEngineStatus;
lv_obj_t * ui_Label2;
lv_obj_t * ui_LabelEngineStatus;
void ui_event_BtnBatteryStatus(lv_event_t * e);
lv_obj_t * ui_BtnBatteryStatus;
lv_obj_t * ui_Label3;
lv_obj_t * ui_LabelBatteryStatus;
void ui_event_BtnTemperatureStatus(lv_event_t * e);
lv_obj_t * ui_BtnTemperatureStatus;
lv_obj_t * ui_Label4;
void ui_event_ImgButton10(lv_event_t * e);
lv_obj_t * ui_ImgButton10;
lv_obj_t * ui_Screen4;
void ui_event_ImgButton9(lv_event_t * e);
lv_obj_t * ui_ImgButton9;
lv_obj_t * ui_Label5;
lv_obj_t * ui_TextArea2;
lv_obj_t * ui_Label1;
lv_obj_t * ui_TextArea1;
lv_obj_t * ui____initial_actions0;



///////////////////// ANIMATIONS ////////////////////

///////////////////// FUNCTIONS ////////////////////
void ui_event_btnSpashScreenBackground(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(ui_Screen2, LV_SCR_LOAD_ANIM_FADE_ON, 500, 0);
    }
}
void ui_event_ImgButton6(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(ui_Screen3, LV_SCR_LOAD_ANIM_MOVE_LEFT, 500, 0);
    }
}
void ui_event_ImgButton14(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(ui_Screen2, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 500, 0);
    }
}
void ui_event_BtnEngineStatus(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        RequestEngineStatus_CB(e);
    }
}
void ui_event_BtnBatteryStatus(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        RequestBatteryStatus_CB(e);
    }
}
void ui_event_BtnTemperatureStatus(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        RequestTempStatus_CB(e);
    }
}
void ui_event_ImgButton10(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(ui_Screen4, LV_SCR_LOAD_ANIM_MOVE_LEFT, 500, 0);
    }
}
void ui_event_ImgButton9(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(ui_Screen3, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 500, 0);
    }
}

///////////////////// SCREENS ////////////////////
void ui_Screen1_screen_init(void)
{
    ui_Screen1 = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Screen1, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_btnSpashScreenBackground = lv_imgbtn_create(ui_Screen1);
    lv_imgbtn_set_src(ui_btnSpashScreenBackground, LV_IMGBTN_STATE_RELEASED, NULL, &ui_img_acek9logo_png, NULL);
    lv_imgbtn_set_src(ui_btnSpashScreenBackground, LV_IMGBTN_STATE_PRESSED, NULL, &ui_img_acek9logo_png, NULL);
    lv_imgbtn_set_src(ui_btnSpashScreenBackground, LV_IMGBTN_STATE_DISABLED, NULL, &ui_img_acek9logo_png, NULL);
    lv_obj_set_width(ui_btnSpashScreenBackground, 325);
    lv_obj_set_height(ui_btnSpashScreenBackground, 108);
    lv_obj_set_x(ui_btnSpashScreenBackground, -1);
    lv_obj_set_y(ui_btnSpashScreenBackground, lv_pct(-23));
    lv_obj_set_align(ui_btnSpashScreenBackground, LV_ALIGN_CENTER);

    lv_obj_add_event_cb(ui_btnSpashScreenBackground, ui_event_btnSpashScreenBackground, LV_EVENT_ALL, NULL);

}
void ui_Screen2_screen_init(void)
{
    ui_Screen2 = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Screen2, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_ImgButton1 = lv_imgbtn_create(ui_Screen2);
    lv_imgbtn_set_src(ui_ImgButton1, LV_IMGBTN_STATE_RELEASED, NULL, &ui_img_batteryenabledicon_png, NULL);
    lv_imgbtn_set_src(ui_ImgButton1, LV_IMGBTN_STATE_PRESSED, NULL, &ui_img_batterydisabledicon_png, NULL);
    lv_obj_set_height(ui_ImgButton1, 64);
    lv_obj_set_width(ui_ImgButton1, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_x(ui_ImgButton1, -140);
    lv_obj_set_y(ui_ImgButton1, -120);
    lv_obj_set_align(ui_ImgButton1, LV_ALIGN_CENTER);

    ui_ImgButton2 = lv_imgbtn_create(ui_Screen2);
    lv_imgbtn_set_src(ui_ImgButton2, LV_IMGBTN_STATE_RELEASED, NULL, &ui_img_cardoorenabledicon_png, NULL);
    lv_imgbtn_set_src(ui_ImgButton2, LV_IMGBTN_STATE_PRESSED, NULL, &ui_img_cardoordisabledicon_png, NULL);
    lv_obj_set_height(ui_ImgButton2, 64);
    lv_obj_set_width(ui_ImgButton2, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_x(ui_ImgButton2, -70);
    lv_obj_set_y(ui_ImgButton2, -120);
    lv_obj_set_align(ui_ImgButton2, LV_ALIGN_CENTER);

    ui_ImgButton3 = lv_imgbtn_create(ui_Screen2);
    lv_imgbtn_set_src(ui_ImgButton3, LV_IMGBTN_STATE_RELEASED, NULL, &ui_img_keyenabledicon_png, NULL);
    lv_imgbtn_set_src(ui_ImgButton3, LV_IMGBTN_STATE_PRESSED, NULL, &ui_img_keyicon_png, NULL);
    lv_obj_set_height(ui_ImgButton3, 64);
    lv_obj_set_width(ui_ImgButton3, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_x(ui_ImgButton3, 0);
    lv_obj_set_y(ui_ImgButton3, -120);
    lv_obj_set_align(ui_ImgButton3, LV_ALIGN_CENTER);

    ui_ImgButton4 = lv_imgbtn_create(ui_Screen2);
    lv_imgbtn_set_src(ui_ImgButton4, LV_IMGBTN_STATE_RELEASED, NULL, &ui_img_engineenabledicon_png, NULL);
    lv_imgbtn_set_src(ui_ImgButton4, LV_IMGBTN_STATE_PRESSED, NULL, &ui_img_enginedisabledicon_png, NULL);
    lv_obj_set_height(ui_ImgButton4, 64);
    lv_obj_set_width(ui_ImgButton4, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_x(ui_ImgButton4, 70);
    lv_obj_set_y(ui_ImgButton4, -120);
    lv_obj_set_align(ui_ImgButton4, LV_ALIGN_CENTER);

    ui_ImgButton5 = lv_imgbtn_create(ui_Screen2);
    lv_imgbtn_set_src(ui_ImgButton5, LV_IMGBTN_STATE_RELEASED, NULL, &ui_img_wifienabledicon_png, NULL);
    lv_imgbtn_set_src(ui_ImgButton5, LV_IMGBTN_STATE_PRESSED, NULL, &ui_img_wifidisabledicon_png, NULL);
    lv_obj_set_height(ui_ImgButton5, 64);
    lv_obj_set_width(ui_ImgButton5, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_x(ui_ImgButton5, 140);
    lv_obj_set_y(ui_ImgButton5, -120);
    lv_obj_set_align(ui_ImgButton5, LV_ALIGN_CENTER);

    ui_ImgButton6 = lv_imgbtn_create(ui_Screen2);
    lv_imgbtn_set_src(ui_ImgButton6, LV_IMGBTN_STATE_RELEASED, NULL, &ui_img_arrowrighticon_png, NULL);
    lv_obj_set_height(ui_ImgButton6, 320);
    lv_obj_set_width(ui_ImgButton6, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_x(ui_ImgButton6, 210);
    lv_obj_set_y(ui_ImgButton6, 0);
    lv_obj_set_align(ui_ImgButton6, LV_ALIGN_CENTER);

    ui_ImgButton7 = lv_imgbtn_create(ui_Screen2);
    lv_imgbtn_set_src(ui_ImgButton7, LV_IMGBTN_STATE_RELEASED, NULL, &ui_img_arrowlefticon_png, NULL);
    lv_imgbtn_set_src(ui_ImgButton7, LV_IMGBTN_STATE_DISABLED, NULL, &ui_img_arrowiconblank_png, NULL);
    lv_obj_set_height(ui_ImgButton7, 320);
    lv_obj_set_width(ui_ImgButton7, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_x(ui_ImgButton7, -210);
    lv_obj_set_y(ui_ImgButton7, 0);
    lv_obj_set_align(ui_ImgButton7, LV_ALIGN_CENTER);
    lv_obj_add_state(ui_ImgButton7, LV_STATE_DISABLED);       /// States

    ui_ImgButton8 = lv_imgbtn_create(ui_Screen2);
    lv_imgbtn_set_src(ui_ImgButton8, LV_IMGBTN_STATE_RELEASED, NULL, &ui_img_greentempcircle_png, NULL);
    lv_imgbtn_set_src(ui_ImgButton8, LV_IMGBTN_STATE_PRESSED, NULL, &ui_img_redtempcircle_png, NULL);
    lv_obj_set_width(ui_ImgButton8, 180);
    lv_obj_set_height(ui_ImgButton8, 180);
    lv_obj_set_x(ui_ImgButton8, 0);
    lv_obj_set_y(ui_ImgButton8, 35);
    lv_obj_set_align(ui_ImgButton8, LV_ALIGN_CENTER);

    ui_LabelTemp1 = lv_label_create(ui_Screen2);
    lv_obj_set_width(ui_LabelTemp1, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_LabelTemp1, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_LabelTemp1, -133);
    lv_obj_set_y(ui_LabelTemp1, 45);
    lv_obj_set_align(ui_LabelTemp1, LV_ALIGN_CENTER);
    lv_label_set_text(ui_LabelTemp1, "0");

    ui_LabelTemp2 = lv_label_create(ui_Screen2);
    lv_obj_set_width(ui_LabelTemp2, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_LabelTemp2, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_LabelTemp2, 132);
    lv_obj_set_y(ui_LabelTemp2, 46);
    lv_obj_set_align(ui_LabelTemp2, LV_ALIGN_CENTER);
    lv_label_set_text(ui_LabelTemp2, "0");

    ui_LabelTempAvg = lv_label_create(ui_Screen2);
    lv_obj_set_width(ui_LabelTempAvg, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_LabelTempAvg, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_LabelTempAvg, 0);
    lv_obj_set_y(ui_LabelTempAvg, 34);
    lv_obj_set_align(ui_LabelTempAvg, LV_ALIGN_CENTER);
    lv_label_set_text(ui_LabelTempAvg, "0");

    lv_obj_add_event_cb(ui_ImgButton6, ui_event_ImgButton6, LV_EVENT_ALL, NULL);

}
void ui_Screen3_screen_init(void)
{
    ui_Screen3 = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Screen3, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_ImgButton13 = lv_imgbtn_create(ui_Screen3);
    lv_imgbtn_set_src(ui_ImgButton13, LV_IMGBTN_STATE_RELEASED, NULL, &ui_img_arrowrighticon_png, NULL);
    lv_imgbtn_set_src(ui_ImgButton13, LV_IMGBTN_STATE_DISABLED, NULL, &ui_img_arrowiconblank_png, NULL);
    lv_obj_set_height(ui_ImgButton13, 320);
    lv_obj_set_width(ui_ImgButton13, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_x(ui_ImgButton13, 210);
    lv_obj_set_y(ui_ImgButton13, 0);
    lv_obj_set_align(ui_ImgButton13, LV_ALIGN_CENTER);
    lv_obj_add_state(ui_ImgButton13, LV_STATE_DISABLED);       /// States

    ui_ImgButton14 = lv_imgbtn_create(ui_Screen3);
    lv_imgbtn_set_src(ui_ImgButton14, LV_IMGBTN_STATE_RELEASED, NULL, &ui_img_arrowlefticon_png, NULL);
    lv_imgbtn_set_src(ui_ImgButton14, LV_IMGBTN_STATE_DISABLED, NULL, &ui_img_arrowiconblank_png, NULL);
    lv_obj_set_height(ui_ImgButton14, 320);
    lv_obj_set_width(ui_ImgButton14, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_x(ui_ImgButton14, -210);
    lv_obj_set_y(ui_ImgButton14, 0);
    lv_obj_set_align(ui_ImgButton14, LV_ALIGN_CENTER);

    ui_BtnEngineStatus = lv_btn_create(ui_Screen3);
    lv_obj_set_width(ui_BtnEngineStatus, 120);
    lv_obj_set_height(ui_BtnEngineStatus, 40);
    lv_obj_set_x(ui_BtnEngineStatus, -102);
    lv_obj_set_y(ui_BtnEngineStatus, -127);
    lv_obj_set_align(ui_BtnEngineStatus, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_BtnEngineStatus, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_BtnEngineStatus, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_BtnEngineStatus, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(ui_BtnEngineStatus, 255, LV_PART_MAIN | LV_STATE_PRESSED);

    ui_Label2 = lv_label_create(ui_BtnEngineStatus);
    lv_obj_set_width(ui_Label2, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label2, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Label2, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label2, "Engine Status");
    lv_obj_clear_flag(ui_Label2, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE);      /// Flags

    ui_LabelEngineStatus = lv_label_create(ui_Screen3);
    lv_obj_set_width(ui_LabelEngineStatus, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_LabelEngineStatus, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_LabelEngineStatus, -11);
    lv_obj_set_y(ui_LabelEngineStatus, -126);
    lv_obj_set_align(ui_LabelEngineStatus, LV_ALIGN_CENTER);
    lv_obj_set_style_text_color(ui_LabelEngineStatus, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_LabelEngineStatus, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_LabelEngineStatus, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_LabelEngineStatus, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_BtnBatteryStatus = lv_btn_create(ui_Screen3);
    lv_obj_set_width(ui_BtnBatteryStatus, 120);
    lv_obj_set_height(ui_BtnBatteryStatus, 40);
    lv_obj_set_x(ui_BtnBatteryStatus, -103);
    lv_obj_set_y(ui_BtnBatteryStatus, -69);
    lv_obj_set_align(ui_BtnBatteryStatus, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_BtnBatteryStatus, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_BtnBatteryStatus, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_BtnBatteryStatus, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(ui_BtnBatteryStatus, 255, LV_PART_MAIN | LV_STATE_PRESSED);

    ui_Label3 = lv_label_create(ui_BtnBatteryStatus);
    lv_obj_set_width(ui_Label3, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label3, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Label3, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label3, "Battery Status");
    lv_obj_clear_flag(ui_Label3, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE);      /// Flags

    ui_LabelBatteryStatus = lv_label_create(ui_Screen3);
    lv_obj_set_width(ui_LabelBatteryStatus, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_LabelBatteryStatus, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_LabelBatteryStatus, -11);
    lv_obj_set_y(ui_LabelBatteryStatus, -71);
    lv_obj_set_align(ui_LabelBatteryStatus, LV_ALIGN_CENTER);
    lv_obj_set_style_text_color(ui_LabelBatteryStatus, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_LabelBatteryStatus, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_LabelBatteryStatus, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_LabelBatteryStatus, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_BtnTemperatureStatus = lv_btn_create(ui_Screen3);
    lv_obj_set_width(ui_BtnTemperatureStatus, 120);
    lv_obj_set_height(ui_BtnTemperatureStatus, 40);
    lv_obj_set_x(ui_BtnTemperatureStatus, -103);
    lv_obj_set_y(ui_BtnTemperatureStatus, -15);
    lv_obj_set_align(ui_BtnTemperatureStatus, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_BtnTemperatureStatus, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_BtnTemperatureStatus, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_BtnTemperatureStatus, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(ui_BtnTemperatureStatus, 255, LV_PART_MAIN | LV_STATE_PRESSED);

    ui_Label4 = lv_label_create(ui_BtnTemperatureStatus);
    lv_obj_set_width(ui_Label4, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label4, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Label4, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label4, "Temp Status");
    lv_obj_clear_flag(ui_Label4, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE);      /// Flags

    ui_ImgButton10 = lv_imgbtn_create(ui_Screen3);
    lv_imgbtn_set_src(ui_ImgButton10, LV_IMGBTN_STATE_RELEASED, NULL, &ui_img_arrowrighticon_png, NULL);
    lv_obj_set_height(ui_ImgButton10, 320);
    lv_obj_set_width(ui_ImgButton10, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_x(ui_ImgButton10, 210);
    lv_obj_set_y(ui_ImgButton10, 0);
    lv_obj_set_align(ui_ImgButton10, LV_ALIGN_CENTER);

    lv_obj_add_event_cb(ui_ImgButton14, ui_event_ImgButton14, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_BtnEngineStatus, ui_event_BtnEngineStatus, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_BtnBatteryStatus, ui_event_BtnBatteryStatus, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_BtnTemperatureStatus, ui_event_BtnTemperatureStatus, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_ImgButton10, ui_event_ImgButton10, LV_EVENT_ALL, NULL);

}
void ui_Screen4_screen_init(void)
{
    ui_Screen4 = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Screen4, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_ImgButton9 = lv_imgbtn_create(ui_Screen4);
    lv_imgbtn_set_src(ui_ImgButton9, LV_IMGBTN_STATE_RELEASED, NULL, &ui_img_arrowlefticon_png, NULL);
    lv_imgbtn_set_src(ui_ImgButton9, LV_IMGBTN_STATE_DISABLED, NULL, &ui_img_arrowiconblank_png, NULL);
    lv_obj_set_height(ui_ImgButton9, 320);
    lv_obj_set_width(ui_ImgButton9, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_x(ui_ImgButton9, -210);
    lv_obj_set_y(ui_ImgButton9, 0);
    lv_obj_set_align(ui_ImgButton9, LV_ALIGN_CENTER);

    ui_Label5 = lv_label_create(ui_Screen4);
    lv_obj_set_width(ui_Label5, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label5, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label5, -133);
    lv_obj_set_y(ui_Label5, -143);
    lv_obj_set_align(ui_Label5, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label5, "TX Data");

    ui_TextArea2 = lv_textarea_create(ui_Screen4);
    lv_obj_set_width(ui_TextArea2, 383);
    lv_obj_set_height(ui_TextArea2, 73);
    lv_obj_set_x(ui_TextArea2, 28);
    lv_obj_set_y(ui_TextArea2, -95);
    lv_obj_set_align(ui_TextArea2, LV_ALIGN_CENTER);
    lv_textarea_set_placeholder_text(ui_TextArea2, "Placeholder...");

    ui_Label1 = lv_label_create(ui_Screen4);
    lv_obj_set_width(ui_Label1, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label1, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label1, -131);
    lv_obj_set_y(ui_Label1, -45);
    lv_obj_set_align(ui_Label1, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label1, "RX Data");

    ui_TextArea1 = lv_textarea_create(ui_Screen4);
    lv_obj_set_width(ui_TextArea1, 383);
    lv_obj_set_height(ui_TextArea1, 180);
    lv_obj_set_x(ui_TextArea1, 30);
    lv_obj_set_y(ui_TextArea1, 59);
    lv_obj_set_align(ui_TextArea1, LV_ALIGN_CENTER);
    lv_textarea_set_placeholder_text(ui_TextArea1, "Placeholder...");

    lv_obj_add_event_cb(ui_ImgButton9, ui_event_ImgButton9, LV_EVENT_ALL, NULL);

}

void ui_init(void)
{
    lv_disp_t * dispp = lv_disp_get_default();
    lv_theme_t * theme = lv_theme_default_init(dispp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED),
                                               true, LV_FONT_DEFAULT);
    lv_disp_set_theme(dispp, theme);
    ui_Screen1_screen_init();
    ui_Screen2_screen_init();
    ui_Screen3_screen_init();
    ui_Screen4_screen_init();
    ui____initial_actions0 = lv_obj_create(NULL);
    lv_disp_load_scr(ui_Screen4);
}
