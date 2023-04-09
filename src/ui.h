// SquareLine LVGL GENERATED FILE
// EDITOR VERSION: SquareLine Studio 1.1.0
// LVGL VERSION: 8.2
// PROJECT: SquareLine_Project

#ifndef _SQUARELINE_PROJECT_UI_H
#define _SQUARELINE_PROJECT_UI_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined __has_include
#if __has_include("lvgl.h")
#include "lvgl.h"
#elif __has_include("lvgl/lvgl.h")
#include "lvgl/lvgl.h"
#else
#include "lvgl.h"
#endif
#else
#include "lvgl.h"
#endif

#include "ui_events.h"
extern lv_obj_t * ui_Screen1;
void ui_event_btnSpashScreenBackground(lv_event_t * e);
extern lv_obj_t * ui_btnSpashScreenBackground;
extern lv_obj_t * ui_Screen2;
extern lv_obj_t * ui_ImgButton1;
extern lv_obj_t * ui_ImgButton2;
extern lv_obj_t * ui_ImgButton3;
extern lv_obj_t * ui_ImgButton4;
extern lv_obj_t * ui_ImgButton5;
void ui_event_ImgButton6(lv_event_t * e);
extern lv_obj_t * ui_ImgButton6;
extern lv_obj_t * ui_ImgButton7;
extern lv_obj_t * ui_ImgButton8;
extern lv_obj_t * ui_LabelTemp1;
extern lv_obj_t * ui_LabelTemp2;
extern lv_obj_t * ui_LabelTempAvg;
extern lv_obj_t * ui_Screen3;
extern lv_obj_t * ui_ImgButton13;
void ui_event_ImgButton14(lv_event_t * e);
extern lv_obj_t * ui_ImgButton14;
void ui_event_BtnEngineStatus(lv_event_t * e);
extern lv_obj_t * ui_BtnEngineStatus;
extern lv_obj_t * ui_Label2;
extern lv_obj_t * ui_LabelEngineStatus;
void ui_event_BtnBatteryStatus(lv_event_t * e);
extern lv_obj_t * ui_BtnBatteryStatus;
extern lv_obj_t * ui_Label3;
extern lv_obj_t * ui_LabelBatteryStatus;
void ui_event_BtnTemperatureStatus(lv_event_t * e);
extern lv_obj_t * ui_BtnTemperatureStatus;
extern lv_obj_t * ui_Label4;
void ui_event_ImgButton10(lv_event_t * e);
extern lv_obj_t * ui_ImgButton10;
extern lv_obj_t * ui_Screen4;
void ui_event_ImgButton9(lv_event_t * e);
extern lv_obj_t * ui_ImgButton9;
extern lv_obj_t * ui_Label5;
extern lv_obj_t * ui_TextArea2;
extern lv_obj_t * ui_Label1;
extern lv_obj_t * ui_TextArea1;
extern lv_obj_t * ui____initial_actions0;


LV_IMG_DECLARE(ui_img_acek9logo_png);    // assets\ACEK9Logo.png
LV_IMG_DECLARE(ui_img_batteryenabledicon_png);    // assets\BatteryEnabledIcon.png
LV_IMG_DECLARE(ui_img_batterydisabledicon_png);    // assets\BatteryDisabledIcon.png
LV_IMG_DECLARE(ui_img_cardoorenabledicon_png);    // assets\CarDoorEnabledIcon.png
LV_IMG_DECLARE(ui_img_cardoordisabledicon_png);    // assets\CarDoorDisabledIcon.png
LV_IMG_DECLARE(ui_img_keyenabledicon_png);    // assets\KeyEnabledIcon.png
LV_IMG_DECLARE(ui_img_keyicon_png);    // assets\KeyIcon.png
LV_IMG_DECLARE(ui_img_engineenabledicon_png);    // assets\EngineEnabledIcon.png
LV_IMG_DECLARE(ui_img_enginedisabledicon_png);    // assets\EngineDisabledIcon.png
LV_IMG_DECLARE(ui_img_wifienabledicon_png);    // assets\WiFiEnabledIcon.png
LV_IMG_DECLARE(ui_img_wifidisabledicon_png);    // assets\WiFiDisabledIcon.png
LV_IMG_DECLARE(ui_img_arrowrighticon_png);    // assets\ArrowRightIcon.png
LV_IMG_DECLARE(ui_img_arrowlefticon_png);    // assets\ArrowLeftIcon.png
LV_IMG_DECLARE(ui_img_arrowiconblank_png);    // assets\ArrowIconBlank.png
LV_IMG_DECLARE(ui_img_greentempcircle_png);    // assets\GreenTempCircle.png
LV_IMG_DECLARE(ui_img_redtempcircle_png);    // assets\RedTempCircle.png




void ui_init(void);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif