// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.1
// LVGL version: 9.1.0
// Project name: sq_t

#include "ui.h"

void ui_ScreenMain_screen_init(void)
{
ui_ScreenMain = lv_obj_create(NULL);
lv_obj_remove_flag( ui_ScreenMain, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_bg_color(ui_ScreenMain, lv_color_hex(0x161616), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_ScreenMain, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_ArcRpm = lv_arc_create(ui_ScreenMain);
lv_obj_set_width( ui_ArcRpm, 230);
lv_obj_set_height( ui_ArcRpm, 230);
lv_obj_set_align( ui_ArcRpm, LV_ALIGN_CENTER );
lv_obj_remove_flag( ui_ArcRpm, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_arc_set_range(ui_ArcRpm, 0,7000);
lv_arc_set_value(ui_ArcRpm, 840);
lv_arc_set_bg_angles(ui_ArcRpm,90,320);
lv_obj_set_style_arc_color(ui_ArcRpm, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_arc_opa(ui_ArcRpm, 30, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_arc_width(ui_ArcRpm, 10, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_arc_rounded(ui_ArcRpm, true, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_set_style_arc_color(ui_ArcRpm, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR | LV_STATE_DEFAULT );
lv_obj_set_style_arc_opa(ui_ArcRpm, 255, LV_PART_INDICATOR| LV_STATE_DEFAULT);
lv_obj_set_style_arc_width(ui_ArcRpm, 10, LV_PART_INDICATOR| LV_STATE_DEFAULT);
lv_obj_set_style_arc_rounded(ui_ArcRpm, true, LV_PART_INDICATOR| LV_STATE_DEFAULT);

lv_obj_set_style_radius(ui_ArcRpm, 22, LV_PART_KNOB| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_ArcRpm, lv_color_hex(0xFFFFFF), LV_PART_KNOB | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_ArcRpm, 0, LV_PART_KNOB| LV_STATE_DEFAULT);
lv_obj_set_style_clip_corner(ui_ArcRpm, true, LV_PART_KNOB| LV_STATE_DEFAULT);

ui_rpm = lv_label_create(ui_ScreenMain);
lv_obj_set_width( ui_rpm, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_rpm, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_rpm, -14 );
lv_obj_set_y( ui_rpm, 88 );
lv_obj_set_align( ui_rpm, LV_ALIGN_CENTER );
lv_label_set_text(ui_rpm,"0000");
lv_obj_set_style_text_color(ui_rpm, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_rpm, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_align(ui_rpm, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_ArcClt = lv_arc_create(ui_ScreenMain);
lv_obj_set_width( ui_ArcClt, 230);
lv_obj_set_height( ui_ArcClt, 230);
lv_obj_set_align( ui_ArcClt, LV_ALIGN_CENTER );
lv_obj_remove_flag( ui_ArcClt, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_arc_set_range(ui_ArcClt, -40,120);
lv_arc_set_value(ui_ArcClt, 50);
lv_arc_set_bg_angles(ui_ArcClt,350,60);
lv_arc_set_mode(ui_ArcClt, LV_ARC_MODE_REVERSE);
lv_obj_set_style_arc_color(ui_ArcClt, lv_color_hex(0x4040FF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_arc_opa(ui_ArcClt, 50, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_arc_width(ui_ArcClt, 7, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_arc_rounded(ui_ArcClt, true, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_set_style_arc_color(ui_ArcClt, lv_color_hex(0x2ED4FF), LV_PART_INDICATOR | LV_STATE_DEFAULT );
lv_obj_set_style_arc_opa(ui_ArcClt, 255, LV_PART_INDICATOR| LV_STATE_DEFAULT);
lv_obj_set_style_arc_width(ui_ArcClt, 7, LV_PART_INDICATOR| LV_STATE_DEFAULT);
lv_obj_set_style_arc_rounded(ui_ArcClt, true, LV_PART_INDICATOR| LV_STATE_DEFAULT);

lv_obj_set_style_bg_color(ui_ArcClt, lv_color_hex(0xFFFFFF), LV_PART_KNOB | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_ArcClt, 0, LV_PART_KNOB| LV_STATE_DEFAULT);

ui_LabelRpm = lv_label_create(ui_ScreenMain);
lv_obj_set_width( ui_LabelRpm, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_LabelRpm, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_LabelRpm, 19 );
lv_obj_set_y( ui_LabelRpm, 89 );
lv_obj_set_align( ui_LabelRpm, LV_ALIGN_CENTER );
lv_label_set_text(ui_LabelRpm,"rpm");
lv_obj_set_style_text_color(ui_LabelRpm, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_LabelRpm, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_LabelRpm, &lv_font_montserrat_10, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Spd = lv_label_create(ui_ScreenMain);
lv_obj_set_width( ui_Spd, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Spd, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_Spd, 0 );
lv_obj_set_y( ui_Spd, -15 );
lv_obj_set_align( ui_Spd, LV_ALIGN_CENTER );
lv_label_set_text(ui_Spd,"000");
lv_obj_set_style_text_color(ui_Spd, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_Spd, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_align(ui_Spd, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_Spd, &lv_font_montserrat_38, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_LabelSpd = lv_label_create(ui_ScreenMain);
lv_obj_set_width( ui_LabelSpd, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_LabelSpd, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_LabelSpd, 5 );
lv_obj_set_y( ui_LabelSpd, 12 );
lv_obj_set_align( ui_LabelSpd, LV_ALIGN_CENTER );
lv_label_set_text(ui_LabelSpd,"km/h");
lv_obj_set_style_text_color(ui_LabelSpd, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_LabelSpd, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_LabelSpd, &lv_font_montserrat_12, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Clt = lv_label_create(ui_ScreenMain);
lv_obj_set_width( ui_Clt, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Clt, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_Clt, 76 );
lv_obj_set_y( ui_Clt, 38 );
lv_obj_set_align( ui_Clt, LV_ALIGN_CENTER );
lv_label_set_text(ui_Clt,"00");
lv_obj_set_style_text_color(ui_Clt, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_Clt, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_deg = lv_label_create(ui_ScreenMain);
lv_obj_set_width( ui_deg, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_deg, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_deg, 89 );
lv_obj_set_y( ui_deg, 38 );
lv_obj_set_align( ui_deg, LV_ALIGN_CENTER );
lv_label_set_text(ui_deg,"°");
lv_obj_set_style_text_color(ui_deg, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_deg, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Label8 = lv_label_create(ui_ScreenMain);
lv_obj_set_width( ui_Label8, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label8, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_Label8, 77 );
lv_obj_set_y( ui_Label8, 51 );
lv_obj_set_align( ui_Label8, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label8,"CLT");
lv_obj_set_style_text_color(ui_Label8, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_Label8, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_Label8, &lv_font_montserrat_10, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_add_event_cb(ui_ScreenMain, ui_event_ScreenMain, LV_EVENT_ALL, NULL);

}
