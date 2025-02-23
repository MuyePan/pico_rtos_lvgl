#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"

#include "DEV_Config.h"
#include "LCD_Driver.h"
#include "LCD_Touch.h"
#include "lvgl/lvgl.h"

#define TFT_WIDTH 320
#define TFT_HEIGHT 480

void lvgl_task(void *pvParameters) {
	LCD_SCAN_DIR lcd_scan_dir = SCAN_DIR_DFT;
	LCD_Init(lcd_scan_dir, 800);
	TP_Init(lcd_scan_dir);
    TP_GetAdFac();

    while (1) {
        uint32_t time_till_next = lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(time_till_next));
    }
}

void my_input_read(lv_indev_t * indev, lv_indev_data_t * data)
{
    if(!DEV_Digital_Read(TP_IRQ_PIN)) {
		TP_Scan(&data->point.x, &data->point.y);
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

void my_flush_cb(lv_display_t * display, const lv_area_t * area, uint8_t * px_map)
{
	LCD_SetWindow(area->x1, area->y1, area->x2 + 1, area->y2 + 1);

	// ideally use by hardware
	// lv_draw_sw_rgb565_swap(px_map, TFT_WIDTH * TFT_HEIGHT / 10);
	uint32_t DataLen = (area->x2 - area->x1 + 1) * (area->y2 - area->y1 + 1) * 2;
    DEV_Digital_Write(LCD_DC_PIN,1);
    DEV_Digital_Write(LCD_CS_PIN,0);
	spi_write_blocking(spi1, px_map, DataLen);
	DEV_Digital_Write(LCD_CS_PIN,1);

    /* IMPORTANT!!!
     * Inform LVGL that flushing is complete so buffer can be modified again. */
    lv_display_flush_ready(display);
}

void init_gui() {
	lv_obj_t *scr = lv_scr_act();  // Get the current screen
	lv_obj_set_style_bg_color(scr, lv_color_make(0xFF, 0xFF, 0xFF), LV_PART_MAIN); // Set to red
	lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN); // Make it fully REDopaque

	// Create 4 buttons
    lv_obj_t *btn1 = lv_btn_create(scr);  // Create the first button on the screen
    lv_obj_set_size(btn1, 120, 50); // Set button size (width: 120px, height: 50px)
    lv_obj_align(btn1, LV_ALIGN_TOP_MID, 0, 20); // Align the first button to the top-middle of the screen

    lv_obj_t *btn2 = lv_btn_create(scr);  // Create the second button on the screen
    lv_obj_set_size(btn2, 120, 50); // Set button size (width: 120px, height: 50px)
    lv_obj_align_to(btn2, btn1, LV_ALIGN_OUT_BOTTOM_MID, 0, 20); // Align the second button below the first button

    lv_obj_t *btn3 = lv_btn_create(scr);  // Create the third button on the screen
    lv_obj_set_size(btn3, 120, 50); // Set button size (width: 120px, height: 50px)
    lv_obj_align_to(btn3, btn2, LV_ALIGN_OUT_BOTTOM_MID, 0, 20); // Align the third button below the second button

    lv_obj_t *btn4 = lv_btn_create(scr);  // Create the fourth button on the screen
    lv_obj_set_size(btn4, 120, 50); // Set button size (width: 120px, height: 50px)
    lv_obj_align_to(btn4, btn3, LV_ALIGN_OUT_BOTTOM_MID, 0, 20); // Align the fourth button below the third button

    // Create labels on the buttons
    lv_obj_t *label1 = lv_label_create(btn1);  // Create label on the first button
    lv_label_set_text(label1, "Button 1");  // Set label text
    lv_obj_align(label1, LV_ALIGN_CENTER, 0, 0); // Align label to the center of the button

    lv_obj_t *label2 = lv_label_create(btn2);  // Create label on the second button
    lv_label_set_text(label2, "Button 2");  // Set label text
    lv_obj_align(label2, LV_ALIGN_CENTER, 0, 0); // Align label to the center of the button

    lv_obj_t *label3 = lv_label_create(btn3);  // Create label on the third button
    lv_label_set_text(label3, "Button 3");  // Set label text
    lv_obj_align(label3, LV_ALIGN_CENTER, 0, 0); // Align label to the center of the button

    lv_obj_t *label4 = lv_label_create(btn4);  // Create label on the fourth button
    lv_label_set_text(label4, "Button 4");  // Set label text
    lv_obj_align(label4, LV_ALIGN_CENTER, 0, 0); // Align label to the center of the button
}

int main()
{
    System_Init();

	// Initialize LVGL
	lv_init();

    // Connect Tick Interface
	lv_tick_set_cb(xTaskGetTickCount);

	// Create a display
	lv_display_t * display = lv_display_create(TFT_WIDTH, TFT_HEIGHT);
	lv_display_set_color_format(display, LV_COLOR_FORMAT_RGB565);
	lv_disp_set_default(display);

	// Set draw buffer for display
    uint32_t draw_buf_size = TFT_WIDTH * TFT_HEIGHT / 10 * LV_COLOR_FORMAT_GET_SIZE(LV_COLOR_FORMAT_RGB565);
    uint8_t *draw_buf = pvPortMalloc(draw_buf_size);
	lv_display_set_buffers(display, draw_buf, NULL, draw_buf_size, LV_DISPLAY_RENDER_MODE_PARTIAL);

	// Set flush callback
	lv_display_set_flush_cb(display, my_flush_cb);

	/* Create input device connected to Default Display. */
	lv_indev_t * indev = lv_indev_create();        
	lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER); 
	lv_indev_set_read_cb(indev, my_input_read);

    init_gui();

    // Create the Hello World task
    xTaskCreate(lvgl_task, "lvgl_task", 1024, NULL, 1, NULL);    

    // Start FreeRTOS scheduler
    vTaskStartScheduler();

    panic("RTOS kernel not running!"); // we shouldn't get here
}
