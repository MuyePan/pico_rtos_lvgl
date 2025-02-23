# Quick Start Guide For FreeRTOS and LVGL on Raspberry Pi Pico
This page shows steps to configure project for lastest FreeRTOS and LVGL on Rpi Pico.  
It is highly recommended to watch video tutorial as there is additional content in the video tutorial.  
Click the following image to view this tutorial on Youtube.  
[![Youtube video link](https://i.ytimg.com/vi/gTd6dm9ONSk/hqdefault.jpg)](//youtu.be/gTd6dm9ONSk "Youtube Video")

This tutorial works with Pico v2.1.1, FreeRTOS v11.0.1 and LVGL v9.2.2.

Install Raspberry Pi Pico VS Code extension and use it to create a new project.  

Download lastest FreeRTOS. Make sure FreeRTOS-Kernel folder exists.   
https://www.freertos.org/  

Create FreeRTOS_Kernel_import.cmake in project folder and copy the content of following link to this file.  
https://github.com/raspberrypi/pico-examples/blob/master/freertos/FreeRTOS_Kernel_import.cmake    

Create FreeRTOSConfig.h in project folder and copy the content of following link to this file.    
https://github.com/raspberrypi/pico-examples/blob/master/freertos/FreeRTOSConfig_examples_common.h  

Download lastest LVGL source code.  
https://github.com/lvgl/lvgl/tree/master  

Create lv_conf.h in project folder and copy the content of following link to this file.   
https://github.com/lvgl/lvgl/blob/master/lv_conf_template.h   

Find #if 0 in lv_conf.h and change the value to 1
```
#if 1 /* Set this to "1" to enable content */
```
Find LV_USE_OS and change the value to LV_OS_FREERTOS  
```
#define LV_USE_OS   LV_OS_FREERTOS
```

Download Pico-ResTouch-LCD-3.5 Demo.  
https://files.waveshare.com/upload/f/fc/Pico-ResTouch-LCD-X_X_Code.zip  
Copy LCD_Driver.h, LCD_Driver.c, LCD_Touch.h, LCD_Touch.c, DEV_Config.h and DEV_Config.c to project folder.   
Remove unnecessary content from the files.  
Change millisecond delay function to vTaskDelay in DEV_Config.c.  
```
void Driver_Delay_ms(uint32_t xms)
{
    vTaskDelay(pdMS_TO_TICKS(xms));
}
```
Modify microsecond delay function as following in DEV_Config.c
```
void Driver_Delay_us(uint32_t xus)
{
    uint32_t start = time_us_32();
    while ((time_us_32() - start) < xus) {
        // Busy-wait loop
    }
}
```

Modify TP_Scan in LCD_Touch.c
```
void TP_Scan(uint32_t *x, uint32_t *y)
{
    while(!TP_Read_TwiceADC(&sTP_DEV.Xpoint, &sTP_DEV.Ypoint));

    if(LCD_2_8==id){

        if(sTP_DEV.TP_Scan_Dir == R2L_D2U) {		//Converts the result to screen coordinates
            *x = sTP_DEV.fXfac * sTP_DEV.Xpoint + sTP_DEV.iXoff;
            *y = sTP_DEV.fYfac * sTP_DEV.Ypoint + sTP_DEV.iYoff;
        } else if(sTP_DEV.TP_Scan_Dir == L2R_U2D) {
            *x = sLCD_DIS.LCD_Dis_Column - sTP_DEV.fXfac * sTP_DEV.Xpoint - sTP_DEV.iXoff;
            *y = sLCD_DIS.LCD_Dis_Page - sTP_DEV.fYfac * sTP_DEV.Ypoint - sTP_DEV.iYoff;
        } else if(sTP_DEV.TP_Scan_Dir == U2D_R2L) {
            *x = sTP_DEV.fXfac * sTP_DEV.Ypoint + sTP_DEV.iXoff;
            *y = sTP_DEV.fYfac * sTP_DEV.Xpoint + sTP_DEV.iYoff;
        } else {
            *x = sLCD_DIS.LCD_Dis_Column - sTP_DEV.fXfac * sTP_DEV.Ypoint - sTP_DEV.iXoff;
            *y = sLCD_DIS.LCD_Dis_Page - sTP_DEV.fYfac * sTP_DEV.Xpoint - sTP_DEV.iYoff;
        }
    }else{
        //DEBUG("(Xad,Yad) = %d,%d\r\n",sTP_DEV.Xpoint,sTP_DEV.Ypoint);
        if(sTP_DEV.TP_Scan_Dir == R2L_D2U) {		//Converts the result to screen coordinates
            *x = sTP_DEV.fXfac * sTP_DEV.Xpoint + sTP_DEV.iXoff;
            *y = sTP_DEV.fYfac * sTP_DEV.Ypoint + sTP_DEV.iYoff;
        } else if(sTP_DEV.TP_Scan_Dir == L2R_U2D) {
            *x = sLCD_DIS.LCD_Dis_Column - sTP_DEV.fXfac * sTP_DEV.Xpoint - sTP_DEV.iXoff;
            *y = sLCD_DIS.LCD_Dis_Page - sTP_DEV.fYfac * sTP_DEV.Ypoint - sTP_DEV.iYoff;
        } else if(sTP_DEV.TP_Scan_Dir == U2D_R2L) {
            *x = sTP_DEV.fXfac * sTP_DEV.Ypoint + sTP_DEV.iXoff;
            *y = sTP_DEV.fYfac * sTP_DEV.Xpoint + sTP_DEV.iYoff;
        } else {
            *x = sLCD_DIS.LCD_Dis_Column - sTP_DEV.fXfac * sTP_DEV.Ypoint - sTP_DEV.iXoff;
            *y = sLCD_DIS.LCD_Dis_Page - sTP_DEV.fYfac * sTP_DEV.Xpoint - sTP_DEV.iYoff;
        }
    }   
}
```

Modify the end of function LCD_SetGramScanWay in LCD_Driver.c to enable hardware RGB565 swap
```
LCD_WriteData(MemoryAccessReg_Data & (~0x08));
```

Add following to CMakeList.txt
```
set(FREERTOS_KERNEL_PATH /home/pmy/Projects/FreeRTOS-Kernel)

include(FreeRTOS_Kernel_import.cmake)

add_subdirectory(lvgl)
```

Modify link libraries in CMakeList.txt
```
set(toolchainVersion 13_3_Rel1) # 14_1_rel1 not working

target_link_libraries(lvgl_rtos_test pico_stdlib FreeRTOS-Kernel-Heap4 lvgl hardware_spi)
```

Add LVGL gateway task in main.c
```
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
```

Callback to read input device data in main.c
```
void my_input_read(lv_indev_t * indev, lv_indev_data_t * data)
{
    if(!DEV_Digital_Read(TP_IRQ_PIN)) {
        TP_Scan(&data->point.x, &data->point.y);
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}
```

Flush callback for LVGL in main.c
```
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
``` 

main function in main.c
```
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
```
