# Quick Start Guide: FreeRTOS and LVGL on Raspberry Pi Pico

This guide provides step-by-step instructions to set up a project with FreeRTOS and LVGL on a Raspberry Pi Pico, compatible with the latest versions as of February 23, 2025. For extra tips and visuals, check out the companion video tutorial on YouTube:  
[![YouTube Video](https://i.ytimg.com/vi/cfpH3ro9d80/hqdefault.jpg)](https://youtu.be/cfpH3ro9d80 "YouTube Video")

## Supported Versions and Hardware
- **Raspberry Pi Pico**: v2.1.1
- **FreeRTOS**: v11.0.1
- **LVGL**: v9.2.2
- **Display**: 3.5" TFT LCD (ILI9488) with XPT2046 touch controller
- **Reference**: [Waveshare Pico-ResTouch-LCD-3.5 Wiki](https://www.waveshare.com/wiki/Pico-ResTouch-LCD-3.5)

## Prerequisites
- Install the **Raspberry Pi Pico VS Code Extension** and create a new project using it.

---

## Step 1: Set Up FreeRTOS

1. **Download FreeRTOS**  
   - Grab the latest version from [freertos.org](https://www.freertos.org/).  
   - Confirm the `FreeRTOS-Kernel` folder is included in the download.

2. **Create `FreeRTOS_Kernel_import.cmake`**  
   - Add this file to your project folder.  
   - Copy the contents from [this GitHub link](https://github.com/raspberrypi/pico-examples/blob/master/freertos/FreeRTOS_Kernel_import.cmake).

3. **Create `FreeRTOSConfig.h`**  
   - Add this file to your project folder.  
   - Copy the contents from [this GitHub link](https://github.com/raspberrypi/pico-examples/blob/master/freertos/FreeRTOSConfig_examples_common.h).

---

## Step 2: Set Up LVGL

1. **Download LVGL**  
   - Download the latest source code from [github.com/lvgl/lvgl](https://github.com/lvgl/lvgl/tree/master).

2. **Link LVGL Source Folder**  
   - Move the downloaded `lvgl` folder into your project directory (e.g., next to your `src` folder).  
   - Alternatively, create a symbolic link to the LVGL source folder:  
     ```bash
     ln -s /path/to/lvgl lvgl
     ```  
   - This step ensures CMake can locate the LVGL source when building.

3. **Create `lv_conf.h`**  
   - Add this file to your project folder.  
   - Copy the contents from [this template](https://github.com/lvgl/lvgl/blob/master/lv_conf_template.h).  
   - Edit the file:  
     - Enable the content by changing `#if 0` to `#if 1`:  
       ```c
       #if 1 /* Set this to "1" to enable content */
       ```  
     - Configure LVGL to use FreeRTOS:  
       ```c
       #define LV_USE_OS   LV_OS_FREERTOS
       ```

---

## Step 3: Add Display and Touch Drivers

1. **Download Demo Code**  
   - Download the Pico-ResTouch-LCD-3.5 demo from [this link](https://files.waveshare.com/upload/f/fc/Pico-ResTouch-LCD-X_X_Code.zip).

2. **Copy Files**  
   - Add these files to your project folder: `LCD_Driver.h`, `LCD_Driver.c`, `LCD_Touch.h`, `LCD_Touch.c`, `DEV_Config.h`, `DEV_Config.c`.  
   - Trim any unnecessary code from these files.

3. **Update Delay Functions in `DEV_Config.c`**  
   - Modify the millisecond delay function:  
     ```c
     void Driver_Delay_ms(uint32_t xms) {
         vTaskDelay(pdMS_TO_TICKS(xms));
     }
     ```  
   - Modify the microsecond delay function:  
     ```c
     void Driver_Delay_us(uint32_t xus) {
         uint32_t start = time_us_32();
         while ((time_us_32() - start) < xus) {
             // Busy-wait loop
         }
     }
     ```

4. **Modify `TP_Scan` in `LCD_Touch.c`**  
   - Update this function to correctly map touch coordinates:  
     ```c
     void TP_Scan(uint32_t *x, uint32_t *y) {
         while (!TP_Read_TwiceADC(&sTP_DEV.Xpoint, &sTP_DEV.Ypoint));

         if (LCD_2_8 == id) {
             if (sTP_DEV.TP_Scan_Dir == R2L_D2U) {
                 *x = sTP_DEV.fXfac * sTP_DEV.Xpoint + sTP_DEV.iXoff;
                 *y = sTP_DEV.fYfac * sTP_DEV.Ypoint + sTP_DEV.iYoff;
             } else if (sTP_DEV.TP_Scan_Dir == L2R_U2D) {
                 *x = sLCD_DIS.LCD_Dis_Column - sTP_DEV.fXfac * sTP_DEV.Xpoint - sTP_DEV.iXoff;
                 *y = sLCD_DIS.LCD_Dis_Page - sTP_DEV.fYfac * sTP_DEV.Ypoint - sTP_DEV.iYoff;
             } else if (sTP_DEV.TP_Scan_Dir == U2D_R2L) {
                 *x = sTP_DEV.fXfac * sTP_DEV.Ypoint + sTP_DEV.iXoff;
                 *y = sTP_DEV.fYfac * sTP_DEV.Xpoint + sTP_DEV.iYoff;
             } else {
                 *x = sLCD_DIS.LCD_Dis_Column - sTP_DEV.fXfac * sTP_DEV.Ypoint - sTP_DEV.iXoff;
                 *y = sLCD_DIS.LCD_Dis_Page - sTP_DEV.fYfac * sTP_DEV.Xpoint - sTP_DEV.iYoff;
             }
         } else {
             if (sTP_DEV.TP_Scan_Dir == R2L_D2U) {
                 *x = sTP_DEV.fXfac * sTP_DEV.Xpoint + sTP_DEV.iXoff;
                 *y = sTP_DEV.fYfac * sTP_DEV.Ypoint + sTP_DEV.iYoff;
             } else if (sTP_DEV.TP_Scan_Dir == L2R_U2D) {
                 *x = sLCD_DIS.LCD_Dis_Column - sTP_DEV.fXfac * sTP_DEV.Xpoint - sTP_DEV.iXoff;
                 *y = sLCD_DIS.LCD_Dis_Page - sTP_DEV.fYfac * sTP_DEV.Ypoint - sTP_DEV.iYoff;
             } else if (sTP_DEV.TP_Scan_Dir == U2D_R2L) {
                 *x = sTP_DEV.fXfac * sTP_DEV.Ypoint + sTP_DEV.iXoff;
                 *y = sTP_DEV.fYfac * sTP_DEV.Xpoint + sTP_DEV.iYoff;
             } else {
                 *x = sLCD_DIS.LCD_Dis_Column - sTP_DEV.fXfac * sTP_DEV.Ypoint - sTP_DEV.iXoff;
                 *y = sLCD_DIS.LCD_Dis_Page - sTP_DEV.fYfac * sTP_DEV.Xpoint - sTP_DEV.iYoff;
             }
         }
     }
     ```

5. **Enable Hardware RGB565 Swap in `LCD_Driver.c`**  
   - Modify last line of `LCD_SetGramScanWay`:  
     ```c
     LCD_WriteData(MemoryAccessReg_Data & (~0x08));
     ```

---

## Step 4: Configure CMake

1. **Update `CMakeLists.txt`**  
   - Specify the FreeRTOS path and include LVGL:  
     ```cmake
     set(FREERTOS_KERNEL_PATH /home/pmy/Projects/FreeRTOS-Kernel)
     include(FreeRTOS_Kernel_import.cmake)
     add_subdirectory(lvgl)
     ```  
   - Link the required libraries (use toolchain 13_3_Rel1 due to compatibility issues with 14_1_rel1):  
     ```cmake
     set(toolchainVersion 13_3_Rel1)
     target_link_libraries(lvgl_rtos_test pico_stdlib FreeRTOS-Kernel-Heap4 lvgl hardware_spi)
     ```

---

## Step 5: Implement Main Code in `main.c`

1. **LVGL Gateway Task**  
   - Create a task to handle LVGL operations:  
     ```c
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
   - Route all LVGL API calls through this task.

2. **Input Device Callback**  
   - Read touch input data:  
     ```c
     void my_input_read(lv_indev_t *indev, lv_indev_data_t *data) {
         if (!DEV_Digital_Read(TP_IRQ_PIN)) {
             TP_Scan(&data->point.x, &data->point.y);
             data->state = LV_INDEV_STATE_PRESSED;
         } else {
             data->state = LV_INDEV_STATE_RELEASED;
         }
     }
     ```

3. **Flush Callback for LVGL**  
   - Render the display buffer:  
     ```c
     void my_flush_cb(lv_display_t *display, const lv_area_t *area, uint8_t *px_map) {
         LCD_SetWindow(area->x1, area->y1, area->x2 + 1, area->y2 + 1);
         uint32_t DataLen = (area->x2 - area->x1 + 1) * (area->y2 - area->y1 + 1) * 2;
         DEV_Digital_Write(LCD_DC_PIN, 1);
         DEV_Digital_Write(LCD_CS_PIN, 0);
         spi_write_blocking(spi1, px_map, DataLen);
         DEV_Digital_Write(LCD_CS_PIN, 1);
         lv_display_flush_ready(display); // Signal LVGL that flushing is complete
     }
     ```

4. **Main Function**  
   - Set up and start the application:  
     ```c
     int main() {
         System_Init();
         lv_init();
         lv_tick_set_cb(xTaskGetTickCount);

         // Initialize display
         lv_display_t *display = lv_display_create(TFT_WIDTH, TFT_HEIGHT);
         lv_display_set_color_format(display, LV_COLOR_FORMAT_RGB565);
         lv_disp_set_default(display);

         // Configure draw buffer
         uint32_t draw_buf_size = TFT_WIDTH * TFT_HEIGHT / 10 * LV_COLOR_FORMAT_GET_SIZE(LV_COLOR_FORMAT_RGB565);
         uint8_t *draw_buf = pvPortMalloc(draw_buf_size);
         lv_display_set_buffers(display, draw_buf, NULL, draw_buf_size, LV_DISPLAY_RENDER_MODE_PARTIAL);

         // Assign flush callback
         lv_display_set_flush_cb(display, my_flush_cb);

         // Set up input device
         lv_indev_t *indev = lv_indev_create();
         lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
         lv_indev_set_read_cb(indev, my_input_read);

         init_gui();

         // Launch LVGL task
         xTaskCreate(lvgl_task, "lvgl_task", 1024, NULL, 1, NULL);
         vTaskStartScheduler();

         panic("RTOS kernel not running!"); // Execution should never reach here
     }
     ```

---

## Notes
- Update `/home/pmy/Projects/FreeRTOS-Kernel` in `CMakeLists.txt` with your actual FreeRTOS path.
- If using a symbolic link for LVGL, replace `/path/to/lvgl` with the correct path to your LVGL folder.
- Define `TFT_WIDTH` and `TFT_HEIGHT` to match your display resolution (e.g., 480x320 for the 3.5" LCD).
- Implement `init_gui()` with your custom UI code before starting the scheduler.

This guide ensures you can successfully run FreeRTOS and LVGL on your Raspberry Pi Pico with the specified hardware. Happy coding!

---
