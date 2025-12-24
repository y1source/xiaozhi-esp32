#include "wifi_board.h"
#include "codecs/box_audio_codec.h"
#include "display/lcd_display.h"
#include "display/emote_display.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "backlight.h"

#include <wifi_station.h>
#include <esp_log.h>

#include <driver/i2c_master.h>
#include <driver/i2c.h>
#include "i2c_device.h"
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_st77916.h>
#include "esp_lcd_touch_cst816s.h"
#include "touch.h"

#include "driver/temperature_sensor.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <cmath>
#include "assets/lang_config.h"

#include "bmi270_config.h"
#include "st77916_config.h"

#define TAG "EchoEar"

temperature_sensor_handle_t temp_sensor = NULL;

float tsens_value;
gpio_num_t AUDIO_I2S_GPIO_DIN = AUDIO_I2S_GPIO_DIN_1;
gpio_num_t AUDIO_CODEC_PA_PIN = AUDIO_CODEC_PA_PIN_1;
gpio_num_t QSPI_PIN_NUM_LCD_RST = QSPI_PIN_NUM_LCD_RST_1;
gpio_num_t TOUCH_PAD2 = TOUCH_PAD2_1;
gpio_num_t UART1_TX = UART1_TX_1;
gpio_num_t UART1_RX = UART1_RX_1;


class Bmi270 : public I2cDevice {
public:
    struct SensorData {
        float accel_x;
        float accel_y;
        float accel_z;
        float gyro_x;
        float gyro_y;
        float gyro_z;
    };

    // 添加晃动检测相关状态
    struct ShakeDetectionState {
        bool is_shaking;
        TickType_t shake_start_time;
        bool shake_detected;
    };

    Bmi270(i2c_master_bus_handle_t i2c_bus, uint8_t addr) 
        : I2cDevice(i2c_bus, addr) {
        shake_state_.is_shaking = false;
        shake_state_.shake_start_time = 0;
        shake_state_.shake_detected = false;
    }

    bool Initialize() {
        // 1.读取芯片ID（0x24）（检查通信是否正确）。接口即将配置为I2C，初始虚拟读取将其配置为SPI
        uint8_t chip_id = ReadReg(0x00);
        if (chip_id != 0x24) {
            ESP_LOGE(TAG, "BMI270 init failed. Chip ID: 0x%02X", chip_id);
            return false;
        }
        // 2.执行初始化序列
        WriteReg(0x7C, 0x00);           // 禁用电源管理配置高级节能模式
        vTaskDelay(pdMS_TO_TICKS(500)); // 等待500毫秒
        WriteReg(0x59, 0x00);           // 准备配置加载
        this->WriteRegs(0x5E, const_cast<uint8_t*>(bmi270_config_file), sizeof(bmi270_config_file));    // 配置文件数组向寄存器写入数据
        WriteReg(0x59, 0x01);           // 完成配置加载
        vTaskDelay(pdMS_TO_TICKS(500)); // 等待500毫秒

        // 3.检查初始化状态是否正确
        uint8_t status_id = ReadReg(0x21);
        if (status_id != 1) {
            ESP_LOGE(TAG, "BMI270 init failed. status_id: 0x%02X", status_id);
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(500));

        // 4.将设备配置为性能模式
        WriteReg(0x7D, 0x0E);   // PWR_CTRL： 获取加速度、陀螺仪和温度传感器数据。禁用辅助接口。
        WriteReg(0x40, 0xA8);   // ACC_CONF： 启用acc_filter_perf；将acc_bwp设置为正常模式；将acc_odr设置为100HZ
        WriteReg(0x42, 0xE9);   // GYR_CONF: 启用gyr_filter_perf；启用gyr_noise_perf；将gyr_bwp设置为正常模式；将gyr_odr设置为200Hz
        WriteReg(0x7C, 0x02);   // PWR_CONF: 禁用adv_power_save；启用fifo
        uint8_t buffer[12];
        ReadRegs(0x0c, buffer, sizeof(buffer));
        vTaskDelay(pdMS_TO_TICKS(500));
        
        ESP_LOGI(TAG, "BMI270 initialized");
        return true;
    }

    SensorData ReadData() {
        SensorData data = {0};
        uint8_t buffer[12];
        
        // 读取0x0C开始的12字节数据(加速度+陀螺仪) DATA_8 ~ DATA_19
        ReadRegs(0x0C, buffer, sizeof(buffer));
        // ESP_LOGI(TAG, "buffer ----------> %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], buffer[6], buffer[7], buffer[8], buffer[9], buffer[10], buffer[11]);
        
        // 解析加速度数据 (LSB = 8192/g)
        data.accel_x = (int16_t)(buffer[1] << 8 | buffer[0]) / 8192.0f;
        data.accel_y = (int16_t)(buffer[3] << 8 | buffer[2]) / 8192.0f;
        data.accel_z = (int16_t)(buffer[5] << 8 | buffer[4]) / 8192.0f;
        
        // 解析陀螺仪数据 (LSB = 16.384 dps)
        data.gyro_x = (int16_t)(buffer[7] << 8 | buffer[6]) / 16.384f;
        data.gyro_y = (int16_t)(buffer[9] << 8 | buffer[8]) / 16.384f;
        data.gyro_z = (int16_t)(buffer[11] << 8 | buffer[10]) / 16.384f;

        // // 读取X轴数据 (Ratex = (DATA_15<<8 + DATA_14) - [GYR_CAS.factor_zx × (DATA_19<<8 + DATA_18)] / 2^9)
        // // 提取低7位并执行符号扩展
        // data.gyro_x = (int16_t)((buffer[7] << 8) | buffer[6]);
        // // 读取Y轴数据 (Ratey = DATA_17<<8 + DATA_16)
        // data.gyro_y = (int16_t)((buffer[9] << 8) | buffer[8]);
        // // 读取Z轴数据 (Ratez = DATA_19<<8 + DATA_18)
        // data.gyro_z = (int16_t)((buffer[11] << 8) | buffer[10]);

        return data;
    }

    // 检测晃动的方法
    bool CheckForShake(const SensorData& data, TickType_t current_time) {
        const float shake_threshold = 100.0f; // 晃动阈值(度/秒)
        const TickType_t min_shake_duration = pdMS_TO_TICKS(1000); // 最小晃动持续时间(1秒)
        
        // 检查是否超过阈值
        bool is_currently_shaking = (fabs(data.gyro_x) > shake_threshold) ||
                                   (fabs(data.gyro_y) > shake_threshold) ||
                                   (fabs(data.gyro_z) > shake_threshold);
        
        if (is_currently_shaking) {
            if (!shake_state_.is_shaking) {
                // 开始晃动
                shake_state_.is_shaking = true;
                shake_state_.shake_start_time = current_time;
                ESP_LOGI(TAG, "Shake started");
            } else {
                // 持续晃动，检查是否达到1秒
                if (!shake_state_.shake_detected && 
                    (current_time - shake_state_.shake_start_time >= min_shake_duration)) {
                    shake_state_.shake_detected = true;
                    ESP_LOGI(TAG, "Real shake detected! (lasted more than 1 seconds)");

                    return true;
                }
            }
        } else if (shake_state_.is_shaking) {
            // 晃动结束
            TickType_t shake_duration = current_time - shake_state_.shake_start_time;
            ESP_LOGI(TAG, "Shake ended, duration: %lu ms", 
                    (unsigned long)(shake_duration * portTICK_PERIOD_MS));
            
            // 重置状态
            shake_state_.is_shaking = false;
            shake_state_.shake_detected = false;
        }
        
        return false;
    }

    static void SensorTask(void* arg) {
        Bmi270* sensor = static_cast<Bmi270*>(arg);
        const TickType_t delay = pdMS_TO_TICKS(200);

        // 等待应用程序和协议初始化完成
        while (!Application::GetInstance().IsStarted()) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        while (true) {
            TickType_t current_time = xTaskGetTickCount();
            auto data = sensor->ReadData();
            
            // 检测晃动
            if (sensor->CheckForShake(data, current_time)) {
                ESP_LOGI(TAG, "Performing action for real shake detection");

                Application& app = Application::GetInstance();
                app.SendSensorEvent("检测到摇晃");

                auto& board = Board::GetInstance();
                auto display = board.GetDisplay();

                // 改变状态、表情
                display->SetStatus(Lang::Strings::STANDBY);
                display->SetEmotion("confused");

                vTaskDelay(pdMS_TO_TICKS(5000));

                // 恢复正常
                display->SetEmotion("idle");
            }
            
            // // 输出传感器数据（可选，用于调试）
            // ESP_LOGI(TAG, "Accel: X=%.2fg Y=%.2fg Z=%.2fg | Gyro: X=%.2f°/s Y=%.2f°/s Z=%.2f°/s",
            //         data.accel_x, data.accel_y, data.accel_z,
            //         data.gyro_x, data.gyro_y, data.gyro_z);
            
            vTaskDelay(delay);
        }
    }
private:
    ShakeDetectionState shake_state_;
};


class Charge : public I2cDevice {
public:
    Charge(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : I2cDevice(i2c_bus, addr)
    {
        read_buffer_ = new uint8_t[8];
    }
    ~Charge()
    {
        delete[] read_buffer_;
    }
    void Printcharge()
    {
        ReadRegs(0x08, read_buffer_, 2);
        ReadRegs(0x0c, read_buffer_ + 2, 2);
        ESP_ERROR_CHECK(temperature_sensor_get_celsius(temp_sensor, &tsens_value));

        int16_t voltage = static_cast<uint16_t>(read_buffer_[1] << 8 | read_buffer_[0]);
        int16_t current = static_cast<int16_t>(read_buffer_[3] << 8 | read_buffer_[2]);
        
        // Use the variables to avoid warnings (can be removed if actual implementation uses them)
        (void)voltage;
        (void)current;
    }
    static void TaskFunction(void *pvParameters)
    {
        Charge* charge = static_cast<Charge*>(pvParameters);
        while (true) {
            charge->Printcharge();
            vTaskDelay(pdMS_TO_TICKS(300));
        }
    }

private:
    uint8_t* read_buffer_ = nullptr;
};

class Cst816s : public I2cDevice {
public:
    struct TouchPoint_t {
        int num = 0;
        int x = -1;
        int y = -1;
    };

    enum TouchEvent {
        TOUCH_NONE,
        TOUCH_PRESS,
        TOUCH_RELEASE,
        TOUCH_HOLD
    };

    Cst816s(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : I2cDevice(i2c_bus, addr)
    {
        read_buffer_ = new uint8_t[6];
        was_touched_ = false;
        press_count_ = 0;

        // Create touch interrupt semaphore
        touch_isr_mux_ = xSemaphoreCreateBinary();
        if (touch_isr_mux_ == NULL) {
            ESP_LOGE("EchoEar", "Failed to create touch semaphore");
        }
    }

    ~Cst816s()
    {
        delete[] read_buffer_;

        // Delete semaphore if it exists
        if (touch_isr_mux_ != NULL) {
            vSemaphoreDelete(touch_isr_mux_);
            touch_isr_mux_ = NULL;
        }
    }

    void UpdateTouchPoint()
    {
        ReadRegs(0x02, read_buffer_, 6);
        tp_.num = read_buffer_[0] & 0x0F;
        tp_.x = ((read_buffer_[1] & 0x0F) << 8) | read_buffer_[2];
        tp_.y = ((read_buffer_[3] & 0x0F) << 8) | read_buffer_[4];
    }

    const TouchPoint_t &GetTouchPoint()
    {
        return tp_;
    }

    TouchEvent CheckTouchEvent()
    {
        bool is_touched = (tp_.num > 0);
        TouchEvent event = TOUCH_NONE;

        if (is_touched && !was_touched_) {
            // Press event (transition from not touched to touched)
            press_count_++;
            event = TOUCH_PRESS;
            ESP_LOGI("EchoEar", "TOUCH PRESS - count: %d, x: %d, y: %d", press_count_, tp_.x, tp_.y);
        } else if (!is_touched && was_touched_) {
            // Release event (transition from touched to not touched)
            event = TOUCH_RELEASE;
            ESP_LOGI("EchoEar", "TOUCH RELEASE - total presses: %d", press_count_);
        } else if (is_touched && was_touched_) {
            // Continuous touch (hold)
            event = TOUCH_HOLD;
            ESP_LOGD("EchoEar", "TOUCH HOLD - x: %d, y: %d", tp_.x, tp_.y);
        }

        // Update previous state
        was_touched_ = is_touched;
        return event;
    }

    int GetPressCount() const
    {
        return press_count_;
    }

    void ResetPressCount()
    {
        press_count_ = 0;
    }

    // Semaphore management methods
    SemaphoreHandle_t GetTouchSemaphore()
    {
        return touch_isr_mux_;
    }

    bool WaitForTouchEvent(TickType_t timeout = portMAX_DELAY)
    {
        if (touch_isr_mux_ != NULL) {
            return xSemaphoreTake(touch_isr_mux_, timeout) == pdTRUE;
        }
        return false;
    }

    void NotifyTouchEvent()
    {
        if (touch_isr_mux_ != NULL) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(touch_isr_mux_, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }

private:
    uint8_t* read_buffer_ = nullptr;
    TouchPoint_t tp_;

    // Touch state tracking
    bool was_touched_;
    int press_count_;

    // Touch interrupt semaphore
    SemaphoreHandle_t touch_isr_mux_;
};

class EspS3Cat : public WifiBoard {
private:
    i2c_master_bus_handle_t i2c_bus_;
    Cst816s* cst816s_;
    Charge* charge_;
    Button boot_button_;
    Display* display_ = nullptr;
    PwmBacklight* backlight_ = nullptr;
    esp_timer_handle_t touchpad_timer_;
    esp_lcd_touch_handle_t tp;   // LCD touch handle

    Bmi270* bmi270_;

    void InitializeI2c()
    {
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));

        temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 50);
        ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor_config, &temp_sensor));
        ESP_ERROR_CHECK(temperature_sensor_enable(temp_sensor));

    }

    // 添加传感器初始化
    void InitializeSensors() {
        // 初始化BMI270 (I2C地址0x68)
        bmi270_ = new Bmi270(i2c_bus_, 0x68);
        if (bmi270_->Initialize()) {
            xTaskCreatePinnedToCore(Bmi270::SensorTask, "bmi270_task", 4096, bmi270_, 5, NULL, 1);
        } else {
            ESP_LOGE(TAG, "Failed to initialize BMI270");
        }

    }

    uint8_t DetectPcbVersion()
    {
        esp_err_t ret = i2c_master_probe(i2c_bus_, 0x18, 100);
        uint8_t pcb_verison = 0;
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "PCB verison V1.0");
            pcb_verison = 0;
        } else {
            gpio_config_t gpio_conf = {
                .pin_bit_mask = (1ULL << GPIO_NUM_48),
                .mode = GPIO_MODE_OUTPUT,
                .pull_up_en = GPIO_PULLUP_DISABLE,
                .pull_down_en = GPIO_PULLDOWN_DISABLE,
                .intr_type = GPIO_INTR_DISABLE
            };
            ESP_ERROR_CHECK(gpio_config(&gpio_conf));
            ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_48, 1));
            vTaskDelay(pdMS_TO_TICKS(100));
            ret = i2c_master_probe(i2c_bus_, 0x18, 100);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "PCB verison V1.2");
                pcb_verison = 1;
                AUDIO_I2S_GPIO_DIN = AUDIO_I2S_GPIO_DIN_2;
                AUDIO_CODEC_PA_PIN = AUDIO_CODEC_PA_PIN_2;
                QSPI_PIN_NUM_LCD_RST = QSPI_PIN_NUM_LCD_RST_2;
                TOUCH_PAD2 = TOUCH_PAD2_2;
                UART1_TX = UART1_TX_2;
                UART1_RX = UART1_RX_2;
            } else {
                ESP_LOGE(TAG, "PCB version detection error");

            }
        }
        return pcb_verison;
    }

    static void touch_isr_callback(void* arg)
    {
        Cst816s* touchpad = static_cast<Cst816s*>(arg);
        if (touchpad != nullptr) {
            touchpad->NotifyTouchEvent();
        }
    }

    static void touch_event_task(void* arg)
    {
        Cst816s* touchpad = static_cast<Cst816s*>(arg);
        if (touchpad == nullptr) {
            ESP_LOGE(TAG, "Invalid touchpad pointer in touch_event_task");
            vTaskDelete(NULL);
            return;
        }

        while (true) {
            if (touchpad->WaitForTouchEvent()) {
                auto &app = Application::GetInstance();
                auto &board = (EspS3Cat &)Board::GetInstance();

                ESP_LOGI(TAG, "Touch event, TP_PIN_NUM_INT: %d", gpio_get_level(TP_PIN_NUM_INT));
                touchpad->UpdateTouchPoint();
                auto touch_event = touchpad->CheckTouchEvent();

                if (touch_event == Cst816s::TOUCH_RELEASE) {
                    if (app.GetDeviceState() == kDeviceStateStarting &&
                            !WifiStation::GetInstance().IsConnected()) {
                        board.ResetWifiConfiguration();
                    } else {
                        app.ToggleChatState();
                    }
                }
            }
        }
    }
    
    void InitializeCharge()
    {
        charge_ = new Charge(i2c_bus_, 0x55);
        xTaskCreatePinnedToCore(Charge::TaskFunction, "batterydecTask", 3 * 1024, charge_, 6, NULL, 0);
    }

    void InitializeCst816sTouchPad()
    {
        cst816s_ = new Cst816s(i2c_bus_, 0x15);

        xTaskCreatePinnedToCore(touch_event_task, "touch_task", 4 * 1024, cst816s_, 5, NULL, 1);

        const gpio_config_t int_gpio_config = {
            .pin_bit_mask = (1ULL << TP_PIN_NUM_INT),
            .mode = GPIO_MODE_INPUT,
            // .intr_type = GPIO_INTR_NEGEDGE
            .intr_type = GPIO_INTR_ANYEDGE
        };
        gpio_config(&int_gpio_config);
        gpio_install_isr_service(0);
        gpio_intr_enable(TP_PIN_NUM_INT);
        gpio_isr_handler_add(TP_PIN_NUM_INT, EspS3Cat::touch_isr_callback, cst816s_);
    }

    void InitializeSpi()
    {
        const spi_bus_config_t bus_config = TAIJIPI_ST77916_PANEL_BUS_QSPI_CONFIG(QSPI_PIN_NUM_LCD_PCLK,
                                                                                  QSPI_PIN_NUM_LCD_DATA0,
                                                                                  QSPI_PIN_NUM_LCD_DATA1,
                                                                                  QSPI_PIN_NUM_LCD_DATA2,
                                                                                  QSPI_PIN_NUM_LCD_DATA3,
                                                                                  QSPI_LCD_H_RES * 80 * sizeof(uint16_t));
        ESP_ERROR_CHECK(spi_bus_initialize(QSPI_LCD_HOST, &bus_config, SPI_DMA_CH_AUTO));
    }

    void Initializest77916Display(uint8_t pcb_verison)
    {

        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;

        const esp_lcd_panel_io_spi_config_t io_config = ST77916_PANEL_IO_QSPI_CONFIG(QSPI_PIN_NUM_LCD_CS, NULL, NULL);
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)QSPI_LCD_HOST, &io_config, &panel_io));
        st77916_vendor_config_t vendor_config = {
            .init_cmds = vendor_specific_init_yysj,
            .init_cmds_size = sizeof(vendor_specific_init_yysj) / sizeof(st77916_lcd_init_cmd_t),
            .flags = {
                .use_qspi_interface = 1,
            },
        };
        const esp_lcd_panel_dev_config_t panel_config = {
            .reset_gpio_num = QSPI_PIN_NUM_LCD_RST,
            .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
            .bits_per_pixel = QSPI_LCD_BIT_PER_PIXEL,
            .flags = {
                .reset_active_high = pcb_verison,
            },
            .vendor_config = &vendor_config,
        };
        ESP_ERROR_CHECK(esp_lcd_new_panel_st77916(panel_io, &panel_config, &panel));

        esp_lcd_panel_reset(panel);
        esp_lcd_panel_init(panel);
        esp_lcd_panel_disp_on_off(panel, true);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);

#if CONFIG_USE_EMOTE_MESSAGE_STYLE
        display_ = new emote::EmoteDisplay(panel, panel_io, DISPLAY_WIDTH, DISPLAY_HEIGHT);
#else
        display_ = new SpiLcdDisplay(panel_io, panel,
            DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
#endif
        backlight_ = new PwmBacklight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
        backlight_->RestoreBrightness();
    }

    void InitializeButtons()
    {
        boot_button_.OnClick([this]() {
            auto &app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting && !WifiStation::GetInstance().IsConnected()) {
                ESP_LOGI(TAG, "Boot button pressed, enter WiFi configuration mode");
                ResetWifiConfiguration();
            }
            app.ToggleChatState();
        });
        gpio_config_t power_gpio_config = {
            .pin_bit_mask = (BIT64(POWER_CTRL)),
            .mode = GPIO_MODE_OUTPUT,

        };
        ESP_ERROR_CHECK(gpio_config(&power_gpio_config));

        gpio_set_level(POWER_CTRL, 0);
    }

public:
    EspS3Cat() : boot_button_(BOOT_BUTTON_GPIO)
    {
        InitializeI2c();
        uint8_t pcb_verison = DetectPcbVersion();
        InitializeCharge();
        InitializeCst816sTouchPad();

        InitializeSensors();  // 新增传感器初始化

        InitializeSpi();
        Initializest77916Display(pcb_verison);
        InitializeButtons();
    }

    virtual AudioCodec* GetAudioCodec() override
    {
        static BoxAudioCodec audio_codec(
            i2c_bus_,
            AUDIO_INPUT_SAMPLE_RATE,
            AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK,
            AUDIO_I2S_GPIO_BCLK,
            AUDIO_I2S_GPIO_WS,
            AUDIO_I2S_GPIO_DOUT,
            AUDIO_I2S_GPIO_DIN,
            AUDIO_CODEC_PA_PIN,
            AUDIO_CODEC_ES8311_ADDR,
            AUDIO_CODEC_ES7210_ADDR,
            AUDIO_INPUT_REFERENCE);
        return &audio_codec;
    }

    virtual Display* GetDisplay() override
    {
        return display_;
    }

    Cst816s* GetTouchpad()
    {
        return cst816s_;
    }

    virtual Backlight* GetBacklight() override
    {
        return backlight_;
    }
};

DECLARE_BOARD(EspS3Cat);
