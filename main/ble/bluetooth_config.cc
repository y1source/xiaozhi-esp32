#include "bluetooth_config.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_spp_api.h"
#include "esp_log.h"
#include <cstring>

#define TAG "BluetoothConfig"
#define SPP_SERVER_NAME "ESP32-WiFi-Config"
#define SPP_APP_ID 0x123456

struct BluetoothConfig::BluetoothData {
    esp_spp_cb_t spp_callback;
    bool spp_connected;
    std::string received_data;
};

BluetoothConfig::BluetoothConfig() 
    : is_provisioning_(false), bt_data_(std::make_unique<BluetoothData>()) {
    bt_data_->spp_connected = false;
}

BluetoothConfig::~BluetoothConfig() {
    StopProvisioning();
}

BluetoothConfig& BluetoothConfig::GetInstance() {
    static BluetoothConfig instance;
    return instance;
}

static void esp_spp_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
    auto& instance = BluetoothConfig::GetInstance();
    
    switch (event) {
        case ESP_SPP_INIT_EVT:
            ESP_LOGI(TAG, "SPP initialized");
            esp_spp_start_srv(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_SLAVE, 0, SPP_SERVER_NAME);
            break;
            
        case ESP_SPP_START_EVT:
            if (param->start.status == ESP_SPP_SUCCESS) {
                ESP_LOGI(TAG, "SPP server started");
                esp_bt_dev_set_device_name(SPP_SERVER_NAME);
                esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
            }
            break;
            
        case ESP_SPP_SRV_OPEN_EVT:
            ESP_LOGI(TAG, "SPP connection opened");
            instance.bt_data_->spp_connected = true;
            instance.bt_data_->received_data.clear();
            break;
            
        case ESP_SPP_CLOSE_EVT:
            ESP_LOGI(TAG, "SPP connection closed");
            instance.bt_data_->spp_connected = false;
            instance.bt_data_->received_data.clear();
            break;
            
        case ESP_SPP_DATA_IND_EVT:
            if (param->data_ind.len > 0) {
                std::string data((char*)param->data_ind.data, param->data_ind.len);
                instance.bt_data_->received_data += data;
                
                // 检查是否收到完整配置（以换行符或分号分隔）
                size_t pos = instance.bt_data_->received_data.find('\n');
                if (pos != std::string::npos) {
                    std::string config = instance.bt_data_->received_data.substr(0, pos);
                    instance.bt_data_->received_data.clear();
                    
                    // 解析配置：格式为 "SSID:PASSWORD"
                    size_t colon_pos = config.find(':');
                    if (colon_pos != std::string::npos) {
                        std::string ssid = config.substr(0, colon_pos);
                        std::string password = config.substr(colon_pos + 1);
                        
                        ESP_LOGI(TAG, "Received WiFi config - SSID: %s, Password: %s", 
                                ssid.c_str(), password.c_str());
                        
                        if (instance.config_callback_) {
                            instance.config_callback_(ssid, password);
                        }
                        
                        // 发送确认消息
                        const char* response = "WiFi configuration received. Connecting...";
                        esp_spp_write(param->data_ind.handle, strlen(response), (uint8_t*)response);
                    }
                }
            }
            break;
            
        default:
            break;
    }
}

bool BluetoothConfig::Initialize() {
    esp_err_t ret;
    
    // 初始化蓝牙控制器
    ret = esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to release BLE memory: %s", esp_err_to_name(ret));
    }
    
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize controller: %s", esp_err_to_name(ret));
        return false;
    }
    
    ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable controller: %s", esp_err_to_name(ret));
        return false;
    }
    
    ret = esp_bluedroid_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bluedroid: %s", esp_err_to_name(ret));
        return false;
    }
    
    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable bluedroid: %s", esp_err_to_name(ret));
        return false;
    }
    
    ret = esp_spp_register_callback(esp_spp_callback);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register SPP callback: %s", esp_err_to_name(ret));
        return false;
    }
    
    ret = esp_spp_init(ESP_SPP_MODE_CB);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPP: %s", esp_err_to_name(ret));
        return false;
    }
    
    return true;
}

void BluetoothConfig::StartProvisioning() {
    if (!is_provisioning_) {
        if (Initialize()) {
            is_provisioning_ = true;
            ESP_LOGI(TAG, "Bluetooth provisioning started");
        }
    }
}

void BluetoothConfig::StopProvisioning() {
    if (is_provisioning_) {
        esp_spp_deinit();
        esp_bluedroid_disable();
        esp_bluedroid_deinit();
        esp_bt_controller_disable();
        esp_bt_controller_deinit();
        
        is_provisioning_ = false;
        ESP_LOGI(TAG, "Bluetooth provisioning stopped");
    }
}

bool BluetoothConfig::IsProvisioning() const {
    return is_provisioning_;
}

void BluetoothConfig::SetConfigCallback(ConfigCallback callback) {
    config_callback_ = callback;
}