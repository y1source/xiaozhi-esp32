/*
 * 蓝牙WiFi配网集成示例
 * 
 * 本文件展示了如何将蓝牙配网功能集成到xiaozhi-esp32项目中
 * 将此代码集成到application.cc中，可以在设备启动时自动启用蓝牙配网
 */

#include "ble_wifi_integration.h"
#include "ble_wifi_config.h"
#include "ble_ota.h"
#include "esp_log.h"
#include "wifi_configuration_ap.h"
#include "board.h"
#include "esp_timer.h"
#include <ssid_manager.h>
#include "system_info.h"

static const char* TAG = "BLE_WIFI_INTEGRATION";

/*
 * 建议的集成方案：
 * 
 * 1. 在Application类中添加蓝牙配网相关的成员函数和变量
 * 2. 在Application::Start()中启动蓝牙配网功能
 * 3. 当WiFi配置改变时，触发WiFi连接
 */

namespace BleWifiIntegration {

// 静态变量，用于跟踪蓝牙配网状态
static bool ble_wifi_config_active = false;
static esp_timer_handle_t clock_timer_handle_ = nullptr;

// 设备发现相关变量
static std::map<std::string, int> discovered_devices; // MAC地址 -> RSSI
static std::mutex devices_mutex;
static bool scanning_active = false;

// 声明函数
void StopBleWifiConfig();

// 扫描回调函数
static void ble_scan_callback(adv_pk_t *adv) {
    if (adv == NULL) {
        // 设备扫描完成
        ESP_LOGI(TAG, "Device scanning completed");
        scanning_active = false;
        return;
    }
    
    // 解析广播数据，查找设备名称
    uint8_t *data = adv->data;
    uint8_t data_len = adv->adv_len;
    uint8_t pos = 0;
    
    std::string device_name;
    bool has_custom_service = false;
    
    // 解析广播数据
    while (pos < data_len) {
        uint8_t field_len = data[pos];
        if (field_len == 0) break;
        
        uint8_t field_type = data[pos + 1];
        
        if (field_type == 0x09) { // Complete Local Name
            // 设备名称
            uint8_t name_len = field_len - 1;
            if (name_len > 0) {
                char name[32];
                memset(name, 0, sizeof(name));
                memcpy(name, &data[pos + 2], name_len > 31 ? 31 : name_len);
                device_name = name;
            }
        } else if (field_type == 0x03) { // Complete List of 16-bit Service UUIDs
            // 检查是否包含我们的服务UUID (0xFDD0)
            for (uint8_t i = 0; i < (field_len - 1) / 2; i++) {
                uint16_t uuid = data[pos + 2 + i * 2] | (data[pos + 2 + i * 2 + 1] << 8);
                if (uuid == BLE_PROTOCOL_SERVICE_UUID_16) {
                    has_custom_service = true;
                    break;
                }
            }
        } else if (field_type == 0xFF) { // Manufacturer Specific Data
            // 检查是否包含我们的制造商ID (0xFFFF)
            if (field_len >= 3) {
                uint16_t manufacturer_id = data[pos + 2] | (data[pos + 3] << 8);
                if (manufacturer_id == BLE_WIFI_CONFIG_MANUFACTURER_ID) {
                    has_custom_service = true;
                }
            }
        }
        
        pos += field_len + 1;
    }
    
    // 检查是否是我们类型的设备
    bool is_our_device = false;
    
    // 方式1: 检查设备名称是否以指定前缀开头
    if (!device_name.empty()) {
        std::string prefix = BLE_PROTOCOL_ADV_NAME_PREFIX;
        if (device_name.find(prefix) == 0) {
            is_our_device = true;
        }
    }
    
    // 方式2: 检查是否包含我们的服务UUID
    if (has_custom_service) {
        is_our_device = true;
    }
    
    if (is_our_device) {
        // 将MAC地址转换为字符串
        char mac_str[18];
        snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
                 adv->mac[0], adv->mac[1], adv->mac[2],
                 adv->mac[3], adv->mac[4], adv->mac[5]);
        
        {
            std::lock_guard<std::mutex> lock(devices_mutex);
            discovered_devices[mac_str] = adv->rssi;
        }
        
        // 发现相同设备先输出
        ESP_LOGI(TAG, "Duplicate device discovered: MAC=%s, RSSI=%d dBm, Name=%s", mac_str, adv->rssi, device_name.c_str());
    }
}

// 开始扫描相同设备
static void start_scan_for_same_devices() {
    if (scanning_active) {
        // 扫描已在进行中
        ESP_LOGW(TAG, "Scan already in progress");
        return;
    }
    
    // 开始扫描相同设备
    ESP_LOGI(TAG, "Starting duplicate device scan...");
    
    // 注册扫描回调
    int ret = esp_ble_scan_cb_register(ble_scan_callback);
    if (ret != 0) {
        // 注册扫描回调失败
        ESP_LOGE(TAG, "Failed to register scan callback: %d", ret);
        return;
    }
    
    // 启动扫描：扫描间隔100ms，扫描窗口50ms，持续时间10秒，被动扫描
    ret = esp_ble_scan_start(100, 50, 10, false);
    if (ret != 0) {
        // 启动扫描失败
        ESP_LOGE(TAG, "Failed to start scanning: %d", ret);
        esp_ble_scan_cb_unregister(ble_scan_callback);
        return;
    }
    
    scanning_active = true;
    {
        std::lock_guard<std::mutex> lock(devices_mutex);
        discovered_devices.clear();
    }
    
    ESP_LOGI(TAG, "Scan started, will continue for 10 seconds");
}

// WiFi配置改变回调函数（用于0x01命令）
static void OnWifiConfigChanged(const std::string& ssid, const std::string& password) {
    ESP_LOGI(TAG, "BLE WiFi config changed - SSID: %s (Command 0x01)", ssid.c_str());
    
    // 尝试连接到新的WiFi网络
    auto& wifi_ap = WifiConfigurationAp::GetInstance();
    wifi_ap.ConnectToWifi(ssid, password);
    
    // 0x01命令不等待结果，直接返回成功
}

// WiFi配置改变回调函数（用于0x06命令，包含连接ID）
static void OnWifiConfigChangedWithConnId(uint16_t conn_id, const std::string& ssid, const std::string& password) {
    ESP_LOGI(TAG, "BLE WiFi config changed - conn_id=%d, SSID: %s (Command 0x06)", conn_id, ssid.c_str());
    
    // 尝试连接到新的WiFi网络
    auto& wifi_ap = WifiConfigurationAp::GetInstance();
    bool connected = wifi_ap.ConnectToWifi(ssid, password);

    // 发送连接结果给APP
    auto& ble_wifi_config = BleWifiConfig::GetInstance();
    ble_wifi_config.SendWifiConnectResult(conn_id, ssid, connected);
    
    if (connected) {
        ESP_LOGI(TAG, "Successfully connected to WiFi: %s", ssid.c_str());
        
        // ========== 只在连接成功后才保存到SSID管理器 ==========
        auto& ssid_manager = SsidManager::GetInstance();
        ssid_manager.AddSsid(ssid, password);
        ESP_LOGI(TAG, "Saved WiFi credentials to SSID manager after successful connection");

        // 连接成功后，可以选择停止蓝牙配网以节省资源
        StopBleWifiConfig();
        
        ESP_LOGI(TAG, "Restarting in 1 second");
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
    } else {
        ESP_LOGW(TAG, "Failed to connect to WiFi: %s", ssid.c_str());
        
        // ========== 新增代码：连接失败后保持蓝牙广播 ==========
        // 连接失败后，我们不停止蓝牙广播，保持可被发现状态
        // 这样用户可以在手机APP上重新输入密码
        
        // 注意：SendWifiConnectResult 函数中已经处理了重新广播的逻辑
        // 这里只需要记录日志，不需要额外操作
        ESP_LOGI(TAG, "WiFi connection failed, Bluetooth advertising has been kept enabled, ready for reconfiguration");
    }
}


static void update_adv(void){
    auto& ble_wifi_config = BleWifiConfig::GetInstance();
    if(!ble_wifi_config_active || ble_wifi_config.IsConnected()){
        return;
    }
    static int last_battery_level = -1;
    static bool last_charging = false;

    int battery_level = 0;
    bool charging = false, discharging = false;
    auto& board = Board::GetInstance();
    board.GetBatteryLevel(battery_level, charging, discharging);
    
    if(battery_level == last_battery_level && charging == last_charging){
        return;
    }

    last_battery_level = battery_level;
    last_charging = charging;

    // auto& wifi_ap = WifiConfigurationAp::GetInstance();
    // std::string ap_ssid = wifi_ap.GetSsid();

    // 将设备ID设置为蓝牙名称后缀，并去掉“:”
    std::string device_id;
    for (char c : SystemInfo::GetMacAddress()) {
        if (c == ':') device_id += "";
        else device_id += c;
    }
    
    ble_wifi_config.StopAdvertising();
    vTaskDelay(pdMS_TO_TICKS(100));
    ble_wifi_config.StartAdvertising(device_id, battery_level, charging);

    ESP_LOGI(TAG, "Advertising name: lr_wificfg-%s", device_id.c_str());
}

// 启动蓝牙配网功能
bool StartBleWifiConfig() {
    ESP_LOGI(TAG, "Starting Bluetooth network configuration function");

    if (ble_wifi_config_active) {
        ESP_LOGW(TAG, "BLE WiFi config already active");
        return true;
    }
    
    ESP_LOGI(TAG, "Starting BLE WiFi configuration service");
    
    // 获取BLE WiFi配置实例
    ESP_LOGI(TAG, "Getting BLE WiFi configuration instance");
    auto& ble_wifi_config = BleWifiConfig::GetInstance();
    
    // 初始化蓝牙配网功能
    ESP_LOGI(TAG, "Initializing Bluetooth network configuration function");
    if (!ble_wifi_config.Initialize()) {
        ESP_LOGE(TAG, "Failed to initialize BLE WiFi config");
        return false;
    }
    
    // 设置WiFi配置改变回调（0x01命令使用）
    ble_wifi_config.SetOnWifiConfigChanged(OnWifiConfigChanged);
    
    // 设置WiFi配置改变回调（0x06命令使用，包含连接ID）
    ble_wifi_config.SetOnWifiConfigChangedWithConnId(OnWifiConfigChangedWithConnId);

    esp_timer_create_args_t clock_timer_args = {
        .callback = [](void* arg) {
            update_adv();
        },
        .name = "update_adv",
        .skip_unhandled_events = true
    };
    esp_timer_create(&clock_timer_args, &clock_timer_handle_);
    
    ble_wifi_config_active = true;

    update_adv();

    esp_timer_start_periodic(clock_timer_handle_, 5000000); // 每5秒更新广播
    
    // 启动扫描相同设备
    start_scan_for_same_devices();

    ESP_LOGI(TAG, "BLE WiFi configuration started successfully");
    
    return true;
}

// 停止蓝牙配网功能
void StopBleWifiConfig() {
    if (!ble_wifi_config_active) {
        return;
    }
    
    ESP_LOGI(TAG, "Stopping BLE WiFi configuration service");
    
    auto& ble_wifi_config = BleWifiConfig::GetInstance();
    ble_wifi_config.Disconnect();
    ble_wifi_config.StopAdvertising();
    ble_wifi_config.Deinitialize();
    
    // 停止扫描
    if (scanning_active) {
        esp_ble_scan_stop();
        esp_ble_scan_cb_unregister(ble_scan_callback);
        scanning_active = false;
        ESP_LOGI(TAG, "Device scanning has stopped");
    }
    
    // 同时清理BLE OTA服务
    auto& ble_ota = BleOta::GetInstance();
    if (ble_ota.IsInitialized()) {
        ble_ota.Deinitialize();
        ESP_LOGI(TAG, "BLE OTA service deinitialized");
    }
    
    ble_wifi_config_active = false;
    ESP_LOGI(TAG, "BLE WiFi configuration stopped");
}

// 检查蓝牙配网是否活跃
bool IsBleWifiConfigActive() {
    return ble_wifi_config_active;
}

// 获取发现的相同设备列表
std::vector<std::pair<std::string, int>> GetDiscoveredDevices() {
    std::lock_guard<std::mutex> lock(devices_mutex);
    std::vector<std::pair<std::string, int>> result;
    
    for (const auto& device : discovered_devices) {
        result.push_back(std::make_pair(device.first, device.second));
    }
    
    return result;
}

// 手动触发设备扫描
void TriggerDeviceScan() {
    start_scan_for_same_devices();
}


} // namespace BleWifiIntegration