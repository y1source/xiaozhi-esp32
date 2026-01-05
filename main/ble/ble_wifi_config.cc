#include "ble_wifi_config.h"
#include "ble_protocol.h"
#include "esp_ble.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include <esp_ota_ops.h>
#include "nimble/ble.h"
#include "host/ble_hs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <string.h>
#include <cJSON.h>
#include <functional>
#include <vector>
#include <algorithm>
#include <map>
#include <mutex>
#include "wifi_configuration_ap.h"
#include "ble_protocol.h"
#include "ssid_manager.h"
#include "board.h"

#define TAG "BleWifiConfig"

// 全局变量
static bool g_ble_initialized = false;
static bool g_ble_advertising = false;
static uint16_t g_conn_handle = 0xFFFF;
static std::function<void(const std::string&, const std::string&)> g_wifi_config_callback;
static std::function<void(uint16_t, const std::string&, const std::string&)> g_wifi_config_callback_with_conn_id;

// 添加连接状态跟踪
typedef struct {
    uint16_t conn_id;
    uint8_t retry_count;
    uint32_t last_attempt_time;
    std::string ssid;
} wifi_config_attempt_t;

static std::map<uint16_t, wifi_config_attempt_t> g_wifi_config_attempts;
static std::mutex g_attempts_mutex;

#define MAX_WIFI_CONFIG_RETRIES 0        // 最大重试次数
#define WIFI_CONFIG_RETRY_INTERVAL_MS 1000  // 重试间隔1秒

// C接口实现
extern "C" {

// 清理连接尝试记录
static void ble_wifi_config_cleanup_attempt(uint16_t conn_id) {
    std::lock_guard<std::mutex> lock(g_attempts_mutex);
    auto it = g_wifi_config_attempts.find(conn_id);
    if (it != g_wifi_config_attempts.end()) {
        g_wifi_config_attempts.erase(it);
        ESP_LOGI(TAG, "WiFi configuration attempt record for connection %d has been cleaned up", conn_id);
    }
}

// 在断开连接时清理记录
static void ble_evt_handler(ble_evt_t* evt) {
    if(evt == NULL) {
        return;
    }

    if(evt->evt_id == BLE_EVT_CONNECTED) {
        // BLE连接建立
        ESP_LOGI(TAG, "BLE connection established, conn_id=%d", evt->params.connected.conn_id);
        g_conn_handle = evt->params.connected.conn_id;
        
        // 清理之前的尝试记录
        ble_wifi_config_cleanup_attempt(g_conn_handle);
        
    } else if(evt->evt_id == BLE_EVT_DISCONNECTED) {
        // BLE连接断开
        ESP_LOGI(TAG, "BLE connection disconnected, conn_id=%d", evt->params.disconnected.conn_id);
        
        // 清理连接尝试记录
        ble_wifi_config_cleanup_attempt(evt->params.disconnected.conn_id);
        
        if (g_conn_handle == evt->params.disconnected.conn_id) {
            g_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        }
        
        // 检查是否应该重新广播（如果没有主动停止）
        if (g_ble_initialized && !g_ble_advertising) {
            // BLE连接断开，重新启动广播
            ESP_LOGI(TAG, "BLE connection disconnected, restarting broadcast");
            
            // 获取当前AP SSID和电池信息
            auto& wifi_ap = WifiConfigurationAp::GetInstance();
            std::string ap_ssid = wifi_ap.GetSsid();
            int battery_level = 0;
            bool charging = false, discharging = false;
            Board::GetInstance().GetBatteryLevel(battery_level, charging, discharging);
            
            // 重新启动广播
            int ret = ble_wifi_config_start_advertising(ap_ssid.c_str(), battery_level, charging);
            if (ret == 0) {
                // 启动成功
                ESP_LOGI(TAG, "BLE broadcast restarted successfully");
            } else {
                // 启动失败
                ESP_LOGE(TAG, "BLE broadcast restart failed: %d", ret);
            }
        }
    }
}


// 获取当前WiFi配置
static int handle_get_wifi_config_cmd(uint16_t conn_id) {
    ESP_LOGI(TAG, "Handling get WiFi config command");
    
    // 从SSID管理器获取当前WiFi配置
    auto& ssid_manager = SsidManager::GetInstance();
    const auto& ssid_list = ssid_manager.GetSsidList();
    
    if (ssid_list.empty()) {
        ESP_LOGW(TAG, "No saved WiFi configurations");
        // 返回空配置
        uint8_t empty_payload[] = {0, 0}; // ssid_len=0, password_len=0
        return ble_protocol_send_response(conn_id, BLE_PROTOCOL_CMD_GET_WIFI_CONFIG, empty_payload, sizeof(empty_payload));
    }
    
    // 取第一个（默认）配置
    const auto& default_ssid = ssid_list[0];
    const std::string& ssid = default_ssid.ssid;
    const std::string& password = default_ssid.password;
    
    // 构建响应载荷：ssid_len + ssid + password_len + password
    size_t payload_size = 1 + ssid.length() + 1 + password.length();
    uint8_t *payload = new uint8_t[payload_size];
    
    size_t offset = 0;
    payload[offset++] = ssid.length();
    memcpy(&payload[offset], ssid.c_str(), ssid.length());
    offset += ssid.length();
    payload[offset++] = password.length();
    memcpy(&payload[offset], password.c_str(), password.length());

    int result = ble_protocol_send_response(conn_id, BLE_PROTOCOL_CMD_GET_WIFI_CONFIG, payload, payload_size);
    delete[] payload;
    
    ESP_LOGI(TAG, "WiFi config response: ssid=%s, password_len=%d", ssid.c_str(), password.length());
    return result;
}

// 设置WiFi配置（同步版本，用于0x01命令）
static int handle_set_wifi_config_cmd(uint16_t conn_id, const uint8_t *payload, size_t payload_len) {
    ESP_LOGI(TAG, "Handling set WiFi config command, payload_len=%d", payload_len);
    
    if (payload_len < 2) {
        ESP_LOGE(TAG, "Invalid payload length for set WiFi config");
        uint8_t error_resp = BLE_PROTOCOL_ACK_ERROR;
        return ble_protocol_send_response(conn_id, BLE_PROTOCOL_CMD_SET_WIFI_CONFIG, &error_resp, 1);
    }
    
    // 解析载荷：ssid_len + ssid + password_len + password
    size_t offset = 0;
    uint8_t ssid_len = payload[offset++];
    
    if (offset + ssid_len >= payload_len) {
        ESP_LOGE(TAG, "Invalid SSID length");
        uint8_t error_resp = BLE_PROTOCOL_ACK_ERROR;
        return ble_protocol_send_response(conn_id, BLE_PROTOCOL_CMD_SET_WIFI_CONFIG, &error_resp, 1);
    }
    
    std::string ssid((char*)&payload[offset], ssid_len);
    offset += ssid_len;
    
    if (offset >= payload_len) {
        ESP_LOGE(TAG, "Missing password length");
        uint8_t error_resp = BLE_PROTOCOL_ACK_ERROR;
        return ble_protocol_send_response(conn_id, BLE_PROTOCOL_CMD_SET_WIFI_CONFIG, &error_resp, 1);
    }
    
    uint8_t password_len = payload[offset++];
    
    if (offset + password_len > payload_len) {
        ESP_LOGE(TAG, "Invalid password length");
        uint8_t error_resp = BLE_PROTOCOL_ACK_ERROR;
        return ble_protocol_send_response(conn_id, BLE_PROTOCOL_CMD_SET_WIFI_CONFIG, &error_resp, 1);
    }
    
    std::string password((char*)&payload[offset], password_len);
    
    ESP_LOGI(TAG, "Setting WiFi config: ssid=%s, password_len=%d", ssid.c_str(), password.length());
    
    // 保存到SSID管理器
    auto& ssid_manager = SsidManager::GetInstance();
    ssid_manager.AddSsid(ssid, password);
    
    // 如果有回调函数，通知WiFi配置改变（使用旧的调用方式，不需要conn_id）
    if (g_wifi_config_callback) {
        g_wifi_config_callback(ssid, password);
    }
    
    // 返回成功响应
    uint8_t success_resp = BLE_PROTOCOL_ACK_SUCCESS;
    return ble_protocol_send_response(conn_id, BLE_PROTOCOL_CMD_SET_WIFI_CONFIG, &success_resp, 1);
}

// 获取WiFi扫描列表
static int handle_get_scan_list_cmd(uint16_t conn_id) {
    ESP_LOGI(TAG, "Handling get scan list command");

    // 获取当前扫描结果
    std::vector<wifi_ap_record_t> local_scan_results = WifiConfigurationAp::GetInstance().GetAccessPoints();
    int ret;
    // 构建响应载荷
    uint16_t len_limit = BLE_PROTOCOL_MAX_PAYLOAD_LEN; // 设置一个安全的MTU限制，避免超过BLE MTU
    uint8_t arr[len_limit];
    uint16_t offset = 0;
    
    int i = 0;
    do {
        memset(arr, 0, sizeof(arr));
        arr[0] = 0;
        offset = 1;
        
        while (i < local_scan_results.size()) {
            const char* ssid_str = (const char*)local_scan_results[i].ssid;
            uint8_t ssid_len = strlen(ssid_str);
            
            // 检查是否会超出缓冲区
            if (offset + ssid_len + 1 > len_limit) {
                break;
            }
            
            arr[0]++;
            arr[offset++] = ssid_len;
            memcpy(&arr[offset], ssid_str, ssid_len);
            offset += ssid_len;
            i++;
        }

        if (arr[0] > 0) {
            ret = ble_protocol_send_response(conn_id, BLE_PROTOCOL_CMD_GET_WIFI_SCAN,
                                               arr, offset);
            
            vTaskDelay(pdMS_TO_TICKS(10)); // 小延迟避免发送过快
        } else {
            break;
        }
        
    } while (i < local_scan_results.size());

    // 发送结束标记
    uint8_t end_marker[] = {0x00};
    ret = ble_protocol_send_response(conn_id, BLE_PROTOCOL_CMD_GET_WIFI_SCAN, end_marker, sizeof(end_marker));

    ESP_LOGI(TAG, "Scan list response sent, found %d APs", (int)local_scan_results.size());
    return ret;
}

// 检查WiFi连接状态
static bool is_wifi_connecting_or_connected() {
    wifi_mode_t mode;
    if (esp_wifi_get_mode(&mode) != ESP_OK) {
        return false;
    }
    
    if (mode & WIFI_MODE_STA) {
        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            return true; // 已连接
        }
        
        // 检查是否正在连接
        wifi_config_t wifi_config;
        if (esp_wifi_get_config(WIFI_IF_STA, &wifi_config) == ESP_OK) {
            // 如果有配置，可能正在连接
            return strlen((char*)wifi_config.sta.ssid) > 0;
        }
    }
    
    return false;
}

// WiFi操作命令处理
static int handle_wifi_operation_cmd(uint16_t conn_id, const uint8_t *payload, size_t payload_len) {
    ESP_LOGI(TAG, "Handling WiFi operation command, payload_len=%d", payload_len);
    
    if (payload_len < 1) {
        ESP_LOGE(TAG, "Invalid payload length for WiFi operation");
        uint8_t error_resp = BLE_PROTOCOL_ACK_ERROR;
        return ble_protocol_send_response(conn_id, BLE_PROTOCOL_CMD_WIFI_OPT, &error_resp, 1);
    }
    
    uint8_t opt = payload[0];
    ESP_LOGI(TAG, "WiFi operation opt: 0x%02x", opt);
    
    switch (opt) {
        case WIFI_OPT_GET_SSID_LIST: {
            ESP_LOGI(TAG, "Getting SSID list");
            
            // 获取SSID列表
            auto& ssid_manager = SsidManager::GetInstance();
            const auto& ssid_list = ssid_manager.GetSsidList();
            
            int ret;
            // 构建响应载荷
            uint16_t len_limit = BLE_PROTOCOL_MAX_PAYLOAD_LEN ; // 设置一个安全的MTU限制，避免超过BLE MTU
            uint8_t arr[len_limit];
            uint16_t offset = 0;
            
            int i = 0;
            do {
                memset(arr, 0, sizeof(arr));
                arr[0] = WIFI_OPT_GET_SSID_LIST;
                arr[1] = 0;
                offset = 2;
                
                while (i < ssid_list.size()) {
                    const char* ssid_str = ssid_list[i].ssid.c_str();
                    uint8_t ssid_len = ssid_list[i].ssid.length();
                    
                    // 检查是否会超出缓冲区
                    if (offset + ssid_len + 1 > len_limit) {
                        break;
                    }
                    
                    arr[1]++;
                    arr[offset++] = ssid_len;
                    memcpy(&arr[offset], ssid_str, ssid_len);
                    offset += ssid_len;
                    i++;
                }

                if (arr[1] > 0) {
                    ret = ble_protocol_send_response(conn_id, BLE_PROTOCOL_CMD_WIFI_OPT,
                                                    arr, offset);
                    
                    vTaskDelay(pdMS_TO_TICKS(10)); // 小延迟避免发送过快
                } else {
                    break;
                }
                
            } while (i < ssid_list.size());

            // 发送结束标记
            uint8_t end_marker[] = {WIFI_OPT_GET_SSID_LIST,0x00};
            ret = ble_protocol_send_response(conn_id, BLE_PROTOCOL_CMD_WIFI_OPT, end_marker, sizeof(end_marker));

            ESP_LOGI(TAG, "ssid list response sent  %d APs", (int)ssid_list.size());
            return ret;
        }
        
        case WIFI_OPT_SET_SSID: {
            ESP_LOGI(TAG, "Setting SSID (0x06 opt=1)");
            
            // 解析载荷：ssid_len + ssid + password_len + password
            const uint8_t* data = payload + 1;
            size_t data_len = payload_len > 0 ? payload_len - 1 : 0;
            
            ESP_LOGI(TAG, "Handling set WiFi config command, data_len=%d", data_len);
            
            if (data_len < 2) {
                ESP_LOGE(TAG, "Invalid payload length for set WiFi config");
                uint8_t end_marker[] = {WIFI_OPT_SET_SSID, BLE_PROTOCOL_ACK_ERROR};
                return ble_protocol_send_response(conn_id, BLE_PROTOCOL_CMD_WIFI_OPT, end_marker, sizeof(end_marker));
            }
            
            size_t offset = 0;
            uint8_t ssid_len = data[offset++];
            
            if (offset + ssid_len >= data_len) {
                ESP_LOGE(TAG, "Invalid SSID length");
                uint8_t end_marker[] = {WIFI_OPT_SET_SSID, BLE_PROTOCOL_ACK_ERROR};
                return ble_protocol_send_response(conn_id, BLE_PROTOCOL_CMD_WIFI_OPT, end_marker, sizeof(end_marker));
            }
            
            std::string ssid((char*)&data[offset], ssid_len);
            offset += ssid_len;
            
            if (offset >= data_len) {
                ESP_LOGE(TAG, "Missing password length");
                uint8_t end_marker[] = {WIFI_OPT_SET_SSID, BLE_PROTOCOL_ACK_ERROR};
                return ble_protocol_send_response(conn_id, BLE_PROTOCOL_CMD_WIFI_OPT, end_marker, sizeof(end_marker));
            }
            
            uint8_t password_len = data[offset++];
            
            if (offset + password_len > data_len) {
                ESP_LOGE(TAG, "Invalid password length");
                uint8_t end_marker[] = {WIFI_OPT_SET_SSID, BLE_PROTOCOL_ACK_ERROR};
                return ble_protocol_send_response(conn_id, BLE_PROTOCOL_CMD_WIFI_OPT, end_marker, sizeof(end_marker));
            }
            
            std::string password((char*)&data[offset], password_len);
            
            ESP_LOGI(TAG, "Setting WiFi config: SSID=%s, password_len=%d", ssid.c_str(), password.length());
            
            // // ========== 检查重试次数 ==========
            // {
            //     std::lock_guard<std::mutex> lock(g_attempts_mutex);
            //     auto it = g_wifi_config_attempts.find(conn_id);
            //     if (it != g_wifi_config_attempts.end()) {
            //         // 检查是否超过重试间隔
            //         uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
            //         if (now - it->second.last_attempt_time < WIFI_CONFIG_RETRY_INTERVAL_MS) {
            //             ESP_LOGW(TAG, "重试间隔太短，请稍后再试");
            //             uint8_t retry_too_fast[] = {WIFI_OPT_SET_SSID, 0x04}; // 自定义错误码：重试过快
            //             return ble_protocol_send_response(conn_id, BLE_PROTOCOL_CMD_WIFI_OPT, retry_too_fast, sizeof(retry_too_fast));
            //         }
                    
            //         // 检查是否超过最大重试次数
            //         if (it->second.retry_count >= MAX_WIFI_CONFIG_RETRIES) {
            //             ESP_LOGE(TAG, "超过最大重试次数(%d)，请检查网络配置", MAX_WIFI_CONFIG_RETRIES);
            //             uint8_t max_retries_exceeded[] = {WIFI_OPT_SET_SSID, 0x05}; // 自定义错误码：超过最大重试次数
            //             return ble_protocol_send_response(conn_id, BLE_PROTOCOL_CMD_WIFI_OPT, max_retries_exceeded, sizeof(max_retries_exceeded));
            //         }
                    
            //         // 更新重试信息
            //         it->second.retry_count++;
            //         it->second.last_attempt_time = now;
            //         it->second.ssid = ssid;
            //     } else {
            //         // 第一次尝试
            //         wifi_config_attempt_t attempt;
            //         attempt.conn_id = conn_id;
            //         attempt.retry_count = 1;
            //         attempt.last_attempt_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
            //         attempt.ssid = ssid;
            //         g_wifi_config_attempts[conn_id] = attempt;
            //     }
            // }

            // 检查WiFi状态，如果正在连接，先断开
            if (is_wifi_connecting_or_connected()) {
                ESP_LOGW(TAG, "WiFi is connecting or already connected, disconnecting current connection first");
                esp_wifi_disconnect();
                vTaskDelay(pdMS_TO_TICKS(200));
            }
            
            // 立即回复"正在连接"
            uint8_t connecting_resp[] = {WIFI_OPT_SET_SSID, BLE_PROTOCOL_ACK_CONNECTING};
            int ret = ble_protocol_send_response(conn_id, BLE_PROTOCOL_CMD_WIFI_OPT, connecting_resp, sizeof(connecting_resp));
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to send connecting response: %d", ret);
                return ret;
            }
            
            // 保存到SSID管理器
            auto& ssid_manager = SsidManager::GetInstance();
            ssid_manager.AddSsid(ssid, password);
            
            // 如果有回调函数，通知WiFi配置改变
            if (g_wifi_config_callback_with_conn_id) {
                ESP_LOGI(TAG, "Calling WiFi configuration change callback, conn_id=%d", conn_id);
                g_wifi_config_callback_with_conn_id(conn_id, ssid, password);
                ESP_LOGI(TAG, "WiFi configuration change callback returned");
            } else {
                ESP_LOGW(TAG, "WiFi configuration change callback not set");
                uint8_t error_resp[] = {WIFI_OPT_SET_SSID, BLE_PROTOCOL_ACK_ERROR};
                return ble_protocol_send_response(conn_id, BLE_PROTOCOL_CMD_WIFI_OPT, error_resp, sizeof(error_resp));
            }
            
            return ESP_OK;
        }
        
        case WIFI_OPT_SCAN: {
            ESP_LOGI(TAG, "WiFi scan (reuse scan list)");

            // 获取当前扫描结果
            std::vector<wifi_ap_record_t> local_scan_results = WifiConfigurationAp::GetInstance().GetAccessPoints();
            int ret;
            // 构建响应载荷
            uint16_t len_limit = BLE_PROTOCOL_MAX_PAYLOAD_LEN; // 设置一个安全的MTU限制，避免超过BLE MTU
            uint8_t arr[len_limit];
            uint16_t offset = 0;

            int i = 0;
            do {
                memset(arr, 0, sizeof(arr));
                arr[0] = WIFI_OPT_SCAN;
                arr[1] = 0;
                offset = 2;
                
                while (i < local_scan_results.size()) {
                    const char* ssid_str = (const char*)local_scan_results[i].ssid;
                    uint8_t ssid_len = strlen(ssid_str);
                    
                    // 检查是否会超出缓冲区
                    if (offset + ssid_len + 1 > len_limit) {
                        break;
                    }
                    
                    arr[1]++;
                    arr[offset++] = ssid_len;
                    memcpy(&arr[offset], ssid_str, ssid_len);
                    offset += ssid_len;
                    i++;
                }

                if (arr[1] > 0) {
                    ret = ble_protocol_send_response(conn_id, BLE_PROTOCOL_CMD_WIFI_OPT,
                                                    arr, offset);
                    
                    vTaskDelay(pdMS_TO_TICKS(10)); // 小延迟避免发送过快
                } else {
                    break;
                }
                
            } while (i < local_scan_results.size());

            // 发送结束标记
            uint8_t end_marker[] = {WIFI_OPT_SCAN,0x00};
            ret = ble_protocol_send_response(conn_id, BLE_PROTOCOL_CMD_WIFI_OPT, end_marker, sizeof(end_marker));

            ESP_LOGI(TAG, "Scan list response sent, found %d APs", (int)local_scan_results.size());
            return ret;
        }
        
        case WIFI_OPT_DELETE_SSID: {
            ESP_LOGI(TAG, "Deleting specific SSID");
            ESP_LOG_BUFFER_HEX(TAG, payload, payload_len);
            
            // 跳过OPT字节
            const uint8_t* data = payload + 1;
            size_t data_len = payload_len > 0 ? payload_len - 1 : 0;
            
            if (data_len == 0 || data_len > 32) {
                ESP_LOGE(TAG, "Invalid payload for delete SSID operation");
                uint8_t end_marker[] = {WIFI_OPT_DELETE_SSID, BLE_PROTOCOL_ACK_ERROR};
                return ble_protocol_send_response(conn_id, BLE_PROTOCOL_CMD_WIFI_OPT, end_marker, sizeof(end_marker));
            }
            
            // 解析SSID：ssid_len + ssid
            std::string target_ssid((char*)data, data_len);
            ESP_LOGI(TAG, "Deleting SSID: %s", target_ssid.c_str());
            
            // 查找并删除指定SSID
            auto& ssid_manager = SsidManager::GetInstance();
            const auto& ssid_list = ssid_manager.GetSsidList();
            
            int found_index = -1;
            for (size_t i = 0; i < ssid_list.size(); i++) {
                if (ssid_list[i].ssid == target_ssid) {
                    found_index = i;
                    break;
                }
            }
            
            if (found_index >= 0) {
                ssid_manager.RemoveSsid(found_index);
                ESP_LOGI(TAG, "Successfully deleted SSID: %s", target_ssid.c_str());
 
                uint8_t end_marker[] = {WIFI_OPT_DELETE_SSID, BLE_PROTOCOL_ACK_SUCCESS};
                return ble_protocol_send_response(conn_id, BLE_PROTOCOL_CMD_WIFI_OPT, end_marker, sizeof(end_marker));
            } else {
                ESP_LOGW(TAG, "SSID not found: %s", target_ssid.c_str());
                uint8_t end_marker[] = {WIFI_OPT_DELETE_SSID, BLE_PROTOCOL_ACK_ERROR};
                return ble_protocol_send_response(conn_id, BLE_PROTOCOL_CMD_WIFI_OPT, end_marker, sizeof(end_marker));
            }
        }
        
        default:
            ESP_LOGE(TAG, "Unknown WiFi operation opt: 0x%02x", opt);
            uint8_t end_marker[] = {opt, BLE_PROTOCOL_ACK_ERROR};
            return ble_protocol_send_response(conn_id, BLE_PROTOCOL_CMD_WIFI_OPT, end_marker, sizeof(end_marker));
    }
}

// 协议处理器包装函数
static esp_err_t ble_wifi_get_config_handler(uint16_t conn_id, const uint8_t *payload, uint16_t payload_len)
{
    ESP_LOGI(TAG, "Handling get WiFi config command");

    int ret = handle_get_wifi_config_cmd(conn_id);

    if(ret){
        ESP_LOGE(TAG, "Failed to get WiFi config: %d", ret);
    }
    return ret;
}

static esp_err_t ble_wifi_set_config_handler(uint16_t conn_id, const uint8_t *payload, uint16_t payload_len)
{
    ESP_LOGI(TAG, "Handling set WiFi config command");

    int ret = handle_set_wifi_config_cmd(conn_id, payload, payload_len);

    if (ret) {
        ESP_LOGE(TAG, "Failed to set WiFi config: %d", ret);
    }
   return ret;
}

static esp_err_t ble_wifi_get_scan_handler(uint16_t conn_id, const uint8_t *payload, uint16_t payload_len)
{
    ESP_LOGI(TAG, "Handling get WiFi scan command");

    int ret = handle_get_scan_list_cmd(conn_id);
    
    if (ret) {
        ESP_LOGE(TAG, "Failed to get WiFi scan list: %d", ret);
    }
   return ret;
}

static esp_err_t ble_wifi_operation_handler(uint16_t conn_id, const uint8_t *payload, uint16_t payload_len)
{
    ESP_LOGI(TAG, "Handling WiFi operation command");

    int ret = handle_wifi_operation_cmd(conn_id, payload, payload_len);

    if (ret) {
        ESP_LOGE(TAG, "Failed to handle WiFi operation: %d", ret);
    }
    return ret;
}

static esp_err_t ble_rst_handler(uint16_t conn_id, const uint8_t *payload, uint16_t payload_len)
{
    ESP_LOGI(TAG, "Handling reset command");

    // 执行重置操作
    int ret;
    uint8_t end_marker[] = {0x00};
    ret = ble_protocol_send_response(conn_id, BLE_PROTOCOL_CMD_GET_WIFI_SCAN, end_marker, sizeof(end_marker));


    if (ret) {
        ESP_LOGE(TAG, "Failed to reset: %d", ret);
    }
    ESP_LOGI(TAG, "Device will restart in 2 seconds...");

    vTaskDelay(pdMS_TO_TICKS(2000));
    esp_restart();
    return ret;
}

static esp_err_t ble_wifi_config_register_handlers(void)
{
    ESP_LOGI(TAG, "Registering BLE WiFi config protocol handlers");
    
    esp_err_t ret;
    
    // 注册获取WiFi配置处理器
    ret = ble_protocol_register_handler(BLE_PROTOCOL_CMD_GET_WIFI_CONFIG, 
                                        ble_wifi_get_config_handler, 
                                        "wifi_get_config");
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register get WiFi config handler: %s", esp_err_to_name(ret));
        goto ERROR;
    }
    
    // 注册设置WiFi配置处理器
    ret = ble_protocol_register_handler(BLE_PROTOCOL_CMD_SET_WIFI_CONFIG, 
                                        ble_wifi_set_config_handler, 
                                        "wifi_set_config");
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register set WiFi config handler: %s", esp_err_to_name(ret));
        goto ERROR;
    }
    
    // 注册WiFi扫描处理器
    ret = ble_protocol_register_handler(BLE_PROTOCOL_CMD_GET_WIFI_SCAN, 
                                        ble_wifi_get_scan_handler, 
                                        "wifi_get_scan");
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register WiFi scan handler: %s", esp_err_to_name(ret));
        goto ERROR;
    }

    // 注册WiFi操作处理器 (0x06)
    ret = ble_protocol_register_handler(BLE_PROTOCOL_CMD_WIFI_OPT, 
                                        ble_wifi_operation_handler, 
                                        "wifi_operation");
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register WiFi operation handler: %s", esp_err_to_name(ret));
        goto ERROR;
    }

    // 注册重置处理器
    ret = ble_protocol_register_handler(BLE_PROTOCOL_CMD_RST,
                                        ble_rst_handler,
                                        "device_reset");
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register reset handler: %s", esp_err_to_name(ret));
        goto ERROR;
    }
    ESP_LOGI(TAG, "BLE WiFi config protocol handlers registered successfully");
    return ESP_OK;
ERROR:
    ble_protocol_unregister_handler(BLE_PROTOCOL_CMD_GET_WIFI_CONFIG);
    ble_protocol_unregister_handler(BLE_PROTOCOL_CMD_SET_WIFI_CONFIG);
    ble_protocol_unregister_handler(BLE_PROTOCOL_CMD_GET_WIFI_SCAN);
    ble_protocol_unregister_handler(BLE_PROTOCOL_CMD_WIFI_OPT);
    ble_protocol_unregister_handler(BLE_PROTOCOL_CMD_RST);
    return ret;
}

static esp_err_t ble_wifi_config_unregister_handlers(void)
{
    ESP_LOGI(TAG, "Unregistering BLE WiFi config protocol handlers");
    
    ble_protocol_unregister_handler(BLE_PROTOCOL_CMD_GET_WIFI_CONFIG);
    ble_protocol_unregister_handler(BLE_PROTOCOL_CMD_SET_WIFI_CONFIG);
    ble_protocol_unregister_handler(BLE_PROTOCOL_CMD_GET_WIFI_SCAN);
    ble_protocol_unregister_handler(BLE_PROTOCOL_CMD_WIFI_OPT);
    ble_protocol_unregister_handler(BLE_PROTOCOL_CMD_RST);
    
    ESP_LOGI(TAG, "BLE WiFi config protocol handlers unregistered");
    return ESP_OK;
}

int ble_wifi_config_init(void) {
    if (g_ble_initialized) {
        ESP_LOGW(TAG, "BLE WiFi config already initialized");
        return 0;
    }

    int ret = esp_ble_init();
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to initialize BLE: %d", ret);
        return ret;
    }

    ble_protocol_init();

    ble_wifi_config_register_handlers();

    esp_ble_register_evt_callback(ble_evt_handler);

    g_ble_initialized = true;
    ESP_LOGI(TAG, "BLE WiFi config initialized");
    return 0;
}

int ble_wifi_config_start_advertising(const char* ap_ssid, int battery_level, bool charging) {
    if (!g_ble_initialized) {
        ESP_LOGE(TAG, "BLE not initialized");
        return -1;
    }
    
    if(g_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        ESP_LOGW(TAG, "Device already connected, cannot start advertising");
        return -1;
    }

    if (g_ble_advertising) {
        ESP_LOGW(TAG, "Already advertising");
        return 0;
    }
    
    // 构建广播名称
    std::string adv_name = BLE_PROTOCOL_ADV_NAME_PREFIX;
    if (ap_ssid) {
        adv_name += ap_ssid;
    } else {
        adv_name += "device";
    }
    
    // 设置广播名称
    int ret = esp_ble_gap_set_advname(const_cast<char*>(adv_name.c_str()));
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to set advertising name: %d", ret);
        return ret;
    }
    
    // 构建广播数据
    static uint8_t adv_data[31];
    size_t adv_len = 0;
    
    // Flags
    adv_data[adv_len++] = 2;  // Length
    adv_data[adv_len++] = 0x01;  // Flags
    adv_data[adv_len++] = 0x06;  // LE General Discoverable + BR/EDR Not Supported
    
    // Complete Local Name
    size_t name_len = adv_name.length();
    if (adv_len + 2 + name_len <= 31) {
        adv_data[adv_len++] = 1 + name_len;  // Length
        adv_data[adv_len++] = 0x09;  // Complete Local Name
        memcpy(&adv_data[adv_len], adv_name.c_str(), name_len);
        adv_len += name_len;
    }
    
    // 16-bit Service UUID
    if (adv_len + 4 <= 31) {
        adv_data[adv_len++] = 3;  // Length
        adv_data[adv_len++] = 0x03;  // Complete List of 16-bit Service UUIDs
        adv_data[adv_len++] = (BLE_PROTOCOL_SERVICE_UUID_16 & 0xFF);
        adv_data[adv_len++] = (BLE_PROTOCOL_SERVICE_UUID_16 >> 8) & 0xFF;
    }

    static uint8_t rsp_data[31];
    size_t rsp_len = 0;
    uint8_t len_idx;
    // 设置广播数据

    len_idx = rsp_len;
    rsp_data[rsp_len++] = 0;  // Length
    rsp_data[rsp_len++] = 0xff;  // Manufacturer Specific Data
    rsp_data[rsp_len++] = (BLE_WIFI_CONFIG_MANUFACTURER_ID & 0xFF);
    rsp_data[rsp_len++] = (BLE_WIFI_CONFIG_MANUFACTURER_ID >> 8) & 0xFF;

    const esp_app_desc_t *p_desc = esp_app_get_description();
    int version[3] = {0};
    sscanf(p_desc->version, "%d.%d.%d", &version[0], &version[1], &version[2]);
    rsp_data[rsp_len++] = version[0] & 0xFF;
    rsp_data[rsp_len++] = version[1] & 0xFF;
    rsp_data[rsp_len++] = version[2] & 0xFF;

    rsp_data[rsp_len++] = (BLE_VERSION & 0xFF);

    if(battery_level < 0) battery_level = 0;
    if(battery_level > 100) battery_level = 100;

    rsp_data[rsp_len++] = (battery_level & 0xFF) | (charging ? 0x80 : 0x00);

    rsp_data[len_idx] = rsp_len - len_idx - 1; // 更新长度字段

    ret = esp_ble_adv_set_data(adv_data, adv_len, rsp_data, rsp_len);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to set advertising data: %d", ret);
        return ret;
    }
    
    // 开始广播
    ret = esp_ble_adv_start(100); // 100ms间隔
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to start advertising: %d", ret);
        return ret;
    }
    
    g_ble_advertising = true;
    ESP_LOGI(TAG, "Started BLE advertising with name: %s", adv_name.c_str());
    return 0;
}

int ble_wifi_config_stop_advertising(void) {
    if (!g_ble_advertising) {
        return 0;
    }
    
    int ret = esp_ble_adv_stop();
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to stop advertising: %d", ret);
        return ret;
    }
    
    g_ble_advertising = false;
    ESP_LOGI(TAG, "Stopped BLE advertising");
    return 0;
}

void ble_wifi_config_deinit(void) {
    if (!g_ble_initialized) {
        return;
    }
    
    ble_wifi_config_stop_advertising();
    
    ble_wifi_config_unregister_handlers();
    
    g_ble_initialized = false;
    ESP_LOGI(TAG, "BLE WiFi config deinitialized");
}

void ble_wifi_config_disconnect(uint16_t conn_handle) {
    if (conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        return;
    }
    
    int ret = esp_ble_disconnect(conn_handle);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to disconnect BLE connection: %d", ret);
    } else {
        ESP_LOGI(TAG, "Disconnected BLE connection, conn_id=%d", conn_handle);
    }
}

// 发送WiFi连接结果
void ble_wifi_config_send_result(uint16_t conn_id, const std::string& ssid, bool success) {
    ESP_LOGI(TAG, "Sending WiFi connection result: conn_id=%d, ssid=%s, result=%s", 
             conn_id, ssid.c_str(), success ? "success" : "failure");
    
    uint8_t resp = success ? BLE_PROTOCOL_ACK_SUCCESS : BLE_PROTOCOL_ACK_ERROR;
    uint8_t end_marker[] = {WIFI_OPT_SET_SSID, resp};
    int ret = ble_protocol_send_response(conn_id, BLE_PROTOCOL_CMD_WIFI_OPT, end_marker, sizeof(end_marker));
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send WiFi connection result: %d", ret);
    }
    
    // ========== 只在连接成功时断开蓝牙 ==========
    if (success) {
        ESP_LOGI(TAG, "WiFi connected successfully, device will reboot in 2 seconds");
    
        // 清理连接尝试记录
        ble_wifi_config_cleanup_attempt(conn_id);
        
        // 短暂延迟确保响应已发送
        vTaskDelay(pdMS_TO_TICKS(100));
        
        // 断开蓝牙连接
        ble_wifi_config_disconnect(conn_id);
        
        // 延迟等待连接完全断开
        vTaskDelay(pdMS_TO_TICKS(500));
        
        // 成功连接WiFi，重启设备
        ESP_LOGI(TAG, "Device is about to reboot...");
        vTaskDelay(pdMS_TO_TICKS(1500));
        esp_restart();
    } else {
        // ========== 关键修改：失败时保持蓝牙连接 ==========
        // 不主动断开连接，允许用户立即重新尝试配网
        ESP_LOGI(TAG, "WiFi connection failed, keeping Bluetooth connection, waiting for user reconfiguration");
        
        // 在这里发送一个额外的提示消息
        const char* hint_msg = "WiFi connection failed, please check password and try again";
        uint8_t hint_payload[64];
        size_t hint_len = ble_protocol_build_packet(0xFF, (const uint8_t*)hint_msg, strlen(hint_msg), hint_payload, sizeof(hint_payload));
        if (hint_len > 0) {
            ble_protocol_send_response(conn_id, 0xFF, hint_payload, hint_len);
        }
    }
}

} // extern "C"

// C++接口实现
BleWifiConfig& BleWifiConfig::GetInstance() {
    static BleWifiConfig instance;
    return instance;
}

bool BleWifiConfig::Initialize() {
    
    return ble_wifi_config_init() == 0;
}

bool BleWifiConfig::StartAdvertising(const std::string& ap_ssid, int battery_level, bool charging) {

    return ble_wifi_config_start_advertising(ap_ssid.c_str(), battery_level, charging) == 0;
}

bool BleWifiConfig::RestartAdvertising() {
    if (!g_ble_initialized) {
        ESP_LOGE(TAG, "BLE not initialized");
        return false;
    }
    
    // 停止当前广播
    if (g_ble_advertising) {
        ble_wifi_config_stop_advertising();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // 获取当前AP SSID和电池信息
    auto& wifi_ap = WifiConfigurationAp::GetInstance();
    std::string ap_ssid = wifi_ap.GetSsid();
    int battery_level = 0;
    bool charging = false, discharging = false;
    auto& board = Board::GetInstance();
    board.GetBatteryLevel(battery_level, charging, discharging);
    
    // 重新启动广播
    return ble_wifi_config_start_advertising(ap_ssid.c_str(), battery_level, charging) == 0;
}

bool BleWifiConfig::StopAdvertising() {
    return ble_wifi_config_stop_advertising() == 0;
}

void BleWifiConfig::Disconnect() {
    if(g_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        ble_wifi_config_disconnect(g_conn_handle);

        while(g_conn_handle != BLE_HS_CONN_HANDLE_NONE){
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

void BleWifiConfig::Deinitialize() {
    ble_wifi_config_deinit();
}

void BleWifiConfig::SetOnWifiConfigChanged(std::function<void(const std::string&, const std::string&)> callback) {
    g_wifi_config_callback = callback;
}

void BleWifiConfig::SetOnWifiConfigChangedWithConnId(std::function<void(uint16_t, const std::string&, const std::string&)> callback) {
    g_wifi_config_callback_with_conn_id = callback;
}

bool BleWifiConfig::IsConnected() { 
    return g_conn_handle != BLE_HS_CONN_HANDLE_NONE; 
}

void BleWifiConfig::SendWifiConnectResult(uint16_t conn_id, const std::string& ssid, bool success) {
    ble_wifi_config_send_result(conn_id, ssid, success);
}
