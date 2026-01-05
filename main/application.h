#ifndef _APPLICATION_H_
#define _APPLICATION_H_

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>
#include <esp_timer.h>

#include <string>
#include <mutex>
#include <deque>
#include <vector>
#include <memory>
#include <unordered_set>

#include "protocol.h"
#include "ota.h"
#include "audio_service.h"
#include "device_state_event.h"

#include "ble_wifi_config.h"
#include "bluetooth_config.h"

#include "esp_http_client.h"

#define MAIN_EVENT_SCHEDULE (1 << 0)
#define MAIN_EVENT_SEND_AUDIO (1 << 1)
#define MAIN_EVENT_WAKE_WORD_DETECTED (1 << 2)
#define MAIN_EVENT_VAD_CHANGE (1 << 3)
#define MAIN_EVENT_ERROR (1 << 4)
#define MAIN_EVENT_CHECK_NEW_VERSION_DONE (1 << 5)
#define MAIN_EVENT_CLOCK_TICK (1 << 6)

// 全局HTTP服务器配置
extern const char* HTTP_API_WRITE_URL;  // 写入URL
extern const char* HTTP_API_READ_URL;   // 读取URL


enum AecMode {
    kAecOff,
    kAecOnDeviceSide,
    kAecOnServerSide,
};

class Application {
public:
    static Application& GetInstance() {
        static Application instance;
        return instance;
    }
    // 删除拷贝构造函数和赋值运算符
    Application(const Application&) = delete;
    Application& operator=(const Application&) = delete;

    void Start();
    void MainEventLoop();
    DeviceState GetDeviceState() const { return device_state_; }
    bool IsVoiceDetected() const { return audio_service_.IsVoiceDetected(); }
    void Schedule(std::function<void()> callback);
    void SetDeviceState(DeviceState state);
    void Alert(const char* status, const char* message, const char* emotion = "", const std::string_view& sound = "");
    void DismissAlert();
    void AbortSpeaking(AbortReason reason);
    void ToggleChatState();
    void StartListening();
    void StopListening();
    void Reboot();
    void WakeWordInvoke(const std::string& wake_word);
    bool UpgradeFirmware(Ota& ota, const std::string& url = "");
    bool CanEnterSleepMode();
    void SendMcpMessage(const std::string& payload);
    void SetAecMode(AecMode mode);
    AecMode GetAecMode() const { return aec_mode_; }
    void PlaySound(const std::string_view& sound);
    AudioService& GetAudioService() { return audio_service_; }

    // 发送传感器事件
    void SendSensorEvent(const std::string& event_type);
    bool IsStarted() const { return is_started_; }

    // HTTP 客户端方法
    bool MakeHttpGetRequest(const std::string& url);
    bool MakeHttpPostRequest(const std::string& url, const std::string& json_data);
    bool MakeHttpPostRequest(const std::string& metric_name,
                            int metric_value,
                            const std::string& user_id,
                            const std::string& post_url = HTTP_API_WRITE_URL);
    esp_err_t HttpEventHandler(esp_http_client_event_t *evt);
    void ProcessHttpResponse(const std::string& response_data);

    // 设置系统时间
    bool SetSystemTimeFromString(const char* time_str);

    // 启动蓝牙配网功能
    void EnableBleWifiConfig(bool enable) { ble_wifi_config_enabled_ = enable; }
    bool IsBleWifiConfigEnabled() const { return ble_wifi_config_enabled_; }

    // 表情容量编码相关方法
    bool HasCapacityEncoding(const std::string& encoding) const;
    const std::unordered_set<std::string>& GetCapacityEncodings() const { return capacity_encodings_; }
    void ClearCapacityEncodings() { capacity_encodings_.clear(); }

private:
    Application();
    ~Application();

    std::mutex mutex_;
    std::deque<std::function<void()>> main_tasks_;
    std::unique_ptr<Protocol> protocol_;
    EventGroupHandle_t event_group_ = nullptr;
    esp_timer_handle_t clock_timer_handle_ = nullptr;
    volatile DeviceState device_state_ = kDeviceStateUnknown;
    ListeningMode listening_mode_ = kListeningModeAutoStop;
    AecMode aec_mode_ = kAecOff;
    std::string last_error_message_;
    AudioService audio_service_;

    bool has_server_time_ = false;
    bool aborted_ = false;
    int clock_ticks_ = 0;
    TaskHandle_t check_new_version_task_handle_ = nullptr;
    TaskHandle_t main_event_loop_task_handle_ = nullptr;

    void OnWakeWordDetected();
    void CheckNewVersion(Ota& ota);
    void CheckAssetsVersion();
    void ShowActivationCode(const std::string& code, const std::string& message);
    void SetListeningMode(ListeningMode mode);


    bool is_started_ = false;
    bool MakeHttpRequest(const std::string& url, const std::string& method, const std::string& content_type, const std::string& body);
    std::string current_http_response;
    bool ble_wifi_config_enabled_ = true;

    // 存储从服务器获取的capacity_encoding值
    std::unordered_set<std::string> capacity_encodings_;
};


class TaskPriorityReset {
public:
    TaskPriorityReset(BaseType_t priority) {
        original_priority_ = uxTaskPriorityGet(NULL);
        vTaskPrioritySet(NULL, priority);
    }
    ~TaskPriorityReset() {
        vTaskPrioritySet(NULL, original_priority_);
    }

private:
    BaseType_t original_priority_;
};

#endif // _APPLICATION_H_