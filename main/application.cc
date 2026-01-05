#include "application.h"
#include "board.h"
#include "display.h"
#include "system_info.h"
#include "audio_codec.h"
#include "mqtt_protocol.h"
#include "websocket_protocol.h"
#include "assets/lang_config.h"
#include "mcp_server.h"
#include "assets.h"
#include "settings.h"

#include <cstring>
#include <esp_log.h>
#include <cJSON.h>
#include <driver/gpio.h>
#include <arpa/inet.h>
#include <font_awesome.h>

#include <esp_http_client.h>

#include "nvs_flash.h"
#include "nvs.h"
#include <time.h>
// #include "http_client.h"

#include "ble_wifi_integration.h"
#include "ble_ota.h"
#include <ssid_manager.h>
#include <font_awesome.h>


#define TAG "Application"


// 全局HTTP服务器配置
// const char* HTTP_SERVER_IP = "47.116.117.182";
// const int HTTP_SERVER_PORT = 5000;
// const char* HTTP_SERVER_BASE_URL = "http://47.116.117.182:5000";
const char* HTTP_API_WRITE_URL = "http://47.116.117.182:5000/api/v1/write";
const char* HTTP_API_READ_URL = "http://47.116.117.182:5000/api/v1/read";


static const char* const STATE_STRINGS[] = {
    "unknown",
    "starting",
    "configuring",
    "idle",
    "connecting",
    "listening",
    "speaking",
    "upgrading",
    "activating",
    "audio_testing",
    "fatal_error",
    "invalid_state"
};

Application::Application() {
    event_group_ = xEventGroupCreate();

#if CONFIG_USE_DEVICE_AEC && CONFIG_USE_SERVER_AEC
#error "CONFIG_USE_DEVICE_AEC and CONFIG_USE_SERVER_AEC cannot be enabled at the same time"
#elif CONFIG_USE_DEVICE_AEC
    aec_mode_ = kAecOnDeviceSide;
#elif CONFIG_USE_SERVER_AEC
    aec_mode_ = kAecOnServerSide;
#else
    aec_mode_ = kAecOff;
#endif

    esp_timer_create_args_t clock_timer_args = {
        .callback = [](void* arg) {
            Application* app = (Application*)arg;
            xEventGroupSetBits(app->event_group_, MAIN_EVENT_CLOCK_TICK);
        },
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "clock_timer",
        .skip_unhandled_events = true
    };
    esp_timer_create(&clock_timer_args, &clock_timer_handle_);
}

Application::~Application() {
    if (clock_timer_handle_ != nullptr) {
        esp_timer_stop(clock_timer_handle_);
        esp_timer_delete(clock_timer_handle_);
    }
    vEventGroupDelete(event_group_);
}

void Application::CheckAssetsVersion() {
    auto& board = Board::GetInstance();
    auto display = board.GetDisplay();
    auto& assets = Assets::GetInstance();

    if (!assets.partition_valid()) {
        ESP_LOGW(TAG, "Assets partition is disabled for board %s", BOARD_NAME);
        return;
    }
    
    Settings settings("assets", true);
    // Check if there is a new assets need to be downloaded
    std::string download_url = settings.GetString("download_url");

    if (!download_url.empty()) {
        settings.EraseKey("download_url");

        char message[256];
        snprintf(message, sizeof(message), Lang::Strings::FOUND_NEW_ASSETS, download_url.c_str());
        Alert(Lang::Strings::LOADING_ASSETS, message, "cloud_arrow_down", Lang::Sounds::OGG_UPGRADE);
        
        // Wait for the audio service to be idle for 3 seconds
        vTaskDelay(pdMS_TO_TICKS(3000));
        SetDeviceState(kDeviceStateUpgrading);
        board.SetPowerSaveMode(false);
        display->SetChatMessage("system", Lang::Strings::PLEASE_WAIT);

        bool success = assets.Download(download_url, [display](int progress, size_t speed) -> void {
            std::thread([display, progress, speed]() {
                char buffer[32];
                snprintf(buffer, sizeof(buffer), "%d%% %uKB/s", progress, speed / 1024);
                display->SetChatMessage("system", buffer);
            }).detach();
        });

        board.SetPowerSaveMode(true);
        vTaskDelay(pdMS_TO_TICKS(1000));

        if (!success) {
            Alert(Lang::Strings::ERROR, Lang::Strings::DOWNLOAD_ASSETS_FAILED, "circle_xmark", Lang::Sounds::OGG_EXCLAMATION);
            vTaskDelay(pdMS_TO_TICKS(2000));
            return;
        }
    }

    // Apply assets
    assets.Apply();
    display->SetChatMessage("system", "");
    display->SetEmotion("microchip_ai");
}

void Application::CheckNewVersion(Ota& ota) {
    const int MAX_RETRY = 10;
    int retry_count = 0;
    int retry_delay = 10; // 初始重试延迟为10秒

    auto& board = Board::GetInstance();
    while (true) {
        SetDeviceState(kDeviceStateActivating);
        auto display = board.GetDisplay();
        display->SetStatus(Lang::Strings::CHECKING_NEW_VERSION);

        if (!ota.CheckVersion()) {
            retry_count++;
            if (retry_count >= MAX_RETRY) {
                ESP_LOGE(TAG, "Too many retries, exit version check");
                return;
            }

            char buffer[256];
            snprintf(buffer, sizeof(buffer), Lang::Strings::CHECK_NEW_VERSION_FAILED, retry_delay, ota.GetCheckVersionUrl().c_str());
            Alert(Lang::Strings::ERROR, buffer, "cloud_slash", Lang::Sounds::OGG_EXCLAMATION);

            ESP_LOGW(TAG, "Check new version failed, retry in %d seconds (%d/%d)", retry_delay, retry_count, MAX_RETRY);
            for (int i = 0; i < retry_delay; i++) {
                vTaskDelay(pdMS_TO_TICKS(1000));
                if (device_state_ == kDeviceStateIdle) {
                    break;
                }
            }
            retry_delay *= 2; // 每次重试后延迟时间翻倍
            continue;
        }
        retry_count = 0;
        retry_delay = 10; // 重置重试延迟时间

        if (ota.HasNewVersion()) {
            if (UpgradeFirmware(ota)) {
                return; // This line will never be reached after reboot
            }
            // If upgrade failed, continue to normal operation (don't break, just fall through)
        }

        // No new version, mark the current version as valid
        ota.MarkCurrentVersionValid();
        if (!ota.HasActivationCode() && !ota.HasActivationChallenge()) {
            xEventGroupSetBits(event_group_, MAIN_EVENT_CHECK_NEW_VERSION_DONE);
            // Exit the loop if done checking new version
            break;
        }

        display->SetStatus(Lang::Strings::ACTIVATION);
        // Activation code is shown to the user and waiting for the user to input
        if (ota.HasActivationCode()) {
            ShowActivationCode(ota.GetActivationCode(), ota.GetActivationMessage());
        }

        // This will block the loop until the activation is done or timeout
        for (int i = 0; i < 10; ++i) {
            ESP_LOGI(TAG, "Activating... %d/%d", i + 1, 10);
            esp_err_t err = ota.Activate();
            if (err == ESP_OK) {
                xEventGroupSetBits(event_group_, MAIN_EVENT_CHECK_NEW_VERSION_DONE);
                break;
            } else if (err == ESP_ERR_TIMEOUT) {
                vTaskDelay(pdMS_TO_TICKS(3000));
            } else {
                vTaskDelay(pdMS_TO_TICKS(10000));
            }
            if (device_state_ == kDeviceStateIdle) {
                break;
            }
        }
    }
}

void Application::ShowActivationCode(const std::string& code, const std::string& message) {
    struct digit_sound {
        char digit;
        const std::string_view& sound;
    };
    static const std::array<digit_sound, 10> digit_sounds{{
        digit_sound{'0', Lang::Sounds::OGG_0},
        digit_sound{'1', Lang::Sounds::OGG_1}, 
        digit_sound{'2', Lang::Sounds::OGG_2},
        digit_sound{'3', Lang::Sounds::OGG_3},
        digit_sound{'4', Lang::Sounds::OGG_4},
        digit_sound{'5', Lang::Sounds::OGG_5},
        digit_sound{'6', Lang::Sounds::OGG_6},
        digit_sound{'7', Lang::Sounds::OGG_7},
        digit_sound{'8', Lang::Sounds::OGG_8},
        digit_sound{'9', Lang::Sounds::OGG_9}
    }};

    // This sentence uses 9KB of SRAM, so we need to wait for it to finish
    Alert(Lang::Strings::ACTIVATION, message.c_str(), "link", Lang::Sounds::OGG_ACTIVATION);

    for (const auto& digit : code) {
        auto it = std::find_if(digit_sounds.begin(), digit_sounds.end(),
            [digit](const digit_sound& ds) { return ds.digit == digit; });
        if (it != digit_sounds.end()) {
            audio_service_.PlaySound(it->sound);
        }
    }
}

void Application::Alert(const char* status, const char* message, const char* emotion, const std::string_view& sound) {
    ESP_LOGW(TAG, "Alert [%s] %s: %s", emotion, status, message);
    auto display = Board::GetInstance().GetDisplay();
    display->SetStatus(status);
    display->SetEmotion(emotion);
    display->SetChatMessage("system", message);
    if (!sound.empty()) {
        audio_service_.PlaySound(sound);
    }
}

void Application::DismissAlert() {
    if (device_state_ == kDeviceStateIdle) {
        auto display = Board::GetInstance().GetDisplay();
        display->SetStatus(Lang::Strings::STANDBY);
        display->SetEmotion("neutral");
        display->SetChatMessage("system", "");
    }
}

void Application::ToggleChatState() {
    if (device_state_ == kDeviceStateActivating) {
        SetDeviceState(kDeviceStateIdle);
        return;
    } else if (device_state_ == kDeviceStateWifiConfiguring) {
        audio_service_.EnableAudioTesting(true);
        SetDeviceState(kDeviceStateAudioTesting);
        return;
    } else if (device_state_ == kDeviceStateAudioTesting) {
        audio_service_.EnableAudioTesting(false);
        SetDeviceState(kDeviceStateWifiConfiguring);
        return;
    }

    if (!protocol_) {
        ESP_LOGE(TAG, "Protocol not initialized");
        return;
    }

    if (device_state_ == kDeviceStateIdle) {
        Schedule([this]() {
            if (!protocol_->IsAudioChannelOpened()) {
                SetDeviceState(kDeviceStateConnecting);
                if (!protocol_->OpenAudioChannel()) {
                    return;
                }
            }

            SetListeningMode(aec_mode_ == kAecOff ? kListeningModeAutoStop : kListeningModeRealtime);
        });
    } else if (device_state_ == kDeviceStateSpeaking) {
        Schedule([this]() {
            AbortSpeaking(kAbortReasonNone);
        });
    } else if (device_state_ == kDeviceStateListening) {
        Schedule([this]() {
            protocol_->CloseAudioChannel();
        });
    }
}

void Application::StartListening() {
    if (device_state_ == kDeviceStateActivating) {
        SetDeviceState(kDeviceStateIdle);
        return;
    } else if (device_state_ == kDeviceStateWifiConfiguring) {
        audio_service_.EnableAudioTesting(true);
        SetDeviceState(kDeviceStateAudioTesting);
        return;
    }

    if (!protocol_) {
        ESP_LOGE(TAG, "Protocol not initialized");
        return;
    }
    
    if (device_state_ == kDeviceStateIdle) {
        Schedule([this]() {
            if (!protocol_->IsAudioChannelOpened()) {
                SetDeviceState(kDeviceStateConnecting);
                if (!protocol_->OpenAudioChannel()) {
                    return;
                }
            }

            SetListeningMode(kListeningModeManualStop);
        });
    } else if (device_state_ == kDeviceStateSpeaking) {
        Schedule([this]() {
            AbortSpeaking(kAbortReasonNone);
            SetListeningMode(kListeningModeManualStop);
        });
    }
}

void Application::StopListening() {
    if (device_state_ == kDeviceStateAudioTesting) {
        audio_service_.EnableAudioTesting(false);
        SetDeviceState(kDeviceStateWifiConfiguring);
        return;
    }

    const std::array<int, 3> valid_states = {
        kDeviceStateListening,
        kDeviceStateSpeaking,
        kDeviceStateIdle,
    };
    // If not valid, do nothing
    if (std::find(valid_states.begin(), valid_states.end(), device_state_) == valid_states.end()) {
        return;
    }

    Schedule([this]() {
        if (device_state_ == kDeviceStateListening) {
            protocol_->SendStopListening();
            SetDeviceState(kDeviceStateIdle);
        }
    });
}

bool IsWifiConfigMode() {
    auto& ssid_manager = SsidManager::GetInstance();
    auto ssid_list = ssid_manager.GetSsidList();
    Settings settings("wifi", true);
    return settings.GetInt("force_ap") == 1 || ssid_list.empty();
}


void Application::Start() {
    auto& board = Board::GetInstance();
    SetDeviceState(kDeviceStateStarting);

    /* Setup the display */
    auto display = board.GetDisplay();

    // Print board name/version info
    display->SetChatMessage("system", SystemInfo::GetUserAgent().c_str());

    /* Setup the audio service */
    auto codec = board.GetAudioCodec();
    audio_service_.Initialize(codec);
    audio_service_.Start();

    AudioServiceCallbacks callbacks;
    callbacks.on_send_queue_available = [this]() {
        xEventGroupSetBits(event_group_, MAIN_EVENT_SEND_AUDIO);
    };
    callbacks.on_wake_word_detected = [this](const std::string& wake_word) {
        xEventGroupSetBits(event_group_, MAIN_EVENT_WAKE_WORD_DETECTED);
    };
    callbacks.on_vad_change = [this](bool speaking) {
        xEventGroupSetBits(event_group_, MAIN_EVENT_VAD_CHANGE);
    };
    audio_service_.SetCallbacks(callbacks);

    // Start the main event loop task with priority 3
    xTaskCreate([](void* arg) {
        ((Application*)arg)->MainEventLoop();
        vTaskDelete(NULL);
    }, "main_event_loop", 2048 * 4, this, 3, &main_event_loop_task_handle_);

    /* Start the clock timer to update the status bar */
    esp_timer_start_periodic(clock_timer_handle_, 1000000);



    if (IsWifiConfigMode() && ble_wifi_config_enabled_) {
        BleWifiIntegration::StartBleWifiConfig();
        
        // 同时启动BLE OTA功能
        auto& ble_ota = BleOta::GetInstance();
        if (ble_ota.Initialize()) {
            ESP_LOGI(TAG, "BLE OTA service initialized successfully");
            
            // 设置OTA进度回调（可选）
            ble_ota.SetProgressCallback([](int progress) {
                ESP_LOGI(TAG, "BLE OTA progress: %d%%", progress);
            });
            
            // 设置OTA完成回调（可选）
            ble_ota.SetCompleteCallback([](bool success) {
                if (success) {
                    ESP_LOGI(TAG, "BLE OTA completed successfully, restarting...");
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    esp_restart();
                } else {
                    ESP_LOGE(TAG, "BLE OTA failed");
                }
            });
        } else {
            ESP_LOGE(TAG, "Failed to initialize BLE OTA service");
        }
    }


    /* Wait for the network to be ready */
    board.StartNetwork();

    // Update the status bar immediately to show the network state
    display->UpdateStatusBar(true);



    // 等待网络连接稳定
    vTaskDelay(pdMS_TO_TICKS(100));

    // 转换字符后，使用HTTP客户端方法执行GET请求
    ESP_LOGI(TAG, "=================== HTTP Get ===================");
    std::string device_id;
    for (char c : SystemInfo::GetMacAddress()) {
        if (c == ':') device_id += "%3A";
        else device_id += c;
    }

    // 使用完整的API读取URL
    std::string get_url = std::string(HTTP_API_READ_URL) + "/" + device_id;
    MakeHttpGetRequest(get_url);



    // Check for new assets version
    CheckAssetsVersion();

    // Check for new firmware version or get the MQTT broker address
    Ota ota;
    // CheckNewVersion(ota);

    // Initialize the protocol
    display->SetStatus(Lang::Strings::LOADING_PROTOCOL);

    // Add MCP common tools before initializing the protocol
    auto& mcp_server = McpServer::GetInstance();
    mcp_server.AddCommonTools();
    mcp_server.AddUserOnlyTools();

    // if (ota.HasMqttConfig()) {
    //     protocol_ = std::make_unique<MqttProtocol>();
    // } else if (ota.HasWebsocketConfig()) {
    //     protocol_ = std::make_unique<WebsocketProtocol>();
    // } else {
    //     ESP_LOGW(TAG, "No protocol specified in the OTA config, using MQTT");
    //     protocol_ = std::make_unique<MqttProtocol>();
    // }
    protocol_ = std::make_unique<WebsocketProtocol>();

    protocol_->OnConnected([this]() {
        DismissAlert();
    });

    protocol_->OnNetworkError([this](const std::string& message) {
        last_error_message_ = message;
        xEventGroupSetBits(event_group_, MAIN_EVENT_ERROR);
    });
    protocol_->OnIncomingAudio([this](std::unique_ptr<AudioStreamPacket> packet) {
        if (device_state_ == kDeviceStateSpeaking) {
            audio_service_.PushPacketToDecodeQueue(std::move(packet));
        }
    });
    protocol_->OnAudioChannelOpened([this, codec, &board]() {
        board.SetPowerSaveMode(false);
        if (protocol_->server_sample_rate() != codec->output_sample_rate()) {
            ESP_LOGW(TAG, "Server sample rate %d does not match device output sample rate %d, resampling may cause distortion",
                protocol_->server_sample_rate(), codec->output_sample_rate());
        }
    });
    protocol_->OnAudioChannelClosed([this, &board]() {
        board.SetPowerSaveMode(true);
        Schedule([this]() {
            auto display = Board::GetInstance().GetDisplay();
            display->SetChatMessage("system", "");
            SetDeviceState(kDeviceStateIdle);
        });
    });
    protocol_->OnIncomingJson([this, display](const cJSON* root) {
        char* json_str = cJSON_PrintUnformatted(root);
        ESP_LOGI(TAG, "Received JSON ============> %s", json_str);
        // Parse JSON data
        auto type = cJSON_GetObjectItem(root, "type");
        if (strcmp(type->valuestring, "tts") == 0) {
            auto state = cJSON_GetObjectItem(root, "state");
            if (strcmp(state->valuestring, "start") == 0) {
                Schedule([this]() {
                    aborted_ = false;
                    if (device_state_ == kDeviceStateIdle || device_state_ == kDeviceStateListening) {
                        SetDeviceState(kDeviceStateSpeaking);
                    }
                });
            } else if (strcmp(state->valuestring, "stop") == 0) {
                Schedule([this]() {
                    if (device_state_ == kDeviceStateSpeaking) {
                        if (listening_mode_ == kListeningModeManualStop) {
                            SetDeviceState(kDeviceStateIdle);
                        } else {
                            SetDeviceState(kDeviceStateListening);
                        }
                    }
                });
            } else if (strcmp(state->valuestring, "sentence_start") == 0) {
                auto text = cJSON_GetObjectItem(root, "text");
                if (cJSON_IsString(text)) {
                    ESP_LOGI(TAG, "<< %s", text->valuestring);
                    Schedule([this, display, message = std::string(text->valuestring)]() {
                        display->SetChatMessage("assistant", message.c_str());
                    });
                }
            }
        } else if (strcmp(type->valuestring, "stt") == 0) {
            auto text = cJSON_GetObjectItem(root, "text");
            if (cJSON_IsString(text)) {
                ESP_LOGI(TAG, ">> %s", text->valuestring);
                Schedule([this, display, message = std::string(text->valuestring)]() {
                    display->SetChatMessage("user", message.c_str());
                });
            }
        } else if (strcmp(type->valuestring, "llm") == 0) {
            auto emotion = cJSON_GetObjectItem(root, "emotion");
            if (cJSON_IsString(emotion)) {
                Schedule([this, display, emotion_str = std::string(emotion->valuestring)]() {
                    display->SetEmotion(emotion_str.c_str());
                });
            }
        } else if (strcmp(type->valuestring, "mcp") == 0) {
            auto payload = cJSON_GetObjectItem(root, "payload");
            if (cJSON_IsObject(payload)) {
                McpServer::GetInstance().ParseMessage(payload);
            }
        } else if (strcmp(type->valuestring, "system") == 0) {
            auto command = cJSON_GetObjectItem(root, "command");
            if (cJSON_IsString(command)) {
                ESP_LOGI(TAG, "System command: %s", command->valuestring);
                if (strcmp(command->valuestring, "reboot") == 0) {
                    // Do a reboot if user requests a OTA update
                    Schedule([this]() {
                        Reboot();
                    });
                } else {
                    ESP_LOGW(TAG, "Unknown system command: %s", command->valuestring);
                }
            }
        } else if (strcmp(type->valuestring, "shake") == 0) {
            auto payload = cJSON_GetObjectItem(root, "payload");
            if (cJSON_IsObject(payload)) {
                std::unique_ptr<char, decltype(&cJSON_free)> payload_json(cJSON_PrintUnformatted(payload), &cJSON_free);
                std::string payload_str(payload_json.get());

                // 获取 shake_num 和 message 字段
                auto shake_num = cJSON_GetObjectItem(payload, "shake_num");
                auto message = cJSON_GetObjectItem(payload, "message");

                if (cJSON_IsNumber(shake_num) && cJSON_IsString(message)) {
                    int shake_value = shake_num->valueint;
                    const char* message_text = message->valuestring;
                    
                    ESP_LOGI(TAG, "Shake number: %d, Message: %s", shake_value, message_text);
                }

                // // 从服务端接收消息，并改变表情（现在不需要服务端再发送回来，所以此处逻辑先注释）
                // auto& board = Board::GetInstance();
                // auto display = board.GetDisplay();
                // // 改变状态、表情，5秒后恢复正常表情
                // display->SetStatus(Lang::Strings::STANDBY);
                // display->SetEmotion("sad"); // 设置伤心表情
                // vTaskDelay(pdMS_TO_TICKS(5000));    // 等待5秒
                // display->SetEmotion("neutral"); // 恢复常态表情
            } else {
                ESP_LOGW(TAG, "Invalid custom message format: missing payload");
            }
        } else if (strcmp(type->valuestring, "alert") == 0) {
            auto status = cJSON_GetObjectItem(root, "status");
            auto message = cJSON_GetObjectItem(root, "message");
            auto emotion = cJSON_GetObjectItem(root, "emotion");
            if (cJSON_IsString(status) && cJSON_IsString(message) && cJSON_IsString(emotion)) {
                Alert(status->valuestring, message->valuestring, emotion->valuestring, Lang::Sounds::OGG_VIBRATION);
            } else {
                ESP_LOGW(TAG, "Alert command requires status, message and emotion");
            }
#if CONFIG_RECEIVE_CUSTOM_MESSAGE
        } else if (strcmp(type->valuestring, "custom") == 0) {
            auto payload = cJSON_GetObjectItem(root, "payload");
            ESP_LOGI(TAG, "Received custom message: %s", cJSON_PrintUnformatted(root));
            if (cJSON_IsObject(payload)) {
                Schedule([this, display, payload_str = std::string(cJSON_PrintUnformatted(payload))]() {
                    display->SetChatMessage("system", payload_str.c_str());
                });
            } else {
                ESP_LOGW(TAG, "Invalid custom message format: missing payload");
            }
#endif
        } else {
            ESP_LOGW(TAG, "Unknown message type: %s", type->valuestring);
        }
    });
    bool protocol_started = protocol_->Start();

    SystemInfo::PrintHeapStats();
    SetDeviceState(kDeviceStateIdle);

    has_server_time_ = ota.HasServerTime();
    if (protocol_started) {
        std::string message = std::string(Lang::Strings::VERSION) + ota.GetCurrentVersion();
        display->ShowNotification(message.c_str());
        display->SetChatMessage("system", "");
        // Play the success sound to indicate the device is ready
        audio_service_.PlaySound(Lang::Sounds::OGG_SUCCESS);

        is_started_ = true;
    }
}

// Add a async task to MainLoop
void Application::Schedule(std::function<void()> callback) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        main_tasks_.push_back(std::move(callback));
    }
    xEventGroupSetBits(event_group_, MAIN_EVENT_SCHEDULE);
}

// The Main Event Loop controls the chat state and websocket connection
// If other tasks need to access the websocket or chat state,
// they should use Schedule to call this function
void Application::MainEventLoop() {
    while (true) {
        auto bits = xEventGroupWaitBits(event_group_, MAIN_EVENT_SCHEDULE |
            MAIN_EVENT_SEND_AUDIO |
            MAIN_EVENT_WAKE_WORD_DETECTED |
            MAIN_EVENT_VAD_CHANGE |
            MAIN_EVENT_CLOCK_TICK |
            MAIN_EVENT_ERROR, pdTRUE, pdFALSE, portMAX_DELAY);

        if (bits & MAIN_EVENT_ERROR) {
            SetDeviceState(kDeviceStateIdle);
            Alert(Lang::Strings::ERROR, last_error_message_.c_str(), "circle_xmark", Lang::Sounds::OGG_EXCLAMATION);
        }

        if (bits & MAIN_EVENT_SEND_AUDIO) {
            while (auto packet = audio_service_.PopPacketFromSendQueue()) {
                if (protocol_ && !protocol_->SendAudio(std::move(packet))) {
                    break;
                }
            }
        }

        if (bits & MAIN_EVENT_WAKE_WORD_DETECTED) {
            OnWakeWordDetected();
        }

        if (bits & MAIN_EVENT_VAD_CHANGE) {
            if (device_state_ == kDeviceStateListening) {
                auto led = Board::GetInstance().GetLed();
                led->OnStateChanged();
            }
        }

        if (bits & MAIN_EVENT_SCHEDULE) {
            std::unique_lock<std::mutex> lock(mutex_);
            auto tasks = std::move(main_tasks_);
            lock.unlock();
            for (auto& task : tasks) {
                task();
            }
        }

        if (bits & MAIN_EVENT_CLOCK_TICK) {
            clock_ticks_++;
            auto display = Board::GetInstance().GetDisplay();
            display->UpdateStatusBar();
        
            // Print the debug info every 10 seconds
            if (clock_ticks_ % 10 == 0) {
                // SystemInfo::PrintTaskCpuUsage(pdMS_TO_TICKS(1000));
                // SystemInfo::PrintTaskList();
                SystemInfo::PrintHeapStats();
            }
        }
    }
}

void Application::OnWakeWordDetected() {
    if (!protocol_) {
        return;
    }

    if (device_state_ == kDeviceStateIdle) {
        audio_service_.EncodeWakeWord();

        if (!protocol_->IsAudioChannelOpened()) {
            SetDeviceState(kDeviceStateConnecting);
            if (!protocol_->OpenAudioChannel()) {
                audio_service_.EnableWakeWordDetection(true);
                return;
            }
        }

        auto wake_word = audio_service_.GetLastWakeWord();
        ESP_LOGI(TAG, "Wake word detected: %s", wake_word.c_str());
#if CONFIG_SEND_WAKE_WORD_DATA
        // Encode and send the wake word data to the server
        while (auto packet = audio_service_.PopWakeWordPacket()) {
            protocol_->SendAudio(std::move(packet));
        }
        // Set the chat state to wake word detected
        protocol_->SendWakeWordDetected(wake_word);
        SetListeningMode(aec_mode_ == kAecOff ? kListeningModeAutoStop : kListeningModeRealtime);
#else
        SetListeningMode(aec_mode_ == kAecOff ? kListeningModeAutoStop : kListeningModeRealtime);
        // Play the pop up sound to indicate the wake word is detected
        audio_service_.PlaySound(Lang::Sounds::OGG_POPUP);
#endif
    } else if (device_state_ == kDeviceStateSpeaking) {
        AbortSpeaking(kAbortReasonWakeWordDetected);
    } else if (device_state_ == kDeviceStateActivating) {
        SetDeviceState(kDeviceStateIdle);
    }
}

void Application::AbortSpeaking(AbortReason reason) {
    ESP_LOGI(TAG, "Abort speaking");
    aborted_ = true;
    if (protocol_) {
        protocol_->SendAbortSpeaking(reason);
    }
}

void Application::SetListeningMode(ListeningMode mode) {
    listening_mode_ = mode;
    SetDeviceState(kDeviceStateListening);
}

void Application::SetDeviceState(DeviceState state) {
    if (device_state_ == state) {
        return;
    }
    
    clock_ticks_ = 0;
    auto previous_state = device_state_;
    device_state_ = state;
    ESP_LOGI(TAG, "STATE: %s", STATE_STRINGS[device_state_]);

    // Send the state change event
    DeviceStateEventManager::GetInstance().PostStateChangeEvent(previous_state, state);

    auto& board = Board::GetInstance();
    auto display = board.GetDisplay();
    auto led = board.GetLed();
    led->OnStateChanged();
    switch (state) {
        case kDeviceStateUnknown:
        case kDeviceStateIdle:
            display->SetStatus(Lang::Strings::STANDBY);
            display->SetEmotion("neutral");
            audio_service_.EnableVoiceProcessing(false);
            audio_service_.EnableWakeWordDetection(true);
            break;
        case kDeviceStateConnecting:
            display->SetStatus(Lang::Strings::CONNECTING);
            display->SetEmotion("neutral");
            display->SetChatMessage("system", "");
            break;
        case kDeviceStateListening:
            display->SetStatus(Lang::Strings::LISTENING);
            display->SetEmotion("neutral");

            // Make sure the audio processor is running
            if (!audio_service_.IsAudioProcessorRunning()) {
                // Send the start listening command
                protocol_->SendStartListening(listening_mode_);
                audio_service_.EnableVoiceProcessing(true);
                audio_service_.EnableWakeWordDetection(false);
            }
            break;
        case kDeviceStateSpeaking:
            display->SetStatus(Lang::Strings::SPEAKING);

            if (listening_mode_ != kListeningModeRealtime) {
                audio_service_.EnableVoiceProcessing(false);
                // Only AFE wake word can be detected in speaking mode
                audio_service_.EnableWakeWordDetection(audio_service_.IsAfeWakeWord());
            }
            audio_service_.ResetDecoder();
            break;
        default:
            // Do nothing
            break;
    }
}

void Application::Reboot() {
    ESP_LOGI(TAG, "Rebooting...");
    // Disconnect the audio channel
    if (protocol_ && protocol_->IsAudioChannelOpened()) {
        protocol_->CloseAudioChannel();
    }
    protocol_.reset();
    audio_service_.Stop();

    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
}

bool Application::UpgradeFirmware(Ota& ota, const std::string& url) {
    auto& board = Board::GetInstance();
    auto display = board.GetDisplay();
    
    // Use provided URL or get from OTA object
    std::string upgrade_url = url.empty() ? ota.GetFirmwareUrl() : url;
    std::string version_info = url.empty() ? ota.GetFirmwareVersion() : "(Manual upgrade)";
    
    // Close audio channel if it's open
    if (protocol_ && protocol_->IsAudioChannelOpened()) {
        ESP_LOGI(TAG, "Closing audio channel before firmware upgrade");
        protocol_->CloseAudioChannel();
    }
    ESP_LOGI(TAG, "Starting firmware upgrade from URL: %s", upgrade_url.c_str());
    
    Alert(Lang::Strings::OTA_UPGRADE, Lang::Strings::UPGRADING, "download", Lang::Sounds::OGG_UPGRADE);
    vTaskDelay(pdMS_TO_TICKS(3000));

    SetDeviceState(kDeviceStateUpgrading);
    
    std::string message = std::string(Lang::Strings::NEW_VERSION) + version_info;
    display->SetChatMessage("system", message.c_str());

    board.SetPowerSaveMode(false);
    audio_service_.Stop();
    vTaskDelay(pdMS_TO_TICKS(1000));

    bool upgrade_success = ota.StartUpgradeFromUrl(upgrade_url, [display](int progress, size_t speed) {
        std::thread([display, progress, speed]() {
            char buffer[32];
            snprintf(buffer, sizeof(buffer), "%d%% %uKB/s", progress, speed / 1024);
            display->SetChatMessage("system", buffer);
        }).detach();
    });

    if (!upgrade_success) {
        // Upgrade failed, restart audio service and continue running
        ESP_LOGE(TAG, "Firmware upgrade failed, restarting audio service and continuing operation...");
        audio_service_.Start(); // Restart audio service
        board.SetPowerSaveMode(true); // Restore power save mode
        Alert(Lang::Strings::ERROR, Lang::Strings::UPGRADE_FAILED, "circle_xmark", Lang::Sounds::OGG_EXCLAMATION);
        vTaskDelay(pdMS_TO_TICKS(3000));
        return false;
    } else {
        // Upgrade success, reboot immediately
        ESP_LOGI(TAG, "Firmware upgrade successful, rebooting...");
        display->SetChatMessage("system", "Upgrade successful, rebooting...");
        vTaskDelay(pdMS_TO_TICKS(1000)); // Brief pause to show message
        Reboot();
        return true;
    }
}

void Application::WakeWordInvoke(const std::string& wake_word) {
    if (device_state_ == kDeviceStateIdle) {
        ToggleChatState();
        Schedule([this, wake_word]() {
            if (protocol_) {
                protocol_->SendWakeWordDetected(wake_word); 
            }
        }); 
    } else if (device_state_ == kDeviceStateSpeaking) {
        Schedule([this]() {
            AbortSpeaking(kAbortReasonNone);
        });
    } else if (device_state_ == kDeviceStateListening) {   
        Schedule([this]() {
            if (protocol_) {
                protocol_->CloseAudioChannel();
            }
        });
    }
}

bool Application::CanEnterSleepMode() {
    if (device_state_ != kDeviceStateIdle) {
        return false;
    }

    if (protocol_ && protocol_->IsAudioChannelOpened()) {
        return false;
    }

    if (!audio_service_.IsIdle()) {
        return false;
    }

    // Now it is safe to enter sleep mode
    return true;
}

void Application::SendMcpMessage(const std::string& payload) {
    if (protocol_ == nullptr) {
        return;
    }

    // Make sure you are using main thread to send MCP message
    if (xTaskGetCurrentTaskHandle() == main_event_loop_task_handle_) {
        protocol_->SendMcpMessage(payload);
    } else {
        Schedule([this, payload = std::move(payload)]() {
            protocol_->SendMcpMessage(payload);
        });
    }
}

void Application::SetAecMode(AecMode mode) {
    aec_mode_ = mode;
    Schedule([this]() {
        auto& board = Board::GetInstance();
        auto display = board.GetDisplay();
        switch (aec_mode_) {
        case kAecOff:
            audio_service_.EnableDeviceAec(false);
            display->ShowNotification(Lang::Strings::RTC_MODE_OFF);
            break;
        case kAecOnServerSide:
            audio_service_.EnableDeviceAec(false);
            display->ShowNotification(Lang::Strings::RTC_MODE_ON);
            break;
        case kAecOnDeviceSide:
            audio_service_.EnableDeviceAec(true);
            display->ShowNotification(Lang::Strings::RTC_MODE_ON);
            break;
        }

        // If the AEC mode is changed, close the audio channel
        if (protocol_ && protocol_->IsAudioChannelOpened()) {
            protocol_->CloseAudioChannel();
        }
    });
}

void Application::PlaySound(const std::string_view& sound) {
    audio_service_.PlaySound(sound);
}



/**
 * @brief 发送传感器事件
 * 
 * @param message 文本信息
 */
void Application::SendSensorEvent(const std::string& message) {
    Schedule([this, message]() {
        // 初始化NVS
        nvs_handle_t nvs_handle;
        esp_err_t err = nvs_open("shake_sensor", NVS_READWRITE, &nvs_handle);
        
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(err));
            return;
        }
        
        // 获取当前日期
        time_t now = 0;
        struct tm timeinfo = {0};
        char current_date[11] = {0}; // YYYY-MM-DD
        
        time(&now);
        localtime_r(&now, &timeinfo);
        strftime(current_date, sizeof(current_date), "%Y-%m-%d", &timeinfo);
        
        ESP_LOGI(TAG, "Current date: %s", current_date);
        
        // 从NVS读取上次摇晃的日期
        char last_shake_date[11] = {0};
        size_t required_size = 0;
        
        // 先获取存储的字符串长度
        esp_err_t ret = nvs_get_str(nvs_handle, "last_shake_date", NULL, &required_size);
        
        if (ret == ESP_OK && required_size > 0) {
            // 读取存储的日期
            nvs_get_str(nvs_handle, "last_shake_date", last_shake_date, &required_size);
            ESP_LOGI(TAG, "Last shake date from NVS: %s", last_shake_date);
        }
        
        // 判断是否是当天首次摇晃
        bool is_first_shake = false;
        if (required_size == 0 || strcmp(last_shake_date, current_date) != 0) {
            // 首次摇晃：没有存储日期或日期不同
            ESP_LOGI(TAG, "今日首次摇晃");
            is_first_shake = true;
            
            // 更新存储的日期为今天
            nvs_set_str(nvs_handle, "last_shake_date", current_date);
            nvs_commit(nvs_handle);
        } else {
            // 非首次摇晃
            ESP_LOGI(TAG, "非首次摇晃");
            is_first_shake = false;
        }
        
        nvs_close(nvs_handle);
        

                
        if (is_first_shake) {
            std::string device_id = SystemInfo::GetMacAddress();
            std::string client_id = Board::GetInstance().GetUuid();
            
            /*
            // 构建包含首次标记的JSON
            cJSON* root = cJSON_CreateObject();
            cJSON_AddStringToObject(root, "type", "shake");
            cJSON_AddStringToObject(root, "device_id", device_id.c_str());
            cJSON_AddStringToObject(root, "client_id", client_id.c_str());
            
            cJSON* payload = cJSON_CreateObject();
            cJSON_AddStringToObject(payload, "message", message.c_str());
            cJSON_AddBoolToObject(payload, "is_first_shake", is_first_shake);
            cJSON_AddStringToObject(payload, "current_date", current_date);
            cJSON_AddItemToObject(root, "payload", payload);
            
            char* json_str = cJSON_PrintUnformatted(root);
            std::string json_text(json_str);
            */


            // 发送传感器事件到协议（无论协议是否已连接）
            if (!protocol_) {
                ESP_LOGE(TAG, "Protocol not initialized");
                return;
            }
            
            // 检查协议是否已连接
            if (!protocol_->IsConnected()) {
                ESP_LOGW(TAG, "Protocol not connected, attempting to connect...");
                
                if (protocol_->OpenAudioChannel()) {
                    ESP_LOGI(TAG, "Connected successfully, sending sensor event");
                    MakeHttpPostRequest("摇晃", 1, device_id.c_str());
                    // http_client_->PostMetric("摇晃", 1, device_id.c_str());
                } else {
                    ESP_LOGE(TAG, "Failed to connect to protocol, cannot send sensor event");
                }
            } else {
                // 协议已连接，直接发送
                ESP_LOGI(TAG, "Protocol already connected, sending sensor event directly");
                MakeHttpPostRequest("摇晃", 1, device_id.c_str());
                // http_client_->PostMetric("摇晃", 1, device_id.c_str());
            }
        }


    });
}

/**
 * @brief 发送 HTTP GET 请求
 * 
 * @param url GET请求URL（可选，使用默认值）
 * @return true 请求成功
 * @return false 请求失败
 */
bool Application::MakeHttpGetRequest(const std::string& url) {
    ESP_LOGI(TAG, "Making HTTP GET request to: %s", url.c_str());
    return MakeHttpRequest(url, "GET", "", "");
}

/**
 * @brief 发送 HTTP POST 请求
 * 
 * @param url 请求 URL
 * @param json_data JSON格式的请求体
 * @return true 请求成功
 * @return false 请求失败
 */
bool Application::MakeHttpPostRequest(const std::string& url, const std::string& json_data) {
    ESP_LOGI(TAG, "Making HTTP POST request to: %s", url.c_str());
    ESP_LOGI(TAG, "POST data: %s", json_data.c_str());
    return MakeHttpRequest(url, "POST", "application/json", json_data);
}

/**
 * @brief 发送 HTTP POST 请求到服务器（发送指标数据）
 * 
 * @param metric_name 指标名称
 * @param metric_value 指标值
 * @param user_id 用户ID
 * @param post_url POST请求URL（可选，使用默认值）
 * @return true 发送成功
 * @return false 发送失败
 */
bool Application::MakeHttpPostRequest(const std::string& metric_name,
                                     int metric_value,
                                     const std::string& user_id,
                                     const std::string& post_url) {
    bool success = false;
    
    ESP_LOGI(TAG, "=================== HTTP Post ===================");
    ESP_LOGI(TAG, "Sending data - Metric: %s, Value: %d, User: %s",
             metric_name.c_str(), metric_value, user_id.c_str());
    
    // 创建JSON数据
    cJSON *post_json = cJSON_CreateObject();
    if (!post_json) {
        ESP_LOGE(TAG, "Failed to create JSON object");
        return false;
    }
    
    // 添加JSON字段
    cJSON_AddStringToObject(post_json, "metric_name", metric_name.c_str());
    cJSON_AddNumberToObject(post_json, "metric_value", metric_value);
    cJSON_AddStringToObject(post_json, "user_id", user_id.c_str());
    
    // 生成JSON字符串
    char *post_data = cJSON_PrintUnformatted(post_json);
    if (post_data) {
        ESP_LOGI(TAG, "JSON data: %s", post_data);
        
        // 发送HTTP请求
        success = MakeHttpRequest(post_url, "POST", "application/json", post_data);
        
        if (success) {
            ESP_LOGI(TAG, "HTTP POST request sent successfully");
        } else {
            ESP_LOGE(TAG, "Failed to send HTTP POST request");
        }
        
        // 释放内存
        free(post_data);
    } else {
        ESP_LOGE(TAG, "Failed to generate JSON string");
    }
    
    // 清理JSON对象
    cJSON_Delete(post_json);
    
    ESP_LOGI(TAG, "=================== HTTP Post End ===============");
    return success;
}

/**
 * @brief HTTP 事件处理器
 */
esp_err_t Application::HttpEventHandler(esp_http_client_event_t *evt) {
    Application *app = static_cast<Application*>(evt->user_data);
    if (!app) {
        return ESP_FAIL;
    }
    
    switch(evt->event_id) {
        case HTTP_EVENT_ON_DATA:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            if (evt->data) {
                // 将响应数据追加到 current_http_response
                app->current_http_response.append((char*)evt->data, evt->data_len);
            }
            break;
        case HTTP_EVENT_ERROR:
            ESP_LOGE(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_FINISH");
            break;
        default:
            // 其他事件不处理
            break;
    }
    return ESP_OK;
}

/**
 * @brief 静态HTTP事件处理器
 */
static esp_err_t _http_event_handler(esp_http_client_event_t *evt) {
    if (evt->user_data) {
        Application *app = static_cast<Application*>(evt->user_data);
        return app->HttpEventHandler(evt);
    }
    return ESP_OK;
}

/**
 * @brief 统一的 HTTP 请求方法
 * 
 * @param url 请求 URL
 * @param method 请求方法（"GET"、"POST"、"PUT"、"DELETE"）
 * @param content_type 请求内容类型
 * @param body 请求体内容（POST/PUT时使用）
 * @return true 请求成功（HTTP 2xx状态码）
 * @return false 请求失败
 */
bool Application::MakeHttpRequest(const std::string& url, const std::string& method,
                                 const std::string& content_type, const std::string& body) {
    bool success = false;
    std::string response_data;
    
    // 配置结构体
    esp_http_client_config_t config = {};
    config.url = url.c_str();
    config.method = HTTP_METHOD_GET; // 默认，下面会根据方法调整
    config.timeout_ms = 10000;
    config.disable_auto_redirect = false;
    config.buffer_size = 4096;
    config.buffer_size_tx = 2048;
    config.event_handler = _http_event_handler;
    config.user_data = static_cast<void*>(this);
    
    // 根据方法设置HTTP方法
    if (method == "POST") {
        config.method = HTTP_METHOD_POST;
    } else if (method == "PUT") {
        config.method = HTTP_METHOD_PUT;
    } else if (method == "DELETE") {
        config.method = HTTP_METHOD_DELETE;
    } else {
        config.method = HTTP_METHOD_GET;
    }
    
    // 初始化客户端
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        ESP_LOGE(TAG, "Failed to initialize HTTP client for URL: %s", url.c_str());
        return false;
    }
    
    // 设置请求头
    if (!content_type.empty()) {
        esp_http_client_set_header(client, "Content-Type", content_type.c_str());
    }
    
    // 添加必要的请求头
    esp_http_client_set_header(client, "User-Agent", "ESP32-HTTP-Client");
    esp_http_client_set_header(client, "Accept", "*/*");
    esp_http_client_set_header(client, "Connection", "close");
    
    // 对于POST/PUT请求，设置请求体
    if ((method == "POST" || method == "PUT") && !body.empty()) {
        esp_http_client_set_post_field(client, body.c_str(), body.length());
    }
    
    // 清空当前响应
    current_http_response.clear();
    
    // 执行请求
    esp_err_t err = esp_http_client_perform(client);
    
    if (err == ESP_OK) {
        int status_code = esp_http_client_get_status_code(client);
        ESP_LOGI(TAG, "HTTP Status Code: %d", status_code);
        
        // 判断是否成功 (2xx 状态码表示成功)
        if (status_code >= 200 && status_code < 300) {
            success = true;
        }
        
        // 获取响应长度
        int content_length = esp_http_client_get_content_length(client);
        ESP_LOGI(TAG, "Content length: %d", content_length);
        
        // 如果事件处理器没有收集到数据，尝试直接读取
        if (current_http_response.empty() && content_length > 0) {
            char *buffer = (char *)malloc(content_length + 1);
            if (buffer) {
                int read_len = esp_http_client_read(client, buffer, content_length);
                if (read_len > 0) {
                    buffer[read_len] = '\0';
                    current_http_response.assign(buffer, read_len);
                }
                free(buffer);
            }
        }
        
        // 处理响应数据
        if (!current_http_response.empty()) {
            response_data = current_http_response;
            ProcessHttpResponse(response_data);
        } else {
            ESP_LOGW(TAG, "No response body received");
        }
        
        if (!success) {
            ESP_LOGE(TAG, "HTTP request failed with status: %d", status_code);
        }
    } else {
        ESP_LOGE(TAG, "HTTP %s request failed: %s", method.c_str(), esp_err_to_name(err));
        
        // 获取错误详情
        int esp_tls_last_error = esp_http_client_get_errno(client);
        if (esp_tls_last_error != 0) {
            ESP_LOGE(TAG, "HTTP client error: %s", strerror(esp_tls_last_error));
        }
    }
    
    esp_http_client_cleanup(client);
    return success;
}

/**
 * @brief 处理HTTP响应数据
 * 
 * @param response_data 响应数据
 */
void Application::ProcessHttpResponse(const std::string& response_data) {
    if (response_data.empty()) {
        return;
    }
    
    ESP_LOGI(TAG, "HTTP Response (length: %zu bytes):", response_data.length());
    
    // 安全打印响应内容
    int print_len = response_data.length();
    if (print_len > 1024) {
        print_len = 1024;
    }
    
    // 打印响应前1024个字符
    for (int i = 0; i < print_len; i++) {
        if (i % 160 == 0 && i > 0) {
            printf("\n");
        }
        char c = response_data[i];
        if (c >= 32 && c <= 126) {
            printf("%c", c);
        } else {
            printf(".");
        }
    }
    printf("\n");
    
    // 尝试解析JSON响应
    cJSON *json = cJSON_Parse(response_data.c_str());
    if (json) {
        // 提取message字段
        cJSON *message = cJSON_GetObjectItem(json, "message");
        if (cJSON_IsString(message) && message->valuestring) {
            ESP_LOGI(TAG, "Response message: %s", message->valuestring);
        }
        
        // 检查是否有current_date字段并设置系统时间
        bool time_set = false;
        cJSON *current_date = cJSON_GetObjectItem(json, "current_date");
        if (cJSON_IsString(current_date) && current_date->valuestring) {
            ESP_LOGI(TAG, "Found current_date field: %s", current_date->valuestring);
            if (SetSystemTimeFromString(current_date->valuestring)) {
                ESP_LOGI(TAG, "System time set successfully from server");
                time_set = true;
            } else {
                ESP_LOGE(TAG, "Failed to set system time from server");
            }
        }
        
        // 提取data字段（现在是一个数组）
        cJSON *data = cJSON_GetObjectItem(json, "data");
        if (data && cJSON_IsArray(data)) {
            ESP_LOGI(TAG, "Response data array (size: %d):", cJSON_GetArraySize(data));
            
            // 清空之前的容量编码列表
            capacity_encodings_.clear();
            
            // 遍历数组中的每个元素
            int array_size = cJSON_GetArraySize(data);
            for (int i = 0; i < array_size; i++) {
                cJSON *item = cJSON_GetArrayItem(data, i);
                if (item && cJSON_IsObject(item)) {
                    ESP_LOGI(TAG, "  Item %d:", i + 1);
                    
                    // 获取各个字段
                    cJSON *capacity_encoding = cJSON_GetObjectItem(item, "capacity_encoding");
                    cJSON *capacity_name = cJSON_GetObjectItem(item, "capacity_name");
                    cJSON *hint_text = cJSON_GetObjectItem(item, "hint_text");
                    cJSON *operation_date = cJSON_GetObjectItem(item, "operation_date");
                    cJSON *user_id = cJSON_GetObjectItem(item, "user_id");
                    
                    if (cJSON_IsString(capacity_encoding) && capacity_encoding->valuestring) {
                        std::string encoding_str = capacity_encoding->valuestring;
                        ESP_LOGI(TAG, "    capacity_encoding: %s", encoding_str.c_str());
                        
                        // 将capacity_encoding保存到全局集合中
                        capacity_encodings_.insert(encoding_str);
                    }
                    
                    if (cJSON_IsString(capacity_name) && capacity_name->valuestring) {
                        ESP_LOGI(TAG, "    capacity_name: %s", capacity_name->valuestring);
                    }
                    
                    if (cJSON_IsString(hint_text) && hint_text->valuestring) {
                        ESP_LOGI(TAG, "    hint_text: %s", hint_text->valuestring);
                    } else if (cJSON_IsNull(hint_text)) {
                        ESP_LOGI(TAG, "    hint_text: null");
                    }
                    
                    if (cJSON_IsNumber(operation_date)) {
                        ESP_LOGI(TAG, "    operation_date: %d", operation_date->valueint);
                    }
                    
                    if (cJSON_IsString(user_id) && user_id->valuestring) {
                        ESP_LOGI(TAG, "    user_id: %s", user_id->valuestring);
                    }
                    
                    // // 检查是否是摇晃数据
                    // if (capacity_encoding && cJSON_IsString(capacity_encoding) && 
                    //     strcmp(capacity_encoding->valuestring, "shake") == 0) {
                    //     ESP_LOGI(TAG, "    [检测到摇晃记录]");
                        
                    //     // 如果有操作日期信息，可以更新本地存储
                    //     if (operation_date && cJSON_IsNumber(operation_date) && 
                    //         operation_date->valueint > 0) {
                    //         ESP_LOGI(TAG, "    今日已摇晃: %d次", operation_date->valueint);
                    //     }
                    // }
                }
            }

            /*// 检查是否存在"shake"类型的编码
            if (Application::GetInstance().HasCapacityEncoding("shake")) {
                ESP_LOGI("OtherClass", "摇晃功能已启用");
            }

            // 检查是否存在"stroke"类型的编码
            if (Application::GetInstance().HasCapacityEncoding("stroke")) {
                ESP_LOGI("OtherClass", "抚摸功能已启用");
            }

            // 获取所有编码
            const auto& encodings = Application::GetInstance().GetCapacityEncodings();
            for (const auto& encoding : encodings) {
                ESP_LOGI("OtherClass", "可用的编码: %s", encoding.c_str());
            }*/
            
            // 打印所有保存的capacity_encoding值
            ESP_LOGI(TAG, "Saved capacity encodings (%zu total):", capacity_encodings_.size());
            for (const auto& encoding : capacity_encodings_) {
                ESP_LOGI(TAG, "  - %s", encoding.c_str());
            }
        } else if (data && cJSON_IsObject(data)) {
            // 如果data是对象而不是数组，保留原来的处理逻辑
            ESP_LOGI(TAG, "Response data fields:");
            
            // 遍历所有数据字段
            cJSON *child = data->child;
            while (child) {
                if (cJSON_IsString(child) && child->valuestring) {
                    ESP_LOGI(TAG, "  %*s: %s", 20, child->string, child->valuestring);
                    
                    // 检查data对象中是否也有current_date字段
                    if (strcmp(child->string, "current_date") == 0 && !time_set) {
                        ESP_LOGI(TAG, "Found current_date in data field: %s", child->valuestring);
                        if (SetSystemTimeFromString(child->valuestring)) {
                            ESP_LOGI(TAG, "System time set successfully from server data field");
                        } else {
                            ESP_LOGE(TAG, "Failed to set system time from server data field");
                        }
                    }
                } else if (cJSON_IsNumber(child)) {
                    ESP_LOGI(TAG, "  %*s: %d", 20, child->string, child->valueint);
                } else if (cJSON_IsBool(child)) {
                    ESP_LOGI(TAG, "  %*s: %s", 20, child->string, cJSON_IsTrue(child) ? "true" : "false");
                }
                child = child->next;
            }
        } else if (data) {
            ESP_LOGW(TAG, "Data field is neither array nor object");
        }
        cJSON_Delete(json);
    } else {
        ESP_LOGI(TAG, "Response is not valid JSON or parse failed");
    }
}

/**
 * @brief 检查是否存在特定的capacity_encoding
 * 
 * @param encoding 要检查的编码字符串
 * @return true 存在
 * @return false 不存在
 */
bool Application::HasCapacityEncoding(const std::string& encoding) const {
    return capacity_encodings_.find(encoding) != capacity_encodings_.end();
}

/**
 * @brief 设置系统时间
 * 
 * @param time_str 时间字符串，格式为"YYYY-MM-DD HH:MM:SS"
 * @return true 设置成功
 * @return false 设置失败
 */
bool Application::SetSystemTimeFromString(const char* time_str) {
    struct tm tm_time;
    memset(&tm_time, 0, sizeof(tm_time));
    
    // 使用 sscanf 解析时间字符串
    int year, month, day, hour, minute, second;
    
    if (sscanf(time_str, "%d-%d-%d %d:%d:%d", 
               &year, &month, &day, &hour, &minute, &second) != 6) {
        ESP_LOGE(TAG, "Invalid time format: %s", time_str);
        return false;
    }
    
    // 设置 tm 结构体
    tm_time.tm_year = year - 1900;  // 年份从1900开始
    tm_time.tm_mon = month - 1;     // 月份 0-11
    tm_time.tm_mday = day;
    tm_time.tm_hour = hour;
    tm_time.tm_min = minute;
    tm_time.tm_sec = second;
    tm_time.tm_isdst = -1;  // 自动判断夏令时
    
    // 转换为 time_t
    time_t t = mktime(&tm_time);
    if (t == -1) {
        ESP_LOGE(TAG, "Failed to convert time");
        return false;
    }
    
    // 设置系统时间
    struct timeval tv;
    tv.tv_sec = t;
    tv.tv_usec = 0;
    
    if (settimeofday(&tv, NULL) != 0) {
        ESP_LOGE(TAG, "Failed to set system time");
        return false;
    }
    
    ESP_LOGI(TAG, "System time set to: %s", time_str);
    
    // 打印当前时间以验证
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    
    char time_buffer[32];
    strftime(time_buffer, sizeof(time_buffer), "%Y-%m-%d %H:%M:%S", &timeinfo);
    ESP_LOGI(TAG, "Current system time is: %s", time_buffer);
    
    return true;
}

