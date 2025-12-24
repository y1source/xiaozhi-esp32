#ifndef BLUETOOTH_CONFIG_H
#define BLUETOOTH_CONFIG_H

#include <string>
#include <functional>

class BluetoothConfig {
public:
    using ConfigCallback = std::function<void(const std::string& ssid, const std::string& password)>;
    
    static BluetoothConfig& GetInstance();
    
    bool Initialize();
    void StartProvisioning();
    void StopProvisioning();
    bool IsProvisioning() const;
    
    void SetConfigCallback(ConfigCallback callback);
    
private:
    BluetoothConfig();
    ~BluetoothConfig();
    
    ConfigCallback config_callback_;
    bool is_provisioning_;
    
    // 蓝牙服务相关
    struct BluetoothData;
    std::unique_ptr<BluetoothData> bt_data_;
};

#endif // BLUETOOTH_CONFIG_H