#ifndef BLE_WIFI_INTEGRATION_H
#define BLE_WIFI_INTEGRATION_H

#include <string>
#include <vector>
#include <utility>  // for std::pair

namespace BleWifiIntegration {

/**
 * @brief 启动蓝牙WiFi配网功能
 * @return true 启动成功, false 启动失败
 */
bool StartBleWifiConfig();

/**
 * @brief 停止蓝牙WiFi配网功能
 */
void StopBleWifiConfig();

/**
 * @brief 检查蓝牙配网是否活跃
 * @return true 活跃, false 不活跃
 */
bool IsBleWifiConfigActive();

/**
 * @brief 获取发现的相同设备列表
 * @return 设备列表，每个元素是pair<MAC地址, RSSI>
 */
std::vector<std::pair<std::string, int>> GetDiscoveredDevices();

/**
 * @brief 手动触发设备扫描
 */
void TriggerDeviceScan();

} // namespace BleWifiIntegration

#endif // BLE_WIFI_INTEGRATION_H