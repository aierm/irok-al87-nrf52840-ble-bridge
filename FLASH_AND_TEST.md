# 固件刷入和测试指南

> 注意：本文是早期调试记录，其中的默认无线启动、同步模式切换、
> 固件大小和部分日志已经过时。当前实现、编译命令和电源策略以
> [README.md](README.md) 为准：上电默认有线待机且不广播，`Fn+F8`/`Fn+F7`
> 分别显式选择蓝牙/有线；新 USB 主机选择有线，拔出 USB 不自动开启蓝牙。

## 📦 编译成功

固件已成功编译：
- **固件位置**: `build_uf2/zephyr/zephyr.uf2`
- **固件大小**: 538 KB
- **Flash使用**: 275,172 bytes (33.93%)
- **RAM使用**: 66,888 bytes (25.52%)

## 🔧 刷入固件

### 方法1：UF2引导加载程序（推荐）

1. **进入引导模式**：
   - 按住nRF52840的BOOT按钮
   - 插入USB线
   - 松开BOOT按钮
   - 系统会识别为USB存储设备（名称类似 "NRF52BOOT"）

2. **拷贝固件**：
   ```bash
   # 自动挂载后
   cp build_uf2/zephyr/zephyr.uf2 /media/$USER/NRF52BOOT/
   
   # 或者手动拷贝到出现的USB驱动器
   ```

3. **等待刷入完成**：
   - LED会闪烁
   - USB驱动器会自动弹出
   - 设备会自动重启进入新固件

### 方法2：JLink/SWD（需要调试器）

```bash
# 使用nrfjprog（如果有JLink）
nrfjprog --program build_uf2/zephyr/zephyr.hex --chiperase -f nrf52 --reset
```

## 🧪 测试计划

### 测试1：启动和初始化 ✓

**步骤**：
1. 刷入固件后上电
2. 观察串口输出（115200波特率）

**预期输出**：
```
SPI -> BLE HIDS keyboard bridge
Bluetooth initialized
Power manager started
Battery: XXXX mV, XX%
Transport mode: wired standby; waiting for QMK policy
```

**验证点**：
- ✓ 蓝牙初始化成功
- ✓ 电量检测正常显示
- ✓ 默认保持有线待机，蓝牙不广播

---

### 测试2：有线+蓝牙切换（修复问题1）⚠️

**场景A：先USB后蓝牙**
1. 插入USB线启动（有线模式）
2. 串口应显示 `Transport mode: wired`
3. 按 Fn+F8 开启蓝牙
4. 串口显示 `Transport mode: wireless`
5. 连接蓝牙设备
6. 测试按键输入

**场景B：电池待机，手动开蓝牙后插USB（关键测试）**
1. 拔掉USB，使用电池冷启动
2. 按 Fn+F8 显式开启蓝牙（拔出 USB 不会自动开启）
3. 连接蓝牙设备并测试按键
4. **保持蓝牙连接**，插入USB线
5. ⚠️ **关键验证**：系统应平滑切换，不卡死
6. 快速连续按键测试流畅性

**场景C：同时启动（极限测试）**
1. 插入USB启动
2. 立即按 Fn+F8
3. 快速按几个按键
4. ⚠️ **关键验证**：不应卡死，模式切换缓冲2秒

**预期行为**：
- ✓ 模式切换前先释放所有按键
- ✓ 2秒内重复切换命令被忽略
- ✓ 无死锁或阻塞
- ✓ 串口可能显示 "Mode switch already in progress"

**失败症状（旧版）**：
- ✗ 输入几个按键后完全卡死
- ✗ 串口无输出
- ✗ 按键无响应

---

### 测试3：电量检测（修复问题2）📊

**准备**：
- 充满电的锂电池
- 万用表（可选，用于校准）
- 蓝牙连接的设备（查看电量显示）

**测试A：初始读数稳定性**
1. 上电后等待2分钟
2. 观察串口电量输出（每30秒一次）
3. 连续10次读数变化应 < 5%

**测试B：负载稳定性**
1. 连接蓝牙
2. 快速连续按键（高负载）
3. 观察电量不应突降
4. 停止按键后电量不应回升

**测试C：断连重连**
1. 记录当前显示电量：______%
2. 断开蓝牙连接
3. 等待1分钟
4. 重新连接蓝牙
5. 电量变化应 < 5%

**测试D：长期监测**
| 时间 | 串口mV | 串口% | 系统显示% | 万用表mV |
|------|--------|-------|-----------|----------|
| 0min | _____ | ___% | ___% | _____ |
| 30min| _____ | ___% | ___% | _____ |
| 60min| _____ | ___% | ___% | _____ |
| 120min| ____ | ___% | ___% | _____ |

**预期行为**：
- ✓ 充满电显示 95-100%
- ✓ 放电曲线平滑下降
- ✓ 单次变化 < 5%
- ✓ 断连重连电量稳定
- ✓ 到0%后还能用一段时间（安全余量）

**失败症状（旧版）**：
- ✗ 100% 快速跳到 20%
- ✗ 重连后电量大幅回升
- ✗ 到0%立即关机

---

### 测试4：省电策略（修复问题3）⏱️

**测试A：连接后空闲超时**
1. 连接蓝牙
2. 不按任何键
3. 30秒后：串口应显示进入idle模式
4. 2分钟后：串口应显示慢速广播

**测试B：无连接自动关机**
1. 按 Fn+F8 开启蓝牙
2. **不连接任何设备**
3. 等待10分钟
4. ⚠️ **关键验证**：设备应自动关机
5. 串口最后输出：`Auto power-off: no connection`

**测试C：SPI无活动关机**
1. 连接蓝牙
2. 拔掉键盘MCU电源（模拟键盘关机）
3. 等待30分钟
4. ⚠️ **关键验证**：nRF应自动关机
5. 串口最后输出：`SPI idle power-off: 30min no activity`

**测试D：手动关机**
1. 按住电源键1秒
2. 设备应立即关机
3. 串口输出：`Power button shutdown`

**预期时间线**：
```
T+0s    按键活动
T+30s   无活动 → Idle连接参数
T+2min  无活动 → 慢速广播
T+10min 无连接 → 自动关机
T+30min SPI无活动 → 自动关机
```

**失败症状（旧版）**：
- ✗ 10分钟后只打印消息不关机
- ✗ 长时间空闲但电量持续消耗
- ✗ 键盘MCU关机后nRF永不关机

---

## 🐛 调试技巧

### 串口监控

```bash
# 使用screen
screen /dev/ttyACM0 115200

# 或使用minicom
minicom -D /dev/ttyACM0 -b 115200

# 退出screen: Ctrl+A, K
```

### 关键日志标记

**正常启动**：
```
SPI -> BLE HIDS keyboard bridge
Bluetooth initialized
Battery: 4180 mV, 100%
Transport mode: wired standby; waiting for QMK policy
```

**模式切换**：
```
Transport mode: wireless
[用户按Fn+F8切换]
BLE release report sent
Transport mode: wired (Bluetooth idle)
```

**省电事件**：
```
Connection idle: switching to slow parameters
Slow advertising started
Auto power-off: no connection for 600000ms
Power button shutdown
```

**电量更新**（每30秒）：
```
Battery: 4050 mV, 90%
Battery: 4000 mV, 82%
Battery: 3950 mV, 75%
```

### 常见问题排查

**问题：编译失败 "No module named 'elftools'"**
```bash
/usr/bin/python3 -m pip install --user --break-system-packages pyelftools
```

**问题：找不到Zephyr SDK**
```bash
export ZEPHYR_SDK_INSTALL_DIR=/home/aier/ncs/toolchains/2ac5840438/opt/zephyr-sdk
```

**问题：设备不进入引导模式**
- 检查BOOT按钮是否正确连接
- 尝试长按5秒
- 或使用nrfjprog完全擦除后重试

**问题：电量显示为0%**
- 检查P0.31连接
- 检查分压电阻值（应为1MΩ+1MΩ）
- 检查电容（10-100nF）
- 万用表测量P0.31电压应为VBAT/2

---

## 📝 测试记录模板

### 我的测试结果

**测试日期**: ___________  
**固件版本**: 修复版 (2025-01-XX)

#### ✅ 测试1：启动和初始化
- [ ] 蓝牙初始化成功
- [ ] 电量显示正常
- [ ] 串口输出正常
- **备注**: _________________________________

#### ✅ 测试2：有线+蓝牙切换
- [ ] 场景A：先USB后蓝牙 ✓
- [ ] 场景B：先蓝牙后USB ✓
- [ ] 场景C：同时启动不卡死 ✓
- **备注**: _________________________________

#### ✅ 测试3：电量检测
- [ ] 初始读数稳定
- [ ] 负载不影响读数
- [ ] 断连重连电量稳定
- [ ] 放电曲线平滑
- **初始电量**: _____%
- **1小时后**: _____%
- **备注**: _________________________________

#### ✅ 测试4：省电策略
- [ ] 30秒进入idle
- [ ] 10分钟无连接关机 ✓
- [ ] 30分钟SPI无活动关机 ✓
- [ ] 手动关机正常
- **备注**: _________________________________

#### 总体评价
- **改进最明显的问题**: _________________________________
- **仍需改进的地方**: _________________________________
- **建议**: _________________________________

---

## 🔄 如需回退

如果新固件有问题，可以刷回旧版：
```bash
# 找到备份的旧固件
cp backup/old_firmware.uf2 /media/$USER/NRF52BOOT/
```

## 📞 技术支持

遇到问题请提供：
1. 串口完整日志
2. 测试场景描述
3. 预期行为 vs 实际行为
4. 电池电压（万用表测量）

---

## ⚙️ 可选调整

如需调整参数，修改后重新编译：

### 调整关机时间
编辑 `src/main.c`:
```c
#define SPI_AUTO_POWEROFF_DELAY_MS 600000U    // 10分钟 → 改成你需要的
#define SPI_LINK_IDLE_POWEROFF_MS 1800000U    // 30分钟 → 改成你需要的
```

### 调整电量采样
编辑 `src/power_manager.c`:
```c
#define BATTERY_SAMPLE_INTERVAL K_SECONDS(30)  // 30秒 → 改成你需要的
#define BATTERY_FILTER_DOWN_WEIGHT 3           // 放电下降滤波：(old*3+new)/4
#define BATTERY_FILTER_UP_WEIGHT 15            // 负载回升滤波：(old*15+new)/16
```

### 重新编译
```bash
cd /home/aier/ncs/v3.2.3
export ZEPHYR_SDK_INSTALL_DIR=/home/aier/ncs/toolchains/2ac5840438/opt/zephyr-sdk
west build --no-sysbuild -d nrf/samples/bluetooth/spi_hids_keyboard_bridge/build_uf2 \
  -b promicro_nrf52840/nrf52840/uf2 \
  nrf/samples/bluetooth/spi_hids_keyboard_bridge \
  -- -DDTC_OVERLAY_FILE=boards/promicro_nrf52840_nrf52840.overlay
```

---

**祝测试顺利！🚀**
