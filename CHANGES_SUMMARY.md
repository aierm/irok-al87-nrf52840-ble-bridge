# 蓝牙键盘桥接修复汇总

> 本文保留为阶段性修改记录，不代表最终状态机。当前行为以
> [README.md](README.md) 和源码为准；特别是蓝牙上电不再自动开启，
> 模式/配对/断连操作也不再在 SPI 接收路径中同步执行。

## 本次修复

### 有线与蓝牙同时启动

- 删除 nRF 启动后的 2 秒模式切换拒绝窗口。
- 修复 `MODE_WIRED` 被忽略却仍被记录和 ACK 的状态机错误。
- QMK 从有线切换到无线前，先向 USB 主机发送全释放报告。
- QMK SPI 键盘队列最多等待 10ms；超时后保留最新绝对按键状态，避免阻塞矩阵扫描。
- nRF 切到有线模式时继续执行 BLE 全释放、断开连接和停止广播。

### 电量检测

- SAADC acquisition time 改为 nRF52840 驱动支持的最大值 `40us`。
- `zephyr,oversampling = <3>`，即 `2^3 = 8` 倍过采样。
- 每 30 秒采样一次，每次连续读取 4 次，并对最近 3 轮结果取平均。
- ADC 失败时不再伪造 `3700mV` 电量；保留上一次有效 BAS 值并打印错误。
- 电压下降使用 1/4 新样本权重，负载移除后的电压回升使用 1/16 新样本权重。
- 更新了 1S 锂电池分段电压曲线；最终精度仍需按实测电池和负载校准。

### 低功耗与关机

- QMK 心跳不再计为用户活动；30 秒空闲后稳定降为每 250ms 一次 SPI 心跳。
- QMK 无线空闲 30 秒后把扫描循环限制到约 1kHz，5 分钟后暂停 LED Matrix，按键后恢复。
- BLE 连接空闲参数调整为 30-50ms、latency 4。
- 无 BLE 连接 10 分钟进入 System OFF；真实按键变化会延长计时，重复心跳不会。
- SPI 链路无活动 30 分钟进入 System OFF。
- 禁用应用未使用的 USB、I2C0、I2C1、PWM0、SPI2 和 802.15.4 设备节点。
- 关闭板级 USB CDC 串口后端并禁用 UART0；日志仍走 RTT。

## 关键参数

| 项目 | 当前值 |
| --- | --- |
| 活跃 SPI 心跳 | 8ms |
| 30 秒后空闲 SPI 心跳 | 250ms |
| BLE 连接空闲 | 30-50ms，latency 4 |
| LED 自动暂停 | 无线空闲 5 分钟 |
| 无 BLE 连接关机 | 10 分钟 |
| SPI 无活动关机 | 30 分钟 |
| 电量采样 | 30 秒一次，4 次 ADC 平均 |
| SAADC acquisition | 40us |
| SAADC oversampling | 8x (`<3>`) |

## 编译结果

- QMK: `/home/aier/qmk_firmware/gmmk_tkl_default.bin`
- nRF UF2: `build_uf2/zephyr/zephyr.uf2`

```bash
eval "$(/home/aier/anaconda3/bin/conda shell.bash hook)"
conda activate qmk

cd /home/aier/qmk_firmware
qmk compile -kb gmmk/tkl -km default

cd /home/aier/ncs/v3.2.3
export ZEPHYR_TOOLCHAIN_VARIANT=zephyr
export ZEPHYR_SDK_INSTALL_DIR=/home/aier/ncs/toolchains/2ac5840438/opt/zephyr-sdk
west build --no-sysbuild -p always \
  -d nrf/samples/bluetooth/spi_hids_keyboard_bridge/build_uf2 \
  -b promicro_nrf52840/nrf52840/uf2 \
  nrf/samples/bluetooth/spi_hids_keyboard_bridge \
  -- -DDTC_OVERLAY_FILE=boards/promicro_nrf52840_nrf52840.overlay
```

## 上板验证

1. USB 和蓝牙同时启动，连续输入、快速按键，并多次执行有线/无线切换。
2. 使用万用表同时测量 VBAT 和 P0.31；P0.31 应约等于 VBAT 的一半。
3. 观察 RTT 中 `Battery: xxxx mV, yy%`，确认电压与万用表差值稳定。
4. 无线连接后空闲 5 分钟，确认灯光暂停且首个按键能恢复。
5. 无 BLE 连接且不操作 10 分钟，确认 nRF 切断键盘电源并进入 System OFF。

## 电量校准说明

电压法只能估算电量，并会受负载、温度、内阻和电池保护板影响。若读取电压与万用表存在固定比例误差，先核对两只 1M 电阻实测值和 P0.31 电容焊接；若电压准确而百分比不符合实际续航，再调整 `src/power_manager.c` 的 `battery_curve[]`。

如果 1M/1M 配合当前电容仍有明显采样偏低，可把分压改为 470k/470k；静态分压电流会从约 2.1uA 增加到约 4.5uA，但 ADC 源阻抗更低。
