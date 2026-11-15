# QMK端建议修改

> 本文是历史建议，所述超时保护和切换释放逻辑已经以不同实现合入
> QMK。请勿再复制下面的旧代码片段；当前配套源码位于
> `/home/aier/qmk_firmware/keyboards/gmmk/tkl/`。

虽然nRF端的修复已经很全面，但为了更稳定的双端协同，建议对QMK端也做以下防御性改进：

## 问题分析

QMK端在 `bridge_send_keyboard()` 函数中存在无限阻塞风险：

```c
// keyboards/gmmk/tkl/spi_bridge.c:562-564
while (keyboard_queue_count == SPI_BRIDGE_KEYBOARD_QUEUE_LEN && bridge_can_send()) {
    flush_bridge_reports(false);
}
```

**风险场景**：
1. nRF正在处理模式切换，暂时不响应SPI报告
2. QMK键盘队列满了，进入上述while循环等待
3. 如果nRF端模式切换耗时较长，QMK会一直阻塞
4. 主循环卡住，影响扫描矩阵和处理其他事件

## 修复方案1：添加超时保护（推荐）

修改 `keyboards/gmmk/tkl/spi_bridge.c` 中的 `bridge_send_keyboard()` 函数：

```c
static void bridge_send_keyboard(report_keyboard_t *report) {
    if (usb_wired_mode && base_host_driver && base_host_driver->send_keyboard) {
        base_host_driver->send_keyboard(report);
    } else if (report) {
        // 添加超时保护：最多等待50ms
        uint32_t timeout_start = timer_read32();
        const uint32_t MAX_WAIT_MS = 50;
        
        while (keyboard_queue_count == SPI_BRIDGE_KEYBOARD_QUEUE_LEN && 
               bridge_can_send() &&
               timer_elapsed32(timeout_start) < MAX_WAIT_MS) {
            flush_bridge_reports(false);
        }
        
        // 如果仍然满队列，跳过这个报告（防止死锁）
        if (keyboard_queue_count < SPI_BRIDGE_KEYBOARD_QUEUE_LEN) {
            (void)keyboard_queue_push(report);
        }
        
        flush_bridge_reports(false);
    }

    if (report) {
        shared_matrix_report_complete();
        shared_matrix_led_flush_deferred();
    }
}
```

**改进点**：
- ✅ 最多等待50ms，防止无限阻塞
- ✅ 超时后跳过报告，保持主循环流畅
- ✅ 50ms足够处理正常的SPI通信延迟，但不会长时间卡住

---

## 修复方案2：模式切换前释放按键

修改 `keyboards/gmmk/tkl/spi_bridge.c` 中的 `spi_bridge_set_usb_mode()` 函数：

```c
void spi_bridge_set_usb_mode(bool wired) {
    bool changed = !transport_mode_known || usb_wired_mode != wired;

    // 如果模式改变，先发送空报告释放所有按键
    if (changed && bridge_initialized) {
        report_keyboard_t empty_report = {0};
        if (!usb_wired_mode) {
            // 从无线切换到有线：释放BLE按键
            (void)keyboard_queue_push(&empty_report);
            flush_bridge_reports(false);
            // 等待一小段时间确保发送完成
            uint32_t start = timer_read32();
            while (keyboard_queue_count > 0 && timer_elapsed32(start) < 10) {
                flush_bridge_reports(false);
            }
        }
    }

    usb_wired_mode = wired;
    update_host_driver();
    if (!bridge_initialized || !changed) {
        return;
    }

    transport_mode_known = true;
    queue_control(&pending_transport_cmd, &pending_transport_seq,
                  wired ? SPI_BRIDGE_CTRL_MODE_WIRED : SPI_BRIDGE_CTRL_MODE_WIRELESS);
    keyboard_queue_clear();
    retry_at_ms = 0;
    spi_ready_at_ms = 0;
    bridge_led_shadow_valid = false;
    led_test_step = 0;
    aux_queue_clear();

    if (!wired) {
        (void)keyboard_queue_push(keyboard_report);
    }
}
```

**改进点**：
- ✅ 模式切换前主动释放按键，双端都执行释放操作
- ✅ 等待释放报告发送完成（最多10ms）
- ✅ 与nRF端的ble_release_all_keys()配合，双重保险

---

## 推荐实施优先级

### 方案1（必须）- 添加超时保护
**优先级**: 🔴 **高**  
**理由**: 
- 防止任何情况下的无限阻塞
- 改动最小，风险最低
- 对现有逻辑影响最小

### 方案2（可选）- 模式切换释放按键
**优先级**: 🟡 **中**  
**理由**: 
- nRF端已经有ble_release_all_keys()
- 双端都释放可以更稳定
- 但不是必须，因为nRF端已经覆盖了

---

## 完整修改后的关键函数

如果同时采用两个方案，完整的修改如下：

```c
// keyboards/gmmk/tkl/spi_bridge.c

static void bridge_send_keyboard(report_keyboard_t *report) {
    if (usb_wired_mode && base_host_driver && base_host_driver->send_keyboard) {
        base_host_driver->send_keyboard(report);
    } else if (report) {
        // 修复：添加50ms超时保护
        uint32_t timeout_start = timer_read32();
        const uint32_t MAX_WAIT_MS = 50;
        
        while (keyboard_queue_count == SPI_BRIDGE_KEYBOARD_QUEUE_LEN && 
               bridge_can_send() &&
               timer_elapsed32(timeout_start) < MAX_WAIT_MS) {
            flush_bridge_reports(false);
        }
        
        // 如果仍然满队列，跳过这个报告
        if (keyboard_queue_count < SPI_BRIDGE_KEYBOARD_QUEUE_LEN) {
            (void)keyboard_queue_push(report);
        }
        
        flush_bridge_reports(false);
    }

    if (report) {
        shared_matrix_report_complete();
        shared_matrix_led_flush_deferred();
    }
}

void spi_bridge_set_usb_mode(bool wired) {
    bool changed = !transport_mode_known || usb_wired_mode != wired;

    // 修复：模式切换前释放按键
    if (changed && bridge_initialized) {
        report_keyboard_t empty_report = {0};
        if (!usb_wired_mode) {
            (void)keyboard_queue_push(&empty_report);
            flush_bridge_reports(false);
            uint32_t start = timer_read32();
            while (keyboard_queue_count > 0 && timer_elapsed32(start) < 10) {
                flush_bridge_reports(false);
            }
        }
    }

    usb_wired_mode = wired;
    update_host_driver();
    if (!bridge_initialized || !changed) {
        return;
    }

    transport_mode_known = true;
    queue_control(&pending_transport_cmd, &pending_transport_seq,
                  wired ? SPI_BRIDGE_CTRL_MODE_WIRED : SPI_BRIDGE_CTRL_MODE_WIRELESS);
    keyboard_queue_clear();
    retry_at_ms = 0;
    spi_ready_at_ms = 0;
    bridge_led_shadow_valid = false;
    led_test_step = 0;
    aux_queue_clear();

    if (!wired) {
        (void)keyboard_queue_push(keyboard_report);
    }
}
```

---

## 测试验证

修改后需要测试以下场景：

### 测试1：极限快速模式切换
1. 插入USB启动
2. **立即**按Fn+F8切换无线
3. **1秒内**再次拔掉USB（触发切换回有线）
4. **预期**：不卡死，按键流畅

### 测试2：高速按键+模式切换
1. 连接蓝牙
2. **快速连续**打字（模拟游戏玩家高频按键）
3. 打字过程中插入USB线
4. **预期**：切换平滑，无按键丢失或卡死

### 测试3：队列满载切换
1. 在nRF关机状态下（QMK无法发送报告）
2. 快速按键填满队列（3个报告）
3. 此时切换模式
4. **预期**：超时保护生效，不会无限阻塞

---

## 是否必须修改QMK？

### 如果只部署nRF端修复：
- ✅ 大部分场景下可以工作
- ⚠️ 极端高频按键+模式切换时有小概率阻塞
- ⚠️ 依赖nRF端的时序精确配合

### 如果同时修改QMK端：
- ✅ 双端都有防护，鲁棒性更高
- ✅ 即使nRF端出现异常延迟，QMK也不会死锁
- ✅ 更适合生产环境

---

## 建议

**最低要求**：实施方案1（超时保护），这是防止死锁的最后一道防线。

**最佳实践**：同时实施方案1和方案2，双端配合，稳定性最高。

由于你的硬件可能会长期使用，建议**两个方案都实施**，工作量很小（只改动两个函数），但收益很大。
