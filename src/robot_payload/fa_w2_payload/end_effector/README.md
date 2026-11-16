# end_effector

W2 的灵巧手（LinkerHand L6，XHand1）。依赖 [`fa_w2_payload`](../common/README.md) 的配置框架。

```bash
./scripts/setup.sh hand     # 弹窗：选 usb / 485 接线，用“开合测试”核对左右手
```

USB 手只按适配器序列号识别（`/dev/serial/by-id/*`），不接受 `ttyUSBn` 和 `by-path`。手指测试只在你点击并确认后才执行。确认记录绑定当前接线和串口，变化后需要重新确认。

## 组成

| 文件 | 内容 |
|---|---|
| `hand.py` | 串口和 485 探测、开合测试；每次请求在独立子进程里运行，不占用控制器 |
| `hand_setup.py` | 手配置会话、窗口卡片、`setup.sh hand` 入口 |
