# web

Web 控制台的前端页面：`index.html`、`app.js`、`style.css`，没有构建步骤。

由 [`teleop_app`](../README.md) 的 `teleop_app.console` 提供服务，页面里启停模块、看状态和日志、选手柄、导出诊断包。

```bash
./scripts/start.sh console --open                       # 或双击仓库根目录的 ocs2_control控制台
./scripts/start.sh console start|stop|restart|status             # 后台启动 / 停止 / 重启 / 查看，默认端口 8765
./scripts/start.sh console --open --host 0.0.0.0        # 让其他电脑访问
```

没有登录：默认只监听本机；加 `--host 0.0.0.0` 后，同一网络里能访问端口的人都能控制机器人。
