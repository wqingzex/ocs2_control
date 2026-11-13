"""Native Qt windows used by setup.sh. No web server or browser is involved."""
from __future__ import annotations

import fcntl
import os


def create_window(session):
    # Keep Qt optional when importing the CLI or running protocol/config tests.
    from concurrent.futures import ThreadPoolExecutor
    from PyQt5 import QtCore, QtGui, QtWidgets as Q

    titles = {'head': '头部', 'left_wrist': '左腕', 'right_wrist': '右腕'}

    class Window(Q.QWidget):
        def __init__(self):
            super().__init__()
            self.session = session
            self.state = session.state()
            self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='w2_setup')
            self.future = None
            self.dirty = False
            self.closing = False
            self.saved = False
            self.cards = {}
            self.setWindowTitle('W2 相机配置' if session.kind == 'cameras' else 'W2 左右手配置')
            self.resize(1160 if session.kind == 'cameras' else 860, 730)
            layout = Q.QVBoxLayout(self)
            heading = Q.QLabel(self.windowTitle())
            heading.setStyleSheet('font-size:22px;font-weight:600')
            layout.addWidget(heading)
            text = ('根据实时画面选择头部、左腕、右腕。依次遮挡左右腕镜头核对位置，并确认画面方向。'
                    if session.kind == 'cameras' else
                    '分别点击“开合测试”：手会慢速半握再张开，据此核对是机器人的左手还是右手。')
            hint = Q.QLabel(text + '\n左右均以机器人自身方向为准。确认保存前，原配置保持不变。')
            hint.setWordWrap(True)
            layout.addWidget(hint)
            self.message = Q.QLabel('正在读取设备…')
            self.message.setWordWrap(True)
            layout.addWidget(self.message)
            self.options = Q.QWidget()
            self.option_layout = Q.QHBoxLayout(self.options)
            self.option_layout.setContentsMargins(0, 0, 0, 0)
            layout.addWidget(self.options)
            scroll = Q.QScrollArea()
            scroll.setWidgetResizable(True)
            content = Q.QWidget()
            grid = Q.QGridLayout(content)
            scroll.setWidget(content)
            layout.addWidget(scroll, 1)
            if session.kind == 'cameras':
                self.build_cameras(grid)
            else:
                self.build_hands(grid)
            buttons = Q.QHBoxLayout()
            self.cancel_button = Q.QPushButton('取消，保留原配置')
            self.cancel_button.clicked.connect(self.close)
            self.swap_button = Q.QPushButton('交换左右腕' if session.kind == 'cameras' else '交换左右手串口')
            self.swap_button.clicked.connect(self.swap)
            self.apply_button = Q.QPushButton('应用配置并预览' if session.kind == 'cameras' else '应用配置')
            self.apply_button.clicked.connect(self.apply)
            self.save_button = Q.QPushButton('确认并保存')
            self.save_button.setDefault(True)
            self.save_button.clicked.connect(self.save)
            for button in (self.cancel_button, self.swap_button, self.apply_button):
                buttons.addWidget(button)
            buttons.addStretch(1)
            buttons.addWidget(self.save_button)
            layout.addLayout(buttons)
            self.timer = QtCore.QTimer(self)
            self.timer.timeout.connect(self.tick)
            self.timer.start(250)
            self.refresh()

        def combo(self, options, selected):
            combo = Q.QComboBox()
            for value, title in options:
                combo.addItem(title, value)
            index = combo.findData(selected)
            combo.setCurrentIndex(max(0, index))
            combo.currentIndexChanged.connect(self.mark_dirty)
            return combo

        def build_cameras(self, grid):
            for device in self.state['devices']:
                box = Q.QGroupBox(device['model'])
                body = Q.QVBoxLayout(box)
                identity = Q.QLabel('序列号：' + device['serial'] + '\n连接：' + (device['connection'] or '未知')
                                    + '  端口：' + (device['usb_port'] or '未知'))
                identity.setWordWrap(True)
                body.addWidget(identity)
                image = Q.QLabel('等待相机画面')
                image.setAlignment(QtCore.Qt.AlignCenter)
                image.setMinimumSize(280, 230)
                image.setStyleSheet('background:#111;color:#bbb;border-radius:5px')
                body.addWidget(image, 1)
                feedback = Q.QLabel('等待最新画面')
                feedback.setWordWrap(True)
                body.addWidget(feedback)
                form = Q.QFormLayout()
                role = self.combo([('', '不使用'), *titles.items()], device['role'])
                rotation = self.combo([(value, f'{value}°') for value in (0, 90, 180, 270)], device['rotation'])
                form.addRow('位置', role)
                form.addRow('画面旋转', rotation)
                body.addLayout(form)
                check = Q.QCheckBox('我已核对该画面的位置和方向')
                body.addWidget(check)
                self.cards[device['serial']] = {'box': box, 'image': image, 'feedback': feedback,
                                               'role': role, 'rotation': rotation, 'check': check, 'stamp': None}
                grid.addWidget(box, 0, device['index'])

        def build_hands(self, grid):
            draft = self.state['draft']
            self.hand = self.combo([('none', '不使用手'), ('linkerhand_l6', 'LinkerHand L6'), ('xhand1', 'XHand1')], draft['hand'])
            self.drive = self.combo([('usb', 'usb（485 转 USB 接电脑）'), ('485', '485（接机械臂接口）')], draft['drive'])
            self.device_ip = Q.QLineEdit(str(draft['device_ip']))
            self.device_ip.textEdited.connect(self.mark_dirty)
            self.ip_label = Q.QLabel('485 控制器 IP')
            for widget in (Q.QLabel('末端'), self.hand, Q.QLabel('接线'), self.drive, self.ip_label, self.device_ip):
                self.option_layout.addWidget(widget)
            ports = [(p['path'], p['path'] + ('（稳定路径）' if p['stable'] else '')) for p in self.state['ports']]
            if not ports:
                ports = [('', '未发现串口；容器需映射设备')]
            for index, side in enumerate(('left', 'right')):
                title = '左手' if side == 'left' else '右手'
                box = Q.QGroupBox(title)
                body = Q.QVBoxLayout(box)
                form = Q.QFormLayout()
                port = self.combo(ports, draft[side + '_port'])
                port_label = Q.QLabel('USB 串口')
                form.addRow(port_label, port)
                channel = self.combo([(value, str(value)) for value in (1, 2, 3)], draft[side + '_channel'])
                channel_label = Q.QLabel('485 通道')
                arm = Q.QLabel('485：机器人' + ('左臂（A）' if side == 'left' else '右臂（B）'))
                body.addWidget(arm)
                form.addRow(channel_label, channel)
                body.addLayout(form)
                actions = Q.QHBoxLayout()
                test = Q.QPushButton('开合测试')
                test.clicked.connect(lambda _=False, s=side: self.hand_action(s, 'test'))
                actions.addWidget(test)
                body.addLayout(actions)
                feedback = Q.QLabel('尚未测试')
                feedback.setWordWrap(True)
                body.addWidget(feedback)
                body.addStretch(1)
                check = Q.QCheckBox('我已核对该设备是机器人的' + title)
                body.addWidget(check)
                self.cards[side] = {'box': box, 'port': port, 'port_label': port_label, 'channel': channel,
                                    'channel_label': channel_label, 'arm': arm, 'test': test,
                                    'feedback': feedback, 'check': check}
                grid.addWidget(box, 0, index)

        def show_message(self, text, error=False):
            self.message.setText(text)
            self.message.setStyleSheet('color:#b42318' if error else 'color:#176b39')

        def mark_dirty(self, *_):
            self.dirty = True
            for card in self.cards.values():
                card['check'].setChecked(False)
            self.show_message('配置已修改，请先应用，再核对实物并重新确认。')
            self.refresh()

        def mapping(self):
            return {serial: {'role': card['role'].currentData(), 'rotation': card['rotation'].currentData()}
                    for serial, card in self.cards.items()}

        def draft(self):
            draft = {'hand': self.hand.currentData(), 'drive': self.drive.currentData(),
                     'device_ip': self.device_ip.text().strip()}
            for side, card in self.cards.items():
                draft[side + '_port'] = card['port'].currentData()
                draft[side + '_channel'] = card['channel'].currentData()
            return draft

        def submit(self, function, payload, success):
            if self.future or self.closing:
                return
            self.success = success
            self.future = self.executor.submit(function, payload)
            self.show_message('正在处理设备，请稍候…')
            self.refresh()

        def apply(self):
            payload = {'mapping': self.mapping()} if session.kind == 'cameras' else {'draft': self.draft()}
            if session.kind == 'cameras':
                conflicts = session.conflicts()
                if conflicts:
                    details = '\n'.join(f"PID {item['pid']}  {item['camera']}  {item['serial']}" for item in conflicts)
                    answer = Q.QMessageBox.question(self, '停止旧相机采集并重启配置',
                        '检测到以下旧相机进程：\n' + details +
                        '\n\n是否停止这些相机采集进程，并自动重启当前相机预览？只处理上述相机进程，不停止运控。',
                        Q.QMessageBox.Yes | Q.QMessageBox.No, Q.QMessageBox.No)
                    if answer != Q.QMessageBox.Yes:
                        self.show_message('未停止旧相机进程。请释放设备后点击“应用配置并预览”。', True)
                        return
                    payload['stop_old_processes'] = conflicts
            self.submit(session.update, payload, 'applied')

        def swap(self):
            if session.kind == 'cameras':
                for card in self.cards.values():
                    value = card['role'].currentData()
                    value = {'left_wrist': 'right_wrist', 'right_wrist': 'left_wrist'}.get(value, value)
                    card['role'].setCurrentIndex(card['role'].findData(value))
            elif self.drive.currentData() == 'usb':
                left, right = self.cards['left']['port'], self.cards['right']['port']
                first, second = left.currentData(), right.currentData()
                left.setCurrentIndex(left.findData(second))
                right.setCurrentIndex(right.findData(first))
            else:
                self.show_message('485 左右臂接口固定，请核对并调整实物接线。', True)
                return
            self.apply()

        def hand_action(self, side, operation):
            if self.dirty:
                self.show_message('请先应用配置，再做开合测试。', True)
                return
            payload = {'side': side, 'operation': operation, 'revision': self.state['revision']}
            if self.state['draft']['drive'] == '485':
                answer = Q.QMessageBox.question(self, '连接末端 485',
                    '将连接机械臂控制器的末端 485。请确认原运控已经停止；只操作手部。是否继续？',
                    Q.QMessageBox.Yes | Q.QMessageBox.No, Q.QMessageBox.No)
                if answer != Q.QMessageBox.Yes:
                    return
                payload['allow_controller_connection'] = True
            if operation == 'test':
                answer = Q.QMessageBox.question(self, '开合测试',
                    '所选的手将慢速半握再张开并回到原位。请确认手周围无遮挡，并观察是哪只手。是否执行？',
                    Q.QMessageBox.Yes | Q.QMessageBox.No, Q.QMessageBox.No)
                if answer != Q.QMessageBox.Yes:
                    return
                payload['allow_motion'] = True
            self.cards[side]['check'].setChecked(False)
            self.submit(session.action, payload, 'hand')

        def save(self):
            selected = [card['role'].currentData() if session.kind == 'cameras' else key
                        for key, card in self.cards.items() if card['check'].isChecked()]
            self.submit(session.save, {'revision': self.state['revision'], 'confirmed': selected}, 'saved')

        def tick(self):
            if self.future:
                if not self.future.done():
                    return
                future, self.future = self.future, None
                try:
                    self.state = future.result()
                    if self.success == 'saved':
                        self.saved = True
                        self.show_message('配置已保存，返回终端继续初始化。')
                        self.close()
                        return
                    if self.success == 'applied':
                        self.dirty = False
                    self.show_message('请核对画面和实物，并逐项勾选确认。')
                except (ValueError, RuntimeError, OSError) as exc:
                    self.state = session.state()
                    self.show_message(str(exc), True)
                except Exception as exc:
                    self.state = session.state()
                    self.show_message('设备操作失败：' + str(exc), True)
                if self.closing:
                    self.close()
                    return
            elif self.closing:
                self.close()
                return
            else:
                self.state = session.state()
            self.refresh()

        def refresh(self):
            busy = self.future is not None or self.closing
            if hasattr(self, 'apply_button'):
                for button in (self.apply_button, self.swap_button):
                    button.setEnabled(not busy)
                self.save_button.setEnabled(not busy and not self.dirty)
            self.options.setEnabled(not busy)
            if session.kind == 'cameras':
                for device in self.state['devices']:
                    card = self.cards[device['serial']]
                    card['role'].setEnabled(not busy)
                    card['rotation'].setEnabled(not busy)
                    card['feedback'].setText(device.get('error') or ('画面正常' if device['fresh'] else '等待最新画面；检查设备连接或占用'))
                    fresh = device['fresh'] and not device.get('error')
                    card['check'].setEnabled(not busy and not self.dirty and bool(device['role']) and fresh)
                    if not fresh:
                        card['check'].setChecked(False)
                        card['image'].clear()
                        card['image'].setText('等待最新画面')
                        card['stamp'] = None
                    elif not busy:
                        frame = session.image(device['index'])
                        if frame:
                            image = QtGui.QImage.fromData(frame[2])
                            if image.isNull():
                                card['check'].setEnabled(False)
                                card['check'].setChecked(False)
                                card['feedback'].setText('画面解码失败，请检查驱动输出')
                            else:
                                card['image'].setPixmap(QtGui.QPixmap.fromImage(image).scaled(
                                    card['image'].size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation))
            else:
                drive, enabled = self.drive.currentData(), self.hand.currentData() != 'none'
                self.ip_label.setVisible(drive == '485' and enabled)
                self.device_ip.setVisible(drive == '485' and enabled)
                self.drive.setEnabled(not busy and enabled)
                for side, card in self.cards.items():
                    card['box'].setEnabled(not busy and enabled)
                    card['port'].setVisible(drive == 'usb')
                    card['port_label'].setVisible(drive == 'usb')
                    for name in ('channel', 'channel_label', 'arm'):
                        card[name].setVisible(drive == '485')
                    result = self.state['results'].get(side, {})
                    card['feedback'].setText(result.get('message', '尚未测试'))
                    card['test'].setEnabled(not busy and not self.dirty and enabled)
                    card['check'].setEnabled(not busy and not self.dirty and enabled and bool(result.get('ok')))
                    if not result.get('ok'):
                        card['check'].setChecked(False)

        def closeEvent(self, event):
            if self.future:
                self.closing = True
                self.show_message('正在结束本次设备操作，随后释放设备并关闭窗口。')
                self.refresh()
                event.ignore()
                return
            self.timer.stop()
            if not self.saved:
                session.cancel()
            self.executor.shutdown(wait=True)
            event.accept()

    return Window()


def run_wizard(session, io):
    import signal
    try:
        from PyQt5 import QtCore, QtWidgets, sip
    except ImportError as exc:
        raise RuntimeError('缺少本地配置窗口依赖 python3-pyqt5，请先运行 ./scripts/setup.sh deps') from exc
    if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY') or os.environ.get('QT_QPA_PLATFORM') == 'offscreen'):
        raise RuntimeError('未找到桌面显示；请在桌面终端执行，或为容器映射 DISPLAY、X11 和授权文件')
    with open(session.config_path + '.wizard.lock', 'a', encoding='utf-8') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError('此配置文件已有初始化窗口，请先完成或取消该窗口') from exc
        app = QtWidgets.QApplication.instance()
        if app is None:
            # QLabel previews use raster painting and do not need a GLX visual.
            os.environ.setdefault('QT_XCB_GL_INTEGRATION', 'none')
            QtCore.QCoreApplication.setAttribute(QtCore.Qt.AA_UseSoftwareOpenGL)
            app = QtWidgets.QApplication(['W2 初始化'])
        app.setQuitOnLastWindowClosed(False)
        window = create_window(session)
        loop = QtCore.QEventLoop()
        window.setAttribute(QtCore.Qt.WA_DeleteOnClose)
        window.destroyed.connect(loop.quit)
        # Python must regularly regain control to handle Ctrl+C during Qt.exec_().
        interrupts = QtCore.QTimer()
        interrupts.timeout.connect(lambda: None)
        interrupts.start(100)
        interrupted = []
        previous_handler = signal.getsignal(signal.SIGINT)

        def cancel_signal(_number, _frame):
            interrupted.append(True)
            if not sip.isdeleted(window):
                window.close()

        signal.signal(signal.SIGINT, cancel_signal)
        io.info('已打开本地配置窗口；核对实物后确认保存，关闭窗口或 Ctrl+C 取消')
        io.out.flush()
        window.show()
        if session.kind == 'cameras':
            QtCore.QTimer.singleShot(0, window.apply)
        try:
            loop.exec_()
            if interrupted:
                raise KeyboardInterrupt
            return session.saved
        finally:
            signal.signal(signal.SIGINT, previous_handler)
            interrupts.stop()
            if not sip.isdeleted(window):
                window.close()
