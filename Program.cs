using nanoFramework.Hardware.Esp32;
using System;
using System.Device.Pwm;
using System.Device.Wifi;
using System.Diagnostics;
using System.Net;
using System.Net.NetworkInformation;
using System.Net.Sockets;
using System.Threading;
using System.Device.Gpio;



namespace NanoEsp32Car
{
    public class Program
    {

        private static WifiAdapter _wifi;
        private static ManualResetEvent _scanDone = new(false);

        // ===== 状态 LED 配置 =====
        private const int STATUS_LED_PIN = 2;  // 如果你用别的脚，就改这里
        private static GpioController _gpio;
        private static int _status = 0;

        // 0 = 正在连 WiFi（慢闪）
        // 1 = WiFi 正常，已进 UDP 循环（长亮）
        // 2 = WiFi 失败（快闪）


        // ===== 电机引脚 & PWM （4 独立电机）=====

        // 左前 LF（沿用你原来的左电机引脚）
        private const int LF_IN1 = 27;
        private const int LF_IN2 = 26;

        // 左后 LR（新增引脚，根据你板子实际可改）
        private const int LR_IN1 = 15;
        private const int LR_IN2 = 14;

        // 右前 RF（沿用你原来的右电机引脚）
        private const int RF_IN1 = 25;
        private const int RF_IN2 = 33;

        // 右后 RR（新增引脚，根据你板子实际可改）
        private const int RR_IN1 = 21;
        private const int RR_IN2 = 32;

        private const int PWM_FREQ = 20000;

        // 每个 IN 引脚一个 PWM 通道 → 8 路 PWM
        private static PwmChannel _pwmLf1;
        private static PwmChannel _pwmLf2;
        private static PwmChannel _pwmLr1;
        private static PwmChannel _pwmLr2;
        private static PwmChannel _pwmRf1;
        private static PwmChannel _pwmRf2;
        private static PwmChannel _pwmRr1;
        private static PwmChannel _pwmRr2;




        private const int UDP_PORT = 4210; // 手机往这个端口发

        // ===== 挡位 / 速度控制 =====
        // 挡位 0~5，0 = 不动，5 = 最快
        private const int MIN_GEAR = 0;
        private const int MAX_GEAR = 5;
        private static int _currentGear = 3; // 默认 3 档

        public static void Main()
        {
            // 1. 配置引脚为 PWM 功能（8 路）
            Configuration.SetPinFunction(LF_IN1, DeviceFunction.PWM1);
            Configuration.SetPinFunction(LF_IN2, DeviceFunction.PWM2);
            Configuration.SetPinFunction(LR_IN1, DeviceFunction.PWM3);
            Configuration.SetPinFunction(LR_IN2, DeviceFunction.PWM4);
            Configuration.SetPinFunction(RF_IN1, DeviceFunction.PWM5);
            Configuration.SetPinFunction(RF_IN2, DeviceFunction.PWM6);
            Configuration.SetPinFunction(RR_IN1, DeviceFunction.PWM7);
            Configuration.SetPinFunction(RR_IN2, DeviceFunction.PWM8);

            // 2. 创建 PWM 通道（8 个）
            _pwmLf1 = PwmChannel.CreateFromPin(LF_IN1, PWM_FREQ, 0);
            _pwmLf2 = PwmChannel.CreateFromPin(LF_IN2, PWM_FREQ, 0);
            _pwmLr1 = PwmChannel.CreateFromPin(LR_IN1, PWM_FREQ, 0);
            _pwmLr2 = PwmChannel.CreateFromPin(LR_IN2, PWM_FREQ, 0);

            _pwmRf1 = PwmChannel.CreateFromPin(RF_IN1, PWM_FREQ, 0);
            _pwmRf2 = PwmChannel.CreateFromPin(RF_IN2, PWM_FREQ, 0);
            _pwmRr1 = PwmChannel.CreateFromPin(RR_IN1, PWM_FREQ, 0);
            _pwmRr2 = PwmChannel.CreateFromPin(RR_IN2, PWM_FREQ, 0);

            _pwmLf1.Start();
            _pwmLf2.Start();
            _pwmLr1.Start();
            _pwmLr2.Start();
            _pwmRf1.Start();
            _pwmRf2.Start();
            _pwmRr1.Start();
            _pwmRr2.Start();


            _gpio = new GpioController();
            _gpio.OpenPin(STATUS_LED_PIN, PinMode.Output);
            _gpio.Write(STATUS_LED_PIN, PinValue.Low); // 先关灯

            // 启动状态灯线程（根据 _status 不同模式闪烁）
            new Thread(StatusLedLoop).Start();

            // 进入 WiFi 连接阶段：_status = 0（慢闪）
            _status = 0;
            // === 用 WifiAdapter 扫描 + 连接 ===
            if (!ConnectWifiViaAdapter())
            {
                Debug.WriteLine("主 / 备用 WiFi 均连接失败，请检查热点 / SSID / 密码。");
                while (true) { Thread.Sleep(5000); }
            }

            // 4. 启动 UDP 监听循环（替代串口循环）
            StartUdpLoop();
        }


        private static void StatusLedLoop()
        {
            while (true)
            {
                try
                {
                    switch (_status)
                    {
                        case 0: // WiFi 连接中：慢闪
                            _gpio.Write(STATUS_LED_PIN, PinValue.High);
                            Thread.Sleep(300);
                            _gpio.Write(STATUS_LED_PIN, PinValue.Low);
                            Thread.Sleep(300);
                            break;

                        case 1: // WiFi OK + UDP 已运行：长亮
                            _gpio.Write(STATUS_LED_PIN, PinValue.High);
                            Thread.Sleep(1000);
                            break;

                        case 2: // 出错：快闪
                            _gpio.Write(STATUS_LED_PIN, PinValue.High);
                            Thread.Sleep(100);
                            _gpio.Write(STATUS_LED_PIN, PinValue.Low);
                            Thread.Sleep(100);
                            break;

                        default: // 其他状态：熄灭
                            _gpio.Write(STATUS_LED_PIN, PinValue.Low);
                            Thread.Sleep(500);
                            break;
                    }
                }
                catch
                {
                    // 防止 GPIO 偶发异常把线程干死
                    Thread.Sleep(500);
                }
            }
        }



        /// <summary> /// 用 Wireless80211Configuration + NetworkInterface 连接 WiFi /// </summary>
        /// <summary>
        /// 总控：先尝试主 WiFi，再尝试备用 WiFi
        /// </summary>
        private static bool ConnectWifiViaAdapter()
        {
            try
            {
                Debug.WriteLine("使用 WifiAdapter 连接 WiFi（扫描附近网络）...");

                // 1. 找到第一个 WiFi 适配器
                var adapters = WifiAdapter.FindAllAdapters();
                if (adapters.Length == 0)
                {
                    Debug.WriteLine("没有找到任何 WifiAdapter！");
                    _status = 2; // 出错：快闪
                    return false;
                }

                // 2. 先尝试主 WiFi
                if (TryConnectSsid(Secrets.WIFI_SSID_PRIMARY, Secrets.WIFI_PASSWORD_PRIMARY))
                {
                    return true;
                }

                Debug.WriteLine("主 WiFi 连接失败，尝试备用 WiFi...");

                // 3. 再尝试备用 WiFi
                if (TryConnectSsid(Secrets.WIFI_SSID_SECONDARY, Secrets.WIFI_PASSWORD_SECONDARY))
                {
                    return true;
                }

                return false;
            }
            catch (Exception ex)
            {
                Debug.WriteLine("ConnectWifiViaAdapter 异常: " + ex.Message);
                _status = 2; // 出错：快闪
                return false;
            }
        }

        private static NetworkInterface GetStationInterface()
        {
            var nis = NetworkInterface.GetAllNetworkInterfaces();

            foreach (var ni in nis)
            {
                if (ni.NetworkInterfaceType == NetworkInterfaceType.Wireless80211)
                {
                    return ni;
                }
            }

            return nis[0];
        }

        private static bool TryConnectSsid(string ssid, string password)
        {
            try
            {
                Debug.WriteLine($"尝试直接连接 WiFi：{ssid}");

                // 拿到 WiFi 适配器
                var adapters = WifiAdapter.FindAllAdapters();
                if (adapters.Length == 0)
                {
                    Debug.WriteLine("没有找到 WifiAdapter！");
                    return false;
                }

                _wifi = adapters[0];

                // 断开当前连接
                _wifi.Disconnect();

                // 直接用 SSID + 密码连接（无需 Scan）
                WifiConnectionResult result = _wifi.Connect(ssid, WifiReconnectionKind.Automatic, password);

                if (result.ConnectionStatus != WifiConnectionStatus.Success)
                {
                    Debug.WriteLine($"连接 {ssid} 失败！状态 = {result.ConnectionStatus}");
                    return false;
                }

                Debug.WriteLine($"已连接到 AP[{ssid}]，正在获取 IP ...");

                // 等待 DHCP
                for (int i = 0; i < 20; i++)
                {
                    var ni = GetStationInterface();
                    Debug.WriteLine($"  当前 IP = {ni.IPv4Address}");

                    if (!string.IsNullOrEmpty(ni.IPv4Address) && ni.IPv4Address != "0.0.0.0")
                    {
                        Debug.WriteLine($"WiFi[{ssid}] 连接成功，IP = {ni.IPv4Address}");
                        // WiFi 成功 → 状态灯改为常亮
                        _status = 1;
                        return true;
                    }

                    Thread.Sleep(500);
                }

                Debug.WriteLine($"WiFi[{ssid}] 已连接但未拿到 IP，可能是 DHCP 慢。");
                return true;
            }
            catch (Exception ex)
            {
                Debug.WriteLine("TryConnectSsid 直接连接异常: " + ex.Message);
                return false;
            }
        }







        /// <summary>
        /// UDP 循环：收 DISCOVER_ESP32 或控制指令
        /// </summary>
        private static void StartUdpLoop()
        {
            System.Diagnostics.Debug.WriteLine($"启动 UDP 监听，端口 {UDP_PORT} …");

            using var udpSocket = new Socket(
                AddressFamily.InterNetwork,
                SocketType.Dgram,
                ProtocolType.Udp);

            udpSocket.Bind(new IPEndPoint(IPAddress.Any, UDP_PORT));

            byte[] buffer = new byte[32];

            while (true)
            {
                try
                {
                    // Poll 200ms 看看有没有数据
                    if (udpSocket.Poll(200 * 1000, SelectMode.SelectRead))
                    {
                        EndPoint remote = new IPEndPoint(IPAddress.Any, 0);
                        int received = udpSocket.ReceiveFrom(buffer, ref remote);

                        if (received > 0)
                        {
                            // 将收到的数据转成字符串（只取前 received 字节）
                            string msg = new string(System.Text.Encoding.UTF8.GetChars(buffer, 0, received));

                            // 处理发现小车命令
                            if (msg == "DISCOVER_ESP32")
                            {
                                SendDiscoverReply(remote);
                            }
                            else if (msg == "ESTOP")
                            {
                                EmergencyStop();
                            }
                            else if (msg.StartsWith("PAD|"))
                            {
                                HandlePadPacket(msg);
                            }
                            else if (msg.Length > 0)
                            {
                                char c = msg[0];
                                HandleCommand(c);

                                var ep = (IPEndPoint)remote;
                                System.Diagnostics.Debug.WriteLine($"Recv '{c}' from {ep.Address}:{ep.Port}");
                            }

                        }
                    }
                }
                catch (Exception ex)
                {
                    System.Diagnostics.Debug.WriteLine("UDP 接收异常: " + ex.Message);
                    Thread.Sleep(100);
                }

                Thread.Sleep(10);
            }
        }

        private static void HandlePadPacket(string msg)
        {
            // 去掉 "PAD|"
            string body = msg.Substring(4);

            int lx = 0, ly = 0, rt = 0;

            try
            {
                var parts = body.Split('|'); // e.g. ["LX=-30","LY=80","RT=60"]

                for (int i = 0; i < parts.Length; i++)
                {
                    var p = parts[i];
                    var kv = p.Split('=');
                    if (kv.Length != 2) continue;

                    string key = kv[0];
                    string value = kv[1];

                    int v;
                    if (!int.TryParse(value, out v)) continue;

                    if (key == "LX") lx = v;
                    else if (key == "LY") ly = v;
                    else if (key == "RT") rt = v;
                }

                HandlePadAxes(lx, ly, rt);
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine("HandlePadPacket parse error: " + ex.Message);
            }
        }



        private static void HandlePadAxes(int lx, int ly, int rt)
        {
            // 死区，防止手柄抖动造成小车抖
            if (lx > -10 && lx < 10) lx = 0;
            if (ly > -10 && ly < 10) ly = 0;

            double f = 1.0;      // 固定向前
            double t = 0.0;      // 不转向
            double mag = 1.0;    // 视作满量程推到底

            // === 情况 1：摇杆完全居中 ===
            if (lx == 0 && ly == 0)
            {
                if (rt <= 0)
                {
                    // 扳机也没扣 → 停车
                    Stop();
                    return;
                }

                // 扳机扣下，但摇杆没动：
                // 约定为“直线前进”，方向固定向前，速度由 RT 决定
                f = 1.0;      // 固定向前
                t = 0.0;      // 不转向
                mag = 1.0;    // 视作满量程推到底

                double throttle = rt / 100.0;              // 0~1
                double baseSpeed = (0.2 + 0.8 * throttle);  // 最少 0.2，拉满 1.0
                double speed = baseSpeed * mag;

                double left = f + t;
                double right = f - t;

                left = Clamp(left, -1.0, 1.0);
                right = Clamp(right, -1.0, 1.0);

                SetMotorByAxis(left, right, speed);

                System.Diagnostics.Debug.WriteLine(
                    $"PAD[RT-only]: LX={lx}, LY={ly}, RT={rt}, L={left:F2}, R={right:F2}, speed={speed:F2}");
                return;
            }

            // === 情况 2：摇杆有偏移 → 用摇杆决定方向，RT 只当油门 ===

            // 映射到 -1 ~ 1
            f = ly / 100.0;    // forward/back（如果前后反，可以改成 -ly/100.0）
            t = -lx / 100.0;   // turn（我们之前修正过左右，所以这里取了负号）

            // 摇杆自身的强度（推得越远越快）
            mag = FastSqrt(f * f + t * t);
            if (mag > 1.0) mag = 1.0;

            // 基础速度来自摇杆距离 + RT 放大
            double throttle2 = rt / 100.0;                 // 0~1
            double baseSpeed2 = (0.2 + 0.8 * throttle2);    // 最少 0.2，拉满 1.0
            double speed2 = baseSpeed2 * mag;

            // 差速
            double left2 = f + t;
            double right2 = f - t;

            left2 = Clamp(left2, -1.0, 1.0);
            right2 = Clamp(right2, -1.0, 1.0);

            SetMotorByAxis(left2, right2, speed2);

            System.Diagnostics.Debug.WriteLine(
                $"PAD: LX={lx}, LY={ly}, RT={rt}, L={left2:F2}, R={right2:F2}, speed={speed2:F2}");
        }

        private static double FastSqrt(double x)
        {
            if (x <= 0) return 0;
            double r = x;
            for (int i = 0; i < 6; i++)
            {
                r = 0.5 * (r + x / r); // 牛顿迭代
            }
            return r;
        }


        /// <summary>简单 Clamp 实现</summary>
        private static double Clamp(double v, double min, double max)
        {
            if (v < min) return min;
            if (v > max) return max;
            return v;
        }

        /// <summary>
        /// 根据左右轴值（-1~1）和基础速度，设置 4 个电机的 PWM。
        /// left > 0 表示左侧前进，<0 表示后退；右侧同理。
        /// </summary>
        private static void SetMotorByAxis(double left, double right, double baseSpeed)
        {
            // 左侧
            double leftAbs = left >= 0 ? left : -left;
            double leftDuty = baseSpeed * leftAbs;
            // 右侧
            double rightAbs = right >= 0 ? right : -right;
            double rightDuty = baseSpeed * rightAbs;

            // 左前/左后
            if (left > 0)
            {
                // 前进：IN1 有 PWM，IN2 0
                _pwmLf1.DutyCycle = leftDuty;
                _pwmLf2.DutyCycle = 0;
                _pwmLr1.DutyCycle = leftDuty;
                _pwmLr2.DutyCycle = 0;
            }
            else if (left < 0)
            {
                // 后退：IN1 0，IN2 有 PWM
                _pwmLf1.DutyCycle = 0;
                _pwmLf2.DutyCycle = leftDuty;
                _pwmLr1.DutyCycle = 0;
                _pwmLr2.DutyCycle = leftDuty;
            }
            else
            {
                _pwmLf1.DutyCycle = 0;
                _pwmLf2.DutyCycle = 0;
                _pwmLr1.DutyCycle = 0;
                _pwmLr2.DutyCycle = 0;
            }

            // 右前/右后
            if (right > 0)
            {
                _pwmRf1.DutyCycle = rightDuty;
                _pwmRf2.DutyCycle = 0;
                _pwmRr1.DutyCycle = rightDuty;
                _pwmRr2.DutyCycle = 0;
            }
            else if (right < 0)
            {
                _pwmRf1.DutyCycle = 0;
                _pwmRf2.DutyCycle = rightDuty;
                _pwmRr1.DutyCycle = 0;
                _pwmRr2.DutyCycle = rightDuty;
            }
            else
            {
                _pwmRf1.DutyCycle = 0;
                _pwmRf2.DutyCycle = 0;
                _pwmRr1.DutyCycle = 0;
                _pwmRr2.DutyCycle = 0;
            }
        }

        private static void EmergencyStop()
        {
            Stop();            // 直接 PWM = 0
            SetGear(0);        // 挡位清零，防止下一次按键残留
            System.Diagnostics.Debug.WriteLine("!!! EMERGENCY STOP !!!");

            // 可以顺便让状态灯快速闪几下提示
            new Thread(() =>
            {
                var old = _status;
                for (int i = 0; i < 5; i++)
                {
                    _gpio.Write(STATUS_LED_PIN, PinValue.High);
                    Thread.Sleep(80);
                    _gpio.Write(STATUS_LED_PIN, PinValue.Low);
                    Thread.Sleep(80);
                }
                _status = old;
            }).Start();
        }

        private static void SendDiscoverReply(EndPoint remote)
        {
            try
            {
                // ----- 获取 MAC 地址 -----
                NetworkInterface ni = NetworkInterface.GetAllNetworkInterfaces()[0];
                byte[] macBytes = ni.PhysicalAddress;

                string mac = "";
                for (int i = 0; i < macBytes.Length; i++)
                {
                    mac += macBytes[i].ToString("X2");
                    if (i < macBytes.Length - 1) mac += ":";
                }

                // ----- 获取 IP 地址 -----
                string ip = ni.IPv4Address;

                // ----- 拼装响应报文 -----
                string response = $"CAR|MAC={mac}|IP={ip}";

                // ----- 发送回去 -----
                var ep = (IPEndPoint)remote;
                byte[] data = System.Text.Encoding.UTF8.GetBytes(response);

                using (var udp = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp))
                {
                    udp.SendTo(data, ep);
                }

                System.Diagnostics.Debug.WriteLine($"DISCOVER 响应已发送：{response}");
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine("SendDiscoverReply Error: " + ex.Message);
            }
        }

        // ============ 挡位相关逻辑 ============

        private static void SetGear(int gear)
        {
            if (gear < MIN_GEAR) gear = MIN_GEAR;
            if (gear > MAX_GEAR) gear = MAX_GEAR;
            _currentGear = gear;
            System.Diagnostics.Debug.WriteLine($"当前挡位：{_currentGear}");
        }

        private static void GearUp()
        {
            SetGear(_currentGear + 1);
        }

        private static void GearDown()
        {
            SetGear(_currentGear - 1);
        }

        /// <summary>
        /// 根据挡位换算占空比（0.0~1.0）
        /// </summary>
        private static double GetDutyByGear()
        {
            switch (_currentGear)
            {
                case 0: return 0.0;   // 熄火
                case 1: return 0.45;  // 慢速
                case 2: return 0.60;  // 常规
                case 3: return 0.75;  // 偏快（推荐日常用）
                case 4: return 0.90;  // 很快
                case 5: return 1.00;  // 地板油
                default: return 0.0;
            }
        }

        // ============ 电机控制逻辑：基于当前挡位的速度（4 独立电机，同侧同速） ============

        /// <summary>
        /// 设置四个电机两侧的“腿”占空比：
        /// L1/L2 对应左侧前后电机的 IN1/IN2；
        /// R1/R2 对应右侧前后电机的 IN1/IN2。
        /// </summary>
        private static void SetMotor(double L1, double L2, double R1, double R2)
        {
            // 左前
            _pwmLf1.DutyCycle = L1;
            _pwmLf2.DutyCycle = L2;
            // 左后
            _pwmLr1.DutyCycle = L1;
            _pwmLr2.DutyCycle = L2;

            // 右前
            _pwmRf1.DutyCycle = R1;
            _pwmRf2.DutyCycle = R2;
            // 右后
            _pwmRr1.DutyCycle = R1;
            _pwmRr2.DutyCycle = R2;
        }

        private static void Stop() => SetMotor(0, 0, 0, 0);

        // 直线前进：用原来“后退”的那组方向
        // 直线前进：用物理上“向前”的方向
        private static void Forward()
        {
            double v = GetDutyByGear();
            // 如果你前后已经改过，这里确保是：车物理上往前跑
            SetMotor(0, v, 0, v);
        }

        // 直线后退：用物理上“向后”的方向
        private static void Backward()
        {
            double v = GetDutyByGear();
            SetMotor(v, 0, v, 0);
        }

        /// <summary>
        /// 差速左转：按 A 键时，希望车“向左拐着前进”
        /// 👉 把“左快右慢”/“左慢右快”对调一下
        /// </summary>
        private static void ForwardLeft()
        {
            double v = GetDutyByGear();
            double vSlow = v * 0.4;
            double vFast = v;

            // 之前如果是左慢右快导致“倒方向”
            // 现在改成：左侧快、右侧慢 → 车向左拐
            SetMotor(vFast, 0, vSlow, 0);
        }

        /// <summary>
        /// 差速右转：按 D 时，车向右拐
        /// </summary>
        private static void ForwardRight()
        {
            double v = GetDutyByGear();
            double vSlow = v * 0.4;
            double vFast = v;

            // 左慢右快 → 车向右拐
            SetMotor(vSlow, 0, vFast, 0);
        }

        /// <summary>
        /// 原地左转（Q）：左轮后退，右轮前进
        /// 如果你现在按 Q 变成向右转，就把左右对调
        /// </summary>
        private static void TurnLeft()
        {
            double v = GetDutyByGear();
            // 左后退，右前进 → 车头向左
            SetMotor(v, 0, 0, v);
        }

        /// <summary>
        /// 原地右转（E）
        /// </summary>
        private static void TurnRight()
        {
            double v = GetDutyByGear();
            // 左前进，右后退 → 车头向右
            SetMotor(0, v, v, 0);
        }


        private static void HandleCommand(char c)
        {
            c = c.ToUpper();

            switch (c)
            {
                // 挡位切换：数字 1~5
                case '0': SetGear(0); break;
                case '1': SetGear(1); break;
                case '2': SetGear(2); break;
                case '3': SetGear(3); break;
                case '4': SetGear(4); break;
                case '5': SetGear(5); break;

                // 挡位加减：Z = 降档，C = 升档
                case 'Z': GearDown(); break;
                case 'C': GearUp(); break;

                // 方向
                case 'W': Forward(); break;
                case 'S': Backward(); break;
                case 'A': TurnRight(); break;
                case 'D': TurnLeft(); break;
                case 'X': Stop(); break;

                default:
                    // 其他字符忽略
                    break;
            }

            System.Diagnostics.Debug.WriteLine($"指令: {c}，挡位：{_currentGear}");

            // 收到任何控制指令时，短暂快闪一下（在状态灯线程基础上叠加）
            new Thread(() =>
            {
                var old = _status;
                _gpio.Write(STATUS_LED_PIN, PinValue.High);
                Thread.Sleep(80);
                _gpio.Write(STATUS_LED_PIN, PinValue.Low);
                Thread.Sleep(80);
                _status = old;
            }).Start();
        }
    }
}
