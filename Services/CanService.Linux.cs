#if LINUX
using System;
using System.IO.Ports;
using System.Threading;
using System.Threading.Tasks;
using System.Device.Gpio;
using System.IO;
using System.Linq;

namespace Cobilsys_CanGui.Services
{
    public partial class CanService
    {
        private SerialPort? _port;

        // ====== 상태 변수 
        private double _vehicleSpeed = 0;   // km/h
        private int _brakeSwitch = 0;       // 0=OFF, 1=ON
        private int _rpm = 0;               // RPM
        private int _tailLamp = 0;          // 0=OFF, 1=ON
        private int _engineStatus = 1;      // 1=OFF, 3=Running

        private readonly object _stateLock = new();

        // ====== 이전 전송값 (중복 로그 방지용) ======
        private double _lastVehicleSpeed = -1;
        private int _lastBrakeSwitch = -1;
        private int _lastRpm = -1;
        private int _lastTailLamp = -1;
        private int _lastEngineStatus = -1;

        // ====== 주기 루프용 ======
        private CancellationTokenSource? _cts;
 
        // 허브 전원
        private void EnsureUsbHubPower(bool activeHigh = true)
        {
            try
            {
                using var gpio = new GpioController();
                int[] pins = { 22, 27 };

                foreach (var p in pins)
                {
                    if (!gpio.IsPinOpen(p)) gpio.OpenPin(p, PinMode.Output);
                    gpio.Write(p, activeHigh ? PinValue.High : PinValue.Low);
                }

                Thread.Sleep(400);
                AddLog($"GPIO 22/27 {(activeHigh ? "HIGH" : "LOW")} → 허브 전원 인가");
            }
            catch (Exception ex)
            {
                AddLog($"GPIO 제어 실패: {ex.Message} (gpio 그룹/권한 확인)", "WARN");

                try
                {
                    var level = activeHigh ? "dh" : "dl";
                    var psi = new System.Diagnostics.ProcessStartInfo("/usr/bin/env", $"bash -lc \"raspi-gpio set 22 op {level}; raspi-gpio set 27 op {level}\"")
                    { RedirectStandardError = true, RedirectStandardOutput = true };
                    System.Diagnostics.Process.Start(psi)?.WaitForExit();
                    Thread.Sleep(300);
                    AddLog("raspi-gpio 폴백으로 22/27 설정 완료");
                }
                catch { }
            }
        }

        // Serial 경로 확인
        private string? ResolveSerialPortPathOnce()
        {
            try
            {
                var byId = "/dev/serial/by-id";
                if (Directory.Exists(byId))
                {
                    var cand = Directory.EnumerateFiles(byId)
                        .FirstOrDefault(p => p.Contains("1a86", StringComparison.OrdinalIgnoreCase)
                                        || p.Contains("CH34", StringComparison.OrdinalIgnoreCase)
                                        || p.Contains("HL-340", StringComparison.OrdinalIgnoreCase));
                    if (cand != null) return cand;
                }
            }
            catch { }

            var usbs = Directory.EnumerateFiles("/dev", "ttyUSB*").OrderBy(p => p).ToList();
            return usbs.FirstOrDefault();
        }

        private string WaitForSerialPortPath(int timeoutMs = 5000, int pollMs = 200)
        {
            var sw = System.Diagnostics.Stopwatch.StartNew();
            while (sw.ElapsedMilliseconds < timeoutMs)
            {
                var path = ResolveSerialPortPathOnce();
                if (!string.IsNullOrEmpty(path) && (File.Exists(path) || path.Contains("/dev/serial/")))
                    return path;

                Thread.Sleep(pollMs);
            }
            throw new Exception("시리얼 장치가 시간 내에 나타나지 않았습니다.");
        }


        // 초기화
        private void PlatformInit()
        {
            try
            {
                // 1) 허브 전원 ON
                EnsureUsbHubPower(activeHigh: true);

                // 2) 장치 enumerate 대기
                string portPath = WaitForSerialPortPath(timeoutMs: 6000);
                AddLog($"Linux: Serial 후보 경로 = {portPath}");

                // 3) 포트 Open
                _port = new SerialPort(portPath, 2000000, Parity.None, 8, StopBits.One);
                _port.Open();
                AddLog($"Linux: {portPath} 연결 성공 (2Mbps)");

                // 4) CAN 설정 및 루프 시작
                SendCommandSettings(500000, CanUsbMode.Normal, CanUsbFrame.Extended);
                Thread.Sleep(50);

                StartPeriodicSend();
                StartReceiveLoop();
            }
            catch (Exception ex)
            {
                AddLog($"Linux: SerialPort 열기 실패 - {ex.Message}", "ERROR");
                _port = null;
            }
        }

        // ENUM 정의
        private enum CanUsbMode : byte
        {
            Normal = 0x00,
            Loopback = 0x01,
            Silent = 0x02,
            LoopbackSilent = 0x03
        }

        private enum CanUsbFrame : byte
        {
            Standard = 0x01,
            Extended = 0x02
        }


        // 설정 프레임 송신
        private void SendCommandSettings(int bitrate, CanUsbMode mode, CanUsbFrame frame)
        {
            if (_port == null || !_port.IsOpen) return;

            byte[] pkt = new byte[20];
            int i = 0;

            pkt[i++] = 0xAA;
            pkt[i++] = 0x55;
            pkt[i++] = 0x12;
            pkt[i++] = ConvertSpeed(bitrate);
            pkt[i++] = (byte)frame;
            pkt[i++] = 0; pkt[i++] = 0; pkt[i++] = 0; pkt[i++] = 0;
            pkt[i++] = 0; pkt[i++] = 0; pkt[i++] = 0; pkt[i++] = 0;
            pkt[i++] = (byte)mode;
            pkt[i++] = 0x01;
            pkt[i++] = 0; pkt[i++] = 0; pkt[i++] = 0; pkt[i++] = 0;
            pkt[i]   = CalcChecksum(pkt, 2, 17);

            _port.Write(pkt, 0, pkt.Length);
            AddLog($"설정 프레임 송신: {bitrate} bps, Mode={mode}, Frame={frame}");
        }

        private byte ConvertSpeed(int bps) => bps switch
        {
            1000000 => 0x01,
            800000  => 0x02,
            500000  => 0x03,
            400000  => 0x04,
            250000  => 0x05,
            200000  => 0x06,
            125000  => 0x07,
            100000  => 0x08,
            50000   => 0x09,
            20000   => 0x0A,
            10000   => 0x0B,
            5000    => 0x0C,
            _       => 0x03
        };

        private byte CalcChecksum(byte[] buf, int start, int len)
        {
            int sum = 0;
            for (int i = start; i < start + len; i++)
                sum += buf[i];
            return (byte)(sum & 0xFF);
        }


        // CAN 프레임 송신
        private void SendCanFrame(uint canId, byte[] data, bool extended = true)
        {
            if (_port == null || !_port.IsOpen) return;
            if (data.Length > 8) throw new ArgumentException("CAN data length must be 0~8");

            byte[] frame = new byte[15];
            int i = 0;

            frame[i++] = 0xAA;
            byte info = 0xC0;
            if (!extended) info &= 0xDF; else info |= 0x20;
            info &= 0xEF;
            info |= (byte)data.Length;
            frame[i++] = info;

            frame[i++] = (byte)(canId & 0xFF);
            frame[i++] = (byte)((canId >> 8) & 0xFF);
            if (extended)
            {
                frame[i++] = (byte)((canId >> 16) & 0xFF);
                frame[i++] = (byte)((canId >> 24) & 0xFF);
            }

            for (int d = 0; d < data.Length; d++)
                frame[i++] = data[d];

            frame[i++] = 0x55;

            _port.Write(frame, 0, i);
        }

        // ============================
        // 비트 패킹
        // ============================
        private void SetBitsLE(byte[] data, int startBit, int length, ulong value)
        {
            if (length == 0) return;
            int bitPos = startBit;
            int bitsLeft = length;
            int byteIndex = bitPos / 8;
            int bitOffset = bitPos % 8;

            while (bitsLeft > 0 && byteIndex < data.Length)
            {
                int canTake = Math.Min(8 - bitOffset, bitsLeft);
                byte mask = (byte)(((1 << canTake) - 1) << bitOffset);

                byte chunk = (byte)(value & ((1u << canTake) - 1u));
                data[byteIndex] &= (byte)~mask;
                data[byteIndex] |= (byte)((chunk << bitOffset) & mask);

                value >>= canTake;
                bitsLeft -= canTake;
                byteIndex++;
                bitOffset = 0;
            }
        }

        // ============================
        // 개별 메시지 송신
        // ============================
        private void Send_CCVS(double speedKph, int brakeSwitch)
        {
            byte[] data = new byte[8];

            long speedRaw = (long)Math.Round(speedKph / 0.00390625);
            if (speedRaw < 0) speedRaw = 0;
            if (speedRaw > 0xFFFD) speedRaw = 0xFFFD;

            SetBitsLE(data, 8, 16, (ulong)speedRaw);
            SetBitsLE(data, 28, 2, (ulong)(brakeSwitch & 0x3));

            SendCanFrame(0x18FEF100, data, true);

            if (speedKph != _lastVehicleSpeed || brakeSwitch != _lastBrakeSwitch)
            {
                AddLog($"CCVS 송신: Speed={speedKph:F1} km/h, Brake={(brakeSwitch == 1 ? "ON" : "OFF")}");
                _lastVehicleSpeed = speedKph;
                _lastBrakeSwitch = brakeSwitch;
            }
        }

        private void Send_EEC1_EngSpeed(int rpm)
        {
            byte[] data = new byte[8];
            long raw = (long)Math.Round(rpm / 0.125);
            if (raw < 0) raw = 0;
            if (raw > 0xFFFD) raw = 0xFFFD;

            SetBitsLE(data, 24, 16, (ulong)raw);
            SendCanFrame(0x0CF00400, data, true);

            if (rpm != _lastRpm)
            {
                AddLog($"EEC1 송신: RPM={rpm}");
                _lastRpm = rpm;
            }
        }

        private void Send_IBU_BCM_TailLamp(int lampReq)
        {
            byte[] data = new byte[8];
            SetBitsLE(data, 52, 2, (ulong)(lampReq & 0x3));
            SendCanFrame(0x18FF44E6, data, true);

            if (lampReq != _lastTailLamp)
            {
                AddLog($"IBU_BCM 송신: TailLamp={(lampReq == 1 ? "ON" : "OFF")}");
                _lastTailLamp = lampReq;
            }
        }

        private void Send_PROSCR2_EngineOp(int status)
        {
            byte[] data = new byte[8];
            SetBitsLE(data, 0, 4, (ulong)(status & 0xF));
            SendCanFrame(0x18FF1100, data, true);

            if (status != _lastEngineStatus)
            {
                AddLog($"PROSCR2 송신: EngineStatus={(status == 3 ? "ON" : "OFF")}");
                _lastEngineStatus = status;
            }
        }

        // ============================
        // 주기 송신 (async loop)
        // ============================
        public void StartPeriodicSend()
        {
            _cts = new CancellationTokenSource();
            Task.Run(() => PeriodicLoop(_cts.Token));
            AddLog("주기 송신 시작됨 (async loop)");
        }

        private async Task PeriodicLoop(CancellationToken ct)
        {
            long ccvsLast = 0;
            long eec1Last = 0;
            long lampLast = 0;
            long proscrLast = 0;

            var sw = System.Diagnostics.Stopwatch.StartNew();
            while (!ct.IsCancellationRequested)
            {
                double speed; int brake; int rpm; int lamp; int eng;

                // lock 으로 값 snapshot
                lock (_stateLock)
                {
                    speed = _vehicleSpeed;
                    brake = _brakeSwitch;
                    rpm = _rpm;
                    lamp = _tailLamp;
                    eng = _engineStatus;
                }

                long ms = sw.ElapsedMilliseconds;

                if (ms - ccvsLast >= 100)
                {
                    Send_CCVS(speed, brake);
                    ccvsLast = ms;
                }
                if (ms - eec1Last >= 10)
                {
                    Send_EEC1_EngSpeed(rpm);
                    eec1Last = ms;
                }
                if (ms - lampLast >= 200)
                {
                    Send_IBU_BCM_TailLamp(lamp);
                    lampLast = ms;
                }
                if (ms - proscrLast >= 10)
                {
                    Send_PROSCR2_EngineOp(eng);
                    proscrLast = ms;
                }

                await Task.Delay(1, ct);
            }
        }

        // ============================
        // 값 세팅 (lock 보호)
        // ============================
        public void SetVehicleSpeed(double speed) { lock (_stateLock) { _vehicleSpeed = speed; } }
        public void SetBrakeSwitch(bool on)      { lock (_stateLock) { _brakeSwitch = on ? 1 : 0; } }
        public void SetRpm(int rpm)              { lock (_stateLock) { _rpm = rpm; } }
        public void SetTailLamp(bool on)         { lock (_stateLock) { _tailLamp = on ? 1 : 0; } }
        public void SetEngineStatus(bool on)     { lock (_stateLock) { _engineStatus = on ? 3 : 1; } }

        // ============================
        // 수신 루프
        // ============================
        private void StartReceiveLoop()
        {
            if (_port == null) return;
            Task.Run(() =>
            {
                try
                {
                    byte[] buffer = new byte[1024];
                    int index = 0;

                    while (_port != null && _port.IsOpen)
                    {
                        int b = _port.ReadByte();
                        if (b < 0) continue;

                        buffer[index++] = (byte)b;

                        if (index > 2 && buffer[0] == 0xAA)
                        {
                            if (buffer[index - 1] == 0x55)
                            {
                                string hex = BitConverter.ToString(buffer, 0, index).Replace("-", " ");
                                AddLog($"수신 RAW: {hex}");
                                index = 0;
                            }
                        }
                        if (index >= buffer.Length) index = 0;
                    }
                }
                catch (Exception ex)
                {
                    AddLog($"수신 루프 종료: {ex.Message}", "ERROR");
                }
            });
        }

        private void PlatformSendSpeed(int speedKph) => SetVehicleSpeed(speedKph);
        private void PlatformSendRpm(int rpm) => SetRpm(rpm);
        private void PlatformSendEngineStatus(bool on) => SetEngineStatus(on);
        private void PlatformSendTailLamp(bool on) => SetTailLamp(on);
        private void PlatformSendBrakeSwitch(bool on) => SetBrakeSwitch(on);
    }
}
#endif
