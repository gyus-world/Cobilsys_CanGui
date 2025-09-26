#if WINDOWS
using System;

namespace Cobilsys_CanGui.Services
{
    public partial class CanService
    {
        private void PlatformInit()
        {
            AddLog("Windows: Can 기능 비활성화(로그만)");
        }

        private void PlatformSendSpeed(int speed)
        {
            AddLog($"Windows 입력 - Speed {speed} (송신안함)");
        }

        private void PlatformSendRpm(int rpm)
        {
            AddLog($"Windows 입력 - RPM {rpm} (송신안함)");
        }

        private void PlatformSendEngineStatus(bool on)
        {
            AddLog($"Windows 입력 - Engine {(on ? "ON" : "OFF")} (송신안함)");
        }

        private void PlatformSendTailLamp(bool on)
        {
            AddLog($"Windows 입력 - Tail Lamp {(on ? "ON" : "OFF")} (송신안함)");
        }

        private void PlatformSendBrakeSwitch(bool on)
        {
            AddLog($"Windows 입력 - Brake Switch {(on ? "ON" : "OFF")} (송신안함)");
        }
    }
}
#endif
