using System;
using System.Collections.ObjectModel;
using NLog;

namespace Cobilsys_CanGui.Services
{
    public partial class CanService
    {
        protected readonly ObservableCollection<string> LogItems;
        private static readonly Logger _log = LogManager.GetCurrentClassLogger();

        public CanService(ObservableCollection<string> logItems)
        {
            LogItems = logItems;
            PlatformInit();
        }

        // ======================
        // Speed / RPM
        // ======================
        public void SendSpeed(int speed)
        {
            AddLog($"SendSpeed({speed}) 호출");
            PlatformSendSpeed(speed);
        }

        public void SendRpm(int rpm)
        {
            AddLog($"SendRpm({rpm}) 호출");
            PlatformSendRpm(rpm);
        }

        // ======================
        // Engine Status
        // ======================
        public void SendEngineStatus(bool on)
        {
            string state = on ? "ON" : "OFF";
            AddLog($"SendEngineStatus({state}) 호출");
            PlatformSendEngineStatus(on);
        }

        // ======================
        // Tail Lamp
        // ======================
        public void SendTailLamp(bool on)
        {
            string state = on ? "ON" : "OFF";
            AddLog($"SendTailLamp({state}) 호출");
            PlatformSendTailLamp(on);
        }

        // ======================
        // Brake Switch
        // ======================
        public void SendBrakeSwitch(bool on)
        {
            string state = on ? "ON" : "OFF";
            AddLog($"SendBrakeSwitch({state}) 호출");
            PlatformSendBrakeSwitch(on);
        }

        // ======================
        // 로그 기록
        // ======================
        protected void AddLog(string message, string level = "INFO")
        {
            string line = $"[{DateTime.Now:HH:mm:ss}] [{level}] {message}";
            LogItems.Add(line);

            switch (level.ToUpper())
            {
                case "ERROR": _log.Error(message); break;
                case "WARN":  _log.Warn(message); break;
                default:      _log.Info(message); break;
            }
        }
    }
}
