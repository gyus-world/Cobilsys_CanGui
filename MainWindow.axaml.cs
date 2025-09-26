using Avalonia;
using Avalonia.Controls;
using Avalonia.Media;
using Avalonia.Threading;
using Avalonia.VisualTree;
using System;
using System.Collections.ObjectModel;
using System.Diagnostics;
using System.Threading.Tasks;
using System.Linq; // 
using NLog;
using Cobilsys_CanGui.Services;
using System.Collections.Specialized;

namespace Cobilsys_CanGui
{
    public partial class MainWindow : Window
    {
        private static readonly Logger _log = LogManager.GetCurrentClassLogger();
        private readonly ObservableCollection<string> _logItems = new();
        private readonly ObservableCollection<string> _wifiList = new();

        private readonly ObservableCollection<string> _updateLogItems = new();

        private readonly CanService _can;

        private int _rpm = 0;
        private int _speed = 0;

        public MainWindow()
        {
            InitializeComponent();
            this.WindowState = WindowState.Maximized;

            LogList.ItemsSource = _logItems;
            _can = new CanService(_logItems);
            _logItems.CollectionChanged += OnLogItemsChanged;

            UpdateLogList.ItemsSource = _updateLogItems;

            // 홈/설정 화면 전환
            HomeButton.Click += (_, __) => { HomePanel.IsVisible = true; SettingsPanel.IsVisible = false; };
            SettingsButton.Click += (_, __) => { HomePanel.IsVisible = false; SettingsPanel.IsVisible = true; };

            // Wi-Fi 목록 초기화
            WifiListBox.ItemsSource = _wifiList;
            WifiRefreshButton.Click += async (_, __) => await LoadWifiList();
            WifiConnectButton.Click += (_, __) => ConnectWifi();
            _ = LoadWifiList();

            
            WifiPasswordBox.GotFocus += (_, __) =>
            {
                try
                {
                    Process.Start(new ProcessStartInfo
                    {
                        FileName = "/usr/bin/onboard",
                        Arguments = "--x 0 --y 1100 --no-reposition",
                        UseShellExecute = false
                    });
                }
                catch (Exception ex)
                {
                    AddLog($"가상 키보드 실행 실패: {ex.Message}", "ERROR");
                }
            };

            WifiPasswordBox.LostFocus += (_, __) =>
            {
                try { Process.Start("pkill", "onboard"); } catch { }
            };

            UpdateButton.Click += async (_, __) =>
            {
                AddUpdateLog("업데이트 요청");
                try
                {
                    var psi = new ProcessStartInfo
                    {
                        FileName = "bash",
                        ArgumentList = { "-c", "/home/cobilsys/Cobilsys_CanGui/update.sh" },
                        RedirectStandardOutput = true,
                        RedirectStandardError = true,
                        UseShellExecute = false,
                        CreateNoWindow = true
                    };

                    using var process = new Process { StartInfo = psi };

                    process.OutputDataReceived += (_, e) =>
                    {
                        if (!string.IsNullOrWhiteSpace(e.Data))
                        {
                            AddUpdateLog(e.Data);

                            // ✅ 완료 신호 감지
                            if (e.Data.Contains("Update complete", StringComparison.OrdinalIgnoreCase))
                            {
                                AddUpdateLog("업데이트가 성공적으로 완료되었습니다. 곧 종료 후 재실행됩니다.");
                                Task.Run(async () =>
                                {
                                    await Task.Delay(1000); 
                                    Environment.Exit(0); 
                                });    

                                }
                        }
                    };

                    process.ErrorDataReceived += (_, e) =>
                    {
                        if (!string.IsNullOrWhiteSpace(e.Data))
                            AddUpdateLog(e.Data, "ERROR");
                    };

                    process.Start();
                    process.BeginOutputReadLine();
                    process.BeginErrorReadLine();
                    await process.WaitForExitAsync(); 

                
                }
                catch (Exception ex)
                {
                    AddUpdateLog($"업데이트 실행 실패: {ex.Message}", "ERROR");
                }
            };



            // ======================
            // RPM 조작 (값만 갱신)
            // ======================
            RpmUpButton.Click += (_, __) =>
            {
                if (_rpm < 8000) _rpm += 100;
                RpmStatus.Text = $"{_rpm} RPM";
                _can.SetRpm(_rpm);
            };
            RpmDownButton.Click += (_, __) =>
            {
                if (_rpm > 0) _rpm -= 100;
                RpmStatus.Text = $"{_rpm} RPM";
                _can.SetRpm(_rpm);
            };

            // ======================
            // Speed 조작 (값만 갱신)
            // ======================
            SpeedUpButton.Click += (_, __) =>
            {
                if (_speed < 250) _speed += 5;
                SpeedStatus.Text = $"{_speed} km/h";
                _can.SetVehicleSpeed(_speed);
            };
            SpeedDownButton.Click += (_, __) =>
            {
                if (_speed > 0) _speed -= 5;
                SpeedStatus.Text = $"{_speed} km/h";
                _can.SetVehicleSpeed(_speed);
            };

            // ======================
            // Engine
            // ======================
            EngineOnButton.Click += (_, __) =>
            {
                _can.SetEngineStatus(true);
                EngineStatus.Text = "ON";
                EngineStatus.Foreground = Brushes.Green;
            };
            EngineOffButton.Click += (_, __) =>
            {
                _can.SetEngineStatus(false);
                EngineStatus.Text = "OFF";
                EngineStatus.Foreground = Brushes.Red;
            };

            // ======================
            // Tail Lamp
            // ======================
            TailLampOnButton.Click += (_, __) =>
            {
                _can.SetTailLamp(true);
                TailLampStatus.Text = "ON";
                TailLampStatus.Foreground = Brushes.Green;
            };
            TailLampOffButton.Click += (_, __) =>
            {
                _can.SetTailLamp(false);
                TailLampStatus.Text = "OFF";
                TailLampStatus.Foreground = Brushes.Red;
            };

            // ======================
            // Brake Switch
            // ======================
            BrakeOnButton.Click += (_, __) =>
            {
                _can.SetBrakeSwitch(true);
                BrakeStatus.Text = "ON";
                BrakeStatus.Foreground = Brushes.Green;
            };
            BrakeOffButton.Click += (_, __) =>
            {
                _can.SetBrakeSwitch(false);
                BrakeStatus.Text = "OFF";
                BrakeStatus.Foreground = Brushes.Red;
            };
        }

        // ======================
        // Wi-Fi 관련 함수
        // ======================
        private async Task LoadWifiList()
        {
            _wifiList.Clear();
            try
            {
                var psi = new ProcessStartInfo
                {
                    FileName = "bash",
                    ArgumentList = { "-c", "nmcli -t -f SSID dev wifi list" },
                    RedirectStandardOutput = true,
                    UseShellExecute = false
                };

                using var process = new Process { StartInfo = psi };
                process.Start();

                string output = await process.StandardOutput.ReadToEndAsync();
                process.WaitForExit();

                var lines = output.Split('\n', StringSplitOptions.RemoveEmptyEntries);
                foreach (var line in lines)
                {
                    string ssid = line.Trim();
                    if (!string.IsNullOrWhiteSpace(ssid))
                        _wifiList.Add(ssid);
                }

                if (_wifiList.Count == 0)
                    _wifiList.Add("(주변 네트워크 없음)");
            }
            catch (Exception ex)
            {
                _wifiList.Add("(검색 실패)");
                AddLog($"Wi-Fi 목록 불러오기 실패: {ex.Message}", "ERROR");
            }
        }

        private void ConnectWifi()
        {
            string ssid = WifiListBox.SelectedItem as string ?? "";
            string password = WifiPasswordBox.Text ?? "";

            if (!string.IsNullOrWhiteSpace(ssid))
            {
                try
                {
                    Process.Start(new ProcessStartInfo
                    {
                        FileName = "bash",
                        ArgumentList = { "-c", $"nmcli dev wifi connect '{ssid}' password '{password}'" },
                        UseShellExecute = false
                    });
                    AddLog($"Wi-Fi 연결 시도: {ssid}");
                }
                catch (Exception ex)
                {
                    AddLog($"Wi-Fi 연결 실패: {ex.Message}", "ERROR");
                }
            }
            else
            {
                AddLog("Wi-Fi 연결 실패: 네트워크를 선택하세요.", "WARN");
            }
        }


        // 로그 출력

        private void OnLogItemsChanged(object? sender, NotifyCollectionChangedEventArgs e)
        {
            if (e.Action != NotifyCollectionChangedAction.Add) return;


            Dispatcher.UIThread.Post(() =>
            {
                if (_logItems.Count == 0) return;


                LogList.SelectedIndex = _logItems.Count - 1;


                var sv = LogList.FindDescendantOfType<ScrollViewer>();
                if (sv != null)
                {
                    sv.ScrollToEnd();
                }
                else
                {

                    LogList.ScrollIntoView(LogList.SelectedItem);
                }
            }, DispatcherPriority.Render);
        }


        private void AddLog(string message, string level = "INFO")
        {
            var line = $"[{DateTime.Now:HH:mm:ss}] [{level}] {message}";


            Dispatcher.UIThread.Post(() =>
            {
                _logItems.Add(line);
            }, DispatcherPriority.Background);

            switch (level.ToUpper())
            {
                case "ERROR": _log.Error(message); break;
                case "WARN": _log.Warn(message); break;
                default: _log.Info(message); break;
            }
        }
        
        private void AddUpdateLog(string message, string level = "INFO")
        {
            var line = $"[{DateTime.Now:HH:mm:ss}] [{level}] {message}";
            Dispatcher.UIThread.Post(() =>
            {
                _updateLogItems.Add(line);
                UpdateLogList.SelectedIndex = _updateLogItems.Count - 1;
                UpdateLogList.ScrollIntoView(UpdateLogList.SelectedItem);
            });
        }





    }
}
