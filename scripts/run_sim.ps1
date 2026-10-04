# run_sim.ps1 - watch the factory hall mission with windows, from Windows.
#
#   1. restart WSL until WSLg's shared memory comes up healthy (otherwise every
#      Linux window shows blank with "[WARN:COPY MODE]" in its title)
#   2. start the run detached (its processes keep the WSL session alive)
#   3. wait for the Gazebo and computer-vision windows and arrange them
#   4. release take-off (or leave it held with -Hold)
#
# Usage:  powershell -ExecutionPolicy Bypass -File run_sim.ps1 [-Hold]
param([switch]$Hold)
$distro = "Ubuntu-22.04"

Add-Type @"
using System;using System.Text;using System.Runtime.InteropServices;
public class Win{
 [DllImport("user32.dll")] public static extern bool EnumWindows(EnumWindowsProc cb,IntPtr l);
 public delegate bool EnumWindowsProc(IntPtr h,IntPtr l);
 [DllImport("user32.dll")] public static extern int GetWindowTextLength(IntPtr h);
 [DllImport("user32.dll")] public static extern int GetWindowText(IntPtr h,StringBuilder s,int c);
 [DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr h);
 [DllImport("user32.dll")] public static extern bool ShowWindow(IntPtr h,int n);
 [DllImport("user32.dll")] public static extern bool SetWindowPos(IntPtr h,IntPtr a,int x,int y,int cx,int cy,uint f);
}
"@
function Find-Window($pattern) {
  $script:found = [IntPtr]::Zero; $script:title = ""
  $cb = [Win+EnumWindowsProc]{ param($w,$l)
    $n = [Win]::GetWindowTextLength($w)
    if ($n -gt 0) { $sb = New-Object Text.StringBuilder($n+1); [void][Win]::GetWindowText($w,$sb,$sb.Capacity)
      if ($sb.ToString() -match $pattern) { $script:found = $w; $script:title = $sb.ToString() } }
    return $true }
  [void][Win]::EnumWindows($cb,[IntPtr]::Zero)
  return $script:found
}

Write-Host "1/4  getting a healthy WSL display session..."
$ok = $false
for ($i = 1; $i -le 6 -and -not $ok; $i++) {
  wsl --shutdown; Start-Sleep 5
  wsl -d $distro --cd / -- bash -c "sleep 6" | Out-Null
  $bad = wsl -d $distro --cd / -- bash -c "grep -ac 'Failed to open' /mnt/wslg/weston.log"
  if ([int]$bad -eq 0) { $ok = $true; Write-Host "     boot $($i) healthy" } else { Write-Host "     boot $($i): shared memory failed, retrying" }
}
if (-not $ok) { Write-Host "No healthy WSL session in 6 tries. Restart Windows and try again."; exit 1 }

Write-Host "2/4  starting the simulation (take-off held)"
wsl -d $distro --cd ~ -- bash -lc "rm -f /tmp/drone_go; GUI=1 VIEW=1 HOLD=1 setsid nohup bash ~/drone_sim/scripts/hall_run.sh </dev/null >/tmp/hall_run.out 2>&1 & sleep 1"

Write-Host "3/4  waiting for the windows..."
$gz = [IntPtr]::Zero; $cv = [IntPtr]::Zero
for ($t = 0; $t -lt 180 -and ($gz -eq [IntPtr]::Zero -or $cv -eq [IntPtr]::Zero); $t += 2) {
  Start-Sleep 2
  $gz = Find-Window 'Gazebo Sim'
  $cv = Find-Window 'Cloud computer vision'
}
if ($gz -eq [IntPtr]::Zero) { Write-Host "Gazebo window did not appear. Log: wsl -d $distro -- tail /tmp/hall_run.out"; exit 1 }
# Gazebo on the left two-thirds, the camera view on the right
[void][Win]::ShowWindow($gz,9); [void][Win]::SetWindowPos($gz,[IntPtr]::Zero,0,0,960,860,0x0040)
if ($cv -ne [IntPtr]::Zero) {
  [void][Win]::ShowWindow($cv,9); [void][Win]::SetWindowPos($cv,[IntPtr]-1,960,0,480,300,0x0040)
} else { Write-Host "     (computer-vision window not found yet - it appears once frames arrive)" }
[void][Win]::SetForegroundWindow($gz)
Write-Host "     windows up: $($script:title)"

if ($Hold) {
  Write-Host "4/4  take-off is HELD. Release it with:  wsl -d $distro -- touch /tmp/drone_go"
} else {
  Start-Sleep 3
  wsl -d $distro --cd / -- touch /tmp/drone_go
  Write-Host "4/4  take-off released - watch the Gazebo window"
}
