param([switch]$CompileOnly)

$ErrorActionPreference = 'Stop'
Add-Type -AssemblyName System.Windows.Forms, System.Drawing

try {
    Add-Type -ReferencedAssemblies System.Windows.Forms, System.Drawing -TypeDefinition @"
using System;
using System.ComponentModel;
using System.Drawing;
using System.Runtime.InteropServices;
using System.Windows.Forms;

public sealed class WindowPin : NativeWindow, IDisposable
{
    private const int HotkeyId = 1;
    private const int WmHotkey = 0x0312;
    private const uint CtrlAltNoRepeat = 0x4003;
    private const uint CtrlAltShiftNoRepeat = 0x4007;
    private const uint KeepPositionSizeAndFocus = 0x0013;
    private readonly NotifyIcon tray;
    private readonly ContextMenuStrip menu;
    public string Shortcut { get; private set; }
    private IntPtr pinnedWindow;
    private uint pinnedProcess;

    [DllImport("user32.dll", SetLastError = true)]
    private static extern bool RegisterHotKey(IntPtr window, int id, uint modifiers, uint key);

    [DllImport("user32.dll")]
    private static extern bool UnregisterHotKey(IntPtr window, int id);

    [DllImport("user32.dll")]
    private static extern IntPtr GetForegroundWindow();

    [DllImport("user32.dll")]
    private static extern uint GetWindowThreadProcessId(IntPtr window, out uint process);

    [DllImport("user32.dll", SetLastError = true)]
    private static extern bool SetWindowPos(IntPtr window, IntPtr order, int x, int y,
        int width, int height, uint flags);

    public WindowPin()
    {
        CreateHandle(new CreateParams());
        bool primary = RegisterHotKey(Handle, HotkeyId, CtrlAltNoRepeat, (uint)Keys.T);
        bool backup = RegisterHotKey(Handle, HotkeyId + 1, CtrlAltShiftNoRepeat, (uint)Keys.T);
        if (!primary && !backup)
        {
            int error = Marshal.GetLastWin32Error();
            DestroyHandle();
            throw new Win32Exception(error, "Could not register a shortcut (Windows error " + error + ").");
        }

        Shortcut = primary ? "Ctrl+Alt+T" : "Ctrl+Alt+Shift+T";
        if (primary && backup)
            Shortcut += " / Ctrl+Alt+Shift+T";
        menu = new ContextMenuStrip();
        menu.Items.Add(Shortcut).Enabled = false;
        menu.Items.Add("Unpin window", null, (sender, args) => TryAction(Unpin));
        menu.Items.Add("Exit", null, (sender, args) => Application.ExitThread());
        tray = new NotifyIcon
        {
            Icon = SystemIcons.Application,
            Text = "Window Pin (" + Shortcut + ")",
            ContextMenuStrip = menu,
            Visible = true
        };
        tray.ShowBalloonTip(3000, "Window Pin", "Shortcut: " + Shortcut, ToolTipIcon.Info);
    }

    protected override void WndProc(ref Message message)
    {
        if (message.Msg == WmHotkey &&
            (message.WParam.ToInt32() == HotkeyId || message.WParam.ToInt32() == HotkeyId + 1))
        {
            TryAction(Toggle);
            return;
        }

        base.WndProc(ref message);
    }

    private void Toggle()
    {
        IntPtr window = GetForegroundWindow();
        uint process;
        if (GetWindowThreadProcessId(window, out process) == 0)
            return;

        bool sameWindow = window == pinnedWindow && process == pinnedProcess;
        Unpin();
        if (sameWindow)
            return;

        SetTopmost(window, true);
        pinnedWindow = window;
        pinnedProcess = process;
    }

    private void Unpin()
    {
        uint process;
        if (pinnedWindow != IntPtr.Zero &&
            GetWindowThreadProcessId(pinnedWindow, out process) != 0 &&
            process == pinnedProcess)
            SetTopmost(pinnedWindow, false);

        pinnedWindow = IntPtr.Zero;
        pinnedProcess = 0;
    }

    private static void SetTopmost(IntPtr window, bool topmost)
    {
        IntPtr order = new IntPtr(topmost ? -1 : -2);
        if (!SetWindowPos(window, order, 0, 0, 0, 0, KeepPositionSizeAndFocus))
            throw new Win32Exception(Marshal.GetLastWin32Error());
    }

    private void TryAction(Action action)
    {
        try
        {
            action();
        }
        catch (Win32Exception error)
        {
            tray.ShowBalloonTip(4000, "Window Pin", error.Message, ToolTipIcon.Error);
        }
    }

    public void Dispose()
    {
        try
        {
            Unpin();
        }
        finally
        {
            UnregisterHotKey(Handle, HotkeyId);
            UnregisterHotKey(Handle, HotkeyId + 1);
            DestroyHandle();
            tray.Dispose();
            menu.Dispose();
        }
    }
}
"@

    if ($CompileOnly) { return }

    $created = $false
    $instance = [System.Threading.Mutex]::new($true, 'Local\CodexWindowPin', [ref]$created)
    try {
        if (!$created) { return }

        $pin = [WindowPin]::new()
        try {
            Set-Content -LiteralPath (Join-Path $PSScriptRoot 'CodexWindowPin.status.txt') -Value ("Running. PID=" + $PID + ". Shortcut=" + $pin.Shortcut)
            [System.Windows.Forms.Application]::Run()
        } finally {
            $pin.Dispose()
        }
    } finally {
        if ($created) { $instance.ReleaseMutex() }
        $instance.Dispose()
    }
} catch {
    if ($CompileOnly) { throw }
    [void][System.Windows.Forms.MessageBox]::Show($_.Exception.Message, 'Window Pin')
    exit 1
}
