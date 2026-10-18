//go:build windows

// Cubli 平衡控制上位机启动器。
//
// 以 pythonw.exe（无控制台）启动与本程序同目录的 main.py：
//   - 编译为 GUI 子系统（-H=windowsgui），双击不出现黑色终端窗口；
//   - 按顺序探测解释器，选择第一个能 import PyQt5/pyqtgraph/serial 的；
//   - 单实例互斥：界面未退出前再次双击只提示，不会启动第二个窗口；
//   - 启动失败或启动后 10 秒内异常退出时弹窗提示，并把子进程输出写入
//     %LOCALAPPDATA%\CubliUpperComputer\gui_stderr.log 便于排查；
//   - 转发命令行参数，例如 Cubli上位机.exe --port COM9。
package main

import (
	"context"
	"os"
	"os/exec"
	"path/filepath"
	"sort"
	"strings"
	"syscall"
	"time"
	"unsafe"
)

const (
	createNoWindow = 0x08000000
	errorExists    = 183 // ERROR_ALREADY_EXISTS
	mutexName      = "CubliUpperComputerSingleInstance"
	windowTitle    = "Cubli 平衡控制上位机"
	depsCheck      = "import PyQt5.QtWidgets, serial, pyqtgraph"
)

var (
	kernel32                = syscall.NewLazyDLL("kernel32.dll")
	user32                  = syscall.NewLazyDLL("user32.dll")
	procCreateMutexW        = kernel32.NewProc("CreateMutexW")
	procMessageBoxW         = user32.NewProc("MessageBoxW")
	procFindWindowW         = user32.NewProc("FindWindowW")
	procShowWindow          = user32.NewProc("ShowWindow")
	procSetForegroundWindow = user32.NewProc("SetForegroundWindow")
)

func messageBox(text string) {
	title, _ := syscall.UTF16PtrFromString(windowTitle)
	body, _ := syscall.UTF16PtrFromString(text)
	procMessageBoxW.Call(0, uintptr(unsafe.Pointer(body)), uintptr(unsafe.Pointer(title)), 0x10)
}

// activateExisting 找到已在运行的主窗口并前置；找不到时返回 false。
func activateExisting() bool {
	title, _ := syscall.UTF16PtrFromString(windowTitle)
	hwnd, _, _ := procFindWindowW.Call(0, uintptr(unsafe.Pointer(title)))
	if hwnd == 0 {
		return false
	}
	procShowWindow.Call(hwnd, 9) // SW_RESTORE
	procSetForegroundWindow.Call(hwnd)
	return true
}

// childAttr 注意：不能使用 HideWindow（会设置 STARTUPINFO SW_HIDE，
// 导致子进程 Qt 主窗口的首次 ShowWindow 被强制隐藏）；pythonw 本身无控制台，
// python.exe 回退路径由 CREATE_NO_WINDOW 保证不出现黑色窗口。
func childAttr() *syscall.SysProcAttr {
	return &syscall.SysProcAttr{CreationFlags: createNoWindow}
}

// acquireSingleInstance 返回 false 表示已有实例在运行。
func acquireSingleInstance() bool {
	name, _ := syscall.UTF16PtrFromString(mutexName)
	handle, _, err := procCreateMutexW.Call(0, 0, uintptr(unsafe.Pointer(name)))
	if handle == 0 {
		return true // 极少数创建失败：不阻止启动
	}
	if errno, ok := err.(syscall.Errno); ok && errno == errorExists {
		return false
	}
	_ = handle // 进程存活期间保持句柄打开
	return true
}

// interpreterCandidates 按优先级列出可能的 pythonw/python 解释器。
func interpreterCandidates() []string {
	var found []string
	add := func(path string) {
		if path == "" {
			return
		}
		if info, err := os.Stat(path); err != nil || info.IsDir() {
			return
		}
		for _, existing := range found {
			if strings.EqualFold(existing, path) {
				return
			}
		}
		found = append(found, path)
	}
	add(os.Getenv("CUBLI_PYTHON"))
	if path, err := exec.LookPath("pythonw.exe"); err == nil {
		add(path)
	}
	if local := os.Getenv("LOCALAPPDATA"); local != "" {
		matches, _ := filepath.Glob(filepath.Join(local, "Programs", "Python", "Python3*", "pythonw.exe"))
		sort.Sort(sort.Reverse(sort.StringSlice(matches)))
		for _, match := range matches {
			add(match)
		}
	}
	if path, err := exec.LookPath("pyw.exe"); err == nil {
		add(path)
	}
	if path, err := exec.LookPath("python.exe"); err == nil {
		add(path)
	}
	return found
}

func hasDependencies(interpreter string) bool {
	ctx, cancel := context.WithTimeout(context.Background(), 20*time.Second)
	defer cancel()
	cmd := exec.CommandContext(ctx, interpreter, "-c", depsCheck)
	cmd.SysProcAttr = childAttr()
	return cmd.Run() == nil
}

func logTail(path string, maxBytes int) string {
	data, err := os.ReadFile(path)
	if err != nil || len(data) == 0 {
		return ""
	}
	if len(data) > maxBytes {
		data = data[len(data)-maxBytes:]
	}
	return strings.ToValidUTF8(string(data), "")
}

func main() {
	if !acquireSingleInstance() {
		if !activateExisting() {
			messageBox("上位机已经在运行，请查看已打开的窗口。\n（窗口可能被其他程序挡住或最小化在任务栏。）")
		}
		return
	}
	exePath, err := os.Executable()
	if err != nil {
		messageBox("无法确定程序位置：" + err.Error())
		return
	}
	dir := filepath.Dir(exePath)
	script := filepath.Join(dir, "main.py")
	if _, err := os.Stat(script); err != nil {
		messageBox("未找到 main.py，请确认本程序与 main.py 位于同一目录：\n" + dir)
		return
	}

	candidates := interpreterCandidates()
	interpreter := ""
	for _, candidate := range candidates {
		if hasDependencies(candidate) {
			interpreter = candidate
			break
		}
	}
	if interpreter == "" {
		if len(candidates) == 0 {
			messageBox("未找到 Python。请先安装 Python 3（勾选加入 PATH），\n然后在 Software 目录执行：\npip install -r requirements.txt")
		} else {
			messageBox("已找到 Python，但缺少 PyQt5/pyqtgraph/pyserial 依赖。\n请在 Software 目录执行：\npip install -r requirements.txt\n\n使用的解释器：" + candidates[0])
		}
		return
	}

	logDir := os.Getenv("LOCALAPPDATA")
	if logDir == "" {
		logDir = os.TempDir()
	}
	logDir = filepath.Join(logDir, "CubliUpperComputer")
	_ = os.MkdirAll(logDir, 0o755)
	logPath := filepath.Join(logDir, "gui_stderr.log")
	logFile, logErr := os.Create(logPath)
	if logErr == nil {
		_, _ = logFile.WriteString(time.Now().Format("2006-01-02 15:04:05") +
			"  interpreter=" + interpreter + "  args=" + strings.Join(os.Args[1:], " ") + "\r\n")
	}

	cmd := exec.Command(interpreter, append([]string{script}, os.Args[1:]...)...)
	cmd.Dir = dir
	cmd.SysProcAttr = childAttr()
	if logErr == nil {
		cmd.Stdout = logFile
		cmd.Stderr = logFile
	}
	started := time.Now()
	if err := cmd.Start(); err != nil {
		if logErr == nil {
			logFile.Close()
		}
		messageBox("启动失败：" + err.Error())
		return
	}
	if logErr == nil {
		logFile.Close() // 子进程已持有自己的句柄
	}
	waitErr := cmd.Wait() // 保持单实例互斥直到界面退出
	if waitErr != nil && time.Since(started) < 10*time.Second {
		tail := logTail(logPath, 1500)
		if strings.TrimSpace(tail) == "" {
			tail = "（日志为空）"
		}
		messageBox("上位机启动后很快退出：" + waitErr.Error() + "\n\n日志末尾：\n" + tail + "\n\n完整日志：" + logPath)
	}
}
