import serial
import serial.tools.list_ports
import sys

print("=== 所有可用串口 ===")
ports = list(serial.tools.list_ports.comports())
for port in ports:
    print(f"  {port.device}: {port.description}")
    print(f"    VID/PID: {port.vid:04X}/{port.pid:04X}" if port.vid else "    VID/PID: N/A")
    print(f"    位置: {port.hwid}")

print()
print("=== 检查 COM20 ===")
com20 = serial.tools.list_ports.grep("COM20")
try:
    for port in com20:
        print(f"找到: {port.device}")
        print(f"  描述: {port.description}")
        print(f"  VID: {port.vid:04X}" if port.vid else "  VID: N/A")
        print(f"  PID: {port.pid:04X}" if port.pid else "  PID: N/A")
        print(f"  硬件ID: {port.hwid}")
except Exception as e:
    print(f"  未找到 COM20: {e}")

print()
print("=== 尝试打开测试 ===")
try:
    ser = serial.Serial("COM20", 115200, timeout=1)
    print("  [OK] 成功打开 COM20!")
    ser.close()
except Exception as e:
    print(f"  [ERROR] 打开失败: {e}")
