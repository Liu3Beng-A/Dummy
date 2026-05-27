import serial
import serial.tools.list_ports
import sys
import time
import os

# 强制刷新输出
sys.stdout = os.fdopen(sys.stdout.fileno(), 'w', buffering=1)

def list_all_ports():
    ports = list(serial.tools.list_ports.comports())
    print("=== Ports ===")
    for port in ports:
        print(port.device)
    print()

def main():
    print("Starting...")
    sys.stdout.flush()
    
    list_all_ports()
    
    port = "COM20"
    baudrate = 115200
    print(f"Watching {port} @ {baudrate}")
    sys.stdout.flush()
    
    while True:
        try:
            print("Trying to connect...")
            sys.stdout.flush()
            ser = serial.Serial(port, baudrate, timeout=1)
            print("Connected!")
            sys.stdout.flush()
            ser.flushInput()
            
            while True:
                if ser.in_waiting > 0:
                    data = ser.read(ser.in_waiting)
                    text = data.decode('utf-8', errors='replace')
                    for line in text.split('\n'):
                        line = line.strip()
                        if line:
                            print(line)
                            sys.stdout.flush()
                time.sleep(0.01)
                
        except serial.SerialException as e:
            print(f"Disconnected: {e}")
            sys.stdout.flush()
            time.sleep(2)
            
        except KeyboardInterrupt:
            print("\nExit")
            break

if __name__ == "__main__":
    main()
