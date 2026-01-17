import sensor
import image
import pyb

# Camera setup
sensor.reset()
sensor.set_framesize(sensor.XGA)
sensor.set_pixformat(sensor.RGB565)
sensor.set_vflip(True)
sensor.set_hmirror(True)

# UART3 uses pins P4 (TX) and P5 (RX)
uart = pyb.UART(3, 115200, timeout_char=10)

# Variables
x, y, z = 0, 0, 0
last_msg = "Waiting..."

while True:
    img = sensor.snapshot()

    # Check for incoming data
    if uart.any():
        try:
            line = uart.readline()
            if line:
                decoded = line.decode().strip()
                last_msg = decoded

                parts = decoded.split(',')
                if len(parts) == 3:
                    x = float(parts[0])
                    y = float(parts[1])
                    z = float(parts[2])
                    print(f"Received: x={x}, y={y}, z={z}")
        except Exception as e:
            print("Error:", e)

    # Draw on frame
    img.draw_rectangle(5, 5, 200, 70, color=(0, 0, 0), fill=True)
    img.draw_string(8, 7, "Current Pos", color=(255, 255, 0), scale=2)
    img.draw_string(8, 25, f"X:{x} Y:{y} Z:{z}", color=(0, 255, 0), scale=2)
    img.draw_string(8, 38, f"Raw:{last_msg}", color=(255, 255, 255), scale=2)
