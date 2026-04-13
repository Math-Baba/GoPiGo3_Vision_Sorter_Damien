import board, busio, time, math
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR

print("Init I2C...")
i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)
time.sleep(3)

print("Init BNO085...")
bno = BNO08X_I2C(i2c, address=0x4b)
time.sleep(2)

print("Enable rotation vector...")
bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
time.sleep(2)

print("Tourne le robot sur le sol!")
print("YAW = rotation horizontale si capteur a plat")
print("PITCH ou ROLL = rotation horizontale si capteur vertical")
print()

for i in range(120):
    try:
        quat = bno.quaternion
        if quat:
            w, x, y, z = quat
            yaw = math.degrees(math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))
            pitch = math.degrees(math.asin(max(-1,min(1,2*(w*y-z*x)))))
            roll = math.degrees(math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)))
            print(f'YAW={yaw:7.1f}  PITCH={pitch:7.1f}  ROLL={roll:7.1f}')
    except Exception as e:
        pass
    time.sleep(0.3)