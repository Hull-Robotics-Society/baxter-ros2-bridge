Header header

float32 voltage   # (V)
float32 rate      # (A) Negative when discharging
float32 charge    # (Ah)
float32 capacity  # (Ah)
uint8 percent     # 0-100
uint8 state       # Enum

uint8 BATTERY_CHARGED=0
uint8 BATTERY_CHARGING=1
uint8 BATTERY_DISCHARGING=2
uint8 BATTERY_WARN=3
uint8 BATTERY_LIMP_HOME=4
uint8 BATTERY_CRITICAL=5
