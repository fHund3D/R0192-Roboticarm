# AxisState.msg --> Die Werte vom Treiber emfangen (Canbus lesen)
uint8 node_id
float64 position            # in Radianten
float64 velocity            # in rad/s
float32 torque              # in Nm oder A
#float32 torque_setpoint     # in Nm oder A
float32 electrical_power    # in Watt
#float32 mechanical_power    # in Watt
uint32 error_code           # Für die Fehler-Diagnose der GDS-Treiber
uint32 axis_state           # Aktueller Status (Idle, Closed-loop, etc.)