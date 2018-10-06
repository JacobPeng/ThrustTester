# Protocol

## raw data

Voltage(V) | Current(A) | Thrust(g)  | Throttle(%) |  RPM(RPM)  | T.E.1(g/W) 
---------- | ---------- | ---------- | ----------- | ---------- | ---------- 
float, %.1f|float, %.2f |int16_t, %d | uint8_t, %d |uint16_t, %d|float, %.2f
T.E.2(g/A) | X Vibr(g0) | Y Vibr(g0) | Z Vibr(g0)  |  Power(W)  |
float, %.1f|float, %.2f |float, %.2f | float, %.2f |float, %.1f |

电压(V)  | 电流(A)  | 推力(g)  | 油门(%)  | 转速(RPM) | 力效1(g/W)
-------- |---------|----------|---------|-----------|-----------
力效2(g/A)|X振动(g0)| Y振动(g0)| Z振动(g0)| 功率(W)   |

