#ROS HEADER
Header header

float64 time

#Joint Positions/Velocities/Accelerations
float64[6] q
float64[6] qp
float64[6] qpp

#Joint Desired Positions/Velocities
float64[6] qd
float64[6] qpd
float64[6] qppd

#Joint Errors Position/Velocities
float64[6] Dq
float64[6] Dqp

#End Effector
float64[6] Xef_0
float64[6] Xd_0
float64[6] DX
float64[6] DXp


#Torques
float64[6] torques


