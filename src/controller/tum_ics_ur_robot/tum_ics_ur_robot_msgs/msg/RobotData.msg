#ROS HEADER
Header header

float64 time

#Joint Positions/Velocities/Accelerations
float64[6] q
float64[6] qp
float64[6] qpp

#Joint Positions/Velocities/Accelerations Filttered
float64[6] qf
float64[6] qpf
float64[6] qppf


#End Effector
float64[6] Xef_0
float64[6] Xefp_0

#End Effector Filttered
float64[6] Xeff_0
float64[6] Xefpf_0

#Torques
float64[6] torques


