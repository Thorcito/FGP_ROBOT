#ROS HEADER
Header header

float64 time

#Computation times

float64 QP_time
float64 skin_time
float64 total_time

#Number of activated cells per rigid body

float64 N_cells1
float64 N_cells2
float64 N_cells3
float64 N_cells4
float64 N_cells5
float64 N_cells6

#Force barycenter coordinates for each rigid body in the base reference frame

float64[3] t_bary1
float64[3] t_bary2
float64[3] t_bary3
float64[3] t_bary4
float64[3] t_bary5
float64[3] t_bary6

#Applied forces, measured by the skin on the force barycenter of each rigid body

float64[6] W_bary1
float64[6] W_bary2
float64[6] W_bary3
float64[6] W_bary4
float64[6] W_bary5
float64[6] W_bary6

#Resulting desired accelerations in Cartesian space

float64[6] Xppd1
float64[6] Xppd2
float64[6] Xppd3
float64[6] Xppd4
float64[6] Xppd5
float64[6] Xppd6

#Accelerations of each joint in Cartesian space
float64[6] Xpp1
float64[6] Xpp2
float64[6] Xpp3
float64[6] Xpp4
float64[6] Xpp5
float64[6] Xpp6

#Torques
float64[6] torques
