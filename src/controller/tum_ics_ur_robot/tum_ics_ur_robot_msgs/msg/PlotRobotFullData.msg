#ROS HEADER
Header header

#Time elapsed since the controller was started
float64     robotTime

#Target joint positions
float64[6]   qTar

#Target joint Velocities
float64[6]   qdTar

#Target joint accelerations
float64[6]   qddTar

#Target joint currents
float64[6]   ITar

#Target joint moments
float64[6]   MTar

#Actual joint positions
float64[6]   qAct

#Actual joint velocities
float64[6]   qdAct

#Actual joint currents
float64[6]   IAct

#Tool x,y and z accelerometer values
float64[3]   TAV

#Generalised forces in the TCP
float64[6]   tcpForce

#Cartesian coordinates of the tool: (x,y,z,rx,ry,rz), 
#where rx, ry and rz is a rotation vector representation of the tool
#orientation
float64[6]   tcpVect

#Speed of the tool given in cartesian coordinates
float64[6]   tcpSpeed





