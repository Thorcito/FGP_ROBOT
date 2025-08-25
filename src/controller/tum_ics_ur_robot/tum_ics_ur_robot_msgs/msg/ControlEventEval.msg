#ROS HEADER
Header header

float64 time

int64 jointEventCnt
int64 forceEventCnt

float64 deltaForce
int64 activeCells

# cells indexed as in yaml file
float64[] extForce

float64[] prox
float64[] force1
float64[] force2
float64[] force3
float64[] totalForce

float64[6] q
float64[6] qp
float64[6] qpp

float64[6] skinTau
float64[6] tau
