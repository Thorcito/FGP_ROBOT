/* Robot State Client class
 * Authors: Emmanuel Dean and Florian Bergner
 * email: dean@tum.de, florian.bergner@tum.de
 *
 * v1: 2015
 */

#ifndef UR_ROBOT_LLI_STATE_H
#define UR_ROBOT_LLI_STATE_H

#include<tumtools/Math/EigenDefs.h>

#include<sensor_msgs/JointState.h>
#include<QString>
#include<QVector>



namespace tum_ics_ur_robot_lli{

using namespace Tum;
/*!
 * \brief The JointState class This is a container class for the joint states of the robot.
 * The class can contain the joint positions, velocities, and torques.
 */
class JointState
{ //TODO: add filtered signals
    // Member variables
public: 
    /*!
     * \brief q joint position of the virtual robot.
     */
    VectorDOFd q; //q(t)
    /*!
     * \brief qp joint velocity of the virtual robot.
     */
    VectorDOFd qp;
    /*!
     * \brief qpp joint acceleration of the virtual robot.
     */
    VectorDOFd qpp;
    /*!
     * \brief tau joint torques computed with the controllers added to the main RobotArm class.
     */
    VectorDOFd tau;

    /*!
     * \brief qReal current joint position received from the real robot.
     */
    VectorDOFd qReal;
    /*!
     * \brief qpReal current joint velocities received from the real robot.
     */
    VectorDOFd qpReal;
    /*!
     * \brief qppReal set to zero since the real robot doesn't provide current joint accelerations.
     */
    VectorDOFd qppReal;
    /*!
     * \brief tauReal current joint torque received from the real robot (motor currents [A]).
     */
    VectorDOFd tauReal;

    typedef QVector<QString> VQString;

private:

    // Member Methods
public:
    /*!
     * \brief JointState Default constructor. It initializes all the states to zero.
     */
    JointState();

    /*!
     * \brief JointState Copy constructor.
     * \param s
     */
    JointState(const JointState &s);
    ~JointState();

    /*!
     * \brief operator -= Substraction operator, it computes (this - other) operation.
     * \param other JointState object to be substracted from this object.
     * \return in returns the substracted JointState
     */
    JointState& operator -=(const JointState& other);

    /*!
     * \brief operator += Addition operator, it computes (this + other) operation.
     * \param other JointState object to be added to this object.
     * \return in returns the added JointState
     */
    JointState& operator +=(const JointState& other);

    /*!
     * \brief toString Transforms the JointState into a string sequence.
     * \return It returns a string with all the joint state vectors
     * with the sequence q,qp,qpp,tau,qReal,qpReal,qppReal, tauReal.
     */
    QString toString() const;

    /*!
     * \brief toJointStateMsg Transforms a JointState object to sensor_msgs::JointState
     * \param jointNames Vector of QStrings with the joint names needed for the message JointState
     * \param t time needed to generate the message JointState
     * \return it returns a sensor_msgs::JointState that can be used to publish q,qp,tau.
     * The message contains the virtual robot state only.
     */
    sensor_msgs::JointState toJointStateMsg(const VQString& jointNames, const ros::Time &t = ros::Time::now()) const;

    /*!
     * \brief updateMsg This function only updates the jointStates (q,qp,tau) in the message without
     * changing the joint names. Usually, this function should be used after calling at least one time
     * the function toJointStateMSg
     * \param msg The JointState message previously generated with --toJointStateMsg-- function.
     * \param t current time used in the header of the message
     * \return it returns the updated JointState message, only for the virtual robot state (q,qp,tau).
     */
    sensor_msgs::JointState& updateMsg(sensor_msgs::JointState& msg, const ros::Time &t = ros::Time::now()) const;

    /*!
     * \brief initQ Initializes this JointState object with the joint positions (q,qReal) from other,
     * and sets qp, qpp, tau, qpReal, qppReal, and tauReal to zero.
     * \param other
     */
    void initQ(const JointState& other);//gets other.q and sets qp, qpp and tau to Zero()

    /*!
     * \brief Zero Sets this JointState object to zero (all member variables).
     */
    void Zero();

    // Member Methods
private:

};



}






#endif // UR_ROBOT_LLI_STATE_H
