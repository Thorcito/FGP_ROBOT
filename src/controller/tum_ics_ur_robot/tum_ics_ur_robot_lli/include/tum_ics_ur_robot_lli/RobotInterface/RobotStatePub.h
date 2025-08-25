/* Robot State Publisher class
 * Authors: Emmanuel Dean and Florian Bergner
 * email: dean@tum.de, florian.bergner@tum.de
 *
 * v1: 2015
 */
#ifndef ROBOTSTATEPUB_H
#define ROBOTSTATEPUB_H


#include <ros/ros.h>



#include<QThread>
#include<QString>
#include<QVector>

#include <tum_ics_ur_robot_lli/JointState.h>
#include<QMutex>

namespace tum_ics_ur_robot_lli{
namespace RobotInterface{


/*!
 * \brief The RobotStatePub class This class provides a thread that publishes the robot joint state. This JointState
 * message is used by rviz to visualize the robot.
 */
class RobotStatePub: QThread
{

    //Variables
public:
    typedef QVector<QString> VQString;

private:
    /*!
     * \brief m_topicName topic name where the joint state message will be published.
     */
    QString m_topicName;

    /*!
     * \brief m_jointNames list of joint names, used in the JointState message
     */
    VQString m_jointNames;

    /*!
     * \brief m_n ros node handler
     */
    ros::NodeHandle m_n;

    /*!
     * \brief m_jointStatePub Robot JointState publisher.
     */
    ros::Publisher m_jointStatePub;

    /*!
     * \brief m_stop flag to stop the internal thread.
     */
    bool m_stop;

    /*!
     * \brief m_jointState Joint state data container.
     */
    JointState m_jointState;

    /*!
     * \brief m_mutex mutex to protect the writing/reading process.
     */
    QMutex m_mutex;

    /*!
     * \brief m_updateRate frequency when the JoitState message will be published.
     */
    double m_updateRate;



    //Functions
public:
    /*!
     * \brief RobotStatePub Default constructor. This constructor initializes the JointState publisher.
     * \param topicName name of the topic.
     * \param jointNames list of joint names for the JointState message.
     * \param updateRate frequency of the publisher.
     */
    RobotStatePub(const QString& topicName, const VQString& jointNames, double updateRate=60.0);
    ~RobotStatePub();

    /*!
     * \brief start starts the internal thread.
     */
    void start();

    /*!
     * \brief stop changes the flag m_stop to stop the internal thread.
     */
    void stop();

    /*!
     * \brief setJointState sets the joint state variable which will be published.
     * \param in desired joint state.
     */
    void setJointState(const JointState& in);


private:

    /*!
     * \brief run callback function of the internal thread. This function transforms the joint state
     * to a JointState message and publish it.
     */
    void run();


};






}
}


#endif // ROBOTSTATEPUB_H
