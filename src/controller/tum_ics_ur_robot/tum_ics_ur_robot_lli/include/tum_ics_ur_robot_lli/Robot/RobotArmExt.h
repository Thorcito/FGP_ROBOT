/* Robot Arm External class for the UR3 robot
 * Authors: Emmanuel Dean
 * email: dean@tum.de
 *
 * v1: 2019
 */


#ifndef ROBOTARMCONSTRAINED_H
#define ROBOTARMCONSTRAINED_H

#include <tum_ics_ur_robot_lli/Robot/RobotArm.h>
#include <tum_ics_ur_robot_lli/RobotControllers/Controller.h>
#include <tum_ics_ur_robot_lli/RobotInterface/RobotStateClient.h>

namespace tum_ics_ur_robot_lli{
namespace Robot{
/*!
 * \brief The RobotArmExt class This class is a wrapper for the main RobotArm class.
 * The main difference is that creates an RobotStateClient externally
 * and pass it as an argument to the robotArm class.
 * This is needed for the UR3 only, we need to further analyse why!
 */
class RobotArmExt
{

private:

    /*!
     * \brief m_jointNames This vector stores the joint_names needed to generate the topic "joint_states" used in rviz
     */
    QVector<QString> m_jointNames;

    /*!
     * \brief m_topicName The name of the joint_state topic published by the robotArm class
     */
    QString m_topicName;

    /*!
     * \brief m_robotIpAddr Robot IP Address.
     */
    QString m_robotIpAddr;

    /*!
     * \brief m_robotVersion Polyscope version installed in the control unit. Currently, the class supports
     * the versions: 1.8, 3.1, 3.4, 3.5 We need to match the exact Polyscope version since
     * the robotArm class uses the Remote Control via TCP/IP real-time time port 30003.
     * In theory, for versions 3.5 and up, the packet description is the same, then we should be
     * able to use version 3.5 for 3.5 and above.
     */
    RobotInterface::RobotStateClient::Version m_robotVersion;

    /*!
     * \brief m_robotStateClient Object of RobotStateClient, this class creates the communication sockets and gets/set information
     * from and to the robot. For the UR3 robot, this instance must be created externally and pass it as
     * an argument to the main RobotArm class.
     */
    RobotInterface::RobotStateClient* m_robotStateClient;

    /*!
     * \brief m_configFilePath file name with absolute path for the config.ini file, containing the
     * robot configuration parameters
     */
    QString m_configFilePath;

    /*!
     * \brief m_errorString In case of communication/parsing errors, the text description of the error will be stored in this variable
     */
    QString m_errorString;

    /*!
     * \brief m_error True with any communication/parsing error.
     */
    bool m_error;

    /*!
     * \brief m_robotType real or simulated robot
     */
    bool m_realRobot;

public:
    /*!
     * \brief RobotArmExt Default constructor. This constructor will parse the config.ini file and
     * creates a RobotStateClient object. The constructor waits until the robotStateClient is
     * generating reliable information and instantiates a RobotArm object with this external
     * RobotStateClient.
     * \param configFilePath robot configuration file with absolute path (config.ini).
     */
    RobotArmExt(const QString &configFilePath);

    ~RobotArmExt();

    /*!
     * \brief m_robot main RobotArm class object. This object contains all the methods, models, and variables
     * to communicate and work with a UR robot.
     */
    RobotArm* m_robot;

    /*!
     * \brief add This function creates a Qlist of controllers where the first appended controller will define the CtrlInterface
     * (e.g. VELOCITY_INTF, EFFORT_INTF, etc.)
     * \param u controller that has to be added to the robotArm.
     * \return  true = if the controller was succsessfully added to the list of controllers.
     */
    bool add(RobotControllers::Controller *u);

    /*!
     * \brief init This is a really important function, and must be runned before starting to use the robot arm
     * The function parses the robot configuration file and sets the local variables, initializes the kinematic model,
     * the dynamic model, loads the internal PID controller gains, and if "real" robot is used, it starts the
     * communication with the real robot, both the state machine in the script running in the robot control unit, and
     * with the real-time joint state socket (30003). It also starts the JointState publisher.
     * \return true if all the initializations where succsessful.
     */
    bool init();


    void start();

    /*!
     * \brief stop Send the signal to stop the state machine running in the script inside the robot control unit and
     * stops the main thread.
     * \return  true
     */
    bool stop();

    /*!
     * \brief qHome gets the Joint State for QHOME defined in the robotArm.
     * \return  joint State for the Home position.
     */
    JointState qHome() const;

    /*!
     * \brief qPark gets the Joint State for QPARK defined in the robotArm.
     * \return  joint State for the Park position.
     */
    JointState qPark() const;

    /*!
     * \brief parseConfig function to parse the robot configuration file (config.ini) and store the parameters in
     * the local variables.
     * \return
     */
    bool parseConfig();

    /*!
     * \brief error access method for the error flag
     * \return it returns the error flag
     */
    bool error() const;

    /*!
     * \brief errorString gets the error description in a Qstring.
     * \return Qstring with the error description.
     */
    const QString& errorString() const;

    /*!
     * \brief isRunning gets the flag to verify if the main thread is still running.
     * \return
     */
    const bool isRunning() const;


};

}


}

#endif // ROBOTARMCONSTRAINED_H
