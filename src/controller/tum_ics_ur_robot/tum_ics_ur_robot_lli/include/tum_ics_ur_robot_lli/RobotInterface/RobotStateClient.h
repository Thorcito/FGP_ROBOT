/* Robot State Client class
 * Authors: Emmanuel Dean and Florian Bergner
 * email: dean@tum.de, florian.bergner@tum.de
 *
 * v1: 2015
 */

#ifndef ROBOTSTATECLIENT_H
#define ROBOTSTATECLIENT_H

#include<QString>
#include<QTcpSocket>

#include<tum_ics_ur_robot_lli/RobotInterface/TcpSocketTools.h>
#include<tum_ics_ur_robot_lli/JointState.h>

namespace tum_ics_ur_robot_lli{
namespace RobotInterface{


/*!
 * \brief The RobotStateClient class This class creates the connection to the real-time port in the robot 30003 and
 * unpacks the information of the robot. The class provides useful access functions to get the real robot
 * joint state. Currently, this class can handle the polyscope versions: 1.8, 3.1, 3.4, and 3.5.
 * For versions above 3.5 the VERSION_3V5 should be used (and should work?).
 */
class RobotStateClient: TcpSocketTools, QTcpSocket
{
public:

    /*!
     * \brief The Version enum Polyscope versions available in this class
     */
    enum Version
    {
        VERSION_1V8 = 0,
        VERSION_3V1,
        VERSION_3V4,
        VERSION_3V5

    };

    // v1.8
    /*!
     * \brief The RobotStateMsgV1V8 struct Structure to store the packet received from the real-time port for Polyscope version 1.8.
     * The real-time interface packet description has been taken from:
     * https://www.universal-robots.com/how-tos-and-faqs/how-to/ur-how-tos/remote-control-via-tcpip-16496/
     * see: tum_ics_ur_robot/tum_ics_ur_robot_docs/Client_Interface_V3.8andV5.2.xlsx
     */
    struct RobotStateMsgV1V8
    {
        int         msgSize;        //      Message Size            integer	1		4
        double      time;           //      Time                    double	1		8
        VectorDOFd  qTar;           //      q target                double	6		48
        VectorDOFd  qpTar;          //      qd target               double	6		48
        VectorDOFd  qppTar;         //      qdd target              double	6		48
        VectorDOFd  iTar;           //      I target                double	6		48
        VectorDOFd  mTar;           //      M target                double	6		48
        VectorDOFd  qAct;           //      q actual                double	6		48
        VectorDOFd  qpAct;          //      qd actual               double	6		48
        VectorDOFd  iAct;           //      I actual                double	6		48
        VectorDOFd  iControl;       //      I control               double	6		48
        Vector6d    vtoolAct;       //      Tool vector actual      double	6		48
        Vector6d    tcpSpeedAct;    //      TCP speed actual        double	6		48
        Vector6d    tcpForce;       //      TCP force               double	6		48
        Vector6d    vtoolTar;       //      Tool vector target      double	6		48
        Vector6d    tcpSpeedTar;    //      TCP speed target        double	6		48
        double      dio;            //      Digital input bits      double	1		8
        VectorDOFd  motorTemp;      //      Motor temperatures      double	6		48
        double      ctrlTimer;      //      Controller Timer        double	1		8
        double      testVal;        //      Test value              double	1		8
        double      robotMode;      //      Robot Mode              double	1		8
        Vector6d    jointModes;     //      Joint Modes             double	6		48
    };

    // v3.1
    /*!
     * \brief The RobotStateMsgV1V8 struct Structure to store the packet received from the real-time port for Polyscope version 3.1.
     * The real-time interface packet description has been taken from:
     * https://www.universal-robots.com/how-tos-and-faqs/how-to/ur-how-tos/remote-control-via-tcpip-16496/
     * see: tum_ics_ur_robot/tum_ics_ur_robot_docs/Client_Interface_V3.8andV5.2.xlsx
     */
    struct RobotStateMsgV3V1
    {
        int         msgSize;        //      Message Size            integer	1		4
        double      time;           //      Time                    double	1		8
        VectorDOFd  qTar;           //      q target                double	6		48
        VectorDOFd  qpTar;          //      qd target               double	6		48
        VectorDOFd  qppTar;         //      qdd target              double	6		48
        VectorDOFd  iTar;           //      I target                double	6		48
        VectorDOFd  mTar;           //      M target                double	6		48
        VectorDOFd  qAct;           //      q actual                double	6		48
        VectorDOFd  qpAct;          //      qd actual               double	6		48
        VectorDOFd  iAct;           //      I actual                double	6		48
        VectorDOFd  iControl;       //      I control               double	6		48

        Vector6d    vtoolAct;       //      Tool vector actual      double	6		48
        Vector6d    tcpSpeedAct;    //      TCP speed actual        double	6		48
        Vector6d    tcpForce;       //      TCP force               double	6		48

        Vector6d    vtoolTar;       //      Tool vector target      double	6		48
        Vector6d    tcpSpeedTar;    //      TCP speed target        double	6		48
        double      dio;            //      Digital input bits      double	1		8
        VectorDOFd  motorTemp;      //      Motor temperatures      double	6		48
        double      ctrlTimer;      //      Controller Timer        double	1		8
        double      testVal;        //      Test value              double	1		8
        double      robotMode;      //      Robot Mode              double	1		8
        Vector6d    jointModes;     //      Joint Modes             double	6		48

        double      safetyMode;     //      Saftety Mode            double	1		8
        Vector6d    undefined;      //      Undefined               double	6		48

        Vector3d    toolAcc;        //      Tool Acc Values         double  3       18
        Vector6d    undefined1;     //      Undefined               double	6		48

        double      speedScaling;   //      Speed Scaling           double  1       8
        double      linMomNorm;     //      Linear Momentum Norm    double  1       8
        double      undefined2;     //      Undefined               double  1       8
        double      undefined3;     //      Undefined               double  1       8

        double      voltMain;       //      main voltage            double  1       8
        double      voltRobot;      //      robot voltage           double  1       8
        double      currentRobot;   //      robot current           double  1       8
        Vector6d    voltActual;     //      actual joint voltages   double  6       48
    };
    // v3.4
    /*!
     * \brief The RobotStateMsgV1V8 struct Structure to store the packet received from the real-time port for Polyscope version 3.4.
     * The real-time interface packet description has been taken from:
     * https://www.universal-robots.com/how-tos-and-faqs/how-to/ur-how-tos/remote-control-via-tcpip-16496/
     * see: tum_ics_ur_robot/tum_ics_ur_robot_docs/Client_Interface_V3.8andV5.2.xlsx
     */
    struct RobotStateMsgV3V4
    {
        int         msgSize;        //      Message Size            integer	1		4
        double      time;           //      Time                    double	1		8
        VectorDOFd  qTar;           //      q target                double	6		48
        VectorDOFd  qpTar;          //      qd target               double	6		48
        VectorDOFd  qppTar;         //      qdd target              double	6		48
        VectorDOFd  iTar;           //      I target                double	6		48
        VectorDOFd  mTar;           //      M target                double	6		48
        VectorDOFd  qAct;           //      q actual                double	6		48
        VectorDOFd  qpAct;          //      qd actual               double	6		48
        VectorDOFd  iAct;           //      I actual                double	6		48
        VectorDOFd  iControl;       //      I control               double	6		48

        Vector6d    vtoolAct;       //      Tool vector actual      double	6		48
        Vector6d    tcpSpeedAct;    //      TCP speed actual        double	6		48
        Vector6d    tcpForce;       //      TCP force               double	6		48

        Vector6d    vtoolTar;       //      Tool vector target      double	6		48
        Vector6d    tcpSpeedTar;    //      TCP speed target        double	6		48
        double      dio;            //      Digital input bits      double	1		8
        VectorDOFd  motorTemp;      //      Motor temperatures      double	6		48
        double      ctrlTimer;      //      Controller Timer        double	1		8
        double      testVal;        //      Test value              double	1		8
        double      robotMode;      //      Robot Mode              double	1		8
        Vector6d    jointModes;     //      Joint Modes             double	6		48

        double      safetyMode;     //      Saftety Mode            double	1		8
        Vector6d    undefined;      //      Undefined               double	6		48

        Vector3d    toolAcc;        //      Tool Acc Values         double  3       18
        Vector6d    undefined1;     //      Undefined               double	6		48

        double      speedScaling;   //      Speed Scaling           double  1       8
        double      linMomNorm;     //      Linear Momentum Norm    double  1       8
        double      undefined2;     //      Undefined               double  1       8
        double      undefined3;     //      Undefined               double  1       8

        double      voltMain;       //      main voltage            double  1       8
        double      voltRobot;      //      robot voltage           double  1       8
        double      currentRobot;   //      robot current           double  1       8
        Vector6d    voltActual;     //      actual joint voltages   double  6       48
        double      dioO;           //      actual digitial outputs double  1       8
        double      stateProg;      //      program state           double  1       8
    };
    // v3.5 and up
     /*!
     * \brief The RobotStateMsgV1V8 struct Structure to store the packet received from the real-time port for Polyscope version 3.5 and up.
     * The real-time interface packet description has been taken from:
     * https://www.universal-robots.com/how-tos-and-faqs/how-to/ur-how-tos/remote-control-via-tcpip-16496/
     * see: tum_ics_ur_robot/tum_ics_ur_robot_docs/Client_Interface_V3.8andV5.2.xlsx
     */
    struct RobotStateMsgV3V5
    {
        int         msgSize;        //      Message Size            integer	1		4
        double      time;           //      Time                    double	1		8
        VectorDOFd  qTar;           //      q target                double	6		48
        VectorDOFd  qpTar;          //      qd target               double	6		48
        VectorDOFd  qppTar;         //      qdd target              double	6		48
        VectorDOFd  iTar;           //      I target                double	6		48
        VectorDOFd  mTar;           //      M target                double	6		48
        VectorDOFd  qAct;           //      q actual                double	6		48
        VectorDOFd  qpAct;          //      qd actual               double	6		48
        VectorDOFd  iAct;           //      I actual                double	6		48
        VectorDOFd  iControl;       //      I control               double	6		48

        Vector6d    vtoolAct;       //      Tool vector actual      double	6		48
        Vector6d    tcpSpeedAct;    //      TCP speed actual        double	6		48
        Vector6d    tcpForce;       //      TCP force               double	6		48

        Vector6d    vtoolTar;       //      Tool vector target      double	6		48
        Vector6d    tcpSpeedTar;    //      TCP speed target        double	6		48
        double      dio;            //      Digital input bits      double	1		8
        VectorDOFd  motorTemp;      //      Motor temperatures      double	6		48
        double      ctrlTimer;      //      Controller Timer        double	1		8
        double      testVal;        //      Test value              double	1		8
        double      robotMode;      //      Robot Mode              double	1		8
        Vector6d    jointModes;     //      Joint Modes             double	6		48

        double      safetyMode;     //      Saftety Mode            double	1		8
        Vector6d    undefined;      //      Undefined               double	6		48

        Vector3d    toolAcc;        //      Tool Acc Values         double  3       18
        Vector6d    undefined1;     //      Undefined               double	6		48

        double      speedScaling;   //      Speed Scaling           double  1       8
        double      linMomNorm;     //      Linear Momentum Norm    double  1       8
        double      undefined2;     //      Undefined               double  1       8
        double      undefined3;     //      Undefined               double  1       8

        double      voltMain;       //      main voltage            double  1       8
        double      voltRobot;      //      robot voltage           double  1       8
        double      currentRobot;   //      robot current           double  1       8
        Vector6d    voltActual;     //      actual joint voltages   double  6       48
        double      dioO;           //      actual digitial outputs double  1       8
        double      stateProg;      //      program state           double  1       8
        Vector3d    elbowPos;       //      elbowPosition           double  3       24
        Vector3d    elbowVel;       //      elbowVelocity           double  3       24
    };

private:
    /*!
     * \brief CONNECT_TIME_OUT Time before trying to connect to the real-time socket.
     */
    static const unsigned int CONNECT_TIME_OUT = 5*1000; // 5 seconds

    /*!
     * \brief TOTAL_CONNECT_TIME_OUT Total time to try to connect, after this time a connexion error will be triggered.
     */
    static const unsigned int TOTAL_CONNECT_TIME_OUT = 30 * 1000;

    /*!
     * \brief PACKET_SIZE_V1V8 expected packet size from the real-time socket for version 1.8.
     */
    static const int PACKET_SIZE_V1V8 = 812;
    /*!
     * \brief PACKET_SIZE_V3V1 expected packet size from the real-time socket for version 3.1.
     */
    static const int PACKET_SIZE_V3V1 = 1044;
    /*!
     * \brief PACKET_SIZE_V3V4 expected packet size from the real-time socket for version 3.4.
     */
    static const int PACKET_SIZE_V3V4 = 1060;

    /*!
     * \brief PACKET_SIZE_V3V5 expected packet size from the real-time socket for version 3.5.
     * The packet size and description is the same for v3.5 and abovde
     */
    static const int PACKET_SIZE_V3V5 = 1108;

    /*!
     * \brief m_vers Polyscope version
     */
    Version m_vers;

    /*!
     * \brief m_buffer buffer to store the packet received from the robot in the real-time socket.
     */
    QByteArray m_buffer;

    /*!
     * \brief m_robotStateMsgV1V8 Struct variables to store the packet information for Polyscope version 1.8.
     *
     */
    RobotStateMsgV1V8 m_robotStateMsgV1V8;
    /*!
     * \brief m_robotStateMsgV3V1 Struct variables to store the packet information for Polyscope version 3.1.
     *
     */
    RobotStateMsgV3V1 m_robotStateMsgV3V1;
    /*!
     * \brief m_robotStateMsgV3V4 Struct variables to store the packet information for Polyscope version 3.4.
     *
     */
    RobotStateMsgV3V4 m_robotStateMsgV3V4;
    /*!
     * \brief m_robotStateMsgV3V5 Struct variables to store the packet information for Polyscope version 3.5 and up.
     *
     */
    RobotStateMsgV3V5 m_robotStateMsgV3V5;


    /*!
     * \brief m_ready Flag to control when the robot is broadcasting real data. We detected a bug in versions
     * 1.8 and 3.1 where the real-time socket reports q=[0,0,0,0,0,0] in the first cycles. This
     * is potentially dangerous since it can trigger an ustable condition in the controller. There
     * for, we reject this value until a "correct" data arrives. This means that the real robot should
     * never start in q=[0,0,0,0,0,0] position. This constraint is realistic since this particular
     * joint position is odd, and most probably will never be used as initial joint position.
     */
    bool m_ready;

    /*!
     * \brief m_errorString In case of communication errors, the description of the error will be stored in this variable
     */
    QString m_errorString;

    /*!
     * \brief m_error True with any communication error.
     */
    bool m_error;

public:
    /*!
     * \brief RobotStateClient Default constructor for the type class.
     * \param robotIpAddr IP address of the robot.
     * \param vers  Polyscope version installed in the controller of the target robot.
     * \param robotStatePort real-time state port, by defaul 300003.
     */
    RobotStateClient(const QString &robotIpAddr,
                     Version vers=VERSION_1V8,
                     quint16 robotStatePort=30003);

    ~RobotStateClient();

    /*!
     * \brief update Gets a packet from the socket and unpacks it in the correspoindig variable,
     * according to the version. Usually, this function should be called inside a thread with 125Hz.
     * \return true if succeed.
     */
    bool update();

    /*!
     * \brief getJointState gets the current joint state from the real robot
     * \return JointSate object with all the current states of the robot.
     */
    JointState getJointState() const;

    /*!
     * \brief getRobotStateMsgV1V8 Access method for the private variable m_robotStateMsgV1V8 with
     * the current data from the real robot.
     * \return it returns a struct variable with all the received information from the real-time socket.
     */
    RobotStateMsgV1V8 getRobotStateMsgV1V8() const;
    /*!
     * \brief getRobotStateMsgV3V1 Access method for the private variable m_robotStateMsgV3V1 with
     * the current data from the real robot.
     * \return it returns a struct variable with all the received information from the real-time socket.
     */
    RobotStateMsgV3V1 getRobotStateMsgV3V1() const;
    /*!
     * \brief getRobotStateMsgV3V4 Access method for the private variable m_robotStateMsgV3V4 with
     * the current data from the real robot.
     * \return it returns a struct variable with all the received information from the real-time socket.
     */
    RobotStateMsgV3V4 getRobotStateMsgV3V4() const;
    /*!
     * \brief getRobotStateMsgV3V5 Access method for the private variable m_robotStateMsgV3V5 with
     * the current data from the real robot.
     * \return it returns a struct variable with all the received information from the real-time socket.
     */
    RobotStateMsgV3V5 getRobotStateMsgV3V5() const;


    /*!
     * \brief isReady gets the ready flag. This flag is set to true when the robot is receiving reliable
     * information from the real-robot.
     * \return it returns the ready flag.
     */
    bool isReady() const;

    /*!
     * \brief error access method for the error flag
     * \return it returns the error flag
     */
    bool error();

    /*!
     * \brief errorString gets the error description in a Qstring.
     * \return Qstring with the error description.
     */
    const QString& errorString();

private:

    /*!
     * \brief unpackDataV1V8 Function called internally by the --update-- function.
     * it decodes the packet in the corresponding format, according to the Polyscope version.
     */
    void unpackDataV1V8();
    /*!
     * \brief unpackDataV3V1 Function called internally by the --update-- function.
     * it decodes the packet in the corresponding format, according to the Polyscope version.
     */
    void unpackDataV3V1();
    /*!
     * \brief unpackDataV3V4 Function called internally by the --update-- function.
     * it decodes the packet in the corresponding format, according to the Polyscope version.
     */
    void unpackDataV3V4();
    /*!
     * \brief unpackDataV3V5 Function called internally by the --update-- function.
     * it decodes the packet in the corresponding format, according to the Polyscope version.
     */
    void unpackDataV3V5();






};



}
}


#endif // ROBOTSTATECLIENT_H
