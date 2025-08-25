/* Main Robot Arm class
 * Authors: Emmanuel Dean and Florian Bergner
 * email: dean@tum.de, florian.bergner@tum.de
 *
 * v1: 2015
 */
#ifndef ROBOTARM_H
#define ROBOTARM_H

#include <QThread>
//#include <ur_robot_lli/RobotInterface/CommInterfaceScriptLoader.h>
#include <RtThreads/Thread.h>
#include <tum_ics_ur_robot_lli/Robot/DynamicModel.h>
#include <tum_ics_ur_robot_lli/Robot/KinematicModel.h>
#include <tum_ics_ur_robot_lli/RobotControllers/Controller.h>
#include <tum_ics_ur_robot_lli/RobotInterface/CommInterface.h>
#include <tum_ics_ur_robot_lli/RobotInterface/RobotStatePub.h>

// forward declaration of classes to make them friends
namespace tum_ics_ur_robot_manager
{
class SkillTransitionManager;
class SkillManager;
}  // namespace tum_ics_ur_robot_manager

namespace tum_ics_ur_robot_lli
{
namespace Robot
{
typedef RtThreads::Thread TThread;

typedef RobotInterface::RobotStatePub::VQString VQString_;

/*!
 * \brief The RobotArm class
 * This is the main robot arm class. This class handles the communication with
 * the real robot (get and set commands to the robot). It also handles the communication with the
 * state machine running in the script inside the real robot controller. The class contains Kinematic and
 * Dynamic model classes to call kinematic and dynamic paramters and functions, e.g. FK, M, C, G, etc.
 * The class has its own thread which gets the current state from the real robot (or the simulated robot),
 * updates the internal clock (used to compute models and controllers), calls the update function of each
 * controller object added to the robot, and publishes messages to visualize the robot in rviz.
 */
class RobotArm : public TThread, public Tum::Tools::MathTools
{
  // for changing controller list with private add/remove while running functions
  friend class tum_ics_ur_robot_manager::SkillTransitionManager;
  friend class tum_ics_ur_robot_manager::SkillManager;

public:
  typedef QList<RobotControllers::Controller *> LController;

private:
  typedef RobotInterface::CommInterface RobotCommInterface;

  // Member variables
private:
  /*!
   * \brief m_errorString In case of communication/parsing/cosntructing/initialization errors,
   * the text description of the error will be stored in this variable.
   */
  QString m_errorString;

  /*!
   * \brief m_error True with any communication/parsing/cosntructing/initialization error.
   */
  bool m_error;

  /*!
   * \brief m_commIntf Provides functions to connect with the state machine and the motion server inside
   * the ur script running in the controller. The class has methods to get the curent joint state of the
   * real robot and set the commanded joint positions/velocities, according to the motion command type
   * defined in the configuration file of the robot (config.ini).
   */
  RobotCommInterface *m_commIntf;

  /*!
   * \brief m_extRobotStateClient Communication Interface class. Connects to the Real-Time socket 30003,
   * unpacks and provides methods to get the joint state of the real robot.
   */
  RobotInterface::RobotStateClient *m_extRobotStateClient;

  /*!
   * \brief m_dynamicModel Dynamic model class, with useful models of the robot.
   */
  DynamicModel *m_dynamicModel;

  /*!
   * \brief m_kinematicModel Kinematic model class, with useful models of the robot, e.g. FK, Jacobians, etc.
   */
  KinematicModel *m_kinematicModel;

  /*!
   * \brief m_robotStatePub
   */
  RobotInterface::RobotStatePub *m_robotStatePub;

  /*!
   * \brief m_controllers List of controllers, which will be executed in the main thread.
   */
  LController m_controllers;

  // ROS
  /*!
   * \brief n ros node handler
   */
  ros::NodeHandle n;

  /*!
   * \brief pubJointsCtrl publisher of robot data, current joint position, velocity, total control, etc.
   */
  ros::Publisher pubJointsCtrl;

  /*!
   * \brief m_pubCycleTime publisher of cycle time. This is used to verify that the real time is preserved.
   */
  ros::Publisher m_pubCycleTime;  //!< pub control cycle time

  /*!
   * \brief xef_old Position vector of the end-effector in the time t-1.
   */
  Vector3d xef_old;

  /*!
   * \brief m_configFilePath robot config file name (config.ini) with absolut path.
   */
  QString m_configFilePath;

  /*!
   * \brief m_stopThread flag to stop the main thread.
   */
  bool m_stopThread;

  /*!
   * \brief m_stoppedThread flag to verify if the main thread is running.
   * true = main thread not running.
   * false = main thread running.
   */
  bool m_stoppedThread;

  /*!
   * \brief m_initTimer Flag to initialize the internal timer of the robot. This timer is
   * imortant since in shared among all the classes, kinematics, dynamics, and all the controllers.
   * In this form, even when the controllers are computed sequencially, for the point of view of the
   * robot thread, they are computed in parallel.
   */
  bool m_initTimer;

  /*!
   * \brief m_startJointStatePub  Flag to verify if the robot JointState publisher is started.
   * true = the JointState publisher is running.
   * false = the JointState publisher is not running.
   */
  bool m_startJointStatePub;

  /*!
   * \brief m_realRobot Flag to verfiy if the real robot is activated.
   * true = the real robot is activated, then all the computed signals will be sent to the
   * real robot. The current joint state will also be taken from the real robot.
   * false = using the simulated robot. In this case, the current joint state will be taken from the /joint_state
   * topic.
   */
  bool m_realRobot;

  /*!
   * \brief m_initQsim Flag to set the QHOME as the initial joint position. QHOME is defined in the robot configuration
   * file (config.ini). This is only valid when running the robot in simulation mode.
   */
  bool m_initQsim;

  /*!
   * \brief m_initQreal Flag to set the current joint position (taken from the real robot) as the initial joint
   * position. The dynamic model initial conditions are also set using this flag.
   */
  bool m_initQreal;

  /*!
   * \brief m_checkQZeros Flag to verify when the robot is sending reliable data that can be used for control.
   */
  bool m_checkQZeros;

  /*!
   * \brief m_Kp Proportional gain for the internal PID velocity control.
   */
  MatrixDOFd m_Kp;

  /*!
   * \brief m_Kd Diferential gain for the internal PID velocity control.
   */
  MatrixDOFd m_Kd;

  /*!
   * \brief m_Ki Integral gain for the internal PID velocity control.
   */
  MatrixDOFd m_Ki;

  /*!
   * \brief m_lowerJointVelThresh saturation threshold to define the dead-zone of the actuators.
   */
  VectorDOFd m_lowerJointVelThresh;

  /*!
   * \brief m_dof DOF of the robot
   */
  int m_dof;

  /*!
   * \brief m_controlPeriodMS control period in ms
   */
  double m_controlPeriodMS;
  /*!
   * \brief m_controlPeriod control period in s
   */
  double m_controlPeriod;

  /*!
   * \brief m_controlPeriod_2 half of the control period.
   */
  double m_controlPeriod_2;

  /*!
   * \brief m_groupName String to define if the robot is right/left/standard
   */
  QString m_groupName;

  /*!
   * \brief m_jointNames Array of joint names, needed by the JointState publisher.
   */
  VQString_ m_jointNames;

  /*!
   * \brief m_topicName Topic name where the JointStates will be published.
   */
  QString m_topicName;

  /*!
   * \brief m_q current joint position.
   */
  JointState m_q;

  /*!
   * \brief m_qSendReal commanded joint position. This joint position may differ from the current
   * joint position m_q due to compensation factors.
   */
  JointState m_qSendReal;

  /*!
   * \brief m_qOut joint position computed with the direct dynamics.
   */
  JointState m_qOut;

  /*!
   * \brief m_qInit Initial joint position, depending if the robot is running in simulation or real, its value
   * will be either QHOME or the initial real robot joint position.
   */
  JointState m_qInit;

  /*!
   * \brief m_qHome Home joint position defined in the robot configuration file (config.ini)
   */
  JointState m_qHome;

  /*!
   * \brief m_qPark Park joint position defined in the robot configuration file (config.ini)
   */
  JointState m_qPark;

  /*!
   * \brief m_mutex mutex to guarantee a safe write/read process.
   */
  QMutex m_mutex;

  // Variables only used with the Skill Manager class, not needed when using controllers directly.
  // controllerList update pending mutex
  // idea change controllers and dynamics model
  // only in new iteration
  QMutex m_ctrlListMutex;
  LController m_pendingCtrls;
  bool m_pendingCtrlsFlag;
  bool m_pendingLoadCtrl;
  bool m_idle;

  /*!
   * \brief m_ti initial ros time. Used to compute the internal robotArm clock.
   */
  ros::Time m_ti;

  /*!
   * \brief m_rTime internal robotArm time data container.
   */
  RobotTime m_rTime;

  /*!
   * \brief m_nControls number of total controllers added to the robotArm.
   */
  uint m_nControls;

  /*!
   * \brief m_ctrlWeights Array of controllers weights to ponderate the contribution of each controller.
   */
  VDouble m_ctrlWeights;

  /*!
   * \brief m_bfCutof cutoff frequency used in all the filters. This frequency is defined in the robot configuration
   * file (config.ini)
   */
  double m_bfCutof;

  /*!
   * \brief m_bfQ Filter for the joitn position
   */
  TButter2 m_bfQ;

  /*!
   * \brief m_bfDQ filter for the error between the virtual joint position and real joint position.
   * This filter is used in the inner velocity control.
   */
  TButter2 m_bfDQ;

  /*!
   * \brief m_bfDv currently not used.
   */
  TButter2 m_bfDv;

  /*!
   * \brief m_bfDQp currently not used.
   */
  TButter2 m_bfDQp;

  /*!
   * \brief m_bfQp filter for the real joint position.
   */
  TButter2 m_bfQp;

  /*!
   * \brief m_bfVelComp filter for the joint velocity compensation.
   */
  TButter2 m_bfVelComp;

  /*!
   * \brief m_noEventMode Flag to control if the event controllers will be used or not.
   * true standard control calculation.
   * false event-driven control.
   * By default this flag is false.
   */
  bool m_noEventMode;

  // Member Methods
public:
  /*!
   * \brief RobotArm Default constructor.
   * \param configFilePath robot configuration file name with absolute path.
   * \param extRobotStateClient external RobotStateClient instance. If this object is passed as argument,
   * then the commInterface class will not create a new one and use this for the real-time communication socket 30003.
   * \param groupName Name of the group for the robot.
   * \param robotDescription currently not used.
   */
  RobotArm(const QString &configFilePath, RobotInterface::RobotStateClient *extRobotStateClient = NULL,
           const QString &groupName = "none", const QString &robotDescription = "robot_description");
  ~RobotArm();

  /*!
   * \brief init This is a really important function, and must be runned before starting to use the robot arm
   * The function parses the robot configuration file and sets the local variables, initializes the kinematic model,
   * the dynamic model, loads the internal PID controller gains, and if "real" robot is used, it starts the
   * communication with the real robot, both the state machine in the script running in the robot control unit, and
   * with the real-time joint state socket (30003). It also starts the JointState publisher.
   * \return true if all the initializations where succsessful.
   */
  bool init();
  // bool start();

  /*!
   * \brief stop Send the signal to stop the state machine running in the script inside the robot control unit and
   * stops the main thread.
   * \return  true
   */
  bool stop();

  /*!
   * \brief isRunning gets the flag to verify if the main thread is still running.
   * \return
   */
  bool isRunning() const;

  // the controller list is empty
  /*!
   * \brief isIdle Flag that verifies if the list of controllers is empty.
   * true = any controller has been added to the robotArm.
   * false = there's at least one controller in the list.
   * \return
   */
  bool isIdle() const;

  /*!
   * \brief error access method for the error flag
   * \return it returns the error flag
   */
  bool error() const;

  /*!
   * \brief errorString gets the error description in a Qstring.
   * \return Qstring with the error description.
   */
  const QString &errorString() const;

  /*!
   * \brief getKinematicModel gets the internal kinematic model.
   * \return pointer to the internal kinematic model.
   */
  KinematicModel *getKinematicModel() const;

  /*!
   * \brief getDynamicModel the internal dynamic model.
   * \return pointer to the internal dynamic model.
   */
  DynamicModel *getDynamicModel() const;

  /*This function creates a Qlist of controllers where the first appended controller will define the CtrlInterface
   * (e.g. VELOCITY_INTF, EFFORT_INTF, etc.)
   *
   */

  /*!
   * \brief add This function creates a Qlist of controllers where the first appended controller will define the
   * CtrlInterface (e.g. VELOCITY_INTF, EFFORT_INTF, etc.) \param u controller that has to be added to the robotArm.
   * \return  true = if the controller was succsessfully added to the list of controllers.
   */
  bool add(RobotControllers::Controller *u);

  /*!
   * \brief remove removes a controller from the controller list.
   * \param u target controller to be removed.
   * \return true = if the controller was succsessfully removed.
   */
  bool remove(RobotControllers::Controller *u);

  /*!
   * \brief setCtrlWeights This function sets the control weight to each controller in the controller list.
   * \return true = if all the controllers weights were succsessfully added to the controllers.
   */
  bool setCtrlWeights();

  /*!
   * \brief getCtrlInterface gets the control interface. This interface is defined by the first controller added
   * to the robotArm. This flag is used to prevent adding controllers with different interface, e.g. torque + velocity.
   * \return current control interface running in the robot.
   */
  const RobotControllers::ControlInterface getCtrlInterface() const;

  /*!
   * \brief jointNames gets the list of joint names.
   * \return Vector of joint names.
   */
  const VQString_ &jointNames() const;  // test//

  /*!
   * \brief resetTimer sets the flag that resets the internal clock.
   * \return true
   */
  bool resetTimer();

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
   * \brief q gets the current joint state.
   * \return current joint state.
   */
  JointState q();

  /*!
   * \brief vqHome gets the Home joint position vector.
   * \return joint position vector for Home position.
   */
  VectorDOFd vqHome() const;

  /*!
   * \brief vqPark gets the Park joint position vector.
   * \return joint position vector for Park position.
   */
  VectorDOFd vqPark() const;

  /*!
   * \brief vqPark gets the current joint position vector.
   * \return current joint position vector.
   */
  VectorDOFd vq();

  QString groupName();

  // Memeber Methods
private:
  /*!
   * \brief run callback function called by the main thread. This function will run two different functions
   * depending on the type of robot, simulation or real.
   */
  void run();

  // We have to modes: Simulated robot and Real robot
  /*!
   * \brief runSim callback function for simulated robot. This function will gets the current joitn state (simulated),
   * updates the kinematic models, computes each controller in the controller list, updates the dynamic model, and
   * computes the output joint state.
   */
  void runSim();

  /*!
   * \brief runReal callback function for real robot. This function will get the current joitn state
   * (from the real-time socket), updates the kinematic models, computes each controller in the controller list,
   * updates the dynamic model, and  compute the output joint state which will be commanded to the real robot.
   */
  void runReal();

  /*!
   * \brief parseConfig function to parse the robot configuration file (config.ini) and store the parameters in
   * the local variables.
   * \return
   */
  bool parseConfig();

  /*!
   * \brief defaultControlValue function to calculate the default control value (or zero control value). This depends
   * on the control interface, i.e. for Position Interface the default value is the previous joint position, for
   * Velocity Interface and Effort Intrefaces the default value should be zero.
   * \return vector of default values.
   */
  VectorDOFd defaultControlValue();

  /*!
   * \brief reloadControlGains reloads the inner velocity control gains from the ros parameter server.
   * \return true = all the parameters could be loaded.
   */
  bool reloadControlGains();

  // This functions are only used by the SKill manager, not needed with individual controllers.
  // special add/remove functions for skill transition manager
  // add/remove while robot is running
  // blocking until added/removed
  bool addWhileRunning(RobotControllers::Controller *u);
  bool removeWhileRunning(RobotControllers::Controller *u);
  void handleControllerListChange();

  /*!
   * \brief reinit reinitializes all the joint states, and the dynamic model (resets the initial condition of the
   * direct dynamic model).
   */
  void reinit();

  /*!
   * \brief cleanup function that deletes all the object classes created inside the robotArm, e.g. kinematic model,
   * dynamic model, robotPublisher, etc.
   */
  void cleanup();
};

}  // namespace Robot
}  // namespace tum_ics_ur_robot_lli

#endif  // ROBOTARM_H
