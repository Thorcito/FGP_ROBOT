#include <tum_ics_ur_robot_lli/Robot/RobotArm.h>
#include <tum_ics_ur_robot_msgs/FloatValue.h>
#include <tum_ics_ur_robot_msgs/RobotData.h>
#include <unistd.h>

#include <QElapsedTimer>
#include <QFileInfo>
#include <QSettings>

namespace tum_ics_ur_robot_lli
{
namespace Robot
{
RobotArm::RobotArm(const QString &configFilePath, RobotInterface::RobotStateClient *extRobotStateClient,
                   const QString &groupName, const QString &robotDescription)
  : TThread(RtThreads::Thread::/*QThreadMode*/ RtThreadMode, "robotArm")
  , m_commIntf(NULL)
  , m_extRobotStateClient(extRobotStateClient)
  , m_dynamicModel(NULL)
  , m_kinematicModel(NULL)
  , m_error(false)
  , m_configFilePath(configFilePath)
  , m_stopThread(false)
  , m_stoppedThread(true)
  , m_initTimer(false)
  , m_startJointStatePub(false)
  , m_robotStatePub(NULL)
  , m_realRobot(false)
  , m_initQsim(false)
  , m_initQreal(false)
  , xef_old(Vector3d::Zero())
  ,  // TODO: remove this variable
  m_Kp(MatrixDOFd::Zero())
  , m_Kd(MatrixDOFd::Zero())
  , m_Ki(MatrixDOFd::Zero())
  , m_pendingCtrlsFlag(false)
  , m_idle(true)
  , m_checkQZeros(true)
  , m_groupName(groupName)
  , m_noEventMode(true)
{
  if (TThread::isRtThread())
  {
    TThread::rtThread()->setPriority(8);
    TThread::rtThread()->setStackSize(16 * 1024 * 1024);
    TThread::rtThread()->setStackPreFaultSize(64 * 1024 * 1024);
  }
}
RobotArm::~RobotArm()
{
  ROS_INFO_STREAM("RobotArm::~RobotArm: Entered RobotArm class destructor");
  QElapsedTimer timer;

  timer.start();

  // Wait until stoppedThread changes or the limit time has been reached (5secs)
  while ((!m_stoppedThread) && (timer.elapsed() < 5000))
  {
    ::usleep(5 * 1E5);
  }

  if (m_stoppedThread)
  {
    ROS_INFO_STREAM("RobotArm::~RobotArm: class start cleanup ...");
    cleanup();
    ROS_INFO_STREAM("RobotArm::~RobotArm: class destroyed.");
    return;
  }

  ROS_INFO_STREAM("RobotArm::~RobotArm: Waiting for thread of RobotArm class to finish ...");

  // if the thread has not been stopped
  if (!TThread::wait(500))
  {
    ROS_INFO_STREAM("RobotArm::~RobotArm: Terminating thread of RobotArm class.");
    TThread::terminate();
    TThread::wait();
    ROS_INFO_STREAM("RobotArm::~RobotArm: Thread forced to end!!!");
  }

  cleanup();

  ROS_INFO_STREAM("RobotArm::~RobotArm: class destroyed.");
}

bool RobotArm::init()
{
  ROS_INFO_STREAM("Loading Configuration File");
  if (!parseConfig())
  {
    return false;
  }

  ROS_INFO_STREAM("Loading Kinematics");
  m_kinematicModel = new KinematicModel(m_configFilePath, m_groupName);

  if (m_kinematicModel->error())
  {
    return false;
  }

  ROS_INFO_STREAM("Loading Dynamics");
  m_dynamicModel = new DynamicModel(m_configFilePath, m_kinematicModel);

  if (m_dynamicModel->error())
  {
    return false;
  }

  ROS_INFO_STREAM("Loading Internal Ctrl Gains");
  if (!reloadControlGains())
  {
    return false;
  }

  // We must init CommInterface only when we use Real robot
  if (m_realRobot)
  {
    ROS_INFO_STREAM("Starting LLI Comm");

    // This if can be removed since m_extRobotStateClient has NULL as default value
    // and the constructor of CommInterface also handles NULL values
    if (m_extRobotStateClient == NULL)
    {
      // The robotStateClient is not provided externally, we need to create it!
      m_commIntf = new RobotCommInterface(m_configFilePath);
    }
    else
    {
      m_commIntf = new RobotCommInterface(m_configFilePath, m_extRobotStateClient);
    }

    if (m_commIntf->error())
    {
      return false;
    }
  }

  //    //For now to test
  //    m_jointNames=RobotInterface::RobotStatePub::VQString()
  //            << "r_shoulder_pan_joint"
  //            << "r_shoulder_lift_joint"
  //            << "r_elbow_joint"
  //            << "r_wrist_1_joint"
  //            << "r_wrist_2_joint"
  //            << "r_wrist_3_joint";

  //    QString topicName="/right_arm_joint_states";

  ROS_INFO_STREAM("Init StatePublisher");
  m_robotStatePub = new RobotInterface::RobotStatePub(m_topicName, m_jointNames);

  // To test
  //    pubJointsCtrl=n.advertise<sensor_msgs::JointState>("JointsCtrl",100);
  pubJointsCtrl = n.advertise<tum_ics_ur_robot_msgs::RobotData>("RobotData", 100);

  m_pubCycleTime = n.advertise<tum_ics_ur_robot_msgs::FloatValue>("robotCtrlCycleTime", 100);

  return true;
}

// bool RobotArm::start()
//{
//    TThread::start();
//}

bool RobotArm::stop()
{
  if ((m_realRobot) && (m_commIntf != NULL))
  {  // set the mstate to finish (this is sent by commInterface to the UR controller and finalize the script)
    m_commIntf->setFinishMState();
  }

  // set the flag to stop the robotArm thread
  m_stopThread = true;

  return true;
}

bool RobotArm::isRunning() const
{
  return !m_stoppedThread;
}

bool RobotArm::isIdle() const
{
  return m_idle;
}

bool RobotArm::error() const
{
  return m_error;
}
const QString &RobotArm::errorString() const
{
  return m_errorString;
}

KinematicModel *RobotArm::getKinematicModel() const
{
  return m_kinematicModel;
}

DynamicModel *RobotArm::getDynamicModel() const
{
  return m_dynamicModel;
}

bool RobotArm::add(RobotControllers::Controller *u)
{
  if (!m_stoppedThread)
  {
    ROS_WARN_STREAM("The controllers are already running. You can not add any more controllers to the list");
    return false;
  }

  if (u->getCtrlIntf() >= RobotControllers::UNKNOWN_ITNF)
  {
    m_error = false;

    std::stringstream s;
    s << "RobotArm(): The control interface --" << u->getCtrlIntf() << "-- is not implemented yet.";
    m_errorString = s.str().c_str();

    ROS_ERROR_STREAM(m_errorString.toStdString());

    return false;
  }

  if (!m_controllers.isEmpty())
  {
    if (!(u->getCtrlIntf() == m_controllers.first()->getCtrlIntf()))
    {
      ROS_WARN_STREAM("The control could not be added to the list");
      ROS_WARN_STREAM("The target controller has a different CNTROL INTERFACE: "
                      << u->getCtrlIntf() << " vs " << m_controllers.first()->getCtrlIntf());
      return false;
    }

    if (u->getCtrlIntf() == RobotControllers::POSITION_INTF)
    {
      ROS_WARN_STREAM("The control could not be added to the list");
      ROS_WARN_STREAM("You can not add two controllers with POSITION INTF ("
                      << u->getCtrlIntf() << "). Use the control manager to fuse these two controllers.");
      return false;
    }
  }

  if (u->getCtrlSampleMode() == RobotControllers::EVENT_SAMPLE_MODE)
  {
    m_noEventMode = false;
  }

  // TODO: verify that event ctrl is at the top of the list, if not move it to the top. For now we do this manually!
  m_controllers.append(u);
  m_idle = m_controllers.isEmpty();

  // Get the control interface (EFFORT, POS or VEL) and assign it to the dynamics
  m_dynamicModel->setCtrlInterface(u->getCtrlIntf());


  // Load gains
  qDebug("Init controller '%s' ...", u->name().toLatin1().data());
  u->init();
  qDebug("Controller '%s' initialized.", u->name().toLatin1().data());

  // Set the Ctrl ID using the index in the controller list (m_controllers)
  //    u->setID(m_controllers.size());

  ROS_INFO_STREAM("Control (" << m_controllers.last()->name().toStdString() << ") added!");

  m_controllers.last()->setLoaded(true);

  return true;
}

bool RobotArm::remove(RobotControllers::Controller *u)
{
  if (!m_stoppedThread)
  {
    ROS_WARN_STREAM("The controllers are already running. You can not remove any controller from the list");
    return false;
  }
  if (m_controllers.contains(u))
  {
    u->setLoaded(false);
  }

  m_controllers.removeOne(u);

  m_noEventMode = true;
  for (int i = 0; i < m_controllers.size(); i++)
  {
    if (m_controllers.at(i)->getCtrlSampleMode() == RobotControllers::EVENT_SAMPLE_MODE)
    {
      m_noEventMode = false;
      break;
    }
  }

  m_idle = m_controllers.isEmpty();

  return true;
}

bool RobotArm::setCtrlWeights()
{
  if (m_controllers.isEmpty())
  {
    ROS_WARN_STREAM("There's any controller in the list. Did you forget to add() a controller?");
    return false;
  }

  int nControllers = m_controllers.size();

  if (!(nControllers == m_ctrlWeights.size()))
  {
    ROS_WARN_STREAM("There number of available controllers (" << nControllers << ") differs from the weights ["
                                                              << m_ctrlWeights.size() << "]");
    return false;
  }
  for (int i = 0; i < nControllers; i++)
  {
    m_controllers[i]->setWeight(m_ctrlWeights.at(i));
    ROS_INFO_STREAM("Control (" << i << ") weight changed to: " << m_ctrlWeights.at(i));
  }

  return true;
}

// bool RobotArm::setSingleCtrlWeight(uint ctrlID, double weight)
//{
//    if(!m_controllers.isEmpty())
//    {
//        ROS_WARN_STREAM("There's any controller in the list. Did you forget to add() a controller?");
//        return false;
//    }

//    int nControllers=m_controllers.size()-1;

//    if(ctrlID>nControllers)
//    {
//        ROS_WARN_STREAM("The target controller id ("<<
//                        nControllers<<") is bigger that the available number of controllers ["<<
//                        m_ctrlWeights.size()<<"]");
//        return false;
//    }

//    m_controllers[ctrlID]->setWeight(weight);

//}

const RobotControllers::ControlInterface RobotArm::getCtrlInterface() const
{
  return m_dynamicModel->getCtrlInterface();
}

const VQString_ &RobotArm::jointNames() const
{
  return m_jointNames;
}

bool RobotArm::resetTimer()
{
  m_initTimer = false;
  return !m_initTimer;
}

JointState RobotArm::qHome() const
{
  return m_qHome;
}
JointState RobotArm::qPark() const
{
  return m_qPark;
}
JointState RobotArm::q()
{
  m_mutex.lock();
  return m_qOut;
  m_mutex.unlock();
}

VectorDOFd RobotArm::vqHome() const
{
  return m_qHome.q;
}
VectorDOFd RobotArm::vqPark() const
{
  return m_qPark.q;
}
VectorDOFd RobotArm::vq()
{
  m_mutex.lock();
  return m_qOut.q;
  m_mutex.unlock();
}
QString RobotArm::groupName()
{
  m_mutex.lock();
  return m_groupName;
  m_mutex.unlock();
}

void RobotArm::run()
{
  qDebug("Robot arm pid: %d", RtThreads::Thread::gettid());

  ROS_INFO_STREAM("Robot Thread running...");
  m_checkQZeros = true;

  if (m_realRobot)
  {
    // This functions have its own loop
    runReal();
  }
  else
  {
    runSim();
  }

  m_robotStatePub->stop();
  m_stoppedThread = true;
  ROS_INFO_STREAM("RobotArm::run(): Run exited");
}

void RobotArm::runSim()
{
  ROS_INFO_STREAM("Sim Robot Thread running...");
  m_stoppedThread = false;
  double initT = 0.0;

  while (!m_stopThread)
  {
    ros::Time tic = ros::Time::now();

    // savely change controller list while the robot is running
    handleControllerListChange();

    if (!m_initTimer)
    {
      m_ti = ros::Time::now();
      m_initTimer = true;
      m_rTime.reset(m_ti);
      // initT=m_rTime.tD();
    }

    if (!m_startJointStatePub)
    {
      // init the publisher

      m_robotStatePub->start();
      m_startJointStatePub = true;
    }

    // Get time
    // double time=tic.toSec()-m_ti.toSec();
    m_rTime.update(tic);

    //        ROS_INFO_STREAM("time: "<<m_rTime.tD());

    if (!m_initQsim)
    {
      // m_qInit is defined in the config file
      // TODO: After switching controller we have to re-set m_qInit to the current q and
      // set m_initQsim = false;
      m_q = m_qInit;
      m_initQsim = true;
    }

    if (!m_initQreal)
    {
      m_initQreal = true;
    }

    // update the joint state for visualization
    m_robotStatePub->setJointState(m_q);

    // Update Kinematic models Xef, Xpef, Jef
    // If there's a event ctrl in the list then this ctrl HAS to update the kinModel!!!
    if (m_noEventMode)
    {
      m_kinematicModel->update(m_rTime, m_q);
    }

    // TODO: reduce the frq of publishing!!!
    m_kinematicModel->publish_FK();

    if (m_kinematicModel->error())
    {
      stop();
      break;
    }

    // add the real robot position to the jointState
    m_q.qReal = m_q.q;
    m_q.qpReal = m_q.qp;

    bool controlFailed = false;

    // compute tau (control)
    if (!m_controllers.isEmpty())
    {
      m_q.tau.setZero();
      for (int i = 0; i < m_controllers.size(); i++)
      {
        VectorDOFd ctrlout = m_controllers[i]->update(m_rTime, m_q);
        // ROS_INFO_STREAM(m_controllers[i]->getName().toStdString()<<" ["<<i<<"]: "<<ctrlout.transpose());

        m_q.tau += ctrlout;
        // ROS_INFO_STREAM("TControl["<<i<<"]: "<<m_q.tau.transpose());

        // catch error in each control, e.g. nan, or inf
        if (m_controllers.at(i)->error())
        {
          stop();
          // not continue because we don't want to send anything to the robot
          controlFailed = true;
          break;
        }
      }
    }
    else
    {
      // just in this test
      m_q.tau = defaultControlValue();
    }
    if (controlFailed)
    {
      // catch the error in the dynamics, e.g. inf or nan
      stop();
      controlFailed = false;
      break;
    }

    //        ROS_INFO_STREAM("Tau3: "<<m_q.tau.transpose());
    //

    // compute qppd,qpd,qd (dynamics)
    m_dynamicModel->update(m_rTime, m_q);
    if (m_dynamicModel->error())
    {
      // catch the error in the dynamics, e.g. inf or nan
      stop();
      break;
    }

    // Set the output from dynamic to the current joint state
    m_q.q = m_dynamicModel->q();
    m_q.qp = m_dynamicModel->qp();
    //        m_q.q=m_dynamicModel->qf();
    //        m_q.qp=m_dynamicModel->qpf();

    // save the current joint for external access
    m_mutex.lock();
    m_qOut = m_q;
    m_mutex.unlock();

    /// Publish data
    // to test
    // sensor_msgs::JointState msg = m_qOut.toJointStateMsg(m_jointNames);
    tum_ics_ur_robot_msgs::RobotData msg;

    msg.header.stamp = m_rTime.tRc();

    msg.time = m_rTime.tD();

    for (int i = 0; i < STD_DOF; i++)
    {
      msg.q[i] = m_dynamicModel->q()(i);
      msg.qp[i] = m_dynamicModel->qp()(i);
      msg.qpp[i] = m_dynamicModel->qpp()(i);

      msg.qf[i] = m_dynamicModel->qf()(i);
      msg.qpf[i] = m_dynamicModel->qpf()(i);
      msg.qppf[i] = m_dynamicModel->qppf()(i);

      msg.torques[i] = m_q.tau(i);
    }
    pubJointsCtrl.publish(msg);

    ros::Time toc = ros::Time::now();

    tum_ics_ur_robot_msgs::FloatValue msg2;
    msg2.header.stamp = m_rTime.tRc();
    msg2.time = m_rTime.tD();
    msg2.value = toc.toSec() - tic.toSec();
    m_pubCycleTime.publish(msg2);

    double elapsedTime = toc.toSec() - tic.toSec();
    double loopPeriod = 8 * 1E-3;
    double delay = (m_controlPeriod - elapsedTime) * 1E6;

    if (elapsedTime > m_controlPeriod)
    {
      continue;
    }
    else if (elapsedTime > loopPeriod)
    {
      ROS_WARN_STREAM("Control loop period exceeds the limit: " << elapsedTime);
      continue;
    }

    // ROS_INFO_STREAM("E time: "<<m_rTime.tD()-initT);
    initT = m_rTime.tD();

    if (TThread::isRtThread())
    {
      TThread::usleep(m_controlPeriodMS * 1000);  // micro s
    }
    else
    {
      TThread::usleep((unsigned long)delay);
    }
  }

  m_robotStatePub->stop();
  m_stoppedThread = true;
  qDebug() << "RobotArm::run(): Sim Thread exited";
}

void RobotArm::runReal()
{
  ROS_WARN_STREAM("Real Robot running...");
  m_stoppedThread = false;
  while (!m_stopThread)
  {
    ros::Time tic = ros::Time::now();
    // check if commInterface is ready before starting the controller!
    // we should not start the controller until the STATE_TRAJ_FOLLOW is enabled
    // the flaf isready is only set in STATE_TRAJ_FOLLOW!!

    if (!m_commIntf->isReady())
    {
      TThread::usleep(100 * 1000);
      continue;
    }

    JointState qO;
    if (m_checkQZeros)
    {
      bool yesta = true;
      while ((!m_stopThread) && (yesta))
      {
        qO = m_commIntf->getCurrentJointState();
        ROS_WARN_STREAM("Waiting for pos: " << qO.q.transpose());

        for (int i = 0; i < STD_DOF; i++)
        {
          if (abs(qO.q(i)) > 1e-3)
          {
            yesta = false;
            break;
          }
        }

        TThread::usleep(100 * 1000);
      }
      m_checkQZeros = false;
    }

    // savely change controller list while the robot is running
    handleControllerListChange();

    if (!m_initTimer)
    {
      m_ti = ros::Time::now();
      m_initTimer = true;
      m_rTime.reset(m_ti);
    }

    if (!m_startJointStatePub)
    {
      // init the publisher
      m_robotStatePub->start();
      m_startJointStatePub = true;
    }

    // Get time
    // double time=tic.toSec()-m_ti.toSec();
    m_rTime.update(tic);

    //        ROS_INFO_STREAM("time: "<<m_rTime.tD());
    //        if(m_rTime.tD()>0.016)
    //        {
    //            stop();
    //            break;
    //        }

    // ROS_INFO_STREAM("robotArm run(): Time= "<<time);

    // get current q,qp
    JointState m_qO = m_commIntf->getCurrentJointState();                 


    // use virtual robot
    if (!m_initQreal)
    {
      m_q = m_qO;
      m_qSendReal = m_qO;
      m_dynamicModel->reinit();
      m_initQreal = true;
    }
    else
    {
      // Set the output from dynamic to the current joint state
      m_q.q = m_dynamicModel->q();
      m_q.qp = m_dynamicModel->qp();
    }

    // update the joint state for visualization
    m_robotStatePub->setJointState(m_qO);

    // Update Kinematic models
    if (m_noEventMode)
    {
      m_kinematicModel->update(m_rTime, m_q);
    }

    //        m_kinematicModel->update(m_rTime,m_q);
    //        m_kinematicModel->update(m_rTime,m_qO);
    m_kinematicModel->publish_FK();

    if (m_kinematicModel->error())
    {
      // catch the error in the kinematics, e.g. inf or nan
      ROS_ERROR_STREAM(
          "ERROR RobotArm: Kinematic Model failed with error: " << m_kinematicModel->errorString().toStdString());
      stop();
      break;
    }

    bool controlFailed = false;

    // add the real robot position to the jointState
    m_q.qReal = m_qO.q;
    m_q.qpReal = m_qO.qp;

    // compute tau (control)
    if (!m_controllers.isEmpty())
    {
      m_q.tau.setZero();
      for (int i = 0; i < m_controllers.size(); i++)
      {
        VectorDOFd ctrlout = m_controllers[i]->update(m_rTime, m_q);
        // ROS_INFO_STREAM(m_controllers[i]->getName().toStdString()<<" ["<<i<<"]: "<<ctrlout.transpose());
        // TODO: implement this as a function where we can define different fusing functions for ctrls, e.g. stack of
        // tasks
        m_q.tau += ctrlout;
        //                ROS_INFO_STREAM("TControl["<<i<<"]: "<<m_q.tau.transpose());

        // catch error in each control, e.g. nan, or inf
        if (m_controllers.at(i)->error())
        {
          stop();
          // not continue because we don't want to send anything to the robot
          // TODO: print out control error output

          ROS_ERROR_STREAM(
              "ERROR RobotArm: Controller failed with error: " << m_controllers.at(i)->errorString().toStdString());
          controlFailed = true;
          break;
        }
      }
    }
    else
    {
      // just in this test
      m_q.tau = defaultControlValue();
    }
    if (controlFailed)
    {
      // catch the error in the dynamics, e.g. inf or nan
      stop();
      controlFailed = false;
      break;
    }

    // compute qppd,qpd,qd (dynamics)
    m_dynamicModel->update(m_rTime, m_q);
    if (m_dynamicModel->error())
    {
      // catch the error in the dynamics, e.g. inf or nan
      ROS_ERROR_STREAM(
          "ERROR RobotArm: Dynamic Model failed with error: " << m_dynamicModel->errorString().toStdString());
      stop();
      break;
    }

    // set qd or qpd (commInterface)
    // ROS_INFO_STREAM("Qf: "<<m_dynamicModel->qf().transpose());
    //        m_kinematicModel->publish_FK(m_dynamicModel->qf(),"_fdyn");

    //        VectorDOFd Dqp=m_qO.qp-m_dynamicModel->qpf();

    VectorDOFd qf = m_bfQ.filter(m_qO.q);

    m_kinematicModel->publish_FK(qf, "_freal");

    VectorDOFd Dq = m_dynamicModel->qf() - qf;  // m_qO.q;

    VectorDOFd Dqf = m_bfDQ.filter(Dq);

    VectorDOFd Qpf = m_bfQp.filter(m_qO.qp);

    VectorDOFd Dqp = m_dynamicModel->qpf() - m_qO.qp;
    //        VectorDOFd Dqp=m_dynamicModel->qpf()-Qpf;
    VectorDOFd Dqpf = Dqp;  // m_bfDQp.filter(Dqp);

    //        VectorDOFd Dv=m_dynamicModel->qpf() + m_Kp*Dqf + m_Kd*Dqpf;

    VectorDOFd velComp = m_Kp * Dq - m_Kd * Qpf;

    VectorDOFd velCompA = velComp;

    VectorDOFd velCompf = m_bfVelComp.filter(velComp);

    for (int i = 0; i < STD_DOF; i++)
    {
      if (std::abs(velComp(i)) < m_lowerJointVelThresh(i))
      {
        velComp(i) = 0.0;  // velCompf(i);
      }
    }

    VectorDOFd Dv = m_dynamicModel->qpf() + velComp;  // f;// + m_Kd*Dqpf;

    // TODO: Test the position int (with caution!!!)
    //        m_commIntf->setQd(m_dynamicModel->qf(),m_dynamicModel->qpf());

    //        m_qSendReal.qp=m_bfDv.filter(Dv);

    if (m_dynamicModel->getCtrlInterface() == RobotControllers::VELOCITY_INTF)
    {
      m_qSendReal.qp = m_dynamicModel->qp();
      m_qSendReal.q = m_dynamicModel->q();
    }
    else if (m_dynamicModel->getCtrlInterface() == RobotControllers::EFFORT_INTF)
    {
      m_qSendReal.qp = Dv;
      m_qSendReal.q += m_qSendReal.qp * m_controlPeriod;
    }
    else
    {
      m_error = false;
      std::stringstream s;
      s << "RobotArm(): The control interface --" << m_dynamicModel->getCtrlInterface() << "-- is not ENABLED yet.";

      m_errorString = s.str().c_str();
      ROS_ERROR_STREAM(m_errorString.toStdString());

      stop();
      break;
    }

    // Send the command to the real robot
    m_commIntf->setQd(m_qSendReal.q, m_qSendReal.qp);

    // save the current joint for external access
    m_mutex.lock();
    m_qOut = m_qO;
    m_mutex.unlock();

    /// Publish data
    // to test
    // sensor_msgs::JointState msg = m_qOut.toJointStateMsg(m_jointNames);
    tum_ics_ur_robot_msgs::RobotData msg;

    msg.header.stamp = m_rTime.tRc();

    msg.time = m_rTime.tD();

    velCompf(0) = m_lowerJointVelThresh(0);
    velCompf(1) = -m_lowerJointVelThresh(0);

    for (int i = 0; i < STD_DOF; i++)
    {
      msg.q[i] = velCompA(i);  // m_qO.q(i);
      msg.qp[i] = m_qO.qp(i);
      msg.qpp[i] = velComp(i);  // m_qO.qpp(i);

      msg.qf[i] = m_qSendReal.q(i);
      msg.qpf[i] = m_qSendReal.qp(i);
      msg.qppf[i] = velCompf(i);  // m_qO.qpp(i);

      //            msg.q[i]=m_dynamicModel->q()(i);
      //            msg.qp[i]=m_dynamicModel->qp()(i);
      //            msg.qpp[i]=m_dynamicModel->qpp()(i);

      //            msg.qf[i]=m_dynamicModel->qf()(i);
      //            msg.qpf[i]=m_dynamicModel->qpf()(i);
      //            msg.qppf[i]=m_dynamicModel->qppf()(i);

      msg.torques[i] = m_q.tau(i);
    }

    Affine3d Tef = m_kinematicModel->Tef_0(m_dynamicModel->qf());

    Vector3d xef = Tef.translation();

    msg.torques[0] = xef(0);
    msg.torques[1] = xef(1);
    msg.torques[2] = xef(2);

    msg.torques[3] = xef(0) - xef_old(0);
    msg.torques[4] = xef(1) - xef_old(1);
    msg.torques[5] = xef(2) - xef_old(2);

    xef_old = xef;

    pubJointsCtrl.publish(msg);

    ros::Time toc = ros::Time::now();

    double elapsedTime = toc.toSec() - tic.toSec();
    double loopPeriod = 8 * 1E-3;
    double delay = (m_controlPeriod - elapsedTime) * 1E6;

    tum_ics_ur_robot_msgs::FloatValue msg2;
    msg2.header.stamp = m_rTime.tRc();
    msg2.time = m_rTime.tD();
    msg2.value = toc.toSec() - tic.toSec();
    m_pubCycleTime.publish(msg2);

    if (elapsedTime > m_controlPeriod)
    {
      continue;
    }
    else if (elapsedTime > loopPeriod)
    {
      ROS_WARN_STREAM("Control loop period exceeds the limit: " << elapsedTime);
      continue;
    }

    if (TThread::isRtThread())
    {
      TThread::usleep(m_controlPeriodMS * 1000);
    }
    else
    {
      TThread::usleep((unsigned long)delay);
    }
  }

  m_robotStatePub->stop();
  m_stoppedThread = true;
  qDebug() << "RobotArm::run(): Real Thread exited";
}

bool RobotArm::parseConfig()
{
  QFileInfo config(m_configFilePath);
  if (!config.exists())
  {
    m_error = false;
    m_errorString = "RobotArm(): Error reading the config file. File: " + m_configFilePath + " does not exist!!!";
    ROS_ERROR_STREAM(m_errorString.toStdString());

    return false;
  }

  QSettings iniFile(m_configFilePath, QSettings::IniFormat);

  iniFile.beginGroup("ROBOT_EXECUTION_PARAMETERS");

  QString robotType = iniFile.value("ROBOT_TYPE", "sim").toString();
  if (!(robotType.compare("real")) || !(robotType.compare("REAL")))
  {
    m_realRobot = true;
  }
  else
  {
    m_realRobot = false;
  }

  ros::param::set("realRobot", m_realRobot);

  //    m_dof=iniFile.value("SOFTWARE_DOF", 6).toInt();
  // Period in ms
  m_controlPeriodMS = iniFile.value("CONTROL_PERIOD", 2).toDouble();
  // Period in s
  m_controlPeriod = m_controlPeriodMS / 1000.0;
  m_controlPeriod_2 = m_controlPeriod / 2.0;

  m_nControls = iniFile.value("NUMBER_CONTROLS", 0).toUInt();
  ROS_INFO_STREAM("Number of Controllers: " << m_nControls);

  for (uint i = 0; i < m_nControls; i++)
  {
    std::stringstream s;

    s << "CONTROL_W_" << i + 1;
    m_ctrlWeights.push_back(iniFile.value(s.str().c_str(), 1.0).toDouble());
    ROS_INFO_STREAM(s.str() << ": " << m_ctrlWeights.at(i));
    s.str("");
  }

  m_qHome.q(0) = iniFile.value("QHOME_1", -50.0).toDouble();
  m_qHome.q(1) = iniFile.value("QHOME_2", -90).toDouble();
  m_qHome.q(2) = iniFile.value("QHOME_3", 90).toDouble();
  m_qHome.q(3) = iniFile.value("QHOME_4", 0.0).toDouble();
  m_qHome.q(4) = iniFile.value("QHOME_5", 90.0).toDouble();
  m_qHome.q(5) = iniFile.value("QHOME_6", -90.0).toDouble();

  // For safety reasons these values are hardcoded
  m_qPark.q(0) = iniFile.value("QPARK_1", -50.0).toDouble();
  m_qPark.q(1) = iniFile.value("QPARK_2", -90).toDouble();
  m_qPark.q(2) = iniFile.value("QPARK_3", 90).toDouble();
  m_qPark.q(3) = iniFile.value("QPARK_4", 0.0).toDouble();
  m_qPark.q(4) = iniFile.value("QPARK_5", 90.0).toDouble();
  m_qPark.q(5) = iniFile.value("QPARK_6", -90.0).toDouble();

  m_qHome.q = DEG2RAD(m_qHome.q);
  m_qPark.q = DEG2RAD(m_qPark.q);

  // For safety reasons we initialize q_d to q_home
  m_qInit = m_qHome;

  ROS_INFO_STREAM("robotType: " << robotType.toStdString() << "(" << m_realRobot << ")");
  ROS_INFO_STREAM("dof: " << STD_DOF /*m_dof*/);
  ROS_INFO_STREAM("control Period (ms): " << m_controlPeriodMS);
  ROS_INFO_STREAM("Control Period (s): " << m_controlPeriod);
  ROS_INFO_STREAM("Half Period (s): " << m_controlPeriod_2);
  ROS_INFO_STREAM("Q Home: " << m_qHome.q.transpose());
  ROS_INFO_STREAM("Q PARK: " << m_qPark.q.transpose());
  ROS_INFO_STREAM("Q Init: " << m_qInit.q.transpose());

  m_groupName = iniFile.value("ARM_GROUP", "right").toString();
  ROS_INFO_STREAM("Group Name: " << m_groupName.toStdString());

  for (int i = 0; i < STD_DOF; i++)
  {
    std::stringstream s;
    s << "JOINT_NAME_" << i + 1;
    QString aux = iniFile.value(s.str().c_str(), "none").toString();
    m_jointNames.push_back(aux);

    ROS_INFO_STREAM(s.str().c_str() << ": " << m_jointNames.at(i).toStdString());
  }

  m_topicName = "/" + iniFile.value("JOINT_TOPIC", "right_arm_joint_states").toString();

  ROS_INFO_STREAM("JOINT_TOPIC: " << m_topicName.toStdString());

  iniFile.endGroup();

  iniFile.beginGroup("ROBOT_DYNAMIC_PARAMETERS");

  m_bfCutof = iniFile.value("CUT_OFF_FREQ", 20.0).toDouble();
  ROS_INFO_STREAM("Butter Filter cut off freq: " << m_bfCutof);

  iniFile.endGroup();

  m_bfDQ.setFilterParams(m_bfCutof, m_controlPeriod);
  m_bfDv.setFilterParams(m_bfCutof, m_controlPeriod);
  m_bfQ.setFilterParams(m_bfCutof, m_controlPeriod);
  m_bfDQp.setFilterParams(m_bfCutof, m_controlPeriod);
  m_bfQp.setFilterParams(m_bfCutof, m_controlPeriod);

  m_bfVelComp.setFilterParams(2.0, m_controlPeriod);

  return true;
}

VectorDOFd RobotArm::defaultControlValue()
{
  // NOTE get m_ctrlIntf from dynamics and compare it
  //    if(m_control->getCtrlIntf()==RobotControllers::POSITION_INTF) //We changed because in case
  // of an empty control list, then we will have segfault
  if (m_dynamicModel->getCtrlInterface() == RobotControllers::POSITION_INTF)
  {
    // the default control value for POSITION_INTF ctrl is
    // the current position
    return m_q.q;
  }
  else if (m_dynamicModel->getCtrlInterface() == RobotControllers::VELOCITY_INTF)
  {
    // the default control value for VELOCITY_INTF ctrl is
    // the Zero vector
    return VectorDOFd::Zero();
  }

  else if (m_dynamicModel->getCtrlInterface() == RobotControllers::EFFORT_INTF)
  {
    // the default control value for EFFORT_INTF ctrl is the
    // Zero vector
    return VectorDOFd::Zero();
  }
  else
  {
    m_error = false;
    std::stringstream s;
    s << "RobotArm(): The control interface --" << m_dynamicModel->getCtrlInterface() << "-- is not implemented yet.";

    m_errorString = s.str().c_str();
    ROS_ERROR_STREAM(m_errorString.toStdString());
    return VectorDOFd::Zero();
  }
}

bool RobotArm::reloadControlGains()
{
  std::string ns = "~pid_int";
  std::stringstream s;

  if (!ros::param::has(ns))
  {
    s << "Internal PID reloadControlGains(): Control gains not defined --" << ns
      << "--, did you load them in the rosparam server??";
    m_error = true;
    m_errorString = s.str().c_str();
    ROS_ERROR_STREAM(s.str());
    return false;
  }

  VDouble p;

  /////D GAINS

  s << ns << "/gains_d";
  ros::param::get(s.str(), p);

  if (p.size() < STD_DOF)
  {
    s.str("");
    s << "Internal PID reloadControlGains(): Wrong number of d_gains --" << p.size() << "--";
    m_error = true;
    m_errorString = s.str().c_str();
    return false;
  }
  for (int i = 0; i < STD_DOF; i++)
  {
    m_Kd(i, i) = p[i];
  }

  ROS_WARN_STREAM("int Kd: \n" << m_Kd);

  /////P GAINS
  s.str("");
  s << ns << "/gains_p";
  ros::param::get(s.str(), p);

  if (p.size() < STD_DOF)
  {
    s.str("");
    s << "PIDControl init(): Wrong number of p_gains --" << p.size() << "--";
    m_error = true;
    m_errorString = s.str().c_str();
    return false;
  }
  for (int i = 0; i < STD_DOF; i++)
  {
    m_Kp(i, i) = p[i];
  }
  ROS_WARN_STREAM("int Kp: \n" << m_Kp);

  /////I GAINS
  s.str("");
  s << ns << "/gains_i";
  ros::param::get(s.str(), p);

  if (p.size() < STD_DOF)
  {
    s.str("");
    s << "PIDControl init(): Wrong number of i_gains --" << p.size() << "--";
    m_error = true;
    m_errorString = s.str().c_str();
    return false;
  }
  for (int i = 0; i < STD_DOF; i++)
  {
    m_Ki(i, i) = p[i];
    //        m_Ki(i,i)=m_Kp(i,i)*m_Kp(i,i)/4.0;
  }

  ROS_WARN_STREAM("int Ki: \n" << m_Ki);

  /////I GAINS
  s.str("");
  s << ns << "/lo_thresh";
  ros::param::get(s.str(), p);

  if (p.size() < STD_DOF)
  {
    s.str("");
    s << "PIDControl init(): Wrong number of lo_thresh gains --" << p.size() << "--";
    m_error = true;
    m_errorString = s.str().c_str();
    return false;
  }
  for (int i = 0; i < STD_DOF; i++)
  {
    m_lowerJointVelThresh(i) = p[i];
    //        m_Ki(i,i)=m_Kp(i,i)*m_Kp(i,i)/4.0;
  }

  ROS_WARN_STREAM("int lower JointVel Thresholds: \n" << m_lowerJointVelThresh);

  return true;
}

bool RobotArm::addWhileRunning(RobotControllers::Controller *u)
{
  if (u->getCtrlIntf() >= RobotControllers::UNKNOWN_ITNF)
  {
    m_error = false;

    std::stringstream s;
    s << "RobotArm(): The control interface --" << u->getCtrlIntf() << "-- is not implemented yet.";
    m_errorString = s.str().c_str();

    ROS_ERROR_STREAM(m_errorString.toStdString());

    return false;
  }

  if (!m_controllers.isEmpty())
  {
    if (!(u->getCtrlIntf() == m_controllers.first()->getCtrlIntf()))
    {
      ROS_WARN_STREAM("The control could not be added to the list");
      ROS_WARN_STREAM("The target controller has a different CNTROL INTERFACE: "
                      << u->getCtrlIntf() << " vs " << m_controllers.first()->getCtrlIntf());
      return false;
    }

    if (u->getCtrlIntf() == RobotControllers::POSITION_INTF)
    {
      ROS_WARN_STREAM("The control could not be added to the list");
      ROS_WARN_STREAM("You can not add two controllers with POSITION INTF ("
                      << u->getCtrlIntf() << "). Use the control manager to fuse these two controllers.");
      return false;
    }
  }

  // Load gains
  u->init();

  // changing controller list is already pending
  while (m_pendingCtrlsFlag && isRunning())
  {
    ::usleep(10 * 1000);
  }

  // lock controller switch frame
  m_ctrlListMutex.lock();

  if (u->getCtrlSampleMode() == RobotControllers::EVENT_SAMPLE_MODE)
  {
    m_noEventMode = false;
  }

  // TODO: verify that event ctrl is at the top of the list, if not move it to the top. For now we do this manually!

  u->setLoaded(false);
  m_pendingCtrls = m_controllers;
  m_pendingCtrls.append(u);
  m_pendingLoadCtrl = true;
  m_pendingCtrlsFlag = true;

  m_ctrlListMutex.unlock();

  while (m_pendingCtrlsFlag && isRunning())
  {
    ::usleep(10 * 1000);
  }

  return true;
}

bool RobotArm::removeWhileRunning(RobotControllers::Controller *u)
{
  // changing controller list is already pending
  while (m_pendingCtrlsFlag && isRunning())
  {
    //        ::usleep(1000);
  }

  // lock controller switch frame
  m_ctrlListMutex.lock();

  m_pendingCtrls = m_controllers;
  m_pendingCtrls.removeOne(u);
  m_pendingLoadCtrl = false;
  m_pendingCtrlsFlag = true;

  m_noEventMode = true;
  for (int i = 0; i < m_controllers.size(); i++)
  {
    if (m_controllers.at(i)->getCtrlSampleMode() == RobotControllers::EVENT_SAMPLE_MODE)
    {
      m_noEventMode = false;
      break;
    }
  }

  m_ctrlListMutex.unlock();

  while (m_pendingCtrlsFlag && isRunning())
  {
    //        ::usleep(1000);
  }
  return true;
}

void RobotArm::reinit()
{
  // WE need to wait until the flag is effective (at least one complete round)
  m_initQreal = false;
  while (!(m_initQreal) && isRunning())
  {
    //        ::usleep(10*1000);
  }
}

void RobotArm::handleControllerListChange()
{
  // lock controller switch frame
  m_ctrlListMutex.lock();

  if (!m_pendingCtrlsFlag)
  {
    m_ctrlListMutex.unlock();
    return;
  }

  m_controllers = m_pendingCtrls;
  m_idle = m_controllers.isEmpty();

  if (m_pendingLoadCtrl)
  {
    m_dynamicModel->setCtrlInterface(m_controllers.first()->getCtrlIntf());
    m_controllers.last()->setLoaded(true);
  }
  else
  {
    m_controllers.last()->setLoaded(false);
  }

  m_pendingCtrlsFlag = false;

  m_ctrlListMutex.unlock();
}

void RobotArm::cleanup()
{
  if (m_dynamicModel != NULL)
  {
    delete m_dynamicModel;
    m_dynamicModel = NULL;
  }

  if (m_kinematicModel != NULL)
  {
    delete m_kinematicModel;
    m_kinematicModel = NULL;
  }

  if (m_commIntf != NULL)
  {
    delete m_commIntf;
    m_commIntf = NULL;
  }

  if (m_robotStatePub != NULL)
  {
    delete m_robotStatePub;
    m_robotStatePub = NULL;
  }
}

}  // namespace Robot
}  // namespace tum_ics_ur_robot_lli
