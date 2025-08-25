/* Controller base class
 * Authors: Emmanuel Dean and Florian Bergner
 * email: dean@tum.de, florian.bergner@tum.de
 *
 * v1: 2015
 */

#ifndef UR_ROBOT_LLI_CONTROLLER_H
#define UR_ROBOT_LLI_CONTROLLER_H

//#include<ur_robot_lli/EigenDefs.h>
//#include<ur_robot_lli/SharedDefs.h>

#include <tum_ics_ur_robot_lli/JointState.h>
#include <tum_ics_ur_robot_lli/RobotTime.h>

#include <tumtools/Math/MathTools.h>

#include <QMutex>

// forward declaration of Robot::Arm class to make it a friend
namespace tum_ics_ur_robot_lli{
namespace Robot{
class RobotArm;
}}


namespace tum_ics_ur_robot_lli{
namespace RobotControllers{

/*!
 * \brief The ControlInterface enum Defines the type of controller interface (level). This type affects how the
 * control output should be treated and integrated to the joint commanded position/velocity.
 */
enum ControlInterface
{
    POSITION_INTF,
    VELOCITY_INTF,
    EFFORT_INTF,
    UNKNOWN_ITNF
};

/*!
 * \brief The ControlType enum type of control. This type is only important when using skill manager.
 */
enum ControlType
{
    STANDARD_TYPE,
    SPLINE_TYPE,
    SKIN_TYPE,
    UNKNOWN_TYPE
};

/*!
 * \brief The ControlSampleMode enum type of control sampling mode. Currently, we have to types, continuous mode or
 * event mode. In general, we will only use continuous mode.
 */
enum ControlSampleMode
{
    NO_EVENT_SAMPLE_MODE,
    EVENT_SAMPLE_MODE,
    UNKNOWN_SAMPLE_MODE
};

/*!
 * \brief The ControlSpace enum defines the type of space the controller uses to define the error functions.
 * This is only important when using skill manager.
 */
enum ControlSpace
{
    JOINT_SPACE,
    CARTESIAN_SPACE
};

/*!
 * \brief The RunState enum states for the state machine.
 */
enum RunState
{
    STATE_INIT,
    STATE_START,
    STATE_UPDATE,
    STATE_STOP
};

/*!
 * \brief The Controller class This is the base class that provides the virtual methods for the Position, Velocity,
 * and Effort Controller class.
 */
class Controller: public Tum::Tools::MathTools
{
    friend class tum_ics_ur_robot_lli::Robot::RobotArm;  // changing loaded/unloaded flag

    // Member variables
protected:
    bool m_failed;      // set when the controller has failed (trigger an error)
    bool m_initialized; // set when the controller is initialized
    bool m_started;     // set when the controller has been started
    bool m_running;     // set when the controller is running
    bool m_finished;    // set when the controller is finished

    /*!
     * \brief m_runState defines the state where the controller is currently, e.g. STATE_STOP, STATE_INIT, etc.
     */
    uint m_runState;

    // TODO: add access functions (r/w)

    /*!
     * \brief m_errorString variable to store the text description of an error triggered in the class.
     */
    QString m_errorString;

    /*!
     * \brief m_error error flag.
     */
    bool m_error;

    /*!
     * \brief m_tau Computed control output. The the units of this variable depend on the ControlType.
     */
    VectorDOFd m_tau;

    /*!
     * \brief m_mutex mutex to guarantee a safe write/read process.
     */
    QMutex m_mutex;

    /*!
     * \brief m_tauExtFlag Flag to controll if an external controller should be added in this controller.
     */
    bool m_tauExtFlag;

    /*!
     * \brief m_ctrlSampleMode sample mode for this controller.
     */
    ControlSampleMode m_ctrlSampleMode; //Event or no_Event


private:

    /*!
     * \brief m_name uniquely defines the controller, must be assigned by
     * by the class which finally instantiates and bodies the abstract class
     */
    QString m_name;

    /*!
     * \brief m_ctrlIntf control interface type for this controller.
     */
    ControlInterface m_ctrlIntf;

    /*!
     * \brief m_ctrlType control type for this controller.
     */
    ControlType m_ctrlType;

    /*!
     * \brief m_ctrlSpace control space for this controller.
     */
    ControlSpace m_ctrlSpace;


    /*!
     * \brief m_weight the weight for the tau output of the update() function.
     */
    double m_weight;

    /*!
     * \brief m_loaded set by the robot arm class when the controller is added to the local controller list
     */
    bool m_loaded;



    VectorDOFd m_qGoal; // joint space goal
    Vector3d m_xGoal;   // cartesian position goal
    Matrix3d m_RGoal;   // cartesian orientation goal
    Vector6d m_XGoal;   // cartesian pose goal
    double m_timeGoal;  // time goal (to reach the desired goal)






    // Member Methods
public:

    /*!
     * \brief Controller default constructor.
     * \param name controller name (unique name to identify this controller).
     * \param intf controller interface.
     * \param type controller type.
     * \param space controller space.
     * \param weight controller weight.
     * \param sampleMode sampling mode for the controller.
     */
    Controller(const QString& name, ControlInterface intf, ControlType type, ControlSpace space, double weight=1.0, ControlSampleMode sampleMode=NO_EVENT_SAMPLE_MODE);
    virtual ~Controller() = 0;


    // called by the add function of the robot
    virtual bool init() = 0;

    // called by whoever who wants to start the controller (different to RosControl)
    virtual bool start() = 0;  // real-time safe (nothing which breaks the realtime, no sleeps etc.)

    // updates the tau (pos, vel, effort; depending on controller)
    // handles for now start, update, stop requests
    // real-time safe (nothing which breaks the realtime, no sleeps etc.)
    virtual VectorDOFd update(const RobotTime& time, const JointState &current) = 0;

    // called by whoever who wants to stop the controller (different to RosControl)
    virtual bool stop() = 0;


    /*!
     * \brief setQInit set the initial joint position for the controller.
     * \param qinit initial joint state.
     */
    virtual void setQInit(const JointState& qinit) = 0;

    /*!
     * \brief setQHome sets the home joint position for the controller.
     * \param qhome home joint state.
     */
    virtual void setQHome(const JointState& qhome) = 0;

    /*!
     * \brief setQPark sets the park joint position for the controller.
     * \param qhome park joint state.
     */
    virtual void setQPark(const JointState& qpark) = 0;


    // weight for the tau output of the controller
    void setWeight(double weight);


    /*!
     * \brief setGoalTime sets the desired goal time variable.
     * \param gTime target time in s.
     */
    void setGoalTime(double gTime);

    /*!
     * \brief setGoalJoints sets the desired joint position variable.
     * \param qGoal desired joint position.
     */
    void setGoalJoints(const VectorDOFd& qGoal);

    /*!
     * \brief setGoalPosition sets the desired cartesian position variable.
     * \param xGoal desired cartesian position.
     */
    void setGoalPosition(const Vector3d& xGoal);

    /*!
     * \brief setGoalOrientation sets the desired cartesian orientation matrix variable.
     * \param RGoal desired orientation matrix.
     */
    void setGoalOrientation(const Matrix3d& RGoal);

    /*!
     * \brief setGoalPose sets the desired Cartesian pose variable.
     * \param XGoal desired Cartesian pose in a 6x1 vector.
     */
    void setGoalPose(const Vector6d& XGoal);

    /*!
     * \brief setGoalPose setGoalPose sets the desired Cartesian pose variables.
     * \param xGoal desired Cartesian position.
     * \param RGoal desired Cartesian orientation.
     */
    void setGoalPose(const Vector3d& xGoal, const Matrix3d& RGoal);

    /*!
     * \brief enableTauExtFlag changes the extTau flag to true.
     */
    void enableTauExtFlag();

    /*!
     * \brief enableTauExtFlag changes the extTau flag to false.
     */
    void disableTauExtFlag();



    /*!
     * \brief name gets the name of this controller
     * \return string with the controller's name.
     */
    const QString& name() const;

    // controller state feedback
    bool isInitialized() const;     // the controller is initialized
    bool isStarted() const;         // the controller is started (start is finished)
    bool isRunning() const;         // the controller is running (after started)
    bool isFinished() const;        // the controller has finished after being stopped
    bool hasFailed() const;         // the controller has failed


    bool isLoaded() const;          // the controller is loaded to the robot arm


    /*!
     * \brief getCtrlIntf gets this controller interface.
     * \return  controller interface.
     */
    ControlInterface getCtrlIntf() const;

    /*!
     * \brief getCtrlType gets this controller type.
     * \return  controller type.
     */
    ControlType getCtrlType() const;

    /*!
     * \brief getCtrlSpace gets this controller space.
     * \return  controller sapace.
     */
    ControlSpace getCtrlSpace() const;

    /*!
     * \brief getCtrlSampleMode gets this controller sampling mode.
     * \return  controller sampling mode.
     */
    ControlSampleMode getCtrlSampleMode() const;


    /*!
     * \brief getCtrlType gets this controller weight factor.
     * \return  controller weight value.
     */
    double weight() const;

    /*!
     * \brief goalJoints gets the desired joint position.
     * \return desired joint position vector.
     */
    const VectorDOFd& goalJoints() const;

    /*!
     * \brief goalPosition gets the desired Cartesian position.
     * \return desired Cartesian position vector.
     */
    const Vector3d& goalPosition() const;

    /*!
     * \brief goalPose gets the desired Cartesian pose.
     * \return desired Cartesian pose vector.
     */
    const Vector6d& goalPose() const;

    /*!
     * \brief getTau gets the computed control output, it depends on the controller interface, e.g. position, vel, or torque.
     * \return computed control output.
     */
    VectorDOFd getTau();



    /*!
     * \brief getName gets this controller name.
     * \return controller's name.
     */
    const QString getName() const;

    //Set the name of the functions as the variables!!
    const VectorDOFd& qGoal() const;
    const Vector3d& xGoal() const;
    const Matrix3d& RGoal() const;
    const Vector6d& XGoal() const;
    const double timeGoal() const;

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
     * \brief satTau stauration function for the computed control output.
     * \param tauIn control output vector.
     * \param tauMax vector of maximum control values.
     */
    void satTau(VectorDOFd &tauIn, const VectorDOFd &tauMax);

protected:

private:
    /*!
     * \brief setLoaded sets the loaded flag. This function us called by the add/remove function of RobotArm.
     * \param loaded
     */
    void setLoaded(bool loaded);

};

inline Controller::~Controller(){}

inline double Controller::weight() const
{
    return m_weight;
}

inline const VectorDOFd& Controller::goalJoints() const
{
    return m_qGoal;
}

inline const Vector3d& Controller::goalPosition() const
{
    return m_xGoal;
}

inline const Vector6d& Controller::goalPose() const
{
    return m_XGoal;
}

inline VectorDOFd Controller::getTau()
{
    VectorDOFd local_tau;

    m_mutex.lock();
    local_tau=m_tau;
    m_mutex.unlock();

    return local_tau;
}

inline const QString Controller::getName() const
{
    return m_name;
}

inline const double Controller::timeGoal() const
{
    return m_timeGoal;
}

inline const VectorDOFd& Controller::qGoal() const
{
    return m_qGoal;
}

inline const Vector3d& Controller::xGoal() const
{
    return m_xGoal;
}

inline const Matrix3d& Controller::RGoal() const
{
    return m_RGoal;
}



inline const Vector6d& Controller::XGoal() const
{
    return m_XGoal;
}


}
}


















#endif // UR_ROBOT_LLI_CONTROLLER_H
