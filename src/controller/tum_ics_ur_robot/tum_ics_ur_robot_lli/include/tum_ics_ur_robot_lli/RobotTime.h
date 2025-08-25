/* Robot Time class
 * Authors: Emmanuel Dean and Florian Bergner
 * email: dean@tum.de, florian.bergner@tum.de
 *
 * v1: 2015
 */
#ifndef ROBOTTIME_H
#define ROBOTTIME_H

#include<ros/ros.h>
#include<tumtools/Math/EigenDefs.h>
#include<QString>


namespace tum_ics_ur_robot_lli{

/*!
 * \brief The RobotTime class This class is a data container for ros::Time. It porvides
 * the initial ros::time when the class was started, the current ros::time, and the elapsed time in s.
 */
class RobotTime
{
    // Member variables
public:

private:

    /*!
     * \brief m_tRi Initial ros::Time.
     */
    ros::Time m_tRi;
    /*!
     * \brief m_tRc Current ros::Time.
     */
    ros::Time m_tRc;
    /*!
     * \brief m_tD elapsed time in s.
     */
    double m_tD;

    // Member Methods
public:
    /*!
     * \brief RobotTime Default constructor, it uses the current ros::Time to initialize the variables.
     */
    RobotTime();

    /*!
     * \brief RobotTime copy constructor.
     * \param time RobotTime to be copied.
     */
    RobotTime(const RobotTime &time);

    /*!
     * \brief RobotTime copy constructor. It initialize the RobotTime using the target ros::Time.
     * \param time initial ros::Time.
     */
    RobotTime(const ros::Time &time);
    ~RobotTime();

    /*!
     * \brief tRi gets the initial ros::Time when the class was initialized.
     * \return  intitial ros::Time
     */
    const ros::Time& tRi() const;

    /*!
     * \brief tRc gets the current ros::time.
     * \return current ros::Time
     */
    const ros::Time& tRc() const;

    /*!
     * \brief tD gets the elapsed time in s.
     * \return elapsed time in s.
     */
    const double tD() const;

    /*!
     * \brief update updates the RobotTime object using the current ros::Time.
     * \param timeC ros::Time which will be used as the initial time. By default is ros::Time::now().
     */
    void update(const ros::Time &timeC = ros::Time::now());

    /*!
     * \brief reset resets the RobotTime using the current ros::Time as the initial time.
     */
    void reset();

    /*!
     * \brief reset resets the RobotTime using the target ros::Time as the initial time.
     * \param time target ros::Time
     */
    void reset(const ros::Time &time);

    // Memeber Methods
private:

};



}

#endif // ROBOTTIME_H
