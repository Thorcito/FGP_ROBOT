#ifndef CONTROLEFFORT_H
#define CONTROLEFFORT_H

#include <tum_ics_ur_robot_lli/RobotControllers/Controller.h>



namespace tum_ics_ur_robot_lli{
namespace RobotControllers{

class ControlEffort: public Controller
{
    // Member variables
private:

    /*!
     * \brief m_tauExt Vector for make the computed control external
     */
    VectorDOFd m_tauExt;


    // Member Methods
public:

    /*!
     * \brief ControlEffort Default constructor.
     * \param name Controller name
     * \param type Controller type
     * \param space Controller space
     * \param weight Controller weight
     */
    ControlEffort(const QString name, ControlType type, ControlSpace space, double weight=1.0);
    virtual ~ControlEffort() = 0;


    /*!
     * \brief tauExt gets external tau vector to expose the control output.
     * \return Vector of external tau.
     */
    VectorDOFd tauExt() const;

    /*!
     * \brief setTauExt Set the external tau vectot
     * \param tExt target control vector.
     */
    void setTauExt(const VectorDOFd& tExt);



    // Memeber Methods
private:


};

}
}

#endif // CONTROLEFFORT_H
