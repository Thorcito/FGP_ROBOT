#ifndef TUM_COMMON_H
#define TUM_COMMON_H

#include <tum_ics_msgs/PlaySkill.h>
#include <tum_ics_msgs/StopSkill.h>
#include <tum_ics_msgs/GetSkillState.h>


namespace Tum{
namespace Common{


enum SkillState
{
    INITIALIZED_STATE,
    STARTED_STATE,
    RUNNING_STATE,
    FINISHED_STATE,
    FAILED_STATE,
    UNKNOWN_STATE
};

enum PlaySkillReply
{
    ACCEPTED_SKILLR,
    REJECTED_SKILLR,
    PENDING_SKILLR
};


}
}


#endif // TUM_COMMON_H
