#include <QApplication>
#include <QDebug>
#include <QDir>

#include <ros/ros.h>
#include <tum_ics_ur_robot_manager/SkillList.h>

#include "../ConsoleReader.h"

using namespace tum_ics_ur_robot_manager;

int main(int argc, char **argv)
{
    ros::init(argc, argv,"skill_list_test",ros::init_options::AnonymousName);
    QApplication app(argc, argv);

    ros::NodeHandle n;
    ros::Rate r(30);

    ConsoleReader console;

    SkillList skillList;

    if(!skillList.load())
    {
        ROS_ERROR("Loading skill list from parameter server failed.");
        return -1;
    }

    QVector<QString> ctrls = QVector<QString>()
            << "GCtrl"
            << "JointCtrl";

    qDebug("%s",skillList.toString().toLatin1().data());

    skillList.update(ctrls);

    qDebug("%s",skillList.toString().toLatin1().data());

    int cnt=0;
    bool flag = true;
    QString line;
    bool hasLine;

    while(ros::ok())
    {
        line = console.getLine(&hasLine);
        if(hasLine && (line == "q"))
        {
            ros::shutdown();
        }


        QApplication::processEvents();
        ros::spinOnce();
        r.sleep();
        cnt++;
    }   

    qDebug("Clean exit");
    return 0;

}

