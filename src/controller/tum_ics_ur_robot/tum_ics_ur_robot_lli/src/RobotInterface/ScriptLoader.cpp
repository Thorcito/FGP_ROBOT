#include<tum_ics_ur_robot_lli/RobotInterface/ScriptLoader.h>
#include<QFile>
#include<QElapsedTimer>
#include<unistd.h>

namespace tum_ics_ur_robot_lli{
namespace RobotInterface{

ScriptLoader::ScriptLoader(const QString &robotIpAddr, const QString &scriptFileName, quint16 robotPort):
    QTcpSocket(),TcpSocketTools(this), m_error(false), m_stop(false)
{
    QElapsedTimer timer;




    bool connected = false;

    timer.start();
    while((!connected))
    {

        if(timer.elapsed()>TOTAL_CONNECT_TIME_OUT)
        {
            m_error=true;
            m_errorString="ScriptLoader(): Total connection time reached! sorry :(, did you turn on the robot?";
            qDebug()<<m_errorString;
            QTcpSocket::close();
            return;
        }

        if(m_stop)
        {
            m_error=true;
            m_errorString="ScriptLoader(): Client connection ABORTED! sorry :(, did you turn on the robot?";
            qDebug()<<m_errorString;
            QTcpSocket::close();
            return;
        }

        QTcpSocket::connectToHost(robotIpAddr, robotPort);
        qDebug()<<"ScriptLoader(): trying to connect to server";

        if (QTcpSocket::waitForConnected(CONNECT_TIME_OUT))
        {
            connected = true;
            QHostAddress hostAddr = QTcpSocket::localAddress();
            if (hostAddr != QHostAddress::Null)
            {
                qDebug()<<QString("ScriptLoader(): Client connected on address %1:%2")\
                          .arg(hostAddr.toString()).arg(QTcpSocket::localPort());
            }



            QString prog = readFromFile(scriptFileName);

            TcpSocketTools::writeString(prog);


        }
        else
        {
            sleep(1);
            qDebug()<<"ScriptLoader(): Client socket failed to connect."+QTcpSocket::errorString();
            qDebug()<<"Trying again ...";
        }

    }

    qDebug()<<"ScriptLoader(): Client finished. Succeeded in sending the script code.";
    QTcpSocket::close();

}
ScriptLoader::~ScriptLoader(){}
bool ScriptLoader::error()
{
    return m_error;
}
const QString& ScriptLoader::errorString()
{
    return m_errorString;
}

void ScriptLoader::stop()
{
    m_stop=true;
}

QString ScriptLoader::readFromFile(const QString& path)
{


    FILE *fp;

    fp=fopen(path.toLatin1().data(),"r");

    if( fp == NULL )
    {
        qDebug("ScriptLoader File: '%s'", path.toLatin1().data());
        ROS_ERROR("Error while opening the file.\n");
    }
    char ch;

        QString data;

    while( ( ch = fgetc(fp) ) != EOF )
    {
        data.append(ch);

    }

    fclose(fp);

//    QFile file(path);
//    if(!file.open(QIODevice::ReadOnly))
//    {

//        qDebug()<<file.errorString();
//        m_error=true;
//        m_errorString=file.errorString();
//    }


//    QTextStream in(&file);


////    QString data;

//    while(!in.atEnd()) {

//        data = in.readAll();

//    }

//    file.close();

    return data;
}
}
}

