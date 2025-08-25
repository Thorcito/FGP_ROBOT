#include <tum_ics_ur_robot_manager/ScriptLoader.h>

#include <unistd.h>

#include <QFile>
#include <QElapsedTimer>
#include <QHostAddress>

namespace tum_ics_ur_robot_manager{


ScriptLoader::ScriptLoader(
        const QString &robotIpAddr,
        quint16 scriptLoaderPort):
    QTcpSocket(),
    m_robotIpAddr(robotIpAddr),
    m_port(scriptLoaderPort),
    m_error(false)
{

}

ScriptLoader::~ScriptLoader()
{

}

bool ScriptLoader::load(const QString& scriptFilePath)
{
    QElapsedTimer timer;
    bool connected = false;

    timer.start();
    while((!connected))
    {

        if(timer.elapsed() > TOTAL_CONNECT_TIME_OUT)
        {
            m_error=true;
            m_errorString="ScriptLoader(): Total connection time reached! sorry :(, did you turn on the robot?";
            qDebug("%s",m_errorString.toLatin1().data());
            QTcpSocket::close();
            return false;
        }

        QTcpSocket::connectToHost(m_robotIpAddr, m_port);
        qDebug("ScriptLoader(): trying to connect to server");

        if (QTcpSocket::waitForConnected(CONNECT_TIME_OUT))
        {
            connected = true;
            QHostAddress hostAddr = QTcpSocket::localAddress();
            if (hostAddr != QHostAddress::Null)
            {
                qDebug("ScriptLoader(): Client connected on address %s:%d",
                       hostAddr.toString().toLatin1().data(),
                       QTcpSocket::localPort());
            }

            QString prog = readFromFile(scriptFilePath);
            writeString(prog);
        }
        else
        {
            usleep(500*1000);
            qDebug("ScriptLoader(): Client socket failed to connect. %s",
                   QTcpSocket::errorString().toLatin1().data());

            qDebug("Trying again ...");
        }
    }

    qDebug("ScriptLoader(): Client finished. Succeeded in sending the script code.");
    QTcpSocket::close();
    return true;
}

bool ScriptLoader::load()
{
    QElapsedTimer timer;
    bool connected = false;

    timer.start();
    while((!connected))
    {

        if(timer.elapsed() > TOTAL_CONNECT_TIME_OUT)
        {
            m_error=true;
            m_errorString="ScriptLoader(): Total connection time reached! sorry :(, did you turn on the robot?";
            qDebug("%s",m_errorString.toLatin1().data());
            QTcpSocket::close();
            return false;
        }

        QTcpSocket::connectToHost(m_robotIpAddr, m_port);
        qDebug("ScriptLoader(): trying to connect to server");

        if (QTcpSocket::waitForConnected(CONNECT_TIME_OUT))
        {
            connected = true;
            QHostAddress hostAddr = QTcpSocket::localAddress();
            if (hostAddr != QHostAddress::Null)
            {
                qDebug("ScriptLoader(): Client connected on address %s:%d",
                       hostAddr.toString().toLatin1().data(),
                       QTcpSocket::localPort());
            }

            QString prog = "def driverProg():\n  HOSTNAME             = \"192.168.1.3\"\n  MANAGER_PORT         = 50001\n  TRAJ_PORT            = 50002\n  GRIPPER_PORT         = 50003\n\n  START_GRIPPER_CMD    = 0\n  STOP_GRIPPER_CMD     = 1\n  START_TRAJ_POS_CMD   = 2\n  STOP_TRAJ_POS_CMD    = 3\n  START_TRAJ_VEL_CMD   = 4\n  STOP_TRAJ_VEL_CMD    = 5\n  QUIT_CMD             = 6\n  \n  ACTIVATE_GRIPPER_CMD    = 20\n  DEACTIVATE_GRIPPER_CMD  = 21\n  OPEN_GRIPPER_CMD        = 22\n  CLOSE_GRIPPER_CMD       = 23\n\n  OK_RESP      = 1\n  ERROR_RESP   = 2\n\n  GRIPPER_THREAD_STATE   = False\n  TRAJ_POS_THREAD_STATE  = False\n  TRAJ_VEL_THREAD_STATE  = False\n\n  MULT_jointstate = 100000000.0\n  MULT_time = 1000000.0\n  MULT_blend = 1000.0\n\n  VEL_MAX = 2.0\n\n  q=[0,0,0,0,0,0]\n  r=0\n\n  TRAJ_POS_FLAG = False\n  TRAJ_VEL_FLAG = False \n\n  qp=[0,0,0,0,0,0]\n\n\n  def send_resp(msg):\n    enter_critical\n    socket_send_int(msg)\n    exit_critical\n  end\n\n  def setGripperThreadState(newVal):\n    enter_critical\n    GRIPPER_THREAD_STATE = newVal\n    exit_critical\n  end\n\n  def setTrajPosThreadState(newVal):\n    enter_critical\n    TRAJ_POS_THREAD_STATE = newVal\n    exit_critical\n  end\n\n  def setTrajPosFlag(newVal):\n    enter_critical\n    TRAJ_POS_FLAG = newVal\n    exit_critical\n  end\n\n  def setTrajVelFlag(newVal):\n    enter_critical\n    TRAJ_VEL_FLAG = newVal\n    exit_critical\n  end\n\n  def setTrajVelThreadState(newVal):\n    enter_critical\n    TRAJ_VEL_THREAD_STATE = newVal\n    exit_critical\n  end \n\n  thread gripperThread():\n    textmsg(\"Started gripper thread.\")\n    textmsg(\"Wait for connection to gripper server ...\")\n    ret = socket_open(HOSTNAME, GRIPPER_PORT, \"socket_gripper\")\n    if ret == False:\n      send_resp(ERROR_RESP)\n      return True\n    else:\n      send_resp(OK_RESP)\n    end\n    textmsg(\"Gripper socket opened\")\n\n    # init: open, inactive\n    set_tool_voltage(12)\n    set_digital_out(9,False)\n    set_digital_out(8,True)\n    sleep(1)\n    set_tool_voltage(0)\n    set_digital_out(9,True)\n\n    while GRIPPER_THREAD_STATE:\n      params = socket_read_binary_integer(1, \"socket_gripper\")\n      if params[0] == 0:\n        textmsg(\"Socket gripper read timed out.\")\n      else:\n        mtype = params[1]\n        if mtype == ACTIVATE_GRIPPER_CMD:\n          textmsg(\"Received ACTIVATE_GRIPPER_CMD\")\n          set_tool_voltage(12)\n          set_digital_out(9,False)\n\n        elif mtype == DEACTIVATE_GRIPPER_CMD:\n          textmsg(\"Received DEACTIVATE_GRIPPER_CMD\")\n          set_tool_voltage(0)\n          set_digital_out(9,True)\n\n        elif mtype == OPEN_GRIPPER_CMD:\n          textmsg(\"Received OPEN_GRIPPER_CMD\")\n          set_digital_out(8,True)\n\n        elif mtype == CLOSE_GRIPPER_CMD:\n          textmsg(\"Received CLOSE_GRIPPER_CMD\")\n          set_digital_out(8,False)\n\n        else:\n          textmsg(\"Gripper: Unknown command\")\n        end\n      end\n    end\n\n    textmsg(\"Close gripper socket ...\")\n    socket_close(\"socket_gripper\")\n    textmsg(\"Socket closed.\")\n    textmsg(\"Stopped gripper thread.\")\n  end\n\n  thread trajPosThread():\n    textmsg(\"Started traj pos thread.\")\n    textmsg(\"Wait for connection to traj server ...\")\n    ret = socket_open(HOSTNAME, TRAJ_PORT, \"socket_traj\")\n    if ret == False:\n      setTrajPosThreadState(False)\n      send_resp(ERROR_RESP)\n      return True\n    else:\n      send_resp(OK_RESP)\n    end\n    textmsg(\"Traj pos socket opened.\")\n\n    while TRAJ_POS_THREAD_STATE:\n      params = socket_read_binary_integer(1+6+4, \"socket_traj\")\n      if params[0] == 0:\n        textmsg(\"Socket traj pos read timed out.\")\n      else:\n        # Unpacks the parameters\n        waypoint_id = params[1]\n        #textmsg(\"waypoint_id: \", waypoint_id)\n        q = [params[2] / MULT_jointstate,\n             params[3] / MULT_jointstate,\n             params[4] / MULT_jointstate,\n             params[5] / MULT_jointstate,\n             params[6] / MULT_jointstate,\n             params[7] / MULT_jointstate]\n\n        a = params[8] / MULT_jointstate\n        v = params[9] / MULT_jointstate\n        t = params[10] / MULT_time\n        r = params[11] / MULT_blend\n\n      if TRAJ_POS_FLAG == False:\n        setTrajPosFlag(True)\n      else:\n      end\n\n      end\n    end\n    textmsg(\"Close traj socket ...\")\n    socket_close(\"socket_traj\")\n    textmsg(\"Socket closed.\")\n    textmsg(\"Stopped traj pos thread.\")\n  end\n\n  thread trajPosCmdThread():\n    textmsg(\"Started traj pos cmd thread.\")\n\n    while TRAJ_POS_THREAD_STATE:\n\n      if TRAJ_POS_FLAG == False:\n        sleep(0.008)\n        continue\n      else:\n      end\n\n      textmsg(\"q\")\n      textmsg(q[0])\n      textmsg(q[1])\n      textmsg(q[2])\n      textmsg(q[3])\n      textmsg(q[4])\n      textmsg(q[5])\n      sleep(1)\n\n      #movej(q, 100.0, 27.0, 0.002, r)\n    end\n    textmsg(\"Stopped traj pos cmd thread.\")\n  end\n\n  thread trajVelThread():\n    textmsg(\"Started traj vel thread.\")\n    textmsg(\"Wait for connection to traj server ...\")\n    ret = socket_open(HOSTNAME, TRAJ_PORT, \"socket_traj\")\n    if ret == False:\n      setTrajVelThreadState(False)\n      send_resp(ERROR_RESP)\n      return True\n    else:\n      send_resp(OK_RESP)\n    end\n    textmsg(\"Traj vel socket opened.\")\n\n    while TRAJ_VEL_THREAD_STATE:\n      params = socket_read_binary_integer(1+6+4, \"socket_traj\")\n      if params[0] == 0:\n        textmsg(\"Socket traj vel read timed out.\")\n      else:\n        # Unpacks the parameters\n        waypoint_id = params[1]\n        #textmsg(\"waypoint_id: \", waypoint_id)\n        if waypoint_id > 2:\n          qp = [params[2] / MULT_jointstate,\n               params[3] / MULT_jointstate,\n               params[4] / MULT_jointstate,\n               params[5] / MULT_jointstate,\n               params[6] / MULT_jointstate,\n               params[7] / MULT_jointstate]\n\n          if TRAJ_VEL_FLAG == False:\n            setTrajVelFlag(True)\n          else:\n          end\n\n        else:\n        qp = [0,0,0,0,0,0]\n        textmsg(\"ZerosRead\")\n        end\n\n\n\n      end\n    end\n    qp = [0,0,0,0,0,0]\n    textmsg(\"Close traj socket ...\")\n    socket_close(\"socket_traj\")\n    textmsg(\"Socket closed.\")\n    textmsg(\"Stopped traj vel thread.\")\n  end\n\n  thread trajVelCmdThread():\n    textmsg(\"Started traj vel cmd thread.\")\n    qp = [0,0,0,0,0,0]\n\n    while TRAJ_VEL_THREAD_STATE:\n\n      if TRAJ_VEL_FLAG == False:\n        sleep(0.008)\n      else:\n        i = 0\n        while i < 6:\n          if qp[i] > VEL_MAX:\n            #textmsg(\"qp\")\n            #textmsg(qp[0])\n            #textmsg(qp[1])\n            #textmsg(qp[2])\n            #textmsg(qp[3])\n            #textmsg(qp[4])\n            #textmsg(qp[5])\n            qp = [0,0,0,0,0,0] \n            break\n          else:\n          end\n          i = i + 1\n        end\n        #qp=[0,0,0,0,0,0]\n        speedj(qp,11,0.008)\n      end\n    end\n      qpl = [0,0,0,0,0,0]\n      speedj(qpl,11,0.008)\n    textmsg(\"Stopped traj vel cmd thread.\")\n  end\n\n  \n  socket_open(HOSTNAME, MANAGER_PORT, \"socket_0\")\n  sync()\n\n  while True:\n    params = socket_read_binary_integer(1)\n    \n    if params[0] == 0:\n      #textmsg(\"Main Thread Received nothing\")\n    elif params[0] > 1:\n      textmsg(\"Manager: The cmd is too long.\")\n    else:\n      mtype = params[1]\n      if mtype == QUIT_CMD:\n        textmsg(\"Mgr: QUIT_CMD\")\n        break\n\n      elif mtype == START_GRIPPER_CMD:\n        textmsg(\"Mgr: START_GRIPPER_CMD\")\n        setGripperThreadState(True)\n        thread_gripper = run gripperThread()\n\n      elif mtype == STOP_GRIPPER_CMD:\n        textmsg(\"Mgr: STOP_GRIPPER_CMD\")\n        setGripperThreadState(False)\n        send_resp(OK_RESP)\n\n      elif mtype == START_TRAJ_POS_CMD:\n        textmsg(\"Mgr: START_TRAJ_POS_CMD\")\n        setTrajPosThreadState(True)\n        setTrajPosFlag(False) \n        thread_traj_pos = run trajPosThread()\n        thread_traj_pos_cmd = run trajPosCmdThread()\n\n      elif mtype == STOP_TRAJ_POS_CMD:\n        textmsg(\"Mgr: STOP_TRAJ_POS_CMD\")\n        setTrajPosThreadState(False)\n        send_resp(OK_RESP)\n\n      elif mtype == START_TRAJ_VEL_CMD:\n        textmsg(\"Mgr: START_TRAJ_VEL_CMD\")\n        setTrajVelFlag(False) \n        setTrajVelThreadState(True)\n        thread_traj_vel = run trajVelThread()\n        thread_traj_vel_cmd = run trajVelCmdThread()\n\n      elif mtype == STOP_TRAJ_VEL_CMD:\n        textmsg(\"Mgr: STOP_TRAJ_VEL_CMD\")\n        setTrajVelThreadState(False)\n        send_resp(OK_RESP)\n\n      else:\n        textmsg(\"Manager: Unknown command\")\n        textmsg(mtype)\n      end\n    end\n  end\n\n  # Stop threads\n  setGripperThreadState(False)\n  setTrajPosThreadState(False)\n  setTrajVelThreadState(False)\n  sleep(1)\n\n  textmsg(\"Close manager socket ...\")\n  socket_close(\"socket_0\")\n  textmsg(\"Socket closed.\")\n\n  sleep(1)\n  textmsg(\"Clean exit.\")\nend\ndriverProg()";
            writeString(prog);
        }
        else
        {
            usleep(500*1000);
            qDebug("ScriptLoader(): Client socket failed to connect. %s",
                   QTcpSocket::errorString().toLatin1().data());

            qDebug("Trying again ...");
        }
    }

    qDebug("ScriptLoader(): Client finished. Succeeded in sending the script code.");
    QTcpSocket::close();
    return true;
}

bool ScriptLoader::error()
{
    return m_error;
}
const QString& ScriptLoader::errorString()
{
    return m_errorString;
}

QString ScriptLoader::readFromFile(const QString& filePath)
{
    FILE *fp;

    fp = fopen(filePath.toLatin1().data(),"r");
    if(fp == NULL)
    {
        m_error = true;
        m_errorString = QString().sprintf("Opening file '%s' failed.",filePath.toLatin1().data());
        qDebug("%s",m_errorString.toLatin1().data());

        return "";
    }

    char ch;
    QString data;

    while((ch = fgetc(fp)) != EOF)
    {
        data.append(ch);
    }

    fclose(fp);
    return data;

// THIS IS NOT WORKING IN RT THREAD!

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

//    return data;
}

void ScriptLoader::writeString(const QString &s)
{
    if (s.length() > 0)
    {
        QTcpSocket::write(s.toLatin1());
        if (!QTcpSocket::waitForBytesWritten())
        {
            qDebug("Socket write failed");
        }
    }
}


}
