#include<tum_ics_ur_robot_lli/RobotInterface/DataUnpacker.h>

#include<QDataStream>
#include<QDebug>



namespace tum_ics_ur_robot_lli {
namespace RobotInterface{


const double DataUnpacker::COMM_PERIOD = 0.008;
const double DataUnpacker::COMM_PERIOD_2 = 0.004;
const double DataUnpacker::MULT_jointstate = 1.0E8;//10000.0;
const double DataUnpacker::MULT_time = 1000000.0;
const double DataUnpacker::MULT_blend = 1000.0;


DataUnpacker::DataUnpacker():
    m_ready(false)
{
    wayPointId = -1;
    for (int idx = 0; idx <= 5; idx++)
    {
        q[idx] = 0;
        qdot[idx] = 0;
        tau[idx] = 0;
    }
}


void DataUnpacker::appendNewData(QByteArray &newData)
{
    pendingData.append(newData);
}

void DataUnpacker::unpackData()
{
    QDataStream s(&pendingData, QIODevice::ReadWrite);
    bool finish = false;
    int header;
    bool successful = false;
    while(!finish)
    {
        successful = false;
        if(pendingData.size()>= int(sizeof(int)))
        {
            s>>header;
            if(header == MSG_OUT)
            {
                char* p = pendingData.data();
                p = p+4;//skip the header
                QString string = "";
                int cnt = 0;
                QString ch;
                while(cnt < (pendingData.size() - int(sizeof(int))))
                {
                    cnt++;
                    ch = QString::fromLatin1(p++,1);
                    string.append(ch);
                    if(ch == "~")
                    {
                        successful = true;
                        break;
                    }
                }
                if(successful)
                {
                    //update pending data
                    //QMutexLocker ml(&mutex_msg);
                    msg.append(string);
                    //ml.unlock();
                    pendingData.remove(0,sizeof(int)+string.length());
                }
                else
                {
                    finish = true;
                }
            }
            else if(header == MSG_JOINT_STATES)
            {

                if(pendingData.size() >= (1+6*3) * int(sizeof(int)))
                {
                    //store the data
                    s >> q_raw[0]>> q_raw[1]>> q_raw[2]>> q_raw[3]>> q_raw[4]>> q_raw[5];
                    s >> qdot_raw[0]>> qdot_raw[1]>> qdot_raw[2]>> qdot_raw[3]>> qdot_raw[4]>> qdot_raw[5];
                    s >> tau_raw[0]>> tau_raw[1]>> tau_raw[2]>> tau_raw[3]>> tau_raw[4]>> tau_raw[5];
                    //QMutexLocker ml(&mutex_config);
                    for(int idx = 0; idx <= 5; idx++)
                    {
                        q[idx] = q_raw[idx] / MULT_jointstate;
                        qdot[idx] = qdot_raw[idx] / MULT_jointstate;
                        tau[idx] = tau_raw[idx] / MULT_jointstate;
                    }
                    //ml.unlock();
                    m_ready=true;

//                    qDebug()<<"Ready: "<<q[0]<<", "<<q[2];

                    pendingData.remove(0,(1+6*3) * int(sizeof(int)));
                }
                else
                {
                    finish = true;
                }
            }
            else if (header == MSG_WAYPOINT_FINISHED)
            {
                if(pendingData.size() >= 2* int(sizeof(int)))
                {
                    s >> wayPointId;
                    pendingData.remove(0,2 * int(sizeof(int)));
                }
                else
                {
                    finish = true;
                }

            }
        }
        else
        {
            finish = true;
        }
    }
}

void DataUnpacker::getJointStates(double q_recv[6], double qdot_recv[6], double tau_recv[6])
{
    //QMutexLocker ml(&mutex_config);
    for(int idx = 0; idx <= 5; idx++)
    {
        q_recv[idx] = q[idx];
        qdot_recv[idx] = qdot[idx];
        tau_recv[idx] = tau[idx];
    }
}

JointState DataUnpacker::getJointState() const
{
    JointState qOut;

    for(unsigned int i=0;i<6;i++)
    {
        qOut.q(i)=q[i];
        qOut.qp(i)=qdot[i];
        qOut.tau(i)=tau[i];
    }

    return qOut;
}

QString DataUnpacker::getMsg()
{
    QString tmp = msg;
    //QMutexLocker ml(&mutex_msg);
    msg.clear();
    return tmp;
}

int DataUnpacker::getWayPointId()
{
    return wayPointId;
}

bool DataUnpacker::isReady() const
{
    //This flag will be changed to true with the first valid packet (JointState packet)
    return m_ready;
}

}
}
