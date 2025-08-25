#include "ConsoleReader.h"

#include <QTextStream>

ConsoleReader::ConsoleReader(QObject* parent) :
    QObject(parent),
    m_notifier(STDIN_FILENO, QSocketNotifier::Read)
{
    connect(&m_notifier,SIGNAL(activated(int)),this,SLOT(text()));

    this->moveToThread(&m_thread);
    m_thread.start();
}

ConsoleReader::~ConsoleReader()
{
    m_thread.exit();
    m_thread.wait();
}

bool ConsoleReader::hasLine()
{
    m_mutex.lock();
    bool empty = m_queue.isEmpty();
    m_mutex.unlock();

    return !empty;
}

QString ConsoleReader::getLine(bool *ok)
{
    m_mutex.lock();
    if(m_queue.isEmpty())
    {
        if(ok != 0)
        {
            *ok = false;
        }
        m_mutex.unlock();
        return QString();
    }

    QString line = m_queue.dequeue();
    m_mutex.unlock();

    if(ok != 0)
    {
        *ok = true;
    }

    return line;
}

void ConsoleReader::text()
{
    QTextStream ts(stdin);
    QString line = ts.readLine();

    m_mutex.lock();
    m_queue.enqueue(line);
    m_mutex.unlock();

    Q_EMIT message(line);
}
