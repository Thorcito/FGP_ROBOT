#ifndef CONSOLEREADER_H
#define CONSOLEREADER_H


#include <QObject>
#include <QSocketNotifier>
#include <QThread>
#include <QMutex>
#include <QQueue>

#include <unistd.h>

class ConsoleReader : public QObject
{
    Q_OBJECT

private:
    QSocketNotifier m_notifier;
    QQueue<QString> m_queue;
    QThread m_thread;
    QMutex m_mutex;

public:
    explicit ConsoleReader(QObject *parent = 0);
    ~ConsoleReader();

    bool hasLine();
    QString getLine(bool *ok);

private Q_SLOTS:
    void text();

Q_SIGNALS:
    void message(QString s);


};





#endif // CONSOLEREADER_H
