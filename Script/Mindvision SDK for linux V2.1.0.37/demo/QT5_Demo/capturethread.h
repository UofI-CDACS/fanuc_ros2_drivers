#ifndef CAPTURETHREAD_H
#define CAPTURETHREAD_H
#include <QThread>
#include <QImage>


class CaptureThread : public QThread
{
    Q_OBJECT
public:
    explicit CaptureThread(QObject *parent = 0);

public:
    void run();
    void stream();
    void pause();
    void stop();

signals:
    void captured(QImage img);
private:
    bool pause_status;
    bool quit;


    QVector<QRgb> grayColourTable;

public slots:

};

#endif // CAPTURETHREAD_H
