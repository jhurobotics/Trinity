#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QString>
#include <QFileDialog>

namespace Ui {
    class MainWindow;
}

#ifndef MAPWIDGET_H
class MapWidget;
#endif // MAPWIDGET_H

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    QFileDialog * openDialog;

    QString robotPath;
    QString sensorPath;
    QString mapPath;

    QMainWindow * mapPreviewWindow;
    MapWidget * mapDisplay;

public:
    QString getRobotPath() const;
    QString getSensorPath() const;
    QString getMapPath() const;

public slots:
    void setRobotPath(const QString& path);
    void setSensorPath(const QString& path);
    void setMapPath(const QString& path);
    void mapBrowse();
    void mapPreview();
    void sensorBrowse();
    void robotBrowse();
    void startSimulation();

signals:
    void robotPathChanged(const QString& path);
    void sensorPathChanged(const QString& path);
    void mapPathChanged(const QString& path);
};

#endif // MAINWINDOW_H
