#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "ui_mapwindow.h"
#include "mapwidget.h"
#include "../src/map.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    openDialog(new QFileDialog(this)),
    mapPreviewWindow(new QMainWindow(this)), mapDisplay(NULL)
{
  ui->setupUi(this);

  connect(ui->robotBrowse, SIGNAL(clicked()), this, SLOT(robotBrowse()));
  connect(this, SIGNAL(robotPathChanged(QString)), ui->robotField, SLOT(setText(QString)));
  connect(ui->robotField, SIGNAL(textChanged(QString)), this, SLOT(setRobotPath(QString)));
  connect(ui->sensorBrowse, SIGNAL(clicked()), this, SLOT(sensorBrowse()));
  connect(this, SIGNAL(sensorPathChanged(QString)), ui->sensorField, SLOT(setText(QString)));
  connect(ui->sensorField, SIGNAL(textChanged(QString)), this, SLOT(setSensorPath(QString)));
  connect(ui->mapBrowse, SIGNAL(clicked()), this, SLOT(mapBrowse()));
  connect(this, SIGNAL(mapPathChanged(QString)), ui->mapField, SLOT(setText(QString)));
  connect(ui->mapField, SIGNAL(textChanged(QString)), this, SLOT(setMapPath(QString)));

  connect(ui->mapPreview, SIGNAL(clicked()), this, SLOT(mapPreview()));
  connect(ui->startButton, SIGNAL(clicked()), this, SLOT(startSimulation()));

  setRobotPath(QString("/Users/paul/Documents/Robotics/sim/robots/first"));
  setSensorPath(QString("/Users/paul/Documents/Robotics/sim/sensors"));
  setMapPath(QString("/Users/paul/Documents/Robotics/sim/maps/basic_map"));

  openDialog->setWindowModality(Qt::ApplicationModal);
  openDialog->setAcceptMode(QFileDialog::AcceptOpen);
  openDialog->setLabelText(QFileDialog::Accept, QString("Select"));
  openDialog->setDirectory(QString("/Users/paul/Document/Robotics/sim"));

  Ui::MapWindow * mapUI = new Ui::MapWindow;
  mapUI->setupUi(mapPreviewWindow);
  mapDisplay = mapUI->centralwidget;
  delete mapUI;
}

MainWindow::~MainWindow()
{
  delete openDialog;
  delete ui;
}

QString MainWindow::getRobotPath() const {
  return robotPath;
}

QString MainWindow::getSensorPath() const {
  return sensorPath;
}

QString MainWindow::getMapPath() const {
  return mapPath;
}

void MainWindow::setRobotPath(const QString& path) {
  if( robotPath != path ) {
    robotPath = path;
    emit robotPathChanged(robotPath);
  }
}

void MainWindow::setSensorPath(const QString& path) {
  if( sensorPath != path ) {
    sensorPath = path;
    emit sensorPathChanged(sensorPath);
  }
}

void MainWindow::setMapPath(const QString& path) {
  if( mapPath != path) {
    mapPath = path;
    emit mapPathChanged(mapPath);
  }
}

void MainWindow::mapBrowse()
{
  openDialog->setFileMode(QFileDialog::ExistingFile);
  openDialog->selectFile(mapPath);
  if( openDialog->exec() == QDialog::Accepted ) {
    setMapPath(openDialog->selectedFiles().first());
  }
}

void MainWindow::mapPreview()
{
  mapDisplay->setMap(sim::read_map(mapPath.toAscii().data()));
  mapPreviewWindow->show();
}

void MainWindow::sensorBrowse()
{
  openDialog->setFileMode(QFileDialog::Directory);
  openDialog->selectFile(sensorPath);
  if( openDialog->exec() == QDialog::Accepted ) {
    setSensorPath(openDialog->selectedFiles().first());
  }
}

void MainWindow::robotBrowse()
{
  openDialog->setFileMode(QFileDialog::ExistingFile);
  openDialog->selectFile(robotPath);
  if( openDialog->exec() == QDialog::Accepted ) {
    setRobotPath(openDialog->selectedFiles().first());
  }
}

void MainWindow::startSimulation()
{
}
