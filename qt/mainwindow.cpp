#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "mapwidget.h"
#include "../src/map.h"
#include "simwidget.h"
#include "../src/simulation.h"
#include "../src/robot.h"
#include "../src/setup.h"
using namespace sim;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    openDialog(new QFileDialog(this)),
    mapPreviewWindow(new QMainWindow()), mapDisplay(NULL),
    simWindow(new QMainWindow()), simDisplay(NULL)
{
  ui->setupUi(this);

  connect(ui->robotBrowse, SIGNAL(clicked()), this, SLOT(robotBrowse()));
  connect(this, SIGNAL(robotPathChanged(QString)), ui->robotField, SLOT(setText(QString)));
  connect(ui->robotField, SIGNAL(editingFinished()), this, SLOT(updateRobotPath()));
  connect(ui->sensorBrowse, SIGNAL(clicked()), this, SLOT(sensorBrowse()));
  connect(this, SIGNAL(sensorPathChanged(QString)), ui->sensorField, SLOT(setText(QString)));
  connect(ui->sensorField, SIGNAL(editingFinished()), this, SLOT(updateSensorPath()));
  connect(ui->mapBrowse, SIGNAL(clicked()), this, SLOT(mapBrowse()));
  connect(this, SIGNAL(mapPathChanged(QString)), ui->mapField, SLOT(setText(QString)));
  connect(ui->mapField, SIGNAL(editingFinished()), this, SLOT(updateMapPath()));

  connect(ui->mapPreview, SIGNAL(clicked()), this, SLOT(mapPreview()));
  connect(ui->startButton, SIGNAL(clicked()), this, SLOT(startSimulation()));

  //change later for myself.

  setRobotPath(QString("/Users/paul/Desktop/Trinity/robots/first"));
  setSensorPath(QString("/Users/paul/Desktop/Trinity/sensors"));
  setMapPath(QString("/Users/paul/Desktop/Trinity/maps/basic_map"));

  openDialog->setWindowModality(Qt::ApplicationModal);
  openDialog->setAcceptMode(QFileDialog::AcceptOpen);
  openDialog->setLabelText(QFileDialog::Accept, QString("Select"));
  openDialog->setDirectory(QString("/Users/paul/Document/Robotics/sim"));

  setupMapWindow();
  setupSimWindow();

  setWindowTitle(QString("Simulation Setup"));
  simWindow->setWindowTitle(QString("Simulation"));
}

MainWindow::~MainWindow()
{
  delete openDialog;
  delete ui;
  delete mapPreviewWindow;
  delete simWindow;
}

void MainWindow::setupMapWindow() {
  mapPreviewWindow->resize(258, 258);
  mapDisplay = new MapWidget(mapPreviewWindow);
  mapPreviewWindow->setCentralWidget(mapDisplay);
}

void MainWindow::setupSimWindow() {
  simWindow->resize(650, 700);
  QWidget * centralwidget = new QWidget(simWindow);
  simDisplay = new sim::SimWidget(centralwidget);
  simDisplay->setGeometry(QRect(0, 0, 650, 650));
  QPushButton *pushButton = new QPushButton(centralwidget);
  pushButton->setText("Step Once");
  pushButton->setGeometry(QRect(120, 660, 114, 32));
  QPushButton *pushButton_2 = new QPushButton(centralwidget);
  pushButton_2->setText("Step 1 Second");
  pushButton_2->setGeometry(QRect(370, 660, 121, 32));
  simWindow->setCentralWidget(centralwidget);

  QObject::connect(pushButton, SIGNAL(clicked()), simDisplay, SLOT(stepOnce()));
  QObject::connect(pushButton_2, SIGNAL(clicked()), simDisplay, SLOT(stepSecond()));
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

void MainWindow::updateRobotPath() {
  setRobotPath(ui->robotField->text());
}

void MainWindow::setSensorPath(const QString& path) {
  if( sensorPath != path ) {
    sensorPath = path;
    emit sensorPathChanged(sensorPath);
  }
}

void MainWindow::updateSensorPath() {
  setSensorPath(ui->sensorField->text());
}

void MainWindow::setMapPath(const QString& path) {
  if( mapPath != path) {
    mapPath = path;
    emit mapPathChanged(mapPath);
  }
}

void MainWindow::updateMapPath() {
  setMapPath(ui->mapField->text());
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
  robot::AbstractRobot * bot;
  if(ui->cppButton->isChecked() ) {
    bot = new_robot(robot::SONAR);
  }
  else if( ui->cppButton_2->isChecked() ) {
    bot = new_robot(robot::CPP_1);
  }
  else if( ui->cppButton_3->isChecked() ) {
    bot = new_robot(robot::CPP_2);
  }
  
  setupflags_t setupflags = 0;
  
  if( ui->motors_sim->isChecked() ) {
    setupflags |= MOTORS_SIM;
  }
  else if( ui->motors_arduino->isChecked() ) {
    setupflags |= MOTORS_ARDUINO;
  }
  else if( ui->motors_maestro->isChecked() ) {
    setupflags |= MOTORS_MAESTRO;
  }
  
  if( ui->sonar_sim->isChecked() ) {
    setupflags |= SONAR_SIM;
  }
  else if( ui->sonar_arduino->isChecked() ) {
    setupflags |= SONAR_ARDUINO;
  }
  
  if( ui->encoder_sim->isChecked() ) {
    setupflags |= ENCODER_SIM;
  }
  else if( ui->encoder_arduino->isChecked() ) {
    setupflags |= ENCODER_ARDUINO;
  }
  
  simDisplay->setWorld(create_world(bot, mapPath.toAscii().data(),
                        robotPath.toAscii().data(), sensorPath.toAscii().data(),
                                    setupflags));
  
  //simDisplay->setWorld(sim::create_simulation(bot, mapPath.toAscii().data(),
  //                          robotPath.toAscii().data(), sensorPath.toAscii().data()));
  simWindow->show();
}
