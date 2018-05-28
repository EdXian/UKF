#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "iostream"
#include "math.h"
#include "ukf.h"

using Eigen::MatrixXd;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    srand(time(NULL));
    ui->setupUi(this);
    ui->customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    ui->customPlot->xAxis->setRange(-10,10);
    ui->customPlot->yAxis->setRange(-10,10);
    ui->customPlot->xAxis->setLabel("x");
    ui->customPlot->yAxis->setLabel("y");

     ui->customPlot->legend->setVisible(true);

    ukf ukf1;
    double velocity = 0.0 ;
    double pos = 0.0;
    double measure=0.0;
    double dt =0.02;
    double T=0.0;

    QVector<double> x(501), y(501) ,z(501),w(501),p(501),q(501); // initialize with entries 0..100
    for (int i=0; i<501; ++i)
    {

      T+=dt;
      pos = pos+ velocity* dt;
      measure = pos + (rand()%100-50)*0.001;
      velocity = cos(pos)+0.99*velocity;


      ukf1.predict();
      ukf1.correct(measure);

      x[i] = T;
      y[i] = pos; // let's plot a quadratic function
      z[i] = velocity;
      w[i] = measure;
      p[i] = ukf1.x_hat(0);
      q[i] = ukf1.x(1);
    }
    // create graph and assign data to it:

    ui->customPlot->addGraph();
    ui->customPlot->graph(0)->setData(x, w);
    ui->customPlot->graph(0)->setName("measurement");
    ui->customPlot->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc, 5));
    ui->customPlot->graph(0)->setLineStyle(QCPGraph::lsNone);

    ui->customPlot->addGraph();
    ui->customPlot->graph(1)->setData(x, p);
    ui->customPlot->graph(1)->setName("UKF");

    QPen pen;
    pen.setColor(Qt::black);
    ui->customPlot->graph(1)->setPen(pen);

//    ui->customPlot->addGraph();
//    ui->customPlot->graph(2)->setData(x, w);

//    pen.setColor(Qt::red);
//    ui->customPlot->graph(2)->setPen(pen);


    // give the axes some labels:
    ui->customPlot->xAxis->setLabel("x");
    ui->customPlot->yAxis->setLabel("y");
    // set axes ranges, so we see all data:
    ui->customPlot->xAxis->setRange(-1, 1);
    ui->customPlot->yAxis->setRange(0, 1);
    ui->customPlot->replot();



}

MainWindow::~MainWindow()
{
    delete ui;
}
