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
    //plot1
    ui->customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    ui->customPlot->xAxis->setLabel("t");
    ui->customPlot->yAxis->setLabel("position");
    ui->customPlot->legend->setVisible(true);

    //plot2
    ui->customPlot_2->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    ui->customPlot_2->xAxis->setLabel("t");
    ui->customPlot_2->yAxis->setLabel("velocity");
    ui->customPlot_2->legend->setVisible(true);


    ukf ukf1;
    double velocity = 0.0 ;
    double pos = 0.0;
    double measure=0.0;
    double dt =0.02;
    double T=0.0;


    ukf1.dt = 0.02;



    QVector<double> x(1001), y(1001) ,z(1001),w(1001),p(1001),q(1001); // initialize with entries 0..100
    for (int i=0; i<1001; ++i)
    {


      T+=dt;
      pos = pos+ velocity* dt;
      measure = pos + (rand()%100-50)*0.001;
      velocity = cos(pos)+0.99*velocity;



      ukf1.predict();
      ukf1.correct(measure);


      x[i] = T;
      y[i] = pos; // let's plot a rquadratic function
      z[i] = velocity;
      w[i] = measure;
      p[i] = ukf1.x(0);
      q[i] = ukf1.x(1);


    }
    // create graph and assign data to it:
    QPen pen;
    pen.setColor(Qt::black);

    ui->customPlot->addGraph();
    ui->customPlot->graph(0)->setData(x, w);
    ui->customPlot->graph(0)->setName("measurement");
    ui->customPlot->graph(0)->setPen(pen);
    ui->customPlot->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc, 5));
    ui->customPlot->graph(0)->setLineStyle(QCPGraph::lsNone);


    pen.setColor(Qt::blue);
    pen.setWidth(2);

    ui->customPlot_2->addGraph();
    ui->customPlot_2->graph(0)->setData(x, q);
    ui->customPlot_2->graph(0)->setName("Estimate");
    ui->customPlot_2->graph(0)->setPen(pen);

//    ui->customPlot_2->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc, 5));
//    ui->customPlot_2->graph(0)->setLineStyle(QCPGraph::lsNone);

    pen.setColor(Qt::red);
    pen.setWidth(2);


    ui->customPlot->addGraph();
     ui->customPlot->graph(1)->setPen(pen);
    ui->customPlot->graph(1)->setData(x, p);
    ui->customPlot->graph(1)->setName("UKF");

    ui->customPlot_2->addGraph();
    ui->customPlot_2->graph(1)->setData(x, z);
    ui->customPlot_2->graph(1)->setName("real");
    ui->customPlot_2->graph(1)->setPen(pen);

    ui->customPlot->xAxis->setRange(0, 7);
    ui->customPlot->yAxis->setRange(0, 4);
    ui->customPlot_2->xAxis->setRange(0, 7);
    ui->customPlot_2->yAxis->setRange(-10.5, 10.5);
    ui->customPlot->replot();
    ui->customPlot_2->replot();

    ui->customPlot->savePng(QString("position.png"), 0, 0, 5,100, -1);
    ui->customPlot_2->savePng(QString("velocity.png"), 0, 0, 5,100, -1);
}

MainWindow::~MainWindow()
{
    delete ui;
}
