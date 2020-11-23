/*
*    Copyright (C) 2020 by PABLO ROMERO MUÑOZ
*
*    This file is part of RoboComp
*
*    RoboComp is free software: you can redistribute it and/or modify
        *    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*    (at your option) any later version.
*
*    RoboComp is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
*/

// NOTAS: evaluar frente hasta detectar obstáculo, si el giro es a derecha, hasta pasar el
//      obstáculo solo evaluo parte izqueirda del laser, contrario si giro izquierda. Habrá
//      que controlar cuando paro de evaluar un solo lado y vuelvo a mirar al frente.

#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx) {
    this->startup_check_flag = startup_check;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker() {
    std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params) {
    return true;
}

void SpecificWorker::initialize(int period) {
    std::cout << "Initialize worker" << std::endl;

    // graphics
    graphicsView = new QGraphicsView(this);
    graphicsView->resize(this->size());
    graphicsView->setScene(&scene);
    graphicsView->setMinimumSize(400,400);
    scene.setItemIndexMethod(QGraphicsScene::NoIndex);
    struct Dimensions
    {
        int TILE_SIZE = 100;
        float HMIN = -2500, VMIN = -2500, WIDTH = 5000, HEIGHT = 5000;
    };
    Dimensions dim;
    scene.setSceneRect(dim.HMIN, dim.VMIN, dim.WIDTH, dim.HEIGHT);
    graphicsView->scale(1, -1);

    graphicsView->show();

    //robot
    QPolygonF poly2;
    float size = ROBOT_LENGTH / 2.f;
    poly2 << QPoint(-size, -size)
          << QPoint(-size, size)
          << QPoint(-size / 3, size * 1.6)
          << QPoint(size / 3, size * 1.6)
          << QPoint(size, size)
          << QPoint(size, -size);
    QBrush brush;
    brush.setColor(QColor("DarkRed"));
    brush.setStyle(Qt::SolidPattern);
    robot_polygon = (QGraphicsItem*) scene.addPolygon(poly2, QPen(QColor("DarkRed")), brush);
    robot_polygon->setZValue(5);
    RoboCompGenericBase::TBaseState bState;
    try
    {
        differentialrobot_proxy->getBaseState(bState);
        robot_polygon->setRotation(qRadiansToDegrees(-bState.alpha));
        robot_polygon->setPos(bState.x,bState.z);
    }
    catch (const Ice::Exception &e) { std::cout << e.what() << std::endl; }

    // box
    //auto caja = innerModel->getTransform("caja1");
    //if( caja )
    //    scene.addRect(caja->backtX-200, caja->backtZ-200, 400, 400, QPen(QColor("Magenta")), QBrush(QColor("Magenta")));

    graphicsView->fitInView(scene.sceneRect(), Qt::KeepAspectRatio );


    this->Period = period;
    if (this->startup_check_flag) {
        this->startup_check();
    } else {
        timer.start(Period);
    }
}

void SpecificWorker::readLaserObstacles() {
    ldata = laser_proxy->getLaserData();          // laserData read
    ldataOrder = ldata;
    std::sort(ldataOrder.begin(), ldataOrder.end(),
              [](RoboCompLaser::TData a, RoboCompLaser::TData b) { return a.dist < b.dist; });
    ldataObstacles.clear();
    for (auto &p : ldataOrder) {                  // push on new list
        if (p.dist > 1000)
            break;
        ldataObstacles.push_back(p);
    }
}

bool SpecificWorker::checkTargetInsideLaserPolygon(QPointF point) {

    // create laser polygon
    QPolygonF laser_poly;
    for (auto &l : ldata)
        laser_poly << QPointF(l.dist * sin(l.angle), l.dist * cos(l.angle));
    QPointF pointF;
    // check intersection
    if (laser_poly.containsPoint(point, Qt::OddEvenFill))  // point to check. Must be in robot’s coordinate system
        return true;
    else
        return false;
    return true;
}

int SpecificWorker::frontDist() {
    float storeDistFront = 0;
    for (long unsigned int i = 25; i <= 75; i++) {
        storeDistFront += ldata.data()[i].dist;
    }
    return storeDistFront / 51;
}

void SpecificWorker::dontHitTheObstacle() {
    int election [2] = {1, -1};

    if (frontDist() < 150){
        differentialrobot_proxy->setSpeedBase(-500, 0);
        usleep(800000);
        differentialrobot_proxy->setSpeedBase(20, MAX_TURN * election[(rand() % 2)]);
        usleep(rand() % (100000 - 10000 + 1) + 100000);
        differentialrobot_proxy->setSpeedBase(550, 0);
        usleep(800000);

    }
}

void SpecificWorker::potentialFieldMethod(Eigen::Vector2f &acumVector) {
    float operation = 1;
    //if (!checkTargetInsideLaserPolygon(acumVector)) {
        for (auto &p : ldataObstacles) {
            //operation = pow(1 / (pow((p.dist / 2000), 2)), 3);
            operation = 1 / (pow((p.dist / 2000), 2));
            if (operation > 90)
                operation = 90;
            Eigen::Vector2f tempVector(-((p.dist * sin(p.angle)) / p.dist) * operation,
                                       -((p.dist * cos(p.angle)) / p.dist) * operation);
            acumVector += tempVector;           // Acum all forces
        }
        draw_vector(acumVector);
    //}
}

void SpecificWorker::goToTarget(Eigen::Matrix<float, 2, 1> tw) {

    Eigen::Vector2f rw(bState.x, bState.z);
    Eigen::Matrix2f rot;

    rot << cos(bState.alpha), -sin(bState.alpha), sin(bState.alpha), cos(bState.alpha);

    auto tr = rot * (tw - rw);
    auto beta = atan2(tr.x(), tr.y());
    auto dist = tr.norm();          //distancia al objetivo

    qDebug() << "                 Distace to target. " << dist << " Beta: " << beta;



    if (dist < 50) {                     // On target
        differentialrobot_proxy->setSpeedBase(0, 0);        // Stop
        tg.setActiveFalse();
        qDebug() << "\n           *** On target ***\n";
        //draw_clean_vector();
    } else {
        auto vrot = std::min((MAX_TURN * beta), float(MAX_TURN));
        // lambda = (-0,5)/Ln(0,1)
        auto vadv = MAX_ADVANCE * std::min(dist / 500, float(1)) * exp(-(vrot * vrot) / 0.2171472);
        // lambda = (-0,5)/Ln(0,3)
        //auto vadv = MAX_ADVANCE * std::min(dist / 500, float(1)) * exp(-(vrot*vrot)/0.415291);
        // lambda = (-0,5)/Ln(0,5)
        //auto vadv = MAX_ADVANCE * std::min(dist / 500, float(1)) * exp(-(vrot * vrot) / 0.7213475);

        differentialrobot_proxy->setSpeedBase(vadv, vrot);  // Go to target
    }
}

void SpecificWorker::compute() {

    try {

        differentialrobot_proxy->getBaseState(bState);

        readLaserObstacles();

        draw_things();

        if (auto newTarget = tg.get(); newTarget.has_value()) {
            auto targetw = newTarget.value();

            Eigen::Vector2f acumVector(0, 0);          // Accumulate all force vectors

            if (!checkTargetInsideLaserPolygon(QPoint(targetw.x(), targetw.y()))) {
                qDebug() << "Target a la vista!";
            }
            potentialFieldMethod(acumVector);
            qDebug() << "               Vector de desvío : ";
            qDebug() << "               x:" << acumVector.x() << " y:" << acumVector.y() << " modulo: "
                     << (float) acumVector.norm();

            targetw += acumVector;


            qDebug() << "    *  Vector target: ";
            qDebug() << "x:" << targetw.x() << " y:" << targetw.y() << " modulo: " << targetw.norm();

            goToTarget(targetw);
        }
    }
    catch (const Ice::Exception &e) {
        std::cout << e << std::endl;
    }
}

void SpecificWorker::draw_things()
{
    //draw robot
    //innerModel->updateTransformValues("base", bState.x, 0, bState.z, 0, bState.alpha, 0);
    robot_polygon->setRotation(qRadiansToDegrees(-bState.alpha));
    robot_polygon->setPos(bState.x, bState.z);
    graphicsView->resize(this->size());

    //draw laser
   if (laser_polygon != nullptr)
       scene.removeItem(laser_polygon);
   QPolygonF poly;
   for( auto &l : ldata)
       poly << robot_polygon->mapToScene(QPointF(l.dist * sin(l.angle), l.dist * cos(l.angle)));
   QColor color("LightGreen");
   color.setAlpha(40);
   laser_polygon = scene.addPolygon(poly, QPen(color), QBrush(color));
   laser_polygon->setZValue(13);


}

void SpecificWorker::draw_vector(const Eigen::Vector2f &vector){

    QColor col("Red");
    QPointF centro = robot_polygon->mapToScene(vector.x(), vector.y());
    result_vector = scene.addEllipse(centro.x(), centro.y(), 50, 50, QPen(col), QBrush(col));
}


int SpecificWorker::startup_check() {
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, qApp, SLOT(quit()));
    return 0;
}


//SUBSCRIPTION to setPick method from RCISMousePicker interface
void SpecificWorker::RCISMousePicker_setPick(RoboCompRCISMousePicker::Pick myPick) {
    //std::cout << myPick.x << " " << myPick.z << endl;
    tg.put(Eigen::Vector2f(myPick.x, myPick.z));
    qDebug() << "\n           *** New target: [" << tg.data.x() << ", " << tg.data.y() << "]  ***\n";
}



/**************************************/
// From the RoboCompDifferentialRobot you can call this methods:
// this->differentialrobot_proxy->correctOdometer(...)
// this->differentialrobot_proxy->getBasePose(...)
// this->differentialrobot_proxy->getBaseState(...)
// this->differentialrobot_proxy->resetOdometer(...)
// this->differentialrobot_proxy->setOdometer(...)
// this->differentialrobot_proxy->setOdometerPose(...)
// this->differentialrobot_proxy->setSpeedBase(...)
// this->differentialrobot_proxy->stopBase(...)

/**************************************/
// From the RoboCompDifferentialRobot you can use this types:
// RoboCompDifferentialRobot::TMechParams

/**************************************/
// From the RoboCompLaser you can call this methods:
// this->laser_proxy->getLaserAndBStateData(...)
// this->laser_proxy->getLaserConfData(...)
// this->laser_proxy->getLaserData(...)

/**************************************/
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData

/**************************************/
// From the RoboCompRCISMousePicker you can use this types:
// RoboCompRCISMousePicker::Pick
