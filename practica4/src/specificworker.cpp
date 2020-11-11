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
    this->Period = period;
    if (this->startup_check_flag) {
        this->startup_check();
    } else {
        timer.start(Period);
    }
}


void SpecificWorker::compute() {

    // Lectura laser

    RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();          // laserData read

    //sort laser data from small to large distances using a lambda function.
    std::sort(ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b) { return a.dist < b.dist; });

    // excluir puntos de distancia menor

    //si no hay ningun obstaculo la resultante será una recta hacia el target, ir metiendo uno a uno los obstaculso e ir  ajustando

    // si hay alun mínimo en que se pare, poner aleatorio o que reucpere hacu atras

    // qgraficview y otro qtl, asignando origen, puedo pintar un circulo y represental los vectores para depurar, a modo de ejemplo pintar un robot y un laser


    RoboCompGenericBase::TBaseState bState;
    differentialrobot_proxy->getBaseState(bState);

    try {
        if (auto newTarget = tg.get(); newTarget.has_value()) {
            auto tw = newTarget.value();

            RoboCompLaser::TLaserData ldataObstacles;     // new vector to avoid obstacles

            for (auto &p : ldata){                  // push on new list
                if (p.dist > 1000)
                    break;
                ldataObstacles.push_back(p);
            }
            Eigen::Vector2f acumVector(0, 0);          // Accumulate all force vectors

            // por cada uno calcular unitario * 1/d² y sumar (restar) al del target
            for (auto &p : ldataObstacles) {
                // The same as before but in one line
                //Eigen::Vector2f tempVector((-((p.dist * cos(p.angle))/p.dist) * 1/pow(p.dist, 2)), -((p.dist * sin(p.angle))/p.dist) * 1 / pow(p.dist, 2));

                Eigen::Vector2f tempVector(p.dist * cos(p.angle), p.dist * sin(p.angle));          // Cortesian coordinates

                tempVector.x() = -tempVector.x(); tempVector.y() = -tempVector.y();                      // Oposite direction

                tempVector.x() = tempVector.x() / p.dist; tempVector.y() = tempVector.y() / p.dist;     // Module 1 vector

                tempVector.x() = tempVector.x() * 100 * pow(100/p.dist, 2);           // Transformation to force
                tempVector.y() = tempVector.y() * 100 * pow(100/p.dist, 2);

                // The same as before but in one line
                //Eigen::Vector2f tempVector((-((p.dist * cos(p.angle))/p.dist) * 1/pow(p.dist, 2)), -((p.dist * sin(p.angle))/p.dist) * 1 / pow(p.dist, 2));
                //qDebug() << "               Vector de desvío : ";
                //qDebug() << "               x:" << tempVector.x() << " y:" << tempVector.y() << " modulo: " << (float)tempVector.norm();

                //std::terminate();

                //tempVector += tempVector;
                acumVector += tempVector;           // Acum all forces
            }

            qDebug() << "               Vector de desvío : ";
            qDebug() << "               x:" << acumVector.x() << " y:" << acumVector.y() << " modulo: " << (float)acumVector.norm();

            tw += acumVector;            // IS this??

            qDebug() << "    *  Vector target calculado: ";
            qDebug() << "x:" << tw.x() << " y:" << tw.y() << " modulo: " << tw.norm();

            //std::terminate();

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
            } else {
                auto vrot = MAX_TURN * beta;
                //auto vadv = MAX_ADVANCE * std::min(dist / 500, float(1)) * exp(-(vrot*vrot)/0.2171472);      // lambda = (-0,5)/Ln(0,1)
                //auto vadv = MAX_ADVANCE * std::min(dist / 500, float(1)) * exp(-(vrot*vrot)/0.415291);       // lambda = (-0,5)/Ln(0,3)
                auto vadv = MAX_ADVANCE * std::min(dist / 500, float(1)) * exp(-(vrot * vrot) / 0.7213475);     // lambda = (-0,5)/Ln(0,5)

                differentialrobot_proxy->setSpeedBase(vadv, vrot);  // Go to target
            }
        }
    }
    catch (const Ice::Exception &e) {
        std::cout << e << std::endl;
    }
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

