/*
 *    Copyright (C) 2020 by YOUR NAME HERE
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

void SpecificWorker::turn(float beta){
    if (fabs(beta) < 0.05){ // Direccion ok
        differentialrobot_proxy->setSpeedBase(0, 0);        // Stop
        currentState = state::MOVE;//cambiar estado a avanzar
        return;
    }else{
        differentialrobot_proxy->setSpeedBase(20, 0.5);        // Stop
    }
}

void SpecificWorker::compute() {

    RoboCompGenericBase::TBaseState bState;
    differentialrobot_proxy->getBaseState(bState);

    try {
        if (auto t = tg.get(); t.has_value()) {
            auto tw = t.value();

            Eigen::Vector2f rw (bState.x, bState.z);
            Eigen::Matrix2f rot;

            rot << cos(bState.alpha), -sin(bState.alpha) , sin(bState.alpha), cos(bState.alpha) ;

            auto tr = rot * (tw - rw);
            auto beta = atan2(tr.x(), tr.y());
            auto dist = tr.norm();

            qDebug() << " Distancia a target. " << dist << " Beta: " << beta;

            if (dist < 50){                     // On target
                differentialrobot_proxy->setSpeedBase(0, 0);        // Stop
                tg.setActiveFalse();
            }else {
                auto vrot = 2 * beta;
                auto vadv = MAX_ADVANCE * std::min(dist / 500, float(1)) * exp(-(vrot*vrot)/0.2171472);;
                differentialrobot_proxy->setSpeedBase(vadv, vrot);
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
    tg.put(Eigen::Vector2f (myPick.x, myPick.z));
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

