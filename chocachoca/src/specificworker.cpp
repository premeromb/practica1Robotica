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
SpecificWorker::SpecificWorker(MapPrx &mprx, bool startup_check) : GenericWorker(mprx) {
    this->startup_check_flag = startup_check;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker() {
    std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params) {
//	THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		std::string innermodel_path = par.value;
//		innerModel = std::make_shared(innermodel_path);
//	}
//	catch(const std::exception &e) { qFatal("Error reading config params"); }

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

    this->spiralTicksNoColision = 0;
    this->wallsTicksNoColision = 0;

    this->wallInit = true;

    this->sideTicks = 2;
    this->currentSideTicks = 2;
    this->currentTraveledTicks = 0;
    this->contSideSpiral = 0;
    this->contTurn = 0;

}


void SpecificWorker::compute() {

    try {
        RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();

        std::vector<RoboCompLaser::TData> ldataWalls(ldata);

        //sort laser data from small to large distances using a lambda function.
        std::sort(ldata.begin(), ldata.end(),[](RoboCompLaser::TData a, RoboCompLaser::TData b) { return a.dist < b.dist; });
        int x, z;
        float f;
        //this->differentialrobot_proxy->getBasePose(x, z, f);            // <- ASI PARA OBTENER POSICION
        //qDebug() << "Salida -> x: " << x << " z : " << z << " alpha: " << f;

        //for(auto &l : ldata) qDebug() << l.angle << l.dist;
        //for(auto &l : ldataWalls) qDebug() << l.angle << l.dist;
        //std::terminate();

        switch (currentState) {
            case state::SPIRAL:
                spiral(ldata);
                break;
            case state::CHOCACHOCA:
                chocachoca(ldata);
                break;
            case state::WALLS:
                walls(ldata);
                break;
        }
    }

    catch (const Ice::Exception &ex) {
        std::cout << ex << std::endl;
    }
}

void SpecificWorker::spiral(RoboCompLaser::TLaserData ldata)
{
    if (ldata.front().dist < threshold)            //si menor
    {
        //en caso de choque suspende espiral
        currentState = state::CHOCACHOCA;
        currentSideTicks = sideTicks;
        currentTraveledTicks = 0;
        contSideSpiral = 0;
        contTurn = 0;
        std::cout << " *** Spiral end ***" << std::endl;

        //Devuelve control a logida chocachoca sin avanzar (puede perder tiempo)

    } else {

        //mintras que no se choca sigue logida de la espiral
        //por cada 2 giros aumenta el radio de la espiral
        if (contTurn == 1) {
            contTurn = 0;
            currentSideTicks = currentSideTicks + sideTicks;
            std::cout << "       --- Spiral aumenta lado a recorrer a: " << currentSideTicks << std::endl;
        }

        if (currentTraveledTicks >= currentSideTicks)        //si ha recorrdo un lado completo gira
        {
            std::cout << "    Spiral giro  " << std::endl;
            currentTraveledTicks = 0;
            differentialrobot_proxy->setSpeedBase(5, -rot);
            usleep(780000);  // tiempo aprox para girar 90º a 2 rad/s
            contTurn++;
        } else {        //sigue recto e increemtna 1 tick recorido
            differentialrobot_proxy->setSpeedBase(speedBase, 0);
            currentTraveledTicks++;
            std::cout << "                  tickSpiral ++  " << std::endl;
        }
    }
}

void SpecificWorker::walls(RoboCompLaser::TLaserData ldata)
{
    if (ldata.front().dist < threshold)            //si choque
    {
        std::cout << ldata.front().dist << std::endl;
        differentialrobot_proxy->setSpeedBase(5, 1);
        usleep(50);
    }
    else if(false) //TODO la dist de las posiciones excede rango se pega a la pared, si disminuye, se separa
    {

    }
    else {
        if(!wallInit)
        {
            std::cout << "           +1 tick , WALLSTicks: " << wallsTicksNoColision << std::endl;
            //TODO Ajustar velocidad a la distacia ldata.front().dist (lineal, exponencial o logaritmico)
            differentialrobot_proxy->setSpeedBase(speedBase, 0);

            if (wallsTicksNoColision > 180) {       //Si n movimientos seguinos sin chocar, inicia espiral, rand 20-25 //TODO AJUSTAR
                currentState = state::CHOCACHOCA;
                std::cout << " *** Walls End ***" << std::endl;
            } else                            //else lo incrementa
                wallsTicksNoColision++;
        }
        else {      //Arranque: gira 90º buscando una pared
            wallInit = false;
            qDebug() << "Entra en init";
            differentialrobot_proxy->setSpeedBase(5, rot);
            usleep(780000);  // tiempo aprox para girar 90º a 2 rad/s
        }
    }

}void SpecificWorker::chocachoca(RoboCompLaser::TLaserData ldata)
{
    if (ldata.front().dist < threshold)            //si menor
    {
        std::cout << ldata.front().dist << std::endl;
        differentialrobot_proxy->setSpeedBase(5, rot);
        usleep(rand() % (1200000 - 100000 + 1) + 100000);  // random wait between 1s and 0.1sec //limitado el rango parawue no guire tanto y se entorsque

        //pone contador de n movimientos seguidos sin chocar a 0
        spiralTicksNoColision = 0;
    } else {
        std::cout << "        +1 tick , totTicks: " << spiralTicksNoColision << std::endl;
        //TODO Ajustar velocidad a la distacia ldata.front().dist (lineal, exponencial o logaritmico)
        differentialrobot_proxy->setSpeedBase(speedBase, 0);

        if (spiralTicksNoColision > 15) {       //Si n movimientos seguinos sin chocar, inicia espiral, rand 20-25
            currentState = state::SPIRAL;
            std::cout << " *** Spiral Start ***" << std::endl;
        } else                            //else lo incrementa
            spiralTicksNoColision++;
    }
}


int SpecificWorker::startup_check() {
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, qApp, SLOT(quit()));
    return 0;

}



/// AQUI ********************* /home/robocomp/robocomp/interfaces/DifferentialRobot.ice ***********************

/**************************************/
// From the RoboCompDifferentialRobot you can call this methods:
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

