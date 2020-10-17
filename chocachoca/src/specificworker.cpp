
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

    this->speedBase = 1000;

    this->spiralTicksNoColision = 0;
    this->wallsTicksNoColision = 0;

    this->wallInit = true;
    this->onWall = false;
    this->distWallMAX = 300;
    this->distWallMIN = 248;
    this->secondStageWall = false;
    this->thresholdWall = 190;
    this->maxTicksWall = 220;

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
        std::sort(ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b) { return a.dist < b.dist; });

        speedControl(ldataWalls);

        // <- OBTENER POSICION
        // int x, z;  float f;
        //this->differentialrobot_proxy->getBasePose(x, z, f);
        //qDebug() << "Salida -> x: " << x << " z : " << z << " alpha: " << f;

        // <- Imprime vector con indices
        //for (long unsigned int i = 0; i < ldataWalls.size(); i++)
        //    std::cout << " Pos: " << i << "  ->  " << ldataWalls.data()[i].dist << std::endl;

        // <- Imprime datos de vectores
        //for(auto &l : ldata) qDebug() << l.angle << l.dist;
        //for (auto &l : ldataWalls) qDebug() << l.angle << l.dist;

        //std::terminate();

        switch (currentState) {
            case state::SPIRAL:
                spiral(ldata);
                break;
            case state::CHOCACHOCA:
                chocachoca(ldata);
                break;
            case state::WALLS:
                walls(ldata, ldataWalls);
                break;
        }
    }
    catch (const Ice::Exception &ex) {
        std::cout << ex << std::endl;
    }
}

void SpecificWorker::spiral(RoboCompLaser::TLaserData ldata) {
    if (ldata.front().dist < threshold)             //en caso de choque suspende espiral
    {
        currentState = state::CHOCACHOCA;
        currentSideTicks = sideTicks;
        currentTraveledTicks = 0;
        contSideSpiral = 0;
        contTurn = 0;
        std::cout << " *** Spiral end ***" << std::endl;

    } else {

        //mintras que no se choca sigue logida de la espiral
        //por cada 2 giros aumenta el radio de la espiral
        if (contTurn == 1) {
            contTurn = 0;
            currentSideTicks = currentSideTicks + sideTicks;
            std::cout << "       --- Spiral increment side to: " << currentSideTicks << std::endl;
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

void SpecificWorker::walls(RoboCompLaser::TLaserData ldata, RoboCompLaser::TLaserData ldataWalls) {

    if (frontDist(ldataWalls) < thresholdWall)                 //si choque
    {
        std::cout << ldata.front().dist << std::endl;
        differentialrobot_proxy->setSpeedBase(5, 2);
        usleep(780000);
        onWall = true;
    } else {

        //TODO AJUSTAR RADIO-TIMPO DE GIRO

        if (!wallInit) {
            //CONTROL DE APTITUD
            if (onWall) {

                if (leftDist(ldataWalls) > distWallMAX) {           //El robot se separa de la pared
                    qDebug() << "\n                   ** Giro izquierda\n";
                    differentialrobot_proxy->setSpeedBase(10, -2);     //giro izquierda
                    usleep(42000);
                } else if (leftDist(ldataWalls) < distWallMIN) {    //El robot se pega a la pared
                    qDebug() << "\n                   ** Giro derecha\n";
                    differentialrobot_proxy->setSpeedBase(10, 2);      //giro derecha
                    usleep(42000);
                }
            }
            //AVANCE
            std::cout << "           +1 tick , WALLSTicks: " << wallsTicksNoColision << std::endl;

            //TODO Ajustar velocidad a la distacia ldata.front().dist (lineal, exponencial o logaritmico)

            differentialrobot_proxy->setSpeedBase(speedBase, 0);
            if (wallsTicksNoColision == 20 && secondStageWall) {
                thresholdWall += 320; //ajustable from 400
                maxTicksWall = 195;   //from 200
            }
            //CONDICION DE SALIDA
            if (wallsTicksNoColision > maxTicksWall) {       //Si n movimientos sin chocar, inicia espiral, rand 20-25
                distWallMAX += 400;
                distWallMIN += 400;
                wallsTicksNoColision = 20;
                if (secondStageWall) {
                    differentialrobot_proxy->setSpeedBase(5, 2);
                    usleep(780000);
                    currentState = state::CHOCACHOCA;
                    std::cout << " *** Walls End ***" << std::endl;
                }
                std::cout << " *** Walls -SECOND STAGE- ***" << std::endl << std::endl;
                secondStageWall = true;
            } else                              //else lo incrementa
                wallsTicksNoColision++;
        } else {                                //Arranque: gira 90º buscando una pared
            wallInit = false;
            qDebug() << "Entra en init";
            differentialrobot_proxy->setSpeedBase(5, rot);
            usleep(780000);         // tiempo aprox para girar 90º a 2 rad/s
        }
    }
}


void SpecificWorker::chocachoca(RoboCompLaser::TLaserData ldata) {
    if (ldata.front().dist < threshold)            //si menor
    {
        std::cout << ldata.front().dist << std::endl;
        differentialrobot_proxy->setSpeedBase(5, rot);
        usleep(rand() % (1000000 - 100000 + 1) + 100000);
        //pone contador de n movimientos seguidos sin chocar a 0
        spiralTicksNoColision = 0;
    } else {
        std::cout << "        +1 tick , totTicks: " << spiralTicksNoColision << std::endl;
        differentialrobot_proxy->setSpeedBase(speedBase, 0);

        if (spiralTicksNoColision > 15) {       //Si n movimientos seguinos sin chocar, inicia espiral, rand 20-25
            currentState = state::SPIRAL;
            std::cout << " *** Spiral Start ***" << std::endl;
        } else                            //else lo incrementa
            spiralTicksNoColision++;
    }
}

void SpecificWorker::speedControl(RoboCompLaser::TLaserData ldataWalls) {
    if (frontDist((ldataWalls)) < 600) {
        speedBase = 800;
    } else
        speedBase = 1000;
}

int SpecificWorker::frontDist(RoboCompLaser::TLaserData ldataWalls) {
    float storeDistFront = 0;
    for (long unsigned int i = 25; i <= 75; i++) {
        storeDistFront += ldataWalls.data()[i].dist;
    }
    return storeDistFront / 51;
}

int SpecificWorker::leftDist(RoboCompLaser::TLaserData ldataWalls) {
    float storeDistLeft = 0;
    for (long unsigned int i = 90; i <= 95; i++)
        storeDistLeft += ldataWalls.data()[i].dist;
    return storeDistLeft / 5;
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

