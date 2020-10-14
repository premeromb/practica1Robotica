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

/**
	\brief
	@author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>

const float threshold = 200;    // millimeters
const float rot = 2;            // rads per second
const int speedBase = 1000;     //1000 mm/s

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx, bool startup_check);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);



public slots:
	void compute();
	int startup_check();
	void initialize(int period);
private:
	std::shared_ptr < InnerModel > innerModel;
	bool startup_check_flag;

    enum class state {CHOCACHOCA, SPIRAL, WALLS};
    state currentState = state::WALLS; //TODO Al init

	void spiral(RoboCompLaser::TLaserData ldata);

	void walls(RoboCompLaser::TLaserData ldata, RoboCompLaser::TLaserData ldataWalls);

	void chocachoca(RoboCompLaser::TLaserData ldata);


    int wallsTicksNoColision;
    bool wallInit;                      //Cuando está a True ejecuta busqueda de pared (solo se realiza una vez)
    bool onWall;
    int distWallMAX;                    // Distance to walls
    int distWallMIN;
    bool secondStageWall;
    int thresholdWall;
    int maxTicksWall;


    int sideTicks;              // numero de ticks inicilaes por cada lado de la espiral
    int currentSideTicks;       // numero de tics para el lado actual

    int currentTraveledTicks;

    int spiralTicksNoColision;        // Before start spiral logic
    int contSideSpiral;             // cuneta los lados recorridos de la espiral // NO SE UASAAAAAAAAAAAAAAAAAAAAAAAA
    int contTurn;                   // cuenta los giros de 90º

};

#endif
