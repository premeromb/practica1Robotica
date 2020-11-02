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

/**
	\brief
	@author Pablo Romero Muñoz
*/

#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>

#include <Eigen/Dense>

#include <innermodel/innermodel.h>

const int MAX_ADVANCE = 1000;   // mm/s
const int MAX_TURN = 2;         // rad/s

class SpecificWorker : public GenericWorker {
Q_OBJECT
public:

    SpecificWorker(TuplePrx tprx, bool startup_check);

    ~SpecificWorker();

    bool setParams(RoboCompCommonBehavior::ParameterList params);

    void RCISMousePicker_setPick(RoboCompRCISMousePicker::Pick myPick);

public slots:

    void compute();

    int startup_check();

    void initialize(int period);

private:
    template<typename T>
    struct Target {
        T data;

        std::atomic<bool> active = false;        //Si false, valor desactualizado
        mutable std::mutex mnt;

        void put(const T &&data_) {  // && cambia el "dueño" de la variable sin nombre sin hacer "copia" a este espacio
            std::lock_guard<std::mutex> lock(mnt);
            data = data_;
            this->active = true;
        }

        //Toma posición objetivo
        std::optional<T> get() const {
            std::lock_guard<std::mutex> lock(mnt);
            if (active)
                //return std::make_tuple(x, z);
                return data;
            else
                return {};
        }

        //Cuando llegue al objetivo, desactiva flag
        void setActiveFalse() {
            this->active = false;
        }
    };

    Target<Eigen::Vector2f> tg;

    std::shared_ptr<InnerModel> innerModel;
    bool startup_check_flag;
};

#endif
