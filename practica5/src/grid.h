//
// Created by salabeta on 24/11/20.
//

#ifndef GOTOXY_GRID_H
#define GOTOXY_GRID_H

#include <QGraphicsItem>

template<typename HMIN, HMIN hmin, typename WIDTH, WIDTH width, typename TILE, TILE tile>
class Grid {

    int hminGrid, widthGrid, tileGrid;
    std::vector<std::tuple<int,int>> operators;
public:
    Grid() {
        array.resize((int) (width / tile));
        for (auto &row : array)
            row.resize((int) (width / tile));
        int k = 0;
        for (int i = hmin; i < width / 2; i += tile, k++) {
            int l = 0;
            for (int j = hmin; j < width / 2; j += tile, l++) {
                array[k][l] = Value{false, nullptr, i, j};
            }
        }
        this->hminGrid = hmin;
        this->widthGrid = width;
        this->tileGrid = tile;

        operators.push_back(std::make_tuple(1,0));
        operators.push_back(std::make_tuple(0,1));
        operators.push_back(std::make_tuple(-1,0));
        operators.push_back(std::make_tuple(0,-1));
    };

    struct Value {
        bool occupied = false;
        QGraphicsRectItem *paint_cell = nullptr;
        int cx, cy;
        int dist = -1; //dist vecinos
    };

    std::vector<std::vector<Value>> array;

    void create_graphic_items(QGraphicsScene &scene) {
        for (auto &row : array)
            for (auto &elem : row) {
                elem.paint_cell = scene.addRect(-tile / 2, -tile / 2, tile, tile, QPen(QColor("Darkgreen")),
                                                QBrush(QColor("Lightgreen")));
                elem.paint_cell->setPos(elem.cx, elem.cy);
            }
    }


public:

    bool isOccupied(int x, int z) {
        auto[i, j] = worldToGrid(x, z);
        return this->array[i][j].occupied;
    }

    bool isInRange(int x, int z){
        return (x >= 0 && z >= 0 && x < 50 && z < 50);
    }

    bool isDistanceUpdated(int x, int z) {
        auto[i, j] = worldToGrid(x, z);
        return array[i][j].dist > 0;;
    }

    int get_dist(int x, int z) {
        if (isOccupied(x, z)) {
            auto[i, j] = worldToGrid(x, z);
            return this->array[x][z].dist;
        }
        return -1;
    }

    void set_Ocupied(int x, int z, bool v) {
        auto[i, j] = worldToGrid(x, z);
        array[i][j].occupied = v;
        if (v)
            array[i][j].paint_cell->setBrush(QColor("Red"));
    }

    void set_dist(int x, int z, int dist) {
        auto[i, j] = worldToGrid(x, z);
        array[i][j].dist = dist;
    }

    // From world -2500->2500 to array 0->5000
    std::tuple<int, int> worldToGrid(int x, int z) {
        int k = x / tileGrid + (widthGrid / tileGrid) / 2;
        int l = z / tileGrid + (widthGrid / tileGrid) / 2;
        return std::make_tuple(k, l);
    }

    std::tuple<int, int> gridToWorld(int k, int l) {
        int x = k * tileGrid - widthGrid / 2;
        int z = l * tileGrid - widthGrid / 2;
        return std::make_tuple(x, z);
    }

    std::vector<std::tuple<int,int>> getNeighbors(int x, int z){
        std::vector<std::tuple<int,int>> neighbors;
        auto[i, j] = worldToGrid(x, z);
        for (auto &[k,l] : operators){
            int auxI = i+k; int auxJ = j+l;
            if(!isOccupied(auxI, auxJ) && !isDistanceUpdated(auxI, auxJ) && isInRange(auxI, auxJ)){
                neighbors.push_back(gridToWorld(auxI, auxJ));
            }
        }
        return neighbors;
    }

};


#endif //GOTOXY_GRID_H
