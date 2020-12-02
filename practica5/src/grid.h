//
// Created by salabeta on 24/11/20.
//

#ifndef GOTOXY_GRID_H
#define GOTOXY_GRID_H

#include <QGraphicsItem>
#include <QGraphicsSimpleTextItem>

template<typename HMIN, HMIN hmin, typename WIDTH, WIDTH width, typename TILE, TILE tile>
class Grid {

    int hminGrid, widthGrid, tileGrid;
    std::vector<std::tuple<int,int>> operators{{-1,-1}, {0,-1},{-1,-1},{-1,0},{1,0},{-1,-1}, {0,-1},{-1,-1}};
public:
    Grid() {
        array.resize((int) (width / tile));
        for (auto &row : array)
            row.resize((int) (width / tile));
        int k = 0;
        for (int i = hmin; i < width / 2; i += tile, k++) {
            int l = 0;
            for (int j = hmin; j < width / 2; j += tile, l++) {
                array[k][l] = Value{false, nullptr, nullptr, i, j};
            }
        }
        this->hminGrid = hmin;
        this->widthGrid = width;
        this->tileGrid = tile;

    };

    struct Value {
        bool occupied = false;
        QGraphicsRectItem *paint_cell = nullptr;
        QGraphicsSimpleTextItem *text_cell = nullptr;
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

                QFont f("times", tile * 0.7);
                elem.text_cell = scene.addSimpleText("-2", f);
                //QFontMetricsF fm(myText->font());

                elem.text_cell->setPos(elem.cx - tile/2, elem.cy - tile/2 );
                // elem.text_cell->setFlag(QGraphicsItem::ItemIgnoresTransformations);
                elem.text_cell->setZValue(15);

            }
    }


public:

    bool isOccupied(int x, int z) {
        //auto[i, j] = worldToGrid(x, z);
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
        if (auto r = worldToGrid(x,z); r.has_value()) {
            auto[i, j] = r.value();
            array[i][j].occupied = v;
            if (v)
                array[i][j].paint_cell->setBrush(QColor("Red"));

        }
    }

    //QString::number(distancia);

    void set_dist(int x, int z, int dist) {
        auto[i, j] = worldToGrid(x, z);
        array[i][j].dist = dist;
    }

    // From world -2500->2500 to array 0->5000
    std::optional <std::tuple<int, int>> worldToGrid(int x, int z) {
        int k = x / tileGrid + (widthGrid / tileGrid) / 2;
        int l = z / tileGrid + (widthGrid / tileGrid) / 2;
        if (isInRange(k, l))
            return std::make_tuple(k, l);
        else
            return {};
    }

    std::tuple<int, int> gridToWorld(int k, int l) {
        int x = k * tileGrid - widthGrid / 2;
        int z = l * tileGrid - widthGrid / 2;
        return std::make_tuple(x, z);
    }

  //std::vector<Value> getNeighbors(int x, int z, int dist){
  //    std::vector<Value> neighbors;
  //    auto[i, j] = worldToGrid(x, z);
  //    for (auto &[dk,dl] : operators){

  //        int auxI = i+dk; int auxJ = j+dl;
  //        if(isInRangeG(auxI, auxJ) and not array(auxI, auxJ).occupied and array(auxI, auxJ).distance > -1){
  //            array(auxI, auxJ).dist = dist;
  //            neighbors.push_back(array(auxI, auxJ));
  //        }
  //    }
  //    return neighbors;
  //}

};


#endif //GOTOXY_GRID_H
