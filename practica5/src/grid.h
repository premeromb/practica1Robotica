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
    std::vector<std::tuple<int, int>> operators{{0,  1},
                                                {0,  -1},
                                                {1,  0},
                                                {-1, 0},
                                                {1,  1},
                                                {-1, -1},
                                                {1,  -1},
                                                {-1, 1}};
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
                elem.text_cell = scene.addSimpleText(" ", f);
                elem.text_cell->setPos(elem.cx - tile / 2, elem.cy - tile / 2);
            }
    }

    void update_graphic_distances(QGraphicsScene &scene) {
        for (auto &row : array)
            for (auto &elem : row) {
                elem.paint_cell->setPos(elem.cx, elem.cy);
                QFont f("times", tile * 0.6);
                elem.text_cell = scene.addSimpleText(QString::number(get_dist_world(elem.cx, elem.cy)), f);
                elem.text_cell->setPos(elem.cx - tile / 2, elem.cy - tile / 2);
            }
    }

public:

    bool is_occupied(int i, int j) {
        return this->array[i][j].occupied;
    }

    bool is_in_range(int i, int j) {
        return (i >= 0 && j >= 0 && i < 50 && j < 50);
    }

    bool is_distance_updated(int i, int j) {
        return array[i][j].dist > -1;;
    }

    int get_dist_world(int x, int z) {
        auto[i, j] = world_to_grid(x, z);
        if (not is_occupied(i, j)) {
            return this->array[i][j].dist;
        }
        return -1;
    }

    int get_dist(int i, int j) {
        if (not is_occupied(i, j)) {
            return this->array[i][j].dist;
        }
        return -1;
    }
    void set_ocupied_world(int x, int z, bool v) {
        auto[i, j] = world_to_grid(x, z);
        set_ocupied(i, j, v);
    }

    void set_ocupied(int i, int j, bool v) {
        array[i][j].occupied = v;
        if (v)
            array[i][j].paint_cell->setBrush(QColor("Red"));
    }

    void set_target(int i , int j){
        array[i][j].dist = 0;
        array[i][j].paint_cell->setBrush(QColor("Gray"));
    }

    void set_dist(int i, int j, int dist) {
        array[i][j].dist = dist;
        if (dist > 0)
            array[i][j].paint_cell->setBrush(QColor("Yellow"));
    }

    std::tuple<int, int> world_to_grid(int i, int j) {
        int k = i / tileGrid + (widthGrid / tileGrid) / 2;
        int l = j / tileGrid + (widthGrid / tileGrid) / 2;
        return std::make_tuple(k, l);
    }

    std::tuple<int, int> grid_to_world(int k, int l) {
        int x = k * tileGrid - widthGrid / 2;
        int z = l * tileGrid - widthGrid / 2;
        return std::make_tuple(x, z);
    }

    std::vector<Value> get_neighbors_and_set_distance(int i, int j, int dist) {
        std::vector<Value> neighbors;
        qDebug() << "                   On get_neighbors with i: " << i << " j: " << j;
        for (auto &[dk, dl] : operators) {
            int auxI = i + dk;
            int auxJ = j + dl;
            if (is_in_range(auxI, auxJ) and not is_occupied(auxI, auxJ) and not is_distance_updated(auxI, auxJ)) {
                qDebug() << "                   Add new neighbor i: " << auxI << " j: " << auxJ;
                set_dist(auxI, auxJ, dist);
                neighbors.push_back(array[auxI][auxJ]);
            }
        }
        return neighbors;
    }

    void calculate_navigation_grid(int x, int z) {

        auto[i, j] = world_to_grid(x, z);

        set_target(i, j);

        int distance = 1;
        std::vector<Value> L1 = get_neighbors_and_set_distance(i, j, distance);
        std::vector<Value> L2 = {};
        distance++;

        bool end = false;

        while (not end) {

            for (auto current_cell : L1) {
                auto [L1_i, L1_j] = world_to_grid(current_cell.cx, current_cell.cy);
                auto current_cell_neighbors = get_neighbors_and_set_distance(L1_i, L1_j, distance);
                for (auto current_neighbor : current_cell_neighbors)
                    L2.push_back(current_neighbor);
            }
            distance ++;
            if(L2.empty())
                end = true;
            L1.swap(L2);
            L2.clear();

        }
    }

};


#endif //GOTOXY_GRID_H
