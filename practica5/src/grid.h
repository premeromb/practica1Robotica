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

    void draw_graphic_items(QGraphicsScene &scene) {
        auto fondo = QColor("LightGreen");
        fondo.setAlpha(40);
        QFont font("Bavaria");
        font.setPointSize(40);
        font.setWeight(QFont::TypeWriter);
        for (auto &row : array) {
            for (auto &elem : row) {
                elem.paint_cell = scene.addRect(-tile / 2, -tile / 2, tile, tile, QPen(QColor("Darkgreen")),
                                                QBrush(fondo));
                elem.paint_cell->setPos(elem.cx, elem.cy);
                elem.text_cell = scene.addSimpleText(" ", font);
                elem.text_cell->setPos(elem.cx - tile / 2, elem.cy - tile / 2);
                // Get the current transform
                QTransform transform(elem.text_cell->transform());
                qreal m11 = transform.m11();    // Horizontal scaling
                qreal m12 = transform.m12();    // Vertical shearing
                qreal m13 = transform.m13();    // Horizontal Projection
                qreal m21 = transform.m21();    // Horizontal shearing
                qreal m22 = transform.m22();    // vertical scaling
                qreal m23 = transform.m23();    // Vertical Projection
                qreal m31 = transform.m31();    // Horizontal Position (DX)
                qreal m32 = transform.m32();    // Vertical Position (DY)
                qreal m33 = transform.m33();    // Addtional Projection Factor
                // Vertical flip
                m22 = -m22;
                // Write back to the matrix
                transform.setMatrix(m11, m12, m13, m21, m22, m23, m31, m32, m33);
                // Set the items transformation
                elem.text_cell->setTransform(transform);
            }
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
        return array[i][j].dist > -1;
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

    void set_target(int i, int j) {
        array[i][j].dist = 0;
        array[i][j].paint_cell->setBrush(QColor("Gray"));
    }

    void set_dist(int i, int j, int dist) {
        array[i][j].dist = dist;
        if (dist > 0)
            array[i][j].paint_cell->setBrush(QColor("Yellow"));
    }

    void set_neighbors_dist(std::vector<Value> neighbors, int dist){
        for (auto neighbor : neighbors){
            auto[i, j] = world_to_grid(neighbor.cx, neighbor.cy);
            set_dist(i, j, dist);
        }
    }

    void reset_cell_distances() {
        for (auto &row : array)
            for (auto &elem : row)
                elem.dist = -1;
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

    bool isValidNeighbor(int i, int j) {
        return is_in_range(i, j) and not is_occupied(i, j) and not is_distance_updated(i, j);
    }

    std::vector<Value> get_neighbors_set_dist_draw(int i, int j, int distance) {
        std::vector<Value> neighbors;
        for (auto &[dk, dl] : operators) {
            int auxI = i + dk;
            int auxJ = j + dl;
            if (isValidNeighbor(auxI, auxJ)) {
                neighbors.push_back(array[auxI][auxJ]);
                set_dist(auxI, auxJ, distance);
                array[auxI][auxJ].text_cell->setText(QString::number(distance));
            }
        }
        return neighbors;
    }

    Value get_short_neighbor(int x, int z) {
        qDebug() << "get short neighbor ********************* con x: " << x << " z: " << z;

        auto[i, j] = world_to_grid(x, z);
        qDebug() << "             transformadas a i: " << i << " j: " << j;
        std::vector<Value> neighbors = get_neighbors(i, j);
        qDebug() << " pasa el get_neighbors";
        qDebug() << " numero de vecinos: " << std::size(neighbors);
        Value short_neighbor = neighbors[0];
        qDebug() << " a esto llega";
        for (auto neighbor : neighbors){
            qDebug() << " pasa por el for";
            if (neighbor.dist < short_neighbor.dist)
                qDebug() << " entra en el if";
            short_neighbor = neighbor;
        }
        qDebug() << "sale de get short neighbor ********************* ";
        return short_neighbor;
    }

    void calculate_navigation_grid(int x, int z) {
        auto[i, j] = world_to_grid(x, z);

        set_target(i, j);

        int distance = 1;
        std::vector<Value> L1 = get_neighbors_set_dist_draw(i, j, distance);
        set_neighbors_dist(L1, distance);
        std::vector<Value> L2 = {};
        distance++;

        bool end = false;
        while (not end) {
            for (auto current_cell : L1) {
                auto[L1_i, L1_j] = world_to_grid(current_cell.cx, current_cell.cy);
                auto current_cell_neighbors = get_neighbors_set_dist_draw(L1_i, L1_j, distance);
                set_neighbors_dist(current_cell_neighbors, distance);
                for (auto current_neighbor : current_cell_neighbors)
                    L2.push_back(current_neighbor);
            }
            distance++;
            if (L2.empty())
                end = true;
            L1.swap(L2);
            L2.clear();
        }
    }

};


#endif //GOTOXY_GRID_H