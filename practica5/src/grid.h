//
// Created by salabeta on 24/11/20.
//

#ifndef GOTOXY_GRID_H
#define GOTOXY_GRID_H

#include <QGraphicsItem>

template<typename HMIN, HMIN hmin, typename WIDTH, WIDTH width, typename TILE, TILE tile>
class Grid
{

    int hminGrid, widthGrid, tileGrid;
    public:
        Grid()
        {
            array.resize((int)(width/tile));
            for (auto &row : array)
                row.resize((int)(width/tile));
            int k=0;
            for (int i = hmin; i < width/2; i += tile, k++)
            {
                int l=0;
                for (int j = hmin; j < width/2; j += tile, l++)
                {
                    array[k][l] = Value{false, nullptr, i, j};
                }
            }
            this->hminGrid = hmin;
            this->widthGrid = width;
            this->tileGrid = tile;
        };

        struct Value
        {
            bool occupied = false;
            QGraphicsRectItem * paint_cell = nullptr;
            int cx, cy;
            int dist = 0; //dist vecinos
        };

        std::vector<std::vector<Value>> array;

        void create_graphic_items(QGraphicsScene &scene)
        {
            for (auto &row : array)
                for (auto &elem : row)
                {
                    elem.paint_cell = scene.addRect(-tile / 2, -tile / 2, tile, tile, QPen(QColor("Darkgreen")),
                                                    QBrush(QColor("Lightgreen")));
                    elem.paint_cell->setPos(elem.cx, elem.cy);
                }
        }


public:
    /**
     * modificamos en funcion de v la coordenada x,z
     * @param x
     * @param z
     * @param v
     */
    void set_Value(int x, int z, bool v)
    {

        auto[i, j] = transformar(x, z);
        array[i][j].occupied = v;
        if (v)
            array[i][j].paint_cell->setBrush(QColor("Red"));
    }

    /**
     * devolvemos el valor de la coordenada x,z
     * @param x
     * @param z
     * @return
     */
    bool get_value(int x, int z)
    {
        auto [i, j] = transformar(x,z);
        return  this->array[x][z];
    }



    std::tuple<int, int> transformar(int i, int j) {
        int k = i/tileGrid + (widthGrid / tileGrid) / 2;
        int l = j/tileGrid + (widthGrid / tileGrid) / 2;
        return std::make_tuple(k, l);
    }

};


#endif //GOTOXY_GRID_H
