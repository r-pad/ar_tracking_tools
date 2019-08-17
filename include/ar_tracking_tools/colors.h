#ifndef COLORS_H
#define COLORS_H

#include <vector>

struct Color 
{
    Color(int r, int g, int b, int a=255) : r(r), g(g), b(b), a(a) {};
    int r = 0; 
    int g = 0; 
    int b = 0;
    int a = 255;
};

// Color list from https://sashat.me/2017/01/11/list-of-20-simple-distinct-colors/
const std::vector<Color> COLORS = {
    Color(230, 25, 75), 
    Color(60, 180, 75), 
    Color(255, 225, 25), 
    Color(0, 130, 200), 
    Color(245, 130, 48), 
    Color(145, 30, 180), 
    Color(70, 240, 240), 
    Color(240, 50, 230), 
    Color(210, 245, 60), 
    Color(250, 190, 190), 
    Color(0, 128, 128), 
    Color(230, 190, 255), 
    Color(170, 110, 40), 
    Color(255, 250, 200), 
    Color(128, 0, 0), 
    Color(170, 255, 195), 
    Color(128, 128, 0), 
    Color(255, 215, 180), 
    Color(0, 0, 128)};

Color getColor(int idx)
{
    return COLORS[idx % COLORS.size()];
}

#endif