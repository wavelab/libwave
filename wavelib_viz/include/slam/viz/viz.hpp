#ifndef __SLAM_VIZ_VIZ_HPP__
#define __SLAM_VIZ_VIZ_HPP__

#include <GL/glut.h>
#include <SDL2/SDL.h>
#include <SDL2/SDL_opengl.h>

#include "slam/utils/utils.hpp"
#include "slam/viz/vizcamera.hpp"


namespace slam {

class VizSettings
{
public:
    int window_width;
    int window_height;
    int window_bpp;
    std::string window_title;

    VizSettings(void);
};

class VizCamera
{
public:
    Vec3 position;
    Vec3 view;
    Vec3 up;

    VizCamera(void);
};

class Viz
{
public:
    bool configured;

    SDL_Window *window;
    SDL_GLContext context;
    VizSettings settings;
    VizCamera camera;

    Viz(void);
    ~Viz(void);
    int initSDL(void);
    int initGL(void);
    int configure(void);
    int updateView(void);
    int renderScene(void);
    int handleKeyboardEvent(void);
    int run(void);
};

}  // end of slam namespace
#endif
