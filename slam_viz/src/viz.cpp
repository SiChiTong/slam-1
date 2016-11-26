#include "slam/viz/viz.hpp"


namespace slam {

// VIZ SETTINGS
VizSettings::VizSettings(void)
{
    this->window_width = 640;
    this->window_height = 480;
    this->window_bpp = 32;
    this->window_title = "SLAM Vizualization";
}


// VIZ CAMERA
VizCamera::VizCamera(void)
{
    this->position << 0.0, 0.0, 0.0;
    this->view << 0.0, 0.0, 0.0;
    this->up << 0.0, 0.0, 0.0;
}


// VIZ
Viz::Viz(void)
{
    this->configured = false;
}

int Viz::configure(void)
{
    this->configured = true;

    // initialize SDL
    if (SDL_Init(SDL_INIT_EVERYTHING) < 0) {
        return -1;
    }

    // create window
    this->window = SDL_CreateWindow(
        this->settings.window_title.c_str(),
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        this->settings.window_width,
        this->settings.window_height,
        SDL_WINDOW_OPENGL
    );
    if (this->window == NULL) {
        return -2;
    }

    // initialize opengl
    this->context = SDL_GL_CreateContext(this->window);

    return 0;
}

int Viz::initGL(void)
{
    // initialize Projection Matrix
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    // initialize Modelview Matrix
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // initialize clear color
    glClearColor(0.f, 0.f, 0.f, 1.f);

    // check for error
    GLenum error = glGetError();
    if (error != GL_NO_ERROR) {
        printf("Error initializing OpenGL! %s\n", gluErrorString(error));
        return -1;
    }

    return 0;
}

int Viz::updateView(void)
{
    gluLookAt(
        this->camera.position(0),
        this->camera.position(1),
        this->camera.position(2),
        this->camera.view(0),
        this->camera.view(1),
        this->camera.view(2),
        this->camera.up(0),
        this->camera.up(1),
        this->camera.up(2)
    );
}

int Viz::renderScene(void)
{
    // render
    glMatrixMode(GL_MODELVIEW);
    glClear(GL_COLOR_BUFFER_BIT);
    glLoadIdentity();

    // draw ground plane
    glColor3f(.3,.3,.3);
    glBegin(GL_QUADS);
        glVertex3f( 0,-0.001, 0);
        glVertex3f( 0,-0.001, 10);
        glVertex3f(10,-0.001, 10);
        glVertex3f(10,-0.001, 0);
    glEnd();

    glBegin(GL_LINES);
    for (int i = 0; i <= 10; i++) {
        if (i == 0) {
            glColor3f(.6,.3,.3);
        } else {
            glColor3f(.25,.25,.25);
        };
        glVertex3f(i,0,0);
        glVertex3f(i,0,10);

        if (i == 0) {
            glColor3f(.3,.3,.6);
        } else {
            glColor3f(.25,.25,.25);
        };
        glVertex3f(0, 0, i);
        glVertex3f(10, 0, i);
    }
    glEnd();

    return 0;
}

int Viz::run(void)
{


    return 0;
}

}  // end of slam namespace
