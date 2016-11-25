#include "slam/viz/viz.hpp"


namespace slam {

VizSettings::VizSettings(void)
{
    this->window_width = 640;
    this->window_height = 480;
    this->window_title = "SLAM Vizualization";
}

Viz::Viz(void)
{
    this->configured = false;
}

int Viz::configure(void)
{
    this->configured = true;

    return 0;
}

int Viz::run(void)
{
    GLFWwindow* window;

    // initialize glfw
    if (!glfwInit()) {
        return -1;
    }

    // create window
    window = glfwCreateWindow(
        this->settings.window_width,
        this->settings.window_height,
        this->settings.window_title.c_str(),
        NULL,
        NULL
    );
    if (!window) {
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);

    glMatrixMode(GL_MODELVIEW);

    // loop
    while (!glfwWindowShouldClose(window)) {
        // render
        glClear(GL_COLOR_BUFFER_BIT);
        glLoadIdentity();

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

        // swap front and back buffers
        glfwSwapBuffers(window);

        // poll for and process events
        glfwPollEvents();
    }
    glfwTerminate();

    return 0;
}

}  // end of slam namespace
