#include <GLFW/glfw3.h>
#include "glWinImage.h"
#include "utils.h"
#include <stdexcept>
#include <iostream>
#include <string>
#include <vector>
#include <math.h>


class MainApp
{
public:
    MainApp()
    {
        if (!glfwInit()) {
            throw std::runtime_error("glfwInit failed");
        }
        mWinImage.initialize("Fractal");
    }

    ~MainApp()
    {
        glfwTerminate();
    }

    void run()
    {
        while (!mWinImage.isFinished()) {
            mWinImage.draw();
        }
    }

private:
    GlWinImage mWinImage;
};

int main([[maybe_unused]] int argc, [[maybe_unused]] char *argv[])
{
    MainApp* sf = new MainApp();
    sf->run();
    delete sf;

    return 0;
}
