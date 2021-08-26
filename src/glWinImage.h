#pragma once
#include <math.h>
#include <cstring>
#include <iostream>
#include "utils.h"
#include <GL/gl.h>
#include <GLFW/glfw3.h>
#include <complex>
#include "raytrace.h"

class GlWinImage
{
    static constexpr uint16_t IMAGE_SIZE = 400;
    using ImageType = Image<IMAGE_SIZE>;
    auto POS(auto x, auto y) { return ImageType::POS(x,y); }

public:
    void draw()
    {
        int width, height;
        glfwGetWindowSize(mpWindow, &width, &height);

        glLoadIdentity();
        glMatrixMode(GL_MODELVIEW);
        glOrtho(0, width, 0, height, -1.0, 1.0);
        glViewport(0,0,width,height);

        drawImage(width, height);
        
        glfwSwapBuffers(mpWindow);
        glfwPollEvents();
    }

    bool isFinished()
    {
        return mQuit || glfwWindowShouldClose(mpWindow);
    }

    void initialize(const std::string& title)
    {
        mpWindow = glfwCreateWindow(1200, 800, title.c_str(), nullptr, nullptr);
        if (!mpWindow) {
            throw std::runtime_error("glfwCreateWindow failed");
        }

        glfwMakeContextCurrent(mpWindow);
        glfwSetWindowUserPointer(mpWindow, this);

        glfwSetMouseButtonCallback(mpWindow, [](GLFWwindow *window, int button, int action, int mods) {
                static_cast<GlWinImage*>(glfwGetWindowUserPointer(window))->mouseEvent(window, button, action, mods); });

        glfwSetCursorPosCallback(mpWindow, [](GLFWwindow *window, double xpos, double ypos) {
                static_cast<GlWinImage*>(glfwGetWindowUserPointer(window))->mouseMoveEvent(window, xpos, ypos); });

        glfwSetKeyCallback(mpWindow, [](GLFWwindow* window, int key, int sc, int action, int mods) {
                static_cast<GlWinImage*>(glfwGetWindowUserPointer(window))->keyEvent(window, key, sc, action, mods); });
    }

private:
    void drawImage(const int width, const int height)
    {
        mRayTrace.draw(mImage);
        glPixelZoom(width/static_cast<float>(IMAGE_SIZE), -height/static_cast<float>(IMAGE_SIZE));
        glRasterPos2i(0, height);
        glDrawPixels(IMAGE_SIZE, IMAGE_SIZE, GL_RGB, GL_FLOAT, &mImage.image[0]);
    }

    void mouseEvent(GLFWwindow *window, int button, int action, [[maybe_unused]] int mods)
    {
        if (button == GLFW_MOUSE_BUTTON_LEFT) {
            double px, py;
            glfwGetCursorPos(window, &px, &py);
            mLastMousePos = XYPair(px, py);
            mMouseLeftDown = GLFW_PRESS==action;

            int width, height;
            glfwGetWindowSize(mpWindow, &width, &height);
            /*
            if (mMouseLeftDown) {
                const double gridX = std::max(0.0, std::min<double>(IMAGE_SIZE-1, IMAGE_SIZE * px / (double)width));
                const double gridY = std::max(0.0, std::min<double>(IMAGE_SIZE-1, IMAGE_SIZE * py / (double)height));
            }
            */
        }
    }

    void mouseMoveEvent([[maybe_unused]] GLFWwindow *window, double xpos, double ypos)
    {
        if (mMouseLeftDown) {
            int width, height;
            glfwGetWindowSize(mpWindow, &width, &height);

            XYPair newMousePos(xpos, ypos);
            [[maybe_unused]] XYPair delta = newMousePos - mLastMousePos;
        }
    }

    void keyEvent([[maybe_unused]] GLFWwindow *window, int key, [[maybe_unused]] int sc, int action, [[maybe_unused]] int mods)
    {
        if (GLFW_PRESS == action) {
            if (key == 'Q') mQuit = true;
            // if (key == 'R') mFrame.reset();
        }
    }

    GLFWwindow *mpWindow{};
    ImageType mImage;

    bool mMouseLeftDown{false};
    XYPair mLastMousePos{};
    bool mQuit{false};
    RayTrace mRayTrace;
};
