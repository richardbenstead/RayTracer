#pragma once
#include "raytrace.h"
#include "utils.h"
#include <GL/gl.h>
#include <GLFW/glfw3.h>
#include <cstring>
#include <iostream>
#include <math.h>

class GlWinImage {
    static constexpr uint16_t IMAGE_SIZE = 800;
    using ImageType = Image<IMAGE_SIZE>;

    // TODO: add some debug asserts
    auto POS(int16_t x, int16_t y) { return ImageType::POS(x, y); }

  public:
    void draw() {
        int width, height;
        glfwGetWindowSize(mpWindow, &width, &height);

        glLoadIdentity();
        glMatrixMode(GL_MODELVIEW);
        glOrtho(0, width, 0, height, -1.0, 1.0);
        glViewport(0, 0, width, height);

        drawImage(width, height);

        glfwSwapBuffers(mpWindow);
        glfwPollEvents();

        if (glfwGetKey(mpWindow, GLFW_KEY_Q) == GLFW_PRESS) {
            [[unlikely]];
            mQuit = true;
        }
        if (glfwGetKey(mpWindow, GLFW_KEY_W) == GLFW_PRESS) {
            [[unlikely]];
            mRayTrace.moveCamera(0, 0, -0.1f);
        }
        if (glfwGetKey(mpWindow, GLFW_KEY_S) == GLFW_PRESS) {
            [[unlikely]];
            mRayTrace.moveCamera(0, 0, 0.1f);
        }
        if (glfwGetKey(mpWindow, GLFW_KEY_A) == GLFW_PRESS) {
            [[unlikely]];
            mRayTrace.moveCamera(-0.1f, 0, 0);
        }
        if (glfwGetKey(mpWindow, GLFW_KEY_D) == GLFW_PRESS) {
            [[unlikely]];
            mRayTrace.moveCamera(0.1f, 0, 0);
        }
        if (glfwGetKey(mpWindow, GLFW_KEY_R) == GLFW_PRESS) {
            [[unlikely]];
            mRayTrace.moveCamera(0, 0.1f, 0);
        }
        if (glfwGetKey(mpWindow, GLFW_KEY_F) == GLFW_PRESS) {
            [[unlikely]];
            mRayTrace.moveCamera(0, -0.1f, 0);
        }
    }

    bool isFinished() { return mQuit || glfwWindowShouldClose(mpWindow); }

    void initialize(const std::string &title) {
        mpWindow = glfwCreateWindow(1200, 800, title.c_str(), nullptr, nullptr);
        if (!mpWindow) {
            throw std::runtime_error("glfwCreateWindow failed");
        }

        glfwMakeContextCurrent(mpWindow);
        glfwSetWindowUserPointer(mpWindow, this);

        glfwSetMouseButtonCallback(mpWindow, [](GLFWwindow *window, int button, int action, int mods) {
            static_cast<GlWinImage *>(glfwGetWindowUserPointer(window))->mouseEvent(window, button, action, mods);
        });

        glfwSetCursorPosCallback(mpWindow, [](GLFWwindow *window, double xpos, double ypos) {
            static_cast<GlWinImage *>(glfwGetWindowUserPointer(window))->mouseMoveEvent(window, xpos, ypos);
        });
    }

  private:
    void drawImage(const int width, const int height) {
        mRayTrace.draw(mImage);
        glPixelZoom(width / static_cast<float>(IMAGE_SIZE), -height / static_cast<float>(IMAGE_SIZE));
        glRasterPos2i(0, height);
        glDrawPixels(IMAGE_SIZE, IMAGE_SIZE, GL_RGB, GL_FLOAT, &mImage.image[0]);
    }

    void mouseEvent(GLFWwindow *window, int button, int action, [[maybe_unused]] int mods) {
        if (button == GLFW_MOUSE_BUTTON_LEFT) {
            double px, py;
            glfwGetCursorPos(window, &px, &py);
            mLastMousePos = XYPair{px, py};
            mMouseLeftDown = GLFW_PRESS == action;

            /*
            int width, height;
            glfwGetWindowSize(mpWindow, &width, &height);

            if (mMouseLeftDown) {
                const double gridX = std::max(0.0, std::min<double>(IMAGE_SIZE-1,
            IMAGE_SIZE * px / (double)width)); const double gridY = std::max(0.0,
            std::min<double>(IMAGE_SIZE-1, IMAGE_SIZE * py / (double)height));
            }
            */
        }
    }

    void mouseMoveEvent([[maybe_unused]] GLFWwindow *window, double xpos, double ypos) {
        if (mMouseLeftDown) {
            int width, height;
            glfwGetWindowSize(mpWindow, &width, &height);

            const XYPair newMousePos{xpos, ypos};
            const XYPair delta = newMousePos - mLastMousePos;

            const double moveX = delta.x / width;
            const double moveY = delta.y / height;
            mRayTrace.rotateCamera(moveX, moveY);

            mLastMousePos = newMousePos;
        }
    }

    GLFWwindow *mpWindow{};
    ImageType mImage;

    bool mMouseLeftDown{false};
    XYPair mLastMousePos{};
    bool mQuit{false};
    RayTrace mRayTrace;
};
