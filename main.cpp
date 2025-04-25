#include <SDL2/SDL.h>
#include <SDL2/SDL_main.h>
#include <SDL2/SDL_ttf.h>
#include <iostream>
#include <cmath>
#include <string>
#include <algorithm>

#undef main

const int WINDOW_WIDTH = 800;
const int WINDOW_HEIGHT = 600;

// PID控制器结构体
struct PIDController {
    double Kp = 50;   // 比例系数
    double Ki = 0;  // 积分系数
    double Kd = 0;   // 微分系数

    double target = 300;  // 目标高度
    double prevError = 0;
    double integral = 0;

    // 计算控制输出
    double calculate(double current, double dt) {
        double error = target - current;
        integral += error * dt;
        double derivative = (error - prevError) / dt;
        prevError = error;

        // 抗积分饱和
        integral = std::clamp(integral, -100.0, 100.0);

        return Kp * error + Ki * integral + Kd * derivative;
    }

    // 设置新目标
    void setTarget(double newTarget) {
        target = newTarget;
        integral = 0;  // 重置积分项
    }
};

int main() {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        SDL_Log("SDL初始化失败: %s", SDL_GetError());
        return 1;
    }

    if (TTF_Init() < 0) {
        SDL_Log("TTF初始化失败: %s", TTF_GetError());
        return 1;
    }

    SDL_Window* window = SDL_CreateWindow(
            "PID高度控制模拟", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
            WINDOW_WIDTH, WINDOW_HEIGHT, SDL_WINDOW_SHOWN
    );
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

    // 加载字体
    TTF_Font* font = TTF_OpenFont("arial.ttf", 24);
    if (!font) {
        SDL_Log("字体加载失败: %s", TTF_GetError());
        font = TTF_OpenFont("C:/Windows/Fonts/Deng.ttf", 24); // Linux备用字体
        if (!font) return 1;
    }

    // 小球属性
    const int BALL_SIZE = 30;
    SDL_Rect ball = {WINDOW_WIDTH/2 - BALL_SIZE/2, WINDOW_HEIGHT/2 - BALL_SIZE/2, BALL_SIZE, BALL_SIZE};
    double ballY = ball.y;  // 浮点精确位置
    double ballVelocityY = 0;
    const double gravity = 9.8;
    const double mass = 1.0;

    // 创建PID控制器
    PIDController pid;
    pid.target = ballY + BALL_SIZE/2;

    bool running = true;
    SDL_Event event;
    Uint32 lastTime = SDL_GetTicks();
    const double MAX_DT = 0.016; // 约60FPS的固定时间步长
    double remainingTime = 0.0;

    while (running) {
        // 事件处理
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                running = false;
            } else if (event.type == SDL_MOUSEBUTTONDOWN) {
                int mouseX, mouseY;
                SDL_GetMouseState(&mouseX, &mouseY);
                pid.setTarget(mouseY - BALL_SIZE/2);
            } else if (event.type == SDL_KEYDOWN) {
                // 键盘调节PID参数
                switch (event.key.keysym.sym) {
                    case SDLK_UP: pid.Kp += 0.01; break;
                    case SDLK_DOWN: pid.Kp -= 0.01; break;
                    case SDLK_LEFT: pid.Ki -= 0.005; break;
                    case SDLK_RIGHT: pid.Ki += 0.005; break;
                    case SDLK_PAGEUP: pid.Kd += 0.01; break;
                    case SDLK_PAGEDOWN: pid.Kd -= 0.01; break;
                    case SDLK_r: // 重置PID参数
                        pid.Kp = 0.2;
                        pid.Ki = 0.05;
                        pid.Kd = 0.1;
                        break;
                }
                // 确保参数不为负
                pid.Kp = std::max(0.0, pid.Kp);
                pid.Ki = std::max(0.0, pid.Ki);
                pid.Kd = std::max(0.0, pid.Kd);
            }
        }

        // 计算时间差
        Uint32 currentTime = SDL_GetTicks();
        double dt = (currentTime - lastTime) / 1000.0;
        lastTime = currentTime;

        // 固定时间步长物理模拟
        remainingTime += dt;
        while (remainingTime >= MAX_DT) {
            // PID控制计算
            double force = pid.calculate(ballY + BALL_SIZE/2, MAX_DT);

            // 应用物理 (F=ma)
            double acceleration = (force / mass) - gravity;
            ballVelocityY += acceleration * MAX_DT;
            ballVelocityY *= 0.99; // 添加速度阻尼

            // 更新位置
            ballY += ballVelocityY * MAX_DT;

            // 边界检查
            if (ballY < 0) {
                ballY = 0;
                ballVelocityY = 0;
            }
            if (ballY + BALL_SIZE > WINDOW_HEIGHT) {
                ballY = WINDOW_HEIGHT - BALL_SIZE;
                ballVelocityY = 0;
            }

            remainingTime -= MAX_DT;
        }

        // 更新整数位置用于渲染
        ball.y = static_cast<int>(ballY);

        // 渲染
        SDL_SetRenderDrawColor(renderer, 240, 240, 240, 255);
        SDL_RenderClear(renderer);

        // 绘制目标高度线
        SDL_SetRenderDrawColor(renderer, 0, 255, 0, 150);
        SDL_RenderDrawLine(renderer, 0, static_cast<int>(pid.target + BALL_SIZE/2),
                           WINDOW_WIDTH, static_cast<int>(pid.target + BALL_SIZE/2));

        // 绘制小球
        SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
        SDL_RenderFillRect(renderer, &ball);

        // 绘制调试信息
        SDL_Color textColor = {0, 0, 0, 255};
        std::string infoText =
                "PID参数调节:\n"
                "上下箭头: Kp = " + std::to_string(pid.Kp).substr(0,4) + "\n" +
                "左右箭头: Ki = " + std::to_string(pid.Ki).substr(0,4) + "\n" +
                "PageUp/Down: Kd = " + std::to_string(pid.Kd).substr(0,4) + "\n" +
                "R键: 重置参数\n" +
                "当前高度: " + std::to_string(static_cast<int>(ballY + BALL_SIZE/2)) + "\n" +
                "目标高度: " + std::to_string(static_cast<int>(pid.target + BALL_SIZE/2));

        // 渲染多行文本
        int yOffset = 10;
        size_t start = 0;
        size_t end = infoText.find('\n');
        while (end != std::string::npos) {
            std::string line = infoText.substr(start, end - start);
            SDL_Surface* textSurface = TTF_RenderText_Solid(font, line.c_str(), textColor);
            SDL_Texture* textTexture = SDL_CreateTextureFromSurface(renderer, textSurface);
            SDL_Rect textRect = {10, yOffset, textSurface->w, textSurface->h};
            SDL_RenderCopy(renderer, textTexture, NULL, &textRect);
            SDL_FreeSurface(textSurface);
            SDL_DestroyTexture(textTexture);

            yOffset += textRect.h + 2;
            start = end + 1;
            end = infoText.find('\n', start);
        }

        SDL_RenderPresent(renderer);

        // 帧率控制
        Uint32 frameTime = SDL_GetTicks() - currentTime;
        if (frameTime < 16) {  // 约60FPS
            SDL_Delay(16 - frameTime);
        }
    }

    TTF_CloseFont(font);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    TTF_Quit();
    SDL_Quit();
    return 0;
}