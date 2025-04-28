#include <SDL.h>
#include <SDL_ttf.h>
#include <memory>
#include <string>
#include <algorithm>
#include <stdexcept>

#undef main

constexpr int WINDOW_WIDTH = 800;
constexpr int WINDOW_HEIGHT = 800;
constexpr int BALL_SIZE = 30;
constexpr double GRAVITY = 98;
constexpr double FIXED_TIMESTEP = 1.0 / 60.0;

class PID_Controller {
public:
    PID_Controller(double kp, double ki, double kd)
            : Kp(kp), Ki(ki), Kd(kd) {}

    double calculate(double setpoint, double pv, double dt) {
        double error = setpoint - pv;
        integral += error * dt;
        integral = std::clamp(integral, -1000.0, 1000.0);
        double derivative = (error - prev_error) / dt;
        prev_error = error;
        return Kp * error + Ki * integral + Kd * derivative;
    }

    void reset() {
        integral = 0.0;
        prev_error = 0.0;
    }

    double Kp = 80.0;
    double Ki = 0;
    double Kd = 0;

private:
    double integral = 0.0;
    double prev_error = 0.0;
};

class Ball {
public:
    SDL_Rect rect{0, 0, BALL_SIZE, BALL_SIZE};  // 修正：初始化x,y位置
    double y = WINDOW_HEIGHT / 2.0;
    double velocity = 0.0;

    Ball() {
        rect.x = WINDOW_WIDTH / 2 - BALL_SIZE / 2;  // 设置初始x位置
        rect.y = static_cast<int>(y);  // 设置初始y位置
    }

    void update(double force, double dt) {
        double acceleration = force - GRAVITY;
        velocity += acceleration * dt;
        y += velocity * dt;
        apply_boundary_constraints();
        rect.y = static_cast<int>(y);
    }

private:
    void apply_boundary_constraints() {
        if (y < 0) {
            y = 0;
            velocity *= -0.3;
        } else if (y > WINDOW_HEIGHT - BALL_SIZE) {
            y = WINDOW_HEIGHT - BALL_SIZE;
            velocity *= -0.3;
        }
    }
};

class App {
public:
    App() {
        if (SDL_Init(SDL_INIT_VIDEO)) throw std::runtime_error(SDL_GetError());
        if (TTF_Init()) throw std::runtime_error(TTF_GetError());

        window.reset(SDL_CreateWindow(
                "PID Control Simulator",
                SDL_WINDOWPOS_CENTERED,
                SDL_WINDOWPOS_CENTERED,
                WINDOW_WIDTH,
                WINDOW_HEIGHT,
                SDL_WINDOW_SHOWN
        ));
        if (!window) throw std::runtime_error(SDL_GetError());

        renderer.reset(SDL_CreateRenderer(
                window.get(),
                -1,
                SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC
        ));
        if (!renderer) throw std::runtime_error(SDL_GetError());

        load_font();
    }

    void run() {
        bool running = true;
        Uint32 last_time = SDL_GetTicks();
        double accumulator = 0.0;

        while (running) {
            Uint32 current_time = SDL_GetTicks();
            double frame_time = (current_time - last_time) / 1000.0;
            last_time = current_time;
            accumulator += frame_time;

            handle_events(running);

            while (accumulator >= FIXED_TIMESTEP) {
                update_physics(FIXED_TIMESTEP);
                accumulator -= FIXED_TIMESTEP;
            }

            render();
        }
    }

private:
    std::unique_ptr<SDL_Window, decltype(&SDL_DestroyWindow)> window{nullptr, SDL_DestroyWindow};
    std::unique_ptr<SDL_Renderer, decltype(&SDL_DestroyRenderer)> renderer{nullptr, SDL_DestroyRenderer};
    std::unique_ptr<TTF_Font, decltype(&TTF_CloseFont)> font{nullptr, TTF_CloseFont};

    Ball ball;
    PID_Controller pid{80.0, 0, 0};
    double setpoint = WINDOW_HEIGHT / 2.0;

    void load_font() {
        font.reset(TTF_OpenFont("C:/Windows/Fonts/arial.ttf", 24));
        if (!font) {
            font.reset(TTF_OpenFont("/usr/share/fonts/truetype/freefont/FreeSans.ttf", 24));
            if (!font) throw std::runtime_error(TTF_GetError());
        }
    }

    void handle_events(bool& running) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) running = false;
            else if (e.type == SDL_MOUSEBUTTONDOWN) {
                setpoint = e.button.y;
            }
            else if (e.type == SDL_KEYDOWN) {
                handle_keypress(e.key.keysym.sym);
            }
        }
    }

    void handle_keypress(SDL_Keycode key) {
        const double step = 5.0;
        switch (key) {
            case SDLK_UP:    pid.Kp += step; break;
            case SDLK_DOWN:  pid.Kp = std::max(0.0, pid.Kp - step); break;
            case SDLK_LEFT:  pid.Ki = std::max(0.0, pid.Ki - 0.1); break;
            case SDLK_RIGHT: pid.Ki += 0.1; break;
            case SDLK_PAGEUP:    pid.Kd += step; break;
            case SDLK_PAGEDOWN:  pid.Kd = std::max(0.0, pid.Kd - step); break;
            case SDLK_r: pid.reset(); break;
        }
    }

    void update_physics(double dt) {
        double force = pid.calculate(setpoint, ball.y + BALL_SIZE/2, dt);
        ball.update(force, dt);
    }

    void render() {
        // Clear screen
        SDL_SetRenderDrawColor(renderer.get(), 240, 240, 240, 255);
        SDL_RenderClear(renderer.get());

        // Draw setpoint line
        SDL_SetRenderDrawColor(renderer.get(), 0, 200, 0, 255);
        SDL_RenderDrawLine(renderer.get(),
                           0, static_cast<int>(setpoint),
                           WINDOW_WIDTH, static_cast<int>(setpoint)
        );

        // Draw ball
        SDL_SetRenderDrawColor(renderer.get(), 200, 0, 0, 255);
        SDL_RenderFillRect(renderer.get(), &ball.rect);

        // Render UI text
        render_text(
                "Controls:\n"
                "Mouse Click - Set Target\n"
                "Up/Down - Kp: " + std::to_string(pid.Kp) + "\n" +
                "Left/Right - Ki: " + std::to_string(pid.Ki) + "\n" +
                "PgUp/PgDn - Kd: " + std::to_string(pid.Kd) + "\n" +
                "R - Reset PID",
                10, 10
        );

        SDL_RenderPresent(renderer.get());
    }

    void render_text(const std::string& text, int x, int y) {
        SDL_Color black = {0, 0, 0, 255};
        SDL_Surface* surface = TTF_RenderText_Blended_Wrapped(font.get(), text.c_str(), black, WINDOW_WIDTH);
        SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer.get(), surface);

        SDL_Rect rect{x, y, surface->w, surface->h};
        SDL_RenderCopy(renderer.get(), texture, nullptr, &rect);

        SDL_FreeSurface(surface);
        SDL_DestroyTexture(texture);
    }
};

int main() {
    try {
        App app;
        app.run();
    } catch (const std::exception& e) {
        SDL_ShowSimpleMessageBox(
                SDL_MESSAGEBOX_ERROR,
                "Error",
                e.what(),
                nullptr
        );
        return 1;
    }
    return 0;
}