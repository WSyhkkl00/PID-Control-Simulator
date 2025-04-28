cmake_minimum_required(VERSION 3.15)
project(SDL_game)

set(CMAKE_CXX_STANDARD 17)

# 设置SDL2和SDL2_ttf的路径
set(SDL2_PATH "D:/MyProject/SDL_game/SDL2")


# 包含目录
include_directories(
        ${SDL2_PATH}/include
        ${SDL2_TTF_PATH}/include
)

# 链接目录
link_directories(
        ${SDL2_PATH}/lib/SDL
        ${SDL2_PATH}/lib/SDL_ttf
)


# 添加可执行文件
add_executable(SDL_game main.cpp)

# 链接库
target_link_libraries(SDL_game
        mingw32
        SDL2main
        SDL2
        SDL2_ttf
)