cmake_minimum_required(VERSION 3.15)
project(SDL_game)

set(CMAKE_CXX_STANDARD 17)

# 设置SDL2和SDL2_ttf的路径
set(SDL2_PATH "D:/MyProject/SDL_game/SDL2-devel-2.30.2-mingw/SDL2-2.30.2/x86_64-w64-mingw32")
set(SDL2_TTF_PATH "D:/MyProject/SDL_game/SDL2_ttf-devel-2.22.0-mingw/SDL2_ttf-2.22.0/x86_64-w64-mingw32")

# 包含目录
include_directories(
        ${SDL2_PATH}/include
        ${SDL2_TTF_PATH}/include
)

# 链接目录
link_directories(
        ${SDL2_PATH}/lib
        ${SDL2_TTF_PATH}/lib
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