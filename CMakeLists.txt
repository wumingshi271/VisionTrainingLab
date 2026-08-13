cmake_minimum_required(VERSION 3.16)
project(VisionTrainingLab)

# 使用 C++ 17 标准
set(CMAKE_CXX_STANDARD 17)
# 强制编译器使用 C++ 17 标准
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 不使用自己扩展的 gnu++17 标准
# 而使用标准库中的标准
set(CMAKE_CXX_EXTENSIONS OFF)

# 依赖发现
# 发现 OpenCV 库
find_package(
    OpenCV REQUIRED 
        COMPONENTS 
            core
            calib3d
            imgproc
            imgcodecs
            highgui
)

# ----------------------------- # Subdirectories # -----------------------------

# 添加 task01 子目录
add_subdirectory(src/task01)
