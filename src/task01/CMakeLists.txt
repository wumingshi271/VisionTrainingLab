# ----------------------------- # ArmorPoseSolver library # -----------------------------

# 添加 task01_pose_solver 库
add_library(
    task01_pose_solver
    armor_pose_solver.cpp
)

# 添加 task01_pose_solver 库的头文件目录
target_include_directories(
    task01_pose_solver
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
)

# 添加 task01_pose_solver 库的链接库
target_link_libraries(
    task01_pose_solver
    PUBLIC
        ${OpenCV_LIBS}
)

# 添加 task01_pose_solver 库的编译特性
target_compile_features(
    task01_pose_solver
    PUBLIC
        cxx_std_17
)


# ----------------------------- # Task01 executable # -----------------------------

# 添加 task01 可执行文件
add_executable(
    task01
    task01_main.cpp
)

# 添加 task01 可执行文件的链接库
target_link_libraries(
    task01
    PRIVATE
        task01_pose_solver
)

# 添加 task01 可执行文件的输出目录
set_target_properties(
    task01
    PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY
            ${PROJECT_BINARY_DIR}/bin
)

# 添加 task01 可执行文件的编译特性
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(
        task01_pose_solver
        PRIVATE
            -Wall
            -Wextra
            -Werror
    )

    target_compile_options(
        task01
        PRIVATE
            -Wall
            -Wextra
            -Werror
    )
endif()
