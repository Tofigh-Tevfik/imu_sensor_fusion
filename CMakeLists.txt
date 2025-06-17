cmake_minimum_required(VERSION 3.10)
project(imu_sensor_fusion)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(IMU_SOURCE_DIR ${CMAKE_SOURCE_DIR}/src/imu)

add_library(imu_reader STATIC
    ${IMU_SOURCE_DIR}/imu_reader.cpp
    ${IMU_SOURCE_DIR}/i2c_reader.cpp
)

target_include_directories(imu_reader PUBLIC ${IMU_SOURCE_DIR})

find_library(I2C_LIBRARY i2c)
if(I2C_LIBRARY)
    target_link_libraries(imu_reader PUBLIC ${I2C_LIBRARY})
else()
    message(WARNING "I2C library not found. I2CReader will not link properly.")
endif()

add_executable(imu_app src/main.cpp)
target_link_libraries(imu_app PRIVATE imu_reader)

target_compile_options(imu_reader PRIVATE -Wall -Wextra -Wpedantic)
target_compile_options(imu_app PRIVATE -Wall -Wextra -Wpedantic)