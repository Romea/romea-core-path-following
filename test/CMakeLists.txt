find_package(GTest REQUIRED)


add_executable(${PROJECT_NAME}_test_fsm test_fsm.cpp)
target_link_libraries(${PROJECT_NAME}_test_fsm ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_fsm PRIVATE -std=c++17)
add_test(test_fsm ${PROJECT_NAME}_test_fsm)

add_executable(${PROJECT_NAME}_test_set_point test_set_point.cpp)
target_link_libraries(${PROJECT_NAME}_test_set_point ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_set_point PRIVATE -std=c++17)
add_test(test_set_point ${PROJECT_NAME}_test_set_point)
