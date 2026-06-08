add_executable(test_ecs test_ecs.cpp)
target_link_libraries(test_ecs PRIVATE voxel_ecs)
add_test(NAME test_ecs COMMAND test_ecs)

add_executable(test_voxel test_voxel.cpp)
target_link_libraries(test_voxel PRIVATE voxel_ecs)
add_test(NAME test_voxel COMMAND test_voxel)

add_executable(test_new_features test_new_features.cpp)
target_link_libraries(test_new_features PRIVATE voxel_ecs)
add_test(NAME test_new_features COMMAND test_new_features)

find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    target_link_libraries(test_ecs PRIVATE OpenMP::OpenMP_CXX)
    target_link_libraries(test_voxel PRIVATE OpenMP::OpenMP_CXX)
    target_link_libraries(test_new_features PRIVATE OpenMP::OpenMP_CXX)
endif()
