cmake_minimum_required(VERSION 3.5)
project(iris_lama LANGUAGES CXX C)

set(IRIS_LAMA_VERSION 1.3.1)

# provides default CMAKE_INSTALL_<dir> variables
include(GNUInstallDirs)
# helpers to generate <package>Config.cmake
include(CMakePackageConfigHelpers)

# install location of generated cmake config files
set(ConfigPackageLocation ${CMAKE_INSTALL_LIBDIR}/${CMAKE_PROJECT_NAME}/cmake)

# dependencies
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# the code
add_subdirectory(src)

# header files, note the "/" after include
install(DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# for build tree / in workspace
export(EXPORT ${CMAKE_PROJECT_NAME}Targets
  FILE ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Targets.cmake
  NAMESPACE ${CMAKE_PROJECT_NAME}::)
# exported targets
install(EXPORT ${CMAKE_PROJECT_NAME}Targets
  FILE ${CMAKE_PROJECT_NAME}Targets.cmake
  NAMESPACE ${CMAKE_PROJECT_NAME}::
  DESTINATION ${ConfigPackageLocation})

# install the package configuration files
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}ConfigVersion.cmake
  DESTINATION ${ConfigPackageLocation})

