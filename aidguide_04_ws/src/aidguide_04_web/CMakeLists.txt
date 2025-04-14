cmake_minimum_required(VERSION 3.8)
project(aidguide_04_web)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Instalar los archivos del frontend
install(DIRECTORY
  app
  components
  context
  hooks
  lib
  public
  styles
  DESTINATION share/${PROJECT_NAME}
)

# Instalar archivos de configuración
install(FILES
  package.json
  tsconfig.json
  tailwind.config.ts
  postcss.config.mjs
  next.config.mjs
  components.json
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package() 