cmake_minimum_required(VERSION 3.8)
project(param_chatter_cpp)

# C++標準 (C++17以上を推奨)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 必要なパッケージを検索
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 1. ノードクラスのライブラリを作成
# これにより、ノードクラスを他の実行ファイルやコンポーネントから再利用できます。
add_library(param_chatter_node_lib
  src/param_chatter_node.cpp
)

# ライブラリに必要な依存関係をリンク
ament_target_dependencies(param_chatter_node_lib
  rclcpp
  std_msgs
)

# 2. 実行可能ファイルの作成
add_executable(param_chatter_node
  src/param_chatter_main.cpp
)

# 実行可能ファイルにノードライブラリをリンク
target_link_libraries(param_chatter_node
  param_chatter_node_lib
)

# 実行可能ファイルに必要な依存関係をリンク
ament_target_dependencies(param_chatter_node
  rclcpp
)

# 3. インクルードディレクトリの設定
target_include_directories(param_chatter_node_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# 4. インストール設定
install(
  TARGETS
    param_chatter_node
    param_chatter_node_lib
  DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# ヘッダファイルをインストール
install(
  DIRECTORY include/
  DESTINATION include
)

# 5. amentのリソースを登録
ament_export_dependencies(rclcpp std_msgs)
ament_export_targets(param_chatter_node_lib)
ament_export_include_directories(include)

ament_package()