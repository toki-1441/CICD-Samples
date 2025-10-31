#include "param_chatter_cpp/param_chatter_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include <memory>

int main(int argc, char * argv[])
{
  // 1. ROS2の初期化
  rclcpp::init(argc, argv);

  // 2. ノードオプションの作成（推奨される方法）
  rclcpp::NodeOptions options;
  
  // 3. ノードのインスタンス化 (std::make_shared を使用)
  auto node = std::make_shared<param_chatter_cpp::ParamChatterNode>(options);

  // 4. ノードをスピン（コールバック処理を開始）
  // KeyboardInterrupt (Ctrl+C) があるまでブロックされます。
  rclcpp::spin(node);

  // 5. ROS2のシャットダウン処理
  rclcpp::shutdown();
  return 0;
}