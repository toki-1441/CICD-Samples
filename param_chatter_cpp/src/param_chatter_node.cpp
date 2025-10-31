#include "param_chatter_cpp/param_chatter_node.hpp"
#include <memory>

namespace param_chatter_cpp
{

// コンストラクタの実装
ParamChatterNode::ParamChatterNode(const rclcpp::NodeOptions & options)
: Node("simple_param_chatter", options) // ノード名を "simple_param_chatter" に設定
{
  // 1. パラメータの宣言とデフォルト値の設定
  // パラメータ名: "word", デフォルト値: "Hello C++"
  this->declare_parameter<std::string>("word", "Hello C++");

  // 2. パブリッシャーの作成
  // トピック名: "chatter", QoS: 10
  publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);

  // 3. タイマーの作成
  // 1.0秒ごとに timer_callback 関数を呼び出す
  timer_ = this->create_wall_timer(
    1s, // (using namespace std::chrono_literals; が必要)
    std::bind(&ParamChatterNode::timer_callback, this) // コールバック関数をバインド
  );

  RCLCPP_INFO(this->get_logger(), "Parameter chatter node has started.");
  RCLCPP_INFO(this->get_logger(), "Publishing to '/chatter' topic every 1 second.");
}

// タイマーコールバックの実装
void ParamChatterNode::timer_callback()
{
  // 4. パラメータサーバーから現在の "word" の値を取得
  std::string my_word;
  this->get_parameter("word", my_word);

  // 5. メッセージを作成
  auto msg = std_msgs::msg::String();
  msg.data = "Chattering: " + my_word;

  // 6. メッセージをパブリッシュ
  publisher_->publish(msg);

  // 7. ログに出力
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.data.c_str());
}

} // namespace param_chatter_cpp