#pragma once // ヘッダの二重インクルード防止

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// C++14以降で標準化されたリテラル (例: 1s) を使うため
using namespace std::chrono_literals;

namespace param_chatter_cpp
{

/**
 * @class ParamChatterNode
 * @brief パラメータで指定された単語をパブリッシュするノードクラス
 */
class ParamChatterNode : public rclcpp::Node
{
public:
  /**
   * @brief コンストラクタ
   * @param options ノードの初期化オプション
   */
  explicit ParamChatterNode(const rclcpp::NodeOptions & options);

private:
  /**
   * @brief タイマーコールバック関数。定期的に呼び出されます。
   */
  void timer_callback();

  // --- メンバ変数 ---

  // パブリッシャー (std_msgs::msg::String 型)
  // 専門的には、shared_ptrで管理するのがROS2の標準です。
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  // タイマー
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace param_chatter_cpp