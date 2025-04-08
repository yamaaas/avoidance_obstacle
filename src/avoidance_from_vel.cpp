#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <limits> // std::numeric_limits のために必要
#include <vector> // std::vector のために必要
#include <algorithm> // std::minmax_element, std::min, std::clamp のために必要

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp" // 必要なら残すが、ここでは直接使わない
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"

// M_PI が定義されていない場合（プラットフォームによる）のために定義
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class LidarAvoidCmd : public rclcpp::Node {

public:
    // コンストラクタ
    LidarAvoidCmd() : Node("lidar_avoid_cmd") {
        // --- パラメータ設定 ---
        this->declare_parameter<bool>("iswork", true);
        this->declare_parameter<double>("distance_threshold", 0.5);
        this->declare_parameter<int>("detection_threshold", 3);
        // avoidance_fov_deg は矩形領域検知では直接使わないが、パラメータとして残す
        this->declare_parameter<double>("avoidance_fov_deg", 60.0);
        this->declare_parameter<double>("avoidance_gain", 0.5);
        this->declare_parameter<double>("min_avoidance_dist", 0.15);
        this->declare_parameter<double>("max_angular_vel", 1.0);
        this->declare_parameter<double>("min_linear_vel_factor", 0.2);
        this->declare_parameter<double>("robot_width", 0.1);
        this->declare_parameter<double>("robot_length", 0.1);
        this->declare_parameter<double>("lidar_forward_offset", 0.1);
        this->declare_parameter<double>("lidar_lateral_offset", 0.0); // 簡単のため0と仮定


        // 初期パラメータ値の取得
        this->get_parameter("iswork", isWork_);
        obstacle_detection_threshold_ = this->get_parameter("detection_threshold").as_int();

        // --- サブスクライバ (データ受信) ---
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan",
            rclcpp::SensorDataQoS(),
            std::bind(&LidarAvoidCmd::scan_callback, this, std::placeholders::_1));

        cmd_vel_remote_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel_smooth",
            10,
            std::bind(&LidarAvoidCmd::cmd_vel_remoteCb, this, std::placeholders::_1));

        iswork_sub_ = this -> create_subscription<std_msgs::msg::Bool>(
            "iswork",
            10,
            std::bind(&LidarAvoidCmd::iswork_Cb, this, std::placeholders::_1));

        // --- パブリッシャ (データ送信) ---
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "cmd_vel",
            10);

        RCLCPP_INFO(this->get_logger(), "Lidar Avoidance Node started.");
        RCLCPP_INFO(this->get_logger(), "Params: iswork=%d, dist_thresh=%.2f, detect_thresh=%d, gain=%.2f, min_dist=%.2f, robot_w=%.2f, robot_l=%.2f, lidar_fwd=%.2f",
                    isWork_, this->get_parameter("distance_threshold").as_double(), obstacle_detection_threshold_,
                    this->get_parameter("avoidance_gain").as_double(),
                    this->get_parameter("min_avoidance_dist").as_double(),
                    this->get_parameter("robot_width").as_double(),
                    this->get_parameter("robot_length").as_double(),
                    this->get_parameter("lidar_forward_offset").as_double());
    }

private:
    // 状態を示すための単純なステートマシン
    enum class State { NORMAL, AVOIDING };
    State state_ = State::NORMAL;

    // サブスクリプションとパブリッシャのポインタ
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_remote_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr iswork_sub_;

    // 保存されるメッセージ
    geometry_msgs::msg::Twist cmd_vel_remote_; // 最後に受信した外部からの速度指令

    // パラメータ (キャッシュされた値)
    bool isWork_;
    int obstacle_detection_threshold_;

    // 内部状態変数
    int obstacle_consecutive_count_ = 0;
    bool obstacle_currently_detected_ = false;
    double closest_obstacle_dist_ = std::numeric_limits<double>::infinity();
    double closest_obstacle_angle_ = 0.0;     // LIDAR正面(0rad)に対する最も近い障害物の角度

    // --- コールバック関数 ---

    void iswork_Cb(const std_msgs::msg::Bool::SharedPtr msg) {
        if (isWork_ != msg->data) {
            isWork_ = msg->data;
            RCLCPP_INFO(this->get_logger(), "isWork changed to: %s", isWork_ ? "true" : "false");
            if (!isWork_) {
                state_ = State::NORMAL;
                obstacle_consecutive_count_ = 0;
                obstacle_currently_detected_ = false;
                publish_velocity(); // 現在のリモートコマンドを発行して停止または通常動作に戻す
            }
        }
    }

    void cmd_vel_remoteCb(const geometry_msgs::msg::Twist::SharedPtr msg) {
        cmd_vel_remote_ = *msg;
        // isWorkがtrueで、かつ回避中でない場合のみここで速度を発行する
        // 回避中の速度発行は scan_callback 内の publish_velocity で行う
        if (isWork_ && state_ != State::AVOIDING) {
             publish_velocity();
        } else if (!isWork_) {
             // isWorkがfalseなら常にリモートコマンドをそのまま流す
             publish_velocity();
        }
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (!isWork_) {
            return; // 回避機能が無効なら何もしない
        }
        process_scan_data(msg); // スキャンデータを処理
        update_state();         // 状態を更新
        publish_velocity();     // 状態に基づき速度指令を発行 (AVOIDING状態なら回避計算を含む)
    }

    // スキャンデータを処理し、指定した矩形領域内の最も近い障害物情報を見つける関数
    void process_scan_data(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        closest_obstacle_dist_ = std::numeric_limits<double>::infinity();
        closest_obstacle_angle_ = 0.0;
        bool obstacle_found_in_scan = false;

        // パラメータ取得
        double margin = this->get_parameter("distance_threshold").as_double();
        double robot_width = this->get_parameter("robot_width").as_double();
        // robot_length は今回の計算では直接使わないが、将来のためにパラメータは維持
        double robot_length = this->get_parameter("robot_length").as_double();
        double lidar_forward_offset = this->get_parameter("lidar_forward_offset").as_double();
        // LIDARの左右オフセットも考慮に入れる（今回は簡単のため0と仮定）
        // double lidar_lateral_offset = this->get_parameter("lidar_lateral_offset").as_double();

        // 検知領域の定義 (LIDAR座標系, X:前方, Y:左方)
        // 考慮するX方向の範囲: LIDAR位置から、ロボット前面 + マージンまで
        double x_min_detect = robot_length/2; // LIDAR位置より前方のみ
        double x_max_detect = robot_length/2 + margin;
        // 考慮するY方向の範囲: ロボットの幅 + 左右マージン
        // LIDAR中心を基準とするため、ロボット幅の半分にマージンを加える
        double y_max_abs_detect = (robot_width / 2.0) + margin;

        // デバウンス閾値
        int obstacle_detection_threshold = this->get_parameter("detection_threshold").as_int();

        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            double current_angle = msg->angle_min + i * msg->angle_increment;
            double current_dist = msg->ranges[i];

            if (!std::isfinite(current_dist) || current_dist <= msg->range_min || current_dist >= msg->range_max) {
                continue;
            }

            // LIDAR座標系へ変換 (X:前方, Y:左方)
            double x = current_dist * std::cos(current_angle);
            double y = -current_dist * std::sin(current_angle);

            // --- 矩形領域内の判定 ---
            // X座標が指定範囲内か？
            // Y座標が指定範囲内か？ (絶対値で左右両方を見る)
           // bool in_detection_zone = (abs(x + robot_length/2 - lidar_forward_offset) >= x_min_detect && abs(x + robot_length/2 - lidar_forward_offset) < x_max_detect &&
            //std::abs(y) < y_max_abs_detect && abs(y) >= robot_width/2);
            
            bool in_detection_zone = true;

            if((abs(y) < y_max_abs_detect) && (abs(x + (robot_length/2) - lidar_forward_offset) < x_max_detect)){
              if(abs(y)>(robot_width/2) || (abs(x + (robot_length/2) - lidar_forward_offset) >= x_min_detect)){
                in_detection_zone = true;
              }
              else {
                in_detection_zone = false;
              }
            }
            else{
              in_detection_zone = false;
            }



            if (in_detection_zone) {
                obstacle_found_in_scan = true;
                // 最も近い障害物情報を更新 (LIDARからの直接距離と角度)
                if (current_dist < closest_obstacle_dist_) {
                    closest_obstacle_dist_ = current_dist;
                    closest_obstacle_angle_ = current_angle;
                }
            }
        }

        // --- デバウンス処理 ---
        if (obstacle_found_in_scan) {
            obstacle_consecutive_count_++;
        } else {
            obstacle_consecutive_count_ = 0;
        }

        // 閾値を超えたか判定し、状態フラグを更新
        bool prev_detected_state = obstacle_currently_detected_;
        if (obstacle_consecutive_count_ >= obstacle_detection_threshold) {
            obstacle_currently_detected_ = true;
            if (!prev_detected_state) { // 検出開始のログ
                RCLCPP_INFO(this->get_logger(), "Obstacle detected in rectangular zone: dist=%.2f, angle=%.2f rad",
                            closest_obstacle_dist_, closest_obstacle_angle_*180./M_PI);
            }
        } else if (obstacle_consecutive_count_ == 0) {
            obstacle_currently_detected_ = false;
            if (prev_detected_state) { // 検出終了のログ
                RCLCPP_INFO(this->get_logger(), "Obstacle cleared from rectangular zone.");
            }
        }
        // 0 < count < threshold の間はフラグ維持
    }


    // ロボットの状態 (NORMAL / AVOIDING) を更新する関数
    void update_state() {
        State previous_state = state_;

        // AVOIDING 状態に遷移する条件:
        // 1. 回避機能が有効 (isWork_)
        // 2. 障害物が継続的に検知されている (obstacle_currently_detected_)
        // 3. ロボットが前進するように指令されている (cmd_vel_remote_.linear.x > 0.01)
        //    ※静止状態(linear.x=0, angular.z=0)では回避状態に入らない。
        //      静止時に障害物を検知しても、次の移動指令が来るまではNORMAL状態を維持。
        if (isWork_ && obstacle_currently_detected_ && cmd_vel_remote_.linear.x > 0.01) {
             state_ = State::AVOIDING;
        } else {
             state_ = State::NORMAL; // 上記条件を満たさなければ NORMAL
        }

        // 状態が変化した場合にログを出力
        if(state_ != previous_state) {
            RCLCPP_INFO(this->get_logger(), "State changed from %s to %s",
                       (previous_state == State::NORMAL ? "NORMAL" : "AVOIDING"),
                       (state_ == State::NORMAL ? "NORMAL" : "AVOIDING"));
        }
    }

    // 速度指令を発行する関数
    void publish_velocity() {
        geometry_msgs::msg::Twist cmd_msg;

        if (!isWork_ || state_ == State::NORMAL) {
            // 回避機能無効、または NORMAL 状態の場合 -> 外部指令をそのまま使用
            cmd_msg = cmd_vel_remote_;
        } else { // state_ == State::AVOIDING の場合 -> 回避ロジックを適用
            // --- パラメータ取得 ---
            double avoidance_gain = this->get_parameter("avoidance_gain").as_double();
            double min_avoidance_dist = this->get_parameter("min_avoidance_dist").as_double();
            double max_angular_vel = this->get_parameter("max_angular_vel").as_double();
            double min_linear_vel_factor = this->get_parameter("min_linear_vel_factor").as_double();
            double distance_threshold = this->get_parameter("distance_threshold").as_double();

            // --- 目標移動方向と障害物との相対角度に基づく回避計算 ---

            // 1. 目標とする移動方向の角度 (ロボット座標系) を決定
            //    - cmd_vel_remote_.linear.x > 0 の場合（AVOIDING状態に入る条件）、
            //      主要な目標方向は「前方」(0 rad) とみなす。
            //    - 静止状態 (linear.x=0, angular.z=0) は update_state で除外されているが、
            //      仮に静止時も回避するなら、目標方向は「前方」と考える（ユーザー指定）。
            //    - 瞬間的な旋回 (angular.z) はここでは考慮せず、障害物の相対位置に基づいて回避方向を決める。
            double target_direction_angle = 0.0; // [rad] ロボットの前方を目標方向とする

            // 2. 障害物の角度 (ロボット座標系、LIDAR正面が0rad)
            double obstacle_angle = closest_obstacle_angle_; // [rad]

            // 3. 目標移動方向に対する障害物の相対角度を計算
            //    relative_angle = obstacle_angle - target_direction_angle
            //    target_direction_angle が 0 なので、 relative_angle = obstacle_angle となる。
            double relative_angle_to_target = obstacle_angle; // [rad]

            // 4. 回避のための追加旋回速度 (obstacle_avoidance_turn) を計算
            //    - 旋回方向: 目標方向に対して障害物がある方向と逆向きに旋回する。
            //      - relative_angle_to_target > 0 (目標の左に障害物) -> 右旋回 (負の角速度)
            //      - relative_angle_to_target < 0 (目標の右に障害物) -> 左旋回 (正の角速度)
            //    - 旋回強度:
            //      - 障害物までの距離 (closest_obstacle_dist_) が近いほど強く旋回する。
            //      - 距離による係数 (turn_strength_factor) を計算 (0.0〜1.0)。
            //        min_avoidance_dist で最大強度(1.0)、distance_threshold で強度0(0.0)。
            double turn_strength_factor = (distance_threshold - closest_obstacle_dist_) / (distance_threshold - min_avoidance_dist + 1e-6); // ゼロ割防止
            turn_strength_factor = std::clamp(turn_strength_factor, 0.0, 1.0); // 0.0から1.0の間に制限

            // 回避旋回速度 = 旋回方向決定因子 * 基本ゲイン * 距離による強度係数
            // copysign(1.0, angle) は angle の符号を返す (+1 or -1)。- をつけて逆方向にする。
            double obstacle_avoidance_turn = -std::copysign(1.0, relative_angle_to_target + 1e-6) // ゼロ付近の不安定さ回避
                                             * avoidance_gain
                                             * turn_strength_factor;

            // 5. 最終的な角速度を計算
            //    元の指令角速度 (cmd_vel_remote_.angular.z) に回避旋回成分を加算する。
            cmd_msg.angular.z = cmd_vel_remote_.angular.z + obstacle_avoidance_turn;
            // 最大角速度で制限
            cmd_msg.angular.z = std::clamp(cmd_msg.angular.z, -max_angular_vel, max_angular_vel);

            // 6. 最終的な並進速度を計算
            //    - 障害物に近づくほど速度を落とす。
            //    - 距離による速度低減係数 (speed_reduction_factor) を計算 (min_linear_vel_factor〜1.0)。
            //      min_avoidance_dist で最低速度比率(min_linear_vel_factor)、distance_threshold で元の速度(1.0)。
            double speed_reduction_factor = (closest_obstacle_dist_ - min_avoidance_dist) / (distance_threshold - min_avoidance_dist + 1e-6); // ゼロ割防止
            speed_reduction_factor = std::clamp(speed_reduction_factor, min_linear_vel_factor, 1.0); // min_linear_vel_factor から 1.0 の間に制限

            // 元の指令速度に低減係数をかける
            cmd_msg.linear.x = cmd_vel_remote_.linear.x * speed_reduction_factor;
            // 並進速度が負にならないように保証 (AVOIDINGに入る条件から通常は正のはず)
            cmd_msg.linear.x = std::max(0.0, cmd_msg.linear.x);

            // 7. 緊急停止条件
            //    障害物が非常に近い場合 (min_avoidance_dist 未満) は完全に停止
            if (closest_obstacle_dist_ < min_avoidance_dist) {
                 RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                      "Obstacle too close (%.2f m < %.2f m)! Stopping linear motion.",
                                      closest_obstacle_dist_, min_avoidance_dist);
                 cmd_msg.linear.x = 0.0;
                 // この場合、回避旋回は継続する設定。必要なら旋回も停止させる。
                 // cmd_msg.angular.z = 0.0; // 必要なら追加
            }

            // --- デバッグログ ---
            RCLCPP_INFO(this->get_logger(), "AVOIDING: Obst(d=%.2f, a=%.2f), Remote(lx=%.2f, az=%.2f), TargetAngle=%.2f, RelAngle=%.2f, AvoidTurn=%.2f -> Final(lx=%.2f, az=%.2f)",
                        closest_obstacle_dist_, obstacle_angle,
                        cmd_vel_remote_.linear.x, cmd_vel_remote_.angular.z,
                        target_direction_angle, relative_angle_to_target,
                        obstacle_avoidance_turn,
                        cmd_msg.linear.x, cmd_msg.angular.z);
        }

        // 計算された最終的な速度指令をパブリッシュ
        cmd_vel_pub_->publish(cmd_msg);
    }

}; // クラス LidarAvoidCmd の終わり

// main 関数
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarAvoidCmd>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
