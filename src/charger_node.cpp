/**
 * @file charger_node.cpp
 * @brief 补能系统控制节点（对应仿真器中的第二只海龟 turtle2）
 * 
 * 核心功能：
 * 1. 初始化时调用 /spawn 服务在补能站 (5.5, 5.5) 外围 (6.5, 5.5) 生成 turtle2；
 * 2. 提供 /request_charging 服务供巡检机器人回撤到站后发起补能；
 * 3. 订阅 /battery_temperature 话题，使用基于循环缓冲区（Circular Buffer）的滑动平均滤波去噪；
 * 4. 充电中控制 turtle2 围绕巡检海龟匀速绕圈画圆（v=1.0, w=1.0）；
 * 5. 过温保护闭环：温度超过损毁阈值（60°C）切断供电并停转，冷却至 42°C 后自动恢复充电；
 * 6. 达到预设有效充电时长后，结束充电并通知巡检海龟。
 */

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/set_pen.hpp"

#include "sheep_patrol/circular_buffer_filter.hpp"

using namespace std::chrono_literals;

namespace sheep_patrol
{

enum class ChargerState
{
    IDLE,               ///< 待命状态：等待巡检机器人到站发起补能
    CHARGING_ACTIVE,    ///< 正常充电状态：turtle2 绕圈，供电产热中
    OVERHEAT_COOLDOWN,  ///< 过温保护状态：切断电源，turtle2 停转，自然散热中
    CHARGING_DONE       ///< 充电完成状态
};

class ChargerNode : public rclcpp::Node
{
public:
    ChargerNode()
    : Node("charger_node"),
      state_(ChargerState::IDLE),
      current_power_(0.0),
      last_raw_temp_(25.0),
      last_filtered_temp_(25.0),
      effective_charge_seconds_(0.0),
      tick_counter_(0),
      turtle2_spawned_(false)
    {
        // 1. 声明并读取关键参数
        this->declare_parameter<double>("power_charge", 100.0);         // 充电产热功率 (W)
        this->declare_parameter<double>("overheat_temp", 60.0);         // 损毁报警阈值 (°C)
        this->declare_parameter<double>("cooling_resume_temp", 42.0);   // 冷却恢复阈值 (°C)
        this->declare_parameter<double>("target_charge_time", 15.0);    // 目标有效充电时长 (秒)
        this->declare_parameter<int>("filter_window_size", 15);         // 循环缓冲区窗口长度

        power_charge_ = this->get_parameter("power_charge").as_double();
        overheat_temp_ = this->get_parameter("overheat_temp").as_double();
        cooling_resume_temp_ = this->get_parameter("cooling_resume_temp").as_double();
        target_charge_time_ = this->get_parameter("target_charge_time").as_double();

        RCLCPP_INFO(get_logger(), "==================================================");
        RCLCPP_INFO(get_logger(), "补能系统节点 (charger_node) 正在启动...");
        RCLCPP_INFO(get_logger(), "配置参数：供电功率=%.1fW, 损毁阈值=%.1f°C, 恢复阈值=%.1f°C, 目标充能=%.1fs",
                    power_charge_, overheat_temp_, cooling_resume_temp_, target_charge_time_);
        RCLCPP_INFO(get_logger(), "数据结构：基于定长循环缓冲区 (N=%zu) 的滑动平均滤波", filter_.capacity());
        RCLCPP_INFO(get_logger(), "==================================================");

        // 2. 通信端口创建
        // 2.1 发布 turtle2 速度（控制绕圈）
        turtle2_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle2/cmd_vel", 10);

        // 2.2 发布充电供电功率指令（向巡检海龟传达当前 P 值）
        power_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64>("/charger_power_cmd", 10);

        // 2.3 发布充电完成通知
        done_pub_ = this->create_publisher<std_msgs::msg::Empty>("/charging_done", 10);

        // 2.4 订阅巡检机器人的电池温度遥测
        temp_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/battery_temperature", 10,
            std::bind(&ChargerNode::temperature_callback, this, std::placeholders::_1));

        // 2.5 补能服务端：等待巡检海龟提交补能请求
        charge_srv_ = this->create_service<std_srvs::srv::SetBool>(
            "/request_charging",
            std::bind(&ChargerNode::handle_charging_request, this, std::placeholders::_1, std::placeholders::_2));

        // 2.6 客户端：调用 turtlesim /spawn 生成 turtle2
        spawn_client_ = this->create_client<turtlesim::srv::Spawn>("/spawn");
        set_pen_client_ = this->create_client<turtlesim::srv::SetPen>("/turtle2/set_pen");

        // 3. 定时器：主控制循环（50ms / 20Hz）
        timer_ = this->create_wall_timer(
            50ms, std::bind(&ChargerNode::control_loop, this));

        // 4. 尝试生成第二只乌龟
        spawn_turtle2();
    }

private:
    /**
     * @brief 生成第二只乌龟作为补能车/充电桩
     */
    void spawn_turtle2()
    {
        if (!spawn_client_->wait_for_service(2s)) {
            RCLCPP_WARN(get_logger(), "turtlesim /spawn 服务暂未就绪，将在稍后重试生成 turtle2...");
            return;
        }

        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        // 补能站中心为 (5.5, 5.5)，生成在 (6.5, 5.5)，半径恰为 1.0，朝向正北 (PI/2)
        request->x = 6.5;
        request->y = 5.5;
        request->theta = M_PI_2;
        request->name = "turtle2";

        spawn_client_->async_send_request(
            request,
            [this](rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture future) {
                try {
                    auto resp = future.get();
                    turtle2_spawned_ = true;
                    RCLCPP_INFO(this->get_logger(), "成功在坐标 (6.5, 5.5) 生成补能海龟：%s", resp->name.c_str());
                    // 设置 turtle2 画笔为金黄色 (RGB: 255, 215, 0)，粗细为 2
                    this->set_turtle2_pen(255, 215, 0, 2, 0);
                } catch (const std::exception &e) {
                    RCLCPP_WARN(this->get_logger(), "生成 turtle2 异常（可能已存在）：%s", e.what());
                    turtle2_spawned_ = true;
                }
            });
    }

    /**
     * @brief 设置 turtle2 画笔颜色
     */
    void set_turtle2_pen(int r, int g, int b, int width, int off)
    {
        if (!set_pen_client_->wait_for_service(500ms)) return;
        auto req = std::make_shared<turtlesim::srv::SetPen::Request>();
        req->r = r; req->g = g; req->b = b; req->width = width; req->off = off;
        set_pen_client_->async_send_request(req);
    }

    /**
     * @brief 响应巡检机器人发来的充电服务请求
     */
    void handle_charging_request(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        if (request->data) {
            RCLCPP_INFO(get_logger(), "==================================================");
            RCLCPP_INFO(get_logger(), "收到巡检机器人发来的补能申请！");
            RCLCPP_INFO(get_logger(), "连接充电回路，启动产热供电 (P = %.1f W)...", power_charge_);
            RCLCPP_INFO(get_logger(), "补能海龟开始围绕巡检海龟画圆指示充电中！");
            RCLCPP_INFO(get_logger(), "==================================================");

            state_ = ChargerState::CHARGING_ACTIVE;
            effective_charge_seconds_ = 0.0;
            filter_.reset();
            current_power_ = power_charge_;
            publish_power(current_power_);

            response->success = true;
            response->message = "补能系统已接通，充电中！";
        } else {
            state_ = ChargerState::IDLE;
            current_power_ = 0.0;
            publish_power(0.0);
            stop_turtle2();
            response->success = true;
            response->message = "充电已手动关停。";
        }
    }

    /**
     * @brief 接收巡检机器人自检发布的温度，经过循环缓冲区滑动均值滤波并执行过温保护
     */
    void temperature_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        last_raw_temp_ = msg->data;
        // 核心数据结构调用：压入循环缓冲区，计算当前窗口平滑平均值
        last_filtered_temp_ = filter_.update(last_raw_temp_);

        // 状态机过温检测
        if (state_ == ChargerState::CHARGING_ACTIVE)
        {
            if (last_filtered_temp_ >= overheat_temp_)
            {
                // 超出损毁阈值：切断供电，触发过温自愈保护
                state_ = ChargerState::OVERHEAT_COOLDOWN;
                current_power_ = 0.0;
                publish_power(0.0);
                stop_turtle2();

                RCLCPP_WARN(get_logger(), "*****************************************************");
                RCLCPP_WARN(get_logger(), "【过温保护触发】滤波后温度达到 %.2f °C (超温阈值: %.1f °C)！",
                            last_filtered_temp_, overheat_temp_);
                RCLCPP_WARN(get_logger(), "补能系统紧急切断电源 (P=0.0W)！补能海龟停止绕圈！");
                RCLCPP_WARN(get_logger(), "等待电池自然散热冷却至 %.1f °C 以下...", cooling_resume_temp_);
                RCLCPP_WARN(get_logger(), "*****************************************************");
            }
        }
        else if (state_ == ChargerState::OVERHEAT_COOLDOWN)
        {
            if (last_filtered_temp_ <= cooling_resume_temp_)
            {
                // 冷却至安全阈值以下：恢复供电充电
                state_ = ChargerState::CHARGING_ACTIVE;
                current_power_ = power_charge_;
                publish_power(current_power_);

                RCLCPP_INFO(get_logger(), "-----------------------------------------------------");
                RCLCPP_INFO(get_logger(), "【电池冷却完毕】滤波后温度已降至 %.2f °C (低于恢复阈值: %.1f °C)！",
                            last_filtered_temp_, cooling_resume_temp_);
                RCLCPP_INFO(get_logger(), "重新恢复供电 (P=%.1fW)，补能海龟恢复画圆继续充电！", power_charge_);
                RCLCPP_INFO(get_logger(), "-----------------------------------------------------");
            }
        }
    }

    /**
     * @brief 主控制循环（50ms 周期）
     */
    void control_loop()
    {
        // 补能小海龟运动控制与充电时长统计
        if (state_ == ChargerState::CHARGING_ACTIVE)
        {
            // 累计有效充电时长
            effective_charge_seconds_ += 0.05;

            // 控制 turtle2 围绕 (5.5, 5.5) 匀速绕圈：半径 R = v / w = 1.0 / 1.0 = 1.0m
            geometry_msgs::msg::Twist cmd;
            cmd.linear.x = 1.0;
            cmd.angular.z = 1.0;
            turtle2_cmd_pub_->publish(cmd);

            // 充电满电判定
            if (effective_charge_seconds_ >= target_charge_time_)
            {
                state_ = ChargerState::CHARGING_DONE;
                current_power_ = 0.0;
                publish_power(0.0);
                stop_turtle2();

                // 广播充电完成
                std_msgs::msg::Empty done_msg;
                done_pub_->publish(done_msg);

                RCLCPP_INFO(get_logger(), "==================================================");
                RCLCPP_INFO(get_logger(), "【补能完成】累计有效充电达到 %.1f 秒，电池已充满！", target_charge_time_);
                RCLCPP_INFO(get_logger(), "通知巡检机器人恢复待命，补能海龟停转退出。");
                RCLCPP_INFO(get_logger(), "==================================================");
            }
        }
        else
        {
            // 待命、冷却中或充电完成时，确保停止小海龟
            stop_turtle2();
        }

        // 定期打印监测日志（每 500ms 一次）
        tick_counter_++;
        if (tick_counter_ % 10 == 0 && (state_ == ChargerState::CHARGING_ACTIVE || state_ == ChargerState::OVERHEAT_COOLDOWN))
        {
            const char* status_str = (state_ == ChargerState::CHARGING_ACTIVE) ? "充电中 (绕圈)" : "过温降温 (停转)";
            RCLCPP_INFO(get_logger(), "[%s] 原始温度: %5.2f°C | 滤波温度: %5.2f°C | 供电: %3.0fW | 充能进度: %4.1fs / %4.1fs",
                        status_str, last_raw_temp_, last_filtered_temp_, current_power_, effective_charge_seconds_, target_charge_time_);
        }
    }

    void publish_power(double power)
    {
        std_msgs::msg::Float64 msg;
        msg.data = power;
        power_cmd_pub_->publish(msg);
    }

    void stop_turtle2()
    {
        geometry_msgs::msg::Twist stop_cmd;
        turtle2_cmd_pub_->publish(stop_cmd);
    }

    // 内部参数与成员变量
    double power_charge_;
    double overheat_temp_;
    double cooling_resume_temp_;
    double target_charge_time_;

    ChargerState state_;
    double current_power_;
    double last_raw_temp_;
    double last_filtered_temp_;
    double effective_charge_seconds_;
    int tick_counter_;
    bool turtle2_spawned_;

    // 循环缓冲区滑动平均滤波对象
    CircularBufferFilter<15> filter_;

    // ROS 2 通信端
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr turtle2_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr power_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr done_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr temp_sub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr charge_srv_;
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client_;
    rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr set_pen_client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace sheep_patrol

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sheep_patrol::ChargerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
