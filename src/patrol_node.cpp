/**
 * @file patrol_node.cpp
 * @brief 基于双向链表(std::list)的海龟巡逻、回溯与智能补能自检节点
 *
 * 流程：
 * 1. 启动后先在 4 个巡检点各画一个白色小圆圈、在补能站画一个红色大圆圈做标记（抬笔瞬移，不留多余轨迹）；
 * 2. 标记完毕后停在补能站待命 (IDLE)；
 * 3. 收到 /start_patrol 命令后从补能站出发，依次沿双向链表正向巡检 4 个点；
 * 4. 巡检完成或中途收到 /trigger_backtrack 命令（模拟电量不足），立即沿双向链表原路回撤 (--it)；
 * 5. 回撤到补能站后，自动向补能系统申请 /request_charging 充电；
 * 6. 充电期间运行电池热力学物理模型（欧拉积分），叠加传感器高斯噪声，向 /battery_temperature 实时发布温度自检数据；
 * 7. 收到补能系统 /charging_done 充满通知后，恢复 IDLE 待命，支持多轮次巡检循环。
 *
 * 平台：Ubuntu 22.04 + ROS 2 Humble + turtlesim
 */

#include <cmath>
#include <list>
#include <chrono>
#include <memory>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/set_pen.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/set_bool.hpp"

/* ================================================================
 * 第一部分：纯 C 语言风格的数据结构定义
 * ================================================================ */
/* 链表节点中存的数据：一个巡逻路点 */
struct Waypoint
{
    double x;
    double y;
};

/* 状态机的五个状态（扩展支持 CHARGING 补能） */
enum State
{
    MARK,       /* 标记：先画 4 个巡检点小圆圈 + 1 个补能站大圆圈 */
    IDLE,       /* 待命：停在补能站，等待 /start_patrol 启动命令 */
    PATROL,     /* 巡逻：沿链表正向走（++it，相当于 p = p->next） */
    BACKTRACK,  /* 撤回：沿链表反向退（--it，相当于 p = p->prev），退回补能站 */
    CHARGING    /* 补能：停在补能站充电，运行热力学模型并自检上报温度 */
};

/* 海龟的实时位姿，由 /turtle1/pose 话题更新（感知结果） */
struct Pose2D
{
    double x;
    double y;
    double theta;
    int    ready;   /* 0=还没收到位姿, 1=已收到（C 语言用 int 当布尔） */
};

/* ---------------- 控制参数 ---------------- */
#define K_LIN        1.0    /* 线速度比例增益 */
#define K_ANG        4.0    /* 角速度比例增益 */
#define MAX_LIN      1.5    /* 线速度限幅 */
#define MAX_ANG      2.5    /* 角速度限幅 */
#define ARRIVE_DIST  0.1    /* 到达判定阈值：距离小于它就算到达路点 */
#define HEADING_OK   0.3    /* 朝向误差大于它时先原地转向，不着急前进 */

/* ---------------- 标记阶段的参数 ---------------- */
#define CIRCLE_R       0.3    /* 巡检点标记小圆圈的半径（白色） */
#define STATION_R      0.5    /* 补能站标记圆圈的半径（红色，更大以示区分） */
#define CIRCLE_OMEGA   2.5    /* 画圆时的角速度 rad/s（线速度=角速度×半径） */
#define DRAW_MAX_TICKS 400    /* 画圆超时保护：最多 400 拍(20s)必停 */

/* ---------------- 电池热力学物理模型参数 ---------------- */
#define T_AMBIENT        25.0   /* 环境温度 (°C) */
#define C_HEAT_CAPACITY  18.0   /* 电池热容量 C (J/°C) */
#define H_COOLING_COEFF  1.5    /* 散热系数 h (W/°C) */
#define NOISE_SIGMA      1.2    /* 温度传感器测量高斯噪声标准差 (°C) */

/* 标记阶段内部的子阶段（抬笔→瞬移→落笔→画圆，循环 4 次） */
enum MarkPhase
{
    M_PEN_OFF,    /* 抬笔：之后瞬移不会在画面上留轨迹 */
    M_TELEPORT,   /* 瞬移到画圆起点（巡检点正下方 CIRCLE_R 处，朝东） */
    M_PEN_ON,     /* 落笔 */
    M_DRAW        /* 匀速圆周运动，累计转过 2π 就画完一圈 */
};

/* 服务调用等待标记：记录当前正等着哪个服务返回 */
#define SVC_NONE  0
#define SVC_TELE  1
#define SVC_PEN   2

/* ================================================================
 * 第二部分：纯 C 语言风格的工具函数
 * ================================================================ */
static double normalize_angle(double a)
{
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

static double clamp(double v, double limit)
{
    if (v >  limit) return  limit;
    if (v < -limit) return -limit;
    return v;
}

static int compute_cmd(const struct Pose2D *pose, const struct Waypoint *goal,
                       double *linear, double *angular)
{
    double dx   = goal->x - pose->x;
    double dy   = goal->y - pose->y;
    double dist = sqrt(dx * dx + dy * dy);
    double err  = normalize_angle(atan2(dy, dx) - pose->theta);
    if (dist < ARRIVE_DIST)
        return 1;
    *angular = clamp(K_ANG * err, MAX_ANG);
    if (fabs(err) > HEADING_OK)
        *linear = 0.0;
    else
        *linear = clamp(K_LIN * dist, MAX_LIN);
    return 0;
}

/* ================================================================
 * 第三部分：节点类（PatrolNode）
 * ================================================================ */
class PatrolNode : public rclcpp::Node
{
public:
    PatrolNode() : Node("patrol_node"),
      battery_temp_(T_AMBIENT),
      charge_power_(0.0),
      rng_(std::random_device{}()),
      noise_dist_(0.0, NOISE_SIGMA)
    {
        /* ---- 1. 建表：补能站作链表头 + 4 个巡检点，push_back 到双向链表尾部 ---- */
        struct Waypoint wps[5] = {
            {5.5, 5.5},   /* 补能站：链表头，巡逻的出发点和撤回的终点 */
            {2.0, 2.0},   /* 巡检点 1 */
            {8.0, 2.0},   /* 巡检点 2 */
            {8.0, 8.0},   /* 巡检点 3 */
            {2.0, 8.0}    /* 巡检点 4 */
        };
        for (int i = 0; i < 5; i++)
            path_.push_back(wps[i]);    /* 尾插建立双向链表 */
        
        it_ = path_.begin();  /* 迭代器指向链表头（补能站） */
        state_ = MARK;        /* 先进入标记阶段：画完 4 巡检点 + 1 补能站再出发 */
        pose_.ready = 0;
        mark_idx_   = 0;
        mark_phase_ = M_PEN_OFF;
        final_leg_  = 0;
        svc_kind_   = SVC_NONE;
        draw_angle_ = 0.0;
        last_theta_ = 0.0;
        draw_ticks_ = 0;

        /* ---- 2. 发布者：向 /turtle1/cmd_vel 发速度（执行控制） ---- */
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

        /* ---- 3. 温度遥测发布者：发布带噪声的电池自检温度 ---- */
        temp_pub_ = create_publisher<std_msgs::msg::Float64>("/battery_temperature", 10);

        /* ---- 4. 服务客户端：set_pen 抬落笔、teleport_absolute 瞬移 ---- */
        pen_client_  = create_client<turtlesim::srv::SetPen>("/turtle1/set_pen");
        tele_client_ = create_client<turtlesim::srv::TeleportAbsolute>("/turtle1/teleport_absolute");

        /* ---- 5. 补能服务客户端：向补能系统申请充电 ---- */
        charge_client_ = create_client<std_srvs::srv::SetBool>("/request_charging");

        /* ---- 6. 订阅者：收 /turtle1/pose 位姿（感知） ---- */
        pose_sub_ = create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10,
            std::bind(&PatrolNode::pose_callback, this, std::placeholders::_1));

        /* ---- 7. 订阅者：收 /start_patrol 启动巡检命令 ---- */
        start_sub_ = create_subscription<std_msgs::msg::Empty>(
            "/start_patrol", 10,
            std::bind(&PatrolNode::start_callback, this, std::placeholders::_1));

        /* ---- 8. 订阅者：收 /trigger_backtrack 撤回命令 ---- */
        trigger_sub_ = create_subscription<std_msgs::msg::Empty>(
            "/trigger_backtrack", 10,
            std::bind(&PatrolNode::trigger_callback, this, std::placeholders::_1));

        /* ---- 9. 订阅者：接收补能系统发来的供电功率 P (/charger_power_cmd) ---- */
        power_cmd_sub_ = create_subscription<std_msgs::msg::Float64>(
            "/charger_power_cmd", 10,
            std::bind(&PatrolNode::power_cmd_callback, this, std::placeholders::_1));

        /* ---- 10. 订阅者：接收充电完成通知 (/charging_done) ---- */
        done_sub_ = create_subscription<std_msgs::msg::Empty>(
            "/charging_done", 10,
            std::bind(&PatrolNode::charging_done_callback, this, std::placeholders::_1));

        /* ---- 11. 定时器：每 50ms 跑一次控制循环（决策、执行与物理建模） ---- */
        timer_ = create_wall_timer(std::chrono::milliseconds(50),
            std::bind(&PatrolNode::control_loop, this));

        RCLCPP_INFO(get_logger(), "巡逻节点启动：1 个补能站 + %d 个巡检点，先进行地图标定",
                    (int)path_.size() - 1);
    }

private:
    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        pose_.x     = msg->x;
        pose_.y     = msg->y;
        pose_.theta = msg->theta;
        pose_.ready = 1;
    }

    void start_callback(const std_msgs::msg::Empty::SharedPtr msg)
    {
        (void)msg;
        if (state_ == IDLE)
        {
            it_ = path_.begin();
            state_ = PATROL;
            RCLCPP_INFO(get_logger(), "收到启动巡检命令，从补能站 (%.1f, %.1f) 出发",
                        it_->x, it_->y);
        }
    }

    void trigger_callback(const std_msgs::msg::Empty::SharedPtr msg)
    {
        (void)msg;
        if (state_ == PATROL)
        {
            RCLCPP_INFO(get_logger(), "收到回撤指令（模拟电量不足），立即原路倒退回撤补能站！");
            enter_backtrack();
        }
    }

    void power_cmd_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        charge_power_ = msg->data;
    }

    void charging_done_callback(const std_msgs::msg::Empty::SharedPtr msg)
    {
        (void)msg;
        RCLCPP_INFO(get_logger(), "==================================================");
        RCLCPP_INFO(get_logger(), "【巡检机器人】收到补能系统充电完成通知！");
        RCLCPP_INFO(get_logger(), "电池电量充满，状态恢复为待命 (IDLE)。");
        RCLCPP_INFO(get_logger(), "随时可再次下发 /start_patrol 命令开始新一轮巡检！");
        RCLCPP_INFO(get_logger(), "==================================================");
        state_ = IDLE;
        charge_power_ = 0.0;
    }

    void enter_backtrack()
    {
        if (it_ == path_.begin())
        {
            state_ = IDLE;
            RCLCPP_INFO(get_logger(), "已在补能站，转入待命");
        }
        else
        {
            --it_;   /* 等价于 C 双向链表的 p = p->prev; O(1) 操作 */
            state_ = BACKTRACK;
            RCLCPP_INFO(get_logger(), "转入撤回状态，目标改为上一节点 (%.1f, %.1f)",
                        it_->x, it_->y);
        }
    }

    void request_charging()
    {
        if (!charge_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(get_logger(), "补能服务 /request_charging 尚未就绪，将在到站后重试...");
            return;
        }

        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = true;
        charge_client_->async_send_request(
            request,
            [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
                try {
                    auto resp = future.get();
                    if (resp->success) {
                        RCLCPP_INFO(this->get_logger(), "补能站已响应：%s", resp->message.c_str());
                    }
                } catch (const std::exception &e) {
                    RCLCPP_WARN(this->get_logger(), "请求补能服务异常：%s", e.what());
                }
            });
    }

    void control_loop()
    {
        geometry_msgs::msg::Twist cmd;
        double linear = 0.0, angular = 0.0;

        /* ---- 1. 电池热力学物理模型更新（离散欧拉积分） ----
         *  C * dT/dt = P - h * (T - T_amb)
         *  离散化：T_{k+1} = T_k + (dt / C) * [P_k - h * (T_k - T_amb)]
         */
        const double dt = 0.05;
        double net_heat = charge_power_ - H_COOLING_COEFF * (battery_temp_ - T_AMBIENT);
        battery_temp_ += (dt / C_HEAT_CAPACITY) * net_heat;

        /* 传感器采集带高斯噪声脏数据并实时发布 */
        double dirty_temp = battery_temp_ + noise_dist_(rng_);
        std_msgs::msg::Float64 temp_msg;
        temp_msg.data = dirty_temp;
        temp_pub_->publish(temp_msg);

        if (!pose_.ready)
            return;

        if (state_ == IDLE || state_ == CHARGING)
        {
            cmd_pub_->publish(cmd);  /* 零速度停在补能站 */
            return;
        }

        if (state_ == MARK)
        {
            mark_step(&cmd);
            cmd_pub_->publish(cmd);
            return;
        }

        /* 感知 → 决策：对当前双向链表路点 it_ 进行闭环比例控制 */
        if (compute_cmd(&pose_, &(*it_), &linear, &angular))
        {
            on_arrive();
        }
        else
        {
            cmd.linear.x  = linear;
            cmd.angular.z = angular;
        }
        cmd_pub_->publish(cmd);
    }

    void call_set_pen(int off, int r, int g, int b)
    {
        auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
        request->r = r; request->g = g; request->b = b;
        request->width = 1;
        request->off   = off;
        pen_future_ = pen_client_->async_send_request(request).future.share();
        svc_kind_ = SVC_PEN;
    }

    void call_teleport(double x, double y, double theta)
    {
        auto request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
        request->x = x; request->y = y; request->theta = theta;
        tele_future_ = tele_client_->async_send_request(request).future.share();
        svc_kind_ = SVC_TELE;
    }

    void mark_step(geometry_msgs::msg::Twist *cmd)
    {
        std::list<Waypoint>::iterator w;
        double radius;
        int i;

        if (svc_kind_ == SVC_TELE)
        {
            if (tele_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
                return;
            svc_kind_ = SVC_NONE;
            mark_phase_++;
            return;
        }
        if (svc_kind_ == SVC_PEN)
        {
            if (pen_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
                return;
            svc_kind_ = SVC_NONE;
            mark_phase_++;
            return;
        }

        switch (mark_phase_)
        {
        case M_PEN_OFF:
            call_set_pen(1, 255, 255, 255);
            break;
        case M_TELEPORT:
            if (final_leg_)
            {
                call_teleport(path_.front().x, path_.front().y, 0.0);
            }
            else
            {
                w = path_.begin();
                for (i = 0; i < mark_idx_; i++) ++w;
                radius = (mark_idx_ == 0) ? STATION_R : CIRCLE_R;
                call_teleport(w->x, w->y - radius, 0.0);
            }
            break;
        case M_PEN_ON:
            if (mark_idx_ == 0 && !final_leg_)
                call_set_pen(0, 255, 60, 60);
            else
                call_set_pen(0, 255, 255, 255);
            break;
        case M_DRAW:
            if (final_leg_)
            {
                state_ = IDLE;
                it_ = path_.begin();
                RCLCPP_INFO(get_logger(), "标记完成，海龟已位于补能站 (%.1f, %.1f) 待命", it_->x, it_->y);
                RCLCPP_INFO(get_logger(), "启动巡检：ros2 topic pub --once /start_patrol std_msgs/msg/Empty \"{}\"");
                return;
            }
            radius = (mark_idx_ == 0) ? STATION_R : CIRCLE_R;
            cmd->linear.x  = CIRCLE_OMEGA * radius;
            cmd->angular.z = CIRCLE_OMEGA;
            if (draw_ticks_ == 0)
            {
                last_theta_ = pose_.theta;
                draw_ticks_ = 1;
                break;
            }
            draw_angle_ += fabs(normalize_angle(pose_.theta - last_theta_));
            last_theta_ = pose_.theta;
            if (draw_angle_ >= 2.0 * M_PI || ++draw_ticks_ > DRAW_MAX_TICKS)
            {
                if (mark_idx_ == 0)
                    RCLCPP_INFO(get_logger(), "补能站标记完成");
                else
                    RCLCPP_INFO(get_logger(), "巡检点 %d 标记完成", mark_idx_);
                draw_angle_ = 0.0;
                draw_ticks_ = 0;
                mark_idx_++;
                mark_phase_ = M_PEN_OFF;
                if (mark_idx_ >= (int)path_.size())
                    final_leg_ = 1;
            }
            break;
        }
    }

    void on_arrive()
    {
        if (state_ == PATROL)
        {
            if (std::next(it_) == path_.end())
            {
                RCLCPP_INFO(get_logger(), "到达最后一个巡检点 (%.1f, %.1f)，自动沿双向链表原路回撤",
                            it_->x, it_->y);
                enter_backtrack();
            }
            else
            {
                RCLCPP_INFO(get_logger(), "到达节点 (%.1f, %.1f)，继续前往下一巡检点",
                            it_->x, it_->y);
                ++it_;   /* C 双向链表 p = p->next; O(1) */
            }
        }
        else if (state_ == BACKTRACK)
        {
            if (it_ == path_.begin())
            {
                RCLCPP_INFO(get_logger(), "已沿双向链表原路回到补能站 (%.1f, %.1f)！", it_->x, it_->y);
                RCLCPP_INFO(get_logger(), "自动向补能系统申请补能服务 (/request_charging)...");
                state_ = CHARGING;
                request_charging();
            }
            else
            {
                RCLCPP_INFO(get_logger(), "退回节点 (%.1f, %.1f)，继续撤回补能站",
                            it_->x, it_->y);
                --it_;   /* C 双向链表 p = p->prev; O(1) */
            }
        }
    }

    /* 成员变量 */
    std::list<Waypoint> path_;
    std::list<Waypoint>::iterator it_;
    enum State state_;
    struct Pose2D pose_;

    int mark_idx_;
    int mark_phase_;
    int final_leg_;
    int svc_kind_;
    double draw_angle_;
    double last_theta_;
    int draw_ticks_;

    /* 电池温度热力学仿真相关 */
    double battery_temp_;
    double charge_power_;
    std::default_random_engine rng_;
    std::normal_distribution<double> noise_dist_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr temp_pub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr start_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr trigger_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr power_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr done_sub_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr pen_client_;
    rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr tele_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr charge_client_;
    rclcpp::Client<turtlesim::srv::SetPen>::SharedFuture pen_future_;
    rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedFuture tele_future_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PatrolNode>());
    rclcpp::shutdown();
    return 0;
}
