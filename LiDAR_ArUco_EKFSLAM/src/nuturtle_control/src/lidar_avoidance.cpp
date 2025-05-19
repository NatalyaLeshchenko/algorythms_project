#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <algorithm>
#include <vector>
#include <cmath>

enum State { FORWARD, AVOID, RECOVERING };

class RobustObstacleAvoider {
public:
    RobustObstacleAvoider() {
        ros::NodeHandle pnh("~");
        // Скорости
        pnh.param("max_speed",    max_speed_,    0.08);
        pnh.param("min_speed",    min_speed_,    0.05);
        pnh.param("turn_speed",   turn_speed_,   0.35);
        pnh.param("backup_speed", backup_speed_, 0.08);
        // Безопасность
        pnh.param("safe_distance", safe_dist_,   0.38);
        pnh.param("stop_distance", stop_dist_,   0.32);
        pnh.param("sector_angle",  sector_angle_,60.0);
        pnh.param("timeout",       timeout_,     4.3);
        pnh.param("recover_time",  recover_time_,1.5);

        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
        scan_sub_= nh_.subscribe("scan", 10, &RobustObstacleAvoider::scanCb, this);

        state_          = FORWARD;
        in_obstacle_    = false;
    }

private:
    void scanCb(const sensor_msgs::LaserScan::ConstPtr& scan) {
        ros::Time now = ros::Time::now();
        geometry_msgs::Twist cmd;

        // 1) Если в режиме восстановления — едем назад фиксированное время
        if (state_ == RECOVERING) {
            double dt = (now - recover_start_).toSec();
            if (dt < recover_time_) {
                cmd.linear.x  = -backup_speed_;
                cmd.angular.z = 0.0;
                ROS_WARN_THROTTLE(1.0, "RECOVERING: backing up %.2f/%.2f s", dt, recover_time_);
                cmd_pub_.publish(cmd);
                return;
            } else {
                state_ = FORWARD;
                in_obstacle_ = false;
                ROS_INFO("Recovered — switching to FORWARD");
            }
        }

        // 2) Базовые проверки
        int total = scan->ranges.size();
        if (scan->angle_increment <= 0 || total < 3) {
            ROS_ERROR_THROTTLE(1.0, "Bad scan: angle_inc=%.3f size=%d",
                               scan->angle_increment, total);
            return;
        }

        // 3) Расчёт секторов с клиппингом
        int half   = total / 2;
        int sector = int(sector_angle_ / scan->angle_increment / 2);
        int i_fs   = std::max(0,        half - sector);
        int i_fe   = std::min(total,    half + sector);
        int i_ls   = i_fe, i_le = total;
        int i_rs   = 0,    i_re = std::max(0, half - sector);

        // 4) Фильтрация inf/NaN
        auto sanitize = [&](int a, int b) {
            std::vector<float> buf;
            buf.reserve(b - a);
            for (int i = a; i < b; i++) {
                if (i < 0 || i >= total) continue;
                float r = scan->ranges[i];
                if (std::isfinite(r)) buf.push_back(r);
            }
            if (buf.empty()) buf.push_back(static_cast<float>(safe_dist_ * 10.0));
            return buf;
        };

        auto front_v = sanitize(i_fs, i_fe);
        auto left_v  = sanitize(i_ls, i_le);
        auto right_v = sanitize(i_rs, i_re);

        float front_min = *std::min_element(front_v.begin(),  front_v.end());
        float left_min  = *std::min_element(left_v.begin(),   left_v.end());
        float right_min = *std::min_element(right_v.begin(),  right_v.end());

        // 5) Детекция препятствия
        bool obstacle = (front_min < stop_dist_) ||
                        (left_min  < safe_dist_) ||
                        (right_min < safe_dist_);

        if (obstacle) {
            if (!in_obstacle_) {
                // Первый цикл обнаружения — запоминаем время
                obstacle_start_ = now;
                in_obstacle_    = true;
            }
            double elapsed = (now - obstacle_start_).toSec();
            if (elapsed > timeout_) {
                // Переходим в режим восстановления один раз
                state_         = RECOVERING;
                recover_start_ = now;
                ROS_ERROR("OBSTACLE TIMEOUT! entering RECOVERING");
                cmd.linear.x  = -backup_speed_;
                cmd.angular.z = 0.0;
                cmd_pub_.publish(cmd);
                return;
            } else {
                // Обычный уклон
                double factor = std::min(1.0, front_min / safe_dist_) * 0.5;
                cmd.linear.x  = min_speed_ + (max_speed_ - min_speed_) * factor;
                cmd.angular.z = (right_min > left_min ? -turn_speed_ : turn_speed_);
                ROS_WARN_THROTTLE(0.5, "AVOIDING (%.2f s)", elapsed);
            }
        } else {
            // Чистая зона — едем вперёд
            state_       = FORWARD;
            in_obstacle_ = false;
            cmd.linear.x  = max_speed_;
            cmd.angular.z = 0.0;
            ROS_INFO_THROTTLE(1.0, "FORWARD");
        }

        // 6) Публикация
        cmd_pub_.publish(cmd);
    }

    ros::NodeHandle nh_;
    ros::Publisher  cmd_pub_;
    ros::Subscriber scan_sub_;

    State     state_;
    bool      in_obstacle_;
    ros::Time obstacle_start_, recover_start_;

    double max_speed_, min_speed_, turn_speed_, backup_speed_;
    double safe_dist_, stop_dist_, sector_angle_, timeout_;
    double recover_time_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "robust_obstacle_avoider");
    RobustObstacleAvoider node;
    ros::spin();
    return 0;
}
