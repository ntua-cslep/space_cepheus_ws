// ee_gripper.cpp
//
// Stateless inputs, local internal state.
// Call update(...) every control loop iteration.
//
// Condition (must hold continuously for 1s):
//   secs > tf && |ee_x - xt| < 0.05 && |ee_y - yt| < 0.05
//
// Action:
//   publish "softgrip"
//   non-blocking delay (1s)
//   publish "hardgrip"

#include <cmath>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>

class EEGripper {
  public:
    explicit EEGripper(ros::NodeHandle &nh) {
        grip_pub_ = nh.advertise<std_msgs::String>("/lefo_hear", 1);

        hardgrip_timer_ = nh.createTimer(ros::Duration(HARD_DELAY_SEC),
                                         &EEGripper::hardGripCallback, this,
                                         true, // oneshot
                                         false // autostart
        );
    }

    // =====================================================
    // Call this every control loop
    // =====================================================
    void update(double secs, double tf, double ee_x, double ee_y, double xt,
                double yt) {
        if (grip_triggered_)
            return;

        const bool condition = (secs > tf) && (std::abs(ee_x - xt) < POS_TOL) &&
                               (std::abs(ee_y - yt) < POS_TOL);

        if (condition) {
            if (!dwell_active_) {
                dwell_start_time_ = ros::Time::now();
                dwell_active_ = true;
            } else {
                const double dwell_time =
                    (ros::Time::now() - dwell_start_time_).toSec();

                if (dwell_time >= DWELL_SEC) {
                    triggerGripSequence();
                    grip_triggered_ = true;
                    dwell_active_ = false;
                }
            }
        } else {
            // Condition broke → reset dwell
            dwell_active_ = false;
        }
    }

    // Reuse in repeated experiments
    void reset() {
        // Cancel delayed hardgrip if armed
        hardgrip_timer_.stop();

        // Reset internal state
        grip_triggered_ = false;
        dwell_active_ = false;

        // Publish release command
        publish("release");

        ROS_INFO("[EE_GRIPPER] Reset → release");
    }

  private:
    // ===== Parameters (local only) =====
    static constexpr double POS_TOL = 0.05;
    static constexpr double DWELL_SEC = 1.0;
    static constexpr double HARD_DELAY_SEC = 1.0;

    // ===== ROS =====
    ros::Publisher grip_pub_;
    ros::Timer hardgrip_timer_;

    // ===== Internal state =====
    bool grip_triggered_ = false;
    bool dwell_active_ = false;
    ros::Time dwell_start_time_;

    // ===== Actions =====
    void triggerGripSequence() {
        publish("softgrip");

        hardgrip_timer_.stop();
        hardgrip_timer_.start();

        ROS_INFO("[EE_GRIPPER] Grip sequence triggered");
    }

    void hardGripCallback(const ros::TimerEvent &) { publish("hardgrip"); }

    void publish(const std::string &cmd) {
        std_msgs::String msg;
        msg.data = cmd;
        grip_pub_.publish(msg);
    }
};