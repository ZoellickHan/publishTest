source install/setup.bash

ros2 topic pub -r 100 /sentry_gimbal_msg msg_interfaces/msg/SentryGimbalMsg "{

cur_cv_mode: 0,
target_color: 0,
bullet_speed: 28.0,
small_q_w: 0.0,
small_q_x: 0.0,
small_q_y: 0.0,
small_q_z: 0.0,
big_q_w: 0.0,
big_q_x: 0.0,
big_q_y: 0.0,
big_q_z: 0.0
}"