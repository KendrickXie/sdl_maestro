

export X_pos=16.3
export Y_pos=22.3
export TH_pos=90.0


rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: 16.3, y: 22.3, z: 0}, orientation: {w: 1.0}}}'

