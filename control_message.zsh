rostopic pub /follow_joint_trajectory/goal control_msgs/FollowJointTrajectoryActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  trajectory:
    header:
      seq: 0
      stamp:
        secs: 0
        nsecs: 0
      frame_id: ''
    joint_names:
    - 'shoulder_pan_joint'
    - 'shoulder_lift_joint'
    - 'elbow_joint'
    - 'wrist_1_joint'
    - 'wrist_2_joint'
    - 'wrist_3_joint'
    points:
    - positions: [-0.13316862192716733, -1.4320426512613473, 2.0900317792632093, -3.7271506176338907, -1.4063863112570307, -0.05375614096142535]
      velocities: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
      accelerations: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
      effort: [10, 10, 10, 10, 10, 10]
      time_from_start: {secs: 3, nsecs: 0} 