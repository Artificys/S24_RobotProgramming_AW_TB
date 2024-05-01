# different movements based on scanned arUco tags:
#   raising arm
#   extending arm
#   base spinning
#   base moving forward / backward

rotate_mobile_base.positions = [6.28]
translate_mobile_base.positions = [1.0]
joint_lift.positions = [0.2, 0, 0]
wrist_extension.positions = [0.2]
joint.velocities = [0.2, 0.2, 2.5]
point0.accelerations = [1.0, 1.0, 3.5]