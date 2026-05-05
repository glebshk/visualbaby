gz service -s /world/world_indoor/set_pose \
  --reqtype gz.msgs.Pose \
  --reptype gz.msgs.Boolean \
  --timeout 1000 \
  --req 'name: "x500",
         position: {x: 1.5, y: 1, z: 0.2},
         orientation: {x: 0, y: 0, z: 0, w: 1}'
