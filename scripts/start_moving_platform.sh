echo "Landing Platform left"
gz topic -t /slider/cmd_vel -m gz.msgs.Double -p "data: -60.0"
sleep 5
while true
do
  echo "Landing Platform right"
  gz topic -t /slider/cmd_vel -m gz.msgs.Double -p "data: 60.0"
  sleep 10
  echo "Landing Platform left"
  gz topic -t /slider/cmd_vel -m gz.msgs.Double -p "data: -60.0"
  sleep 10
done

