while true; do
   /usr/bin/test "$(cat /sys/class/gpio/gpio389/value)" == '1' && (echo "LEAK" > /home/nvidia/test; echo "LEAK DETECTED" | /usr/bin/wall; sleep 20)
    sleep 1
done
