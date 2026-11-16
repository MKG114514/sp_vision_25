sleep 5
cd ~/Desktop/sp_vision_25/
chmod +x /home/hero/Desktop/sp_vision_25z5z/sp_vision_25z/watchdog.sh
screen \
    -L \
    -Logfile logs/$(date "+%Y-%m-%d_%H-%M-%S").screenlog \
    -d \
    -m \
    bash -c "./watchdog.sh"
