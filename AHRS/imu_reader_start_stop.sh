
case "$1" in
    --help)
        echo "Usage: imu_reader_start_stop.sh [start|stop]"
        ;;
    start)
        /usr/bin/AHRS -d
        ;;
    stop)
        killall AHRS
        ;;
    *)
        echo "Usage: imu_reader_start_stop.sh [start|stop]"
        exit 1
        ;;
esac