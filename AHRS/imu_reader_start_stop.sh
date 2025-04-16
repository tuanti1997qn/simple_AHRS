
case "$1" in
    --help)
        echo "Usage: imu_reader_start_stop.sh [start|stop]"
        ;;
    start)
        /lib/modules/6.1.46/extra/imu_reader_load
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