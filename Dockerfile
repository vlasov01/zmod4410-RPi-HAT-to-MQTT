FROM balenalib/raspberrypi3-debian:latest-run

WORKDIR /home/pi/zmod/zmod2mqtt

RUN install_packages libmosquitto-dev

COPY zmod2mqtt ./

CMD ["./zmod2mqtt"]
