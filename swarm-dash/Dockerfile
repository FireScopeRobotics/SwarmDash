FROM ros:noetic

RUN apt-get update
RUN apt-get install -y python3-pip
RUN apt-get install ffmpeg \
    libsm6 \
    libxext6 \
    libgtkmm-3.0-1v5 \
    gstreamer-1.0 \
    gstreamer1.0-dev \
    libgstreamer1.0-0 \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \-y
WORKDIR /swarm-dash/
COPY . .
RUN pip install -r requirements.txt
RUN pip install opencv-python/*.whl
ENV PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages
ENV ROS_ROOT=/opt/ros/noetic/share/ros

CMD ["python3", "/swarm-dash/app.py"]
