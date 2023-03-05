FROM ros:noetic

RUN apt-get update
RUN apt-get install -y python3-pip
RUN apt-get install ffmpeg libsm6 libxext6  -y
WORKDIR /swarm-dash/
COPY requirements.txt ./requirements.txt
RUN pip install -r requirements.txt
RUN pip install --upgrade numpy

COPY . .
ENV PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages
ENV ROS_ROOT=/opt/ros/noetic/share/ros

CMD ["python3", "/swarm-dash/app.py"]]
