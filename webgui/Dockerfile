FROM ros:melodic

# Install pip
ADD https://bootstrap.pypa.io/get-pip.py /get-pip.py
RUN python /get-pip.py && rm /get-pip.py

ADD . /ros_ws/src/webgui/

WORKDIR /ros_ws
RUN /bin/bash -c "source /ros_entrypoint.sh && catkin_make"
RUN pip install Flask

RUN chmod +x src/webgui/src/app.py

EXPOSE 5000