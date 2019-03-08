FROM ros:melodic

# Install pip
ADD https://bootstrap.pypa.io/get-pip.py /get-pip.py
RUN python /get-pip.py && rm /get-pip.py

ADD . /ros_ws/src/kinematics/

WORKDIR /ros_ws
RUN /bin/bash -c "source /ros_entrypoint.sh && catkin_make"
RUN pip install -r src/kinematics/requirements.txt

RUN chmod +x src/kinematics/src/fwd_kinematics.py
RUN chmod +x src/kinematics/src/inv_kinematics.py
