FROM ros:humble
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip python3-pytest python3-opencv python3-numpy python3-yaml \
    ros-humble-cv-bridge ros-humble-message-filters ros-humble-tf2-ros \
    && rm -rf /var/lib/apt/lists/*
# CPU model runtime; Jetson TensorRT engines belong in a Jetson-specific image.
RUN pip3 install --no-cache-dir torch==2.5.1 torchvision==0.20.1 \
    --index-url https://download.pytorch.org/whl/cpu \
    && pip3 install --no-cache-dir transformers==4.57.6 'numpy<2' pillow==11.3.0
WORKDIR /ws
ENV ROS_DOMAIN_ID=83 ROS_LOCALHOST_ONLY=1
CMD ["bash", "scripts/perception_software_check.sh"]
