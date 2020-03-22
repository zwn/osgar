FROM ros:melodic

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y python-pip && apt-get install -y python3-pip
RUN python -m pip install pyzmq

RUN /usr/bin/python3 -m pip install --user --no-cache-dir msgpack "opencv-python>3.4.2,<4" pyzmq

COPY --chown=developer:developer . ./osgar/

ENTRYPOINT ["./osgar/moon/docker/rover/entrypoint.bash"]

CMD ["./osgar/moon/docker/rover/run_solution.bash"]
