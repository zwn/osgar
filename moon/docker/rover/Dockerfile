FROM ros:melodic

SHELL ["/bin/bash", "-c"]

RUN /usr/bin/python3 -m pip install --user --no-cache-dir msgpack "opencv-python>3.4.2,<4" pyzmq

COPY --chown=developer:developer . ./osgar/

ENTRYPOINT ["./osgar/moon/docker/rover/entrypoint.bash"]

CMD ["./osgar/moon/docker/rover/run_solution.bash"]
