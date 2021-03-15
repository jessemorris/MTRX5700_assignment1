docker run -i -d --gpus all --name ocrtoc_container \
        -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v $HOME/Code/src/MTRX5700_assignment1/question_2:/root/ocrtoc_ws \
        tejaswid/sydney_siders:mtrx7500
