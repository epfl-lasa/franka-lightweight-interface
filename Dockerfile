FROM ghcr.io/epfl-lasa/control-libraries/development-dependencies as source-dependencies

RUN apt-get update && apt-get install -y \
  libsodium-dev \
  libzmq3-dev \
  libpoco-dev

WORKDIR /source
RUN wget https://github.com/zeromq/cppzmq/archive/v4.7.1.tar.gz -O cppzmq-4.7.1.tar.gz && tar -xzf cppzmq-4.7.1.tar.gz
RUN cd cppzmq-4.7.1 && mkdir build && cd build && cmake .. -DCPPZMQ_BUILD_TESTS=OFF && make -j4 install
RUN rm -rf cppzmq*

RUN git clone --recursive https://github.com/frankaemika/libfranka
RUN cd libfranka && git checkout 0.8.0 && git submodule update && mkdir build
WORKDIR /source/libfranka/build
RUN cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF .. && cmake --build . && make -j && make install -j && ldconfig

WORKDIR /source
RUN git clone -b develop --single-branch https://github.com/epfl-lasa/control-libraries.git
RUN cd control-libraries/source && sudo ./install.sh --auto --no-controllers --no-dynamical-systems --no-robot-model
RUN cd control-libraries/protocol && sudo ./install.sh --auto
RUN rm -rf /source/control-libraries

# add user
RUN useradd -m remote && yes | passwd remote && usermod -s /bin/bash remote
WORKDIR /home/remote
COPY --chown=remote ./ ./franka_lightweight_interface


FROM source-dependencies as runtime

RUN cd franka_lightweight_interface && mkdir build && cd build && cmake -DCMAKE_BUILD_TYPE=Release .. && make \
  && make install && ldconfig
WORKDIR /home/remote
RUN rm -rf /home/remote/franka_lightweight_interface
USER remote

ENTRYPOINT /bin/bash


FROM source-dependencies as remote-development

RUN ( \
    echo 'LogLevel DEBUG2'; \
    echo 'PubkeyAuthentication yes'; \
    echo 'Subsystem sftp /usr/lib/openssh/sftp-server'; \
  ) > /etc/ssh/sshd_config_development \
  && mkdir /run/sshd

RUN ( \
    echo '#!/bin/bash'; \
    echo 'mkdir -p /home/remote/.ssh'; \
    echo 'echo "$1" > /home/remote/.ssh/authorized_keys'; \
    echo 'chmod -R 755 /home/remote/.ssh'; \
    echo 'chown -R remote:remote /home/remote/.ssh'; \
    echo '/usr/sbin/sshd -D -e -f /etc/ssh/sshd_config_development'; \
  ) > /.sshd_entrypoint.sh && chmod 744 /.sshd_entrypoint.sh

ENTRYPOINT ["/.sshd_entrypoint.sh"]