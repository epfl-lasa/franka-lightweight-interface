FROM ghcr.io/aica-technology/network-interfaces:v1.2 as source-dependencies

RUN apt-get update && apt-get install -y libpoco-dev

WORKDIR /source
RUN git clone --recursive https://github.com/frankaemika/libfranka
RUN cd libfranka && git checkout f1f46fb008a37eb0d1dba00c971ff7e5a7bfbfd3 && git submodule update && mkdir build
WORKDIR /source/libfranka/build
RUN cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF .. && cmake --build . && make -j && make install -j && ldconfig

WORKDIR ${HOME}
RUN rm -rf /source


FROM source-dependencies as runtime

COPY --chown=${USER} ./source ./
RUN cd franka_lightweight_interface && mkdir build && cd build && cmake -DCMAKE_BUILD_TYPE=Release .. && make \
  && make install && ldconfig
WORKDIR ${HOME}
RUN rm -rf ${HOME}/franka_lightweight_interface
USER ${USER}

ENTRYPOINT /bin/bash
