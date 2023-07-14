FROM ros:noetic-perception

 
# Catkin tools
RUN apt-get update && apt-get install -y --no-install-recommends libopenni2-dev wget && \
    apt-get install -y --no-install-recommends \
    cmake libfreeimage-dev libfreeimageplus-dev \
    qt5-default freeglut3-dev libxi-dev libxmu-dev liblua5.2-dev \
    lua5.2 doxygen graphviz libgraphviz-dev asciidoc

# Dependencies
RUN apt update && apt install -y \
    libproj-dev \
    protobuf-compiler \
    libprotobuf-dev \
    libyaml-cpp-dev \
    autoconf \
    psmisc \
    libtool m4 automake

# Tools
RUN apt update && apt install -y nano git &&\
    apt install -y ssh-client &&\
    rm -rf /var/lib/apt/lists/*  &&\
    apt-get clean

RUN rm /bin/sh && ln -s /bin/bash /bin/sh

RUN git config --global http.sslverify false

#RUN mkdir /root/.ssh/
#COPY id_rsa /root/.ssh/id_rsa
#RUN chmod 600 /root/.ssh/id_rsa
# Create known_hosts
#RUN touch /root/.ssh/known_hosts
# Add git keys
#RUN ssh-keyscan git.mistlab.ca >> /root/.ssh/known_hosts
#RUN ssh-keyscan github.com >> /root/.ssh/known_hosts

# Install ARGoS3 
RUN git clone https://github.com/ilpincy/argos3.git /home/docker/argos3
RUN cd /home/docker/argos3 &&\
       #git checkout e3b31e21f310eac24ca0210a066f1fd9e73982c6 &&\
       mkdir build &&\
       cd build &&\
       cmake -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DARGOS_BUILD_FOR=simulator \
        -DARGOS_BUILD_NATIVE=OFF \
        -DARGOS_THREADSAFE_LOG=ON \
        -DARGOS_DYNAMIC_LOADING=ON \
        -DARGOS_USE_DOUBLE=ON \
        -DARGOS_DOCUMENTATION=OFF \
        -DARGOS_INSTALL_LDSOCONF=ON \
        ../src &&\
       make -j4 install

# Install Buzz
RUN git clone https://github.com/buzz-lang/Buzz.git /home/buzz
RUN cd /home/buzz &&\
    mkdir build &&\
    cd build &&\
    cmake -DCMAKE_BUILD_TYPE=Debug ../src &&\
    make &&\
    make install &&\
    ldconfig

# Install argos3_khepera plugin
RUN git clone https://github.com/ilpincy/argos3-kheperaiv.git /home/docker/argos3_kh4
RUN cd /home/docker/argos3_kh4 &&\
       mkdir build &&\
       cd build &&\
       cmake ../src &&\
       make &&\
       make install &&\
       ldconfig



ARG UPDATE=0
# clone the experimental repo, recurse and build
RUN mkdir -p /home/docker/Hir/KheperaIV/
RUN git clone -b main https://github.com/MISTLab/Particle_swarm.git /home/docker/Hir/KheperaIV/particle_swarm
RUN cd /home/docker/Hir/KheperaIV/particle_swarm/Simulation/Movement_experiments/loop_funcs &&\
    ls &&\
    mkdir build &&\
    cd build &&\
    cmake .. &&\
    make

RUN git config --global user.email "vivek-shankar.varadharajan@polymtl.ca" &&\
    git config --global user.name "Vivek"


ARG COPY_ENTRY=0

COPY entrypoint.sh /entrypoint.sh

COPY entrypoint.sh /ros_entrypoint.sh

RUN chmod +x /entrypoint.sh

RUN chmod +x /ros_entrypoint.sh
