FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

RUN apt update && apt install -y \
    software-properties-common \
    wget \
    curl \
    gnupg \
    ca-certificates \
    lsb-release

# Install LLVM 17
RUN wget https://apt.llvm.org/llvm.sh && \
    chmod +x llvm.sh && \
    ./llvm.sh 17 && \
    rm llvm.sh

# Install required packages
RUN apt update && apt install -y \
    g++-13 \
    clang-17 \
    ccache \
    cmake \
    exfatprogs \
    git \
    graphviz \
    libasound2-dev \
    libbox2d-dev \
    libgl-dev \
    libgl1 \
    libglx-mesa0 \
    libegl1 \
    libxrandr2 \
    libxinerama1 \
    libxcursor1 \
    libxi6 \
    llvm-17 \
    lld-17 \
    mesa-utils \
    mold \
    net-tools \
    ninja-build \
    pigz \
    qt6-base-dev \
    qt6-svg-dev \
    rsync \
    xxd \
    x11-apps \
    sudo

RUN update-alternatives --install /usr/bin/clang clang /usr/bin/clang-17 100 && \
    update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang++-17 100


# Add dummy sound as sometimes errors may pop up because of it.
RUN echo 'pcm.!default { type null }' > /etc/asound.conf && \
    echo 'ctl.!default { type null }' >> /etc/asound.conf

WORKDIR /workspace

CMD ["/bin/bash"]