# Headless grSim library — no Qt, no network, no GUI
FROM ubuntu:24.04 AS build
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    pkg-config \
    libode-dev \
    libyaml-cpp-dev \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /grsim
COPY CMakeLists.txt LICENSE.md README.md ./
COPY config ./config
COPY libgrsim ./libgrsim

RUN cmake -S . -B build \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr/local \
      -DGRSIM_BUILD_TESTS=ON \
      -DGRSIM_BUILD_APPS=ON \
 && cmake --build build -j$(nproc) \
 && ctest --test-dir build/libgrsim --output-on-failure \
 && cmake --install build

FROM ubuntu:24.04
ENV DEBIAN_FRONTEND=noninteractive
ENV LD_LIBRARY_PATH=/usr/local/lib

RUN apt-get update && apt-get install -y --no-install-recommends \
        tini \
        libode8t64 \
        libyaml-cpp0.8 \
    && rm -rf /var/lib/apt/lists/*

COPY --from=build /usr/local /usr/local
COPY config /usr/local/share/grsim/config

RUN useradd -ms /bin/bash grsim
USER grsim
WORKDIR /home/grsim

ENTRYPOINT ["tini", "--", "grsim_run"]
CMD ["--config", "/usr/local/share/grsim/config/default.yaml", "--duration", "5", "--mode", "sync", "--behavior", "circle", "--no-log"]
