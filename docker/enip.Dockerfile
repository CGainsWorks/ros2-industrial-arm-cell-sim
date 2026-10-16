# The EtherNet/IP adapter and a scanner to test it with, in one image.
#
#   build     OpENer (the adapter stack, C) with the cell's application in
#             place of OpENer's sample application, and EIPScanner (a scanner
#             stack, C++) with the small test client in
#             src/cell_fieldbus/enip/scanner. Both from pinned commits.
#   runtime   Ubuntu 22.04 with Python, the cell sources, pycomm3 for explicit
#             messaging, and the two binaries.
#
# docker/enip-compose.yml runs two containers from this image on one Docker
# network: the cell (OpENer plus scripts/enip_cell.py) and a scanner running
# tests/test_enip.py against it. Two containers rather than one because both
# stacks bind UDP 2222 for implicit I/O, as a real PLC and a real adapter on
# separate hosts would.

FROM ubuntu:22.04 AS build

ARG OPENER_COMMIT=d8820ef41ce30dda06d213438ba81c81de33ffe2
ARG EIPSCANNER_COMMIT=12c89a57ec2e7449efb1348be52af10be9ef9701

RUN apt-get update \
    && apt-get install -y --no-install-recommends build-essential cmake git ca-certificates libcap-dev \
    && rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/EIPStackGroup/OpENer.git /src/OpENer \
    && git -C /src/OpENer checkout --quiet "${OPENER_COMMIT}" \
    && git clone https://github.com/nimbuscontrols/EIPScanner.git /src/EIPScanner \
    && git -C /src/EIPScanner checkout --quiet "${EIPSCANNER_COMMIT}"

# The adapter. The identity is a placeholder: vendor ID 65535 is not an ODVA
# assignment, and the product name says what this is.
COPY src/cell_fieldbus/enip/adapter/cell_application.c \
     /src/OpENer/source/src/ports/POSIX/sample_application/sampleapplication.c
RUN cmake -S /src/OpENer/source -B /build/opener \
        -DCMAKE_C_COMPILER=gcc -DOpENer_PLATFORM:STRING=POSIX -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_SHARED_LIBS:BOOL=OFF \
        -DOpENer_Device_Config_Vendor_Id=65535 \
        -DOpENer_Device_Config_Device_Type=43 \
        -DOpENer_Device_Config_Product_Code=1 \
        "-DOpENer_Device_Config_Device_Name=Arm cell (simulated)" \
    && cmake --build /build/opener -j"$(nproc)" \
    && install -D /build/opener/src/ports/POSIX/OpENer /opt/enip/OpENer

# The scanner library and the test client.
RUN cmake -S /src/EIPScanner -B /build/eipscanner -DCMAKE_BUILD_TYPE=Release -DENABLE_VENDOR_SRC=OFF \
    && cmake --build /build/eipscanner -j"$(nproc)" \
    && cmake --install /build/eipscanner
COPY src/cell_fieldbus/enip/scanner /src/io_scanner
RUN cmake -S /src/io_scanner -B /build/io_scanner -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /build/io_scanner -j"$(nproc)" \
    && install -D /build/io_scanner/io_scanner /opt/enip/io_scanner \
    && cp -a /usr/local/lib/libEIPScanner.so* /opt/enip/


FROM ubuntu:22.04 AS runtime

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        python3 python3-pip python3-numpy python3-scipy python3-yaml iproute2 libcap2 \
    && rm -rf /var/lib/apt/lists/*
RUN pip3 install --no-cache-dir "pycomm3==1.2.16" "pytest==8.3.5"

COPY --from=build /opt/enip /opt/enip
ENV LD_LIBRARY_PATH=/opt/enip
ENV ENIP_OPENER=/opt/enip/OpENer
ENV ENIP_IO_SCANNER=/opt/enip/io_scanner

WORKDIR /workspace
COPY conftest.py ./
COPY simulation ./simulation
COPY src ./src
COPY scripts ./scripts
COPY tests ./tests
