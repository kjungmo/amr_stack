FROM ubuntu:20.04
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3 python3-pip python3-tk python3-venv \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /opt/amr
COPY pyproject.toml setup.py README.md ./
COPY amr/ amr/
COPY configs/ configs/
RUN python3 -m pip install --no-cache-dir .
ENTRYPOINT ["amr"]
CMD ["demo"]
