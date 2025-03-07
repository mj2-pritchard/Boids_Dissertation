# Use an Ubuntu base image.
FROM ubuntu:22.04

# Install required packages: build-essential, cmake, librabbitmq-dev, and netcat-openbsd.
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    librabbitmq-dev \
    netcat-openbsd

# Set working directory.
WORKDIR /app

# Copy the entire project into the container.
COPY . /app

# Clean any previous build directory and build the project.
RUN rm -rf build && mkdir -p build && cd build && cmake .. && cmake --build .

# Default command: use our wait-for-rabbitmq script to delay starting distributed_main until RabbitMQ is ready.
CMD ["/app/wait-for-rabbitmq.sh", "rabbitmq", "5672", "./build/distributed_main", "0", "1"]
