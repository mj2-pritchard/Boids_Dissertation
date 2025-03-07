#!/bin/bash
# wait-for-rabbitmq.sh
# Usage: ./wait-for-rabbitmq.sh <host> <port> <command> [args...]
host="$1"
port="$2"
shift 2

echo "Waiting for RabbitMQ at $host:$port..."
while ! nc -z "$host" "$port"; do
    echo "RabbitMQ not available at $host:$port yet, sleeping 1 second..."
    sleep 1
done
echo "RabbitMQ is available, executing command."
exec "$@"
