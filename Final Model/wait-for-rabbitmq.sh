#!/bin/bash
# ---------------------
# wait-for-rabbitmq.sh
# ---------------------

# This script is used to pause execution until the RabbitMQ service is available on a specified host and port. 
# Once RabbitMQ is detected, the script proceeds to execute a given command with its arguments.

# Usage: ./wait-for-rabbitmq.sh <host> <port> <command> [args...]
#   <host>    - The hostname or IP address where RabbitMQ is expected
#   <port>    - The port number on which RabbitMQ is listening
#   <command> - The command to execute once RabbitMQ is available
#   [args...] - Any additional arguments to pass to the command

host="$1"   # Assign the first command-line argument to the variable 'host'
port="$2"   # Assign the second command-line argument to the variable 'port'
shift 2     # Remove the first two arguments so that "$@" now contains only the command and any additional arguments to be executed once RabbitMQ is available

echo "Waiting for RabbitMQ at $host:$port..." # Informing that the script is waiting for RabbitMQ to become available

while ! nc -z "$host" "$port"; do   # The 'nc -z' command checks for an open TCP connection without sending data

    # Inform the user that RabbitMQ is still not available.
    echo "RabbitMQ not available at $host:$port yet, sleeping 1 second..."
    
    # Wait for 1 second before checking again
    sleep 1 
done
echo "RabbitMQ is available, executing command."    # Once the while loop exits, RabbitMQ is available
exec "$@"                                           # Replace the current shell process with the specified command and its arguments
