#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "messaging.h"

// Include rabbitmq-c headers.
#include <amqp.h>
#include <amqp_tcp_socket.h>
#include <amqp_framing.h>

// Global variables for our connection.
static amqp_connection_state_t conn;
static amqp_socket_t *socket = NULL;
static int channel = 1;

// For consuming, we store a declared queue name.
static int queueDeclared = 0;
static amqp_bytes_t queueName = {0};

int initMessaging(const char *host) {
    // Create a new connection.
    conn = amqp_new_connection();
    socket = amqp_tcp_socket_new(conn);
    if (!socket) {
        fprintf(stderr, "initMessaging: Failed to create TCP socket\n");
        return -1;
    }
    // Open a TCP socket to the host on port 5672.
    int status = amqp_socket_open(socket, host, 5672);
    if (status) {
        fprintf(stderr, "initMessaging: Failed to open TCP socket to %s\n", host);
        return -1;
    }
    // Login using default vhost "/" and guest credentials.
    amqp_rpc_reply_t reply = amqp_login(conn, "/", 0, 131072, 0,
                                          AMQP_SASL_METHOD_PLAIN, "guest", "guest");
    if (reply.reply_type != AMQP_RESPONSE_NORMAL) {
        fprintf(stderr, "initMessaging: Failed to login\n");
        return -1;
    }
    // Open a channel.
    amqp_channel_open(conn, channel);
    reply = amqp_get_rpc_reply(conn);
    if (reply.reply_type != AMQP_RESPONSE_NORMAL) {
        fprintf(stderr, "initMessaging: Failed to open channel\n");
        return -1;
    }
    // Declare an exchange named "boids_exchange" of type "fanout".
    amqp_exchange_declare(conn, channel, amqp_cstring_bytes("boids_exchange"),
                          amqp_cstring_bytes("fanout"),
                          0, 0, 0, 0, amqp_empty_table);
    reply = amqp_get_rpc_reply(conn);
    if (reply.reply_type != AMQP_RESPONSE_NORMAL) {
        fprintf(stderr, "initMessaging: Failed to declare exchange\n");
        return -1;
    }
    return 0;
}

int setupConsumerQueue(void) {
    if (!queueDeclared) {
        amqp_queue_declare_ok_t *r = amqp_queue_declare(conn, channel,
                                                        amqp_empty_bytes, // let the server choose a name
                                                        0, 1, 0, 1, amqp_empty_table);
        amqp_rpc_reply_t reply = amqp_get_rpc_reply(conn);
        if (reply.reply_type != AMQP_RESPONSE_NORMAL) {
            fprintf(stderr, "setupConsumerQueue: Failed to declare queue\n");
            return -1;
        }
        queueName = amqp_bytes_malloc_dup(r->queue);
        if (queueName.bytes == NULL) {
            fprintf(stderr, "setupConsumerQueue: Out of memory while copying queue name\n");
            return -1;
        }
        amqp_queue_bind(conn, channel, queueName,
                        amqp_cstring_bytes("boids_exchange"),
                        amqp_empty_bytes, amqp_empty_table);
        reply = amqp_get_rpc_reply(conn);
        if (reply.reply_type != AMQP_RESPONSE_NORMAL) {
            fprintf(stderr, "setupConsumerQueue: Failed to bind queue\n");
            return -1;
        }
        // *** NEW: Register a consumer on the declared queue.
        amqp_basic_consume(conn, channel, queueName,
                           amqp_empty_bytes, // consumer tag (server-assigned)
                           1,                // no_local
                           1,                // no_ack (auto-acknowledge messages)
                           0,                // not exclusive
                           amqp_empty_table);
        reply = amqp_get_rpc_reply(conn);
        if (reply.reply_type != AMQP_RESPONSE_NORMAL) {
            fprintf(stderr, "setupConsumerQueue: Failed to start consuming\n");
            return -1;
        }
        queueDeclared = 1;
    }
    return 0;
}


int publishGlobalState(const double *allStates, size_t dataSize, int rank) {
    amqp_bytes_t message_body;
    message_body.len = dataSize;
    message_body.bytes = (void *)allStates;
    
    // FIX: Swap the exchange and routing key parameters.
    // Previously, you were publishing with exchange = amqp_empty_bytes and routing key = "boids_exchange"
    // The correct call is to use "boids_exchange" as the exchange and an empty routing key.
    int ret = amqp_basic_publish(conn, channel,
                                 amqp_cstring_bytes("boids_exchange"), // Exchange name
                                 amqp_empty_bytes,                       // Routing key (unused in fanout)
                                 0, 0, NULL, message_body);
    if (ret < 0) {
        fprintf(stderr, "publishGlobalState: Failed to publish message from rank %d\n", rank);
        return -1;
    }
    return 0;
}

int consumeGlobalState(double *allStates, size_t dataSize) {
    // This function now loops until it receives a message with the expected size.
    struct timeval timeout;
    timeout.tv_sec = 10;
    timeout.tv_usec = 0;
    
    amqp_envelope_t envelope;
    memset(&envelope, 0, sizeof(envelope));
    
    while (1) {
        amqp_rpc_reply_t status = amqp_consume_message(conn, &envelope, &timeout, 0);
        if (status.reply_type != AMQP_RESPONSE_NORMAL) {
            fprintf(stderr, "consumeGlobalState: Timeout or error waiting for message (reply type %d)\n", status.reply_type);
            return -1;
        }
        if (envelope.message.body.len == dataSize) {
            memcpy(allStates, envelope.message.body.bytes, dataSize);
            amqp_destroy_envelope(&envelope);
            return 0;
        } else {
            fprintf(stderr, "consumeGlobalState: Discarding message size (%zu) as it does not match expected (%zu)\n",
                    envelope.message.body.len, dataSize);
            amqp_destroy_envelope(&envelope);
            // Loop again to wait for the correct message.
        }
    }
}

