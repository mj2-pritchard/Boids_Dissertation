#ifndef MESSAGING_H
#define MESSAGING_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Initialize RabbitMQ messaging by connecting to the specified host.
// Returns 0 on success, nonzero on error.
int initMessaging(const char *host);

// Publish a binary message containing 'dataSize' bytes from 'allStates'.
// The 'rank' parameter is used for logging.
int publishGlobalState(const double *allStates, size_t dataSize, int rank);

// Consume a binary message and copy its content into the provided buffer 'allStates'.
// The caller must ensure that 'allStates' has room for 'dataSize' bytes.
// Returns 0 on success, nonzero on error.
int consumeGlobalState(double *allStates, size_t dataSize);

// Set up (declare and bind) a consumer queue for this connection.
// This function does not wait for a message—it only ensures that the queue exists.
// Returns 0 on success, nonzero on error.
int setupConsumerQueue(void);

#ifdef __cplusplus
}
#endif

#endif
