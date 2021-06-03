
#ifndef __MESSAGE_WRAPPER__
#define __MESSAGE_WRAPPER__
#include <string.h>
#include <stdio.h>
#include <assert.h>


#ifdef __cplusplus
extern "C"
{
#endif

#define MESSAGE_SIZE 2048
typedef struct Message {
	long mtype;
	char buffer[MESSAGE_SIZE];
	int err;
	int len;
} Message;

void clearMessage(Message* msg);
void setMessage(Message* msg, char* buffer, int len, int mtype);
const char* messageToString(Message* msg);


#ifdef __cplusplus
}
#endif

#endif
