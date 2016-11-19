#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include "winsockserver/client_shared.h"

void error(const char *msg)
{
    perror(msg); // or printf ("%s: %s\n", msg, strerror(errno));
    exit(0);
}

bool socket_send_recv(const char* send_message, int send_message_len, const char* hostname)
{
	int portno = atoi(DEFAULT_PORT); 
    int sockfd, n;
    struct sockaddr_in serv_addr;
    struct hostent *server;

	const int recv_message_len = 512;
	char recv_message[recv_message_len];

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        error("ERROR opening socket");
	}
    server = gethostbyname(hostname);
    if (server == NULL) {
        fprintf(stderr,"ERROR, no such host\n");
        exit(0);
    }
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr, 
         (char *)&serv_addr.sin_addr.s_addr,
         server->h_length);
    serv_addr.sin_port = htons(portno);
    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) 
        error("ERROR connecting");

    n = write(sockfd,send_message,send_message_len);
    if (n < 0) 
         error("ERROR writing to socket");
    bzero(recv_message,recv_message_len);
    n = read(sockfd,recv_message,recv_message_len-1);
    if (n < 0) 
         error("ERROR reading from socket");

    close(sockfd);
	if (n >= (int)strlen(RETURN_SUCCESS_MESSAGE) && !strncmp(recv_message, RETURN_SUCCESS_MESSAGE,strlen(RETURN_SUCCESS_MESSAGE)))
		return true;

    return false;
}
