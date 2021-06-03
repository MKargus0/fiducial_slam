#pragma once
#include<arpa/inet.h>
#include<sys/socket.h>
#include<string.h> 
#include<stdio.h>
#include<stdlib.h> 	

// адрес сервера на который мы будем отправлять пакеты
#define SERVER "127.0.0.1"
// порт отправки пакетов
#define PORT 8888
#define BUFLEN 512	//максимальный размер буфера

// структура описывает адрес интернет сокета
struct sockaddr_in si_otherUDP;
int sock, slen;

void die(char *s);

void initUDPClient(int sock, int slen, sockaddr_in *si_otherUDP);

// x y z roll pitch yaw
void sendMessageUDP(double message[], sockaddr_in *si_otherUDP);