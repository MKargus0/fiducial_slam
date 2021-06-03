#include <sendPoseDataUDP.h>


void die(char *s)
{
	// пишем ошибку
	perror(s);
	// неудачное завершение 
	exit(1);
}

void initUDPClient(int sock, int slen, sockaddr_in *si_otherUDP)
{	
	sock, slen=sizeof(si_otherUDP);
	char buf[BUFLEN];
	char message[BUFLEN];

	// создаем UDP сокет
	if ( (sock=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
	{
		// функция выводит ошибку и завершает процесс
		die("socket");
	}

	// 1 указатель на массив 2 - то чем заполняем  3 - количество заполняемых элементов
	memset((char *) &si_otherUDP, 0, sizeof(si_otherUDP));
	// выставляем семейство сокетов 
	*si_otherUDP.sin_family = AF_INET;
	//ставим номер порта
	*si_otherUDP.sin_port = htons(PORT);

	// Convert Internet host address from numbers-and-dots notation in CP
	// into binary data and store the result in the structure INP.
	if (inet_aton(SERVER , *si_otherUDP.sin_addr) == 0) 
	{
		fprintf(stderr, "inet_aton() failed\n");
		exit(1);
	}

}

// x y z roll pitch yaw
void sendMessageUDP(double message[], sockaddr_in *si_otherUDP)
{
	if (sendto(sock, message, sizeof(message) , 0 , (struct sockaddr *) &si_otherUDP, slen)==-1)
	{
			die("sendto()");
	}
}
