/*
 * 설 명 : 시리얼 포트를 사용하여 데이타를 전송하는 예제이다.
 * */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/signal.h>
#include <sys/ioctl.h>
#include <sys/poll.h>

#include <termios.h>

/*
 * 설명 : 시리얼포트를 연다.
 * 주의 : RTS/CTS 를 제어하지 않는다.
 * 시리얼포트를 열고 이전의 포트설정상태를 저장하지 않았다.
 * */
int open_serial( char *dev_name, int baud, int vtime, int vmin );


/*
 * 설명 : 시리얼포트를 닫는다.
 * 주의 :
 * */
 
void close_serial( int fd );

