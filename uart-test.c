#include <stdio>
#include <wiringPi.h>
#include <String.h>
#include <wiringPiSerial.h>
#include <errno.h>

int main ()
{
  int fd;
  int data;

  if ((fd = serialOpen ("/dev/ttyAMA0", 115200)) < 0)
  {
    fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno));
    return 1;
  }

  printf ("\nRaspberry Pi UART Test");

  while(1)
  {
    data = serialGetchar(fd);
    printf ("\nPC > RPi = %c", (char)data);
    serialPutchar(fd, data);
    serialPuts(fd, "\n");
    fflush(stdout);
  }

  return 0 ;
}
