#include "mbed.h"
#include "lpc546xx_lcd.h"

#include "image.h"

DigitalOut led1(LED1);
LPC546XX_LCD display(s_frameBufs);

// main() runs in its own thread in the OS
int main() {

   display.on();

    while (true) {
        led1 = !led1;
        printf("hello\r\n");
        wait(0.5);
    }
}
