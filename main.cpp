#include "mbed.h"
#include "mpl_lcd.h"

DigitalOut led1(LED1);

// main() runs in its own thread in the OS
int main() {

   LCD_Init();

    while (true) {
        led1 = !led1;
        printf("hello\r\n");
        wait(0.5);
    }
}
