#include <stdbool.h>
#include <stdint.h>
#include "stdlib.h"
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_timer.h"
#include "inc/hw_gpio.h"

#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/systick.h"
#include "driverlib/ssi.h"
#include "driverlib/systick.h"

#include "inc/tm4c123gh6pm.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "priorities.h"
#include "drivers/buttons.h"
//#define SWITCHTASKSTACKSIZE        128         // Stack size in words

//#define TARGET_IS_TM4C123_RB1

//xSemaphoreHandle xBinarySemaphore;
xQueueHandle xQueue;

static void vSENSORTask(void *pvParameters);
static void vUARTTask(void *pvParameters);
static void vLCDTask(void *pvParameters);
typedef struct
{
    //sw_t buttonValue;
    unsigned int taskSource;

} Data_t;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}

#endif


void vApplicationStackOverflowHook(xTaskHandle *pxTask, char *pcTaskName)
{

    while(1)
    {
    }
}

//
// Configure the UART and its pins.  This must be called before UARTprintf().
//


void ConfigureUART(void)
{
    //
        // Enable the GPIO Peripheral used by the UART.
        //
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
        // Enable UART0
        SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
        //
        // Configure GPIO Pins for UART mode.
        //
        GPIOPinConfigure(GPIO_PA0_U0RX);
        GPIOPinConfigure(GPIO_PA1_U0TX);
        GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
        UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
        //
        // Use the internal 16MHz oscillator as the UART clock source.
        //
        UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
        UARTStdioConfig(0, 115200, SysCtlClockGet());
}


int main(){
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    //
    // Cau hinh timer 2 de do thoi gian phan hoi cua sieu am
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
    SysCtlDelay(3);
    TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC_UP);
    TimerEnable(TIMER2_BASE,TIMER_A);

    //Configure Trigger pin
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlDelay(3);
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_3);

    //Configure Echo pin
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlDelay(3);
    GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_2);
    // Cấu hình UART
    ConfigureUART();
    //Cấu hình LCD
    lcd_init();

    xQueue = xQueueCreate( 10, sizeof(long));

          //
        // Initialize the buttons
        //
     //   ButtonsInit();
    xTaskCreate( vLCDTask, "LCD", configMINIMAL_STACK_SIZE, NULL, 1, NULL );
    xTaskCreate( vSENSORTask, "Sensor", configMINIMAL_STACK_SIZE, NULL, 2, NULL );
    xTaskCreate( vUARTTask, "UART", configMINIMAL_STACK_SIZE, NULL, 1, NULL );

    vTaskStartScheduler();


    for(;;)
    {
    }
}

static void vSENSORTask(void *pvParameters)
{
   // uint8_t echowait ;
     long pulse;
     long timestart;
     long timeend;
     long time ;
     long dis ;
     const double temp = 1.0/16.0;

    const portTickType xTicksToWait = 100 / portTICK_RATE_MS;
//    portBASE_TYPE xStatus1;

 //   long * rcvPulse;
  //  portTickType ui32WakeTime;
   // uint32_t ui32LEDToggleDelay;
    //uint8_t i8Message;
    //     xSemaphoreTake(xBinarySemaphore, 0);

    for(;;)
    {

        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_PIN_3);
       SysCtlDelay(60); // (16.000.000hz / 3.000.000)*10 = 54, lay tron 60
       GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0);
       while(GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_2) == 0);
       timestart = TimerValueGet(TIMER2_BASE,TIMER_A);
       // Cho cho den khi nhan duoc gia tri tu Pin Echo
       while(GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_2) == GPIO_PIN_2);
       timeend = TimerValueGet(TIMER2_BASE,TIMER_A); //record value
       pulse = timeend - timestart;
       // Chuyen tu so xung dem duoc => thoi gian
       time = (long)(temp * pulse);
       dis = time * 0.0172;
       xQueueSendToBack( xQueue, &dis, 0 );
       vTaskDelay(( rand() & 0x1FF ));
     // UARTprintf("start: %2d, end: %2d, kc: %2d, time: %2d\n", timestart, timeend, dis, time);

    }
}
static void vUARTTask(void *pvParameters){

    long   dis;
   // portBASE_TYPE xStatus;
      // const portTickType xTicksToWait = 100 / portTICK_RATE_MS;
       for(;;){
            xQueueReceive( xQueue, &dis, portMAX_DELAY );

   //   Data was successfully received from the queue, print out the received     value.
           UARTprintf("Distance: %2d cm\n", dis);


       }
}

static void vLCDTask(void *pvParameters)
{
long   data;
portBASE_TYPE xStatus;
const portTickType xTicksToWait = 100 / portTICK_RATE_MS;
       for(;;){
           xStatus = xQueueReceive( xQueue, &data, xTicksToWait );

        // Thiet lap LCD

    if( xStatus == pdPASS )
     {
   //   Data was successfully received from the queue, print out the received value.
            lcd_clear();
            lcd_gotoxy(1,0);
            lcd_puts("distance:");
            lcd_gotoxy(1,1);
            lcd_put_num(data,0,0);
     }
     else
     {

     UARTprintf( "Could not receive from the queue.\r\n" );
         }
       }
}
