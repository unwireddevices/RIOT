#include <stdio.h>
#include "periph/gpio.h"
#include "periph/pm.h"
#include "thread.h"
#include "opt3001.h"
#include "xtimer.h"

#define ENABLE_DEBUG (1)
#include "debug.h"

#define LED_CONTROL_STACK_SIZE (1024)

static kernel_pid_t led_control_pid;
static msg_t led_control_msg;

static void* led_control(void * arg) {
	(void)arg;
	msg_t message;
	while(1) {
		msg_receive(&message);
		gpio_toggle(GPIO_PIN(PORT_B, 0));
		DEBUG("%s\n", "Irq spotted!" );
	}
	return NULL;
}

static opt3001_t opt3001;

static void interrupt_handler(void* arg) {
	(void)arg;
	msg_send(&led_control_msg, led_control_pid);
	opt3001_clear_int(&opt3001);
}


int main(void)
{
	xtimer_init();

	pm_init();
	pm_prevent_sleep = 1;
	
    puts("Led toggling program. Control the led by irq's sended from opt3001");

    printf("You are running RIOT on a(n) %s board.\n", RIOT_BOARD);
    printf("This board features a(n) %s MCU.\n", RIOT_MCU);

    gpio_init(GPIO_PIN(PORT_B, 0), GPIO_OUT);
    gpio_set(GPIO_PIN(PORT_B, 0));

    opt3001.i2c = 1;
	opt3001_init(&opt3001);
    opt3001_init_int(&opt3001, OPT3001_LATCHED, OPT3001_NO_EOC, OPT3001_IRQ_ON_LOW, OPT3001_FC_1, 10000, 100);
    gpio_init_int(GPIO_PIN(PORT_A,7), GPIO_IN_PU, GPIO_FALLING, interrupt_handler, NULL);

    char stack[LED_CONTROL_STACK_SIZE];
    led_control_pid = thread_create(stack, LED_CONTROL_STACK_SIZE, 
    	THREAD_PRIORITY_MAIN - 1, THREAD_CREATE_STACKTEST, 
    	led_control, NULL, "Toggling the led");

    return 0;
}
// 