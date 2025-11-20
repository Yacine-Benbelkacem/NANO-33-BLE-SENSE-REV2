#define MCU_FREQ 64000000UL
#define MILISECONDS_IN_HZ (MCU_FREQ / 1000UL)

#define GPIO_PORT0_BASE 0x50000000UL
#define GPIO_PORT1_BASE 0x50000300UL

#define GPIO_PORT0_OUT (*(volatile unsigned int *)(GPIO_PORT0_BASE + 0x504))
#define GPIO_PORT0_OUTSET (*(volatile unsigned int *)(GPIO_PORT0_BASE + 0x508))
#define GPIO_PORT0_OUTCLR (*(volatile unsigned int *)(GPIO_PORT0_BASE + 0x50C))
#define GPIO_PORT0_IN (*(volatile unsigned int *)(GPIO_PORT0_BASE + 0x510))
#define GPIO_PORT0_DIR (*(volatile unsigned int *)(GPIO_PORT0_BASE + 0x514))
#define GPIO_PORT0_DIRSET (*(volatile unsigned int *)(GPIO_PORT0_BASE + 0x518))
#define GPIO_PORT0_DIRCLR (*(volatile unsigned int *)(GPIO_PORT0_BASE + 0x51C))
#define GPIO_PORT0_LATCH (*(volatile unsigned int *)(GPIO_PORT0_BASE + 0x520))
#define GPIO_PORT0_DETECTMODE (*(volatile unsigned int *)(GPIO_PORT0_BASE + 0x524))
#define GPIO_PORT0_PIN_CNF(n) (*(volatile unsigned int *)(GPIO_PORT0_BASE + 0x700 + (n)*4))

typedef enum {
    GPIO_DIR_INPUT = 0,
    GPIO_DIR_OUTPUT = 1
} gpio_dir_t;

typedef enum {
    GPIO_INPUT_BUFFER_DISCONNECTED = 0,
    GPIO_INPUT_BUFFER_CONNECTED = 1
} gpio_inbuff_t;

typedef enum {
    GPIO_PULL_DISABLED = 0,
    GPIO_PULL_DOWN = 1,
    GPIO_PULL_UP = 3
} gpio_pull_t;

typedef enum {
    GPIO_DRIVE_S0S1 = 0,
    GPIO_DRIVE_H0S1 = 1,
    GPIO_DRIVE_S0H1 = 2,
    GPIO_DRIVE_H0H1 = 3,
    GPIO_DRIVE_D0S1 = 4,
    GPIO_DRIVE_D0H1 = 5,
    GPIO_DRIVE_S0D1 = 6,
    GPIO_DRIVE_H0D1 = 7
} gpio_drive_t;

typedef enum {
    GPIO_SENSE_DISABLED = 0,
    GPIO_SENSE_HIGH = 2,
    GPIO_SENSE_LOW = 3
} gpio_sense_t;

void gpio_reset(){
    // Configure GPIO pin 13 as output
    int i=0;
    for (i = 0; i < 20; i++)
    {
        GPIO_PORT0_PIN_CNF(i) = 0x00000002; // Reset all pins to default configuration
    }
}

int gpio_read_regbit(unsigned int regval, int bit){
    return regval & (1 << bit);
}

int gpio_set_pindir(int pin){
    GPIO_PORT0_DIR |= (1 << pin);
    return gpio_read_regbit(GPIO_PORT0_DIR, pin);
}

int gpio_clear_pindir(int pin){
    GPIO_PORT0_DIRCLR = (1 << pin);
    return gpio_read_regbit(GPIO_PORT0_DIR, pin);
}

void gpio_setup_pin(int pin, gpio_dir_t dir, gpio_inbuff_t inbuff, gpio_pull_t pull, gpio_drive_t drive, gpio_sense_t sense){
    GPIO_PORT0_PIN_CNF(pin) = (dir << 0) | (inbuff << 1) | (pull << 2) | (drive << 8) | (sense << 16);  
}

int gpio_set_pinout(int pin){
    GPIO_PORT0_OUT |= (1 << pin);
    return gpio_read_regbit(GPIO_PORT0_OUT, pin);
}
int gpio_clear_pinout(int pin){
    GPIO_PORT0_OUTCLR |= (1 << pin);
    return gpio_read_regbit(GPIO_PORT0_OUT, pin);
}

void sleep(int ms){
    // Simulate sleep by busy-waiting
    volatile int count = ms * MILISECONDS_IN_HZ;
    for (volatile int i = 0; i < count; i++);
}

int main(){

    gpio_reset();
    gpio_setup_pin(13, GPIO_DIR_OUTPUT, GPIO_INPUT_BUFFER_DISCONNECTED, GPIO_PULL_DISABLED, GPIO_DRIVE_S0S1, GPIO_SENSE_DISABLED);
    
    for(;;){
        // Infinite loop
        gpio_set_pinout(13);
        sleep(100);
        gpio_clear_pinout(13);
        sleep(100);
    }

    return 0;
}