#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/console/console.h> /* Necessário para console_getline */
#include <string.h>

LOG_MODULE_REGISTER(integrity_app, LOG_LEVEL_INF);

#define MODE_PERIOD_SECONDS 5
#define DEBOUNCE_MS 50
#define PKT_HEADER  0x7E
#define MAX_PAYLOAD 32

/* UART1 (PTE0/PTE1) - Comunicação entre placas */
#define UART_NODE DT_NODELABEL(uart1)

/* Aliases */
#define LED_TX_NODE   DT_ALIAS(led0) 
#define LED_RX_NODE   DT_ALIAS(led1) 
#define LED_SYNC_NODE DT_ALIAS(led2) 
#define BUTTON_NODE   DT_ALIAS(sw0)

static const struct gpio_dt_spec led_tx   = GPIO_DT_SPEC_GET(LED_TX_NODE, gpios);
static const struct gpio_dt_spec led_rx   = GPIO_DT_SPEC_GET(LED_RX_NODE, gpios);
static const struct gpio_dt_spec led_sync = GPIO_DT_SPEC_GET(LED_SYNC_NODE, gpios);
static const struct gpio_dt_spec button   = GPIO_DT_SPEC_GET(BUTTON_NODE, gpios);

static const struct device *uart_comm = DEVICE_DT_GET(UART_NODE);

/* Estado Global */
typedef enum { MODE_RX = 0, MODE_TX = 1 } radio_mode_t;
static volatile radio_mode_t current_mode = MODE_RX;

static struct k_timer mode_timer;
static struct k_work_delayable button_work;
static struct gpio_callback button_cb;

void set_leds(radio_mode_t mode) {
    if (mode == MODE_TX) {
        gpio_pin_set_dt(&led_tx, 1);
        gpio_pin_set_dt(&led_rx, 0);
    } else {
        gpio_pin_set_dt(&led_tx, 0);
        gpio_pin_set_dt(&led_rx, 1);
    }
}

static void mode_timer_handler(struct k_timer *timer) {
    gpio_pin_set_dt(&led_sync, 0);
    current_mode = (current_mode == MODE_RX) ? MODE_TX : MODE_RX;
    set_leds(current_mode);
    LOG_INF("TIMER: Alternando para %s", current_mode == MODE_TX ? "TX" : "RX");
}

/* --- TX RESTAURADO --- */
/* Voltamos a usar k_sleep(1ms) para garantir que a thread RX possa rodar 
   nos intervalos e não travamos a CPU com busy_wait */
void send_packet(const char *msg) {
    uint8_t len = strlen(msg);
    if (len > MAX_PAYLOAD) len = MAX_PAYLOAD;

    uint8_t checksum = 0;
    for(int i=0; i<len; i++) checksum += msg[i];

    uart_poll_out(uart_comm, PKT_HEADER);
    k_sleep(K_MSEC(1)); // Delay importante para sincronia e yield
    
    uart_poll_out(uart_comm, len);
    k_sleep(K_MSEC(1));

    for(int i=0; i<len; i++) {
        uart_poll_out(uart_comm, msg[i]);
        k_sleep(K_MSEC(1));
    }
    
    uart_poll_out(uart_comm, checksum);
    
    LOG_INF("TX: Enviado '%s'", msg);
}

/* --- RX RESTAURADO --- */
/* Voltamos a usar k_usleep(50) para polling rápido. 10ms era lento demais. */
void rx_thread_fn(void) {
    uint8_t c;
    enum { WAIT_HEADER, WAIT_LEN, WAIT_PAYLOAD, WAIT_CHECK } state = WAIT_HEADER;
    
    uint8_t rx_len = 0;
    uint8_t rx_idx = 0;
    uint8_t rx_buf[MAX_PAYLOAD + 1]; 
    uint8_t rx_calc_checksum = 0;

    LOG_INF("RX THREAD: Monitorando UART1...");

    while (1) {
        // Loop de leitura rápida
        while (uart_poll_in(uart_comm, &c) == 0) {
            
            if (c == PKT_HEADER) {
                state = WAIT_LEN;
                rx_calc_checksum = 0;
                continue; 
            }

            switch (state) {
                case WAIT_LEN:
                    rx_len = c;
                    if (rx_len > MAX_PAYLOAD) state = WAIT_HEADER;
                    else { rx_idx = 0; state = WAIT_PAYLOAD; }
                    break;

                case WAIT_PAYLOAD:
                    rx_buf[rx_idx] = c;
                    rx_calc_checksum += c;
                    rx_idx++;
                    if (rx_idx >= rx_len) state = WAIT_CHECK;
                    break;

                case WAIT_CHECK: {
                    if (rx_calc_checksum == c) {
                        rx_buf[rx_len] = '\0';
                        LOG_INF("RX: Recebido '%s'", rx_buf);
                        
                        // Lógica de Sincronismo
                        if (strcmp(rx_buf, "SYNC_CMD") == 0) {
                            LOG_INF(">>> COMANDO SYNC RECEBIDO <<<");
                            gpio_pin_set_dt(&led_sync, 1);
                            
                            current_mode = MODE_RX;
                            set_leds(MODE_RX);
                            k_timer_start(&mode_timer, K_SECONDS(MODE_PERIOD_SECONDS), K_SECONDS(MODE_PERIOD_SECONDS));
                            
                            // k_sleep aqui é seguro pois estamos na thread RX
                            k_msleep(200); 
                            gpio_pin_set_dt(&led_sync, 0);
                        }
                    } else {
                         // Checksum falhou
                         gpio_pin_toggle_dt(&led_rx);
                    }
                    state = WAIT_HEADER;
                    break;
                }
            }
        }
        // RESTAURADO: 50 micro-segundos. Rápido o suficiente para não perder bytes.
        k_usleep(50); 
    }
}

static void button_work_handler(struct k_work *work) {
    if (gpio_pin_get_dt(&button) <= 0) return;
    
    send_packet("SYNC_CMD");

    current_mode = MODE_TX;
    set_leds(MODE_TX);
    k_timer_start(&mode_timer, K_SECONDS(MODE_PERIOD_SECONDS), K_SECONDS(MODE_PERIOD_SECONDS));
}

static void button_pressed_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    k_work_reschedule(&button_work, K_MSEC(DEBOUNCE_MS));
}

/* --- CONSOLE THREAD --- */
/* Mantido o console_getline, pois resolve o problema original de montar a mensagem */
static void console_thread_fn(void) {
    LOG_INF("Console Thread iniciada. Digite e pressione Enter.");

    while (1) {
        char *line = console_getline();

        if (line) {
            // Remove quebra de linha final se houver
            int len = strlen(line);
            if (len > 0 && (line[len-1] == '\n' || line[len-1] == '\r')) {
                line[len-1] = '\0';
            }

            if (strlen(line) > 0) {
                LOG_INF("Console enviando: '%s'", line);
                send_packet(line);
            }
        }
    }
}

K_THREAD_DEFINE(console_tid, 1024, console_thread_fn, NULL, NULL, NULL, 3, 0, 0);
K_THREAD_DEFINE(rx_tid, 1024, rx_thread_fn, NULL, NULL, NULL, 5, 0, 0);

int main(void) {
    if (!device_is_ready(uart_comm)) {
        LOG_ERR("UART de comunicacao indisponivel");
        return 0;
    }

    /* INICIALIZACAO OBRIGATÓRIA PARA O CONSOLE */
    console_getline_init();

    gpio_pin_configure_dt(&led_tx, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led_rx, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led_sync, GPIO_OUTPUT_INACTIVE);
    
    gpio_pin_configure_dt(&button, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
    
    gpio_init_callback(&button_cb, button_pressed_isr, BIT(button.pin));
    gpio_add_callback(button.port, &button_cb);
    k_work_init_delayable(&button_work, button_work_handler);

    k_timer_init(&mode_timer, mode_timer_handler, NULL);
    k_timer_start(&mode_timer, K_SECONDS(MODE_PERIOD_SECONDS), K_SECONDS(MODE_PERIOD_SECONDS));

    set_leds(MODE_RX); 
    return 0;
}