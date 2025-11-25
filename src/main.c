#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(sync_merged, LOG_LEVEL_INF);

/* --- Configurações --- */
#define MODE_PERIOD_SECONDS 5
#define SYNC_BYTE 0xA5U
#define DEBOUNCE_MS 50

/* UART1 (Definida no seu overlay) */
#define UART_NODE DT_NODELABEL(uart1)

/* Aliases */
#define LED_TX_NODE   DT_ALIAS(led0) // Verde
#define LED_RX_NODE   DT_ALIAS(led1) // Vermelho
#define LED_SYNC_NODE DT_ALIAS(led2) // Azul
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

/* --- Helpers --- */
void set_leds(radio_mode_t mode) {
    if (mode == MODE_TX) {
        gpio_pin_set_dt(&led_tx, 1);
        gpio_pin_set_dt(&led_rx, 0);
    } else {
        gpio_pin_set_dt(&led_tx, 0);
        gpio_pin_set_dt(&led_rx, 1);
    }
}

/* --- Timer Handler --- */
static void mode_timer_handler(struct k_timer *timer) {
    gpio_pin_set_dt(&led_sync, 0); // Apaga LED Azul
    
    // Alterna o modo
    current_mode = (current_mode == MODE_RX) ? MODE_TX : MODE_RX;
    set_leds(current_mode);
    
    LOG_INF("TIMER: Modo alterado para %s", current_mode == MODE_TX ? "TX" : "RX");
}

/* * --- THREAD DE LEITURA (SUBSTITUI A INTERRUPÇÃO) --- 
 * Esta thread roda em paralelo e usa o método de polling que funcionou no teste.
 */
void rx_thread_fn(void) {
    uint8_t c;

    while (1) {
        // Tenta ler um byte (não bloqueante ou timeout curto)
        // Se retornar 0, é porque chegou dado
        if (uart_poll_in(uart_comm, &c) == 0) {
            
            // LOG PARA PROVAR QUE O HARDWARE FUNCIONA
            LOG_INF("RX DETECTADO: 0x%02X", c);

            if (c == SYNC_BYTE) {
                LOG_INF(">>> SYNC VALIDADO! Sincronizando... <<<");
                
                // 1. Feedback Visual
                gpio_pin_set_dt(&led_sync, 1);
                
                // 2. Lógica de Sincronismo (Vira Escravo)
                current_mode = MODE_RX;
                set_leds(MODE_RX);

                // 3. Reseta Timer
                k_timer_start(&mode_timer, K_SECONDS(MODE_PERIOD_SECONDS), K_SECONDS(MODE_PERIOD_SECONDS));
                
                // Pequeno delay para o LED azul ser visto
                k_msleep(100);
                gpio_pin_set_dt(&led_sync, 0);
            }
        }
        
        // Cede processamento para não travar o sistema (Sleep curto)
        k_usleep(100); 
    }
}

/* Define a Thread RX (Pilha de 1024 bytes, Prioridade 5) */
K_THREAD_DEFINE(rx_tid, 1024, rx_thread_fn, NULL, NULL, NULL, 5, 0, 0);


/* --- Botão (TX) --- */
static void button_work_handler(struct k_work *work) {
    if (gpio_pin_get_dt(&button) <= 0) return;

    LOG_INF("BTN: Enviando Sync...");
    gpio_pin_set_dt(&led_sync, 1);

    // Envia byte
    uart_poll_out(uart_comm, SYNC_BYTE);

    // Vira Mestre (TX)
    current_mode = MODE_TX;
    set_leds(MODE_TX);

    // Reseta Timer
    k_timer_start(&mode_timer, K_SECONDS(MODE_PERIOD_SECONDS), K_SECONDS(MODE_PERIOD_SECONDS));
    
    k_msleep(100); 
    gpio_pin_set_dt(&led_sync, 0);
}

static void button_pressed_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    k_work_reschedule(&button_work, K_MSEC(DEBOUNCE_MS));
}

/* --- Main --- */
int main(void) {
    
    if (!device_is_ready(uart_comm)) {
        LOG_ERR("UART1 FALHOU");
        return 0;
    }
    
    gpio_pin_configure_dt(&led_tx, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led_rx, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led_sync, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&button, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
    
    gpio_init_callback(&button_cb, button_pressed_isr, BIT(button.pin));
    gpio_add_callback(button.port, &button_cb);
    k_work_init_delayable(&button_work, button_work_handler);

    // Configura Timer
    k_timer_init(&mode_timer, mode_timer_handler, NULL);
    k_timer_start(&mode_timer, K_SECONDS(MODE_PERIOD_SECONDS), K_SECONDS(MODE_PERIOD_SECONDS));
    
    set_leds(MODE_RX); 
    return 0;
}