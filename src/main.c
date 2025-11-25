#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/atomic.h>
#include <string.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(uart_sync, LOG_LEVEL_DBG);

/* ------------ Configurações ------------ */

/* Tempo de alternância (5 segundos) */
#define MODE_PERIOD_SECONDS 5

/* Byte "Mágico" para Sincronismo */
#define SYNC_BYTE 0xA5U

/* Debounce do botão */
#define DEBOUNCE_MS 50

/* Hardware Aliases */
#define LED_GREEN_NODE  DT_ALIAS(led0) // TX Indication
#define LED_BLUE_NODE   DT_ALIAS(led2) // RX Indication
#define BUTTON_NODE     DT_ALIAS(sw0)

static const struct gpio_dt_spec led_green = GPIO_DT_SPEC_GET(LED_GREEN_NODE, gpios);
static const struct gpio_dt_spec led_blue  = GPIO_DT_SPEC_GET(LED_BLUE_NODE, gpios);
static const struct gpio_dt_spec button    = GPIO_DT_SPEC_GET(BUTTON_NODE, gpios);

/* UART (Node Label) */
static const struct device *uart_comm = DEVICE_DT_GET(DT_NODELABEL(uart0));

/* ------------ Controle de Estado ------------ */

typedef enum {
    MODE_RX = 0,
    MODE_TX = 1
} radio_mode_t;

static radio_mode_t current_mode = MODE_RX;
static struct k_mutex mode_lock;
static struct k_sem tx_sem;     // Acorda a thread de TX
static struct k_timer mode_timer; // Timer dos 5 segundos

/* Flag atômica para evitar loops de sincronismo infinitos */
static atomic_t synced = ATOMIC_INIT(0);

/* Fila simples para saber se precisa enviar SYNC */
K_MSGQ_DEFINE(tx_msgq, sizeof(uint8_t), 4, 4);

/* Work/Callback do Botão */
static struct gpio_callback button_cb;
static struct k_work_delayable button_work;

/* ------------ Helpers Visuais ------------ */

static void set_mode_leds(radio_mode_t mode)
{
    if (mode == MODE_TX) {
        gpio_pin_set_dt(&led_green, 1); // Verde ON
        gpio_pin_set_dt(&led_blue, 0);  // Azul OFF
    } else {
        gpio_pin_set_dt(&led_green, 0); // Verde OFF
        gpio_pin_set_dt(&led_blue, 1);  // Azul ON
    }
}

static void enter_mode(radio_mode_t mode)
{
    k_mutex_lock(&mode_lock, K_FOREVER);
    current_mode = mode;
    k_mutex_unlock(&mode_lock);
    
    set_mode_leds(mode);
    LOG_INF("[MODE] Alterado para %s", mode == MODE_TX ? "TX" : "RX");
}

/* ------------ Lógica do Botão (Trigger de Sync) ------------ */

static void button_work_handler(struct k_work *work)
{
    /* Verifica estado físico do botão */
    if (gpio_pin_get_dt(&button) <= 0) return; // Soltou ou ruído

    LOG_INF("[BTN] Botão Pressionado -> Solicitando Sincronismo");

    /* Enfileira o byte de sync para envio */
    uint8_t data = SYNC_BYTE;
    
    /* Se estiver em TX, a thread TX pegará da fila. 
       Se estiver em RX, fica na fila até virar TX. */
    k_msgq_put(&tx_msgq, &data, K_NO_WAIT);
}

static void button_pressed_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    k_work_reschedule(&button_work, K_MSEC(DEBOUNCE_MS));
}

/* ------------ Timer: Cérebro da Alternância 5s/5s ------------ */

static void mode_timer_handler(struct k_timer *timer)
{
    k_mutex_lock(&mode_lock, K_FOREVER);
    radio_mode_t mode = current_mode;
    k_mutex_unlock(&mode_lock);

    if (mode == MODE_RX) {
        /* RX -> TX */
        enter_mode(MODE_TX);
        k_sem_give(&tx_sem); // Acorda thread de envio
    } else {
        /* TX -> RX */
        enter_mode(MODE_RX);
    }
}

/* ------------ Thread TX: Envia dados quando permitido ------------ */

void tx_thread_fn(void)
{
    uint8_t data_to_send;

    while (1) {
        /* Dorme até entrar no modo TX */
        k_sem_take(&tx_sem, K_FOREVER);

        /* Enquanto estiver em modo TX, verifica se há algo na fila para enviar */
        while (1) {
            k_mutex_lock(&mode_lock, K_FOREVER);
            bool is_tx = (current_mode == MODE_TX);
            k_mutex_unlock(&mode_lock);

            if (!is_tx) break; // O timer estourou e voltamos para RX

            /* Tenta pegar pedido de SYNC da fila */
            if (k_msgq_get(&tx_msgq, &data_to_send, K_NO_WAIT) == 0) {
                
                uart_poll_out(uart_comm, data_to_send);
                LOG_INF("[TX] SYNC_BYTE enviado!");
                
                /* Define flag local para não resincronizar com o próprio eco se houver */
                atomic_set(&synced, 1); 
            }

            k_sleep(K_MSEC(100)); // Polling lento da fila enquanto em TX
        }
    }
}

/* ------------ Thread RX: Escuta e Sincroniza ------------ */

void rx_thread_fn(void)
{
    unsigned char c;
    while (1) {
        /* Leitura não bloqueante (polling rápido) */
        if (uart_poll_in(uart_comm, &c) == 0) {
            
            /* Lógica de Sincronismo */
            if (c == SYNC_BYTE) {
                
                /* Se acabamos de enviar (atomic synced == 1), ignoramos nosso eco */
                if (atomic_get(&synced)) {
                    atomic_set(&synced, 0); // Limpa flag e ignora
                } else {
                    LOG_WRN("[RX] SYNC RECEBIDO! Realinhando Timer...");

                    /* 1. Força mudança imediata para TX (reage ao sync do outro) */
                    enter_mode(MODE_TX);
                    k_sem_give(&tx_sem);

                    /* 2. Reinicia o timer para contar 5s a partir de AGORA */
                    k_timer_start(&mode_timer, K_SECONDS(MODE_PERIOD_SECONDS), K_SECONDS(MODE_PERIOD_SECONDS));
                }
            } else {
                // Apenas loga caracteres lixo ou dados normais
                // printk("%c", c); 
            }
        } else {
            k_sleep(K_MSEC(1)); // Alivia CPU
        }
    }
}

/* ------------ Definição das Threads ------------ */
K_THREAD_DEFINE(tx_tid, 1024, tx_thread_fn, NULL, NULL, NULL, 5, 0, 0);
K_THREAD_DEFINE(rx_tid, 1024, rx_thread_fn, NULL, NULL, NULL, 5, 0, 0);

/* ------------ Main ------------ */

int main(void)
{
    /* Inicialização de Hardware */
    if (!device_is_ready(uart_comm) || !device_is_ready(led_green.port) || !device_is_ready(button.port)) {
        return 0;
    }

    gpio_pin_configure_dt(&led_green, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led_blue,  GPIO_OUTPUT_INACTIVE);
    
    gpio_pin_configure_dt(&button, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&button_cb, button_pressed_isr, BIT(button.pin));
    gpio_add_callback(button.port, &button_cb);

    k_work_init_delayable(&button_work, button_work_handler);
    k_mutex_init(&mode_lock);
    k_sem_init(&tx_sem, 0, 1);

    LOG_INF("Sistema Iniciado: Modo 5s/5s com Sincronismo por Botão");

    /* Inicia Timer */
    enter_mode(MODE_RX);
    k_timer_init(&mode_timer, mode_timer_handler, NULL);
    k_timer_start(&mode_timer, K_SECONDS(MODE_PERIOD_SECONDS), K_SECONDS(MODE_PERIOD_SECONDS));

    return 0;
}