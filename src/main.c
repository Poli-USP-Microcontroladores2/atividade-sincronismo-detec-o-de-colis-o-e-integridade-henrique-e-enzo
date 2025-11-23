/*
 * FSM RX/TX para FRDM-KL25Z (Zephyr)
 *
 * Pacote: HEADER(0x7E) | LENGTH(1 byte) | PAYLOAD(N bytes) | CHECKSUM(1 byte -> soma(payload) % 256)
 *
 * - rx_thread: detecta frames e valida checksum
 * - tx_thread: acorda quando modo TX é ativado (timer a cada 5s alterna modos)
 * - Botão com debounce cria mensagem SYNC (enfileira ou envia imediatamente se em TX)
 * - LEDs: Azul=RX, Verde=TX, Vermelho=Erro
 *
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/util.h>
#include <string.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(uart_rx, LOG_LEVEL_INF);

/* ------------ Configuração (ajuste se necessário) ------------ */

/* Header do pacote */
#define PKT_HEADER 0x7EU
#define MAX_PAYLOAD 64U

/* Debounce */
#define DEBOUNCE_MS 50

/* Periodo de alternância de modo (5 segundos) */
#define MODE_PERIOD_SECONDS 5

/* Capacidades das filas */
#define TX_MSGQ_MAX 8
#define RX_MSGQ_MAX 8

/* Threads */
#define RX_STACK_SIZE 1024
#define TX_STACK_SIZE 1024
#define RX_PRIORITY 6
#define TX_PRIORITY 6

/* Aliases / devicetree */
#define LED_GREEN_NODE  DT_ALIAS(led0)
#define LED_RED_NODE    DT_ALIAS(led1)
#define LED_BLUE_NODE   DT_ALIAS(led2)
#define BUTTON_NODE     DT_ALIAS(sw0)
//#define UART_ALIAS      DT_ALIAS(uart0)

static const struct gpio_dt_spec led_red   = GPIO_DT_SPEC_GET(LED_RED_NODE, gpios);
static const struct gpio_dt_spec led_green = GPIO_DT_SPEC_GET(LED_GREEN_NODE, gpios);
static const struct gpio_dt_spec led_blue  = GPIO_DT_SPEC_GET(LED_BLUE_NODE, gpios);
static const struct gpio_dt_spec button    = GPIO_DT_SPEC_GET(BUTTON_NODE, gpios);

/* ---------- UART0 (sem alias) ---------- */
static const struct device *uart_comm = DEVICE_DT_GET(DT_NODELABEL(uart0));

#if !DT_NODE_HAS_STATUS(DT_NODELABEL(uart0), okay)
#error "UART0 não está habilitada no Devicetree!"
#endif


/* ------------ Estruturas de mensagem ------------ */

struct msg {
    uint8_t len;
    uint8_t data[MAX_PAYLOAD];
};

K_MSGQ_DEFINE(tx_msgq, sizeof(struct msg), TX_MSGQ_MAX, 4);
K_MSGQ_DEFINE(rx_msgq, sizeof(struct msg), RX_MSGQ_MAX, 4);

/* ------------ Estado global e sincronização ------------ */

typedef enum {
    MODE_RX = 0,
    MODE_TX = 1
} radio_mode_t;

static radio_mode_t current_mode = MODE_RX;
static struct k_mutex mode_lock;

/* Semáforo usado para acordar tx_thread quando entramos em TX */
static struct k_sem tx_sem;

/* Timer para alternância de modo */
static struct k_timer mode_timer;

/* Callback e work para debounce do botão */
static struct gpio_callback button_cb;
static struct k_work_delayable button_work;

/* ------------ Helpers: LEDs e sinal de erro ------------ */

static void set_leds(bool r, bool g, bool b)
{
    /* FRDM-KL25Z LEDs são ativos em nível baixo */
    gpio_pin_set(led_red.port,   led_red.pin,   r ? 0 : 1);
    gpio_pin_set(led_green.port, led_green.pin, g ? 0 : 1);
    gpio_pin_set(led_blue.port,  led_blue.pin,  b ? 0 : 1);
}

static void enter_rx_mode(void)
{
    k_mutex_lock(&mode_lock, K_FOREVER);
    current_mode = MODE_RX;
    k_mutex_unlock(&mode_lock);

    set_leds(false, false, true); /* Azul */
    LOG_INF("[MODE] RX\n");
}

/* Notar: chamamos esta função também no timer ao entrar em TX */
static void enter_tx_mode(void)
{
    k_mutex_lock(&mode_lock, K_FOREVER);
    current_mode = MODE_TX;
    k_mutex_unlock(&mode_lock);

    set_leds(false, true, false); /* Verde */
    LOG_INF("[MODE] TX\n");
}

static void signal_error(void)
{
    /* Apaga todos antes de iniciar */
    set_leds(false, false, false);

    for (int i = 0; i < 3; i++) {
        set_leds(true, false, false);  // vermelho ON
        k_sleep(K_MSEC(120));
        set_leds(false, false, false); // tudo OFF
        k_sleep(K_MSEC(120));
    }

    /* Após erro, deixa somente vermelho aceso */
    set_leds(true, false, false);  
}


/* ------------ Funções de pacote (montar / enviar / checksum) ------------ */

static uint8_t calc_checksum(const uint8_t *payload, uint8_t len)
{
    uint32_t s = 0;
    for (uint8_t i = 0; i < len; ++i) {
        s += payload[i];
    }
    return (uint8_t)(s & 0xFFU);
}

/* envia bytes usando uart_poll_out (bloqueante) */
static void uart_send_bytes(const struct device *uart, const uint8_t *buf, size_t len)
{
    for (size_t i = 0; i < len; ++i) {
        uart_poll_out(uart, buf[i]);
    }
}

/* Constrói o pacote e envia pela uart */
static void send_packet_now(const struct device *uart, const uint8_t *payload, uint8_t len)
{
    if (len > MAX_PAYLOAD) {
        LOG_ERR("[TX] payload too large\n");
        signal_error();
        return;
    }

    uint8_t chk = calc_checksum(payload, len);

    /* pacote: header | length | payload | checksum */
    uart_poll_out(uart, PKT_HEADER);
    uart_poll_out(uart, len);
    uart_send_bytes(uart, payload, len);
    uart_poll_out(uart, chk);

    LOG_INF("[TX] Enviado: len=%u chk=0x%02X\n", len, chk);
}

/* ------------ Botão: debounce work handler ------------ */

static void button_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    /* leitura estável após debounce */
    int val = gpio_pin_get(button.port, button.pin);
    if (val <= 0) {
        /* liberado / ruído */
        return;
    }

    LOG_INF("[BTN] Pressionado (debounced)\n");

    /* construir mensagem SYNC */
    const char sync[] = "SYNC";
    struct msg m;
    m.len = (uint8_t)MIN(sizeof(sync) - 1, (size_t)MAX_PAYLOAD); /* sem terminador */
    memcpy(m.data, sync, m.len);

    /* se estamos em TX -> enviar imediatamente.
     * se estamos em RX -> enfileirar para enviar no próximo TX
     */
    k_mutex_lock(&mode_lock, K_FOREVER);
    radio_mode_t mode = current_mode;
    k_mutex_unlock(&mode_lock);

    if (mode == MODE_TX) {
        LOG_INF("[BTN] Estamos em TX -> envio imediato\n");
        send_packet_now(uart_comm, m.data, m.len);
    } else {
        int ret = k_msgq_put(&tx_msgq, &m, K_NO_WAIT);
        if (ret != 0) {
            LOG_ERR("[BTN] fila TX cheia, descartar\n");
            signal_error();
        } else {
            LOG_INF("[BTN] Mensagem SYNC enfileirada para próximo TX\n");
        }
    }
}

/* ISR do botão: agenda o debounce work */
static void button_pressed_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);

    /* schedule debounce work after DEBOUNCE_MS */
    k_work_reschedule(&button_work, K_MSEC(DEBOUNCE_MS));
}

/* ------------ Timer de modo: alterna entre RX <-> TX ------------ */

static void mode_timer_handler(struct k_timer *timer)
{
    ARG_UNUSED(timer);

    k_mutex_lock(&mode_lock, K_FOREVER);
    if (current_mode == MODE_RX) {
        /* mudar para TX e liberar semáforo para tx_thread */
        current_mode = MODE_TX;
        k_mutex_unlock(&mode_lock);

        enter_tx_mode();
        k_sem_give(&tx_sem);
    } else {
        /* mudar para RX */
        current_mode = MODE_RX;
        k_mutex_unlock(&mode_lock);

        enter_rx_mode();
    }
}

/* ------------ rx_thread: leitura e framing ------------ */

static void rx_thread_fn(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

    uint8_t state = 0; /* 0 = aguarda header, 1 = length, 2 = payload, 3 = checksum */
    uint8_t expected_len = 0;
    uint8_t payload_buf[MAX_PAYLOAD];
    uint8_t payload_idx = 0;

    while (1) {
        /* tentar ler um byte (não-bloqueante) */
        unsigned char cbyte;
        int r = uart_poll_in(uart_comm, &cbyte);
        if (r == 0) {
            uint8_t b = (uint8_t)cbyte;

            switch (state) {
            case 0: /* aguardando header */
                if (b == PKT_HEADER) {
                    state = 1;
                    payload_idx = 0;
                }
                break;

            case 1: /* ler length */
                expected_len = b;
                if (expected_len == 0) {
                    /* pacote vazio: ainda tem checksum a seguir */
                    state = 3; /* pular para esperar checksum */
                } else if (expected_len <= MAX_PAYLOAD) {
                    state = 2;
                } else {
                    /* length inválido -> reset */
                    LOG_ERR("[RX] length inválido: %u\n", expected_len);
                    state = 0;
                    signal_error();
                }
                break;

            case 2: /* lendo payload */
                payload_buf[payload_idx++] = b;
                if (payload_idx >= expected_len) {
                    state = 3; /* próximo byte será checksum */
                }
                break;

            case 3: /* checksum */
            {
                uint8_t received_chk = b;
                uint8_t calc = calc_checksum(payload_buf, expected_len);
                if (calc == received_chk) {
                    /* pacote válido -> entregar à fila RX */
                    struct msg m;
                    m.len = expected_len;
                    if (expected_len > 0) {
                        memcpy(m.data, payload_buf, expected_len);
                    }
                    int ret = k_msgq_put(&rx_msgq, &m, K_NO_WAIT);
                    if (ret != 0) {
                        LOG_ERR("[RX] fila RX cheia, descartar\n");
                        signal_error();
                    } else {
                        LOG_INF("[RX] pacote OK len=%u chk=0x%02X\n", expected_len, received_chk);
                    }
                } else {
                    /* checksum falhou */
                    LOG_ERR("[RX] checksum mismatch: calc=0x%02X recv=0x%02X\n", calc, received_chk);
                    signal_error();
                }
                /* reset para próximo pacote */
                state = 0;
                expected_len = 0;
                payload_idx = 0;
            }
                break;

            default:
                state = 0;
                break;
            }
        } else {
            /* sem dado: dormir curto para economizar CPU */
            k_sleep(K_MSEC(5));
        }
    }
}

/* ------------ tx_thread: aguarda modo TX e transmite filas ------------ */

static void tx_thread_fn(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

    struct msg m;

    while (1) {
        /* espera até que o timer indique entrada em TX (semaphore) */
        k_sem_take(&tx_sem, K_FOREVER);

        /* garantimos que o LED já foi ajustado no handler do timer (enter_tx_mode) */
        /* enquanto estivermos em TX (o timer determina quando voltamos a RX), enviamos tudo que estiver na fila */
        while (1) {
            /* se há mensagem na fila, envia; se fila vazia, sai */
            int ret = k_msgq_get(&tx_msgq, &m, K_NO_WAIT);
            if (ret == 0) {
                send_packet_now(uart_comm, m.data, m.len);
                /* pequena pausa entre pacotes para evitar congestionamento (ajustável) */
                k_sleep(K_MSEC(20));
            } else {
                /* nada mais a enviar */
                break;
            }
        }

        /* quando terminar de enviar tudo, aguardamos até o timer nos tirar de TX para retornar a RX.
         * Simples: checar periodicamente se current_mode mudou para RX
         */
        while (1) {
            k_mutex_lock(&mode_lock, K_FOREVER);
            radio_mode_t mode = current_mode;
            k_mutex_unlock(&mode_lock);

            if (mode == MODE_RX) {
                /* já retornamos a RX */
                break;
            }
            k_sleep(K_MSEC(50));
        }
    }
}

/* ------------ Thread Definitions ------------ */

K_THREAD_DEFINE(rx_thread_id, RX_STACK_SIZE, rx_thread_fn, NULL, NULL, NULL, RX_PRIORITY, 0, 0);
K_THREAD_DEFINE(tx_thread_id, TX_STACK_SIZE, tx_thread_fn, NULL, NULL, NULL, TX_PRIORITY, 0, 0);

/* ------------ main init ------------ */

void main(void)
{
    LOG_INF("Iniciando FRDM-KL25Z FSM (UART0) - Zephyr\n");

    /* checar devices */
    if (!device_is_ready(led_red.port) || !device_is_ready(led_green.port) || !device_is_ready(led_blue.port)) {
        LOG_ERR("Erro: LEDs não prontos no devicetree\n");
    }

    if (!device_is_ready(button.port)) {
        LOG_ERR("Erro: botão não pronto no devicetree\n");
    }

    if (!device_is_ready(uart_comm)) {
        LOG_ERR("Erro: uart_comm (UART0) não disponível\n");
        return;
    }

    /* configurar LEDs */
    gpio_pin_configure_dt(&led_red, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led_green, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led_blue, GPIO_OUTPUT_INACTIVE);

    /* configurar botão */
    gpio_pin_configure_dt(&button, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&button_cb, button_pressed_isr, BIT(button.pin));
    gpio_add_callback(button.port, &button_cb);

    /* init debounce work */
    k_work_init_delayable(&button_work, button_work_handler);

    /* init mutex/sem */
    k_mutex_init(&mode_lock);
    k_sem_init(&tx_sem, 0, 1);

    /* iniciar em RX */
    enter_rx_mode();

    /* timer para alternar modos */
    k_timer_init(&mode_timer, mode_timer_handler, NULL);
    k_timer_start(&mode_timer, K_SECONDS(MODE_PERIOD_SECONDS), K_SECONDS(MODE_PERIOD_SECONDS));

    LOG_INF("Setup concluído. Alternando modos a cada %d segundos.\n", MODE_PERIOD_SECONDS);

    /* main pode ficar monitorando mensagens recebidas (ex.: processar payloads) */
    while (1) {
        struct msg rmsg;
        int ret = k_msgq_get(&rx_msgq, &rmsg, K_SECONDS(1));
        if (ret == 0) {
            /* Exemplo: apenas imprime a mensagem recebida */
            LOG_INF("[APP] Mensagem recebida len=%u: ", rmsg.len);
            for (uint8_t i = 0; i < rmsg.len; ++i) {
                char c = (char)rmsg.data[i];
                LOG_INF("%c", (c >= 32 && c <= 126) ? c : '.'); /* printável ou '.' */
            }
            LOG_INF("\n");
        } else {
            /* sem mensagem: pode realizar outras tarefas */
            k_sleep(K_MSEC(100));
        }
    }
}
