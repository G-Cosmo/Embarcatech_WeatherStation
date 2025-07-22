#include <stdio.h>
#include "pico/stdlib.h"
#include "dht.h"
#include "hardware/i2c.h"
#include "bmp280.h"
#include "ssd1306.h"
#include "font.h"
#include "pico/cyw43_arch.h"
#include "lwip/tcp.h"
#include <math.h>
#include "pico/bootrom.h"

#define WIFI_SSID "Liz Linda"
#define WIFI_PASS "Lizm2016"

#define I2C_PORT i2c0               // i2c0 pinos 0 e 1, i2c1 pinos 2 e 3
#define I2C_SDA 0                   // 0 ou 2
#define I2C_SCL 1                   // 1 ou 3
#define SEA_LEVEL_PRESSURE 101325.0 // Pressão ao nível do mar em Pa

// Display na I2C
#define I2C_PORT_DISP i2c1
#define I2C_SDA_DISP 14
#define I2C_SCL_DISP 15
#define endereco 0x3C

#define botaoB 6
#define botaoA 5

uint64_t last_time = 0;
bool display_flag = false;

// Variáveis globais para armazenar os dados dos sensores
float temperature_dht_c = 0.0;
float humidity = 0.0;
float temperature_bmp_c = 0.0;
double altitude = 0.0;

const char HTML_BODY[] =
    "<!DOCTYPE html><html><head><meta charset='UTF-8'><title>Estação Meteorológica</title>"
    "<style>"
    "body { font-family: Arial, sans-serif; text-align: center; padding: 20px; margin: 0; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); color: white; min-height: 100vh; }"
    "h1 { color: #ffffff; text-shadow: 2px 2px 4px rgba(0,0,0,0.3); margin-bottom: 30px; }"
    ".container { max-width: 600px; margin: auto; background: rgba(255,255,255,0.1); padding: 30px; border-radius: 15px; box-shadow: 0 8px 32px rgba(0,0,0,0.3); backdrop-filter: blur(10px); border: 1px solid rgba(255,255,255,0.2); }"
    ".sensor-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 20px; margin-top: 30px; }"
    ".sensor-card { background: rgba(255,255,255,0.2); padding: 20px; border-radius: 10px; border: 1px solid rgba(255,255,255,0.3); }"
    ".sensor-title { font-size: 18px; font-weight: bold; margin-bottom: 15px; color: #ffffff; }"
    ".reading { font-size: 24px; font-weight: bold; margin: 10px 0; color: #ffffff; text-shadow: 1px 1px 2px rgba(0,0,0,0.5); }"
    ".unit { font-size: 16px; opacity: 0.8; }"
    ".refresh-info { margin-top: 20px; opacity: 0.8; font-style: italic; }"
    "</style>"
    "<script>"
    "function atualizar() {"
    "  fetch('/dados').then(res => res.json()).then(data => {"
    "    document.getElementById('temp_dht').innerText = data.temp_dht.toFixed(1);"
    "    document.getElementById('humidity').innerText = data.humidity.toFixed(0);"
    "    document.getElementById('temp_bmp').innerText = data.temp_bmp.toFixed(1);"
    "    document.getElementById('altitude').innerText = data.altitude.toFixed(1);"
    "  }).catch(err => console.error('Erro ao buscar dados:', err));"
    "}"
    "setInterval(atualizar, 2000);"
    "window.onload = atualizar;"
    "</script></head><body>"
    "<div class='container'>"
    "<h1>🌤️ Estação Meteorológica</h1>"
    "<div class='sensor-grid'>"
    "<div class='sensor-card'>"
    "<div class='sensor-title'>DHT22</div>"
    "<div class='reading'><span id='temp_dht'>--</span><span class='unit'>°C</span></div>"
    "<div class='reading'><span id='humidity'>--</span><span class='unit'>%</span></div>"
    "</div>"
    "<div class='sensor-card'>"
    "<div class='sensor-title'>BMP280</div>"
    "<div class='reading'><span id='temp_bmp'>--</span><span class='unit'>°C</span></div>"
    "<div class='reading'><span id='altitude'>--</span><span class='unit'>m</span></div>"
    "</div>"
    "</div>"
    "<div class='refresh-info'>Dados atualizados a cada 2 segundos</div>"
    "</div>"
    "</body></html>";

// change this to match your setup
static const dht_model_t DHT_MODEL = DHT22;
static const uint DATA_PIN = 28;

static float celsius_to_fahrenheit(float temperature) {
    return temperature * (9.0f / 5) + 32;
}

// Função para calcular a altitude a partir da pressão atmosférica
double calculate_altitude(double pressure)
{
    return 44330.0 * (1.0 - pow(pressure / SEA_LEVEL_PRESSURE, 0.1903));
}

// Trecho para modo BOOTSEL com botão B
void gpio_irq_handler(uint gpio, uint32_t events)
{
    uint64_t current_time = to_ms_since_boot(get_absolute_time());

    if(current_time - last_time >= 200){

     if(gpio == botaoB)
     {
        reset_usb_boot(0, 0);
     }

     display_flag = !display_flag;
     last_time = current_time;
    }
}

struct http_state
{
    char response[4096];
    size_t len;
    size_t sent;
};

static err_t http_sent(void *arg, struct tcp_pcb *tpcb, u16_t len);
static err_t http_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
static err_t connection_callback(void *arg, struct tcp_pcb *newpcb, err_t err);
static void start_http_server(void);

int main() {
    stdio_init_all();

    // Para ser utilizado o modo BOOTSEL com botão B
    gpio_init(botaoB);
    gpio_set_dir(botaoB, GPIO_IN);
    gpio_pull_up(botaoB);

    gpio_init(botaoA);
    gpio_set_dir(botaoA, GPIO_IN);
    gpio_pull_up(botaoA);

    gpio_set_irq_enabled_with_callback(botaoB, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    gpio_set_irq_enabled_with_callback(botaoA, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
   // Fim do trecho para modo BOOTSEL com botão B

   // I2C do Display funcionando em 400Khz.
    i2c_init(I2C_PORT_DISP, 400 * 1000);

    gpio_set_function(I2C_SDA_DISP, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_DISP, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_DISP);
    gpio_pull_up(I2C_SCL_DISP);

    ssd1306_t ssd;
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT_DISP);
    ssd1306_config(&ssd);
    ssd1306_fill(&ssd, false);
    ssd1306_draw_string(&ssd, "Iniciando Wi-Fi", 0, 0);
    ssd1306_draw_string(&ssd, "Aguarde...", 0, 30);    
    ssd1306_send_data(&ssd);

    if (cyw43_arch_init())
    {
        ssd1306_fill(&ssd, false);
        ssd1306_draw_string(&ssd, "WiFi => FALHA", 0, 0);
        ssd1306_send_data(&ssd);
        return 1;
    }

    cyw43_arch_enable_sta_mode();
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASS, CYW43_AUTH_WPA2_AES_PSK, 10000))
    {
        ssd1306_fill(&ssd, false);
        ssd1306_draw_string(&ssd, "WiFi => ERRO", 0, 0);
        ssd1306_send_data(&ssd);
        return 1;
    }

    uint8_t *ip = (uint8_t *)&(cyw43_state.netif[0].ip_addr.addr);
    char ip_str[24];
    snprintf(ip_str, sizeof(ip_str), "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);

    ssd1306_fill(&ssd, false);
    ssd1306_draw_string(&ssd, "WiFi => OK", 0, 0);
    ssd1306_draw_string(&ssd, ip_str, 0, 10);
    ssd1306_send_data(&ssd);

    start_http_server();

    // Inicializa o I2C para o sensor
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Inicializa o BMP280
    bmp280_init(I2C_PORT);
    struct bmp280_calib_param params;
    bmp280_get_calib_params(I2C_PORT, &params);

    puts("\nDHT test");

    dht_t dht;
    dht_init(&dht, DHT_MODEL, pio0, DATA_PIN, true /* pull_up */);

    int32_t raw_temp_bmp;
    int32_t raw_pressure;

    char str_tmp1[5];
    char str_humd[5];
    char str_tmp2[5];
    char str_alt[5];

    bool cor = true;
    do {
        dht_start_measurement(&dht);

        dht_result_t result = dht_finish_measurement_blocking(&dht, &humidity, &temperature_dht_c);
        if (result == DHT_RESULT_OK) {
            printf("%.1f C (%.1f F), %.1f%% humidity\n", temperature_dht_c, celsius_to_fahrenheit(temperature_dht_c), humidity);
            sprintf(str_tmp1, "%.1fC", temperature_dht_c);
            sprintf(str_humd, "%.0f%%", humidity);
        } else if (result == DHT_RESULT_TIMEOUT) {
            puts("DHT sensor not responding. Please check your wiring.");
        } else {
            assert(result == DHT_RESULT_BAD_CHECKSUM);
            puts("Bad checksum");
        }

        // Leitura do BMP280
        bmp280_read_raw(I2C_PORT, &raw_temp_bmp, &raw_pressure);
        int32_t temperature = bmp280_convert_temp(raw_temp_bmp, &params);
        int32_t pressure = bmp280_convert_pressure(raw_pressure, raw_temp_bmp, &params);

        // Atualiza as variáveis globais
        temperature_bmp_c = temperature / 100.0;
        altitude = calculate_altitude(pressure);
      
        sprintf(str_tmp2, "%.1fC", temperature_bmp_c);
        sprintf(str_alt, "%.1fm", altitude);  

        printf("Pressao = %.3f kPa\n", pressure / 1000.0);
        printf("Temperatura BMP: = %.2f C\n", temperature_bmp_c);
        printf("Altitude estimada: %.2f m\n", altitude);

        if(!display_flag)
        {
            ssd1306_fill(&ssd, false);
            ssd1306_draw_string(&ssd, "WiFi => OK", 0, 0);
            ssd1306_draw_string(&ssd, ip_str, 0, 10);
            ssd1306_send_data(&ssd);

        }else{
            //  Atualiza o conteúdo do display com animações
            ssd1306_fill(&ssd, !cor);
            ssd1306_rect(&ssd, 3, 3, 122, 60, cor, !cor);
            ssd1306_line(&ssd, 3, 25, 123, 25, cor);
            ssd1306_line(&ssd, 3, 37, 123, 37, cor);
            ssd1306_draw_string(&ssd, "ESTACAO", 30, 6);
            ssd1306_draw_string(&ssd, "METEOROLOGICA", 10, 16);
            ssd1306_draw_string(&ssd, "BMP280  DHT22", 10, 28);
            ssd1306_line(&ssd, 63, 25, 63, 60, cor);
            ssd1306_draw_string(&ssd, str_tmp2, 14, 41);
            ssd1306_draw_string(&ssd, str_humd, 14, 52);
            ssd1306_draw_string(&ssd, str_tmp1, 73, 41);
            ssd1306_draw_string(&ssd, str_alt, 73, 52);
            ssd1306_send_data(&ssd);
        }

        sleep_ms(1000);
    } while (true);
}

static err_t http_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
    struct http_state *hs = (struct http_state *)arg;
    hs->sent += len;
    if (hs->sent >= hs->len)
    {
        tcp_close(tpcb);
        free(hs);
    }
    return ERR_OK;
}

static err_t http_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    if (!p)
    {
        tcp_close(tpcb);
        return ERR_OK;
    }

    char *req = (char *)p->payload;
    struct http_state *hs = malloc(sizeof(struct http_state));
    if (!hs)
    {
        pbuf_free(p);
        tcp_close(tpcb);
        return ERR_MEM;
    }
    hs->sent = 0;

    // Endpoint para enviar os dados dos sensores meteorológicos
    if (strstr(req, "GET /dados"))
    {
        char json_payload[256];
        int json_len = snprintf(json_payload, sizeof(json_payload),
                                "{\"temp_dht\":%.2f,\"humidity\":%.2f,\"temp_bmp\":%.2f,\"altitude\":%.2f}",
                                temperature_dht_c, humidity, temperature_bmp_c, altitude);

        printf("[DEBUG] JSON: %s\n", json_payload);

        hs->len = snprintf(hs->response, sizeof(hs->response),
                           "HTTP/1.1 200 OK\r\n"
                           "Content-Type: application/json\r\n"
                           "Content-Length: %d\r\n"
                           "Connection: close\r\n"
                           "\r\n"
                           "%s",
                           json_len, json_payload);
    }
    // Serve a página HTML principal
    else
    {
        hs->len = snprintf(hs->response, sizeof(hs->response),
                           "HTTP/1.1 200 OK\r\n"
                           "Content-Type: text/html\r\n"
                           "Content-Length: %d\r\n"
                           "Connection: close\r\n"
                           "\r\n"
                           "%s",
                           (int)strlen(HTML_BODY), HTML_BODY);
    }

    tcp_arg(tpcb, hs);
    tcp_sent(tpcb, http_sent);

    tcp_write(tpcb, hs->response, hs->len, TCP_WRITE_FLAG_COPY);
    tcp_output(tpcb);

    pbuf_free(p);
    return ERR_OK;
}

static err_t connection_callback(void *arg, struct tcp_pcb *newpcb, err_t err)
{
    tcp_recv(newpcb, http_recv);
    return ERR_OK;
}

static void start_http_server(void)
{
    struct tcp_pcb *pcb = tcp_new();
    if (!pcb)
    {
        printf("Erro ao criar PCB TCP\n");
        return;
    }
    if (tcp_bind(pcb, IP_ADDR_ANY, 80) != ERR_OK)
    {
        printf("Erro ao ligar o servidor na porta 80\n");
        return;
    }
    pcb = tcp_listen(pcb);
    tcp_accept(pcb, connection_callback);
    printf("Servidor HTTP rodando na porta 80...\n");
}