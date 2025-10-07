// Planta Atuador Tacogerador
// Desenvolvida por: Gabriel Marlon

#include <stdio.h> // Para comandos básicos como printf
#include <string.h> // Para manipulação de strings
#include "freertos/FreeRTOS.h" // Para operações do FreeRTOS como delays
#include "esp_system.h" // FUnções de inicialização do sistema
#include "esp_wifi.h" // Funções de inicialização e operações de Wi-Fi
#include "esp_log.h" // Para exibição de logs
#include "esp_event.h" // Para manipulação de eventos, incluindo eventos de Wi-Fi
#include "nvs_flash.h" // Para armazenamento não volátil (NVS)
#include "lwip/err.h" // Para tratamento de erros do stack TCP/IP leve (LwIP)
#include "lwip/sys.h" // Aplicações do sistema para LwIP
#include "mqtt_client.h" // Funções relacionadas ao cliente MQTT
#include "esp_sntp.h" // Para sincronizar o horário do dispositivo usando SNTP
#include <time.h> // Biblioteca padrão para trabalhar com tempo
#include "cJSON.h" // Biblioteca para montar JSON dinamicamente
#include "driver/ledc.h" // Biblioteca para gerar PWM
#include "hal/adc_types.h"
#include "esp_adc/adc_oneshot.h"

// Informações de wifi
int wifi_retry_count = 0; // Contador de tentativas para se conectar ao wi-fi
float u = 0.0; // Sinal de controle
int adc_voltage = 0; // Variável para armazenar o valor lido do tacogerador

#define WIFI_RETRY_NUM 5 // Número de tentativas para se conectar ao wi-fi
#define LEDC_GPIO 4 // GPIO de saída do PWM
#define LEDC_RESOLUTION 1024 // Resolução do PWM
#define LEDC_FREQ 1000 // Frequência de chaveamento do motor
#define ADC_UNIT ADC_UNIT_1 // Unidade do ADC
#define ADC_CHANNEL ADC_CHANNEL_5 // Canal do ADC

esp_mqtt_client_handle_t client = NULL;
adc_oneshot_unit_handle_t handle = NULL;
ledc_channel_config_t channel_LEDC;
ledc_timer_config_t timer;

static const char *TAG = "MQTT_MOTOR_TACOGERADOR";
static char duty_cycle[10] = {0};

static void log_error_if_nonzero(const char *message, int error_code) {
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

// Inicializa o SNTP
void initialize_sntp(void) {
    ESP_LOGI("SNTP", "Inicializando SNTP");
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL); // Define o modo de operação
    esp_sntp_setservername(0, "pool.ntp.org"); // Configura o servidor NTP
    esp_sntp_init(); // Inicializa o cliente SNTP

    time_t now = 0;
    struct tm timeinfo = { 0 };
    for (int retry = 0; retry < 10; retry++) {
        ESP_LOGI("SNTP", "Aguardando sincronização... (%d/10)", retry + 1);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        time(&now);
        localtime_r(&now, &timeinfo);
        if (timeinfo.tm_year > (2023 - 1900)) {
            ESP_LOGI("SNTP", "Horário sincronizado: %s", asctime(&timeinfo));
            return;
        }
    }
    ESP_LOGE("SNTP", "Falha ao sincronizar o horário");
}

// Define o Timezone
void set_timezone(void) {
    setenv("TZ", "<-03>3", 1); // Configura UTC-3 sem horário de verão
    tzset();
    ESP_LOGI("TIMEZONE", "Fuso horário configurado para UTC-3");
}

// Função Wifi Handler
static void wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    switch (event_id) {
        case WIFI_EVENT_STA_START:
            printf("Conectando ao Wifi... \n");
            break;
        case WIFI_EVENT_STA_CONNECTED:
            printf("Wifi conectado... \n");
            break;
        case WIFI_EVENT_STA_DISCONNECTED:
            printf("Wifi perdeu a conexão... \n");
            if (wifi_retry_count < WIFI_RETRY_NUM) {
                esp_wifi_connect();
                wifi_retry_count++;
                printf("Tentando reconectar...\n");
            }
            break;
        case IP_EVENT_STA_GOT_IP:
            printf("Wifi conseguiu um IP... \n\n");
            wifi_retry_count = 0;
            break;
        default:
            break;
    }
}

// Função para inicializar o wifi
void wifi_connection() {
    // Inicialização do sistema
    esp_netif_init();
    esp_event_loop_create_default();

    esp_netif_create_default_wifi_sta();
    wifi_init_config_t wifi_initiation = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wifi_initiation);
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);
    wifi_config_t wifi_configuration = {
        .sta = {
            .ssid = "WIFI_SSID",
            .password = "WIFI_PASSWORD",
        }
    };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_configuration);
    esp_wifi_start();
    esp_wifi_connect();
}

// Função MQTT Handler
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    client = event->client;

    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            esp_mqtt_client_subscribe(client, "planta/motor/pwm", 0);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;
        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            printf("\nTÓPICO = %.*s\r\n", event->topic_len, event->topic);
            printf("DADO = %.*s\r\n", event->data_len, event->data);

            memset(duty_cycle, 0, sizeof(duty_cycle));
            snprintf(duty_cycle, sizeof(duty_cycle), "%.*s", event->data_len, event->data);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
                log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
                log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
                ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
            }
            break;
        default:
            ESP_LOGI(TAG, "Other event id: %d", event->event_id);
            break;
    }
}

// Função para inicializar MQTT
static void mqtt_app_start(void) {
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtt://BROKER_IP:1883/mqtt",
        .credentials.client_id	= "ESP32",
    };
    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
}

// Função para configurar o PWM
static void pwm_config(void) {
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = LEDC_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };

    ledc_timer_config(&timer);

    ledc_channel_config_t channel_LEDC = {
        .gpio_num = LEDC_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };

    ledc_channel_config(&channel_LEDC);
}

// Função para configurar o ADC (Conversor Analógico-Digital)
static void adc_config(void) {
    // Configura a unidade do ADC
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    adc_oneshot_new_unit(&init_cfg, &handle);

    // Configura o canal do ADC
    adc_oneshot_chan_cfg_t ch_cfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,
    };
    adc_oneshot_config_channel(handle, ADC_CHANNEL, &ch_cfg);
}

// Converte a saída do ADC para tensão
float converte_tensao_adc(int Dout, int Vmax, int Dmax) {
    return (Dout * Vmax / Dmax);
}
void enviar_dados(int u) {
    // Obtém o horário atual
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);

    // Formata a mensagem JSON
    char json_message[128];
    snprintf(json_message, sizeof(json_message), 
        "{\"time\": %lld, \"adc_bits\": %d, \"voltage\": %f, \"control_signal\": %d}",
        now, ADC_BITWIDTH_12, converte_tensao_adc(adc_voltage, 3100, 4095), u);

    // Faz o envio dos dados por MQTT
    esp_mqtt_client_publish(client, "planta/tacogerador/voltage", json_message, 0, 1, 0);
}

void app_main(void) {
    nvs_flash_init();
    wifi_connection(); // Configura o wifi
    initialize_sntp();
    set_timezone();
    mqtt_app_start(); // Configura o MQTT
    pwm_config(); // Configura o PWM para o motor
    adc_config(); // Configura o ADC para o tacogerador

    snprintf(duty_cycle, sizeof(duty_cycle), "%d", 45);

    while (1) {
        adc_oneshot_read(handle, ADC_CHANNEL, &adc_voltage); // Leitura do ADC (Tacogerador)

        u = atof(duty_cycle);  // Converte o duty_cycle de string para float
        
        // Configura a faixa de atuação do PWM
        if (u < 45) {
            u = 0;
        } else if (u > 100) {
            u = 100;
        } else {
            u = u;
        }

        // Gera o PWM no GPIO
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, u/100 * 1024.0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

        enviar_dados(u); // Comunicação com a IHM

        vTaskDelay(50 / portTICK_PERIOD_MS); // Delay de 50 ms
    }
}