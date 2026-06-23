// Planta Motor-Tacogerador
// Desenvolvido por: Gabriel Marlon

#include <stdio.h> // Para comandos básicos como printf
#include <driver/ledc.h> // Biblioteca para gerar PWM
#include <esp_err.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h> // Para operações do FreeRTOS como delays
#include <freertos/task.h>
#include <esp_log.h> // Para exibição de logs
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include <string.h> // Para manipulação de strings
#include <math.h>
#include "esp_system.h" // FUnções de inicialização do sistema
#include "esp_wifi.h" // Funções de inicialização e operações de Wi-Fi
#include "esp_event.h" // Para manipulação de eventos, incluindo eventos de Wi-Fi
#include "nvs_flash.h" // Para armazenamento não volátil (NVS)
#include "lwip/err.h" // Para tratamento de erros do stack TCP/IP leve (LwIP)
#include "lwip/sys.h" // Aplicações do sistema para LwIP
#include "mqtt_client.h" // Funções relacionadas ao cliente MQTT
#include "hal/adc_types.h"
#include "esp_adc/adc_oneshot.h"
#include "my_data.h"

// Constantes LEDS
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO 4 // GPIO de saída do PWM
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT // Resolução do PWM
#define LEDC_FREQUENCY 1000 // Frequência de chaveamento do motor

// Constantes ADC
#define ADC_UNIT ADC_UNIT_1 // Unidade do ADC
#define ADC_CHANNEL ADC_CHANNEL_5 // Canal do ADC
#define ADC_ATTEN ADC_ATTEN_DB_12 // Atenuação do ADC

// Outras constantes
#define WIFI_RETRY_NUM 5 // Número de tentativas para se conectar ao wi-fi
#define T_AMOSTRAGEM 10 // Tempo de amostragem (em ms)
#define ZONA_MORTA 44 // Zona morta do motor (em %)

// Variáveis
int wifi_retry_count = 0; // Contador de tentativas para se conectar ao wi-fi
int buffer_count = 0;
float e[] = {0, 0}; // Erro
float uc[] = {0, 0}; // Sinal que sai do controlador
static char buffer1[2000];
static char buffer2[2000];
static char buffer_item[100];
static bool muda_buffer = false;
static int adc_voltage = 0; // Variável para armazenar o valor lido do tacogerador
static int adc_raw = 0; // Variável para armazenar o valor lido do tacogerador e enviar para dashboard
static int adc_voltage1 = 0; // Variável para armazenar o valor lido do tacogerador
static int adc_raw1 = 0; // Variável para armazenar o valor lido do tacogerador e enviar para dashboard
static int t_timer = 0; // Contador de tempo
static float u0 = 45.0;
static float ur = 0; // Sinal de duty real
static float u = 0;
static char r[10] = {0}; // Referência

esp_mqtt_client_handle_t mqtt_client = NULL;
adc_oneshot_unit_handle_t adc_handle;
adc_cali_handle_t adc_cali_handle = NULL;

static const char *TAG = "PLANTA_MOTOR_TACOGERADOR";

static void log_error_if_nonzero(const char *message, int error_code) {
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
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
            .ssid = SSID,
            .password = PASS,
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
    mqtt_client = event->client;

    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            esp_mqtt_client_subscribe(mqtt_client, "planta/tacogerador/referencia", 0);
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

            memset(r, 0, sizeof(r));
            snprintf(r, sizeof(r), "%.*s", event->data_len, event->data);
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
    esp_mqtt_client_config_t mqtt_config = {
        .broker.address.uri = BROKER_URI,
        .credentials.client_id	= "ESP32-Planta-de-Controle",
    };
    mqtt_client = esp_mqtt_client_init(&mqtt_config);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, mqtt_client);
    esp_mqtt_client_start(mqtt_client);
}

// Função para configurar o PWM
static void pwm_config(void) {
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = LEDC_OUTPUT_IO,
        .duty = 0,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

// Função para configurar o ADC (Conversor Analógico-Digital)
static void adc_config(void) {
    // Configura a unidade do ADC
    adc_oneshot_unit_init_cfg_t adc_unit = {
        .unit_id = ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    adc_oneshot_new_unit(&adc_unit, &adc_handle);

    // Configura o canal do ADC
    adc_oneshot_chan_cfg_t adc_channel = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN,
    };
    adc_oneshot_config_channel(adc_handle, ADC_CHANNEL, &adc_channel);
}

// Função para calibração do ADC.
static void adc_calibration(void) {
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT,
        .chan = ADC_CHANNEL,
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_config, &adc_cali_handle));
}

// Função para comunicação com a IHM
void enviar_dados() {
    while (1) {
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL, &adc_raw)); // Leitura do ADC (dado bruto)
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle, adc_raw, &adc_voltage)); // Calibração do dado obtido pelo ADC (em mV)
        t_timer += T_AMOSTRAGEM;
        
        // Verifica se o buffer encheu, envia os dados e intercala com o outro buffer
        if (buffer_count >= 50) {
            if (!muda_buffer) {
                esp_mqtt_client_publish(mqtt_client, "planta/tacogerador/voltage", buffer1, 0, 1, 0); // Faz o envio do buffer1 por MQTT
                memset(buffer1, 0, sizeof(buffer1));
            } else {
                esp_mqtt_client_publish(mqtt_client, "planta/tacogerador/voltage", buffer2, 0, 1, 0); // Faz o envio do buffer2 por MQTT
                memset(buffer2, 0, sizeof(buffer2));
            }
            muda_buffer = !muda_buffer;
            buffer_count = 0;
        }

        // Preenche os buffers
        if (buffer_count == 0) {
            sprintf(buffer_item, "%d,%d,%.2f,%.2f", t_timer, adc_voltage, u, e[1]);
        } else {
            sprintf(buffer_item, ",%d,%d,%.2f,%.2f", t_timer, adc_voltage, u, e[1]);
        }

        if (!muda_buffer) {
            strcat(buffer1, buffer_item);
        } else {
            strcat(buffer2, buffer_item);
        }
        buffer_count++;
        
        vTaskDelay(T_AMOSTRAGEM / portTICK_PERIOD_MS); // Delay de T ms
    }
}

void app_main(void) {
    ESP_LOGI("APP", "Iniciando a Planta de Controle: Motor-Tacogerador");
    nvs_flash_init();
    wifi_connection(); // Configura o wifi
    mqtt_app_start(); // Configura o MQTT
    pwm_config(); // Configura o PWM para o motor
    adc_config(); // Configura o ADC para o tacogerador
    adc_calibration(); // Calibração do ADC.

    snprintf(r, sizeof(r), "%d", 1550); // Inicializa a referência com 1550 mV
    
    // Repassando a tarefa de comunicação com a IHM para o outro núcleo
    xTaskCreatePinnedToCore(enviar_dados, "communication", 4096, NULL, 1, NULL, PRO_CPU_NUM);

    while (1) {
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL, &adc_raw1)); // Leitura do ADC (dado bruto)
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle, adc_raw, &adc_voltage1)); // Calibração do dado obtido pelo ADC (em mV)

        e[0] = e[1];
        uc[0] = uc[1];

        e[1] = atoi(r) - adc_voltage1; // Cálculo do erro

        uc[1] = uc[0] - 0.5833 * (1 - 2.0449) * e[1] - 0.5833 * e[0]; // Controlador Síntese Direta
        // uc[1] = uc[0] + 0.6095 * e[1] + (0.0262 - 0.6095) * e[0]; // Controlador Modelo Interno
        // uc[1] = uc[0] + 2 * e[1] - 1.322 * e[0]; // Controlador LGR (Pelo Modelo ARX)

        u = u0 + uc[1]; // Sinal de controle
        
        // Configura a faixa de atuação do PWM
        if (u < 45) {
            u = 0;
        } else if (u > 100) {
            u = 100;
        } else {
            u = u;
        }

        // Gera o PWM no GPIO
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, (float) u/100 * pow(2, LEDC_DUTY_RES));
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

        vTaskDelay(T_AMOSTRAGEM / portTICK_PERIOD_MS); // Delay de T ms
    }

    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc_handle));
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(adc_cali_handle));
}