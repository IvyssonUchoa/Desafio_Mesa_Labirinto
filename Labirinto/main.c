/* ===================== Inclusão de bibliotecas ===================== */
// Drivers de hardware e periféricos do ESP-IDF 
#include "driver/i2c.h"
#include "driver/adc.h"
#include "driver/ledc.h"
#include "driver/gpio.h"

// Bibliotecas padrão da linguagem C 
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

// FreeRTOS 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// Sistema e logging 
#include "esp_log.h"
#include "esp_system.h"

// Tag utilizada para identificação das mensagens de log 
static const char *TAG = "mesa_labirinto";

/* ===================== Mapeamento de pinos ===================== */
// Joystick 
#define JOY_H_PIN         ADC1_CHANNEL_6  // Horizontal (GPIO34)
#define JOY_V_PIN         ADC1_CHANNEL_7  // Vertical (GPIO35)
#define JOY_BTN_GPIO      GPIO_NUM_32     // Botão

// Sensor indutivo de proximidade (NPN) 
#define SENSOR_PIN        GPIO_NUM_5  

//servo motor
#define SERVO1_LEDC_CH    LEDC_CHANNEL_0  // Servo X
#define SERVO1_GPIO_NUM   13              
#define SERVO2_LEDC_CH    LEDC_CHANNEL_1  // Servo Y
#define SERVO2_GPIO_NUM   12              

// Timer LEDC utilizado para ambos os servos
#define SERVO_LEDC_TIMER  LEDC_TIMER_0

/* LED de status do sistema */
#define STATUS_LED_GPIO   GPIO_NUM_23     // Indica estado do sistema (calibração/jogando/vitoria)

/* ===================== Configuração do barramento I2C ===================== */
#define I2C_MASTER_SCL_IO     26           // Linha SCL do I2C
#define I2C_MASTER_SDA_IO     25           // Linha SDA do I2C
#define I2C_MASTER_NUM        I2C_NUM_0    // Controlador I2C utilizado
#define I2C_MASTER_FREQ_HZ    400000       // Frequência do barramento I2C (400 kHz)

/* ===================== Configuração do MPU6050 ===================== */
#define MPU_ADDR              0x68         // Endereço I2C do MPU6050
#define MPU6050_PWR_MGMT_1    0x6B         // Registrador de controle de energia
#define MPU6050_GYRO_XOUT_H   0x43         // Registrador do giroscópio eixo X
#define MPU6050_GYRO_YOUT_H   0x45         // Registrador do giroscópio eixo Y

/* ===================== Configuração do ADC ===================== */
#define ADC_WIDTH             ADC_WIDTH_BIT_12  // Resolução de 12 bits
#define ADC_MAX_VALUE         4095.0f            // Valor máximo do ADC
#define ADC_ATTEN             ADC_ATTEN_DB_11    // Atenuação para leitura até ~3.3V

/* ===================== Parâmetros do PWM dos servos ===================== */
#define SERVO_FREQ_HZ         50          // Frequência padrão de servo (50 Hz)
#define SERVO_MIN_US          500         // Pulso mínimo (posição 0°)
#define SERVO_MAX_US          2500        // Pulso máximo (posição 180°)

/* Resolução do PWM (LEDC) */
#define LEDC_DUTY_BITS        16
#define LEDC_DUTY_MAX         ((1 << LEDC_DUTY_BITS) - 1)

/* ===================== Limites de segurança ===================== */
/* Limites absolutos de operação dos servos */
#define ANGLE_MIN             8.0f
#define ANGLE_MAX             180.0f

/* ===================== Temporizações das tasks ===================== */
#define JOY_SAMPLE_MS         20      // Período de leitura do joystick (50 Hz)
#define SERVO_STEP_MS         10      // Intervalo da rampa de movimento do servo
#define MONITOR_MS            500     // Período de impressão no monitor serial

/* ===================== Parâmetros de calibração ===================== */
#define CAL_SAMPLES           200     // Número de amostras para calibração do joystick
#define DEADZONE_NORM         0.02f   // Zona morta (2%) em torno do centro do joystick

// Usado pra alinhar a mesa
float offset_X = 6.0f;   // Correção do eixo X
float offset_Y = 5.0f;   // Correção do eixo Y

// Flag usada para pausar as tasks durante a vitoria do Jogo
static volatile bool stop_task = false;


// Armazena os limites mecânicos mínimo e máximo definidos na calibração
typedef struct {
    float min_x;   
    float max_x; 
    float min_y;  
    float max_y; 
} servo_limits_t;

// Limites globais dos servos
static servo_limits_t g_limits = {
    .min_x = 8.0f,
    .max_x = 180.0f,
    .min_y = 8.0f,
    .max_y = 180.0f
};

// Flag que indica se o processo de calibração já foi concluído 
static bool g_calibration_done = false;


/* ===================== Funções de comunicação I2C ===================== */
// Envia um byte de dados para um registrador específico do MPU6050 
esp_err_t i2c_write_byte(uint8_t reg, uint8_t data){
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(cmd);

    return ret;
}

// Realiza a leitura de múltiplos bytes a partir de um registrador do MPU6050 
esp_err_t i2c_read_bytes(uint8_t reg, uint8_t *buf, size_t len){
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // Seleciona o registrador inicial 
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);

    // Inicia leitura sequencial 
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, buf, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(20));
    i2c_cmd_link_delete(cmd);

    return ret;
}

/* Inicializa o barramento I2C no modo master */
static esp_err_t i2c_master_init(void){
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));

    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}


/* ===================== Inicialização do MPU6050 ===================== */
// Configura o sensor inercial 
void mpu_init(){
    i2c_write_byte(MPU6050_PWR_MGMT_1, 0x00); // Remove modo sleep
    i2c_write_byte(0x1B, 0x00);               // Giroscópio ±250 °/s
    i2c_write_byte(0x1C, 0x00);               // Acelerômetro ±2 g

    ESP_LOGI(TAG, "MPU6050 Inicializado");
}

// Leitura dos dados do acelerômetro (em g) 
esp_err_t mpu_read_accel(float *ax, float *ay, float *az){
    uint8_t data[6];

    esp_err_t ret = i2c_read_bytes(0x3B, data, 6);
    if (ret != ESP_OK) return ret;

    int16_t raw_x = (data[0] << 8) | data[1];
    int16_t raw_y = (data[2] << 8) | data[3];
    int16_t raw_z = (data[4] << 8) | data[5];

    *ax = raw_x / 16384.0f;
    *ay = raw_y / 16384.0f;
    *az = raw_z / 16384.0f;

    return ESP_OK;
}

// Leitura dos dados do giroscópio (em °/s) 
esp_err_t mpu_read_gyro(float *gx, float *gy, float *gz){
    uint8_t data[6];

    esp_err_t ret = i2c_read_bytes(0x43, data, 6);
    if (ret != ESP_OK) return ret;

    int16_t raw_x = (data[0] << 8) | data[1];
    int16_t raw_y = (data[2] << 8) | data[3];
    int16_t raw_z = (data[4] << 8) | data[5];

    *gx = raw_x / 131.0f;
    *gy = raw_y / 131.0f;
    *gz = raw_z / 131.0f;

    return ESP_OK;
}


/* ===================== Estado compartilhado entre tasks ===================== */
// Ângulos de referência que comandam os servomotores
typedef struct {
    float target_angle_x;   // Ângulo desejado do servo X
    float target_angle_y;   // Ângulo desejado do servo Y
} shared_state_t;

// Estrutura global de estado
static shared_state_t g_state;

// Mutex para proteção do acesso concorrente ao estado 
static SemaphoreHandle_t g_state_mutex;


/* ===================== Variáveis de calibração do joystick ===================== */
// Valores normalizados do centro do joystick
static float calib_center_h = 0.5f;
static float calib_center_v = 0.5f;

// Fatores de escala para normalização (-1 a 1) 
static float calib_scale_h = 1.0f;
static float calib_scale_v = 1.0f;


/* ===================== Utilitário de mapeamento PWM ===================== */
// Converte ângulo (graus) em duty-cycle compatível com o LEDC 
static inline uint32_t angle_to_duty_us(float angle)
{
    // Garante que o ângulo respeite os limites de segurança 
    if (angle < ANGLE_MIN) angle = ANGLE_MIN;
    if (angle > ANGLE_MAX) angle = ANGLE_MAX;

    // Mapeia ângulo para largura de pulso em microssegundos 
    float pulse_us = SERVO_MIN_US +
        (angle / 180.0f) * (SERVO_MAX_US - SERVO_MIN_US);

    // Calcula fração do duty-cycle 
    float period_us = 1000000.0f / SERVO_FREQ_HZ;
    float duty_fraction = pulse_us / period_us;

    return (uint32_t)roundf(duty_fraction * LEDC_DUTY_MAX);
}


/* ===================== Inicialização de periféricos ===================== */
// Configuração do ADC para leitura do joystick 
static void init_adc(void) {
    adc1_config_width(ADC_WIDTH);
    adc1_config_channel_atten(JOY_H_PIN, ADC_ATTEN);
    adc1_config_channel_atten(JOY_V_PIN, ADC_ATTEN);

    ESP_LOGI(TAG, "ADC inicializado para leitura do joystick");
}

// Configuração do módulo LEDC para controle PWM dos servos
static void init_ledc(void) {
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_16_BIT,
        .timer_num = SERVO_LEDC_TIMER,
        .freq_hz = SERVO_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    // Canal do servo X 
    ledc_channel_config_t ch0 = {
        .gpio_num = SERVO1_GPIO_NUM,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = SERVO1_LEDC_CH,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = SERVO_LEDC_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&ch0);

    // Canal do servo Y 
    ledc_channel_config_t ch1 = {
        .gpio_num = SERVO2_GPIO_NUM,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = SERVO2_LEDC_CH,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = SERVO_LEDC_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&ch1);

    ESP_LOGI(TAG, "LEDC configurado para controle dos servos");
}

// Configuração dos GPIOs digitais 
static void init_gpio(void) {
    // Botão do joystick 
    gpio_reset_pin(JOY_BTN_GPIO);
    gpio_set_direction(JOY_BTN_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(JOY_BTN_GPIO, GPIO_PULLUP_ONLY);

    // LED de status 
    gpio_reset_pin(STATUS_LED_GPIO);
    gpio_set_direction(STATUS_LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(STATUS_LED_GPIO, 0);

    // Sensor indutivo NPN 
    gpio_reset_pin(SENSOR_PIN);
    gpio_set_direction(SENSOR_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(SENSOR_PIN, GPIO_PULLUP_ONLY);
}




/* =====================  Calibração do joystick ===================== */
static void calibrate_joystick(void) {
    uint32_t sum_h = 0;
    uint32_t sum_v = 0;

    // Coleta múltiplas amostras para reduzir ruído 
    for (int i = 0; i < CAL_SAMPLES; ++i) {
        int raw_h = adc1_get_raw(JOY_H_PIN);
        int raw_v = adc1_get_raw(JOY_V_PIN);
        sum_h += raw_h;
        sum_v += raw_v;
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    // Cálculo do valor médio das leituras 
    float avg_h = (float)sum_h / (float)CAL_SAMPLES;
    float avg_v = (float)sum_v / (float)CAL_SAMPLES;

    // Normaliza o centro do joystick para a faixa 0.0 – 1.0 
    calib_center_h = avg_h / ADC_MAX_VALUE;
    calib_center_v = avg_v / ADC_MAX_VALUE;

    // Define o ganho de escala para mapear o deslocamento em -1 a 1 
    float half_span = 0.45f;   // Margem segura para evitar saturação
    calib_scale_h = 1.0f / half_span;
    calib_scale_v = 1.0f / half_span;

    ESP_LOGI(TAG,
             "Calibracao do joystick concluida: center_h=%.3f center_v=%.3f",
             calib_center_h, calib_center_v);
}


/* ===================== Normalização da leitura do joystick ===================== */
static float norm_from_raw(int raw, float center, float scale) {
    // Normaliza o valor bruto para a faixa 0.0 – 1.0 
    float n = (float)raw / ADC_MAX_VALUE;

    // Remove o offset do centro e aplica ganho 
    float d = (n - center) * scale;

    // Limita o valor ao intervalo [-1, 1] 
    if (d > 1.0f)  d = 1.0f;
    if (d < -1.0f) d = -1.0f;

    // Aplica zona morta para evitar oscilações indesejadas
    if (fabsf(d) < DEADZONE_NORM) {
        d = 0.0f;
    }

    return d;
}


/* ===================== Task: Leitura do Joystick ===================== */
static void joystick_task(void *pv) {
    const float alpha = 0.08f;   // Fator do filtro exponencial (EMA)
    float ema_h = 0.0f;
    float ema_v = 0.0f;
    bool first = true;

    // Calibração inicial do centro do joystick 
    calibrate_joystick();

    while (1) {
        // Leitura bruta do ADC
        int raw_h = adc1_get_raw(JOY_H_PIN);
        int raw_v = adc1_get_raw(JOY_V_PIN);

        // Normalização para o intervalo [-1, 1] 
        float d_h = norm_from_raw(raw_h, calib_center_h, calib_scale_h);
        float d_v = norm_from_raw(raw_v, calib_center_v, calib_scale_v);

        // Conversão para a faixa [0, 1], onde 0.5 representa o centro 
        float n_h = (d_h + 1.0f) * 0.5f;
        float n_v = (d_v + 1.0f) * 0.5f;

        // Inicialização do filtro na primeira execução 
        if (first) {
            ema_h = n_h;
            ema_v = n_v;
            first = false;
        } else {
            // Filtro exponencial para reduzir ruído e vibrações 
            ema_h = alpha * n_h + (1.0f - alpha) * ema_h;
            ema_v = alpha * n_v + (1.0f - alpha) * ema_v;
        }

        // Conversão do valor filtrado em ângulo (graus) 
        float angle_x = ema_h * 180.0f;   // Eixo X
        float angle_y = ema_v * 180.0f;   // Eixo Y

        // Aplicação de offsets para compensação mecânic
        angle_x += offset_X;
        angle_y += offset_Y;

        // Atualiza os ângulos apenas se o jogo não estiver finalizado 
        if (!stop_task) {
            if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                g_state.target_angle_x = angle_x;
                g_state.target_angle_y = angle_y;
                xSemaphoreGive(g_state_mutex);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(JOY_SAMPLE_MS));
    }
}


/* ===================== Task: Controle dos Servomotores ===================== */
static void servo_task(void *pv)
{
    float current_x = 90.0f;   // Posição inicial do servo X
    float current_y = 90.0f;   // Posição inicial do servo Y

    // Sincroniza a posição inicial com o estado atual 
    if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        current_x = g_state.target_angle_x;
        current_y = g_state.target_angle_y;
        xSemaphoreGive(g_state_mutex);
    }

    const float step_deg = 0.5f;  // Incremento angular por ciclo (rampa)

    // Aplica a posição inicial aos servos
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, SERVO1_LEDC_CH,
                  angle_to_duty_us(current_x));
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, SERVO1_LEDC_CH);

    ledc_set_duty(LEDC_HIGH_SPEED_MODE, SERVO2_LEDC_CH,
                  angle_to_duty_us(current_y));
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, SERVO2_LEDC_CH);

    while (1)
    {
        float target_x = current_x;
        float target_y = current_y;

        // Leitura protegida dos ângulos desejados 
        if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            target_x = g_state.target_angle_x;
            target_y = g_state.target_angle_y;
            xSemaphoreGive(g_state_mutex);
        }

        // Atualização gradual do servo X 
        if (fabsf(target_x - current_x) > 0.5f) {
            current_x += (target_x > current_x) ? step_deg : -step_deg;
            current_x = fminf(fmaxf(current_x, g_limits.min_x), g_limits.max_x);

            ledc_set_duty(LEDC_HIGH_SPEED_MODE, SERVO1_LEDC_CH,
                          angle_to_duty_us(current_x));
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, SERVO1_LEDC_CH);
        }

        // Atualização gradual do servo Y 
        if (fabsf(target_y - current_y) > 0.5f) {
            current_y += (target_y > current_y) ? step_deg : -step_deg;
            current_y = fminf(fmaxf(current_y, g_limits.min_y), g_limits.max_y);

            ledc_set_duty(LEDC_HIGH_SPEED_MODE, SERVO2_LEDC_CH,
                          angle_to_duty_us(current_y));
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, SERVO2_LEDC_CH);
        }

        vTaskDelay(pdMS_TO_TICKS(SERVO_STEP_MS));
    }
}


/* ===================== Task: Monitoramento Serial ===================== */
static void monitor_task(void *pv) {
    while (1) {
        // Atualiza o valor do eixo x e y apenas se o jogo não estiver finalizado 
        if (!stop_task) {
            float tx, ty;

            if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                tx = g_state.target_angle_x;
                ty = g_state.target_angle_y;
                xSemaphoreGive(g_state_mutex);
            } else {
                tx = 0.0f;
                ty = 0.0f;
            }

            ESP_LOGI(TAG,
                     "{\"alvo_x\":%.1f,\"alvo_y\":%.1f}",
                     tx - offset_X, ty - offset_Y);
        }

        vTaskDelay(pdMS_TO_TICKS(MONITOR_MS));
    }
}

/* ===================== Task: Leitura do sensor MPU6050 ===================== */
void mpu_task(void *pvParameters){
    float ax, ay, az;   // Acelerometro
    float gx, gy, gz;   // Giroscopio

    // Ângulos filtrados estimados 
    float angle_x = 0.0f;
    float angle_y = 0.0f;

    // Valores anteriores para detecção de variação significativa 
    float prev_angle_x = 0.0f;
    float prev_angle_y = 0.0f;

    const float alpha = 0.98f;  // Peso do giroscópio no filtro complementar
    const float dt = 0.02f;     // Período de amostragem (20 ms)

    const float PRINT_THRESHOLD = 1.0f;  // Variação mínima (graus) para imprimir

    uint32_t last_print = 0;    // Controle de tempo entre impressões

    while (1)
    {
        // Leitura simultânea do acelerômetro e do giroscópio 
        if (mpu_read_accel(&ax, &ay, &az) == ESP_OK &&
            mpu_read_gyro(&gx, &gy, &gz) == ESP_OK)
        {
            // Estimativa dos ângulos a partir do acelerômetro 
            float acc_angle_x = atan2(ay, az) * 180.0f / M_PI;
            float acc_angle_y = atan2(ax, az) * 180.0f / M_PI;

            // Aplicação do filtro complementar 
            angle_x = alpha * (angle_x + gx * dt) +
                      (1.0f - alpha) * acc_angle_x;

            angle_y = alpha * (angle_y + gy * dt) +
                      (1.0f - alpha) * acc_angle_y;

            uint32_t now = xTaskGetTickCount();

            /*
             * Imprime os valores apenas se:
             *  - houve variação significativa do ângulo
             *  - respeitado o intervalo mínimo entre impressões
             *  - o sistema não estiver no estado de fim do labirinto
             */
            if ( (fabsf(angle_x - prev_angle_x) > PRINT_THRESHOLD ||
                  fabsf(angle_y - prev_angle_y) > PRINT_THRESHOLD) &&
                 (now - last_print > pdMS_TO_TICKS(100)) )
            {
                if (!stop_task) {
                    printf("{\"pitch\": %.2f, \"roll\": %.2f}\n",
                           angle_x, angle_y);
                }

                prev_angle_x = angle_x;
                prev_angle_y = angle_y;
                last_print = now;
            }

        } else {
            // Indica falha na comunicação com o MPU6050 
            printf("Erro lendo MPU.\n");
        }

        // Mantém a taxa de amostragem compatível com o dt
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}


/* ===================== Botão do joystick ===================== */
static bool button_pressed(void) {
    static uint32_t last = 0;
    uint32_t now = xTaskGetTickCount();

    if (gpio_get_level(JOY_BTN_GPIO) == 0 &&
        (now - last) > pdMS_TO_TICKS(300)) {

        last = now;
        return true;
    }

    return false;
}


/* ===================== Rotina de calibração dos limites dos servos ===================== */
static void calibrate_servo_limits(void) {
    float pos = 90.0f;   // Posição inicial do servo durante a calibração

    ESP_LOGI(TAG, "=== MODO CONFIGURACAO DE LIMITES ===");

    /* ================= SERVO X ================= */
    ESP_LOGI(TAG, "Servo X - Defina o MINIMO e pressione o botao");

    // Ajustar a posição mínima do servo X
    while (!button_pressed()) {
        int raw = adc1_get_raw(JOY_H_PIN);
        float d = norm_from_raw(raw, calib_center_h, calib_scale_h);

        // Incrementa ou decrementa a posição conforme o deslocamento do joystick
        pos += d * 0.5f;

        // Garante que o valor permaneça dentro do intervalo absoluto 
        pos = fminf(fmaxf(pos, 0.0f), 180.0f);

        // Atualiza a posição do servo em tempo real
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, SERVO1_LEDC_CH,
                      angle_to_duty_us(pos));
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, SERVO1_LEDC_CH);

        vTaskDelay(pdMS_TO_TICKS(20));
    }

    // Armazena o limite mínimo do servo X */
    g_limits.min_x = pos;

    ESP_LOGI(TAG, "Servo X - Defina o MAXIMO e pressione o botao");

    // Ajustar a posição máxima do servo X
    while (!button_pressed()) {
        int raw = adc1_get_raw(JOY_H_PIN);
        float d = norm_from_raw(raw, calib_center_h, calib_scale_h);

        pos += d * 0.5f;
        pos = fminf(fmaxf(pos, 0.0f), 180.0f);

        ledc_set_duty(LEDC_HIGH_SPEED_MODE, SERVO1_LEDC_CH,
                      angle_to_duty_us(pos));
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, SERVO1_LEDC_CH);

        vTaskDelay(pdMS_TO_TICKS(20));
    }

    // Retorna o servo X para a posição central antes de iniciar o eixo Y
    ESP_LOGI(TAG, "Retornando Servo X para 90 graus");

    pos = 90.0f;
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, SERVO1_LEDC_CH,
                  angle_to_duty_us(pos));
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, SERVO1_LEDC_CH);

    vTaskDelay(pdMS_TO_TICKS(300));   // Tempo para estabilização mecânica


    /* ================= SERVO Y ================= */
    pos = 90.0f;
    ESP_LOGI(TAG, "Servo Y - Defina o MINIMO e pressione o botao");

    // Ajustar a posição mínima do servo Y 
    while (!button_pressed()) {
        int raw = adc1_get_raw(JOY_V_PIN);
        float d = norm_from_raw(raw, calib_center_v, calib_scale_v);

        pos += d * 0.5f;
        pos = fminf(fmaxf(pos, 0.0f), 180.0f);

        ledc_set_duty(LEDC_HIGH_SPEED_MODE, SERVO2_LEDC_CH,
                      angle_to_duty_us(pos));
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, SERVO2_LEDC_CH);

        vTaskDelay(pdMS_TO_TICKS(20));
    }

    // Armazena o limite mínimo do servo Y 
    g_limits.min_y = pos;

    ESP_LOGI(TAG, "Servo Y - Defina o MAXIMO e pressione o botao");

    //Ajustar a posição máximo do servo Y 
    while (!button_pressed()) {
        int raw = adc1_get_raw(JOY_V_PIN);
        float d = norm_from_raw(raw, calib_center_v, calib_scale_v);

        pos += d * 0.5f;
        pos = fminf(fmaxf(pos, 0.0f), 180.0f);

        ledc_set_duty(LEDC_HIGH_SPEED_MODE, SERVO2_LEDC_CH,
                      angle_to_duty_us(pos));
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, SERVO2_LEDC_CH);

        vTaskDelay(pdMS_TO_TICKS(20));
    }

    // Armazena o limite máximo do servo Y
    g_limits.max_y = pos;

    ESP_LOGI(TAG, "=== CALIBRACAO FINALIZADA ===");
    g_calibration_done = true;
}


/* ===================== Task: Sensor indutivo (vitoria) ===================== */
static void sensor_task(void *pvParameters) {
    bool last_state = true;    // Estado anterior do sensor (pull-up)
    bool led_state = false;   // Estado interno do LED para efeito de pisca

    while (1) {
        // Sensor NPN: nível baixo indica detecção de metal 
        bool sensor_active = (gpio_get_level(SENSOR_PIN) == 0);

        // Transição: entrada na zona final 
        if (sensor_active && last_state) {
            ESP_LOGI(TAG, "Voce chegou ao fim do Labirinto!!");
            stop_task = true;
        }

        // Transição: saída da zona final 
        if (!sensor_active && !last_state) {
            ESP_LOGI(TAG, "Tente chegar ao fim novamente!");
            stop_task = false;
        }

        // Controle visual por LED 
        if (sensor_active) {
            // Pisca enquanto o fim do labirinto estiver ativo 
            led_state = !led_state;
            gpio_set_level(STATUS_LED_GPIO, led_state);
        } else {
            // LED aceso continuamente durante operação normal 
            gpio_set_level(STATUS_LED_GPIO, 1);
        }

        // Atualiza o estado anterior do sensor
        last_state = !sensor_active;

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* ===================== APP_MAIN ===================== */
void app_main(void) {
    ESP_LOGI(TAG, "Inicializando Mesa Labirinto");

    // Inicialização de sincronização

    g_state_mutex = xSemaphoreCreateMutex();
    if (g_state_mutex == NULL) {
        ESP_LOGE(TAG, "Falha ao criar mutex");
        abort();   // Interrompe a execução em caso de erro crítico
    }

    // Inicialização de periféricos
    init_gpio();   // GPIOs: botão, LED de status e sensor indutivo
    init_adc();    // Conversores ADC para leitura do joystick
    init_ledc();   // PWM (LEDC) para controle dos servomotores

    // Inicialização da interface I2C e do sensor MPU6050 
    ESP_ERROR_CHECK(i2c_master_init());
    mpu_init();

    // Calibração dos Servo motores
    calibrate_servo_limits();

    // Criação das task
    BaseType_t r;

    // Task do sensor indutivo de proximidade
    r = xTaskCreate(sensor_task, "sensor_task", 2048, NULL, 5, NULL);
    if (r != pdPASS) ESP_LOGE(TAG, "Falha ao criar sensor_task");

    // Task do joystick
    r = xTaskCreate(joystick_task, "joystick_task", 4096, NULL, 6, NULL);
    if (r != pdPASS) ESP_LOGE(TAG, "Falha ao criar joystick_task");

    // Task dos servomotores
    r = xTaskCreate(servo_task, "servo_task", 4096, NULL, 7, NULL);
    if (r != pdPASS) ESP_LOGE(TAG, "Falha ao criar servo_task");

    // Task de monitoramento serial
    r = xTaskCreate(monitor_task, "monitor_task", 3072, NULL, 5, NULL);
    if (r != pdPASS) ESP_LOGE(TAG, "Falha ao criar monitor_task");

    // Task do MPU6050
    r = xTaskCreate(mpu_task, "mpu_task", 4096, NULL, 1, NULL);
    if (r != pdPASS) ESP_LOGE(TAG, "Falha ao criar mpu_task");

    ESP_LOGI(TAG, "Sistema iniciado. Use o joystick para mover a mesa.");
}
