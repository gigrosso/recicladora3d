/**
 * Sistema de Controle de Extrusora
 * Raspberry Pi Pico W
 * Desenvolvido para controle de temperatura e motor de extrusora de plástico
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "pico/time.h"
#include <string.h>
#include <math.h>

// ==================== DEFINIÇÕES DE PINOS ====================
// Preservando GP0 e GP1 para UART (conforme documentação)
// GP0 = UART0_TX, GP1 = UART0_RX

// Multiplexadores CD74HC4067E
#define MUX1_S0 2   // Multiplexador 1 (Botões) - Sinal S0
#define MUX1_S1 3   // Multiplexador 1 (Botões) - Sinal S1
#define MUX1_S2 4   // Multiplexador 1 (Botões) - Sinal S2
#define MUX1_S3 5   // Multiplexador 1 (Botões) - Sinal S3
#define MUX1_SIG 26 // Multiplexador 1 (Botões) - Sinal analógico/digital (ADC0)

#define MUX2_S0 6   // Multiplexador 2 (LEDs) - Sinal S0
#define MUX2_S1 7   // Multiplexador 2 (LEDs) - Sinal S1
#define MUX2_S2 8   // Multiplexador 2 (LEDs) - Sinal S2
#define MUX2_S3 9   // Multiplexador 2 (LEDs) - Sinal S3
#define MUX2_SIG 10 // Multiplexador 2 (LEDs) - Sinal digital de saída

// Sensores PT100 (via ADC)
#define SENSOR1_ADC 27 // ADC1 - Bico da extrusora
#define SENSOR2_ADC 28 // ADC2 - Zona de dosagem
#define SENSOR3_ADC 29 // ADC3 - Zona de compressão
// Sensor 4 também pode usar GPIO 22 como ADC se necessário

// MOSFETs
#define MOSFET1 11 // Aquecimento zona de dosagem (Sensor 2)
#define MOSFET2 12 // Aquecimento zona de compressão (Sensor 3)
#define MOSFET3 13 // Resfriamento zona de alimentação (Sensor 4)
#define MOSFET4 14 // Corrente para o motor

// Driver TB6600 (Nema 23)
#define MOTOR_STEP 15  // Pino de STEP
#define MOTOR_DIR 16   // Pino de DIRECTION
#define MOTOR_ENABLE 17 // Pino de ENABLE

// LCD I2C (JHD 162A com módulo I2C)
#define I2C_PORT i2c0
#define I2C_SDA 20
#define I2C_SCL 21
#define LCD_ADDR 0x27 // Endereço I2C comum para módulos LCD I2C

// Sensor Ultrassônico
#define ULTRASONIC_TRIG 18
#define ULTRASONIC_ECHO 19

// ==================== ESTRUTURAS DE DADOS ====================

// Estados dos botões
typedef enum {
    BTN_ATIVA = 0,
    BTN_ALTERA_UNIDADE = 1,
    BTN_AUMENT_T1 = 2,
    BTN_DIMIN_T1 = 3,
    BTN_AUMENT_T2 = 4,
    BTN_DIMIN_T2 = 5,
    BTN_AUMENT_T3 = 6,
    BTN_DIMIN_T3 = 7,
    BTN_AUMENT_T4 = 8,
    BTN_DIMIN_T4 = 9,
    BTN_PARA_EXTRUSAO = 10,
    BTN_INICIA_EXTRUSAO = 11,
    BTN_SELECIONA_MATERIAL = 12,
    BTN_PARA_TUDO = 13,
    BTN_MOTOR_TOGGLE = 14,
    BTN_AUMENTA_RPM = 15
} Botoes;

// Estados dos LEDs
typedef enum {
    LED_ON = 0,
    LED_AQUECENDO_S2 = 1,
    LED_AQUECENDO_S3 = 2,
    LED_TEMP_ALVO_1 = 3,
    LED_TEMP_ALVO_2 = 4,
    LED_TEMP_ALVO_3 = 5,
    LED_TEMP_4_OK = 6,
    LED_UNIDADE_1 = 7,
    LED_UNIDADE_2 = 8,
    LED_MATERIAL_ACABANDO = 9,
    LED_MATERIAL_ACABOU = 10,
    LED_TEMP_ALTA = 11,
    LED_RESERVA_12 = 12,
    LED_RESERVA_13 = 13,
    LED_RESERVA_14 = 14,
    LED_RESERVA_15 = 15
} Leds;

// Tipos de material
typedef enum {
    MAT_ABS = 0,
    MAT_PLA = 1,
    MAT_PET = 2,
    MAT_SMD = 3
} TipoMaterial;

// Estrutura de configuração de temperatura para cada material
typedef struct {
    float extrusor_min;
    float extrusor_max;
    float dosagem_min;
    float dosagem_max;
    float compressao_min;
    float compressao_max;
    float alimentacao_max;
    const char* nome;
} ConfigMaterial;

// Estrutura principal do sistema
typedef struct {
    TipoMaterial material_atual;
    float temp_atual[4];      // Temperaturas atuais dos 4 sensores
    float temp_alvo[4];       // Temperaturas alvo dos 4 sensores
    bool aquecimento_ativo[3]; // Aquecimento das zonas 1, 2, 3
    bool resfriamento_ativo;   // Resfriamento da zona 4
    bool motor_ligado;
    bool extrusao_ativa;
    bool sistema_ativo;
    bool erro_ativo;
    uint8_t unidade_alteracao; // 0=Celsius, 1=Fahrenheit
    uint32_t ultimo_tempo_mudanca_temp[4];
    int rpm_motor;
    bool motor_rodando_temp;
    float nivel_material;
} SistemaExtrusora;

// ==================== VARIÁVEIS GLOBAIS ====================

SistemaExtrusora sistema;
bool botoes_estado_anterior[16] = {false};
bool leds_estado[16] = {false};

// Configurações de temperatura para cada material
ConfigMaterial config_materiais[4] = {
    // ABS
    {210, 250, 180, 220, 160, 200, 100, "ABS"},
    // PLA
    {180, 220, 160, 200, 140, 180, 70, "PLA"},
    // PET
    {230, 270, 200, 240, 180, 220, 100, "PET"},
    // SMD (Sem material definido - valores padrão)
    {0, 300, 0, 300, 0, 300, 100, "SMD"}
};

// ==================== FUNÇÕES DE HARDWARE ====================

// Inicializa os pinos do multiplexador
void init_mux_pins() {
    // Multiplexador 1 (Botões)
    gpio_init(MUX1_S0);
    gpio_init(MUX1_S1);
    gpio_init(MUX1_S2);
    gpio_init(MUX1_S3);
    gpio_init(MUX1_SIG);
    
    gpio_set_dir(MUX1_S0, GPIO_OUT);
    gpio_set_dir(MUX1_S1, GPIO_OUT);
    gpio_set_dir(MUX1_S2, GPIO_OUT);
    gpio_set_dir(MUX1_S3, GPIO_OUT);
    gpio_set_dir(MUX1_SIG, GPIO_IN);
    gpio_pull_up(MUX1_SIG); // Pull-up para botões
    
    // Multiplexador 2 (LEDs)
    gpio_init(MUX2_S0);
    gpio_init(MUX2_S1);
    gpio_init(MUX2_S2);
    gpio_init(MUX2_S3);
    gpio_init(MUX2_SIG);
    
    gpio_set_dir(MUX2_S0, GPIO_OUT);
    gpio_set_dir(MUX2_S1, GPIO_OUT);
    gpio_set_dir(MUX2_S2, GPIO_OUT);
    gpio_set_dir(MUX2_S3, GPIO_OUT);
    gpio_set_dir(MUX2_SIG, GPIO_OUT);
}

// Seleciona canal do multiplexador
void select_mux_channel(uint8_t channel, bool is_mux1) {
    if (is_mux1) {
        gpio_put(MUX1_S0, channel & 0x01);
        gpio_put(MUX1_S1, (channel >> 1) & 0x01);
        gpio_put(MUX1_S2, (channel >> 2) & 0x01);
        gpio_put(MUX1_S3, (channel >> 3) & 0x01);
    } else {
        gpio_put(MUX2_S0, channel & 0x01);
        gpio_put(MUX2_S1, (channel >> 1) & 0x01);
        gpio_put(MUX2_S2, (channel >> 2) & 0x01);
        gpio_put(MUX2_S3, (channel >> 3) & 0x01);
    }
    sleep_us(10); // Pequeno delay para estabilização
}

// Lê estado do botão
bool ler_botao(uint8_t botao) {
    select_mux_channel(botao, true);
    return !gpio_get(MUX1_SIG); // Invertido porque é pull-up
}

// Controla LED
void set_led(uint8_t led, bool estado) {
    select_mux_channel(led, false);
    gpio_put(MUX2_SIG, estado);
    leds_estado[led] = estado;
}

// Inicializa I2C para LCD
void init_i2c_lcd() {
    i2c_init(I2C_PORT, 100 * 1000); // 100 kHz
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
}

// Funções básicas de LCD I2C (implementação simplificada)
void lcd_send_byte(uint8_t val, int mode) {
    uint8_t high = mode | (val & 0xF0) | 0x08; // Enable bit high
    uint8_t low = mode | ((val << 4) & 0xF0) | 0x08;
    
    uint8_t data[4];
    data[0] = high | 0x04; // En=1
    data[1] = high & ~0x04; // En=0
    data[2] = low | 0x04;  // En=1
    data[3] = low & ~0x04; // En=0
    
    i2c_write_blocking(I2C_PORT, LCD_ADDR, data, 4, false);
}

void lcd_clear() {
    lcd_send_byte(0x01, 0);
    sleep_ms(2);
}

void lcd_set_cursor(int line, int position) {
    int val = (line == 0) ? 0x80 + position : 0xC0 + position;
    lcd_send_byte(val, 0);
}

void lcd_char(char val) {
    lcd_send_byte(val, 1);
}

void lcd_string(const char *s) {
    while (*s) {
        lcd_char(*s++);
    }
}

void lcd_init() {
    sleep_ms(50);
    lcd_send_byte(0x03, 0);
    sleep_ms(5);
    lcd_send_byte(0x03, 0);
    sleep_ms(5);
    lcd_send_byte(0x03, 0);
    sleep_ms(1);
    lcd_send_byte(0x02, 0);
    
    lcd_send_byte(0x28, 0); // Function set: 4-bit, 2 line, 5x8
    lcd_send_byte(0x0C, 0); // Display on, cursor off, blink off
    lcd_send_byte(0x06, 0); // Entry mode
    lcd_clear();
}

// Inicializa ADC para sensores PT100
void init_adc() {
    adc_init();
    adc_gpio_init(SENSOR1_ADC);
    adc_gpio_init(SENSOR2_ADC);
    adc_gpio_init(SENSOR3_ADC);
}

// Lê temperatura do sensor PT100 (conversão simplificada)
float ler_temperatura_pt100(uint8_t sensor) {
    uint8_t adc_channel;
    
    switch(sensor) {
        case 0: adc_channel = 1; break; // ADC1 (GPIO27)
        case 1: adc_channel = 2; break; // ADC2 (GPIO28)
        case 2: adc_channel = 3; break; // ADC3 (GPIO29)
        default: return 0.0;
    }
    
    adc_select_input(adc_channel);
    uint16_t adc_value = adc_read();
    
    // Conversão simplificada de ADC para temperatura
    // PT100: 100Ω a 0°C, ~0.385Ω/°C
    // Esta é uma aproximação - ajustar conforme calibração real
    float voltage = (adc_value / 4095.0) * 3.3;
    float resistance = (voltage / (3.3 - voltage)) * 1000.0; // Assumindo divisor de tensão
    float temperatura = (resistance - 100.0) / 0.385;
    
    return temperatura;
}

// Inicializa MOSFETs
void init_mosfets() {
    gpio_init(MOSFET1);
    gpio_init(MOSFET2);
    gpio_init(MOSFET3);
    gpio_init(MOSFET4);
    
    gpio_set_dir(MOSFET1, GPIO_OUT);
    gpio_set_dir(MOSFET2, GPIO_OUT);
    gpio_set_dir(MOSFET3, GPIO_OUT);
    gpio_set_dir(MOSFET4, GPIO_OUT);
    
    gpio_put(MOSFET1, 0);
    gpio_put(MOSFET2, 0);
    gpio_put(MOSFET3, 0);
    gpio_put(MOSFET4, 0);
}

// Inicializa motor e driver TB6600
void init_motor() {
    gpio_init(MOTOR_STEP);
    gpio_init(MOTOR_DIR);
    gpio_init(MOTOR_ENABLE);
    
    gpio_set_dir(MOTOR_STEP, GPIO_OUT);
    gpio_set_dir(MOTOR_DIR, GPIO_OUT);
    gpio_set_dir(MOTOR_ENABLE, GPIO_OUT);
    
    gpio_put(MOTOR_STEP, 0);
    gpio_put(MOTOR_DIR, 1); // Direção padrão
    gpio_put(MOTOR_ENABLE, 1); // Desabilitado (lógica invertida em alguns drivers)
}

// Controla rotação do motor
void motor_step(int rpm) {
    if (rpm <= 0) return;
    
    // Cálculo do delay entre passos
    // 200 passos/revolução (motor Nema 23 típico)
    // delay_us = (60 * 1000000) / (rpm * 200 * 2) // *2 porque é toggle
    uint32_t delay_us = (60 * 1000000) / (rpm * 200 * 2);
    
    gpio_put(MOTOR_STEP, 1);
    sleep_us(delay_us);
    gpio_put(MOTOR_STEP, 0);
    sleep_us(delay_us);
}

// Inicializa sensor ultrassônico
void init_ultrasonic() {
    gpio_init(ULTRASONIC_TRIG);
    gpio_init(ULTRASONIC_ECHO);
    gpio_set_dir(ULTRASONIC_TRIG, GPIO_OUT);
    gpio_set_dir(ULTRASONIC_ECHO, GPIO_IN);
}

// Lê distância do sensor ultrassônico
float ler_distancia_ultrasonic() {
    gpio_put(ULTRASONIC_TRIG, 0);
    sleep_us(2);
    gpio_put(ULTRASONIC_TRIG, 1);
    sleep_us(10);
    gpio_put(ULTRASONIC_TRIG, 0);
    
    uint32_t timeout = 30000;
    uint32_t start_time, end_time;
    
    // Espera pelo início do pulso ECHO
    while (gpio_get(ULTRASONIC_ECHO) == 0 && timeout > 0) {
        timeout--;
    }
    start_time = time_us_32();
    
    timeout = 30000;
    // Espera pelo fim do pulso ECHO
    while (gpio_get(ULTRASONIC_ECHO) == 1 && timeout > 0) {
        timeout--;
    }
    end_time = time_us_32();
    
    uint32_t pulse_duration = end_time - start_time;
    float distance = (pulse_duration * 0.0343) / 2.0; // Distância em cm
    
    return distance;
}

// ==================== FUNÇÕES DE CONTROLE ====================

// Inicializa sistema
void init_sistema() {
    sistema.material_atual = MAT_SMD;
    sistema.motor_ligado = false;
    sistema.extrusao_ativa = false;
    sistema.sistema_ativo = false;
    sistema.erro_ativo = false;
    sistema.unidade_alteracao = 0; // Celsius
    sistema.rpm_motor = 4;
    sistema.motor_rodando_temp = false;
    sistema.nivel_material = 100.0;
    sistema.resfriamento_ativo = false;
    
    for (int i = 0; i < 4; i++) {
        sistema.temp_atual[i] = 20.0;
        sistema.temp_alvo[i] = 0.0;
        sistema.ultimo_tempo_mudanca_temp[i] = time_us_32();
    }
    
    for (int i = 0; i < 3; i++) {
        sistema.aquecimento_ativo[i] = false;
    }
}

// Converte Celsius para Fahrenheit
float celsius_para_fahrenheit(float celsius) {
    return (celsius * 9.0 / 5.0) + 32.0;
}

// Atualiza display LCD
void atualizar_display() {
    if (sistema.erro_ativo) {
        lcd_clear();
        lcd_set_cursor(0, 0);
        lcd_string("     ERRO      ");
        return;
    }
    
    char linha1[17];
    char linha2[17];
    
    // Primeira linha: M:XXX E123/123
    const char* nome_material = config_materiais[sistema.material_atual].nome;
    
    int temp1_atual = (int)sistema.temp_atual[0];
    int temp1_alvo = (int)sistema.temp_alvo[0];
    
    snprintf(linha1, 17, "M:%-3s E%3d/%-3d", nome_material, temp1_atual, temp1_alvo);
    
    // Segunda linha: D123/123C123/123
    int temp2_atual = (int)sistema.temp_atual[1];
    int temp2_alvo = (int)sistema.temp_alvo[1];
    int temp3_atual = (int)sistema.temp_atual[2];
    int temp3_alvo = (int)sistema.temp_alvo[2];
    
    snprintf(linha2, 17, "D%3d/%-3dC%3d/%-3d", 
             temp2_atual, temp2_alvo, 
             temp3_atual, temp3_alvo);
    
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_string(linha1);
    lcd_set_cursor(1, 0);
    lcd_string(linha2);
}

// Configura temperaturas padrão do material
void configurar_material(TipoMaterial material) {
    sistema.material_atual = material;
    
    if (material != MAT_SMD) {
        // Configura temperaturas alvo como média das faixas
        sistema.temp_alvo[0] = (config_materiais[material].extrusor_min + 
                                config_materiais[material].extrusor_max) / 2.0;
        sistema.temp_alvo[1] = (config_materiais[material].dosagem_min + 
                                config_materiais[material].dosagem_max) / 2.0;
        sistema.temp_alvo[2] = (config_materiais[material].compressao_min + 
                                config_materiais[material].compressao_max) / 2.0;
        sistema.temp_alvo[3] = config_materiais[material].alimentacao_max - 10;
    }
}

// Verifica se temperatura está dentro da faixa permitida
bool temperatura_dentro_faixa(int sensor) {
    if (sistema.material_atual == MAT_SMD) {
        return true; // Modo manual
    }
    
    ConfigMaterial* config = &config_materiais[sistema.material_atual];
    float temp = sistema.temp_atual[sensor];
    
    switch (sensor) {
        case 0: // Extrusor
            return (temp >= config->extrusor_min && temp <= config->extrusor_max);
        case 1: // Dosagem
            return (temp >= config->dosagem_min && temp <= config->dosagem_max);
        case 2: // Compressão
            return (temp >= config->compressao_min && temp <= config->compressao_max);
        case 3: // Alimentação
            return (temp <= config->alimentacao_max);
        default:
            return false;
    }
}

// Controla aquecimento e resfriamento
void controlar_temperatura() {
    uint32_t tempo_atual = time_us_32();
    
    // Sensor 1 (Extrusor) - sem controle direto de aquecimento neste exemplo
    // Sensor 2 (Dosagem) - MOSFET1
    if (sistema.temp_atual[1] < sistema.temp_alvo[1] - 2.0) {
        gpio_put(MOSFET1, 1);
        sistema.aquecimento_ativo[1] = true;
        set_led(LED_AQUECENDO_S2, true);
    } else if (sistema.temp_atual[1] >= sistema.temp_alvo[1]) {
        gpio_put(MOSFET1, 0);
        sistema.aquecimento_ativo[1] = false;
        set_led(LED_AQUECENDO_S2, false);
        set_led(LED_TEMP_ALVO_2, true);
    }
    
    // Sensor 3 (Compressão) - MOSFET2
    if (sistema.temp_atual[2] < sistema.temp_alvo[2] - 2.0) {
        gpio_put(MOSFET2, 1);
        sistema.aquecimento_ativo[2] = true;
        set_led(LED_AQUECENDO_S3, true);
    } else if (sistema.temp_atual[2] >= sistema.temp_alvo[2]) {
        gpio_put(MOSFET2, 0);
        sistema.aquecimento_ativo[2] = false;
        set_led(LED_AQUECENDO_S3, false);
        set_led(LED_TEMP_ALVO_3, true);
    }
    
    // Sensor 4 (Alimentação) - MOSFET3 para resfriamento
    float temp_max_alimentacao = config_materiais[sistema.material_atual].alimentacao_max;
    if (sistema.temp_atual[3] > temp_max_alimentacao) {
        gpio_put(MOSFET3, 1);
        sistema.resfriamento_ativo = true;
        set_led(LED_TEMP_ALTA, true);
    } else {
        gpio_put(MOSFET3, 0);
        sistema.resfriamento_ativo = false;
        set_led(LED_TEMP_ALTA, false);
        if (sistema.temp_atual[3] <= temp_max_alimentacao - 5) {
            set_led(LED_TEMP_4_OK, true);
        }
    }
    
    // Verifica se temperatura mudou (para detecção de erro)
    for (int i = 0; i < 4; i++) {
        if (fabs(sistema.temp_atual[i] - ler_temperatura_pt100(i)) > 1.0) {
            sistema.ultimo_tempo_mudanca_temp[i] = tempo_atual;
        }
        
        // Verifica se passou 5 minutos sem mudança
        if ((tempo_atual - sistema.ultimo_tempo_mudanca_temp[i]) > 5 * 60 * 1000000) {
            sistema.erro_ativo = true;
        }
    }
}

// Verifica se pode ligar motor
bool pode_ligar_motor() {
    // Motor só liga quando temperaturas estão até 5% mais frias que a faixa configurada
    float margem_seguranca = 0.95; // 5% mais frio
    
    ConfigMaterial* config = &config_materiais[sistema.material_atual];
    
    bool temp1_ok = sistema.temp_atual[0] >= (config->extrusor_min * margem_seguranca) &&
                    sistema.temp_atual[0] <= config->extrusor_max;
    bool temp2_ok = sistema.temp_atual[1] >= (config->dosagem_min * margem_seguranca) &&
                    sistema.temp_atual[1] <= config->dosagem_max;
    bool temp3_ok = sistema.temp_atual[2] >= (config->compressao_min * margem_seguranca) &&
                    sistema.temp_atual[2] <= config->compressao_max;
    
    return temp1_ok && temp2_ok && temp3_ok && !sistema.erro_ativo;
}

// Processa botões
void processar_botoes() {
    for (int i = 0; i < 16; i++) {
        bool estado_atual = ler_botao(i);
        
        // Detecta borda de subida (botão pressionado)
        if (estado_atual && !botoes_estado_anterior[i]) {
            switch (i) {
                case BTN_ATIVA:
                    sistema.sistema_ativo = !sistema.sistema_ativo;
                    set_led(LED_ON, sistema.sistema_ativo);
                    sistema.erro_ativo = false;
                    break;
                    
                case BTN_ALTERA_UNIDADE:
                    sistema.unidade_alteracao = !sistema.unidade_alteracao;
                    set_led(LED_UNIDADE_1, sistema.unidade_alteracao == 0);
                    set_led(LED_UNIDADE_2, sistema.unidade_alteracao == 1);
                    break;
                    
                case BTN_AUMENT_T1:
                    sistema.temp_alvo[0] += (sistema.unidade_alteracao == 0) ? 5.0 : 9.0;
                    break;
                    
                case BTN_DIMIN_T1:
                    sistema.temp_alvo[0] -= (sistema.unidade_alteracao == 0) ? 5.0 : 9.0;
                    break;
                    
                case BTN_AUMENT_T2:
                    sistema.temp_alvo[1] += (sistema.unidade_alteracao == 0) ? 5.0 : 9.0;
                    break;
                    
                case BTN_DIMIN_T2:
                    sistema.temp_alvo[1] -= (sistema.unidade_alteracao == 0) ? 5.0 : 9.0;
                    break;
                    
                case BTN_AUMENT_T3:
                    sistema.temp_alvo[2] += (sistema.unidade_alteracao == 0) ? 5.0 : 9.0;
                    break;
                    
                case BTN_DIMIN_T3:
                    sistema.temp_alvo[2] -= (sistema.unidade_alteracao == 0) ? 5.0 : 9.0;
                    break;
                    
                case BTN_AUMENT_T4:
                    sistema.temp_alvo[3] += (sistema.unidade_alteracao == 0) ? 5.0 : 9.0;
                    break;
                    
                case BTN_DIMIN_T4:
                    sistema.temp_alvo[3] -= (sistema.unidade_alteracao == 0) ? 5.0 : 9.0;
                    break;
                    
                case BTN_PARA_EXTRUSAO:
                    sistema.extrusao_ativa = false;
                    sistema.motor_ligado = false;
                    gpio_put(MOTOR_ENABLE, 1);
                    gpio_put(MOSFET4, 0);
                    break;
                    
                case BTN_INICIA_EXTRUSAO:
                    if (pode_ligar_motor()) {
                        sistema.extrusao_ativa = true;
                        sistema.motor_ligado = true;
                        gpio_put(MOTOR_ENABLE, 0);
                        gpio_put(MOSFET4, 1);
                    }
                    break;
                    
                case BTN_SELECIONA_MATERIAL:
                    sistema.material_atual = (TipoMaterial)((sistema.material_atual + 1) % 4);
                    configurar_material(sistema.material_atual);
                    break;
                    
                case BTN_PARA_TUDO:
                    sistema.sistema_ativo = false;
                    sistema.motor_ligado = false;
                    sistema.extrusao_ativa = false;
                    gpio_put(MOSFET1, 0);
                    gpio_put(MOSFET2, 0);
                    gpio_put(MOSFET3, 0);
                    gpio_put(MOSFET4, 0);
                    gpio_put(MOTOR_ENABLE, 1);
                    set_led(LED_ON, false);
                    break;
                    
                case BTN_MOTOR_TOGGLE: // I14 - temporário
                    sistema.motor_rodando_temp = !sistema.motor_rodando_temp;
                    if (!sistema.motor_rodando_temp) {
                        sistema.rpm_motor = 4;
                    }
                    break;
                    
                case BTN_AUMENTA_RPM: // I15 - temporário
                    if (sistema.motor_rodando_temp) {
                        sistema.rpm_motor++;
                    }
                    break;
            }
        }
        
        botoes_estado_anterior[i] = estado_atual;
    }
}

// Atualiza leitura dos sensores
void atualizar_sensores() {
    // Lê temperaturas
    sistema.temp_atual[0] = ler_temperatura_pt100(0);
    sistema.temp_atual[1] = ler_temperatura_pt100(1);
    sistema.temp_atual[2] = ler_temperatura_pt100(2);
    sistema.temp_atual[3] = ler_temperatura_pt100(3);
    
    // Lê nível de material
    float distancia = ler_distancia_ultrasonic();
    sistema.nivel_material = 100.0 - (distancia / 50.0 * 100.0); // Assumindo 50cm como máximo
    
    if (sistema.nivel_material < 10.0) {
        set_led(LED_MATERIAL_ACABOU, true);
    } else if (sistema.nivel_material < 20.0) {
        set_led(LED_MATERIAL_ACABANDO, true);
    } else {
        set_led(LED_MATERIAL_ACABOU, false);
        set_led(LED_MATERIAL_ACABANDO, false);
    }
}

// ==================== FUNÇÃO PRINCIPAL ====================

int main() {
    // Inicializa sistema
    stdio_init_all();
    
    // Aguarda estabilização
    sleep_ms(1000);
    
    printf("Inicializando Sistema de Controle de Extrusora...\n");
    
    // Inicializa hardware
    init_mux_pins();
    init_i2c_lcd();
    lcd_init();
    init_adc();
    init_mosfets();
    init_motor();
    init_ultrasonic();
    init_sistema();
    
    printf("Sistema inicializado com sucesso!\n");
    
    // Display inicial
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_string("Sistema Pronto");
    sleep_ms(2000);
    
    uint32_t last_update = time_us_32();
    uint32_t last_display_update = time_us_32();
    
    // Loop principal
    while (true) {
        uint32_t now = time_us_32();
        
        // Processa botões (a cada ciclo)
        processar_botoes();
        
        // Atualiza sensores (a cada 100ms)
        if ((now - last_update) > 100000) {
            atualizar_sensores();
            last_update = now;
        }
        
        // Controla temperatura e motores
        if (sistema.sistema_ativo && !sistema.erro_ativo) {
            controlar_temperatura();
            
            // Controle do motor (modo temporário com botões I14 e I15)
            if (sistema.motor_rodando_temp) {
                motor_step(sistema.rpm_motor);
            }
            
            // Controle normal do motor
            if (sistema.motor_ligado && sistema.extrusao_ativa && pode_ligar_motor()) {
                motor_step(10); // RPM padrão durante extrusão
            }
        }
        
        // Atualiza display (a cada 500ms)
        if ((now - last_display_update) > 500000) {
            atualizar_display();
            last_display_update = now;
        }
        
        sleep_ms(1); // Pequeno delay para não sobrecarregar
    }
    
    return 0;
}
