#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"

// Definições dos pinos
#define BUTTON_PIN 22 // Pino GPIO conectado ao botão do joystick
#define I2C_SDA 14    // Pino SDA do I2C
#define I2C_SCL 15    // Pino SCL do I2C
#define OLED_ADDR 0x3C // Endereço I2C do display OLED

// Variáveis globais
uint16_t amplitude = 0;   // Valor da amplitude
uint16_t frequencia = 0;  // Valor da frequência
bool locked = false;      // Estado de trava dos valores

// Matriz de fontes simplificada (apenas caracteres necessários)
const uint8_t font[59][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, // Espaço (32)
    {0x00, 0x00, 0x5F, 0x00, 0x00}, // !
    {0x00, 0x07, 0x00, 0x07, 0x00}, // "
    {0x14, 0x7F, 0x14, 0x7F, 0x14}, // #
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, // $
    {0x23, 0x13, 0x08, 0x64, 0x62}, // %
    {0x36, 0x49, 0x55, 0x22, 0x50}, // &
    {0x00, 0x05, 0x03, 0x00, 0x00}, // '
    {0x00, 0x1C, 0x22, 0x41, 0x00}, // (
    {0x00, 0x41, 0x22, 0x1C, 0x00}, // )
    {0x08, 0x2A, 0x1C, 0x2A, 0x08}, // *
    {0x08, 0x08, 0x3E, 0x08, 0x08}, // +
    {0x00, 0x50, 0x30, 0x00, 0x00}, // ,
    {0x08, 0x08, 0x08, 0x08, 0x08}, // -
    {0x00, 0x60, 0x60, 0x00, 0x00}, // .
    {0x20, 0x10, 0x08, 0x04, 0x02}, // /
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 0
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // 1
    {0x42, 0x61, 0x51, 0x49, 0x46}, // 2
    {0x21, 0x41, 0x45, 0x4B, 0x31}, // 3
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // 4
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 6
    {0x01, 0x71, 0x09, 0x05, 0x03}, // 7
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 8
    {0x06, 0x49, 0x49, 0x29, 0x1E}, // 9
    {0x00, 0x36, 0x36, 0x00, 0x00}, // :
    {0x00, 0x56, 0x36, 0x00, 0x00}, // ;
    {0x08, 0x14, 0x22, 0x41, 0x00}, // <
    {0x14, 0x14, 0x14, 0x14, 0x14}, // =
    {0x00, 0x41, 0x22, 0x14, 0x08}, // >
    {0x02, 0x01, 0x51, 0x09, 0x06}, // ?
    {0x32, 0x49, 0x79, 0x41, 0x3E}, // @
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, // A
    {0x7F, 0x49, 0x49, 0x49, 0x36}, // B
    {0x3E, 0x41, 0x41, 0x41, 0x22}, // C
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, // D
    {0x7F, 0x49, 0x49, 0x49, 0x41}, // E
    {0x7F, 0x09, 0x09, 0x09, 0x01}, // F
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, // G
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, // H
    {0x00, 0x41, 0x7F, 0x41, 0x00}, // I
    {0x20, 0x40, 0x41, 0x3F, 0x01}, // J
    {0x7F, 0x08, 0x14, 0x22, 0x41}, // K
    {0x7F, 0x40, 0x40, 0x40, 0x40}, // L
    {0x7F, 0x02, 0x04, 0x02, 0x7F}, // M
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, // N
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, // O
    {0x7F, 0x09, 0x09, 0x09, 0x06}, // P
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, // Q
    {0x7F, 0x09, 0x19, 0x29, 0x46}, // R
    {0x46, 0x49, 0x49, 0x49, 0x31}, // S
    {0x01, 0x01, 0x7F, 0x01, 0x01}, // T
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, // U
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, // V
    {0x7F, 0x20, 0x18, 0x20, 0x7F}, // W
    {0x63, 0x14, 0x08, 0x14, 0x63}, // X
    {0x03, 0x04, 0x78, 0x04, 0x03}, // Y
    {0x61, 0x51, 0x49, 0x45, 0x43}, // Z
};

// Função para enviar um comando ao display OLED
void oled_send_command(uint8_t command) {
    uint8_t buf[2] = {0x00, command}; // 0x00 é o byte de controle para comandos
    i2c_write_blocking(i2c1, OLED_ADDR, buf, 2, false);
}

// Função para inicializar o display OLED
void oled_init() {
    // Inicializa o I2C
    i2c_init(i2c1, 400 * 1000); // 400 kHz
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Sequência de inicialização do SSD1306
    oled_send_command(0xAE); // Desliga o display
    oled_send_command(0xD5); // Configura o clock do display
    oled_send_command(0x80); // Valor recomendado no datasheet
    oled_send_command(0xA8); // Configura a multiplexação
    oled_send_command(0x3F); // Para displays de 128x64
    oled_send_command(0xD3); // Configura o offset do display
    oled_send_command(0x00); // Sem offset
    oled_send_command(0x40); // Configura a linha inicial
    oled_send_command(0x8D); // Configura o charge pump
    oled_send_command(0x14); // Habilita o charge pump
    oled_send_command(0x20); // Configura o modo de endereçamento
    oled_send_command(0x00); // Modo horizontal
    oled_send_command(0xA1); // Configura a orientação do segmento
    oled_send_command(0xC8); // Configura a orientação do COM
    oled_send_command(0xDA); // Configura os pinos de hardware
    oled_send_command(0x12); // Para displays de 128x64
    oled_send_command(0x81); // Configura o contraste
    oled_send_command(0xCF); // Valor do contraste
    oled_send_command(0xD9); // Configura o pré-carregamento
    oled_send_command(0xF1); // Fase 1 e 2
    oled_send_command(0xDB); // Configura a tensão VCOMH
    oled_send_command(0x40); // Valor recomendado
    oled_send_command(0xA4); // Habilita o display RAM
    oled_send_command(0xA6); // Configura o modo de exibição normal
    oled_send_command(0xAF); // Liga o display
}

// Função para limpar o display
void oled_clear() {
    uint8_t buf[129];
    buf[0] = 0x40; // Byte de controle para dados
    for (int i = 1; i < 129; i++) buf[i] = 0x00; // Preenche com zeros

    for (int page = 0; page < 8; page++) {
        oled_send_command(0xB0 + page); // Define a página
        oled_send_command(0x00);        // Coluna baixa
        oled_send_command(0x10);        // Coluna alta
        i2c_write_blocking(i2c1, OLED_ADDR, buf, 129, false);
    }
}

// Função para exibir uma string no display
void oled_draw_string(uint8_t x, uint8_t y, const char *str) {
    while (*str) {
        // Define a posição
        oled_send_command(0xB0 + y); // Define a página
        oled_send_command(0x00 + (x & 0x0F)); // Coluna baixa
        oled_send_command(0x10 + ((x >> 4) & 0x0F)); // Coluna alta

        // Envia os dados do caractere
        uint8_t buf[6];
        buf[0] = 0x40; // Byte de controle para dados
        for (int i = 0; i < 5; i++) {
            buf[i + 1] = font[*str - 32][i]; // Usa a fonte simplificada
        }
        i2c_write_blocking(i2c1, OLED_ADDR, buf, 6, false);

        x += 6; // Avança para o próximo caractere
        str++;
    }
}

// Função para inicializar o joystick
void init_joystick() {
    adc_init();
    adc_gpio_init(26);  // Eixo X (GPIO 26)
    adc_gpio_init(27);  // Eixo Y (GPIO 27)
    gpio_init(BUTTON_PIN);
    gpio_set_dir(BUTTON_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_PIN);
}

// Função para atualizar os valores com base no joystick
void update_values() {
    if (!locked) {
        // Leitura do eixo Y (Amplitude)
        adc_select_input(0); // Seleciona o canal ADC 0 (GPIO 26)
        uint adc_y_raw = adc_read();
        if (adc_y_raw > 2500) amplitude++; // Incrementa se o joystick estiver para cima
        else if (adc_y_raw < 1500) amplitude--; // Decrementa se o joystick estiver para baixo

        // Leitura do eixo X (Frequência)
        adc_select_input(1); // Seleciona o canal ADC 1 (GPIO 27)
        uint adc_x_raw = adc_read();
        if (adc_x_raw > 2500) frequencia++; // Incrementa se o joystick estiver para a direita
        else if (adc_x_raw < 1500) frequencia--; // Decrementa se o joystick estiver para a esquerda

        // Limita os valores para evitar overflow
        if (amplitude > 1023) amplitude = 1023;
        if (amplitude < 0) amplitude = 0;
        if (frequencia > 1023) frequencia = 1023;
        if (frequencia < 0) frequencia = 0;
    }
}

// Função para exibir os valores no display OLED
void display_values() {
    char text[32];
    snprintf(text, sizeof(text), "Amplitude: %3u", amplitude);
    oled_draw_string(0, 0, text);
    snprintf(text, sizeof(text), "Frequencia: %3u", frequencia);
    oled_draw_string(0, 1, text);
}

int main() {
    // Inicializa as interfaces
    stdio_init_all();
    init_joystick();
    oled_init();
    oled_clear();

    // Loop principal
    while (true) {
        // Verifica se o botão foi pressionado para travar/destravar
        if (!gpio_get(BUTTON_PIN)) {
            locked = !locked; // Alterna o estado de trava
            sleep_ms(300); // Debounce simples
        }

        // Atualiza os valores do joystick
        update_values();

        // Exibe os valores no display
        display_values();

        // Pausa para evitar leituras muito rápidas
        sleep_ms(50);
    }

    return 0;
}
