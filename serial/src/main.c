/*
 * Copyright (c) 2017 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/__assert.h>
#include <string.h>
#include <stdio.h>
#include <zephyr/drivers/uart.h>
#include <string.h>

/* change this to any other UART peripheral if desired */
#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)// define a UART




//K_MSGQ_DEFINE(uart_msgq, 32, 10, 4);// define a mensage queue (=fifo)

static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);


static char echo_buf[32];// declara o buffer que guarda os bits recebidos
static int echo_buf_pos;// declara a variável para posição do buffer

K_MSGQ_DEFINE (teclado_msgq, 128, 50, 4);// define a FIFO que envia dados

uint8_t buffer;// declara um buffer de 8 bits
int i=0;// contador para a posição dos bits a serem transmitidos
int f=0;// contador para 
int j=0;// contador para a posição dentro do vetor frase
int d=0;
int k=0;
int m=0;
int count_buf4 = 0;// contador para montar o buffer_bits4, o qual recebe 4 leituras do mesmo bit 
int recebendo = 0;// inteiro que quando maior que 0 indica que o rx está recebendo dados
int ordem = 0;// contador para localizar em qual parte do pacote está a recepção (ordem 0= u, ordem 1= sync...)
bool rx_exterior_flag = false;// flag para indicar se está recebendo uma mensagem de outro dispositivo
bool tx_flag = false;

int tamanhobits_tx; // declara uma variável inteira para armazenar o número total de bits da mensagem recebida
int tamanhobits_rx;// declara uma variável inteira para armazenar o número total de bits da mensagem recebida

uint8_t u = 0b01010101;// declara U
uint8_t sync = 0b00010110;// declara o sync
char frase[64] = {0};// declara um vetor para armazenar a mensagem
char frase_confirmação[7] = {0};
uint8_t cabecalho = 0b11010000; // declara o cabeçalho -> ID (11010) + tamanho da mensagem
uint8_t pos = 0b00010111;// declara pos, que indica o fim da transmissão
    
uint8_t u_rx = 0b01010101;// declara o U a ser recebido pelo RX
uint8_t sync_rx = 0b00010110;// declara o sync a ser recebido pelo RX
char frase_rx[8] = {0};// declara um vetor para armazenar a mensagem a ser recebida pelo RX 
uint8_t cabecalho_rx = 0b11010000;// declara parte do cabeçalho a ser recebido pelo RX (o ID)
uint8_t pos_rx = 0b00010111;// declara pos a ser recebido por RX, o qual indica o fim da transmissão
uint8_t buffer_bits4;// declara um inteiro de 8 bits para armazenar as 4 leituras de um mesmo bit realizadas na recepção

const struct device *sb = DEVICE_DT_GET(DT_NODELABEL(gpiob));// coleta informações do device tree sobre o gpiob e insere no ponteiro "sb"



K_SEM_DEFINE(semafaro_tx_transmitindo, 1, 1);// define o semáforo semaforo_tx_transmitindo




// inicio da transmissão
void tx(void){// função que realiza a transmissão 
    int aux=0;// declara auxiliar para 
    if (i==0){
        gpio_pin_set(sb, 0x3, 0);// coloca a gpio em 0
        if (k_sem_take(&semafaro_tx_transmitindo, K_NO_WAIT) == 0) {// se o semáforo semafaro_tx_transmitindo for obtido, então a transmissão começa
            if (k_msgq_get(&teclado_msgq, &frase, K_NO_WAIT) == 0) {// coloca as informações contidas em teclado_msgq no vetor frase
                cabecalho = cabecalho | strlen(frase);// altera o valor dos três últimos bits do cabeçalho para definir o tamanho em bytes da mensagem
                tamanhobits_tx = 8*(cabecalho & 0b111);// calcula o tamanho em bits da mensagem
                for (int g; g<7; g++) {
                    frase_confirmação[g] = frase[g];// preenche o vetor frase_confirmação com a mensagem
                }
                i += 1;// soma 1 no contador
            } else {
                k_sem_give(&semafaro_tx_transmitindo);// libera o semáforo semaforo_tx_transmitindo
            }
        }
    } else if (i == 1) {
        gpio_pin_set(sb, 0X3, 1);// coloca gpio em 1 (start bit)
        i += 1;// soma 1 no contador
    } 
    else if (i < 10) {// entra nessa condição até seja maior ou igual a 10
        aux = u & 0b1;// coloca em aux o valor do bit menos significativo de u
        u = u >> 1;// desloca "u" um bit para a direita para na próxima iteração pegar o próximo bit
        gpio_pin_set(sb, 0X3, aux);// coloca na gpio o mesmo valor obtido em aux
        i+=1;//soma 1 no somador 
    } else if (i < 18) {// entra nessa condição até seja maior ou igual a 18
        aux = sync & 0b1;// coloca em aux o valor do bit menos significativo de sync
        sync = sync >> 1;// desloca "sync" um bit para a direita para na próxima iteração pegar o próximo bit
        gpio_pin_set(sb, 0X3, aux);// coloca na gpio o mesmo valor obtido em aux
        i+=1;// soma 1 no contador i
    } else if (i < 26) {// entra nessa condição até seja maior ou igual a 26
        aux = cabecalho & 0b1;// coloca em aux o valor do bit menos significativo do cabeçalho
        cabecalho = cabecalho >> 1;// desloca "cabeçalho" um bit para a direita para na próxima iteração pegar o próximo bit
        gpio_pin_set(sb, 0X3, aux);// coloca na gpio o mesmo valor obtido em aux
        i+=1;// soma 1 no contador i
    } else if (i < (26 + tamanhobits_tx)) {// entra nessa condição até seja maior ou igual a 26 + tamanho da mensagem
        if (j!=(tamanhobits_tx/8)) {// entra nessa condição se j é diferente do tamanho da mensagem
            aux = frase[j] & 0b1;// coloca em aux o valor do bit menos significativo do byte da posição j
            frase[j] = frase[j] >> 1;// desloca 1 vez para a direita o byte da posição j
            gpio_pin_set(sb, 0X3, aux);// coloca na gpio o mesmo valor obtido em aux
            i+=1;// soma 1 no contador i
            k+=1;// soma 1 no contador k
        }
        if (k == 8) {// k=8 então o byte da posição j já foi lido
            k = 0;// zera o contador k
            j++;// soma 1 no contador j e vai para a próxima posição do vetor frase
        }
    } else if (i < (34 + tamanhobits_tx)) {// entra nessa condição até seja maior ou igual a 34 + tamanho da mensagem
        aux = pos & 0b1;// coloca em aux o valor do bit menos significativo de pos
        pos = pos >> 1;// desloca "pos" um bit para a direita para na próxima iteração pegar o próximo bit
        gpio_pin_set(sb, 0X3, aux);// coloca na gpio o mesmo valor obtido em aux
        i+=1;// soma 1 no contador i
    } else {
        i=0;// reseta o contador i
        j=0;// reseta o contador j
        k=0;// reseta o contador k
        //gpio_pin_set(sb, 0X3, 0);
        u = 0b01010101;// regenera u
        sync = 0b00010110;// regenera sync
        cabecalho = 0b11010000; // regenera cabecalho
        pos = 0b00010111;// regenera pos
        // fim da transmissão
    }

}


K_TIMER_DEFINE(tx_call, tx, NULL);// define o timer tx_call, que será atrelado a função tx


// função de callback para interpretar os dados do teclado 
void serial_cb(const struct device *dev, void *user_data)
{
    uint8_t c;// declara um inteiro de 8 bits para receber dados do teclado
   
    if (!uart_irq_update(uart_dev)) {
        return;
    }

    if (!uart_irq_rx_ready(uart_dev)) {
        return;
    }

    
    while (uart_fifo_read(uart_dev, &c, 1) == 1) {// enquanto houver algo na uart entra na condição
        if ((c == '\n' || c == '\r') && echo_buf_pos > 0) {// entra nessa condição quando a mensagem acabou e o "enter" foi apertado
            
            echo_buf[echo_buf_pos] = '\0';// coloca um espaço vazio na próxima posição em relação ao último caractere escrito

            for (int t = 0; t < (strlen(echo_buf) / 7) + (strlen(echo_buf) % 7 != 0); t++) { 
                char echo_buf_7[8] = {0}; // Zera o buffer para cada segmento
                // Verifica se é o último segmento incompleto
                if (t == (strlen(echo_buf) / 7) && strlen(echo_buf) % 7 != 0) {
                    // Copia os caracteres restantes
                    for (int y = 0; y < strlen(echo_buf) % 7; y++) {
                        echo_buf_7[y] = echo_buf[(t * 7) + y];
                    }
                } else {
                    // Copia um bloco completo de 7 caracteres
                    for (int y = 0; y < 7; y++) {
                        echo_buf_7[y] = echo_buf[(t * 7) + y];
                    }
                }
                // Envia o bloco de caracteres para a mensagem
                k_msgq_put(&teclado_msgq, &echo_buf_7, K_NO_WAIT);
            }

            echo_buf_pos = 0;// reseta o contador da posição do echo_buf
        } else if (echo_buf_pos < (sizeof(echo_buf) - 1)) {// entra nessa condição enquanto o enter ainda não foi apertado
            echo_buf[echo_buf_pos++] = c;// coloca o caractere digitado no char echo_buff e adiciona 1 no contador echo_buf_pos
            
        }
        
    }
}

// função para imprimir na tela as mensagens recebidas pelo RX 
void mensagem() {
    if(rx_exterior_flag){// entra nessa condição se a mensagem vem de outro dispositivo
    printk("Você recebeu uma mensagem:\n");}
    else {// entra nessa condição se a mensagem foi enviada pelo próprio dispositivo
        printk("você mandou essa mensagem:\n");}
    
    for (int s = 0; s < strlen(frase_rx); s++) {// imprime a mensagem na tela
        printk("%c", frase_rx[s]);
    }
    printk("\n");
}


void rx_buffer(void) {// função que recebe as informações do pino 2 da gpio B
    buffer_bits4 = (buffer_bits4 >> 1) | (gpio_pin_get(sb,0x2) << 7);// monta o buffer_bits4 que contém as 4 leituras de um mesmo bit recebido pelo RX através no pino 2 na gpio B
    // entra nessa condição se for reconhecido o start bit (1)
    if ((((buffer_bits4 & 0b11100000) == 0b11100000) || ((buffer_bits4 & 0b01110000) == 0b01110000) || ((buffer_bits4 & 0b11110000) == 0b11110000)) && (recebendo == 0)) {
        recebendo++;// soma 1 no contador recebendo
        count_buf4 = 0;
        f = 0;
        memset(frase_rx, 0, sizeof(frase_rx));// reseta o vetor frase_rx
        // entra nessa condição quando consegue pegar o semaforo semafaro_tx_transmitindo
        if (k_sem_take(&semafaro_tx_transmitindo, K_NO_WAIT) == 0) {
            rx_exterior_flag = true;// indica que a mensagem está sendo recebida de outro tx
            k_timer_stop(&tx_call);// bloqueia a ação do transmissor
        } else {
            rx_exterior_flag = false;// indica que a mensagem esta sendo recebida do proprio tx
        }
    }
    if (recebendo > 0) {// entra nessa condição depois de reconhecer o start bit
        // entra nessa condição se os dois bits das posições 2 e 3 do buffer_bits4 são iguais a 1
        if (((buffer_bits4 & 0b01100000) == 0b01100000) && (count_buf4%4 == 0)) {
            buffer = (buffer >> 1) | 0b10000000;// adiciona o bit 1 no buffer e o desloca um bit para a direita
            count_buf4 = 0;// reinicia o contador do buffer_bits4
            rx();// chama a função rx

            // entra nessa condição se os dois bits das posições 2 e 3 são iguais a 0
        } else if (((buffer_bits4 & 0b01100000) == 0b00000000) && (count_buf4%4 == 0)) {
            buffer = (buffer >> 1);// adiciona o bit 0 no buffer e o desloca um bit para a direita
            count_buf4 = 0;// reinicia o contador do buffer_bits4
            rx();// chama a função rx
        }
        count_buf4++;// adiciona 1 no contador count_buf4
    }
}
// função que realiza a recepção 
void rx(void) {
    if((f < 8) && (ordem == 0)) {// entra nessa condição enquanto o buffer está recebendo U
        f++;
    } else if(ordem == 0) {
        if ((buffer ^ u_rx) == 0) {// entra nessa condição quando o buffer é igual a u_rx
            ordem++;// soma 1 no contador "ordem" para ir para a próxima parte da recepção
            f++;
           
        } else {// se o buffer não é igual a u_rx, então a transmissão reinicia
            recebendo = 0;
            ordem = 0;
            buffer = 0;
            if (rx_exterior_flag) {// se a recepção é de um dispositivo externo, então o tx executa novamente
                k_sem_give(&semafaro_tx_transmitindo);
                k_timer_start(&tx_call, K_NO_WAIT, K_MSEC(40));
            } else {// se a mensagem é do próprio dispositivo, então o tx é bloqueado, renicializado e desbloqueado novamente 
                k_timer_stop(&tx_call);
                i = 1;
                for (int g; g<7; g++) {
                    frase[g] = frase_confirmação[g];
                }
                j=0;
                k=0;
                u = 0b01010101;
                sync = 0b00010110;
                cabecalho = (0b11010000 |strlen(frase)); 
                pos = 0b00010111;
                k_timer_start(&tx_call, K_NO_WAIT, K_MSEC(40));
            }
        }
    } else if (f < 16) {// entra nessa condição enquanto o buffer está recebendo sync_rx
        f++;
    } else if(ordem == 1) {
        if ((buffer ^ sync_rx) ==0 ) {// entra nessa condição quando o buffer é igual a sync_rx
            ordem++;// soma 1 no contador "ordem" para ir para a próxima parte da recepção
            f++;
            
        } else {// se o buffer não é igual a sync_rx, então a transmissão reinicia
            recebendo = 0;
            ordem = 0;
            buffer = 0;
            if (rx_exterior_flag) {// se a recepção é de um dispositivo externo, então o tx executa novamente
                k_sem_give(&semafaro_tx_transmitindo);
                k_timer_start(&tx_call, K_NO_WAIT, K_MSEC(40));
            } else {// se a mensagem é do próprio dispositivo, então o tx é bloqueado, renicializado e desbloqueado novamente
                k_timer_stop(&tx_call);
                i = 1;
                for (int g; g<7; g++) {
                    frase[g] = frase_confirmação[g];
                }
                j=0;
                k=0;
                u = 0b01010101;
                sync = 0b00010110;
                cabecalho = (0b11010000 |strlen(frase)); 
                pos = 0b00010111;
                k_timer_start(&tx_call, K_NO_WAIT, K_MSEC(40));
                
            }
        }
    } else if (f < 24) {// entra nessa condição enquanto o buffer está recebendo cabecalho_rx
        f++;
    } else if(ordem == 2) {
        if (((buffer & 0b11111000) ^ (cabecalho_rx)) == 0) {// entra nessa condição quando o buffer é igual a cabecalho_rx
            ordem++;// soma 1 no contador "ordem" para ir para a próxima parte da recepção
            f++;
            tamanhobits_rx = (buffer & 0b00000111)*8;
            
        } else {// se o buffer não é igual a cabecalho_rx, então a transmissão reinicia
            recebendo = 0;
            ordem = 0;
            buffer = 0;
            if (rx_exterior_flag) {// se a recepção é de um dispositivo externo, então o tx executa novamente
                k_sem_give(&semafaro_tx_transmitindo);
                k_timer_start(&tx_call, K_NO_WAIT, K_MSEC(40));
            } else {// se a mensagem é do próprio dispositivo, então o tx é bloqueado, renicializado e desbloqueado novamente
                k_timer_stop(&tx_call);
                i = 1;
                for (int g; g<7; g++) {
                    frase[g] = frase_confirmação[g];
                }
                j=0;
                k=0;
                u = 0b01010101;
                sync = 0b00010110;
                cabecalho = (0b11010000 |strlen(frase)); 
                pos = 0b00010111;
                k_timer_start(&tx_call, K_NO_WAIT, K_MSEC(40));
            }
        }
    } else if(ordem == 3) {
            if((f % 8 == 0) && (d != (tamanhobits_rx/8))){// entra nessa condição enquanto o buffer está recebendo a mensagem
                frase_rx[d] = buffer;// armazena em frase_rx a mensagem transmitida
                 d++;
            }
            if(d == (tamanhobits_rx/8)){
                ordem++;
                d = 0;
            }
            f++; 
    } else if (f < (32 + tamanhobits_rx)) {// entra nessa condição enquanto o buffer está recebendo o pos_rx
        f++;
    } else if(ordem == 4) {
        if ((buffer ^ pos_rx) == 0) {// entra nessa condição quando o buffer é igual a pos_rx
        
            recebendo = 0;// zera o contador "recebendo"
            ordem = 0;// reinicia o contador "ordem"
            buffer = 0b0;// zera o buffer
            mensagem();// chama a função mensagem que irá imprimir na tela a mensagem
            k_sem_give(&semafaro_tx_transmitindo);// libera o semáforo
        } else {// se o buffer não é igual a pos_rx, então a transmissão reinicia
            recebendo = 0;
            ordem = 0;
            buffer = 0;
            if (rx_exterior_flag) {// se a recepção é de um dispositivo externo, então o tx executa novamente
                k_sem_give(&semafaro_tx_transmitindo);
                k_timer_start(&tx_call, K_NO_WAIT, K_MSEC(40));
            } else {// se a mensagem é do próprio dispositivo, então o tx é bloqueado, renicializado e desbloqueado novamente
                k_timer_stop(&tx_call);
                i = 1;
                for (int g; g<7; g++) {
                    frase[g] = frase_confirmação[g];
                }
                j=0;
                k=0;
                u = 0b01010101;
                sync = 0b00010110;
                cabecalho = (0b11010000 |strlen(frase)); 
                pos = 0b00010111;
                k_timer_start(&tx_call, K_NO_WAIT, K_MSEC(40));
            }
        }
    } else {
        recebendo = 0;
        ordem = 0;
        if (rx_exterior_flag) {
            k_sem_give(&semafaro_tx_transmitindo);
            k_timer_start(&tx_call, K_NO_WAIT, K_MSEC(40));
        } else {
            k_timer_stop(&tx_call);
            i = 1;
            for (int g; g<7; g++) {
                frase[g] = frase_confirmação[g];
            }
            j=0;
            k=0;
            u = 0b01010101;
            sync = 0b00010110;
            cabecalho = (0b11010000 |strlen(frase)); 
            pos = 0b00010111;
            k_timer_start(&tx_call, K_NO_WAIT, K_MSEC(40));
        }
    }
    
}

K_TIMER_DEFINE(rx_buffer_call, rx_buffer, NULL);// define o timer rx_buffer_call  


void main(void) {
    k_timer_start(&rx_buffer_call, K_NO_WAIT, K_MSEC(10));// aciona o timer rx_buffer_call com intervalo de chamada de 10ms
    k_timer_start(&tx_call, K_NO_WAIT, K_MSEC(40));// aciona o timer tx_call com intervalo de chamada de 40ms
    gpio_pin_configure(sb, 0x3, GPIO_OUTPUT_LOW);// configura o PTB3 como pino de saida
    gpio_pin_configure(sb, 0x2, GPIO_INPUT| GPIO_ACTIVE_HIGH);// configura o PTB2 como pino de entrada
    uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);// configura serial_cb como função de callback, sempre que há dados novos na uart serial_cb é executada
    uart_irq_rx_enable(uart_dev);// habilita a uart
    printk("Digite suas mensagens (máximo de 7 caracter por vez)\n:");
}