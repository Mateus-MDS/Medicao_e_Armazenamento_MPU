/*
 * ================================================================================
 * SISTEMA DE COLETA DE DADOS MPU6050 COM RASPBERRY PI PICO
 * ================================================================================
 * 
 * Descri��o: Sistema embarcado para coleta cont�nua de dados do sensor MPU6050
 *            com armazenamento em cart�o SD e interface OLED para visualiza��o
 * ================================================================================
 */

// ================================================================================
// INCLUDES - BIBLIOTECAS NECESS�RIAS
// ================================================================================

#include <ctype.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

// Bibliotecas do Pico SDK
#include "hardware/adc.h"
#include "hardware/rtc.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "math.h"

// Bibliotecas espec�ficas do projeto
#include "ssd1306.h"      // Driver do display OLED
#include "font.h"         // Fontes para o display
#include "MPU6050.h"      // Driver do sensor MPU6050

// Bibliotecas para SD Card (FatFS)
#include "ff.h"
#include "diskio.h"
#include "f_util.h"
#include "hw_config.h"
#include "my_debug.h"
#include "rtc.h"
#include "sd_card.h"

// ================================================================================
// DEFINI��ES DE HARDWARE - MAPEAMENTO DOS PINOS
// ================================================================================

// Configura��o I2C para MPU6050
#define I2C_PORT i2c0                 // I2C0 usa pinos 0 e 1
#define I2C_SDA 0                     // Pino SDA do MPU6050
#define I2C_SCL 1                     // Pino SCL do MPU6050

// Configura��o I2C para Display OLED
#define I2C_PORT_DISP i2c1            // I2C1 para display
#define I2C_SDA_DISP 14               // Pino SDA do display
#define I2C_SCL_DISP 15               // Pino SCL do display
#define ENDERECO_DISP 0x3C            // Endere�o I2C do display SSD1306

// Defini��o dos LEDs indicadores
#define Led_verde 11                  // LED verde - sistema OK
#define Led_azul 12                   // LED azul - opera��es de leitura
#define Led_vermelho 13               // LED vermelho - erro/captura

// Perif�ricos de interface
#define buzzer 21                     // Buzzer para sinais sonoros
#define botaoA 5                      // Bot�o A - controle de captura
#define botaoB 6                      // Bot�o B - controle de SD

// ================================================================================
// CONSTANTES E CONFIGURA��ES DO SISTEMA
// ================================================================================

// Configura��es do MPU6050
static int addr = 0x68;               // Endere�o padr�o do MPU6050

// Configura��es de logging do MPU6050
static const uint32_t mpu_log_period = 100;  // 100ms = 10Hz
static char mpu_filename[20] = "mpu_data2.csv"; // Nome do arquivo CSV

// Configura��es gerais de logging
static const uint32_t period = 1000;  // Per�odo geral de 1 segundo

// ================================================================================
// VARI�VEIS GLOBAIS - CONTROLE DE ESTADO DO SISTEMA
// ================================================================================

// Controle de estado principal
char Estado = 'L';                    // Estado atual do sistema
char estado_fut = 'L';                // Estado futuro para compara��o

// Estados dos LEDs
bool Estado_led_verde = false;
bool Estado_led_azul = false;
bool Estado_led_vermelho = false;

// Estados de opera��o
bool Estado_coleta_dados = false;     // Flag de coleta de dados ativa
bool Estado_montar_cartao = false;   // Flag de cart�o SD montado

// Estados anteriores para detec��o de mudan�as
bool Estado_coleta_dados_prev = false;
bool Estado_montar_cartao_prev = false;

// Vari�veis de controle do logging MPU6050
static bool mpu_logging_enabled = false;      // Flag de logging ativo
static absolute_time_t next_mpu_log_time;     // Pr�ximo tempo de log
static FIL mpu_file;                          // Handle do arquivo CSV
static uint32_t sample_counter = 0;           // Contador de amostras

// Vari�veis de controle geral
static bool logger_enabled;
static absolute_time_t next_log_time;

// ================================================================================
// ENUMERA��ES - DEFINI��O DE TIPOS
// ================================================================================

/**
 * Enumera os diferentes tipos de eventos sonoros do buzzer
 * Cada evento tem um padr�o espec�fico de bipes
 */
typedef enum {
    Som_desmontando,        // 2 bipes curtos - sistema iniciado
    Som_montando,           // 2 bipes curtos - cart�o montado
    Som_iniciando_captura,  // 1 bipe longo - in�cio da captura
    Som_encerrando_captura, // 2 bipes (curto + longo) - fim da captura
    som_leitura,           // 3 bipes curtos - leitura de dados
    Som_erro               // 3 bipes longos - erro no sistema
} Evento_buzzer;

// ================================================================================
// ESTRUTURAS E TIPOS - DEFINI��ES DE COMANDOS
// ================================================================================

typedef void (*p_fn_t)();

/**
 * Estrutura para defini��o de comandos do terminal
 * Mapeia strings de comando para suas respectivas fun��es
 */
typedef struct
{
    char const *const command;         // String do comando
    p_fn_t const function;            // Ponteiro para fun��o
    char const *const help;           // Texto de ajuda
} cmd_def_t;

// ================================================================================
// FUN��ES AUXILIARES - GERENCIAMENTO DE SD CARD
// ================================================================================

/**
 * Busca um cart�o SD pelo nome
 * @param name Nome do cart�o a ser buscado
 * @return Ponteiro para estrutura sd_card_t ou NULL se n�o encontrado
 */
static sd_card_t *sd_get_by_name(const char *const name)
{
    for (size_t i = 0; i < sd_get_num(); ++i)
        if (0 == strcmp(sd_get_by_num(i)->pcName, name))
            return sd_get_by_num(i);
    DBG_PRINTF("%s: unknown name %s\n", __func__, name);
    return NULL;
}

/**
 * Busca o sistema de arquivos de um cart�o SD pelo nome
 * @param name Nome do cart�o
 * @return Ponteiro para estrutura FATFS ou NULL se n�o encontrado
 */
static FATFS *sd_get_fs_by_name(const char *name)
{
    for (size_t i = 0; i < sd_get_num(); ++i)
        if (0 == strcmp(sd_get_by_num(i)->pcName, name))
            return &sd_get_by_num(i)->fatfs;
    DBG_PRINTF("%s: unknown name %s\n", __func__, name);
    return NULL;
}

// ================================================================================
// FUN��ES DE COMANDO - INTERFACE DO TERMINAL
// ================================================================================

/**
 * Configura o RTC (Real Time Clock) do sistema
 * Formato: setrtc DD MM YY hh mm ss
 */
static void run_setrtc()
{
    // Parsing dos argumentos de data e hora
    const char *dateStr = strtok(NULL, " ");
    if (!dateStr)
    {
        printf("Missing argument\n");
        return;
    }
    int date = atoi(dateStr);

    const char *monthStr = strtok(NULL, " ");
    if (!monthStr)
    {
        printf("Missing argument\n");
        return;
    }
    int month = atoi(monthStr);

    const char *yearStr = strtok(NULL, " ");
    if (!yearStr)
    {
        printf("Missing argument\n");
        return;
    }
    int year = atoi(yearStr) + 2000;

    const char *hourStr = strtok(NULL, " ");
    if (!hourStr)
    {
        printf("Missing argument\n");
        return;
    }
    int hour = atoi(hourStr);

    const char *minStr = strtok(NULL, " ");
    if (!minStr)
    {
        printf("Missing argument\n");
        return;
    }
    int min = atoi(minStr);

    const char *secStr = strtok(NULL, " ");
    if (!secStr)
    {
        printf("Missing argument\n");
        return;
    }
    int sec = atoi(secStr);

    // Configura��o da estrutura datetime_t
    datetime_t t = {
        .year = (int16_t)year,
        .month = (int8_t)month,
        .day = (int8_t)date,
        .dotw = 0, // 0 is Sunday
        .hour = (int8_t)hour,
        .min = (int8_t)min,
        .sec = (int8_t)sec};
    
    // Aplica a configura��o no RTC
    rtc_set_datetime(&t);
}

/**
 * Formata o cart�o SD com sistema de arquivos FAT
 */
static void run_format()
{
    const char *arg1 = strtok(NULL, " ");
    if (!arg1)
        arg1 = sd_get_by_num(0)->pcName;
    
    FATFS *p_fs = sd_get_fs_by_name(arg1);
    if (!p_fs)
    {
        printf("Unknown logical drive number: \"%s\"\n", arg1);
        return;
    }
    
    /* Formata o drive com par�metros padr�o */
    FRESULT fr = f_mkfs(arg1, 0, 0, FF_MAX_SS * 2);
    if (FR_OK != fr)
        printf("f_mkfs error: %s (%d)\n", FRESULT_str(fr), fr);
}

/**
 * Monta o cart�o SD no sistema de arquivos
 */
static void run_mount()
{
    const char *arg1 = strtok(NULL, " ");
    if (!arg1)
        arg1 = sd_get_by_num(0)->pcName;
    
    FATFS *p_fs = sd_get_fs_by_name(arg1);
    if (!p_fs)
    {
        printf("Unknown logical drive number: \"%s\"\n", arg1);
        Estado = 'E';  // Define estado de erro
        return;
    }
    
    FRESULT fr = f_mount(p_fs, arg1, 1);
    if (FR_OK != fr)
    {
        printf("f_mount error: %s (%d)\n", FRESULT_str(fr), fr);
        Estado = 'E';  // Define estado de erro
        return;
    }
    
    // Marca o cart�o como montado
    sd_card_t *pSD = sd_get_by_name(arg1);
    myASSERT(pSD);
    pSD->mounted = true;
    printf("Processo de montagem do SD ( %s ) conclu�do\n", pSD->pcName);
}

/**
 * Desmonta o cart�o SD do sistema
 */
static void run_unmount()
{
    const char *arg1 = strtok(NULL, " ");
    if (!arg1)
        arg1 = sd_get_by_num(0)->pcName;
    
    FATFS *p_fs = sd_get_fs_by_name(arg1);
    if (!p_fs)
    {
        printf("Unknown logical drive number: \"%s\"\n", arg1);
        return;
    }
    
    FRESULT fr = f_unmount(arg1);
    if (FR_OK != fr)
    {
        printf("f_unmount error: %s (%d)\n", FRESULT_str(fr), fr);
        Estado = 'E';  // Define estado de erro
        return;
    }
    
    // Marca o cart�o como desmontado
    sd_card_t *pSD = sd_get_by_name(arg1);
    myASSERT(pSD);
    pSD->mounted = false;
    pSD->m_Status |= STA_NOINIT; // caso o meio seja removido
    printf("SD ( %s ) desmontado\n", pSD->pcName);
}

/**
 * Obt�m informa��es de espa�o livre no cart�o SD
 */
static void run_getfree()
{
    const char *arg1 = strtok(NULL, " ");
    if (!arg1)
        arg1 = sd_get_by_num(0)->pcName;
    
    DWORD fre_clust, fre_sect, tot_sect;
    FATFS *p_fs = sd_get_fs_by_name(arg1);
    if (!p_fs)
    {
        printf("Unknown logical drive number: \"%s\"\n", arg1);
        return;
    }
    
    FRESULT fr = f_getfree(arg1, &fre_clust, &p_fs);
    if (FR_OK != fr)
    {
        printf("f_getfree error: %s (%d)\n", FRESULT_str(fr), fr);
        Estado = 'E';  // Define estado de erro
        return;
    }
    
    // Calcula espa�o total e livre
    tot_sect = (p_fs->n_fatent - 2) * p_fs->csize;
    fre_sect = fre_clust * p_fs->csize;
    printf("%10lu KiB total drive space.\n%10lu KiB available.\n", tot_sect / 2, fre_sect / 2);
}

/**
 * Lista arquivos e diret�rios no cart�o SD
 */
static void run_ls()
{
    const char *arg1 = strtok(NULL, " ");
    if (!arg1)
        arg1 = "";
    
    char cwdbuf[FF_LFN_BUF] = {0};
    FRESULT fr;
    char const *p_dir;
    
    if (arg1[0])
    {
        p_dir = arg1;
    }
    else
    {
        // Obt�m diret�rio atual se n�o especificado
        fr = f_getcwd(cwdbuf, sizeof cwdbuf);
        if (FR_OK != fr)
        {
            printf("f_getcwd error: %s (%d)\n", FRESULT_str(fr), fr);
            Estado = 'E';
            return;
        }
        p_dir = cwdbuf;
    }
    
    printf("Directory Listing: %s\n", p_dir);
    DIR dj;
    FILINFO fno;
    memset(&dj, 0, sizeof dj);
    memset(&fno, 0, sizeof fno);
    
    // Busca primeiro arquivo/diret�rio
    fr = f_findfirst(&dj, &fno, p_dir, "*");
    if (FR_OK != fr)
    {
        printf("f_findfirst error: %s (%d)\n", FRESULT_str(fr), fr);
        Estado = 'E';
        return;
    }
    
    // Lista todos os arquivos encontrados
    while (fr == FR_OK && fno.fname[0])
    {
        const char *pcWritableFile = "writable file",
                   *pcReadOnlyFile = "read only file",
                   *pcDirectory = "directory";
        const char *pcAttrib;
        
        // Determina o tipo do item
        if (fno.fattrib & AM_DIR)
            pcAttrib = pcDirectory;
        else if (fno.fattrib & AM_RDO)
            pcAttrib = pcReadOnlyFile;
        else
            pcAttrib = pcWritableFile;
        
        printf("%s [%s] [size=%llu]\n", fno.fname, pcAttrib, fno.fsize);
        fr = f_findnext(&dj, &fno);
    }
    f_closedir(&dj);
}

/**
 * Exibe o conte�do de um arquivo (comando cat)
 */
static void run_cat()
{
    char *arg1 = strtok(NULL, " ");
    if (!arg1)
    {
        printf("Missing argument\n");
        return;
    }
    
    FIL fil;
    FRESULT fr = f_open(&fil, arg1, FA_READ);
    if (FR_OK != fr)
    {
        printf("f_open error: %s (%d)\n", FRESULT_str(fr), fr);
        Estado = 'E';
        return;
    }
    
    // L� e exibe o arquivo linha por linha
    char buf[256];
    while (f_gets(buf, sizeof buf, &fil))
    {
        printf("%s", buf);
    }
    
    fr = f_close(&fil);
    if (FR_OK != fr)
        printf("f_open error: %s (%d)\n", FRESULT_str(fr), fr);
        Estado = 'E';
}

/**
 * Exibe a lista de comandos dispon�veis
 */
static void run_help()
{
    printf("\nComandos dispon�veis:\n\n");
    printf("Digite 'a' para montar o cart�o SD\n");
    printf("Digite 'b' para desmontar o cart�o SD\n");
    printf("Digite 'c' para listar arquivos\n");
    printf("Digite 'd' para mostrar conte�do do arquivo\n");
    printf("Digite 'e' para obter espa�o livre no cart�o SD\n");
    printf("Digite 'f' para formatar o cart�o SD\n");
    printf("Digite 'g' para exibir os comandos dispon�veis\n");
    printf("Digite 'h' para INICIAR captura cont�nua do MPU6050\n");
    printf("Digite 'i' para PARAR captura cont�nua do MPU6050\n");
    printf("\nEscolha o comando:  ");
}

// ================================================================================
// FUN��ES DE GERENCIAMENTO DE ARQUIVOS MPU6050
// ================================================================================

/**
 * Inicializa o arquivo CSV para armazenar dados do MPU6050
 * Cria o arquivo e escreve o cabe�alho com as colunas de dados
 * @return true se inicializado com sucesso, false caso contr�rio
 */
bool init_mpu_csv_file() {
    FRESULT res = f_open(&mpu_file, mpu_filename, FA_WRITE | FA_CREATE_ALWAYS);
    if (res != FR_OK) {
        printf("[ERRO] N�o foi poss�vel criar o arquivo CSV do MPU6050. Verifique se o cart�o est� montado.\n");
        Estado = 'E';
        return false;
    }
    
    // Escreve o cabe�alho do arquivo CSV
    const char* header = "Sample,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,Roll,Pitch\n";
    UINT bw;
    res = f_write(&mpu_file, header, strlen(header), &bw);
    if (res != FR_OK) {
        printf("[ERRO] N�o foi poss�vel escrever o cabe�alho no arquivo CSV.\n");
        Estado = 'E';
        f_close(&mpu_file);
        return false;
    }
    
    printf("Arquivo CSV do MPU6050 inicializado: %s\n", mpu_filename);
    return true;
}

/**
 * Inicia a captura cont�nua de dados do MPU6050
 * Configura o sistema para logging autom�tico a 10Hz
 */
void start_mpu_logging() {
    if (mpu_logging_enabled) {
        printf("Captura do MPU6050 j� est� ativa!\n");
        Estado = 'E';
        return;
    }
    
    if (!init_mpu_csv_file()) {
        return;
    }
    
    // Configura vari�veis de controle do logging
    mpu_logging_enabled = true;
    sample_counter = 0;
    next_mpu_log_time = make_timeout_time_ms(mpu_log_period);
    
    printf("Iniciada captura cont�nua do MPU6050 (10Hz)\n");
    printf("Pressione 'i' para parar a captura.\n");
}

/**
 * Para a captura de dados do MPU6050
 * Fecha o arquivo e exibe estat�sticas da captura
 */
void stop_mpu_logging() {
    if (!mpu_logging_enabled) {
        printf("Captura do MPU6050 n�o est� ativa!\n");
        Estado = 'E';
        return;
    }
    
    mpu_logging_enabled = false;
    f_close(&mpu_file);
    printf("Captura do MPU6050 finalizada. Total de amostras: %lu\n", sample_counter);
    printf("Dados salvos em: %s\n", mpu_filename);
}

/**
 * Captura e salva uma amostra de dados do MPU6050 em formato CSV
 * @param aceleracao Array com dados de acelera��o [x, y, z]
 * @param gyro Array com dados do girosc�pio [x, y, z]
 * @param roll �ngulo roll calculado
 * @param pitch �ngulo pitch calculado
 */
void capture_mpu_data_to_csv(int16_t aceleracao[3], int16_t gyro[3], float roll, float pitch) {
    if (!mpu_logging_enabled) return;
    
    // Converte valores brutos para unidades f�sicas
    float ax = aceleracao[0] / 16384.0f; // Acelera��o em g
    float ay = aceleracao[1] / 16384.0f; // Acelera��o em g
    float az = aceleracao[2] / 16384.0f; // Acelera��o em g
    
    float gx = gyro[0] / 131.0f; // Velocidade angular em graus/s
    float gy = gyro[1] / 131.0f; // Velocidade angular em graus/s
    float gz = gyro[2] / 131.0f; // Velocidade angular em graus/s

    // Formata os dados em linha CSV
    char csv_line[200];
    snprintf(csv_line, sizeof(csv_line), 
             "%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f\n",
             sample_counter++, ax, ay, az, gx, gy, gz, roll, pitch);
    
    // Escreve no arquivo
    UINT bw;
    FRESULT res = f_write(&mpu_file, csv_line, strlen(csv_line), &bw);
    if (res != FR_OK) {
        printf("[ERRO] Falha ao escrever dados do MPU6050 no arquivo CSV.\n");
        Estado = 'E';
        stop_mpu_logging();
        return;
    }
    
    // Sincroniza arquivo a cada 50 amostras (aproximadamente 5 segundos)
    if (sample_counter % 50 == 0) {
        f_sync(&mpu_file);
        printf("Salvos %lu amostras do MPU6050...\n", sample_counter);
    }
}

/**
 * Fun��o melhorada para leitura e exibi��o de arquivos
 * @param filename Nome do arquivo a ser lido
 */
void read_file(const char *filename)
{
    if (!filename || strlen(filename) == 0) {
        printf("[ERRO] Nome do arquivo n�o fornecido.\n");
        printf("Uso: Pressione 'd' e forne�a o nome do arquivo\n\n");
        Estado = 'E';
        return;
    }
    
    FIL file;
    FRESULT res = f_open(&file, filename, FA_READ);
    
    if (res != FR_OK) {
        printf("[ERRO] N�o foi poss�vel abrir o arquivo '%s'.\n", filename);
        printf("Erro: %s (%d)\n", FRESULT_str(res), res);
        Estado = 'E';
        
        // Fornece sugest�es baseadas no tipo de erro
        switch (res) {
            case FR_NO_FILE:
                printf("Arquivo n�o encontrado. Use 'c' para listar arquivos dispon�veis.\n");
                break;
            case FR_NO_FILESYSTEM:
                printf("Sistema de arquivos n�o encontrado. Use 'a' para montar o SD.\n");
                break;
            case FR_DISK_ERR:
                printf("Erro no disco. Verifique a conex�o do cart�o SD.\n");
                break;
            default:
                printf("Verifique se o cart�o SD est� montado e o arquivo existe.\n");
                break;
        }
        printf("\n");
        return;
    }
    
    // Exibe informa��es do arquivo
    DWORD file_size = f_size(&file);
    printf("\n=== VISUALIZA��O DO ARQUIVO ===\n");
    printf("Nome: %s\n", filename);
    printf("Tamanho: %lu bytes\n", file_size);
    printf("Conte�do:\n");
    printf("?????????????????????????????????\n");
    
    char buffer[128];
    UINT bytes_read;
    int line_number = 1;
    
    // Para arquivos pequenos, mostra numera��o de linha
    bool show_line_numbers = (file_size < 2048);
    
    while (f_read(&file, buffer, sizeof(buffer) - 1, &bytes_read) == FR_OK && bytes_read > 0) {
        buffer[bytes_read] = '\0';
        
        if (show_line_numbers) {
            // Processa linha por linha para mostrar numera��o
            char *line_start = buffer;
            char *line_end;
            
            while ((line_end = strchr(line_start, '\n')) != NULL) {
                *line_end = '\0';
                printf("%3d: %s\n", line_number++, line_start);
                line_start = line_end + 1;
            }
            
            // �ltima linha sem quebra de linha
            if (strlen(line_start) > 0) {
                printf("%3d: %s", line_number++, line_start);
            }
        } else {
            printf("%s", buffer);
        }
    }
    
    f_close(&file);
    
    printf("\n?????????????????????????????????\n");
    printf("Arquivo lido com sucesso!\n");
    
    if (file_size > 1024) {
        printf("\nCopie os dados para transferir.\n");
    }
    
    printf("\n");
}

// ================================================================================
// TABELA DE COMANDOS - MAPEAMENTO DE STRINGS PARA FUN��ES
// ================================================================================

/**
 * Tabela que mapeia comandos de texto para suas respectivas fun��es
 * Usado pelo parser de comandos do terminal
 */
static cmd_def_t cmds[] = {
    {"setrtc", run_setrtc, "setrtc <DD> <MM> <YY> <hh> <mm> <ss>: Set Real Time Clock"},
    {"format", run_format, "format [<drive#:>]: Formata o cart�o SD"},
    {"mount", run_mount, "mount [<drive#:>]: Monta o cart�o SD"},
    {"unmount", run_unmount, "unmount <drive#:>: Desmonta o cart�o SD"},
    {"getfree", run_getfree, "getfree [<drive#:>]: Espa�o livre"},
    {"ls", run_ls, "ls: Lista arquivos"},
    {"cat", run_cat, "cat <filename>: Mostra conte�do do arquivo"},
    {"help", run_help, "help: Mostra comandos dispon�veis"}
};

// ================================================================================
// PROCESSAMENTO DE COMANDOS - PARSER DO TERMINAL
// ================================================================================

/**
 * Processa caracteres recebidos via stdin e executa comandos
 * Implementa um parser simples de linha de comando
 * @param cRxedChar Caractere recebido
 */
static void process_stdio(int cRxedChar)
{
    static char cmd[256];  // Buffer para comando
    static size_t ix;      // �ndice atual no buffer

    // Filtra apenas caracteres v�lidos
    if (!isprint(cRxedChar) && !isspace(cRxedChar) && '\r' != cRxedChar &&
        '\b' != cRxedChar && cRxedChar != (char)127)
        return;
    
    printf("%c", cRxedChar); // Echo do caractere
    stdio_flush();
    
    if (cRxedChar == '\r')  // Enter pressionado
    {
        printf("%c", '\n');
        stdio_flush();

        if (!strnlen(cmd, sizeof cmd))
        {
            printf("> ");
            stdio_flush();
            return;
        }
        
        // Tokeniza o comando
        char *cmdn = strtok(cmd, " ");
        if (cmdn)
        {
            size_t i;
            // Busca o comando na tabela
            for (i = 0; i < count_of(cmds); ++i)
            {
                if (0 == strcmp(cmds[i].command, cmdn))
                {
                    (*cmds[i].function)();  // Executa a fun��o correspondente
                    break;
                }
            }
            if (count_of(cmds) == i)
                printf("Command \"%s\" not found\n", cmdn);
        }
        
        // Reset do buffer de comando
        ix = 0;
        memset(cmd, 0, sizeof cmd);
        printf("\n> ");
        stdio_flush();
    }
    else
    {
        // Tratamento de backspace
        if (cRxedChar == '\b' || cRxedChar == (char)127)
        {
            if (ix > 0)
            {
                ix--;
                cmd[ix] = '\0';
            }
        }
        else
        {
            // Adiciona caractere ao buffer se h� espa�o
            if (ix < sizeof cmd - 1)
            {
                cmd[ix] = cRxedChar;
                ix++;
            }
        }
    }
}

// ================================================================================
// HANDLERS DE INTERRUP��O - CONTROLE DOS BOT�ES
// ================================================================================

/**
 * Handler de interrup��o GPIO para os bot�es A e B
 * Implementa debounce por software e controla estados do sistema
 * @param gpio N�mero do pino GPIO que gerou a interrup��o
 * @param events Tipo de evento (borda de subida/descida)
 */
void gpio_irq_handler(uint gpio, uint32_t events) {
    static uint32_t last_time = 0;
    uint32_t current_time = to_us_since_boot(get_absolute_time());

    // Debouncing de 300ms (300000 microsegundos)
    if ((current_time - last_time) > 300000) {
        
        // Bot�o A: Controla coleta de dados
        if (gpio == botaoA && !gpio_get(botaoA)) {
            last_time = current_time;
            Estado_coleta_dados = !Estado_coleta_dados;  // Toggle do estado
        }
        
        // Bot�o B: Controla montagem do cart�o SD
        if (gpio == botaoB && !gpio_get(botaoB)) {
            last_time = current_time;
            Estado_montar_cartao = !Estado_montar_cartao;  // Toggle do estado
        }
    }
}

// ================================================================================
// CONTROLE DO BUZZER - SINAIS SONOROS
// ================================================================================

/**
 * Gera um bipe no buzzer por tempo especificado
 * @param time_ms Dura��o do bipe em milissegundos
 */
void buzzer_beep(int time_ms) {
    gpio_put(buzzer, 1);      // Liga o buzzer
    sleep_ms(time_ms);        // Mant�m ligado pelo tempo especificado
    gpio_put(buzzer, 0);      // Desliga o buzzer
    sleep_ms(100);            // Pequeno intervalo entre bipes
}

/**
 * Gera padr�es de bipes espec�ficos para diferentes eventos do sistema
 * @param evento Tipo de evento que determina o padr�o sonoro
 */
void buzzer_signal(Evento_buzzer evento) {
    switch (evento) {
        case Som_desmontando:       // 2 bipes curtos - SD desmontado
            buzzer_beep(200);
            buzzer_beep(200);
            break;

        case Som_montando:          // 2 bipes curtos - SD montado
            buzzer_beep(100);
            buzzer_beep(100);
            break;

        case Som_iniciando_captura: // 1 bipe longo - in�cio da captura
            buzzer_beep(300);
            break;

        case Som_encerrando_captura: // 2 bipes (curto + longo) - fim da captura
            buzzer_beep(100);
            buzzer_beep(300);
            break;

        case som_leitura:           // 3 bipes curtos - leitura de dados
            buzzer_beep(100);
            buzzer_beep(100);
            buzzer_beep(100);
            break;

        case Som_erro:              // 3 bipes longos - erro no sistema
            buzzer_beep(300);
            buzzer_beep(300);
            buzzer_beep(300);
            break;
    }
}

// ================================================================================
// INICIALIZA��O DE PERIF�RICOS
// ================================================================================

/**
 * Inicializa todos os perif�ricos GPIO do sistema
 * Configura LEDs, buzzer, bot�es e suas interrup��es
 */
void iniciando_perifericos(){
    // Configura��o dos LEDs como sa�da
    gpio_init(Led_verde);
    gpio_set_dir(Led_verde, GPIO_OUT);

    gpio_init(Led_azul);
    gpio_set_dir(Led_azul, GPIO_OUT);

    gpio_init(Led_vermelho);
    gpio_set_dir(Led_vermelho, GPIO_OUT);

    // Configura��o do buzzer como sa�da
    gpio_init(buzzer);
    gpio_set_dir(buzzer, GPIO_OUT);

    // Configura��o do bot�o A com pull-up e interrup��o
    gpio_init(botaoA);
    gpio_set_dir(botaoA, GPIO_IN);
    gpio_pull_up(botaoA);
    gpio_set_irq_enabled_with_callback(botaoA, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    // Configura��o do bot�o B com pull-up e interrup��o
    gpio_init(botaoB);
    gpio_set_dir(botaoB, GPIO_IN);
    gpio_pull_up(botaoB);
    gpio_set_irq_enabled_with_callback(botaoB, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
}

// ================================================================================
// FUN��O PRINCIPAL - MAIN LOOP
// ================================================================================

/**
 * Fun��o principal do sistema
 * Inicializa todos os perif�ricos e executa o loop principal
 */
int main()
{
    // ============================================================================
    // INICIALIZA��O DO SISTEMA
    // ============================================================================
    
    stdio_init_all();           // Inicializa comunica��o serial
    iniciando_perifericos();    // Inicializa GPIOs
    
    // Sequ�ncia de inicializa��o com LEDs
    Estado = 'A';               // Estado de inicializa��o
    gpio_put(Led_verde, true);
    gpio_put(Led_vermelho, true);
    sleep_ms(5000);             // Aguarda 5 segundos
    Estado = 'N';               // Estado normal
    
    // Inicializa��o de perif�ricos do sistema
    time_init();                // Inicializa sistema de tempo
    adc_init();                 // Inicializa conversor A/D

    // ============================================================================
    // CONFIGURA��O DO DISPLAY OLED (I2C1)
    // ============================================================================
    
    // Inicializa I2C1 para o display OLED em 400kHz
    i2c_init(I2C_PORT_DISP, 400 * 1000);
    gpio_set_function(I2C_SDA_DISP, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_DISP, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_DISP);
    gpio_pull_up(I2C_SCL_DISP);

    // Configura��o e inicializa��o do display SSD1306
    ssd1306_t ssd;
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, ENDERECO_DISP, I2C_PORT_DISP);
    ssd1306_config(&ssd);
    ssd1306_send_data(&ssd);

    // Limpa o display inicialmente
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);

    // ============================================================================
    // CONFIGURA��O DO MPU6050 (I2C0)
    // ============================================================================
    
    // Inicializa I2C0 para o MPU6050 em 400kHz
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Declara os pinos como I2C na Binary Info (para picotool)
    bi_decl(bi_2pins_with_func(I2C_SDA, I2C_SCL, GPIO_FUNC_I2C));
    
    // Reset e inicializa��o do MPU6050
    mpu6050_reset();

    // ============================================================================
    // INICIALIZA��O DA INTERFACE DO USU�RIO
    // ============================================================================
    
    // Vari�veis para dados do MPU6050
    int16_t aceleracao[3], gyro[3], temp;
    bool cor = true;   // Vari�vel de controle de cor do display

    // Configura��o inicial do terminal
    printf("FatFS SPI example\n");
    printf("\033[2J\033[H");    // Limpa tela do terminal
    printf("\n> ");
    stdio_flush();
    run_help();                 // Exibe comandos dispon�veis

    // ============================================================================
    // LOOP PRINCIPAL DO SISTEMA
    // ============================================================================
    
    while (true)
    {
        // ========================================================================
        // PROCESSAMENTO DE MUDAN�AS DE ESTADO VIA BOT�ES
        // ========================================================================
        
        // Verifica mudan�a no estado de montagem do cart�o SD
        if(Estado_montar_cartao != Estado_montar_cartao_prev){
            if(Estado_montar_cartao){
                Estado = 'M';   // Estado: Montando SD
                printf("\nMontando o SD...\n");
                run_mount();
                printf("\nEscolha o comando (g = help):  ");
            }else{
                Estado = 'D';   // Estado: Desmontando SD
                printf("\nDesmontando o SD. Aguarde...\n");
                run_unmount();
                printf("\nEscolha o comando (g = help):  ");
            }
            Estado_montar_cartao_prev = Estado_montar_cartao;
        }

        // Verifica mudan�a no estado de coleta de dados
        if(Estado_coleta_dados != Estado_coleta_dados_prev){
            if(Estado_coleta_dados){
                Estado = 'I';   // Estado: Iniciando captura
                start_mpu_logging();
                printf("\nEscolha o comando (g = help):  ");
            }else{
                Estado = 'T';   // Estado: Terminando captura
                stop_mpu_logging();
                printf("\nEscolha o comando (g = help):  ");
            }
            Estado_coleta_dados_prev = Estado_coleta_dados;
        }

        // ========================================================================
        // PROCESSAMENTO DE COMANDOS VIA TERMINAL
        // ========================================================================
        
        // Verifica se h� caracteres dispon�veis no terminal
        int cRxedChar = getchar_timeout_us(0);
        if (PICO_ERROR_TIMEOUT != cRxedChar)
            process_stdio(cRxedChar);

        // ========================================================================
        // PROCESSAMENTO DE COMANDOS DIRETOS (TECLAS �NICAS)
        // ========================================================================
        
        if (cRxedChar == 'a') // Monta o SD card
        {
            Estado = 'M';
            printf("\nMontando o SD...\n");
            run_mount();
            printf("\nEscolha o comando (g = help):  ");
            Estado_montar_cartao = true;
            Estado_montar_cartao_prev = true;
        }
        
        if (cRxedChar == 'b') // Desmonta o SD card
        {
            Estado = 'D';
            printf("\nDesmontando o SD. Aguarde...\n");
            run_unmount();
            printf("\nEscolha o comando (g = help):  ");
            Estado_montar_cartao = false;
            Estado_montar_cartao_prev = false;
        }
        
        if (cRxedChar == 'c') // Lista arquivos
        {
            Estado = 'V';
            printf("\nListagem de arquivos no cart�o SD.\n");
            run_ls();
            printf("\nListagem conclu�da.\n");
            printf("\nEscolha o comando (g = help):  ");
        }
        
        if (cRxedChar == 'd') // Exibe conte�do do arquivo
        {
            Estado = 'L';
            read_file(mpu_filename);
            printf("Escolha o comando (g = help):  ");
        }
        
        if (cRxedChar == 'e') // Obt�m espa�o livre no SD
        {
            Estado = 'S';
            printf("\nObtendo espa�o livre no SD.\n\n");
            run_getfree();
            printf("\nEspa�o livre obtido.\n");
            printf("\nEscolha o comando (g = help):  ");
        }
        
        if (cRxedChar == 'f') // Formata o SD card
        {
            Estado = 'F';
            printf("\nProcesso de formata��o do SD iniciado. Aguarde...\n");
            run_format();
            printf("\nFormata��o conclu�da.\n\n");
            printf("\nEscolha o comando (g = help):  ");
        }
        
        if (cRxedChar == 'g') // Exibe comandos dispon�veis
        {
            run_help();
            Estado = 'H';
        }
        
        if (cRxedChar == 'h') // Inicia captura cont�nua do MPU6050
        {
            Estado = 'I';
            start_mpu_logging();
            printf("\nEscolha o comando (g = help):  ");
            Estado_coleta_dados = true;
            Estado_coleta_dados_prev = true;
        }
        
        if (cRxedChar == 'i') // Para captura cont�nua do MPU6050
        {
            Estado = 'T';
            stop_mpu_logging();
            printf("\nEscolha o comando (g = help):  ");
            Estado_coleta_dados = false;
            Estado_coleta_dados_prev = false;
        }

        // ========================================================================
        // CONTROLE DE LEDs E BUZZER BASEADO NO ESTADO
        // ========================================================================
        
        // Atualiza LEDs e buzzer apenas quando o estado muda
        if (Estado != estado_fut){
            switch (Estado)
            {
            case 'G':   // Estado: Gen�rico/Verde
                Estado_led_verde = true;
                Estado_led_azul = false;
                Estado_led_vermelho = false;
                break;
                
            case 'A':   // Estado: Inicializando (�mbar)
                Estado_led_verde = true;
                Estado_led_azul = false;
                Estado_led_vermelho = true;
                buzzer_signal(Som_desmontando);
                break;
                
            case 'L':   // Estado: Leitura (Azul)
                Estado_led_verde = false;
                Estado_led_azul = true;
                Estado_led_vermelho = false;
                buzzer_signal(som_leitura);
                break;
                
            case 'V':   // Estado: Visualiza��o (Azul)
                Estado_led_verde = false;
                Estado_led_azul = true;
                Estado_led_vermelho = false;
                buzzer_signal(som_leitura);
                break;
                
            case 'I':   // Estado: Iniciando captura (Roxo)
                Estado_led_verde = false;
                Estado_led_azul = true;
                Estado_led_vermelho = true;
                buzzer_signal(Som_iniciando_captura);
                break;
                
            case 'T':   // Estado: Terminando captura (Verde)
                Estado_led_verde = true;
                Estado_led_azul = false;
                Estado_led_vermelho = false;
                buzzer_signal(Som_encerrando_captura);
                break;
                
            case 'E':   // Estado: Erro (Vermelho)
                Estado_led_verde = false;
                Estado_led_azul = false;
                Estado_led_vermelho = true;
                buzzer_signal(Som_erro);
                break;
                
            case 'N':   // Estado: Normal (Branco)
                Estado_led_verde = true;
                Estado_led_azul = true;
                Estado_led_vermelho = true;
                break;
        
            default:
                break;
            }
            estado_fut = Estado;
        }

        // Atualiza os LEDs fisicamente
        gpio_put(Led_verde, Estado_led_verde);
        gpio_put(Led_azul, Estado_led_azul);
        gpio_put(Led_vermelho, Estado_led_vermelho);

        // ========================================================================
        // LEITURA E PROCESSAMENTO DE DADOS DO MPU6050
        // ========================================================================
        
        // L� dados brutos do MPU6050
        mpu6050_read_raw(aceleracao, gyro, &temp);

        // Converte acelera��o para unidades de 'g' (gravidade terrestre)
        float ax = aceleracao[0] / 16384.0f;
        float ay = aceleracao[1] / 16384.0f;
        float az = aceleracao[2] / 16384.0f;

        // Calcula �ngulos Roll e Pitch em graus
        float roll  = atan2(ay, az) * 180.0f / M_PI;
        float pitch = atan2(-ax, sqrt(ay*ay + az*az)) * 180.0f / M_PI;

        // Captura dados para arquivo CSV se o logging estiver ativo
        if (mpu_logging_enabled && absolute_time_diff_us(get_absolute_time(), next_mpu_log_time) <= 0) {
            capture_mpu_data_to_csv(aceleracao, gyro, roll, pitch);
            next_mpu_log_time = make_timeout_time_ms(mpu_log_period);
        }

        // ========================================================================
        // ATUALIZA��O DO DISPLAY OLED
        // ========================================================================
        
        ssd1306_fill(&ssd, !cor);  // Limpa o display

        // Exibe diferentes telas baseadas no estado atual
        if(Estado == 'I'){  // Tela de captura ativa
            char str_roll[20];
            char str_pitch[20];

            snprintf(str_roll,  sizeof(str_roll),  "%5.1f", roll);
            snprintf(str_pitch, sizeof(str_pitch), "%5.1f", pitch);

            // Desenha interface de captura
            ssd1306_rect(&ssd, 3, 3, 122, 60, cor, !cor);
            ssd1306_line(&ssd, 3, 25, 123, 25, cor);
            ssd1306_line(&ssd, 3, 37, 123, 37, cor);
            ssd1306_draw_string(&ssd, "CAPTURANDO", 22, 6);
            ssd1306_draw_string(&ssd, "DADOS", 33, 16);
            ssd1306_draw_string(&ssd, "IMU    MPU6050", 10, 28);
            ssd1306_line(&ssd, 63, 35, 63, 60, cor);
            ssd1306_draw_string(&ssd, "roll", 14, 41);
            ssd1306_draw_string(&ssd, str_roll, 14, 52);
            ssd1306_draw_string(&ssd, "pitch", 73, 41);
            ssd1306_draw_string(&ssd, str_pitch, 73, 52);
        }
        else if((Estado == 'N') || (Estado == 'D') || (Estado == 'M')){  // Tela inicial/status
            ssd1306_rect(&ssd, 3, 3, 122, 60, cor, !cor);
            ssd1306_line(&ssd, 3, 30, 123, 30, cor);
            ssd1306_line(&ssd, 3, 47, 123, 47, cor);
            ssd1306_draw_string(&ssd, "SISTEMA", 35, 8);
            ssd1306_draw_string(&ssd, "INICIADO", 33, 20);
            if(Estado_montar_cartao){
                ssd1306_draw_string(&ssd, "SD: MONTADO", 18, 36);
            }else{
                ssd1306_draw_string(&ssd, "SD: DESMONTADO", 8, 36);
            }
            ssd1306_draw_string(&ssd, "g=HELP", 35, 52);
        }
        else if(Estado == 'V'){  // Tela de visualiza��o
            ssd1306_rect(&ssd, 3, 3, 122, 60, cor, !cor);
            ssd1306_line(&ssd, 3, 18, 123, 18, cor);
            ssd1306_line(&ssd, 3, 30, 123, 30, cor);
            ssd1306_draw_string(&ssd, "DADOS DO SD", 22, 8);
            ssd1306_draw_string(&ssd, "VISUALIZACAO", 15, 20);
            ssd1306_draw_string(&ssd, "LISTA DE", 30, 32);
            ssd1306_draw_string(&ssd, "ARQUIVOS", 30, 42);
            ssd1306_draw_string(&ssd, "NO TERMINAL", 22, 52);
        }
        else if(Estado == 'L'){  // Tela de leitura
            ssd1306_rect(&ssd, 3, 3, 122, 60, cor, !cor);
            ssd1306_line(&ssd, 3, 18, 123, 18, cor);
            ssd1306_line(&ssd, 3, 30, 123, 30, cor);
            ssd1306_draw_string(&ssd, "DADOS DO SD", 22, 8);
            ssd1306_draw_string(&ssd, "LEITURA", 33, 20);
            ssd1306_draw_string(&ssd, "LEITURA DOS", 26, 32);
            ssd1306_draw_string(&ssd, "DADOS", 38, 42);
            ssd1306_draw_string(&ssd, "NO TERMINAL", 22, 52);
        }
        else if(Estado == 'T'){  // Tela de captura finalizada
            char str_amostras[20];
            snprintf(str_amostras, sizeof(str_amostras), "%d", sample_counter);

            ssd1306_rect(&ssd, 3, 3, 122, 60, cor, !cor);
            ssd1306_line(&ssd, 3, 30, 123, 30, cor);
            ssd1306_line(&ssd, 3, 47, 123, 47, cor);
            ssd1306_draw_string(&ssd, "DADOS GRAVADOS", 9, 8);
            ssd1306_draw_string(&ssd, "NO CARTAO SD", 15, 20);
            ssd1306_draw_string(&ssd, "N AMOSTRAS:", 8, 35);
            ssd1306_draw_string(&ssd, str_amostras, 100, 35);
            ssd1306_draw_string(&ssd, "NOME: mpu_data", 5, 50);
        }
        else if(Estado == 'E'){  // Tela de erro
            ssd1306_rect(&ssd, 3, 3, 122, 60, cor, !cor);
            ssd1306_line(&ssd, 3, 30, 123, 30, cor);
            ssd1306_line(&ssd, 3, 47, 123, 47, cor);
            ssd1306_draw_string(&ssd, "ERRO DE COMANDO", 3, 8);
            ssd1306_draw_string(&ssd, "VERIFIQUE", 22, 20);
            if(Estado_montar_cartao){
                ssd1306_draw_string(&ssd, "SD: MONTADO", 18, 36);
            }else{
                ssd1306_draw_string(&ssd, "SD: DESMONTADO", 8, 36);
            }
            ssd1306_draw_string(&ssd, "g=HELP", 35, 52);
        }
        else if(Estado == 'H'){  // Tela de ajuda
            ssd1306_rect(&ssd, 3, 3, 122, 60, cor, !cor);
            ssd1306_line(&ssd, 3, 18, 123, 18, cor);
            ssd1306_draw_string(&ssd, "BITDOGLAB", 24, 8);
            ssd1306_draw_string(&ssd, "BO A=MONTAR", 6, 22);
            ssd1306_draw_string(&ssd, "BO A=DESMONTAR", 6, 32);
            ssd1306_draw_string(&ssd, "BO B=INICI CAP", 6, 42);
            ssd1306_draw_string(&ssd, "BO B=ENCER CAP", 6, 52);
        }
       
        // Envia dados atualizados para o display
        ssd1306_send_data(&ssd);
        sleep_ms(500);  // Delay para atualiza��o da tela
    }
    
    return 0;
}
