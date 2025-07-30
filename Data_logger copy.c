/*
 * ================================================================================
 * SISTEMA DE COLETA DE DADOS MPU6050 COM RASPBERRY PI PICO
 * ================================================================================
 * 
 * Descrição: Sistema embarcado para coleta contínua de dados do sensor MPU6050
 *            com armazenamento em cartão SD e interface OLED para visualização
 * ================================================================================
 */

// ================================================================================
// INCLUDES - BIBLIOTECAS NECESSÁRIAS
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

// Bibliotecas específicas do projeto
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
// DEFINIÇÕES DE HARDWARE - MAPEAMENTO DOS PINOS
// ================================================================================

// Configuração I2C para MPU6050
#define I2C_PORT i2c0                 // I2C0 usa pinos 0 e 1
#define I2C_SDA 0                     // Pino SDA do MPU6050
#define I2C_SCL 1                     // Pino SCL do MPU6050

// Configuração I2C para Display OLED
#define I2C_PORT_DISP i2c1            // I2C1 para display
#define I2C_SDA_DISP 14               // Pino SDA do display
#define I2C_SCL_DISP 15               // Pino SCL do display
#define ENDERECO_DISP 0x3C            // Endereço I2C do display SSD1306

// Definição dos LEDs indicadores
#define Led_verde 11                  // LED verde - sistema OK
#define Led_azul 12                   // LED azul - operações de leitura
#define Led_vermelho 13               // LED vermelho - erro/captura

// Periféricos de interface
#define buzzer 21                     // Buzzer para sinais sonoros
#define botaoA 5                      // Botão A - controle de captura
#define botaoB 6                      // Botão B - controle de SD

// ================================================================================
// CONSTANTES E CONFIGURAÇÕES DO SISTEMA
// ================================================================================

// Configurações do MPU6050
static int addr = 0x68;               // Endereço padrão do MPU6050

// Configurações de logging do MPU6050
static const uint32_t mpu_log_period = 100;  // 100ms = 10Hz
static char mpu_filename[20] = "mpu_data2.csv"; // Nome do arquivo CSV

// Configurações gerais de logging
static const uint32_t period = 1000;  // Período geral de 1 segundo

// ================================================================================
// VARIÁVEIS GLOBAIS - CONTROLE DE ESTADO DO SISTEMA
// ================================================================================

// Controle de estado principal
char Estado = 'L';                    // Estado atual do sistema
char estado_fut = 'L';                // Estado futuro para comparação

// Estados dos LEDs
bool Estado_led_verde = false;
bool Estado_led_azul = false;
bool Estado_led_vermelho = false;

// Estados de operação
bool Estado_coleta_dados = false;     // Flag de coleta de dados ativa
bool Estado_montar_cartao = false;   // Flag de cartão SD montado

// Estados anteriores para detecção de mudanças
bool Estado_coleta_dados_prev = false;
bool Estado_montar_cartao_prev = false;

// Variáveis de controle do logging MPU6050
static bool mpu_logging_enabled = false;      // Flag de logging ativo
static absolute_time_t next_mpu_log_time;     // Próximo tempo de log
static FIL mpu_file;                          // Handle do arquivo CSV
static uint32_t sample_counter = 0;           // Contador de amostras

// Variáveis de controle geral
static bool logger_enabled;
static absolute_time_t next_log_time;

// ================================================================================
// ENUMERAÇÕES - DEFINIÇÃO DE TIPOS
// ================================================================================

/**
 * Enumera os diferentes tipos de eventos sonoros do buzzer
 * Cada evento tem um padrão específico de bipes
 */
typedef enum {
    Som_desmontando,        // 2 bipes curtos - sistema iniciado
    Som_montando,           // 2 bipes curtos - cartão montado
    Som_iniciando_captura,  // 1 bipe longo - início da captura
    Som_encerrando_captura, // 2 bipes (curto + longo) - fim da captura
    som_leitura,           // 3 bipes curtos - leitura de dados
    Som_erro               // 3 bipes longos - erro no sistema
} Evento_buzzer;

// ================================================================================
// ESTRUTURAS E TIPOS - DEFINIÇÕES DE COMANDOS
// ================================================================================

typedef void (*p_fn_t)();

/**
 * Estrutura para definição de comandos do terminal
 * Mapeia strings de comando para suas respectivas funções
 */
typedef struct
{
    char const *const command;         // String do comando
    p_fn_t const function;            // Ponteiro para função
    char const *const help;           // Texto de ajuda
} cmd_def_t;

// ================================================================================
// FUNÇÕES AUXILIARES - GERENCIAMENTO DE SD CARD
// ================================================================================

/**
 * Busca um cartão SD pelo nome
 * @param name Nome do cartão a ser buscado
 * @return Ponteiro para estrutura sd_card_t ou NULL se não encontrado
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
 * Busca o sistema de arquivos de um cartão SD pelo nome
 * @param name Nome do cartão
 * @return Ponteiro para estrutura FATFS ou NULL se não encontrado
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
// FUNÇÕES DE COMANDO - INTERFACE DO TERMINAL
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

    // Configuração da estrutura datetime_t
    datetime_t t = {
        .year = (int16_t)year,
        .month = (int8_t)month,
        .day = (int8_t)date,
        .dotw = 0, // 0 is Sunday
        .hour = (int8_t)hour,
        .min = (int8_t)min,
        .sec = (int8_t)sec};
    
    // Aplica a configuração no RTC
    rtc_set_datetime(&t);
}

/**
 * Formata o cartão SD com sistema de arquivos FAT
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
    
    /* Formata o drive com parâmetros padrão */
    FRESULT fr = f_mkfs(arg1, 0, 0, FF_MAX_SS * 2);
    if (FR_OK != fr)
        printf("f_mkfs error: %s (%d)\n", FRESULT_str(fr), fr);
}

/**
 * Monta o cartão SD no sistema de arquivos
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
    
    // Marca o cartão como montado
    sd_card_t *pSD = sd_get_by_name(arg1);
    myASSERT(pSD);
    pSD->mounted = true;
    printf("Processo de montagem do SD ( %s ) concluído\n", pSD->pcName);
}

/**
 * Desmonta o cartão SD do sistema
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
    
    // Marca o cartão como desmontado
    sd_card_t *pSD = sd_get_by_name(arg1);
    myASSERT(pSD);
    pSD->mounted = false;
    pSD->m_Status |= STA_NOINIT; // caso o meio seja removido
    printf("SD ( %s ) desmontado\n", pSD->pcName);
}

/**
 * Obtém informações de espaço livre no cartão SD
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
    
    // Calcula espaço total e livre
    tot_sect = (p_fs->n_fatent - 2) * p_fs->csize;
    fre_sect = fre_clust * p_fs->csize;
    printf("%10lu KiB total drive space.\n%10lu KiB available.\n", tot_sect / 2, fre_sect / 2);
}

/**
 * Lista arquivos e diretórios no cartão SD
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
        // Obtém diretório atual se não especificado
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
    
    // Busca primeiro arquivo/diretório
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
 * Exibe o conteúdo de um arquivo (comando cat)
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
    
    // Lê e exibe o arquivo linha por linha
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
 * Exibe a lista de comandos disponíveis
 */
static void run_help()
{
    printf("\nComandos disponíveis:\n\n");
    printf("Digite 'a' para montar o cartão SD\n");
    printf("Digite 'b' para desmontar o cartão SD\n");
    printf("Digite 'c' para listar arquivos\n");
    printf("Digite 'd' para mostrar conteúdo do arquivo\n");
    printf("Digite 'e' para obter espaço livre no cartão SD\n");
    printf("Digite 'f' para formatar o cartão SD\n");
    printf("Digite 'g' para exibir os comandos disponíveis\n");
    printf("Digite 'h' para INICIAR captura contínua do MPU6050\n");
    printf("Digite 'i' para PARAR captura contínua do MPU6050\n");
    printf("\nEscolha o comando:  ");
}

// ================================================================================
// FUNÇÕES DE GERENCIAMENTO DE ARQUIVOS MPU6050
// ================================================================================

/**
 * Inicializa o arquivo CSV para armazenar dados do MPU6050
 * Cria o arquivo e escreve o cabeçalho com as colunas de dados
 * @return true se inicializado com sucesso, false caso contrário
 */
bool init_mpu_csv_file() {
    FRESULT res = f_open(&mpu_file, mpu_filename, FA_WRITE | FA_CREATE_ALWAYS);
    if (res != FR_OK) {
        printf("[ERRO] Não foi possível criar o arquivo CSV do MPU6050. Verifique se o cartão está montado.\n");
        Estado = 'E';
        return false;
    }
    
    // Escreve o cabeçalho do arquivo CSV
    const char* header = "Sample,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,Roll,Pitch\n";
    UINT bw;
    res = f_write(&mpu_file, header, strlen(header), &bw);
    if (res != FR_OK) {
        printf("[ERRO] Não foi possível escrever o cabeçalho no arquivo CSV.\n");
        Estado = 'E';
        f_close(&mpu_file);
        return false;
    }
    
    printf("Arquivo CSV do MPU6050 inicializado: %s\n", mpu_filename);
    return true;
}

/**
 * Inicia a captura contínua de dados do MPU6050
 * Configura o sistema para logging automático a 10Hz
 */
void start_mpu_logging() {
    if (mpu_logging_enabled) {
        printf("Captura do MPU6050 já está ativa!\n");
        Estado = 'E';
        return;
    }
    
    if (!init_mpu_csv_file()) {
        return;
    }
    
    // Configura variáveis de controle do logging
    mpu_logging_enabled = true;
    sample_counter = 0;
    next_mpu_log_time = make_timeout_time_ms(mpu_log_period);
    
    printf("Iniciada captura contínua do MPU6050 (10Hz)\n");
    printf("Pressione 'i' para parar a captura.\n");
}

/**
 * Para a captura de dados do MPU6050
 * Fecha o arquivo e exibe estatísticas da captura
 */
void stop_mpu_logging() {
    if (!mpu_logging_enabled) {
        printf("Captura do MPU6050 não está ativa!\n");
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
 * @param aceleracao Array com dados de aceleração [x, y, z]
 * @param gyro Array com dados do giroscópio [x, y, z]
 * @param roll Ângulo roll calculado
 * @param pitch Ângulo pitch calculado
 */
void capture_mpu_data_to_csv(int16_t aceleracao[3], int16_t gyro[3], float roll, float pitch) {
    if (!mpu_logging_enabled) return;
    
    // Converte valores brutos para unidades físicas
    float ax = aceleracao[0] / 16384.0f; // Aceleração em g
    float ay = aceleracao[1] / 16384.0f; // Aceleração em g
    float az = aceleracao[2] / 16384.0f; // Aceleração em g
    
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
 * Função melhorada para leitura e exibição de arquivos
 * @param filename Nome do arquivo a ser lido
 */
void read_file(const char *filename)
{
    if (!filename || strlen(filename) == 0) {
        printf("[ERRO] Nome do arquivo não fornecido.\n");
        printf("Uso: Pressione 'd' e forneça o nome do arquivo\n\n");
        Estado = 'E';
        return;
    }
    
    FIL file;
    FRESULT res = f_open(&file, filename, FA_READ);
    
    if (res != FR_OK) {
        printf("[ERRO] Não foi possível abrir o arquivo '%s'.\n", filename);
        printf("Erro: %s (%d)\n", FRESULT_str(res), res);
        Estado = 'E';
        
        // Fornece sugestões baseadas no tipo de erro
        switch (res) {
            case FR_NO_FILE:
                printf("Arquivo não encontrado. Use 'c' para listar arquivos disponíveis.\n");
                break;
            case FR_NO_FILESYSTEM:
                printf("Sistema de arquivos não encontrado. Use 'a' para montar o SD.\n");
                break;
            case FR_DISK_ERR:
                printf("Erro no disco. Verifique a conexão do cartão SD.\n");
                break;
            default:
                printf("Verifique se o cartão SD está montado e o arquivo existe.\n");
                break;
        }
        printf("\n");
        return;
    }
    
    // Exibe informações do arquivo
    DWORD file_size = f_size(&file);
    printf("\n=== VISUALIZAÇÃO DO ARQUIVO ===\n");
    printf("Nome: %s\n", filename);
    printf("Tamanho: %lu bytes\n", file_size);
    printf("Conteúdo:\n");
    printf("?????????????????????????????????\n");
    
    char buffer[128];
    UINT bytes_read;
    int line_number = 1;
    
    // Para arquivos pequenos, mostra numeração de linha
    bool show_line_numbers = (file_size < 2048);
    
    while (f_read(&file, buffer, sizeof(buffer) - 1, &bytes_read) == FR_OK && bytes_read > 0) {
        buffer[bytes_read] = '\0';
        
        if (show_line_numbers) {
            // Processa linha por linha para mostrar numeração
            char *line_start = buffer;
            char *line_end;
            
            while ((line_end = strchr(line_start, '\n')) != NULL) {
                *line_end = '\0';
                printf("%3d: %s\n", line_number++, line_start);
                line_start = line_end + 1;
            }
            
            // Última linha sem quebra de linha
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
// TABELA DE COMANDOS - MAPEAMENTO DE STRINGS PARA FUNÇÕES
// ================================================================================

/**
 * Tabela que mapeia comandos de texto para suas respectivas funções
 * Usado pelo parser de comandos do terminal
 */
static cmd_def_t cmds[] = {
    {"setrtc", run_setrtc, "setrtc <DD> <MM> <YY> <hh> <mm> <ss>: Set Real Time Clock"},
    {"format", run_format, "format [<drive#:>]: Formata o cartão SD"},
    {"mount", run_mount, "mount [<drive#:>]: Monta o cartão SD"},
    {"unmount", run_unmount, "unmount <drive#:>: Desmonta o cartão SD"},
    {"getfree", run_getfree, "getfree [<drive#:>]: Espaço livre"},
    {"ls", run_ls, "ls: Lista arquivos"},
    {"cat", run_cat, "cat <filename>: Mostra conteúdo do arquivo"},
    {"help", run_help, "help: Mostra comandos disponíveis"}
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
    static size_t ix;      // Índice atual no buffer

    // Filtra apenas caracteres válidos
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
                    (*cmds[i].function)();  // Executa a função correspondente
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
            // Adiciona caractere ao buffer se há espaço
            if (ix < sizeof cmd - 1)
            {
                cmd[ix] = cRxedChar;
                ix++;
            }
        }
    }
}

// ================================================================================
// HANDLERS DE INTERRUPÇÃO - CONTROLE DOS BOTÕES
// ================================================================================

/**
 * Handler de interrupção GPIO para os botões A e B
 * Implementa debounce por software e controla estados do sistema
 * @param gpio Número do pino GPIO que gerou a interrupção
 * @param events Tipo de evento (borda de subida/descida)
 */
void gpio_irq_handler(uint gpio, uint32_t events) {
    static uint32_t last_time = 0;
    uint32_t current_time = to_us_since_boot(get_absolute_time());

    // Debouncing de 300ms (300000 microsegundos)
    if ((current_time - last_time) > 300000) {
        
        // Botão A: Controla coleta de dados
        if (gpio == botaoA && !gpio_get(botaoA)) {
            last_time = current_time;
            Estado_coleta_dados = !Estado_coleta_dados;  // Toggle do estado
        }
        
        // Botão B: Controla montagem do cartão SD
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
 * @param time_ms Duração do bipe em milissegundos
 */
void buzzer_beep(int time_ms) {
    gpio_put(buzzer, 1);      // Liga o buzzer
    sleep_ms(time_ms);        // Mantém ligado pelo tempo especificado
    gpio_put(buzzer, 0);      // Desliga o buzzer
    sleep_ms(100);            // Pequeno intervalo entre bipes
}

/**
 * Gera padrões de bipes específicos para diferentes eventos do sistema
 * @param evento Tipo de evento que determina o padrão sonoro
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

        case Som_iniciando_captura: // 1 bipe longo - início da captura
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
// INICIALIZAÇÃO DE PERIFÉRICOS
// ================================================================================

/**
 * Inicializa todos os periféricos GPIO do sistema
 * Configura LEDs, buzzer, botões e suas interrupções
 */
void iniciando_perifericos(){
    // Configuração dos LEDs como saída
    gpio_init(Led_verde);
    gpio_set_dir(Led_verde, GPIO_OUT);

    gpio_init(Led_azul);
    gpio_set_dir(Led_azul, GPIO_OUT);

    gpio_init(Led_vermelho);
    gpio_set_dir(Led_vermelho, GPIO_OUT);

    // Configuração do buzzer como saída
    gpio_init(buzzer);
    gpio_set_dir(buzzer, GPIO_OUT);

    // Configuração do botão A com pull-up e interrupção
    gpio_init(botaoA);
    gpio_set_dir(botaoA, GPIO_IN);
    gpio_pull_up(botaoA);
    gpio_set_irq_enabled_with_callback(botaoA, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    // Configuração do botão B com pull-up e interrupção
    gpio_init(botaoB);
    gpio_set_dir(botaoB, GPIO_IN);
    gpio_pull_up(botaoB);
    gpio_set_irq_enabled_with_callback(botaoB, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
}

// ================================================================================
// FUNÇÃO PRINCIPAL - MAIN LOOP
// ================================================================================

/**
 * Função principal do sistema
 * Inicializa todos os periféricos e executa o loop principal
 */
int main()
{
    // ============================================================================
    // INICIALIZAÇÃO DO SISTEMA
    // ============================================================================
    
    stdio_init_all();           // Inicializa comunicação serial
    iniciando_perifericos();    // Inicializa GPIOs
    
    // Sequência de inicialização com LEDs
    Estado = 'A';               // Estado de inicialização
    gpio_put(Led_verde, true);
    gpio_put(Led_vermelho, true);
    sleep_ms(5000);             // Aguarda 5 segundos
    Estado = 'N';               // Estado normal
    
    // Inicialização de periféricos do sistema
    time_init();                // Inicializa sistema de tempo
    adc_init();                 // Inicializa conversor A/D

    // ============================================================================
    // CONFIGURAÇÃO DO DISPLAY OLED (I2C1)
    // ============================================================================
    
    // Inicializa I2C1 para o display OLED em 400kHz
    i2c_init(I2C_PORT_DISP, 400 * 1000);
    gpio_set_function(I2C_SDA_DISP, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_DISP, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_DISP);
    gpio_pull_up(I2C_SCL_DISP);

    // Configuração e inicialização do display SSD1306
    ssd1306_t ssd;
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, ENDERECO_DISP, I2C_PORT_DISP);
    ssd1306_config(&ssd);
    ssd1306_send_data(&ssd);

    // Limpa o display inicialmente
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);

    // ============================================================================
    // CONFIGURAÇÃO DO MPU6050 (I2C0)
    // ============================================================================
    
    // Inicializa I2C0 para o MPU6050 em 400kHz
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Declara os pinos como I2C na Binary Info (para picotool)
    bi_decl(bi_2pins_with_func(I2C_SDA, I2C_SCL, GPIO_FUNC_I2C));
    
    // Reset e inicialização do MPU6050
    mpu6050_reset();

    // ============================================================================
    // INICIALIZAÇÃO DA INTERFACE DO USUÁRIO
    // ============================================================================
    
    // Variáveis para dados do MPU6050
    int16_t aceleracao[3], gyro[3], temp;
    bool cor = true;   // Variável de controle de cor do display

    // Configuração inicial do terminal
    printf("FatFS SPI example\n");
    printf("\033[2J\033[H");    // Limpa tela do terminal
    printf("\n> ");
    stdio_flush();
    run_help();                 // Exibe comandos disponíveis

    // ============================================================================
    // LOOP PRINCIPAL DO SISTEMA
    // ============================================================================
    
    while (true)
    {
        // ========================================================================
        // PROCESSAMENTO DE MUDANÇAS DE ESTADO VIA BOTÕES
        // ========================================================================
        
        // Verifica mudança no estado de montagem do cartão SD
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

        // Verifica mudança no estado de coleta de dados
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
        
        // Verifica se há caracteres disponíveis no terminal
        int cRxedChar = getchar_timeout_us(0);
        if (PICO_ERROR_TIMEOUT != cRxedChar)
            process_stdio(cRxedChar);

        // ========================================================================
        // PROCESSAMENTO DE COMANDOS DIRETOS (TECLAS ÚNICAS)
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
            printf("\nListagem de arquivos no cartão SD.\n");
            run_ls();
            printf("\nListagem concluída.\n");
            printf("\nEscolha o comando (g = help):  ");
        }
        
        if (cRxedChar == 'd') // Exibe conteúdo do arquivo
        {
            Estado = 'L';
            read_file(mpu_filename);
            printf("Escolha o comando (g = help):  ");
        }
        
        if (cRxedChar == 'e') // Obtém espaço livre no SD
        {
            Estado = 'S';
            printf("\nObtendo espaço livre no SD.\n\n");
            run_getfree();
            printf("\nEspaço livre obtido.\n");
            printf("\nEscolha o comando (g = help):  ");
        }
        
        if (cRxedChar == 'f') // Formata o SD card
        {
            Estado = 'F';
            printf("\nProcesso de formatação do SD iniciado. Aguarde...\n");
            run_format();
            printf("\nFormatação concluída.\n\n");
            printf("\nEscolha o comando (g = help):  ");
        }
        
        if (cRxedChar == 'g') // Exibe comandos disponíveis
        {
            run_help();
            Estado = 'H';
        }
        
        if (cRxedChar == 'h') // Inicia captura contínua do MPU6050
        {
            Estado = 'I';
            start_mpu_logging();
            printf("\nEscolha o comando (g = help):  ");
            Estado_coleta_dados = true;
            Estado_coleta_dados_prev = true;
        }
        
        if (cRxedChar == 'i') // Para captura contínua do MPU6050
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
            case 'G':   // Estado: Genérico/Verde
                Estado_led_verde = true;
                Estado_led_azul = false;
                Estado_led_vermelho = false;
                break;
                
            case 'A':   // Estado: Inicializando (Âmbar)
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
                
            case 'V':   // Estado: Visualização (Azul)
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
        
        // Lê dados brutos do MPU6050
        mpu6050_read_raw(aceleracao, gyro, &temp);

        // Converte aceleração para unidades de 'g' (gravidade terrestre)
        float ax = aceleracao[0] / 16384.0f;
        float ay = aceleracao[1] / 16384.0f;
        float az = aceleracao[2] / 16384.0f;

        // Calcula ângulos Roll e Pitch em graus
        float roll  = atan2(ay, az) * 180.0f / M_PI;
        float pitch = atan2(-ax, sqrt(ay*ay + az*az)) * 180.0f / M_PI;

        // Captura dados para arquivo CSV se o logging estiver ativo
        if (mpu_logging_enabled && absolute_time_diff_us(get_absolute_time(), next_mpu_log_time) <= 0) {
            capture_mpu_data_to_csv(aceleracao, gyro, roll, pitch);
            next_mpu_log_time = make_timeout_time_ms(mpu_log_period);
        }

        // ========================================================================
        // ATUALIZAÇÃO DO DISPLAY OLED
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
        else if(Estado == 'V'){  // Tela de visualização
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
        sleep_ms(500);  // Delay para atualização da tela
    }
    
    return 0;
}
