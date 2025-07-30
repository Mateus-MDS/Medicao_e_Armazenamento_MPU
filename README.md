Data Logger com MPU6050 e Raspberry Pi Pico

Descrição:

Sistema de aquisição de dados de movimento usando sensor MPU6050 (acelerômetro + giroscópio) com armazenamento em cartão SD e interface OLED. Projeto ideal para monitoramento de vibração e análise de movimento.

Componentes Principais:

BitDogLab
Sensor MPU6050
suporte para cartão SD
Cartão SD

Funcionalidades:

Leitura de dados do MPU6050 (aceleração, giroscópio, ângulos)
Armazenamento em CSV no cartão SD (10Hz padrão)
Visualização em tempo real no display OLED
Controle por botões e terminal serial
Sistema de alertas com LEDs e buzzer

Conexões:

MPU6050: I2C0 (GPIO0-SDA, GPIO1-SCL)
OLED: I2C1 (GPIO14-SDA, GPIO15-SCL)
Botão A: GPIO5 (inicia/para captura)
Botão B: GPIO6 (monta/desmonta SD)
LEDs: GPIO11 (verde), GPIO12 (azul), GPIO13 (vermelho)
Buzzer: GPIO21
Cartão SD: Interface SPI

Comandos via Terminal Serial (115200 baud):

a - Montar cartão SD
b - Desmontar cartão SD
c - Listar arquivos
d - Ler arquivo CSV (ex: d mpu_data.csv)
e - Ver espaço livre
f - Formatar cartão SD
g - Ajuda (lista comandos)
h - Iniciar captura
i - Parar captura

Estrutura do Arquivo CSV:

text
Sample,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,Roll,Pitch
0,0.12,-0.45,0.98,1.23,-5.67,9.01,12.34,56.78
1,0.11,-0.22,0.33,4.44,-5.55,6.66,11.11,22.22
Estados do Sistema
Inicialização: LEDs piscam

SD montado: LED azul
Captura ativa: LEDs azul+vermelho
Erro: LED vermelho + buzzer

Como Usar:

Conectar componentes conforme diagrama
Inserir cartão SD formatado
Gravar firmware no Pico
Controlar via botões ou terminal serial

Link Github: https://github.com/Mateus-MDS/Medicao_e_Armazenamento_MPU.git
