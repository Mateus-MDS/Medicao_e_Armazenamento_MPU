Data Logger com MPU6050 e Raspberry Pi Pico

Descri��o:

Sistema de aquisi��o de dados de movimento usando sensor MPU6050 (aceler�metro + girosc�pio) com armazenamento em cart�o SD e interface OLED. Projeto ideal para monitoramento de vibra��o e an�lise de movimento.

Componentes Principais:

BitDogLab
Sensor MPU6050
suporte para cart�o SD
Cart�o SD

Funcionalidades:

Leitura de dados do MPU6050 (acelera��o, girosc�pio, �ngulos)
Armazenamento em CSV no cart�o SD (10Hz padr�o)
Visualiza��o em tempo real no display OLED
Controle por bot�es e terminal serial
Sistema de alertas com LEDs e buzzer

Conex�es:

MPU6050: I2C0 (GPIO0-SDA, GPIO1-SCL)
OLED: I2C1 (GPIO14-SDA, GPIO15-SCL)
Bot�o A: GPIO5 (inicia/para captura)
Bot�o B: GPIO6 (monta/desmonta SD)
LEDs: GPIO11 (verde), GPIO12 (azul), GPIO13 (vermelho)
Buzzer: GPIO21
Cart�o SD: Interface SPI

Comandos via Terminal Serial (115200 baud):

a - Montar cart�o SD
b - Desmontar cart�o SD
c - Listar arquivos
d - Ler arquivo CSV (ex: d mpu_data.csv)
e - Ver espa�o livre
f - Formatar cart�o SD
g - Ajuda (lista comandos)
h - Iniciar captura
i - Parar captura

Estrutura do Arquivo CSV:

text
Sample,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,Roll,Pitch
0,0.12,-0.45,0.98,1.23,-5.67,9.01,12.34,56.78
1,0.11,-0.22,0.33,4.44,-5.55,6.66,11.11,22.22
Estados do Sistema
Inicializa��o: LEDs piscam

SD montado: LED azul
Captura ativa: LEDs azul+vermelho
Erro: LED vermelho + buzzer

Como Usar:

Conectar componentes conforme diagrama
Inserir cart�o SD formatado
Gravar firmware no Pico
Controlar via bot�es ou terminal serial

Link Github: 
Link Video: