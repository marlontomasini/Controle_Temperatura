// Inclui bibliotecas necessárias para o programa funcionar
#include <Arduino.h>          // Biblioteca principal do Arduino, essencial para usar funções como pinMode e digitalWrite
#include <Wire.h>             // Biblioteca para comunicação I2C, usada pelo display LCD
#include <LiquidCrystal_I2C.h> // Biblioteca para controlar o display LCD com I2C
#include "max6675.h"          // Biblioteca para os sensores de temperatura MAX6675

// Cria um objeto chamado "lcd" para controlar o display LCD
// O endereço 0x27 é o endereço I2C do LCD, e ele tem 20 colunas e 4 linhas
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Define constantes (valores fixos) que o programa vai usar
const int HISTERESE = 2;                // Margem de tolerância da temperatura (2°C acima ou abaixo do setpoint)
const unsigned int INTERVALO_ATUACAO = 20000; // Tempo mínimo entre ações dos relés (20 segundos, em milissegundos)
const int NUM_AMOSTRAS = 5;             // Quantas leituras de temperatura serão usadas para calcular a média
const int DELAY_ENTRE_LEITURAS = 300;   // Tempo de espera entre leituras dos sensores (300 milissegundos)
const float ALPHA = 0.3;                // Fator para suavizar a temperatura (valor entre 0 e 1, usado no filtro)

// Declara variáveis globais (disponíveis em todo o programa)
int setpoint1, setpoint2;               // Temperaturas desejadas (setpoints) para os secadores 1 e 2
int temp_termopar1, temp_termopar2;     // Temperaturas atuais lidas pelos sensores
unsigned long time_registro1 = 0, time_registro2 = 0; // Guarda o tempo da última ação dos relés
int erro1, erro2;                       // Diferença entre a temperatura atual e o setpoint
int tempo_ligado1, tempo_ligado2;       // Tempo que os relés ficam ligados (em milissegundos)

// Arrays (listas) para armazenar várias leituras de temperatura
float leituras_termopar1[NUM_AMOSTRAS] = {0}; // Lista para as últimas 5 leituras do sensor 1
float leituras_termopar2[NUM_AMOSTRAS] = {0}; // Lista para as últimas 5 leituras do sensor 2
int indice_leitura = 0;                 // Controla qual posição do array estamos preenchendo

// Variáveis para guardar temperaturas suavizadas (filtradas)
float temp_filtrada1 = 0, temp_filtrada2 = 0;

// Define os pinos do Arduino conectados aos relés (dispositivos que ligam/desligam coisas)
#define rele1 22  // Pino 22: Relé que abre o secador 1
#define rele2 23  // Pino 23: Relé que fecha o secador 1
#define rele3 24  // Pino 24: Relé que abre o secador 2
#define rele4 25  // Pino 25: Relé que fecha o secador 2
#define rele7 28  // Pino 28: Relé do sinaleiro do secador 1
#define rele8 29  // Pino 29: Relé do sinaleiro do secador 2
#define rele10 31 // Pino 31: Relé do sinaleiro verde geral

// Define os pinos dos interruptores (botões ou chaves)
#define switch1 50 // Pino 50: Interruptor do secador 1
#define switch2 51 // Pino 51: Interruptor do secador 2

// Cria objetos para os sensores de temperatura MAX6675
// Os números são os pinos do Arduino: SCK (clock), CS (seleção), SO (saída de dados)
MAX6675 ktc1(4, 3, 2);  // Sensor do termopar 1
MAX6675 ktc2(7, 6, 5);  // Sensor do termopar 2

// Declara funções que serão usadas (são como "tarefas" que o programa sabe fazer)
void desliga_reles_motor(); // Desliga os relés dos motores
void atualiza_variaveis();  // Atualiza valores e mostra no LCD
void controla_secador(int setpoint, int temp_termopar, int rele_abre, int rele_fecha, int rele_sinaleiro,
                      unsigned long &time_registro, int tempo_ligado); // Controla um secador
float ler_termopar_estabilizado(MAX6675 &sensor, float leituras[], float &temp_filtrada); // Lê temperatura com filtro
void controla_interruptor(int pino_switch, int pino_rele); // Controla interruptores

// Função setup: Executada uma vez quando o Arduino liga
void setup()
{
  // Configura os pinos dos relés (22 a 37) como saídas e desliga todos
  for (int i = 22; i <= 37; i++) {      // Loop: passa por cada pino de 22 até 37
    pinMode(i, OUTPUT);                // Define o pino como saída (vai enviar sinais)
    digitalWrite(i, HIGH);             // Desliga o relé (HIGH = desligado, por causa da lógica dos relés)
  }

  // Configura os pinos dos potenciômetros (entradas analógicas)
  pinMode(A1, INPUT);                  // Pino A1: Lê o potenciômetro do setpoint 1
  pinMode(A2, INPUT);                  // Pino A2: Lê o potenciômetro do setpoint 2

  // Configura os pinos dos interruptores como entradas
  pinMode(switch1, INPUT);             // Pino 50: Lê o interruptor 1
  pinMode(switch2, INPUT);             // Pino 51: Lê o interruptor 2

  // Inicia a comunicação serial (para ver mensagens no computador)
  Serial.begin(9600);                  // Velocidade de 9600 baud (padrão para debug)

  // Inicia o display LCD e liga a luz de fundo
  lcd.init();                          // Prepara o LCD para uso
  lcd.backlight();                     // Acende a luz do LCD

  // Faz a primeira leitura dos sensores para começar com valores válidos
  temp_filtrada1 = ktc1.readCelsius(); // Lê a temperatura do sensor 1
  temp_filtrada2 = ktc2.readCelsius(); // Lê a temperatura do sensor 2

  // Preenche os arrays de leituras com valores iniciais
  for (int i = 0; i < NUM_AMOSTRAS; i++) { // Loop: faz 5 leituras iniciais
    delay(DELAY_ENTRE_LEITURAS);          // Espera 300ms entre cada leitura
    leituras_termopar1[i] = ktc1.readCelsius(); // Armazena leitura do sensor 1
    leituras_termopar2[i] = ktc2.readCelsius(); // Armazena leitura do sensor 2
  }

  delay(500);                          // Espera 500ms para estabilizar tudo
  digitalWrite(rele10, LOW);           // Liga o sinaleiro verde (LOW = ligado)

  // Ativa o "pull-up" interno nos interruptores (evita leituras erradas)
  pinMode(switch1, INPUT_PULLUP);      // Pino 50 com pull-up
  pinMode(switch2, INPUT_PULLUP);      // Pino 51 com pull-up
}

// Função loop: Executada repetidamente enquanto o Arduino está ligado
void loop()
{
  // Controla os interruptores e seus relés
  controla_interruptor(switch1, rele1); // Interruptor 1 controla o rele1
  controla_interruptor(switch2, rele3); // Interruptor 2 controla o rele3

  atualiza_variaveis();                // Atualiza valores e mostra no LCD

  // Controla os secadores com base nas temperaturas
  controla_secador(setpoint1, temp_termopar1, rele1, rele2, rele7, time_registro1, tempo_ligado1);
  desliga_reles_motor();               // Desliga os relés após o controle

  controla_secador(setpoint2, temp_termopar2, rele3, rele4, rele8, time_registro2, tempo_ligado2);
  desliga_reles_motor();               // Desliga os relés novamente
}

// Função que controla um interruptor e seu relé
void controla_interruptor(int pino_switch, int pino_rele)
{
  // Enquanto o interruptor estiver ligado (LOW, ou 0), mantém o relé ligado
  while (digitalRead(pino_switch) == 0) { // Lê o pino do interruptor
    digitalWrite(pino_rele, LOW);         // Liga o relé (LOW = ligado)
    delay(100);                          // Espera 100ms para não travar o programa
  }
  digitalWrite(pino_rele, HIGH);         // Desliga o relé quando o interruptor é solto
}

// Função que lê a temperatura de um sensor com suavização
float ler_termopar_estabilizado(MAX6675 &sensor, float leituras[], float &temp_filtrada)
{
  float leitura_atual = sensor.readCelsius(); // Lê a temperatura atual do sensor
  // Verifica se a leitura é válida (entre 0 e 300°C)
  if (leitura_atual < 0 || leitura_atual > 300) {
    return temp_filtrada;                    // Se inválida, retorna o valor anterior
  }

  leituras[indice_leitura] = leitura_atual;  // Coloca a leitura no array
  float soma = 0;                            // Variável para somar as leituras
  for (int i = 0; i < NUM_AMOSTRAS; i++) {   // Loop: soma todas as leituras do array
    soma += leituras[i];
  }
  float media = soma / NUM_AMOSTRAS;         // Calcula a média das 5 leituras
  // Aplica um filtro para suavizar (mistura a média nova com o valor antigo)
  temp_filtrada = ALPHA * media + (1 - ALPHA) * temp_filtrada;
  return temp_filtrada;                      // Retorna a temperatura suavizada
}

// Função que controla um secador
void controla_secador(int setpoint, int temp_termopar, int rele_abre, int rele_fecha, int rele_sinaleiro,
                      unsigned long &time_registro, int tempo_ligado)
{
  if (setpoint < 70) {                       // Se o setpoint for menor que 70°C
    digitalWrite(rele_sinaleiro, HIGH);      // Desliga o sinaleiro
  } else {
    // Liga o sinaleiro se a temperatura estiver 5°C abaixo do setpoint
    digitalWrite(rele_sinaleiro, (temp_termopar < setpoint - 5) ? LOW : HIGH);
    // Verifica se a temperatura está fora da faixa aceitável
    if (temp_termopar > (setpoint + HISTERESE) || temp_termopar < (setpoint - HISTERESE)) {
      // Verifica se já passou o tempo mínimo desde a última ação
      if ((millis() - time_registro) >= INTERVALO_ATUACAO) {
        if ((temp_termopar - (setpoint - HISTERESE)) > 1) { // Se estiver muito quente
          digitalWrite(rele_abre, LOW);                    // Abre o secador
          delay(tempo_ligado);                             // Espera o tempo calculado
          digitalWrite(rele_abre, HIGH);                   // Desliga o relé
        } else {                                           // Se estiver muito frio
          digitalWrite(rele_fecha, LOW);                   // Fecha o secador
          delay(tempo_ligado);                             // Espera o tempo calculado
          digitalWrite(rele_fecha, HIGH);                  // Desliga o relé
        }
        time_registro = millis();                          // Atualiza o tempo da última ação
      }
    }
  }
}

// Função que atualiza variáveis e exibe informações
void atualiza_variaveis()
{
  // Lê os potenciômetros (valores de 0 a 1023)
  int leituraA1 = analogRead(A1);           // Lê o potenciômetro do setpoint 1
  int leituraA2 = analogRead(A2);           // Lê o potenciômetro do setpoint 2

  // Converte as leituras em setpoints (0, 70, 80, 90 ou 100°C)
  setpoint1 = (leituraA1 < 200) ? 0 : ((leituraA1 < 400) ? 70 : ((leituraA1 < 600) ? 80 : ((leituraA1 < 800) ? 90 : 100)));
  setpoint2 = (leituraA2 < 200) ? 0 : ((leituraA2 < 400) ? 70 : ((leituraA2 < 600) ? 80 : ((leituraA2 < 800) ? 90 : 100)));

  indice_leitura = (indice_leitura + 1) % NUM_AMOSTRAS; // Atualiza o índice do array (roda em círculo)

  // Lê as temperaturas dos sensores com suavização
  temp_termopar1 = ler_termopar_estabilizado(ktc1, leituras_termopar1, temp_filtrada1);
  delay(DELAY_ENTRE_LEITURAS);              // Espera 300ms antes da próxima leitura
  temp_termopar2 = ler_termopar_estabilizado(ktc2, leituras_termopar2, temp_filtrada2);

  // Calcula o erro (diferença entre temperatura atual e setpoint)
  erro1 = abs(temp_termopar1 - setpoint1);  // abs() dá o valor absoluto
  erro2 = abs(temp_termopar2 - setpoint2);

  // Define o tempo de atuação com base no erro
  if (erro1 > 30) {
    tempo_ligado1 = 7000;                   // Erro grande: 7 segundos
  } else {
    tempo_ligado1 = map(erro1, 0, 30, 2000, 7000); // Mapeia o erro de 2 a 7 segundos
  }
  if (erro2 > 30) {
    tempo_ligado2 = 7000;
  } else {
    tempo_ligado2 = map(erro2, 0, 30, 2000, 7000);
  }

  // Mostra informações no monitor serial (para debug)
  Serial.println("-------------------------");
  Serial.print("Secador 1 - Temp: "); Serial.print(temp_termopar1);
  Serial.print(" | Setpoint: "); Serial.print(setpoint1);
  Serial.print(" | Erro: "); Serial.print(erro1);
  Serial.print(" | Tempo de Atuacao: "); Serial.println(tempo_ligado1);

  Serial.print("Secador 2 - Temp: "); Serial.print(temp_termopar2);
  Serial.print(" | Setpoint: "); Serial.print(setpoint2);
  Serial.print(" | Erro: "); Serial.print(erro2);
  Serial.print(" | Tempo de Atuacao: "); Serial.println(tempo_ligado2);
  Serial.println("-------------------------");

  // Mostra informações no display LCD
  lcd.clear();                              // Limpa o display
  lcd.setCursor(0, 0);                      // Posiciona o cursor na linha 1, coluna 1
  lcd.print("Sec Set Temp Status");         // Cabeçalho do display
  lcd.setCursor(0, 1); lcd.print("1");      // Mostra "1" para secador 1
  lcd.setCursor(4, 1); lcd.print(setpoint1); // Mostra o setpoint 1
  lcd.setCursor(9, 1); lcd.print(temp_termopar1); // Mostra a temperatura 1
  lcd.setCursor(14, 1); lcd.print((setpoint1 >= 70) ? "ON " : "OFF"); // Mostra status
  lcd.setCursor(0, 2); lcd.print("2");      // Mostra "2" para secador 2
  lcd.setCursor(4, 2); lcd.print(setpoint2); // Mostra o setpoint 2
  lcd.setCursor(9, 2); lcd.print(temp_termopar2); // Mostra a temperatura 2
  lcd.setCursor(14, 2); lcd.print((setpoint2 >= 70) ? "ON " : "OFF"); // Mostra status
}

// Função que desliga os relés dos motores
void desliga_reles_motor()
{
  digitalWrite(rele1, HIGH);                // Desliga o rele1
  digitalWrite(rele2, HIGH);                // Desliga o rele2
  digitalWrite(rele3, HIGH);                // Desliga o rele3
  digitalWrite(rele4, HIGH);                // Desliga o rele4
}