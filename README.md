# Controle de Temperatura de Secadores com Arduino

## Descrição
Este projeto utiliza um Arduino para controlar a temperatura de dois secadores industriais, mantendo-a próxima de valores desejados (setpoints) definidos pelo usuário. O sistema lê a temperatura com sensores termopares, ajusta os secadores abrindo ou fechando relés e exibe informações em um display LCD e no monitor serial.

O objetivo é automatizar o controle térmico, garantindo eficiência e precisão em processos que dependem de temperaturas específicas, como secagem de materiais ou grãos.

## Componentes Utilizados
- **Arduino**: Placa principal para processar dados e controlar os dispositivos.
- **Sensores MAX6675**: Dois termopares para medir temperaturas entre 20°C e 300°C.
- **Relés**: Controlam a abertura e fechamento dos secadores (4 relés no total) e sinaleiros (2 relés).
- **Display LCD I2C (20x4)**: Mostra o setpoint, temperatura atual e status de cada secador.
- **Potenciômetros**: Dois potenciômetros para ajustar os setpoints (valores entre 70°C e 100°C).
- **Interruptores**: Dois interruptores para controle manual dos relés, se necessário.

## Funcionamento
1. **Leitura de Temperatura**: Os sensores MAX6675 medem a temperatura dos secadores a cada 300 ms, aplicando uma média móvel e filtro para suavizar os dados.
2. **Ajuste do Setpoint**: O usuário define o setpoint (0, 70, 80, 90 ou 100°C) girando os potenciômetros.
3. **Controle**: 
   - Se a temperatura está fora da faixa desejada (setpoint ± 2°C), o sistema aciona os relés por um tempo proporcional ao erro (de 2 a 7 segundos).
   - Para erros até 30°C, o tempo é calculado com `map()`. Acima disso, usa o máximo (7 segundos).
   - Há um intervalo mínimo de 20 segundos entre atuações para evitar sobrecarga.
4. **Monitoramento**: 
   - O LCD exibe o número do secador, setpoint, temperatura e status ("ON" ou "OFF").
   - O monitor serial mostra detalhes como temperatura, setpoint, erro e tempo de atuação.

## Como Usar
1. Conecte os componentes ao Arduino conforme os pinos definidos no código:
   - Sensores: Pinos 2-4 (termopar 1) e 5-7 (termopar 2).
   - Relés: Pinos 22-25 (motores) e 28-29 (sinaleiros).
   - Potenciômetros: Pinos A1 e A2.
   - Interruptores: Pinos 50 e 51.
   - LCD: Conexão I2C (endereço 0x27).
2. Carregue o código (`Controle_Temp_Secador.ino`) no Arduino usando o Arduino IDE.
3. Abra o monitor serial (9600 baud) para ver os dados em tempo real.
4. Ajuste os potenciômetros para definir os setpoints e observe o sistema controlar os secadores.

## Arquivos
- **`Controle_Temp_Secador.ino`**: Código principal do projeto.

## Melhorias Futuras
- Implementar controle PID para maior precisão.
- Criar uma interface gráfica para monitoramento remoto.

## Autor
Marlon Tomasini (`marlontomasini`) - Desenvolvido com ajuda do Grok (xAI).

## Licença
Este projeto é de código aberto e pode ser usado ou modificado livremente.
