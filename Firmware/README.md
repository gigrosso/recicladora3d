# Sistema de Controle de Extrusora - Raspberry Pi Pico W

## Descrição
Sistema completo para controle de extrusora de plástico (ABS, PLA, PET) utilizando Raspberry Pi Pico W.

## Componentes Utilizados

### Hardware Principal
- **Microcontrolador**: Raspberry Pi Pico W
- **Multiplexadores**: 2x CD74HC4067E (16 canais)
- **Sensores de Temperatura**: 4x PT100 (WZP)
- **Sensor de Nível**: Ultrassônico HC-SR04
- **MOSFETs**: 4x IRLB3034226P4Z
- **Motor**: Nema 23
- **Driver de Motor**: TB6600
- **Display**: LCD JHD 162A com módulo I2C

### Sensores de Temperatura (PT100)
1. Bico da extrusora
2. Zona de dosagem
3. Zona de compressão
4. Zona de alimentação

## Mapeamento de Pinos

### Pinos Preservados (UART)
- GP0: UART0_TX (Saída Serial)
- GP1: UART0_RX (Entrada Serial)

### Multiplexador 1 (Botões)
- GP2-5: Sinais S0-S3
- GP26: Sinal (ADC0)

### Multiplexador 2 (LEDs)
- GP6-9: Sinais S0-S3
- GP10: Sinal de saída

### Sensores PT100 (ADC)
- GP27: Sensor 1 (ADC1)
- GP28: Sensor 2 (ADC2)
- GP29: Sensor 3 (ADC3)

### MOSFETs
- GP11: MOSFET1 - Aquecimento zona de dosagem
- GP12: MOSFET2 - Aquecimento zona de compressão
- GP13: MOSFET3 - Resfriamento zona de alimentação
- GP14: MOSFET4 - Alimentação do motor

### Motor TB6600
- GP15: STEP (Pulso de passo)
- GP16: DIR (Direção)
- GP17: ENABLE (Habilitação)

### LCD I2C
- GP20: SDA (I2C0)
- GP21: SCL (I2C0)
- Endereço I2C: 0x27

### Sensor Ultrassônico
- GP18: TRIG (Trigger)
- GP19: ECHO (Echo)

## Botões (Multiplexador 1)

| Canal | Função |
|-------|--------|
| I0 | Ativa/Atualiza/Reativa sistema |
| I1 | Altera unidade (°C/°F) |
| I2 | Aumenta temperatura sensor 1 |
| I3 | Diminui temperatura sensor 1 |
| I4 | Aumenta temperatura sensor 2 |
| I5 | Diminui temperatura sensor 2 |
| I6 | Aumenta temperatura sensor 3 |
| I7 | Diminui temperatura sensor 3 |
| I8 | Aumenta temperatura sensor 4 |
| I9 | Diminui temperatura sensor 4 |
| I10 | Para extrusão |
| I11 | Inicia extrusão |
| I12 | Seleciona material |
| I13 | Para tudo (emergência) |
| I14 | Toggle motor (temporário - teste) |
| I15 | Aumenta RPM (temporário - teste) |

## LEDs (Multiplexador 2)

| Canal | Função |
|-------|--------|
| I0 | Sistema ligado |
| I1 | Aquecendo sensor 2 |
| I2 | Aquecendo sensor 3 |
| I3 | Temperatura alvo 1 atingida |
| I4 | Temperatura alvo 2 atingida |
| I5 | Temperatura alvo 3 atingida |
| I6 | Temperatura 4 dentro do limite |
| I7 | Unidade 1 selecionada (°C) |
| I8 | Unidade 2 selecionada (°F) |
| I9 | Material acabando |
| I10 | Material acabou |
| I11 | Temperatura alta (perigo) |

## Materiais Suportados

### ABS
- Extrusor: 210-250°C
- Zona de Dosagem: 180-220°C
- Zona de Compressão: 160-200°C
- Zona de Alimentação: Máx. 100°C

### PLA
- Extrusor: 180-220°C
- Zona de Dosagem: 160-200°C
- Zona de Compressão: 140-180°C
- Zona de Alimentação: Máx. 70°C

### PET
- Extrusor: 230-270°C
- Zona de Dosagem: 200-240°C
- Zona de Compressão: 180-220°C
- Zona de Alimentação: Máx. 100°C

### SMD (Sem Material Definido)
- Configuração manual de todas as temperaturas

## Display LCD

### Primeira Linha
```
M:XXX E123/123
```
- M:XXX - Material selecionado (ABS, PLA, PET, SMD)
- E123 - Temperatura atual do extrusor
- /123 - Temperatura alvo do extrusor

### Segunda Linha
```
D123/123C123/123
```
- D123/123 - Dosagem (atual/alvo)
- C123/123 - Compressão (atual/alvo)

## Segurança

### Condições de Erro
- Sem mudança de temperatura em 5 minutos
- Quando erro ativo, display mostra apenas "ERRO"

### Proteções
- Motor só liga quando temperaturas estão 5% abaixo da faixa configurada
- Resfriamento automático quando temperatura de alimentação excede máximo
- Botão de emergência (I13) para tudo

## Compilação

### Pré-requisitos
1. Instalar Pico SDK
2. Configurar variável de ambiente PICO_SDK_PATH
3. Instalar CMake e compilador ARM

### Passos
```bash
# Clone o repositório ou copie os arquivos
mkdir build
cd build
cmake ..
make
```

### Resultado
O arquivo `extrusora_controller.uf2` será gerado na pasta build.

## Instalação no Raspberry Pi Pico W

1. Segure o botão BOOTSEL no Pico W
2. Conecte o cabo USB ao computador
3. Solte o botão BOOTSEL
4. O Pico W aparecerá como dispositivo de armazenamento
5. Copie o arquivo `extrusora_controller.uf2` para o dispositivo
6. O Pico W reiniciará automaticamente

## Calibração

### Sensores PT100
Os sensores PT100 precisam ser calibrados. No código, ajuste a função `ler_temperatura_pt100()` conforme seu circuito:
- Verifique o divisor de tensão usado
- Ajuste a constante de resistência (0.385 Ω/°C é o valor padrão)
- Calibre com termômetro de referência

### Sensor Ultrassônico
Ajuste a distância máxima na função `ler_distancia_ultrasonic()` conforme a altura do seu reservatório.

### Motor
Configure os micropassos no driver TB6600 conforme necessário:
- Padrão: 200 passos/revolução
- Com micropassos: ajuste o cálculo em `motor_step()`

## Operação

### Inicialização
1. Ligue o sistema
2. Pressione I0 para ativar
3. Selecione o material com I12
4. Aguarde aquecimento

### Durante Operação
- LEDs indicam status de aquecimento
- Display mostra temperaturas em tempo real
- Sistema controla automaticamente aquecimento/resfriamento

### Extrusão
1. Certifique-se que temperaturas estão corretas
2. Pressione I11 para iniciar extrusão
3. Motor começará a girar automaticamente
4. Pressione I10 para parar extrusão

### Emergência
Pressione I13 para parar tudo imediatamente:
- Desliga aquecimento
- Para motor
- Desativa sistema

## Notas Importantes

1. **Pinos UART**: GP0 e GP1 estão reservados para comunicação serial
2. **Modo Teste**: Botões I14 e I15 são temporários para teste do motor
3. **Segurança**: Sempre monitore o sistema durante operação
4. **Calibração**: Calibre os sensores antes do uso em produção

## Troubleshooting

### Display não funciona
- Verifique endereço I2C (pode ser 0x27 ou 0x3F)
- Teste conexões SDA e SCL

### Temperaturas erradas
- Calibre os sensores PT100
- Verifique divisor de tensão
- Ajuste fórmula de conversão

### Motor não gira
- Verifique configuração do TB6600
- Confirme pinos STEP, DIR e ENABLE
- Teste micropassos

### Botões não respondem
- Verifique resistores pull-up
- Teste multiplexador
- Confirme alimentação 5V

## Licença
Código desenvolvido para fins educacionais e industriais.

## Suporte
Para dúvidas ou melhorias, consulte a documentação do Raspberry Pi Pico SDK:
https://www.raspberrypi.com/documentation/microcontrollers/
