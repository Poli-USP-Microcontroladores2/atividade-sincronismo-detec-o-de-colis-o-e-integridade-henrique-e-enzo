# PSI-Microcontroladores2-Aula12
Atividade: Sincronismo, Detecção de Colisão e Integridade

## Introdução

Na atividade anterior, vocês desenvolveram um código de comunicação serial que utiliza filas e interrupção para operar em dois modos: recepção por 5 segundos e transmissão por 5 segundos.

Nesta atividade, o objetivo é realizar em duplas a comunicação entre duas placas e refinar o protocolo de comunicação com sincronismo, detecção de colisão e verificação de integridade.

_Lembrete_: o código-base para a atividade anterior está disponível em: https://github.com/zephyrproject-rtos/zephyr/tree/main/samples/drivers/uart/echo_bot

## Etapa 1: Modelagem e Planejamento de Testes

Considerando o cenário proposto de comunicação entre duas placas com modo de operação simples de 5 segundos para transmitir e 5 segundos para receber, é natural que ocorram problemas de sincronismo: uma placa pode acabar transmitindo enquanto a outra está transmitindo também, e mesmo no recebimento podemos não receber a mensagem completa.

### 1.1. Sincronismo por Botão

#### Diagrama
<img src="docs/diagramas/Diagrama Botão.png" alt="Diagrama Botão">

---
#### Caso de Teste

    CT-01: Inicialização e Alternância Automática

    Objetivo: Verificar se o timer interno está controlando os estados TX/RX.

    Ação: Ligar as duas placas e não pressionar nada.

    Observação Visual: Observar os LEDs das placas e as mensagens no terminal por 15 segundos.

    Resultado Esperado:

    O terminal imprime --- [MODO TX] --- ou --- [MODO RX] --- a cada 5 segundos.

    Os LEDs alternam entre Verde (TX) e Azul (RX).

    Critério de Aceite: A alternância ocorre periodicamente e de forma contínua.

---

    CT-02: Sincronismo Forçado por Botão (Master Trigger)

    Objetivo: Verificar se o botão reseta o ciclo e alinha as duas placas (Mestre/Escravo).

    Pré-condição: As placas podem estar desalinhadas (ambas em TX ou ambas em RX).

    Ação: Pressionar o botão SW0 na Placa A.

    Resultado Esperado:

    Placa A (Mestre): O terminal exibe [BTN] Botao detectado!, força imediatamente o modo para TX (LED Verde) e envia o pacote SYNC_BTN.

    Placa B (Escravo): O terminal exibe [RX RECEBIDO]: SYNC_BTN seguido de >>> SYNC RECONHECIDO! <<<. O LED muda para RX (Azul) e reinicia seu timer interno.

    Critério de Aceite: Após o botão ser pressionado, a Placa A deve estar transmitindo e a Placa B escutando, garantindo o alinhamento do ciclo TDM.


### 1.2. Detecção de Colisão

#### Diagrama
<img src="docs/diagramas/Diagrama Colisão.png" alt="Diagrama Colisão">

---
#### Caso de Teste

    CT-01: Digitação em Modo de Escuta (Teste de Fila)

    Objetivo: Verificar se o sistema impede o envio imediato durante o modo RX e armazena a mensagem para depois.

    Pré-condição: Aguardar a Placa A entrar em Modo RX (LED Azul / Terminal: [RX ATIVO]).

    Ação: Digitar "Teste Fila" e pressionar Enter enquanto ainda estiver no modo RX.

    Resultado Esperado:

    A mensagem NÃO deve aparecer no terminal da Placa B imediatamente.

    O terminal da Placa A deve exibir: [Agendado] 'Teste Fila'.

    Ação Secundária: Aguardar a troca automática para Modo TX (LED Verde).

    Resultado Final:

    Assim que entrar em TX, a Placa A exibe: [TX Enviando]: Teste Fila.

    A Placa B recebe a mensagem imediatamente.

    Critério de Aceite: A transmissão física só ocorre quando o LED está Verde (TX), garantindo que o canal estava reservado para aquela placa.

### 1.3. Verificação de Integridade

#### Diagrama
<img src="docs/diagramas/Diagrama Integração.png" alt="Diagrama Integração">

---
#### Caso de Teste

    CT-01: Transmissão de Pacote Íntegro

    Objetivo: Verificar se o cálculo de Checksum e o empacotamento estão corretos.

    Ação: Na Placa A (durante modo TX), digitar "Ola" e dar Enter.

    Processamento Interno (Caixa Branca): O sistema calcula Header (0x7E) + Len (3) + Payload ("Ola") + Checksum.

    Resultado Esperado:

    A Placa B recebe os bytes.

    A thread de RX calcula o checksum localmente.

    O terminal da Placa B exibe: [RX RECEBIDO]: Ola.

    Critério de Aceite: A mensagem aparece limpa no receptor, sem caracteres de lixo.

## Etapa 2: Desenvolvimento Orientado a Testes

A partir da modelagem realizada e dos testes planejados, faça o desenvolvimento da solução para contemplar os 3 requisitos e passar nos 3 testes descritos.

O uso de IA Generativa é incentivado: _veja a diferença entre fazer prompts sem fornecer os requisitos e testes planejados, ou usar prompts com os diagramas e testes planejados_.

Além dos testes de cada requisito em cada etapa, faça **testes de regressão** também, para garantir que os requisitos das etapas anteriores estão funcionando (Dica: podemos ter modos de operação diferentes para testar diferentes features e não nos confundirmos com os comportamentos dos leds em cada situação).
Isto é: se o sincronismo continua funcionando após a integração da detecção de colisão, e se o sincronismo e a detecção continuam funcionando após a adição da verificação de integridade.

_Faça o upload de todos os códigos no repositório_ (pode ser em branches diferentes, ou até organizar em pull requests as diferentes features).

_Vocês devem adicionar todas as evidências de funcionamento (como por exemplo capturas de tela e fotos) dos testes realizados, mostrando todos os testes realizados no README.
As imagens e outras evidências de funcionamento devem estar descritas no README e devem estar em uma pasta chamada "results" no repositório._

### 2.1. Sincronismo por Botão

Insira aqui as descrições dos resultados e referencie as fotos e capturas de tela que mostram o funcionamento.

### 2.2. Detecção de Colisão

Insira aqui as descrições dos resultados e referencie as fotos e capturas de tela que mostram o funcionamento.

### 2.3. Verificação de Integridade

Insira aqui as descrições dos resultados e referencie as fotos e capturas de tela que mostram o funcionamento.
