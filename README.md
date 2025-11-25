# PSI-Microcontroladores2-Aula12
Atividade: Sincronismo, Detecção de Colisão e Integridade

## Introdução

Na atividade anterior, vocês desenvolveram um código de comunicação serial que utiliza filas e interrupção para operar em dois modos: recepção por 5 segundos e transmissão por 5 segundos.

Nesta atividade, o objetivo é realizar em duplas a comunicação entre duas placas e refinar o protocolo de comunicação com sincronismo, detecção de colisão e verificação de integridade.

_Lembrete_: o código-base para a atividade anterior está disponível em: https://github.com/zephyrproject-rtos/zephyr/tree/main/samples/drivers/uart/echo_bot

## Etapa 1: Modelagem e Planejamento de Testes

Considerando o cenário proposto de comunicação entre duas placas com modo de operação simples de 5 segundos para transmitir e 5 segundos para receber, é natural que ocorram problemas de sincronismo: uma placa pode acabar transmitindo enquanto a outra está transmitindo também, e mesmo no recebimento podemos não receber a mensagem completa.

### 1.1. Sincronismo por Botão

<img src="docs/diagramas/Diagrama Botão.png" alt="Diagrama Botão">

#### Caso de Teste
Teste Botão:

Leds
- Leds indicam o estado do protocolo

Modos
- Uma placa emite um pulso quando a outra receber o pulso alterna o modo.

Botão
- Força o estado das placas para um determinado modo e reinicia a contagem do tempo.


### 1.2. Detecção de Colisão

Teste Colisão:

Modo
- Ambas as placas iniciam no modo RX


CT-Colisão – Detecção de Canal Ocupado

Entrada:

Placa B começa a transmitir (Segurar botão na B).

Enquanto B transmite, pressionar Botão na Placa A.

Saída Esperada:

A Placa A verifica que o RX está recebendo bits.

A Placa A NÃO acende o LED de TX.

A Placa A mantém o LED de RX (ou pisca um LED de Erro rápido).

Critério de Aceitação:

A transmissão da Placa B não é corrompida.

A Placa A respeita o tráfego existente e não "atropela" a comunicação.


CT2 – Disparo de Sincronismo (Master Trigger)
Este teste verifica se o botão força a placa local a transmitir e se a placa remota obedece ao comando de troca de modo.

Entrada:

Pressionar o botão na Placa A.

Saída Esperada:

Placa A: O LED altera imediatamente para o estado TX e inicia a contagem do tempo de transmissão.

Placa B: Ao receber o "pulso" (mensagem de sync) da Placa A, o LED da Placa B deve confirmar a entrada no modo RX (ou piscar brevemente para indicar que o contador interno foi reiniciado/sincronizado).

Critério de Aceitação:

Ocorre a mudança visual de estado na Placa A (Transmissor) e, com atraso imperceptível, a reação visual na Placa B (Receptor), confirmando o sincronismo.


CT1 – Inicialização em Modo RX (Silêncio)
Este teste garante que o sistema começa em um estado seguro e conhecido por um breve período.

Entrada:

Ligar a alimentação de ambas as placas (Placa A e Placa B).

Não pressionar nenhum botão.

Saída Esperada:

Os LEDs de ambas as placas devem indicar o estado RX (Ex: LEDs apagados ou acesos em uma cor específica de "escuta").

Critério de Aceitação:

As placas permanecem estáticas em modo de recepção até o fim do tempo estabelecido.

### 1.3. Verificação de Integridade

CT3 – Verificação de Integridade (Rejeição de Mensagem Corrompida)
Entrada:

Configurar a Placa A para enviar um pacote "malformado" intencional para a Placa B.

Ação: A Placa A envia um cabeçalho declarando um tamanho de mensagem de 5 bytes, mas envia efetivamente apenas 3 bytes de dados (ex: string "ABC") e encerra a transmissão.

Saída esperada:

A Placa B recebe os 3 bytes, aguarda brevemente pelos bytes faltantes (timeout) e percebe que a conta não fecha.

A Placa B NÃO deve acionar o LED de "Sucesso" (que acenderia numa mensagem íntegra).

A Placa B deve acionar o indicador visual de ERRO (ex: piscar o LED rapidamente ou acender um LED de cor diferente) e descartar os dados armazenados no buffer.

Critério de Aceitação:

O sistema receptor deve ser capaz de distinguir entre um pacote completo e um incompleto, garantindo que mensagens com divergência de tamanho sejam rejeitadas e não processadas pela aplicação.

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
