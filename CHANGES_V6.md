# CHANGES V5 → V6

## Objetivo
Compatibilizar o ESP32-J2534 com **Diagalyzer** (Peugeot/Citroën) e
outros softwares J2534 que usam **CAN puro 11-bit** com ISO-TP feito
em software no próprio aplicativo.

## Contexto
Log que motivou as mudanças (tentativa V5 com Diagalyzer):
```
[DLL-V5] Connect chId=1 proto=5 baud=500000 flags=0x00000800
[DLL-V5] Filter fid=1 type=0 mask=FFFFFFFF pat=00000000 fc=00000000
[DLL-V5] Filter fid=2 type=0 mask=FFFFFFFF pat=00000752 fc=00000000
[DLL-V5] Filter fid=3 type=0 mask=FFFFFFFF pat=00000652 fc=00000000
[DLL-V5] Close
```
- `proto=5` = CAN puro (não ISO15765). O Diagalyzer faz ISO-TP sozinho.
- `flags=0x00000800` = `CAN_ID_BOTH`, pede TX+RX 11-bit.
- Filtros são todos `PASS_FILTER` (nenhum FlowControl).
- Diagalyzer desconectou sem trafegar nada → algo travou antes.

## Arquivos modificados

### `protocol.h` + `firmware/protocol.h`
- Adicionado `MSG_FLAG_TX_ECHO = (1u << 2)` — flag usado no campo
  `flags` de `RSP_CAN_IND` para indicar que o frame é eco do próprio
  TX, não um frame recebido do barramento.

### `firmware/ESP32_J2534_V6.ino`  (era `ESP32_J2534_V5.ino`)
- Nova função `send_tx_echo(id, data, dlc, flags)`: envia um
  `RSP_CAN_IND` ao PC com `MSG_FLAG_TX_ECHO` ligado, replicando a
  mensagem que acabou de ser transmitida (respeitando padding se
  habilitado).
- `handle_write_can()`: agora chama `send_tx_echo()` após um
  `twai_transmit()` bem-sucedido. O eco **não** é enviado se a
  transmissão falhar.
- `handle_connect()`: aceita qualquer baud rate aproximando para o
  padrão mais próximo (125k / 250k / 500k / 800k / 1M), em vez de
  rejeitar valores não-exatos com `STATUS_ERR_PARAM`.
- Cabeçalho atualizado para V6.

### `dll/include/j2534.h`
- Novos defines:
  - `CAN_ID_BOTH = 0x00000800` (flag PSA/Diagalyzer)
  - `ISO9141_NO_CHECKSUM = 0x00000200` (padrão, não usado pelo device)

### `dll/src/j2534.cpp`
- Prefixo dos logs mudou de `[DLL-V5]` para `[DLL-V6]`.
- `PassThruReadVersion`: versão bumpada para `06.00.00.001`.
- `calcRxId()`: fallback por addressing refeito
  - `0x7DF` e `0x7E0-0x7E7` → `+8` (OBD-II clássico)
  - `0x740-0x7BF` → `-0x100` (PSA: 752→652, 75A→65A, 7A2→6A2)
  - Outros 11-bit → `+8` (fallback legado)
  - Continua priorizando `fcId` do filtro quando existe.
- `PassThruConnect`: log agora decodifica flags —
  `[11BIT] [29BIT] [PSA/Diagalyzer] [RAW] [ISO-TP]`.
- `PassThruWriteMsgs`: adicionado `logDbg("TX CAN -> ID:...")` no
  caminho de CAN puro (antes só ISO-TP era logado).
- `PassThruReadMsgs`:
  - Detecta `MSG_FLAG_TX_ECHO` no frame recebido.
  - Marca `out.RxStatus |= TX_MSG_TYPE` quando é eco.
  - **Ecos passam pelos filtros sem checagem** (software precisa ver
    seus próprios TXs independente do filtro PASS configurado).
  - Log diferenciado: `TX echo ->` para ecos, `RX CAN <-` para
    frames do barramento.
- `PassThruIoctl`:
  - `READ_VBATT`: agora escreve `12000` em `*(ULONG*)pOutput` (12 V
    stub) em vez de retornar `STATUS_NOERROR` sem preencher nada.
    Apps PSA travam quando `pOutput` fica com lixo.

## V6 hotfix #2 (TransferData / flash)

**Sintoma:** durante flash de software (serviço UDS `36 01` com payload
~3700 bytes), Diagalyzer retransmite a mesma mensagem várias vezes.

**Causa raiz:** o ISO-TP TX de 3700 bytes vira ~528 Consecutive Frames.
Com STmin típico da ECU em flash (10-20 ms), o tempo total do TX
chegava a 5-10+ segundos. O timeout do `sendCmd` da DLL para
`CMD_WRITE_ISOTP` era 10 s — apertado, podendo expirar antes do
firmware terminar e retornar o ACK final.

**Causas secundárias possíveis (mitigadas):**
- Buffer USB CDC RX do ESP32 ficava no default (~256 B); a 2 Mbps
  enche em 1.3 ms. Se o PC manda outro comando enquanto o firmware
  está bloqueado em STmin, bytes podem ser perdidos.
- Sem visibilidade de **qual** erro o firmware reportou — só "falhou".

**Fixes:**
1. **DLL** — timeout de `CMD_WRITE_ISOTP` aumentado para
   `max(Timeout, 120000)` ms (2 min).
2. **DLL** — `sendCmdRaw` que retorna o byte cru de status do firmware,
   e log detalhado: `ISO-TP TX falhou -> ERR 0x04 (TIMEOUT (FC nao
   chegou da ECU))` etc. Distingue claramente N_Bs vs OVFLW vs
   WAIT_LIMIT vs BUSY vs HW vs sem ACK.
3. **Firmware** — `Serial.setRxBufferSize(8192)` e
   `setTxBufferSize(2048)` em `setup()` antes de `Serial.begin`.
   Folga ampla para comandos `CMD_WRITE_ISOTP` de até ~4 kB.

**Como interpretar o novo log:**
- `ERR 0x04 (TIMEOUT (FC nao chegou da ECU))` → ECU não mandou Flow
  Control depois do First Frame. Cheque pares de IDs no filtro FC e
  `STATUS_ERR_TIMEOUT` no firmware (variável `g_fc_timeout_ms`).
- `ERR 0x05 (ISOTP (FC=OVFLW))` → ECU disse que não tem buffer pro
  payload. Reduzir tamanho de bloco no software de diagnóstico.
- `ERR 0x06 (WAIT_LIMIT)` → ECU pediu WAIT mais de `N_WAIT_MAX` vezes
  (default 10). Pode aumentar via Ioctl `PARAM_N_WAIT_MAX`.
- `ERR 0x01 (BUSY)` → fila TWAI cheia ou CAN não iniciado.
- `ERR 0xFF (NO ACK)` → firmware travou, USB caiu, ou timeout serial
  (>120 s).

---


---

## V6 hotfix #1 (FC após FF — multi-frame)

**Bug:** multi-frame TX (ex: `2E F1 84 ...` com 16 bytes de payload)
travava com timeout esperando o Flow Control da ECU. Causa: `calcRxId`
retornava `f.fcId` em vez de `f.patId` quando havia filtro FlowControl.

**Semântica J2534 correta:**
- `pPatternMsg` (patId) = ID que a ECU usa para responder (rxId)
- `pFlowControlMsg` (fcId) = txId do tester (correlação ECU→FC)

**Fix:** `calcRxId` agora procura um filtro FC cujo `fcId` bate com o
`txId` que estamos transmitindo, e retorna o `patId` daquele filtro.
Fallback secundário: qualquer FC filter com `patId` definido. Por
último, fallback por addressing 29-bit/PSA/OBD.

**Bonus:** log de TX UDS agora mostra também o rxId calculado,
formato `TX UDS -> ID:18DA40F1 (rx:18DAF140) | ...`.

---

- Toda a máquina de estados ISO-TP do firmware (SF/FF/CF/FC, SN,
  N_Bs, N_Cr, BS, STmin, WAIT/OVFLW).
- Scheduler de mensagens periódicas.
- Formato dos frames serial PC↔ESP32.
- Todos os comandos `CMD_*` e códigos de erro existentes.
- Hardware e pinagem.

## Como reverter para V5 se precisar
Basta usar a pasta `v5_final/` original (preservada). O V6 está em
pasta separada e o firmware renomeado para `ESP32_J2534_V6.ino`.

## Teste esperado com Diagalyzer após V6
Log esperado de uma sessão que funciona:
```
[DLL-V6] === PassThruOpen V6 ===
[DLL-V6] COM_PORT=COM5
[DLL-V6] OK
[DLL-V6] Connect chId=1 proto=5 baud=500000 flags=0x00000800 [11BIT] [PSA/Diagalyzer] [RAW]
[DLL-V6] READ_VBATT -> 12000mV (stub)
[DLL-V6] Filter fid=1 type=0 mask=FFFFFFFF pat=00000000 fc=00000000
[DLL-V6] Filter fid=2 type=0 mask=FFFFFFFF pat=00000752 fc=00000000
[DLL-V6] Filter fid=3 type=0 mask=FFFFFFFF pat=00000652 fc=00000000
[DLL-V6] TX CAN -> ID:00000752 | 02 10 03 00 00 00 00 00
[DLL-V6] TX echo -> ID:00000752 | 02 10 03 00 00 00 00 00
[DLL-V6] RX CAN <- ID:00000652 | 06 50 03 00 32 01 F4 00
...
```
Se o log mostrar TX + echo + RX nessa ordem, o Diagalyzer está
conversando com a ECU via o adapter.
