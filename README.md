# ESP32-J2534 V6  —  Nível OEM + compat PSA/Diagalyzer

## O que mudou no V6 (V5 → V6)

V5 já era OEM-compliant para ISO-TP sobre J2534. V6 adiciona
compatibilidade com softwares que usam **CAN puro 11-bit** e esperam
comportamentos específicos da J2534 — especialmente **Diagalyzer**
(Peugeot/Citroën), que faz o ISO-TP ele mesmo e usa o adapter só como
pipe CAN.

| Mudança | V5 | V6 |
|---------|----|----|
| TX echo (loopback de transmissão) | ❌ ausente | ✅ FW emite `RSP_CAN_IND` com `MSG_FLAG_TX_ECHO` a cada `CMD_WRITE_CAN` |
| `RxStatus = TX_MSG_TYPE` nos ecos | ❌ | ✅ DLL marca `RxStatus` ao repassar eco |
| `calcRxId` fallback para 11-bit PSA | `txId+8` único | ✅ `0x7DF/0x7E0-0x7E7 → +8`, `0x740-0x7BF → -0x100` (PSA) |
| `READ_VBATT` | retornava `STATUS_NOERROR` sem preencher output | ✅ preenche 12000 mV no `pOutput` |
| `CAN_ID_BOTH (0x800)` | flag ignorado no log | ✅ reconhecido e logado `[PSA/Diagalyzer]` |
| Filter × eco | eco bloqueado pelos filtros | ✅ ecos passam sempre (são resposta ao próprio TX) |
| Baud rate | 4 valores fixos ou falha | ✅ 125/250/500/800/1000k + aproximação do mais próximo |
| Log TX no modo CAN puro | só logava ISO-TP | ✅ loga `TX CAN` e `RX CAN` |

Tudo que era V5 continua funcionando — V6 é estritamente aditivo.

---

## Por que o Diagalyzer precisa disso

O Diagalyzer (e vários outros softwares PSA via J2534) conecta como
`ProtocolID = CAN (0x05)`, não `ISO15765 (0x06)`. Ele faz o ISO-TP no
software, em cima do CAN raw. Para o software funcionar:

1. **Ele precisa ver o eco das próprias mensagens TX** — caso contrário
   não sabe se o adapter realmente transmitiu. Daí o TX echo.
2. **Ele registra filtros PASS para os IDs que quer ouvir** (ex: 752 e
   652 em um Peugeot). O eco não pode ser filtrado fora, porque o
   filtro PASS não cobre o próprio TX — por isso ecos passam sempre.
3. **Ele chama `READ_VBATT` no início para detectar se o OBD está
   energizado.** Se isso retornar lixo, o software desiste.

Addressing PSA típico:
- `752 ↔ 652` — BSI
- `75A ↔ 65A` — ABS
- `7A2 ↔ 6A2` — outro módulo
- etc.

---

## Fluxo CAN puro com TX echo (V6)

```
Diagalyzer → PassThruWriteMsgs(CAN, arbId=0x752, data=[...])
DLL      → CMD_WRITE_CAN
ESP32:
    twai_transmit()  → barramento
    send_tx_echo()   → RSP_CAN_IND com flag TX_ECHO
                       (o PC "ouve" sua própria transmissão)
DLL      → na próxima PassThruReadMsgs(), entrega o eco
           com RxStatus = TX_MSG_TYPE | (CAN_29BIT_ID se aplicável)
Diagalyzer → "OK, transmitido", espera resposta da ECU

ECU → 0x652 (resposta)
ESP32    → RSP_CAN_IND (sem flag TX_ECHO)
DLL      → filtra, entrega com RxStatus = 0
Diagalyzer → processa resposta
```

---

## Hardware

| Sinal   | GPIO | SN65HVD230 |
|---------|------|------------|
| CAN TX  | **33** | D (pino 1) |
| CAN RX  | **34** | R (pino 4) |
| S (mode)| —    | **GND** (obrigatório) |
| VCC     | 3.3V | VCC |
| GND     | GND  | GND |

120Ω CANH-CANL em cada extremo do barramento.

---

## Instalação

1. Grave `firmware/ESP32_J2534_V6.ino` no ESP32 (Arduino IDE,
   board ESP32 Dev Module, porta correta).
2. Edite `config.ini`: `COM_PORT=COMx` — troque pela porta real do ESP32.
3. Copie `config.ini` para `C:\esp32_j2534\`.
4. Compile a DLL (Win32, obrigatoriamente 32-bit):
   ```bat
   cd dll
   mkdir build
   cd build
   cmake .. -G "Visual Studio 17 2022" -A Win32
   cmake --build . --config Release
   mkdir C:\esp32_j2534\bin
   copy Release\esp32_j2534.dll C:\esp32_j2534\bin\
   ```
5. Registre o device no Windows: duplo-clique em
   `register_j2534.reg` → Sim → confirme no UAC.
6. Abra Diagalyzer → Options/Settings → J2534 device → selecione
   `ESP32 J2534 Bridge`.
7. Conecte-se ao veículo. No log (`C:\esp32_j2534\j2534_esp32_log.txt`)
   você deve ver linhas `TX CAN -> ID:00000752 | ...` seguidas de
   `TX echo -> ID:00000752 | ...` e depois `RX CAN <- ID:00000652 | ...`.

---

## Parâmetros configuráveis (Ioctl)

| Param J2534 | Param ESP32 | Padrão | Descrição |
|-------------|-------------|--------|-----------|
| `ISO15765_BS` / `BS` | `PARAM_BS` | 0 | Block Size (0 = sem limite) |
| `ISO15765_STMIN` / `STMIN` | `PARAM_STMIN` | 0 | STmin local em ms |
| `ISO15765_FRAME_PAD` (TxFlag) | `PARAM_TX_PADDING_EN/VAL` | off | Padding TX (PSA pede 0x00) |
| — | `PARAM_FC_TIMEOUT` | 1000 ms | N_Bs: aguarda FC da ECU |
| — | `PARAM_CF_TIMEOUT` | 1000 ms | N_Cr: aguarda cada CF |
| — | `PARAM_N_WAIT_MAX` | 10 | Máx de FC.FS=WAIT consecutivos |

---

## Log — novos marcadores V6

`C:\esp32_j2534\j2534_esp32_log.txt`

- `[DLL-V6]` — prefixo em todas as linhas (era `[DLL-V5]`)
- `Connect chId=... flags=0x... [11BIT] [PSA/Diagalyzer] [RAW]` — flags decodificados
- `TX CAN -> ID:... | hex` — transmissão em modo CAN puro
- `TX echo -> ID:... | hex` — eco devolvido ao PC (confirmação de TX)
- `RX CAN <- ID:... | hex` — frame recebido no modo CAN puro
- `TX UDS -> ID:... | hex` — transmissão ISO-TP (inalterado)
- `RX UDS <- ID:... | NB | hex` — resposta ISO-TP (inalterado)
- `READ_VBATT -> 12000mV (stub)` — stub do Ioctl de tensão

---

## Troubleshooting Diagalyzer

**Sintoma:** log mostra `Connect ... Close` sem TX/RX no meio.
→ Significa que o Diagalyzer desistiu antes de enviar qualquer frame.
   Geralmente é falha no `READ_VBATT` ou o software não encontrou o
   device no registro. V6 corrige o `READ_VBATT`.

**Sintoma:** TX aparece no log mas nenhum RX da ECU.
→ Barramento não responde: verifique terminação 120Ω, pinos CANH/CANL,
   chave do veículo em ACC/ON, e se a ECU responde a esse ID (use um
   scanner conhecido para confirmar).

**Sintoma:** TX e eco aparecem, RX aparece, mas Diagalyzer não
reconhece a ECU.
→ IDs errados. Veja quais pares `0x7xx ↔ 0x6xx` aparecem no log e
   confirme na documentação do modelo específico (BSI geralmente 752/652).
