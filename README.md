# G.E.C.K. - Garden of Eden Creation Kit (Versão Embarcada)

## Sobre o Projeto
O **G.E.C.K. (Garden of Eden Creation Kit)** é um sistema embarcado inspirado no universo de Fallout, projetado para gerenciamento de bunkers e suporte à vida em ambientes inóspitos. Com sensores ambientais e atuadores, ele monitora e ajusta as condições internas para garantir a segurança dos residentes.

Este projeto foi desenvolvido como parte da disciplina de **Sistemas Embarcados (2024.2)**, utilizando **FreeRTOS** e implementado no **PlatformIO (VS Code)**. O sistema foi simulado no **Wokwi** para testes.

---

## Funcionalidades
- **Monitoramento Ambiental:**
  - **RadScan-3000**: Sensor fictício de radiação.
  - **DHT22**: Medidor de temperatura e umidade.
  - **NTC**: Sensor de temperatura de alta precisão.
  - **MPU6050**: Acelerômetro e giroscópio para detecção de movimentos.

- **Atuação Inteligente:**
  - **Servo Motor**: Controle de válvulas ou mecanismos automatizados.
  - **Relé**: Controle de sistemas elétricos internos.
  - **LEDs de Alerta**: Indicação visual do status ambiental (verde, laranja e vermelho).

- **Interface e Navegação:**
  - **LCD 20x4** com múltiplas telas interativas.
  - **Botões UP, DOWN e SELECT** para navegação entre menus.

- **Gerenciamento de Tarefas (FreeRTOS):**
  - Uso de **filas** para comunicação entre tarefas.
  - **Mutex** para acesso concorrente aos dados dos sensores.
  - **Interrupções** para processamento eficiente dos botões.

---

---

## Recursos
- **Simulação no Wokwi:** [Clique aqui](https://wokwi.com/projects/424461205323732993)

---

## Como Executar
1. Clone o repositório:
   ```bash
   git clone https://github.com/maelsilvatt/G.E.C.K.-Fallout-Sistemas-Embarcados-2024-2.git
   ```
2. Abra o projeto no **VS Code** com **PlatformIO** instalado.
3. Conecte sua placa compatível ou utilize o **Wokwi** para simulação.
4. Compile e faça o upload para a placa.

---

## Considerações
- **Mock de Terraformagem:** Como o Wokwi possui limitações, a regulação do ambiente foi simulada via dados fictícios, enquanto os sensores operam normalmente.

---

