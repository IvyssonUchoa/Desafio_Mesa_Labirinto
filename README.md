# Mesa Labirinto Controlada por Joystick

## Visão Geral

Este repositório contém o projeto **Mesa Labirinto Controlada por Joystick**, desenvolvido como Projeto Final da disciplina **Sistemas Embarcados (2025.2)** do curso de **Bacharelado em Engenharia de Computação – IFPB Campus Campina Grande**.

O sistema consiste em uma mesa labirinto física, controlada por um microcontrolador **ESP32**, que permite ao usuário movimentar uma esfera metálica por meio da inclinação da mesa nos eixos X e Y. A inclinação é controlada por um **joystick analógico**, atuando sobre dois **servomotores**, enquanto a orientação da mesa é monitorada por um **sensor inercial MPU6050**.

Além do controle físico, o projeto implementa um **Gêmeo Digital**, no qual os dados de inclinação (pitch e roll) são enviados em tempo real para um **dashboard no Grafana**, utilizando **InfluxDB** como banco de dados de séries temporais.

---

## Objetivos do Projeto

* Desenvolver um sistema embarcado interativo utilizando o ESP32;
* Controlar a inclinação de uma mesa labirinto em dois eixos (X e Y);
* Permitir interação do usuário via joystick analógico;
* Monitorar a orientação da mesa com um sensor MPU6050;
* Enviar dados em tempo real para visualização em um dashboard Grafana;
* Implementar arquitetura concorrente utilizando **FreeRTOS**;
* Criar um modelo virtual (Gêmeo Digital) sincronizado com o sistema físico.

---

## Arquitetura do Sistema

O sistema é composto por três camadas principais:

1. **Camada Física**

   * Mesa labirinto com estrutura impressa em 3D;
   * Dois servomotores responsáveis pela inclinação da mesa;
   * Esfera metálica que percorre o labirinto.

2. **Camada Embarcada (ESP32)**

   * Leitura do joystick analógico (ADC);
   * Controle PWM dos servomotores;
   * Leitura do sensor MPU6050 via I²C;
   * Detecção de condição de vitória por sensor indutivo;
   * Execução concorrente utilizando tarefas do FreeRTOS.

3. **Camada de Monitoramento (PC)**

   * Script em Python para leitura serial (UART/USB);
   * Armazenamento dos dados no InfluxDB;
   * Visualização em tempo real no Grafana.

---

## Hardware Utilizado

| Quantidade | Componente                      | Função                         |
| ---------: | ------------------------------- | ------------------------------ |
|          1 | ESP32                           | Microcontrolador principal     |
|          1 | Joystick analógico              | Controle dos eixos X e Y       |
|          2 | Servomotores 90g                | Inclinação da mesa             |
|          1 | MPU6050                         | Medição de pitch e roll        |
|          1 | Sensor indutivo NPN (TL-W5MC1)  | Detecção de vitória            |
|          1 | LED                             | Indicação de status do sistema |
|          - | Resistores, cabos e periféricos | Suporte eletrônico             |

---

## Software e Ferramentas

* **ESP-IDF** – Framework de desenvolvimento para o ESP32;
* **FreeRTOS** – Sistema operacional de tempo real;
* **Grafana** – Visualização dos dados em tempo real;
* **InfluxDB** – Banco de dados de séries temporais;
* **Python** – Gateway de comunicação serial e gravação no banco de dados;
* **Docker** – Deploy local do Grafana e InfluxDB.

---

## Organização do Firmware (FreeRTOS)

O firmware foi estruturado de forma modular, utilizando tarefas independentes:

| Task              | Função                                        |
| ----------------- | --------------------------------------------- |
| `joystick_task()` | Leitura, normalização e filtragem do joystick |
| `servo_task()`    | Controle suave dos servomotores (PWM)         |
| `mpu_task()`      | Leitura e cálculo de pitch e roll (MPU6050)   |
| `monitor_task()`  | Logs e debug via serial                       |
| `sensor_task()`   | Detecção da condição de vitória               |

A comunicação entre tarefas é protegida por **Mutex**, garantindo integridade dos dados compartilhados.

---

## Fluxo de Dados

1. O usuário movimenta o joystick;
2. O ESP32 processa o sinal analógico e define a inclinação desejada;
3. Os servomotores ajustam a posição da mesa de forma suave;
4. O MPU6050 mede a orientação real da mesa;
5. Os dados de pitch e roll são enviados via UART;
6. Um script Python grava os dados no InfluxDB;
7. O Grafana exibe a orientação da mesa em tempo real.

---

## Gêmeo Digital (Grafana + InfluxDB)

* Os dados são armazenados no bucket `mesa_labirinto` do InfluxDB;
* O Grafana exibe:
  * Gráficos em tempo real de pitch e roll;
  * Indicadores visuais da orientação da mesa;
* O dashboard reflete fielmente o comportamento físico do sistema.

---

## Funcionalidades Extras

* Detecção automática de vitória utilizando sensor indutivo;
* Indicação visual por LED ao concluir o percurso do labirinto;
* Filtro complementar para estabilização da leitura do MPU6050;
* Rampa de aceleração para suavização do movimento dos servos.

---

# Video demonstrativo do funcionamento 

* No vídeo é possivel ver o funcionamento e a explicação completa do sistema.
* [Assista ao vídeo do projeto aqui](https://youtube.com/shorts/KlTK687V3Gc)

---

## Autores

* Daniel Lima Neto
* Geovana Stefani Lopes Bezerra
* Ivysson Fernandes de Queiroz Uchôa
* Luis Felipe Ferreira Tavares Nonato

---

## Instituição

Instituto Federal de Educação, Ciência e Tecnologia da Paraíba
Campus Campina Grande
Curso de Engenharia de Computação

---

## Licença

Este projeto foi desenvolvido para fins acadêmicos. O uso, modificação e redistribuição devem respeitar os termos definidos pelos autores.
