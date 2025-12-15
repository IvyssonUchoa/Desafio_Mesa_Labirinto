# 1. Instalação e Execução (Infraestrutura)

Utilizamos o Docker Compose para subir o **InfluxDB** (Banco de Dados) e o **Grafana** (Visualização) simultaneamente, eliminando a necessidade de instalar serviços manualmente no sistema operacional.

a. Navegue até a pasta Dashboard_grafana:
   ```bash
      cd Dashboard_grafana
   ```
b. Inicie os serviços em segundo plano:
   ```bash
      docker-compose up -d
   ```
OBS: Para parar os serviços posteriormente, utilize docker-compose stop. Para reiniciar, use docker-compose start.

---

# 2. Configuração do InfluxDB
Antes de rodar o script Python, é necessário configurar o banco de dados e obter o Token de segurança.

a. Acesse http://localhost:8086 no seu navegador.

b. Clique em "Get Started" e configure o usuário inicial:
* Username/Password: Crie seu usuário admin.
* Organization Name: Defina um nome (ex: universidade).
* Bucket Name: Defina como mesa_labirinto.
* Clique em "Continue"

c. Salve o Token que será exibido, você precisará dela no próximo passo.

d. Clique em "Configure Later".

# 3. Configuração do Gateway (Python)
O script Python lê a porta serial e grava no InfluxDB.

a. Navegue até a pasta Gateway:
   ```bash
      cd Gateway
   ```

b. Instale as dependências
```bash
    pip install -r requirements.txt
```

c. Edite o arquivo gateway.py: Abra o arquivo e atualize as constantes iniciais com os dados do InfluxDB:
```python
    SERIAL_PORT = '/dev/ttyUSB0'  # Linux (ou 'COM3' no Windows)
    INFLUX_TOKEN = "SEU TOKEN"
    INFLUX_ORG = "SUA ORG"
    INFLUX_BUCKET = "SEU BUCKET"
   ```

d. Execute o script:
```bash
    python gateway.py
``` 

Se tudo estiver correto, você verá mensagens como: Enviado: Pitch=10.5, Roll=-5.2.

---

# 4. Configuração do Grafana (Dashboard)
Com os dados chegando no banco, configure a visualização.

a. Acesse http://localhost:3000 no navegador.
* User/Pass: admin / admin (troque a senha se solicitado).

b. Conectar ao Banco de Dados:
* Vá em Connections (ou Configuration) -> Data Sources -> Add data source.
* Selecione InfluxDB.
* Query Language: Mude de "InfluxQL" para Flux.
* URL: http://influxdb:8086 (Como estamos usando docker-compose, usamos o nome do container, não localhost).
* Organization: (Sua org).
* Token: Cole o mesmo Token do InfluxDB.
* Default Bucket: seu bucket.
* Clique em Save & Test

c. Criar o Dashboard:
* Vá em Dashboards -> New dashboard -> Add visualization.
* Selecione o data source criado.
* No editor de query, cole o código Flux abaixo para visualizar os dados:
```
from(bucket: "mesa_labirinto")
    |> range(start: -5m)
    |> filter(fn: (r) => r["_measurement"] == "orientacao_mesa")
    |> filter(fn: (r) => r["_field"] == "pitch" or r["_field"] == "roll")
    |> aggregateWindow(every: 100ms, fn: mean, createEmpty: false)
    |> yield(name: "mean")
``` 

d. Clique em "refresh" para carregar os dados vindos do InfluxDB.