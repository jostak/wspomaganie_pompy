#include <WiFi.h>
#include <WebServer.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ArduinoOTA.h>
#include <HardwareSerial.h> 




// Definiujemy tylko bezpieczne piny dla ESP32 (omijamy piny pamięci Flash 6-11)
// Twój główny pin to GPIO 2
const uint8_t bezpiecznePiny[] = {2, 4, 12, 13, 14, 15, 25, 26, 27};


const int oneWireBus = 2; 
const int NUM_SENSORS = 3;

// 24 godziny * 60 minut * 6 próbek na minutę (co 10 sekund) = 8640 próbek
const int HISTORY_LENGTH = 8640; 
const unsigned long READ_INTERVAL = 10000; 

// ==========================================
// KONFIGURACJA WEJŚĆ/WYJŚĆ (AUTOMATYKA)
// ==========================================
const int HEATER_PIN = 12;         // Wyjście sterujące grzałką (HIGH = włącz)
const int HEATER_PERMIT_PIN = 13;  // Wejście zezwolenia (INPUT_PULLUP, LOW = zezwolenie)

// ==========================================
// KONFIGURACJA MODBUS RS485 (DTSU666) - SPRZĘTOWY UART2
// ==========================================
const int RS485_RX_PIN = 16;       
const int RS485_TX_PIN = 17;       
const uint8_t DTSU666_ADDR = 0x01; 

HardwareSerial rs485Serial(2);

struct DebugConfig {
  bool showVoltages    = true;   
  bool showCurrents    = false;  
  bool showActivePower = true;   
  bool showReactive    = false;  
  bool showEnergy      = true;   
  bool showRawFrames   = false;  
};
DebugConfig debugMask;

// ==========================================
// STRUKTURA DANYCH GLOBALNYCH Z LICZNIKA DTSU666
// ==========================================
struct DTSU666_Data {
  float ua; float ub; float uc;       
  float ia; float ib; float ic;       
  float pa; float pb; float pc;       
  float pTotal;                       // Dodatnia = Import, Ujemna = Eksport
  float qa; float qb; float qc;       
  float qTotal;                       
  float pfTotal;                      
  float energyImport;                 
  float energyExport;                 
  unsigned long lastUpdateTime;       
  bool dataValid = false;             
};
DTSU666_Data dtsuData;

// CONFIG: OPISY I FUNKCJE CZUJNIKÓW (LEGENDA)
const String DESC_1 = "Zasilanie Solar";
const String DESC_2 = "Powrot Solar";
const String DESC_3 = "Zasobnik CWU";

// Wskaźniki dla dynamicznej alokacji buforów w RAM (unikamy stack overflow na ESP32)
float* tempHistory[NUM_SENSORS];
float* powerHistory = nullptr;    

int historyIndex = 0;
unsigned long lastReadTime = 0;
bool bufferFull = false; 

float sensor_1 = DEVICE_DISCONNECTED_C;
float sensor_2 = DEVICE_DISCONNECTED_C;
float sensor_3 = DEVICE_DISCONNECTED_C;

String status_1 = "INIT";
String status_2 = "INIT";
String status_3 = "INIT";

// Sztywne adresy sprzętowe czujników
const String ADDR_1 = "28b48f2300000099";
const String ADDR_2 = "280cf7210000006b";
const String ADDR_3 = "289ac1cb00000022";

DeviceAddress devAddr1, devAddr2, devAddr3;
bool hasAddr1 = false, hasAddr2 = false, hasAddr3 = false;

OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);
WebServer server(80);

// Deklaracje funkcji forward
String getAddressString(DeviceAddress deviceAddress);
bool parseHexString(String hexStr, uint8_t* dest);
void handleRoot();
void handleSensorList();
void handleTemperature();
void handleHistory();
void updateHistory();
void setupOTA();
void wykonajDodatkoweOperacje(float s1, bool s1_ok, float s2, bool s2_ok, float s3, bool s3_ok);

// Deklaracje funkcji RS485/Modbus
void checkRS485Bus();
bool validateCRC(uint8_t* buffer, int length);
uint16_t calculateCRC(uint8_t* buffer, int length);
float parseModbusFloat(uint8_t* buffer, int startIndex);
void decodeDTSU666Register(uint16_t byteCount, uint8_t* byteBuffer, int length);
void printSelectedDTSUData();

// ==========================================
// INTERFEJS WEB (HTML, CSS Tailwind, Chart.js)
// ==========================================
const char MAIN_page[] PROGMEM = R"=====(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>ESP_Solar (ESP32 24h)</title>
  <script src="https://cdn.jsdelivr.net/npm/chart.js@3.9.1"></script>
  <link href="https://cdn.jsdelivr.net/npm/tailwindcss@2.1.2/dist/tailwind.min.css" rel="stylesheet">
</head>
<body class="bg-gray-100 font-sans min-h-screen flex flex-col">
  <div class="container mx-auto p-6 flex-grow">
    <h1 class="text-2xl font-semibold text-gray-800 mb-4">ESP_Solar Monitor (Baza Dobowa 24h)</h1>

    <div class="flex mb-6">
      <div class="cursor-pointer px-4 py-2 bg-blue-500 text-white rounded-lg shadow hover:bg-blue-400 active:scale-95" onclick="showTab('dashboard')">Dashboard</div>
      <div class="cursor-pointer px-4 py-2 bg-gray-200 rounded-lg shadow hover:bg-gray-300 active:scale-95 ml-4" onclick="showTab('api')">API Docs</div>
      <div class="cursor-pointer px-4 py-2 bg-gray-200 rounded-lg shadow hover:bg-gray-300 active:scale-95 ml-4" onclick="showTab('setup')">Setup</div>
    </div>

    <div id="dashboard" class="tab-content">
      <div class="flex justify-between items-center mb-6">
        <button class="px-6 py-2 bg-green-500 text-white rounded-lg shadow hover:bg-green-400 active:scale-95" onclick="refreshData()">Refresh Data</button>
        <div id="last-update" class="text-sm text-gray-500">Last update: Never</div>
      </div>
      
      <div id="error-banner" class="hidden mb-6 p-4 bg-red-100 border-l-4 border-red-500 text-red-700 rounded shadow">
        <h3 class="font-bold mb-1">Wykryto problemy z czujnikami:</h3>
        <ul id="error-list" class="list-disc list-inside text-sm"></ul>
      </div>
      
      <div id="current-values" class="grid grid-cols-1 md:grid-cols-3 gap-4 mb-6"></div>

      <div id="dtsu-tile" class="bg-white p-6 rounded-lg shadow mb-6 border-l-4 border-purple-500">
        <div class="flex justify-between items-center mb-4">
          <h2 class="text-lg font-bold text-gray-800 uppercase tracking-wider">Licznik sieciowy Chint DTSU666</h2>
          <span id="dtsu-status" class="text-xs px-2 py-1 rounded font-semibold bg-gray-200 text-gray-700">BRAK DANYCH</span>
        </div>
        <div class="grid grid-cols-1 md:grid-cols-4 gap-4 text-sm text-gray-700">
          <div class="bg-gray-50 p-3 rounded">
            <div class="font-semibold text-gray-500 text-xs uppercase">Napięcia fazowe</div>
            <div class="mt-1">Ua: <span id="dtsu-ua" class="font-bold">--</span> V</div>
            <div>Ub: <span id="dtsu-ub" class="font-bold">--</span> V</div>
            <div>Uc: <span id="dtsu-uc" class="font-bold">--</span> V</div>
          </div>
          <div class="bg-gray-50 p-3 rounded">
            <div class="font-semibold text-gray-500 text-xs uppercase">Prądy fazowe</div>
            <div class="mt-1">Ia: <span id="dtsu-ia" class="font-bold">--</span> A</div>
            <div>Ib: <span id="dtsu-ib" class="font-bold">--</span> A</div>
            <div>Ic: <span id="dtsu-ic" class="font-bold">--</span> A</div>
          </div>
          <div class="bg-gray-50 p-3 rounded">
            <div class="font-semibold text-gray-500 text-xs uppercase">Bilans i moce czynne</div>
            <div class="mt-1 font-bold text-base text-blue-600">P-Total: <span id="dtsu-ptotal">--</span> kW</div>
            <div class="text-xs mt-1">Pa: <span id="dtsu-pa">--</span> kW | Pb: <span id="dtsu-pb">--</span> kW</div>
            <div class="text-xs">Pc: <span id="dtsu-pc">--</span> kW | PF: <span id="dtsu-pf">--</span></div>
          </div>
          <div class="bg-gray-50 p-3 rounded">
            <div class="font-semibold text-gray-500 text-xs uppercase">Liczniki energii</div>
            <div class="mt-1 text-xs">Import: <span id="dtsu-energy-in" class="font-bold text-green-600">--</span> kWh</div>
            <div class="text-xs">Eksport: <span id="dtsu-energy-out" class="font-bold text-orange-600">--</span> kWh</div>
            <div class="text-xs mt-1 text-gray-400">Q-Total: <span id="dtsu-qtotal">--</span> kvar</div>
          </div>
        </div>
      </div>

      <div class="bg-white p-6 rounded-lg shadow mb-6">
        <h2 class="text-lg font-semibold text-gray-700 mb-4">Wykres dobowy (Ostatnie 24h)</h2>
        <div class="relative" style="height: 500px;">
          <canvas id="multiIndexChart"></canvas>
        </div>
      </div>
    </div>

    <div id="api" class="tab-content hidden mt-8">
      <h2 class="text-xl font-semibold text-gray-800 mb-4">API</h2>
      <div class="bg-white p-6 rounded-lg shadow">
        <div class="mb-4"><span class="font-semibold text-blue-500">GET</span> <a href="/temperature" class="text-blue-500 hover:underline">/temperature</a></div>
        <div class="mb-4"><span class="font-semibold text-blue-500">GET</span> <a href="/sensors" class="text-blue-500 hover:underline">/sensors</a></div>
        <div><span class="font-semibold text-blue-500">GET</span> <a href="/history" class="text-blue-500 hover:underline">/history</a></div>
      </div>
    </div>

    <div id="setup" class="tab-content hidden mt-8">
      <h2 class="text-xl font-semibold text-gray-800 mb-4">Setup Instructions</h2>
      <div class="bg-white p-6 rounded-lg shadow leading-relaxed"><p>System portowany na platforme ESP32 z buforem 24h realizowanym sprzętowo.</p></div>
    </div>
  </div>

  <footer class="bg-gray-800 text-white p-4 mt-auto text-center"><p>&copy; 2026 ESP32 Solar Monitor 24h. All Rights Reserved.</p></footer>

  <script>
    let globalChart = null;
    const chartColors = [
      { border: 'rgb(239, 68, 68)', bg: 'rgba(239, 68, 68, 0.1)' },
      { border: 'rgb(59, 130, 246)', bg: 'rgba(59, 130, 246, 0.1)' },
      { border: 'rgb(16, 185, 129)', bg: 'rgba(16, 185, 129, 0.1)' }
    ];

    const showTab = (name) => {
      document.querySelectorAll('.tab-content').forEach(c => c.classList.add('hidden'));
      document.getElementById(name).classList.remove('hidden');
    };

    const buildCurrentValuesHTML = (sensors) => {
      return sensors.map((s, index) => {
        const isOk = s.status === "OK";
        const color = isOk ? chartColors[index % chartColors.length].border : 'rgb(220, 38, 38)';
        const displayTemp = isOk ? `${parseFloat(s.celsius).toFixed(1)}°C` : "ERROR";
        
        return `
          <div class="bg-white p-4 rounded-lg shadow border-l-4 ${!isOk ? 'bg-red-50' : ''}" style="border-color: ${color}">
            <div class="flex justify-between items-center mb-1">
              <div>
                <span class="text-xs font-bold text-gray-400 uppercase tracking-wider">sensor_${s.id}</span>
                <div class="text-xs font-semibold text-blue-600">${s.description}</div>
              </div>
              <span class="text-xs px-2 py-0.5 rounded font-semibold ${isOk ? 'bg-green-100 text-green-800' : 'bg-red-200 text-red-800'}">
                ${s.status}
              </span>
            </div>
            <div class="text-2xl font-bold ${isOk ? 'text-gray-800' : 'text-red-600'} mt-2">${displayTemp}</div>
            <div class="text-xs text-gray-500 truncate mt-1" title="${s.address}">ROM: ${s.address}</div>
          </div>
        `;
      }).join('');
    };

    const updateDTSUTile = (dtsu) => {
      const statusEl = document.getElementById('dtsu-status');
      if(!dtsu || !dtsu.valid) {
        statusEl.className = "text-xs px-2 py-1 rounded font-semibold bg-red-100 text-red-800";
        statusEl.innerText = "BRAK SYGNAŁU MODBUS";
        return;
      }
      
      statusEl.className = "text-xs px-2 py-1 rounded font-semibold bg-green-100 text-green-800";
      statusEl.innerText = "ONLINE (OK)";
      
      document.getElementById('dtsu-ua').innerText = dtsu.ua.toFixed(1);
      document.getElementById('dtsu-ub').innerText = dtsu.ub.toFixed(1);
      document.getElementById('dtsu-uc').innerText = dtsu.uc.toFixed(1);
      
      document.getElementById('dtsu-ia').innerText = dtsu.ia.toFixed(2);
      document.getElementById('dtsu-ib').innerText = dtsu.ib.toFixed(2);
      document.getElementById('dtsu-ic').innerText = dtsu.ic.toFixed(2);
      
      document.getElementById('dtsu-ptotal').innerText = dtsu.pTotal.toFixed(3);
      document.getElementById('dtsu-pa').innerText = dtsu.pa.toFixed(3);
      document.getElementById('dtsu-pb').innerText = dtsu.pb.toFixed(3);
      document.getElementById('dtsu-pc').innerText = dtsu.pc.toFixed(3);
      document.getElementById('dtsu-pf').innerText = dtsu.pfTotal.toFixed(2);
      
      document.getElementById('dtsu-energy-in').innerText = dtsu.energyImport.toFixed(1);
      document.getElementById('dtsu-energy-out').innerText = dtsu.energyExport.toFixed(1);
      document.getElementById('dtsu-qtotal').innerText = dtsu.qTotal.toFixed(3);
    };

    const updateChart = (datasets) => {
      const ctx = document.getElementById('multiIndexChart');
      if (!ctx) return;
      if (globalChart) { globalChart.destroy(); }

      globalChart = new Chart(ctx, {
        type: 'line',
        data: { datasets: datasets },
        options: {
          responsive: true,
          maintainAspectRatio: false,
          animation: false, 
          plugins: {
            legend: { position: 'top', labels: { boxWidth: 20 } },
            tooltip: {
              mode: 'index',
              intersect: false,
              callbacks: {
                title: (items) => new Date(items[0].parsed.x).toLocaleTimeString([], {hour: '2-digit', minute:'2-digit'}),
                label: (ctx) => {
                  const unit = ctx.dataset.yAxisID === 'yPower' ? ' kW' : '°C';
                  return `${ctx.dataset.label}: ${ctx.parsed.y ? parseFloat(ctx.parsed.y).toFixed(2) + unit : 'Brak'}`;
                }
              }
            }
          },
          interaction: { mode: 'nearest', intersect: true },
          scales: {
            x: {
              type: 'linear',
              position: 'bottom',
              ticks: { autoSkip: true, maxTicksLimit: 12, callback: (v) => new Date(v).toLocaleTimeString([], {hour: '2-digit', minute:'2-digit'}) },
              title: { display: true, text: 'Czas (Ostatnie 24h)' }
            },
            y: {
              type: 'linear',
              position: 'left',
              title: { display: true, text: 'Temperatura (°C)' },
              grid: { color: 'rgba(0,0,0,0.05)' }
            },
            yPower: {
              type: 'linear',
              position: 'right',
              title: { display: true, text: 'Moc czynna sumaryczna (kW)' },
              grid: { drawOnChartArea: false }, 
              ticks: { callback: (v) => v + " kW" }
            }
          }
        }
      });
    };

    const downsample = (data, maxPoints = 500) => {
      if (data.length <= maxPoints) return data;
      const step = Math.ceil(data.length / maxPoints);
      const sampled = [];
      for (let i = 0; i < data.length; i += step) {
        sampled.push(data[i]);
      }
      return sampled;
    };

    const refreshData = async () => {
      try {
        const currentValsDiv = document.getElementById('current-values');
        const errorBanner = document.getElementById('error-banner');
        const errorList = document.getElementById('error-list');
        
        const td = await fetch('/temperature'); const tempData = await td.json();
        const hd = await fetch('/history'); const historyData = await hd.json();
        
        currentValsDiv.innerHTML = buildCurrentValuesHTML(tempData.sensors);
        updateDTSUTile(tempData.dtsu666);
        
        document.getElementById('last-update').innerText = "Last update: " + new Date().toLocaleTimeString();

        const errors = tempData.sensors.filter(s => s.status !== "OK");
        if (errors.length > 0) {
          errorList.innerHTML = errors.map(s => `<li><strong>sensor_${s.id} (${s.description})</strong>: ${s.status}</li>`).join('');
          errorBanner.classList.remove('hidden');
        } else {
          errorBanner.classList.add('hidden');
        }

        const now = Date.now();
        const interval = historyData.interval_ms || 10000;

        const datasets = tempData.sensors.map((s, index) => {
          const sensorHist = historyData.sensors.find(h => h.id === s.id);
          const colorSet = chartColors[index % chartColors.length];
          if (!sensorHist) return null;
          
          const totalPoints = sensorHist.history.length;
          let points = sensorHist.history.map((v, i) => {
            return { x: now - (totalPoints - 1 - i) * interval, y: v };
          });

          points = downsample(points, 400);

          return {
            label: `sensor_${s.id} (${s.description})`,
            data: points,
            borderColor: colorSet.border,
            backgroundColor: colorSet.bg,
            borderWidth: 2,
            pointRadius: 0, 
            lineTension: 0.1,
            fill: false,
            yAxisID: 'y'
          };
        }).filter(d => d !== null);

        if (historyData.power_history) {
          const totalPowerPoints = historyData.power_history.length;
          let powerPoints = historyData.power_history.map((v, i) => {
            return { x: now - (totalPowerPoints - 1 - i) * interval, y: v };
          });

          powerPoints = downsample(powerPoints, 400);

          datasets.push({
            label: "Moc sumaryczna P-Total (kW)",
            data: powerPoints,
            borderColor: 'rgb(147, 51, 234)', 
            backgroundColor: 'rgba(147, 51, 234, 0.05)',
            borderWidth: 1.5,
            borderDash: [3, 3], 
            pointRadius: 0, 
            lineTension: 0.1,
            fill: false,
            yAxisID: 'yPower' 
          });
        }

        updateChart(datasets);
      } catch(e) {
        console.error(e);
        document.getElementById('current-values').innerHTML = '<div class="text-red-500 font-bold p-4">Error loading data from ESP32.</div>';
      }
    };

    refreshData();
    setInterval(refreshData, 11000);
  </script>
</body>
</html>
)=====";

// ==========================================
// FUNKCJA INICJALIZACYJNA (SETUP)
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n\n===============================================");
  Serial.println("     [SYSTEM START] - MONITOR SOLARNY ESP32 24H  ");
  Serial.println("===============================================");

  // Dynamiczna alokacja pamięci na stercie (Heap) dla bazy 24h
  Serial.println("[SYSTEM] Alokacja dynamiczna buforów historii w RAM...");
  for (int i = 0; i < NUM_SENSORS; i++) {
    tempHistory[i] = (float*)malloc(HISTORY_LENGTH * sizeof(float));
    if (tempHistory[i] == nullptr) {
      Serial.printf("!!! KRYTYCZNY BŁĄD: Brak pamięci dla tempHistory[%d] !!!\n", i);
      while(1) delay(1000); 
    }
  }
  
  powerHistory = (float*)malloc(HISTORY_LENGTH * sizeof(float));
  if (powerHistory == nullptr) {
    Serial.println("!!! KRYTYCZNY BŁĄD: Brak pamięci dla powerHistory !!!");
    while(1) delay(1000);
  }
  Serial.println("[SYSTEM] Alokacja zakończona sukcesem.");

  // Inicjalizacja sprzętowego UART2 (Modbus RS485 Chint DTSU666)
  rs485Serial.begin(9600, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  Serial.printf("[RS485] Sprzetowy UART2 aktywowany. RX: GPIO %d, TX: GPIO %d (9600 bps)\n", RS485_RX_PIN, RS485_TX_PIN);

  pinMode(HEATER_PIN, OUTPUT);
  digitalWrite(HEATER_PIN, LOW);
  
  pinMode(HEATER_PERMIT_PIN, INPUT_PULLUP);

  pinMode(oneWireBus, INPUT_PULLUP);

  sensors.begin();
  sensors.setResolution(12); 
  sensors.setWaitForConversion(true);

  hasAddr1 = parseHexString(ADDR_1, devAddr1);
  hasAddr2 = parseHexString(ADDR_2, devAddr2);
  hasAddr3 = parseHexString(ADDR_3, devAddr3);

  // Czyszczenie zaalokowanych buforów historii
  for (int i = 0; i < NUM_SENSORS; i++) {
    for (int j = 0; j < HISTORY_LENGTH; j++) {
      tempHistory[i][j] = DEVICE_DISCONNECTED_C;
    }
  }
  for (int j = 0; j < HISTORY_LENGTH; j++) {
    powerHistory[j] = 0.0;
  }

  Serial.print("[WiFi] Laczenie z: ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\n[WiFi] Połączono!");
  Serial.print("[WiFi] IP: ");
  Serial.println(WiFi.localIP());

  setupOTA(); 

  server.on("/", HTTP_GET, handleRoot);
  server.on("/temperature", HTTP_GET, handleTemperature);
  server.on("/sensors", HTTP_GET, handleSensorList);
  server.on("/history", HTTP_GET, handleHistory); 

  server.begin();
  Serial.println("[HTTP] Serwer WWW ESP32 aktywny.");
  
  // Najpierw wymuszamy zebranie świeżych odczytów, żeby zmienne miały wartości
  updateHistory();
  
  // Wywołanie startowe automatyki
  wykonajDodatkoweOperacje(sensor_1, (status_1 == "OK"), sensor_2, (status_2 == "OK"), sensor_3, (status_3 == "OK"));

  lastReadTime = millis();
}

// ==========================================
// GŁÓWNA PĘTLA PROGRAMU (LOOP)
// ==========================================
void loop() {
  ArduinoOTA.handle(); 
  server.handleClient();
  
  // Nieblokujący nasłuch magistrali RS485
  checkRS485Bus();
  
  unsigned long t = millis();
  if (t - lastReadTime >= READ_INTERVAL) {
    lastReadTime = t; 
    
    updateHistory();
    
    bool s1_sprawny = (status_1 == "OK");
    bool s2_sprawny = (status_2 == "OK");
    bool s3_sprawny = (status_3 == "OK");
    
    wykonajDodatkoweOperacje(sensor_1, s1_sprawny, sensor_2, s2_sprawny, sensor_3, s3_sprawny);
    printSelectedDTSUData();
  }
}

// ==========================================
// CENTRALNY PUNKT LOGIKI I AUTOMATYKI
// ==========================================
void wykonajDodatkoweOperacje(float s1, bool s1_ok, float s2, bool s2_ok, float s3, bool s3_ok) {
  
  /* ========================================================================================
   ŚCIĄGAWKA DOSTĘPNYCH ZMIENNYCH Z LICZNIKA CHINT DTSU666 (STRUKTURA dtsuData)
   ========================================================================================
   Zanim użyjesz jakiejkolwiek zmiennej sieciowej, ZAWSZE sprawdź, czy: dtsuData.dataValid == true
   
   1. FLAGA WALIDACJI:
      dtsuData.dataValid    -> (bool) true = komunikacja RS485 OK, dane poprawne; false = błąd Modbus
      
   2. BILANS ENERGII I MOC CAŁKOWITA:
      dtsuData.pTotal       -> (float) Sumaryczna moc czynna całego domu [w kW, np. 1.234 lub -2.550]
                               POWYŻEJ ZERA ( > 0 )  -> IMPORT (pobierasz prąd z sieci)
                               PONIŻEJ ZERA ( < 0 )  -> EKSPORT (nadprodukcja, oddajesz do sieci)
                               Przykład użycia: if(dtsuData.pTotal < -1.5) { // włącz grzałkę, oddajesz ponad 1.5kW }
                               
   3. NAPIĘCIA FAZOWE (V):
      dtsuData.ua           -> (float) Napięcie na Fazie A (L1) [V]
      dtsuData.ub           -> (float) Napięcie na Fazie B (L2) [V]
      dtsuData.uc           -> (float) Napięcie na Fazie C (L3) [V]
                               Przykład: Ochrona przed wyłączeniem falownika (limit sieci to 253V):
                               if(dtsuData.ua > 251.0 || dtsuData.ub > 251.0) { // włącz grzałkę, zbij napięcie }
                               
   4. MOCOWANIE POSZCZEGÓLNYCH FAZ (kW):
      dtsuData.pa           -> (float) Moc czynna na Fazie A [kW]
      dtsuData.pb           -> (float) Moc czynna na Fazie B [kW]
      dtsuData.pc           -> (float) Moc czynna na Fazie C [kW]
      
   5. PRĄDY FAZOWE (A):
      dtsuData.ia           -> (float) Natężenie prądu na Fazie A [A]
      dtsuData.ib           -> (float) Natężenie prądu na Fazie B [A]
      dtsuData.ic           -> (float) Natężenie prądu na Fazie C [A]
      
   6. MOC BIERNA I WSPÓŁCZYNNIK MOCY (Power Factor):
      dtsuData.qTotal       -> (float) Sumaryczna moc bierna całego układu [kvar]
      dtsuData.pfTotal      -> (float) Współczynnik mocy cos(phi) [od 0.00 do 1.00]
      
   7. LICZNIKI ENERGII (SKUMULOWANE kWh):
      dtsuData.energyImport -> (float) Całkowita energia pobrana z sieci od nowości [kWh]
      dtsuData.energyExport -> (float) Całkowita energia oddana do sieci od nowości [kWh]
   ========================================================================================
  */

  // 1. ZABEZPIECZENIE: Jeśli któryś czujnik DS18B20 zgłasza błąd, wyłączamy grzałkę
if (!s1_ok || !s2_ok || !s3_ok)
{
  digitalWrite(HEATER_PIN, LOW);

  Serial.println("  [AUTOMATYKA] Awaria czujników temperatury:");

  if (!s1_ok)
    Serial.printf("     -> Sensor 1 (%s)\n", DESC_1.c_str());

  if (!s2_ok)
    Serial.printf("     -> Sensor 2 (%s)\n", DESC_2.c_str());

  if (!s3_ok)
    Serial.printf("     -> Sensor 3 (%s)\n", DESC_3.c_str());

  Serial.println("     Grzalka WYLACZONA awaryjnie.");
  return;
}
  
  // 2. BLOKADA SPRZĘTOWA: Wejście blokady zewnętrznej (np. termostat bezpieczeństwa STB lub przełącznik)
  if (digitalRead(HEATER_PERMIT_PIN) == HIGH) {
    digitalWrite(HEATER_PIN, LOW);
    Serial.println("  [AUTOMATYKA] Brak fizycznego zezwolenia (HEATER_PERMIT_PIN = HIGH). Grzałka WYŁĄCZONA.");
    return; 
  }
  
  // 3. OBLICZENIA LOGICZNE
  float deltaSolar = s1 - s2; // Różnica temperatur: Zasilanie - Powrót
  
  bool warunekTermiczny = (s3 < 62.0); // Chcemy grzać wodę w CWU maksymalnie do 62°C
  bool warunekNadwyzkiPradu = false;
  
  // Bezpieczne sprawdzenie danych z licznika DTSU
  if (dtsuData.dataValid) {
    // Jeżeli pTotal jest mniejsze niż -0.500 kW, to znaczy, że wysyłamy do sieci ponad 500W nadwyżki
    if (dtsuData.pTotal < -0.500) {
      warunekNadwyzkiPradu = true;
    }
  }
  
  // 4. DECYZJA STEROWANIA GRZAŁKĄ
  if (warunekTermiczny && (warunekNadwyzkiPradu || deltaSolar > 8.0)) {
    digitalWrite(HEATER_PIN, HIGH);
    Serial.print("  [AUTOMATYKA] Warunki OK. Grzałka: WŁĄCZONA.");
  } else {
    digitalWrite(HEATER_PIN, LOW);
    Serial.print("  [AUTOMATYKA] Warunki niespełnione. Grzałka: WYŁĄCZONA.");
  }

  // Monitorowanie stanu w oknie Serial Portu
  if (dtsuData.dataValid) {
    Serial.printf(" | Delta: %.1f *C | CWU: %.1f *C | Sieć: %.3f kW (%s)\n", 
                  deltaSolar, s3, dtsuData.pTotal, (dtsuData.pTotal >= 0 ? "IMPORT" : "EKSPORT"));
  } else {
    Serial.printf(" | Delta: %.1f *C | CWU: %.1f *C | Modbus: Brak danych z licznika\n", deltaSolar, s3);
  }
}

// ==========================================
// DEKODER RS485 (SPRZĘTOWY BUFOR)
// ==========================================
void checkRS485Bus() {
  static uint8_t frameBuffer[64];
  static int bufIndex = 0;
  static unsigned long lastByteTime = 0;

  while (rs485Serial.available() > 0) {
    uint8_t incomingByte = rs485Serial.read();
    unsigned long now = millis();

    if (now - lastByteTime > 15 && bufIndex > 0) {
      bufIndex = 0;
    }
    lastByteTime = now;

    if (bufIndex < (int)sizeof(frameBuffer)) {
      frameBuffer[bufIndex++] = incomingByte;
    }

    if (bufIndex >= 7) {
      if (frameBuffer[0] == DTSU666_ADDR && frameBuffer[1] == 0x03) {
        uint8_t expectedBytes = frameBuffer[2]; 
        int totalExpectedLength = expectedBytes + 5; 

        if (bufIndex == totalExpectedLength) {
          if (validateCRC(frameBuffer, totalExpectedLength)) {
            decodeDTSU666Register(expectedBytes, &frameBuffer[3], expectedBytes);
          }
          bufIndex = 0; 
        }
      }
    }
  }
}

void decodeDTSU666Register(uint16_t byteCount, uint8_t* byteBuffer, int length) {
  if (byteCount == 24) { 
    dtsuData.ua = parseModbusFloat(byteBuffer, 0);
    dtsuData.ub = parseModbusFloat(byteBuffer, 4);
    dtsuData.uc = parseModbusFloat(byteBuffer, 8);
    dtsuData.ia = parseModbusFloat(byteBuffer, 12);
    dtsuData.ib = parseModbusFloat(byteBuffer, 16);
    dtsuData.ic = parseModbusFloat(byteBuffer, 20);
    dtsuData.lastUpdateTime = millis(); dtsuData.dataValid = true;
  } 
  else if (byteCount == 16) {
    dtsuData.pa = parseModbusFloat(byteBuffer, 0);
    dtsuData.pb = parseModbusFloat(byteBuffer, 4);
    dtsuData.pc = parseModbusFloat(byteBuffer, 8);
    dtsuData.pTotal = parseModbusFloat(byteBuffer, 12);
    dtsuData.lastUpdateTime = millis(); dtsuData.dataValid = true;
  }
  else if (byteCount == 8) { 
    float val1 = parseModbusFloat(byteBuffer, 0);
    float val2 = parseModbusFloat(byteBuffer, 4);
    if (val1 > 0.1 && val2 >= 0.0) { dtsuData.energyImport = val1; dtsuData.energyExport = val2; }
    else { dtsuData.qTotal = val1; dtsuData.pfTotal = val2; }
    dtsuData.lastUpdateTime = millis(); dtsuData.dataValid = true;
  }
  else if (byteCount == 4) {
    dtsuData.pTotal = parseModbusFloat(byteBuffer, 0);
    dtsuData.lastUpdateTime = millis(); dtsuData.dataValid = true;
  }
}

float parseModbusFloat(uint8_t* buffer, int startIndex) {
  union { uint8_t b[4]; float f; } converter;
  converter.b[0] = buffer[startIndex + 3]; converter.b[1] = buffer[startIndex + 2];
  converter.b[2] = buffer[startIndex + 1]; converter.b[3] = buffer[startIndex + 0];
  return converter.f;
}

bool validateCRC(uint8_t* buffer, int length) {
  if (length < 3) return false;
  uint16_t receivedCRC = buffer[length - 1] << 8 | buffer[length - 2];
  return (calculateCRC(buffer, length - 2) == receivedCRC);
}

uint16_t calculateCRC(uint8_t* buffer, int length) {
  uint16_t crc = 0xFFFF;
  for (int pos = 0; pos < length; pos++) {
    crc ^= (uint16_t)buffer[pos];
    for (int i = 8; i != 0; i--) {
      if ((crc & 0x0001) != 0) { crc >>= 1; crc ^= 0xA001; } else { crc >>= 1; }
    }
  }
  return crc;
}

void printSelectedDTSUData() {
  if (!dtsuData.dataValid) return;
  if (debugMask.showActivePower) {
    Serial.printf("[DTSU32] Moc calkowita: %.3f kW (%s)\n", dtsuData.pTotal, (dtsuData.pTotal >= 0 ? "IMPORT" : "EKSPORT"));
  }
}

// ==========================================
// DETALE OPERACYJNE (OTA, STRUMIENIOWANIE, CZUJNIKI)
// ==========================================
void setupOTA() {
  ArduinoOTA.setHostname(ota_hostname);
  ArduinoOTA.begin();
}

bool parseHexString(String hexStr, uint8_t* dest) {
  if (hexStr.length() != 16) return false;
  for (int i = 0; i < 8; i++) {
    String byteStr = hexStr.substring(i * 2, (i * 2) + 2);
    dest[i] = strtol(byteStr.c_str(), NULL, 16);
  }
  return true;
}

void processSensor(int sensorId, bool hasAddress, DeviceAddress deviceAddress,
                   float &sensorVar, const float offset = 0.0)
{
  String sensorName;

  switch(sensorId)
  {
    case 1: sensorName = DESC_1; break;
    case 2: sensorName = DESC_2; break;
    case 3: sensorName = DESC_3; break;
    default: sensorName = "Nieznany"; break;
  }

  // Brak poprawnego adresu ROM
  if (!hasAddress)
  {
    Serial.printf("[DS18B20] CZUJNIK %d (%s)\n",
                  sensorId, sensorName.c_str());
    Serial.println("           BLAD: Niepoprawny adres ROM");

    sensorVar = DEVICE_DISCONNECTED_C;

    if (sensorId == 1) status_1 = "ERR";
    else if (sensorId == 2) status_2 = "ERR";
    else status_3 = "ERR";

    tempHistory[sensorId - 1][historyIndex] = DEVICE_DISCONNECTED_C;
    return;
  }

  // Czujnik nie odpowiada
  if (!sensors.isConnected(deviceAddress))
  {
    Serial.printf("[DS18B20] CZUJNIK %d (%s)\n",
                  sensorId, sensorName.c_str());
    Serial.println("           BLAD: Brak odpowiedzi na magistrali 1-Wire");

    sensorVar = DEVICE_DISCONNECTED_C;

    if (sensorId == 1) status_1 = "ERR";
    else if (sensorId == 2) status_2 = "ERR";
    else status_3 = "ERR";

    tempHistory[sensorId - 1][historyIndex] = DEVICE_DISCONNECTED_C;
    return;
  }

  float rawTemp = sensors.getTempC(deviceAddress);

  // Biblioteka zwróciła błąd
  if (rawTemp == DEVICE_DISCONNECTED_C)
  {
    Serial.printf("[DS18B20] CZUJNIK %d (%s)\n",
                  sensorId, sensorName.c_str());
    Serial.println("           BLAD: DEVICE_DISCONNECTED_C");

    sensorVar = DEVICE_DISCONNECTED_C;

    if (sensorId == 1) status_1 = "ERR";
    else if (sensorId == 2) status_2 = "ERR";
    else status_3 = "ERR";

    tempHistory[sensorId - 1][historyIndex] = DEVICE_DISCONNECTED_C;
    return;
  }

  // Temperatura poza zakresem
  if (rawTemp < 0.0 || rawTemp > 80.0)
  {
    Serial.printf("[DS18B20] CZUJNIK %d (%s)\n",
                  sensorId, sensorName.c_str());
    Serial.printf("           BLAD: Temperatura poza zakresem: %.2f C\n",
                  rawTemp);

    sensorVar = DEVICE_DISCONNECTED_C;

    if (sensorId == 1) status_1 = "ERR";
    else if (sensorId == 2) status_2 = "ERR";
    else status_3 = "ERR";

    tempHistory[sensorId - 1][historyIndex] = DEVICE_DISCONNECTED_C;
    return;
  }

  // Poprawny odczyt
  sensorVar = round((rawTemp + offset) * 10.0) / 10.0;

  if (sensorId == 1) status_1 = "OK";
  else if (sensorId == 2) status_2 = "OK";
  else status_3 = "OK";

  tempHistory[sensorId - 1][historyIndex] = sensorVar;
}

void updateHistory() {
  sensors.requestTemperatures();
  processSensor(1, hasAddr1, devAddr1, sensor_1, -0.1);
  processSensor(2, hasAddr2, devAddr2, sensor_2, 0.0);
  processSensor(3, hasAddr3, devAddr3, sensor_3, 0.25);

  powerHistory[historyIndex] = dtsuData.dataValid ? dtsuData.pTotal : 0.0;

  historyIndex++;
  if (historyIndex >= HISTORY_LENGTH) {
    historyIndex = 0;
    bufferFull = true;
  }
}

void handleRoot() { server.send(200, "text/html", MAIN_page); }

void handleSensorList() {
  String json = "{\"sensors\":[";
  json += "{\"id\":1,\"address\":\"" + ADDR_1 + "\",\"description\":\"" + DESC_1 + "\"},";
  json += "{\"id\":2,\"address\":\"" + ADDR_2 + "\",\"description\":\"" + DESC_2 + "\"},";
  json += "{\"id\":3,\"address\":\"" + ADDR_3 + "\",\"description\":\"" + DESC_3 + "\"}";
  json += "]}";
  server.send(200, "application/json", json);
}

void handleTemperature() {
  String json = "{\"sensors\":[";
  json += "{\"id\":1,\"address\":\"" + ADDR_1 + "\",\"description\":\"" + DESC_1 + "\",\"celsius\":" + (sensor_1 == DEVICE_DISCONNECTED_C ? "-127.0" : String(sensor_1, 1)) + ",\"status\":\"" + status_1 + "\"},";
  json += "{\"id\":2,\"address\":\"" + ADDR_2 + "\",\"description\":\"" + DESC_2 + "\",\"celsius\":" + (sensor_2 == DEVICE_DISCONNECTED_C ? "-127.0" : String(sensor_2, 1)) + ",\"status\":\"" + status_2 + "\"},";
  json += "{\"id\":3,\"address\":\"" + ADDR_3 + "\",\"description\":\"" + DESC_3 + "\",\"celsius\":" + (sensor_3 == DEVICE_DISCONNECTED_C ? "-127.0" : String(sensor_3, 1)) + ",\"status\":\"" + status_3 + "\"}";
  json += "],";
  json += "\"dtsu666\":{";
  json += "\"valid\":" + String(dtsuData.dataValid ? "true" : "false") + ",";
  json += "\"ua\":" + String(dtsuData.ua, 2) + ",\"ub\":" + String(dtsuData.ub, 2) + ",\"uc\":" + String(dtsuData.uc, 2) + ",";
  json += "\"ia\":" + String(dtsuData.ia, 3) + ",\"ib\":" + String(dtsuData.ib, 3) + ",\"ic\":" + String(dtsuData.ic, 3) + ",";
  json += "\"pa\":" + String(dtsuData.pa, 4) + ",\"pb\":" + String(dtsuData.pb, 4) + ",\"pc\":" + String(dtsuData.pc, 4) + ",";
  json += "\"pTotal\":" + String(dtsuData.pTotal, 4) + ",\"qTotal\":" + String(dtsuData.qTotal, 4) + ",\"pfTotal\":" + String(dtsuData.pfTotal, 2) + ",";
  json += "\"energyImport\":" + String(dtsuData.energyImport, 2) + ",\"energyExport\":" + String(dtsuData.energyExport, 2);
  json += "}}";
  server.send(200, "application/json", json);
}

void handleHistory() {
  server.sendHeader("Connection", "close");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/json", ""); 

  String chunk = "{\"interval_ms\":" + String(READ_INTERVAL) + ",\"sensors\":[";
  server.sendContent(chunk);

  int count = bufferFull ? HISTORY_LENGTH : historyIndex;

  for (int i = 0; i < NUM_SENSORS; i++) {
    if (i > 0) server.sendContent(",");
    String addr = (i == 0) ? ADDR_1 : ((i == 1) ? ADDR_2 : ADDR_3);
    
    chunk = "{\"id\":" + String(i + 1) + ",\"address\":\"" + addr + "\",\"history\":[";
    server.sendContent(chunk);

    chunk = "";
    for (int j = 0; j < count; j++) {
      int idx = bufferFull ? (historyIndex + j) % HISTORY_LENGTH : j;
      if (j > 0) chunk += ",";
      
      if (tempHistory[i][idx] == DEVICE_DISCONNECTED_C) chunk += "null";
      else chunk += String(tempHistory[i][idx], 1);

      if (chunk.length() > 1200) {
        server.sendContent(chunk);
        chunk = "";
      }
    }
    server.sendContent(chunk + "]}");
  }

  server.sendContent("],\"power_history\":[");
  chunk = "";
  for (int j = 0; j < count; j++) {
    int idx = bufferFull ? (historyIndex + j) % HISTORY_LENGTH : j;
    if (j > 0) chunk += ",";
    chunk += String(powerHistory[idx], 3);

    if (chunk.length() > 1200) {
      server.sendContent(chunk);
      chunk = "";
    }
  }
  server.sendContent(chunk + "]}");
  
  server.sendContent(""); 
}

String getAddressString(DeviceAddress deviceAddress) {
  String addr = "";
  for (uint8_t i = 0; i < 8; i++) {
    if (deviceAddress[i] < 16) addr += "0";
    addr += String(deviceAddress[i], HEX);
  }
  return addr;
}