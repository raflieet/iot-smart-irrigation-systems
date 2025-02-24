// Pin tempat sensor YF-B10 terhubung
const int flowSensorPin = 34;

// Variabel untuk menyimpan jumlah pulsa
volatile long pulseCount = 0;

// Variabel untuk menyimpan laju aliran (dalam liter per menit)
float flowRate = 0.0;

// Variabel untuk menyimpan total volume air yang telah mengalir
float totalVolume = 0.0;

// Faktor kalibrasi sensor (disesuaikan dengan datasheet sensor)
float calibrationFactor = 8.83; // Contoh: 7.5 pulsa per liter per menit

// Waktu terakhir pembacaan
unsigned long oldTime = 0;

void setup() {
  // Inisialisasi serial monitor
  Serial.begin(115200);

  // Set pin sensor sebagai input
  pinMode(flowSensorPin, INPUT);

  // Attach interrupt untuk menghitung pulsa
  attachInterrupt(digitalPinToInterrupt(flowSensorPin), pulseCounter, FALLING);
}

void loop() {
  // Dapatkan waktu saat ini
  unsigned long currentTime = millis();

  // Hitung selisih waktu sejak pembacaan terakhir
  unsigned long elapsedTime = currentTime - oldTime;

  // Jika selisih waktu lebih dari 1 detik, hitung laju aliran
  if (elapsedTime > 1000) {
    Serial.println(elapsedTime);

    // Non-aktifkan interrupt sementara
    detachInterrupt(digitalPinToInterrupt(flowSensorPin));

    // Hitung laju aliran (dalam liter per menit)
    flowRate = ((1000.0 / elapsedTime) * pulseCount) / calibrationFactor;

    // Hitung total volume air yang telah mengalir (dalam liter)
    totalVolume += (flowRate / 60); // Konversi dari liter per menit ke liter per detik

    // Tampilkan hasil ke serial monitor
    Serial.print("Flow Rate: ");
    Serial.print(flowRate);
    Serial.print(" L/min");
    Serial.print("  Total Volume: ");
    Serial.print(totalVolume);
    Serial.print(" L");
    Serial.print("Pulse Count: ");
    Serial.println(pulseCount);

    // Reset pulse counter
    pulseCount = 0;

    // Simpan waktu saat ini sebagai waktu terakhir pembacaan
    oldTime = currentTime;

    // Aktifkan kembali interrupt
    attachInterrupt(digitalPinToInterrupt(flowSensorPin), pulseCounter, FALLING);
  }
}

// Fungsi interrupt untuk menghitung pulsa
void pulseCounter() {
  pulseCount++;

}
