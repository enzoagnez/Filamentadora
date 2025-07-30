#define MAX_CHANNELS 8
#define PWM_PERIOD 500
#define ZONA_MORTA_MIN 5
#define ZONA_MORTA_MAX 95

struct SoftwarePWM {
  byte pin;
  byte potencia; // valor atual de potência (0–100)
  unsigned long lastToggle;
  bool estado;
};

SoftwarePWM canaisPWM[MAX_CHANNELS];
int totalCanais = 0;

// Registra o pino como canal PWM
void adicionaCanal(byte pino) {
  if (totalCanais >= MAX_CHANNELS) return;

  pinMode(pino, OUTPUT);
  canaisPWM[totalCanais].pin = pino;
  canaisPWM[totalCanais].potencia = 0;
  canaisPWM[totalCanais].lastToggle = millis();
  canaisPWM[totalCanais].estado = false;
  digitalWrite(pino, LOW);
  totalCanais++;
}

// Atualiza a potência de um canal pelo número do pino
void atualizaPotencia(byte pino, byte novaPotencia) {
  for (int i = 0; i < totalCanais; i++) {
    if (canaisPWM[i].pin == pino) {
      canaisPWM[i].potencia = novaPotencia;
      break;
    }
  }
}

// Atualiza todos os canais de PWM (deve ser chamada no loop)
void atualizaPWM() {
  unsigned long agora = millis();

  for (int i = 0; i < totalCanais; i++) {
    SoftwarePWM *canal = &canaisPWM[i];
    byte potencia = canal->potencia;

    // Zona morta
    if (potencia >= ZONA_MORTA_MAX) {
      digitalWrite(canal->pin, HIGH);
      continue;
    } else if (potencia <= ZONA_MORTA_MIN) {
      digitalWrite(canal->pin, LOW);
      continue;
    }

    unsigned long onTime = (PWM_PERIOD * potencia) / 100;
    unsigned long offTime = PWM_PERIOD - onTime;

    if (canal->estado) {
      if (agora - canal->lastToggle >= onTime) {
        digitalWrite(canal->pin, LOW);
        canal->estado = false;
        canal->lastToggle = agora;
      }
    } else {
      if (agora - canal->lastToggle >= offTime) {
        digitalWrite(canal->pin, HIGH);
        canal->estado = true;
        canal->lastToggle = agora;
      }
    }
  }
}
