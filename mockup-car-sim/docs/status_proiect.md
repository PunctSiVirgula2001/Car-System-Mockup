# Statusul proiectului _mockup-car-sim_

## 1. Context general

Acest proiect folosește framework‑ul ESP‑IDF pe platforma ESP32 pentru a simula/realiza o „mașinuță” cu afișaj OLED, control de motor prin I2C (dsPIC), senzori de distanță HC‑SR04, LED‑uri RGB față/spate și un encoder rotativ pentru interfață cu utilizatorul. Punctul de intrare este `main/hello_world_main.c`, unde funcția `app_tasks_init()` pregătește perifericele și task‑urile FreeRTOS.

Structura codului este împărțită pe straturi:
- `main/config` – configurări hardware (pini, adrese, constante per dispozitiv);
- `main/hal` – abstracție de periferice (GPIO, timere, PWM, I2C);
- `main/drivers` – drivere de dispozitiv (OLED, senzori, motoare, LED‑uri, encoder);
- `main/tasks` – task‑uri FreeRTOS pentru input, senzori, control, motor și UI.

## 2. Stadiu hardware

- Până în acest moment, pentru fiecare componentă software planificată în proiect (OLED, controller de motor pe I2C, senzori ultrasonici HC‑SR04, LED‑uri RGB față și spate, encoder rotativ) există modulul hardware corespondent montat pe placa de prototipare.
- Toate firele necesare pentru aceste componente (I2C SCL/SDA, TRIG/ECHO pentru HC‑SR04, pini pentru encoder și LED‑uri RGB etc.) sunt trase și lipite corect pe placa de prototipare, în concordanță cu definițiile de pini și semnale folosite în fișierele din `main/config`.

## 3. Arhitectură software

- `project_config.h` agregă toate fișierele de configurare (OLED, motor, encoder, senzori, LED‑uri), astfel încât driverele și HAL‑ul să poată folosi direct constantele de proiect.
- Stratul HAL (`main/hal`) oferă o interfață generică pentru GPIO, timere, PWM (LEDC) și I2C, astfel încât restul codului să nu depindă direct de API‑urile ESP‑IDF.
- Driverele din `main/drivers` construiesc logica specifică fiecărui dispozitiv (de ex. SSD1306 pentru OLED, HC‑SR04 pentru măsurare distanță etc.).
- Task‑urile din `main/tasks/app_tasks.c` vor rula logica aplicației: citirea input‑urilor, citirea senzorilor, calculul comenzilor pentru motoare/LED‑uri și actualizarea interfeței de utilizator pe OLED.

## 4. Stadiu software – componente implementate

### 4.1. HAL GPIO/Timere

Fișiere relevante: `main/hal/gpio_timers/hal_gpio_timers.c`, `main/hal/gpio_timers/hal_gpio_timers.h`

- Există funcții pentru:
  - configurarea unui pin (`hal_gpio_set_mode` – intrare/ieșire, pull‑up/pull‑down);
  - scriere/citire digitală (`hal_gpio_write`, `hal_gpio_read`);
  - atașarea și detașarea de ISR pe diferite tipuri de front (rising/falling/any);
  - generarea de PWM folosind perifericul LEDC (`hal_pwm_init`, `hal_pwm_set_duty`, `hal_pwm_deinit`);
  - generarea și măsurarea impulsurilor în microsecunde (`hal_pulse_out_us`, `hal_pulse_in_us`), utile în special pentru senzorii HC‑SR04.
- Acest strat este practic funcțional și poate fi folosit de driverele HC‑SR04, RGB LED și encoder.

### 4.2. HAL I2C

Fișiere relevante: `main/hal/i2c/hal_i2c.c`, `main/hal/i2c/hal_i2c.h`

- Configurare master I2C pe portul `I2C_NUM_0`, cu pini SCL/SDA și frecvență de lucru de 400 kHz.
- Funcția `hal_i2c_init()` creează un bus I2C (`i2c_new_master_bus`) și păstrează handle‑ul global `i2c_bus_handle`, folosit ulterior de driverul OLED și, în viitor, de driverul controller‑ului de motor.

### 4.3. Driver OLED SSD1306

Fișiere relevante: `main/drivers/oled_i2c/oled_i2c.c`, `main/drivers/oled_i2c/oled_i2c.h`, `main/config/oled_config.h`

- `oled_config.h` definește:
  - dimensiunea display‑ului (128x64), adresa I2C, dimensiunea fontului 5x7 și spațierea orizontală;
  - pozițiile pe ecran pentru layout‑ul CONFIG/DEBUG (titluri, text, separator, zonă de grafic).
- `oled_i2c.c` implementează:
  - framebuffer‑ul intern și funcții pentru setarea de pixeli, linii orizontale/verticale, dreptunghiuri și text;
  - `oled_init()` care creează panoul SSD1306 peste I2C (folosind `esp_lcd_panel_*`), îl resetează, îl inițializează și pornește afișarea;
  - `oled_draw_hello()` – ecran simplu cu textul „HELLO” centrat;
  - `oled_draw_debug_screen()` – ecran de diagnostic cu două coloane:
    - coloana CONFIG afișează viteza setată (SP:xxx), starea farurilor (HL:ON/OFF) și a mersului înapoi (REV:ON/OFF);
    - coloana DEBUG afișează starea frânei de urgență, viteza actuală și un mic „bar graph” din pătrățele.
- Driverul este integrat în aplicație prin `ui_task`, care apelează periodic `oled_draw_debug_screen()`.

### 4.4. Configuri hardware

Fișiere relevante: `main/config/*.h`

- `hcsr04_config.h` – definește pinii TRIG/ECHO pentru senzorii HC‑SR04 față și spate.
- `rgb_led_config.h` – definește pinii pentru LED‑urile RGB față/spate, frecvența și rezoluția PWM, precum și canalele LEDC asociate.
- `rotary_encoder_config.h` – definește pinii A/B ai encoderului rotativ și pinul butonului.
- `motor_ctrl_config.h` – creat ca schelet pentru a conține adresa I2C, ID‑urile de comenzi și time‑out‑urile pentru controllerul de motor (urmează a fi completat).

### 4.5. Task‑uri FreeRTOS și integrare

Fișier relevant: `main/tasks/app_tasks.c`

- Sunt definite cinci task‑uri:
  - `input_task` – destinat citirii input‑urilor utilizatorului (butoane, encoder);
  - `sensors_task` – pentru citirea senzorilor (HC‑SR04 etc.);
  - `control_task` – pentru calculul comenzilor către motoare și LED‑uri pe baza input‑urilor și senzorilor;
  - `motor_task` – pentru trimiterea efectivă a comenzilor către controllerul de motor (I2C);
  - `ui_task` – pentru actualizarea afișajului OLED și a elementelor de interfață.
- În `app_tasks_init()`:
  - se inițializează busul I2C (`hal_i2c_init()`);
  - se inițializează OLED‑ul (`oled_init()`);
  - se creează efectiv `ui_task`, care rulează și actualizează în buclă ecranul de debug.
- Crearea task‑urilor `input_task`, `sensors_task`, `control_task`, `motor_task` este momentan comentată, urmând a fi activată după implementarea logicii lor.

## 5. Stadiu software – componente în lucru / neimplementate

### 5.1. Drivere senzori și actuatori

- **HC‑SR04** (`main/drivers/hcsr04/hcsr04.c`, `hcsr04.h`):
  - În prezent există doar scheletul cu include‑uri și comentarii.
  - Urmează de implementat: generarea impulsului TRIG, măsurarea duratei ECHO folosind `hal_pulse_out_us` / `hal_pulse_in_us` și conversia timpului în distanță (cm).

- **RGB LED** (`main/drivers/rgb_led/rgb_led.c`, `rgb_led.h`):
  - Codul conține doar include‑uri, fără inițializare și fără funcții de setare a culorii.
  - Urmează de implementat: inițializarea canalelor LEDC pe pinii configurați și API pentru setarea culorilor față/spate (inclusiv eventuale animații).

- **Rotary encoder** (`main/drivers/rotary_encoder/rotary_encoder.c`, `rotary_encoder.h`):
  - Momentan doar schelet.
  - Urmează de implementat: decodarea cuadratură (direcție și pași), gestionarea butonului (click, long‑press) și eventual filtrare/debounce.

- **Motor control I2C** (`main/drivers/motor_ctrl_i2c/motor_ctrl_i2c.c`, `motor_ctrl_i2c.h`):
  - În acest moment conține numai structura de bază.
  - Urmează de definit protocolul cu dsPIC (adrese, comenzi, format mesaje, telemetrie) și implementate funcțiile de trimitere/recepție.

### 5.2. Logica din task‑uri

- `input_task`, `sensors_task`, `control_task` și `motor_task` conțin doar TODO‑uri (comentarii care descriu intenția), fără cod efectiv.
- Nu există încă legături concrete între:
  - valorile citite din senzori (HC‑SR04),
  - poziția encoderului și butoanele,
  - comenzile către motoare și LED‑uri.
- În consecință, în acest moment aplicația rulează doar UI‑ul de test de pe OLED, fără feedback real de la senzori și fără control efectiv al motoarelor sau al LED‑urilor.

### 5.3. Documentație

- Directorul `docs/` nu conținea anterior fișiere; prezentul document descrie statusul actual.
- `README.md` din rădăcina proiectului este încă cel implicit de la exemplul „hello_world” al ESP‑IDF și nu reflectă arhitectura actuală.

## 6. Concluzii și pași următori

- Hardware‑ul este pregătit: toate componentele planificate (OLED, HC‑SR04, RGB LED‑uri, encoder, controller motor) sunt montate, iar firele sunt lipite corect pe placa de prototipare, conform configurațiilor software.
- Partea de infrastructură software (HAL GPIO/Timere, HAL I2C, driver OLED, fișiere de configurare, schelet de task‑uri) este implementată și funcțională pentru afișajul OLED.
- Urmează să fie implementate:
  - driverele HC‑SR04, RGB LED, encoder și controller de motor pe I2C;
  - logica efectivă din `input_task`, `sensors_task`, `control_task`, `motor_task`;
  - integrarea completă a acestor componente în aplicație.

După implementarea acestor părți, proiectul va putea folosi hardware‑ul existent pentru a simula comportamentul „mașinuței” (citire distanță, reacție la obstacole, control viteze și direcții, feedback vizual pe LED‑uri și OLED).

