#define SERVO_PIN1 PB0 // Вибір піну для сигналу
#define SERVO_PIN2 PB1 // Вибір піну для сигналу
#define BUTTON1_PIN PB4 // Кнопка 1
#define BUTTON2_PIN PB3 // Кнопка 2
#define LED_PIN PB1 // Сввітлодіод
#define ANALOG  

#define BUTTON1_PIN PB4 // PB3 (Button 2)
#define BUTTON2_PIN PB3 // PB3 (Button 2)

#include <util/delay.h>
#include <avr/io.h>

// Задай мінімальну і максимальну ширину імпульсу для твого серво
#define PULSE_MIN 500 // мінімум для твого серво (~0°)
#define PULSE_MAX 2300 // максимум для твого серво (~180°)

void setServoAngle(int16_t angle,int16_t numServo) {
    // Розрахунок ширини імпульсу для заданого кута
    uint16_t pulseWidth = PULSE_MIN + ((int32_t)(angle + 90) * (PULSE_MAX - PULSE_MIN) / 180);
    for (uint8_t i = 0; i < 50; i++) { // 50 імпульсів для 1 секунди
        PORTB |= (1 << numServo); // HIGH
        delayMicroseconds(pulseWidth); // Тривалість HIGH
        PORTB &= ~(1 << numServo); // LOW
        delayMicroseconds(20000 - pulseWidth); // Тривалість LOW
    }
}

void ADC_init() {
    ADMUX = (1 << MUX0); // Вибір ADC1 (PB2)
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1); // Включення ADC, переддільник 64
}

uint16_t ADC_read() {
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADC;
}

void setup() {
    DDRB |= (1 << SERVO_PIN1); // Налаштування SERVO_PIN як виходу
    DDRB |= (1 << SERVO_PIN2); // Налаштування SERVO_PIN як виходу
    DDRB |= (1 << LED_PIN); // Налаштування піну 3 (PB3) на вихід
    ADC_init();
    DDRB &= ~(1 << BUTTON1_PIN); // PB4 як вхід
    DDRB &= ~(1 << BUTTON2_PIN); // PB3 як вхід

    // Увімкнення внутрішніх підтягувальних резисторів
    PORTB |= (1 << BUTTON1_PIN); // Підтягувальний резистор для PB4
    PORTB |= (1 << BUTTON2_PIN);
}
bool st1 = 0;
bool st2 = 0;
void loop() {

      // Ініціалізація ADC
    //uint8_t button1State = PINB & (1 << BUTTON1_PIN); // Стан PB4
    //uint8_t button2State = PINB & (1 << BUTTON2_PIN); // Стан PB3
     
  
   if (PINB & (1 << BUTTON2_PIN)) { // Якщо PB3 HIGH
      setServoAngle(-90, SERVO_PIN1);  // -90 градусів для першого серво
       _delay_ms(500);
       setServoAngle(-90, SERVO_PIN2);  // -90 градусів для першого серво
       _delay_ms(500);  
    } else if (PINB & (1 << BUTTON1_PIN)){ // Якщо PB3 LOW
        setServoAngle(90, SERVO_PIN1);  // -90 градусів для першого серво
        _delay_ms(500);
        setServoAngle(90, SERVO_PIN2);  // -90 градусів для першого серво
        _delay_ms(500);
        
        while(1) {
          
          if (ADC_read() > 600 and st1 == 0 and st2 == 0) {
            setServoAngle(-90, SERVO_PIN1);  // -90 градусів для першого серво
            st1 = 1;
            _delay_ms(1500);
          }
          if (ADC_read() > 600 and st1 == 1 and st2 == 0) {
            setServoAngle(-90, SERVO_PIN2);  // -90 градусів для першого серво
            break;
          }
         }
    }
    _delay_ms(100); // Антидребезг
/*
    if (analogValue < 600) {
        PORTB |= (1 << LED_PIN );
    } else {
        PORTB &= ~(1 << LED_PIN );
    }
*/    
}
 /*  setServoAngle(-90, SERVO_PIN1);  // -90 градусів для першого серво
    _delay_ms(1000);     // Затримка 1 секунда

    setServoAngle(0, SERVO_PIN1);    // 0 градусів для першого серво
    _delay_ms(1000);     // Затримка 1 секунда

    setServoAngle(90, SERVO_PIN1);   // 90 градусів для першого серво
    _delay_ms(1000);     // Затримка 1 секунда

    setServoAngle(-90, SERVO_PIN2);  // -90 градусів для другого серво
    _delay_ms(1000);     // Затримка 1 секунда

    setServoAngle(0, SERVO_PIN2);    // 0 градусів для другого серво
    _delay_ms(1000);     // Затримка 1 секунда

    setServoAngle(90, SERVO_PIN2);   // 90 градусів для другого серво
    _delay_ms(1000);     // Затримка 1 секунда

}
*/
/*
#define BUTTON1_PIN PB4 // PB3 (Button 2)
#define BUTTON2_PIN PB3 // PB3 (Button 2)
#define LED_PIN PB0     // Світлодіод на PB5

void setup() {
    DDRB &= ~(1 << BUTTON2_PIN); // PB3 як вхід
    DDRB &= ~(1 << BUTTON1_PIN); 
    PORTB |= (1 << BUTTON2_PIN); // Увімкнення підтягувального резистора
    PORTB |= (1 << BUTTON1_PIN);
    DDRB |= (1 << LED_PIN);      // PB5 як вихід
}

void loop() {
    if (PINB & (1 << BUTTON2_PIN)) { // Якщо PB3 HIGH
        PORTB |= (1 << LED_PIN); // Увімкнути світлодіод
    } else if (PINB & (1 << BUTTON1_PIN)){ // Якщо PB3 LOW
        PORTB &= ~(1 << LED_PIN); // Вимкнути світлодіод
    }
    _delay_ms(100); // Антидребезг
}
*/
