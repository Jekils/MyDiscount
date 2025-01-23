#define SERVO_PIN1 PB0 
#define SERVO_PIN2 PB1
#define BUTTON1_PIN PB4 
#define BUTTON2_PIN PB3
#define LED_PIN PB1 
#define ANALOG  

#define BUTTON1_PIN PB4 
#define BUTTON2_PIN PB3 

#include <util/delay.h>
#include <avr/io.h>


#define PULSE_MIN 500 
#define PULSE_MAX 2300 

void setServoAngle(int16_t angle,int16_t numServo) {
    
    uint16_t pulseWidth = PULSE_MIN + ((int32_t)(angle + 90) * (PULSE_MAX - PULSE_MIN) / 180);
    for (uint8_t i = 0; i < 50; i++) { 
        PORTB |= (1 << numServo); 
        delayMicroseconds(pulseWidth); 
        PORTB &= ~(1 << numServo); 
        delayMicroseconds(20000 - pulseWidth); 
    }
}

void ADC_init() {
    ADMUX = (1 << MUX0); 
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1); 
}

uint16_t ADC_read() {
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADC;
}

void setup() {
    DDRB |= (1 << SERVO_PIN1); 
    DDRB |= (1 << SERVO_PIN2); 
    DDRB |= (1 << LED_PIN); 
    ADC_init();
    DDRB &= ~(1 << BUTTON1_PIN); 
    DDRB &= ~(1 << BUTTON2_PIN); 

   
    PORTB |= (1 << BUTTON1_PIN); 
    PORTB |= (1 << BUTTON2_PIN);
}
bool st1 = 0;
bool st2 = 0;
void loop() {

      // Ініціалізація ADC
    //uint8_t button1State = PINB & (1 << BUTTON1_PIN); 
    //uint8_t button2State = PINB & (1 << BUTTON2_PIN); 
     
  
   if (PINB & (1 << BUTTON2_PIN)) { 
      setServoAngle(-90, SERVO_PIN1);  
       _delay_ms(500);
       setServoAngle(-90, SERVO_PIN2);  
       _delay_ms(500);  
    } else if (PINB & (1 << BUTTON1_PIN)){ // Якщо PB3 LOW
        setServoAngle(90, SERVO_PIN1);  
        _delay_ms(500);
        setServoAngle(90, SERVO_PIN2);  
        _delay_ms(500);
        
        while(1) {
          
          if (ADC_read() > 600 and st1 == 0 and st2 == 0) {
            setServoAngle(-90, SERVO_PIN1);  
            st1 = 1;
            _delay_ms(1500);
          }
          if (ADC_read() > 600 and st1 == 1 and st2 == 0) {
            setServoAngle(-90, SERVO_PIN2);  
            break;
          }
         }
    }
    _delay_ms(100); 
/*
    if (analogValue < 600) {
        PORTB |= (1 << LED_PIN );
    } else {
        PORTB &= ~(1 << LED_PIN );
    }
*/    
}
 /*  setServoAngle(-90, SERVO_PIN1); 
    _delay_ms(1000);    
    setServoAngle(0, SERVO_PIN1);    
    setServoAngle(90, SERVO_PIN1);   
    _delay_ms(1000);    
    setServoAngle(-90, SERVO_PIN2);  
    _delay_ms(1000);    

    setServoAngle(0, SERVO_PIN2);    
    _delay_ms(1000);     

    setServoAngle(90, SERVO_PIN2);  
    _delay_ms(1000);    
}
*/
/*
#define BUTTON1_PIN PB4 
#define BUTTON2_PIN PB3 
#define LED_PIN PB0    

void setup() {
    DDRB &= ~(1 << BUTTON2_PIN); 
    DDRB &= ~(1 << BUTTON1_PIN); 
    PORTB |= (1 << BUTTON2_PIN);
    PORTB |= (1 << BUTTON1_PIN);
    DDRB |= (1 << LED_PIN);     
}

void loop() {
    if (PINB & (1 << BUTTON2_PIN)) { 
        PORTB |= (1 << LED_PIN); 
    } else if (PINB & (1 << BUTTON1_PIN)){ 
        PORTB &= ~(1 << LED_PIN); 
    }
    _delay_ms(100);
}
*/
