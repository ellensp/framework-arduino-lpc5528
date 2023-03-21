
#ifndef _GPIO_H_
#define _GPIO_H_

/*!
 * @addtogroup GPIO
 * @{
 */

/***********************************************************************************************************************
 * API
 **********************************************************************************************************************/
// typedef int16_t pin_t;

namespace LPC5528 {

[[gnu::always_inline]] inline void gpio_set_input(const pin_t pin) {
   pinMode(pin,INPUT);
}

[[gnu::always_inline]] inline void gpio_set_output(const pin_t pin) {
  pinMode(pin,OUTPUT);
}

[[gnu::always_inline]] inline void gpio_direction(const pin_t pin, bool value) {
  
}

[[gnu::always_inline, nodiscard]] inline bool gpio_direction(const pin_t pin) {
  return true;
}

[[gnu::always_inline]] inline void gpio_set(const pin_t pin) {
digitalWrite(pin,HIGH); 
}

[[gnu::always_inline]] inline void gpio_clear(const pin_t pin) {
digitalWrite(pin,LOW);  
}

[[gnu::always_inline]] inline void gpio_set_port(const uint8_t port, const uint32_t pinbitset) {
  
}

[[gnu::always_inline]] inline void gpio_clear_port(const uint8_t port, const uint32_t pinbitset) {
  
}

[[gnu::always_inline]] inline void gpio_set_port_mask(const uint8_t port, const uint32_t pinbitset) {

}

[[gnu::always_inline]] inline void gpio_set(const pin_t pin, const bool value) {
  pinMode(pin,OUTPUT);
  digitalWrite(pin,value);  
}

[[gnu::always_inline, nodiscard]] inline bool gpio_get(const pin_t pin) {
  // return true;
  return (digitalRead(pin));
}

[[gnu::always_inline]] inline void gpio_toggle(const pin_t pin) {
  
}

[[gnu::always_inline, nodiscard]] constexpr bool gpio_interrupt_capable(const pin_t pin) {
  return true;
}

}

/*!
 * @}
 */
#endif /* _GPIO_H_ */

/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
