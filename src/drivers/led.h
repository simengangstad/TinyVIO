#ifndef LED_H
#define LED_H

namespace led {
    /**
     * @brief Initializes the user LED.
     */
    void init();

    /**
     * @brief Turns the user LED on.
     */
    void on();

    /**
     * @brief Turns the user LED off.
     */
    void off();

    /**
     * @brief Toggles the user LED.
     */
    void toggle();
}

#endif
