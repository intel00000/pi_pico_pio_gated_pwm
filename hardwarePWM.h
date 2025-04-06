#pragma once

#include "hardware/pwm.h"
#include "hardware/clocks.h"

/**
 * Sets up hardware PWM on the given pin with the requested frequency (Hz) and duty cycle (0..100%).
 * Uses the 8:4 fractional divider (div_int, div_frac4) for best precision.
 * Tries to maximize wrap (up to 65535) so duty resolution is as high as possible.
 *
 * Returns true on success, false if the frequency is out of achievable range.
 */
bool setPWM(uint pin, float freqHz, float dutyCyclePercent, bool verbose = false, bool allowPhaseCorrect = true)
{
    // 1) Validate inputs
    if (freqHz <= 0.f || dutyCyclePercent < 0.f || dutyCyclePercent > 100.f)
    {
        return false;
    }

    // 2) attempt from wrap=65535 down to 1, to maximize the wrap value.
    // freq = SYS_CLK_FREQ / ( (div_int + div_frac4/16) * (wrap + 1) )
    // freq = SYS_CLK_FREQ / ( (div_int + div_frac4/16) * (wrap + 1) * 2) if enabled phase correct
    // We want 1 <= div_int <= 255, 0 <= div_frac4 <= 15
    // We'll store the best wrap, plus the best (div_int, div_frac4).
    uint16_t bestWrap = 0;
    int bestDivInt = 0;
    int bestDivFrac4 = 0; // 4-bit fractional part, 0..15

    for (uint32_t wrap = 65535; wrap >= 1; wrap--)
    {
        // The exact divider we need:
        // float exactDiv = SYS_CLK_FREQ / (freqHz * (wrap + 1) * 2)
        float exactDiv = (float)clock_get_hz(clk_sys) / (freqHz * (wrap + 1) * (allowPhaseCorrect ? 2.f : 1.f));
        int divInt = (int)exactDiv;     // integer part
        if (divInt < 1 || divInt > 255) // div_int must be in the range [1, 255]
        {
            if (verbose)
                Serial.printf("divInt out of range: %d\n", divInt);
            continue; // Not feasible, try next smaller wrap
        }
        if (verbose)
            Serial.printf("wrap=%u, exactDiv=%.6f\n", wrap, exactDiv); // Debugging output
        float frac = (exactDiv - (float)divInt) * 16.f;                // div_frac4 range is 0..15
        int fracInt = (int)frac;                                       // Round to nearest
        if (fracInt > 15)
        {
            fracInt = 0;
            divInt++;
            if (divInt > 255)
            {
                continue; // not feasible, next wrap
            }
        }

        // Now we have integer+fraction: (divInt + fracInt/16)
        // This is a valid combination. use the first (largest wrap) that fits.
        bestWrap = wrap;
        bestDivInt = divInt;
        bestDivFrac4 = fracInt;
        break; // done searching
    }

    if (bestWrap == 0)
    {
        return false; // Could not find a valid wrap, freq out of range
    }

    // Now we have bestWrap, bestDivInt, bestDivFrac4
    // 3) Configure the PWM slice
    uint sliceNum = pwm_gpio_to_slice_num(pin);
    gpio_init(pin);
    gpio_set_function(pin, GPIO_FUNC_PWM);                                                // Connect pin to PWM
    pwm_config config = pwm_get_default_config();                                         // Get configuration
    pwm_config_set_phase_correct(&config, allowPhaseCorrect);                             // Set phase correct
    pwm_config_set_wrap(&config, (uint16_t)bestWrap);                                     // Set wrap
    pwm_config_set_clkdiv_int_frac(&config, (uint32_t)bestDivInt, (uint8_t)bestDivFrac4); // Set integer+fraction
    uint16_t level = (uint16_t)lroundf((dutyCyclePercent / 100.f) * (bestWrap + 1));      // compare level for duty cycle
    pwm_set_gpio_level(pin, level);

    pwm_init(sliceNum, &config, true); // Start PWM

    return true;
}
