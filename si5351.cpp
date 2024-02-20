/*
 * Copyright (c) 2022, 2024 Tracey Emery <tracey@openbsd.org>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <stdio.h>
#include <stdlib.h>

#include "pico/malloc.h"
#include "pico/stdlib.h"

#include "hardware/i2c.h"
#include "libsi5351.h"

int main(void);

/*
 * PLL A
 * Input Frequency (MHz) = 25.000000000
 * VCO Frequency (MHz) = 864.300000000
 * Feedback Divider = 34 143/250
 * SSC disabled
 *
 * Output Clocks
 * Channel 0
 * Output Frequency (MHz) = 144.050000000
 * Multisynth Output Frequency (MHz) = 144.050000000
 * Multisynth Divider = 6
 * R Divider = 1
 * PLL source = PLLA
 * Initial phase offset (ns) = 0.000
 * Powered down = No
 * Inverted = No
 * Drive Strength = b11
 * Disable State = Low
 * Clock Source = b11
 */

/* PLL A
 * Input Frequency (MHz) = 25.000000000
 * VCO Frequency (MHz) =  864.210000000
 * Feedback Divider = 34  1421/2500
 * SSC disabled
 *
 * Output Clocks
 * Channel 0
 * Output Frequency (MHz) = 144.035000000
 * Multisynth Output Frequency (MHz) = 144.035000000
 * Multisynth Divider = 6
 * R Divider = 1
 * PLL source = PLLA
 * Initial phase offset (ns) = 0.000
 * Powered down = No
 * Inverted = No
 * Drive Strength = b11
 * Disable State = Low
 * Clock Source = b11
 */

/*
 * XXX: Trying to set the SI5351 to the 144.05 frequency resulted in a central
 * frequency of 144.06, with a 1MHz bandwidth output. Setting to 144.035
 * frequency outputs 144.045 - 144.055, with the central desired frequency of
 * 144.05. Not sure if this is isolated to this particular board or not.
 */

int
main()
{
	struct env *config;

	config = (struct env *) malloc(sizeof(struct env *));

	config->i2c_addr = SI5351_ADDR_LOW;
	config->i2c_channel = I2C0;
	config->i2c_speed = 400000;
	config->initialized = false;
	config->crystal_freq = SI5351_CRYSTAL_FREQ_25MHZ;
	config->crystal_load = SI5351_CRYSTAL_LOAD_10PF;
	config->phase = PHASE180;

	config->pll_a_configured = false;
	config->pll_a_freq = 0;
	config->pll_b_configured = false;
	config->pll_b_freq = 0;

	/*
	 * Get these values from the SILabs Clockbuilder tool
	 */

	config->f_multiplier = 34;
	config->f_numerator = 1421;
	config->f_denominator = 2500;

	config->m_divider = 6;
	config->m_numerator = 0;
	config->m_denominator = 1;

	stdio_init_all();

	Si5351 si(config);
	si.setup_pll(config, SI5351_PLL_A);
	si.setup_multisynth(config, SI5351_OUTPUT_1, SI5351_PLL_A);
	si.setup_r_div(config, SI5351_OUTPUT_1, SI5351_R_DIV_1);
	si.enable(config);

	free(config);

	printf("\nDone!\n");
	while(1);

	return 0;
}
