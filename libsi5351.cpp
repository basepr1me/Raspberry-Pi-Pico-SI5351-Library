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

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "libsi5351.h"

#define DEBUG		 1
#define DPRINTF(x,y)	 do { if (DEBUG) { printf("%s", x); sleep_ms(y); }} \
			    while(0)

static void		 begin(struct env *);
static void		 stop(void);
static void		 write(struct env *, const uint8_t *, size_t, bool);
static void		 read(struct env *, uint8_t *, size_t, bool);
static void		 set_phase(struct env *, uint8_t, uint8_t);

Si5351::Si5351(struct env *config)
{
	uint8_t i;

	if (DEBUG)
		sleep_ms(3000);
	DPRINTF("Initializing I2C\n", 500);

	switch(config->i2c_channel) {
	case I2C1:
		config->i2c = i2c1;
		i2c_init(config->i2c, config->i2c_speed);
		gpio_set_function(PICO_I2C1_SDA_PIN, GPIO_FUNC_I2C);
		gpio_set_function(PICO_I2C1_SCL_PIN, GPIO_FUNC_I2C);
		gpio_pull_up(PICO_I2C1_SDA_PIN);
		gpio_pull_up(PICO_I2C1_SCL_PIN);
		break;
	case I2C0:
	default:
		config->i2c = i2c0;
		i2c_init(config->i2c, config->i2c_speed);
		gpio_set_function(PICO_I2C0_SDA_PIN, GPIO_FUNC_I2C);
		gpio_set_function(PICO_I2C0_SCL_PIN, GPIO_FUNC_I2C);
		gpio_pull_up(PICO_I2C0_SDA_PIN);
		gpio_pull_up(PICO_I2C0_SCL_PIN);
		break;
	}

	begin(config);
}

void
Si5351::enable(struct env *config)
{
	uint8_t data[2];

	if (!config->initialized) {
		DPRINTF("SI5351 not initialized!\n", 0);
		stop();
	}

	memset(&data, 0, sizeof(data));
	data[0] = SI5351_REGISTER_3_OUTPUT_ENABLE_CONTROL;
	data[1] = 0x00;
	write(config, data, 2, true);

	DPRINTF("Outputs enabled\n", 500);
}

void
Si5351::disable(struct env *config)
{
	uint8_t data[2];

	if (!config->initialized) {
		DPRINTF("SI5351 not initialized!\n", 0);
		stop();
	}

	DPRINTF("Outputs disabled\n", 500);

	memset(&data, 0, sizeof(data));
	data[0] = SI5351_REGISTER_3_OUTPUT_ENABLE_CONTROL;
	data[1] = 0xFF;
	write(config, data, 2, true);
}

static void
begin(struct env *config)
{
	int ret;
	uint8_t data[2];
	uint8_t status = 1;

	/* give module a little initization time when not debugging */
	if (!DEBUG)
		sleep_ms(100);

	DPRINTF("Checking for SI5351 device on bus\n", 500);

	memset(&data, 0, sizeof(data));
    	ret = i2c_read_blocking(config->i2c, SI5351_ADDR_LOW, &data[0], 1,
	    false);

	if (ret < 0) {
    		ret = i2c_read_blocking(config->i2c, SI5351_ADDR_HIGH, &data[1],
		    1, false);
		if (ret < 0) {
			DPRINTF("SI5351 not found\n", 0);
			stop();
		} else
			DPRINTF("SI5351 found at 0x61\n", 500);
	} else
		DPRINTF("SI5351 found at 0x60\n", 500);

	DPRINTF("Checking SI5351 status\n", 500);

	memset(&data, 0, sizeof(data));

	data[0] = SI5351_REGISTER_0_DEVICE_STATUS;
	while (status) {
		write(config, &data[0], 1, true);
		read(config, &data[1], 1, false);

		if (ret < 0) {
			DPRINTF("Status read failed!\n", 0);
			stop();
		} else {
			status = ((data[1] >> 7) & 0x01);
			if (status)
				DPRINTF(".", 500);
		}
	}

	DPRINTF("Configuring SI5351\n", 1000);

	memset(&data, 0, sizeof(data));
	data[0] = SI5351_REGISTER_3_OUTPUT_ENABLE_CONTROL;
	data[1] = 0xFF;
	write(config, data, 2, true);

	DPRINTF("Outputs disabled\n", 500);

	data[0] = SI5351_REGISTER_16_CLK0_CONTROL;
	data[1] = 0x80;
	write(config, data, 2, false);

	data[0] = SI5351_REGISTER_17_CLK1_CONTROL;
	data[1] = 0x80;
	write(config, data, 2, false);

	data[0] = SI5351_REGISTER_18_CLK2_CONTROL;
	data[1] = 0x80;
	write(config, data, 2, false);

	data[0] = SI5351_REGISTER_19_CLK3_CONTROL;
	data[1] = 0x80;
	write(config, data, 2, false);

	data[0] = SI5351_REGISTER_20_CLK4_CONTROL;
	data[1] = 0x80;
	write(config, data, 2, false);

	data[0] = SI5351_REGISTER_21_CLK5_CONTROL;
	data[1] = 0x80;
	write(config, data, 2, false);

	data[0] = SI5351_REGISTER_22_CLK6_CONTROL;
	data[1] = 0x80;
	write(config, data, 2, false);

	data[0] = SI5351_REGISTER_23_CLK7_CONTROL;
	data[1] = 0x80;
	write(config, data, 2, false);

	data[0] = SI5351_REGISTER_183_CRYSTAL_INTERNAL_LOAD_CAPACITANCE;
	data[1] = config->crystal_load;
	write(config, data, 2, false);

	data[0] = SI5351_REGISTER_149_SPREAD_SPECTRUM_PARAMETERS;
	data[1] &= ~0x80;
	write(config, data, 2, false);

	config->initialized = true;
}

void
Si5351::setup_pll(struct env *config, uint8_t pll)
{
	uint32_t P1, P2, P3;
	uint8_t addr, data[9];
	float freq_vco;

	DPRINTF("Configuring PLL\n", 500);

	if (pll < SI5351_PLL_A || pll > SI5351_PLL_B) {
		DPRINTF("Bad PLL value!\n", 0);
		stop();
	}
	if (!config->initialized) {
		DPRINTF("SI5351 not initialized!\n", 0);
		stop();
	}
	if (config->f_multiplier < 15 || config->f_multiplier > 90) {
		DPRINTF("Bad config->f_multiplier!\n", 0);
		stop();
	}
	if (config->f_denominator == 0) {
		DPRINTF("Divide by zero!\n", 0);
		stop();
	}
	if (config->f_numerator > 0xFFFFF) {
		DPRINTF("config->f_numerator exceeds 20-bit limit!\n", 0);
		stop();
	}
	if (config->f_denominator > 0xFFFFF) {
		DPRINTF("config->f_denominator exceeds 20-bit limit!\n", 0);
		stop();
	}

	if (config->f_numerator == 0) {
		P1 = 128 * config->f_multiplier - 512;
		P2 = config->f_numerator;
		P3 = config->f_denominator;
	} else {
		P1 = (uint32_t) (128 * config->f_multiplier + floor (128 *
		    ((float) config->f_numerator /
		    (float) config->f_denominator)) - 512);
		P2 = (uint32_t) (128 * config->f_numerator -
		    config->f_denominator * floor (128 *
		    ((float) config->f_numerator /
		    (float) config->f_denominator)));
		P3 = config->f_denominator;
	}

	switch(pll) {
	case SI5351_PLL_A:
		addr = SI5351_REGISTER_26_MSNA_P3_A_15_8;
		break;
	case SI5351_PLL_B:
		addr = SI5351_REGISTER_34_MSNA_P3_B_15_8;
		break;
	}
	memset(&data, 0, sizeof(data));
	data[0] = addr;
	data[1] = (P3 & 0x0000FF00) >> 8;
	data[2] = (P3 & 0x000000FF);
	data[3] = (P1 & 0x00030000) >> 16;
	data[4] = (P1 & 0x0000FF00) >> 8;
	data[5] = (P1 & 0x000000FF);
	data[6] = ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16);
	data[7] = (P2 & 0x0000FF00) >> 8;
	data[8] = (P2 & 0x000000FF);
	write(config, data, 9, false);

	memset(&data, 0, sizeof(data));
	data[0] = SI5351_REGISTER_177_PLL_RESET;
	data[1] = 0xAC;
	write(config, data, 2, false);

	freq_vco = config->crystal_freq * (config->f_multiplier +
	    (float) config->f_numerator /
	    (float) config->f_denominator);

	switch(pll) {
	case SI5351_PLL_A:
	default:
		DPRINTF("SI5351_PLL_A configured\n", 500);
		config->pll_a_configured = true;
		config->pll_a_freq = (uint32_t) floor(freq_vco);
		break;
	case SI5351_PLL_B:
		DPRINTF("SI5351_PLL_B configured\n", 500);
		config->pll_b_configured = true;
		config->pll_b_freq = (uint32_t) floor(freq_vco);
		break;
	}
}

void
Si5351::setup_multisynth(struct env *config, uint8_t output, uint8_t pll)
{
	uint32_t P1, P2, P3;
	uint8_t clk_ctrl_reg = 0x0F;
	uint8_t addr, data[9];

	DPRINTF("Configuring Multisynth\n", 500);

	if (output < SI5351_OUTPUT_1 || output > SI5351_OUTPUT_3) {
		DPRINTF("Bad output value!\n", 0);
		stop();
	}
	if (pll < SI5351_PLL_A || pll > SI5351_PLL_B) {
		DPRINTF("Bad PLL value!\n", 0);
		stop();
	}
	if (!config->initialized) {
		DPRINTF("SI5351 not initialized!\n", 0);
		stop();
	}
	if (config->m_divider < 3 || config->m_divider > 2049) {
		DPRINTF("Bad config->m_divider!\n", 0);
		stop();
	}
	if (config->m_denominator == 0) {
		DPRINTF("Divide by zero!\n", 0);
		stop();
	}
	if (config->m_numerator > 0xFFFFF) {
		DPRINTF("config->m_numerator exceeds 20-bit limit!\n", 0);
		stop();
	}
	if (config->m_denominator > 0xFFFFF) {
		DPRINTF("config->m_denominator exceeds 20-bit limit!\n", 0);
		stop();
	}

	switch(pll) {
	case SI5351_PLL_A:
	default:
		if (!config->pll_a_configured) {
			DPRINTF("PLL A not configured!\n", 0);
			stop();
		}
		break;
	case SI5351_PLL_B:
		if (!config->pll_b_configured) {
			DPRINTF("PLL B not configured!\n", 0);
			stop();
		}
		break;
	}

	switch(output) {
	case SI5351_OUTPUT_1:
		addr = SI5351_REGISTER_42_MULTISYNTH0_PARAMETERS_1;
		break;
	case SI5351_OUTPUT_2:
		addr = SI5351_REGISTER_50_MULTISYNTH1_PARAMETERS_1;
		break;
	case SI5351_OUTPUT_3:
		addr = SI5351_REGISTER_58_MULTISYNTH2_PARAMETERS_1;
		break;
	}

	if (config->m_numerator == 0) {
		P1 = 128 * config->m_divider - 512;
		P2 = 0;
		P3 = config->m_denominator;
	} else if (config->m_denominator == 1) {
		P1 = 128 * config->m_divider + 128 * config->m_numerator - 512;
		P2 = 128 * config->m_numerator - 128;
		P3 = 1;
	} else {
		P1 = (uint32_t) (128 * config->m_divider + floor(128 *
		    ((float) config->m_numerator /
		    (float) config->m_denominator)) - 512);
		P2 = (uint32_t) (128 * config->m_numerator -
		    config->m_denominator *
		    floor(128 * ((float) config->m_numerator /
		    (float) config->m_denominator)));
		P3 = config->m_denominator;
	}

	memset(&data, 0, sizeof(data));
	data[0] = addr;
	data[1] = (P3 & 0xFF00) >> 8;
	data[2] = P3 & 0xFF;
	data[3] = ((P1 & 0x30000) >> 16);
	data[4] = (P1 & 0xFF00) >> 8;
	data[5] = P1 & 0xFF;
	data[6] = ((P3 & 0xF0000) >> 12) | ((P2 & 0xF0000) >> 16);
	data[7] = (P2 & 0xFF00) >> 8;
	data[8] = P2 & 0xFF;
	write(config, data, 9, false);

	switch (output) {
	case SI5351_OUTPUT_1:
		addr = SI5351_REGISTER_16_CLK0_CONTROL;
		break;
	case SI5351_OUTPUT_2:
		addr = SI5351_REGISTER_17_CLK1_CONTROL;
		break;
	case SI5351_OUTPUT_3:
		addr = SI5351_REGISTER_18_CLK2_CONTROL;
		break;
	}

	if (config->phase != PHASE0) {
		clk_ctrl_reg |= (1 << 4);
		set_phase(config, pll, output);
	}
	if (pll == SI5351_PLL_B)
		clk_ctrl_reg |= (1 << 5);
	if (config->m_numerator == 0 && config->phase == PHASE0)
		clk_ctrl_reg |= (1 << 6);

	memset(&data, 0, sizeof(data));
	data[0] = addr;
	data[1] = clk_ctrl_reg;
	write(config, data, 2, false);
}

void
Si5351::setup_r_div(struct env *config, uint8_t output, uint8_t divider)
{
	uint8_t addr, data[2];

	if (output < SI5351_OUTPUT_1 || output > SI5351_OUTPUT_3) {
		DPRINTF("Bad output value!\n", 0);
		stop();
	}
	if (!config->initialized) {
		DPRINTF("SI5351 not initialized!\n", 0);
		stop();
	}

	switch(output) {
	case SI5351_OUTPUT_1:
		addr = SI5351_REGISTER_44_MULTISYNTH0_PARAMETERS_3;
		break;
	case SI5351_OUTPUT_2:
		addr = SI5351_REGISTER_52_MULTISYNTH1_PARAMETERS_3;
		break;
	case SI5351_OUTPUT_3:
		addr = SI5351_REGISTER_60_MULTISYNTH2_PARAMETERS_3;
		break;
	}

	memset(&data, 0, sizeof(data));
	data[0] = addr;

	write(config, &data[0], 1, true);
	read(config, &data[1], 1, false);

	data[1] &= 0x0F;
	divider &= 0x07;
	divider <<= 4;
	data[1] &= divider;

	write(config, data, 2, false);
}

static void
set_phase(struct env *config, uint8_t pll, uint8_t output)
{
	float freq_vco;
	uint8_t offset, p_m, addr, data[6];

	DPRINTF("Configuring Phase\n", 500);

	if (output < SI5351_OUTPUT_1 || output > SI5351_OUTPUT_3) {
		DPRINTF("Bad output value!\n", 0);
		stop();
	}
	if (pll < SI5351_PLL_A || pll > SI5351_PLL_B) {
		DPRINTF("Bad PLL value!\n", 0);
		stop();
	}
	if (!config->initialized) {
		DPRINTF("SI5351 not initialized!\n", 0);
		stop();
	}
	if (config->phase < PHASE0 || config->phase > PHASE270) {
		DPRINTF("Bad phase setting!\n", 0);
		stop();
	}
	switch(pll) {
	case SI5351_PLL_A:
		freq_vco = (float) config->pll_a_freq;
		break;
	case SI5351_PLL_B:
		freq_vco = (float) config->pll_b_freq;
		break;
	}
	switch (output) {
	case SI5351_OUTPUT_1:
		addr = SI5351_REGISTER_165_CLK0_INITIAL_PHASE_OFFSET;
		break;
	case SI5351_OUTPUT_2:
		addr = SI5351_REGISTER_166_CLK1_INITIAL_PHASE_OFFSET;
		break;
	case SI5351_OUTPUT_3:
		addr = SI5351_REGISTER_167_CLK2_INITIAL_PHASE_OFFSET;
		break;
	}

	if (config->phase == PHASE180)
		p_m = 2;
	else
		p_m = 4;

	offset = floor((1 / (p_m * (freq_vco / config->m_divider))) * 4 *
	    freq_vco);

	if (offset % 2)
		offset += 1;

	if (offset > 126)
		offset = 126;

	memset(&data, 0, sizeof(data));
	data[0] = addr;
	data[1] = offset;
	write(config, data, 9, false);

	memset(&data, 0, sizeof(data));
	data[0] = SI5351_REGISTER_177_PLL_RESET;
	data[1] = 0xAC;
	write(config, data, 2, false);
}

static void
write(struct env *config, const uint8_t *src, size_t len, bool nostop)
{
	int ret;

	ret = i2c_write_blocking(config->i2c, config->i2c_addr, src, len,
	    nostop);
	sleep_us(200);
	if (ret == PICO_ERROR_GENERIC) {
		DPRINTF("WRITE FAILED!\n", 0);
		stop();
	}
}

static void
read(struct env *config, uint8_t *dst, size_t len, bool nostop)
{
	int ret;

	ret = i2c_read_blocking(config->i2c, config->i2c_addr, dst, len,
	    nostop);
	sleep_us(200);
	if (ret < 0) {
		DPRINTF("READ FAILED!\n", 0);
		stop();
	}
}

static void
stop(void)
{
	DPRINTF("STOPPING ...", 10);
	while(1);
}
