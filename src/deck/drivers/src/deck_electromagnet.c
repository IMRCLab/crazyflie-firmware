/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2023 BitCraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * deck_electromagnet.c - Deck driver for the electromagnet deck
 */

#define DEBUG_MODULE "DeckElectromagnet"
#include "debug.h"

#include "deck.h"
#include "param.h"

#define GATE_PIN    DECK_GPIO_IO1
uint8_t state = 0;

static void activateCallback(void);

static void initializeDeckElectromagnet()
{
	DEBUG_PRINT("Initializing electromagnet deck!\n");

	// Configure pin IO1 as an output with pull-down resistor
	pinMode(GATE_PIN, OUTPUT);

	// initialize with given default state
	activateCallback();

	// Test: Activate electromagnet
	// digitalWrite(GATE_PIN, HIGH);

	DEBUG_PRINT("Electromagnet deck initialized successfully!\n");
}

static void activateCallback(void)
{
	if (state) {
		DEBUG_PRINT("Activating electromagnet deck!\n");
		digitalWrite(GATE_PIN, HIGH);
	} else {
		DEBUG_PRINT("Deactivating electromagnet deck!\n");
		digitalWrite(GATE_PIN, LOW);
	}
}

static const DeckDriver driverElectromagnet = {
	.vid = 0,
  	.pid = 0,
	.name = "deckElectromagnet",
	.usedGpio = DECK_USING_IO_1,
	.init = initializeDeckElectromagnet,
};

DECK_DRIVER(driverElectromagnet);

// add a parameter to the deck driver
PARAM_GROUP_START(electromagnet)
PARAM_ADD_WITH_CALLBACK(PARAM_UINT8, activate, &state, &activateCallback)
PARAM_GROUP_STOP(electromagnet)
