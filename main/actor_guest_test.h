#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Actor/Guest test for IR radiation communication
 * 
 * This test demonstrates:
 * - Guest mode: Receives IR signals and applies radiation
 * - Actor mode: Sends IR signals with radiation values
 * - Button hold (A+C for 5s) to switch between modes
 * - 2-second cooldown for guest to prevent spam
 * - Movement detection to reduce radiation
 */
void actor_guest_test_main(void);

#ifdef __cplusplus
}
#endif


