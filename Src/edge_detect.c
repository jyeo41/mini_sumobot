#include "edge_detect.h"
#include "qre1113.h"
#include "assert_handler.h"
#include "trace.h"
#include <stdbool.h>

/* 16 possible edges because 2^4. We use 4 different sensors */
#define NUMBER_OF_EDGE_POSSIBILITIES    16

/* 4096 or high voltage means black surface, 0 or low voltage means white surface.
 * The sensor's range is really short, so having the threshold be at about 75% seems to be a good threshold.
 * NOTE: Can still be up for debate and testing in the future. */
#define EDGE_DETECTED_THRESHOLD         3000

static bool initialized = false;

/* This lookup table is to be used with a bitmask that maps each of the 4 sensors to a bit, in a 4 bit wide variable.
 * The sensors will be placed on the bottom corners of a square/rectangle shape chassis.
 * Front Left is bit 0
 * Front Right is bit 1
 * Back Left is bit 2
 * Back Right is bit 3
 *
 * Note: There is no detection for detecting 3 sensors because the edge isn't wide enough to have 3 sensors detect it all at once.
 *
 * Example: 0b0110 bitmask would mean, the Front Right and Back Left sensors are detecting an edge, which would mean
 *          there is an edge on the left side of the sumobot. */
static const edge_detect_location_e edge_locations_lookup[NUMBER_OF_EDGE_POSSIBILITIES] = {
    EDGE_DETECT_LOCATION_NONE,              /* 0b0000 */
    EDGE_DETECT_LOCATION_FRONT_LEFT,        /* 0b0001 */
    EDGE_DETECT_LOCATION_FRONT_RIGHT,       /* 0b0010 */
    EDGE_DETECT_LOCATION_FRONT,             /* 0b0011 */
    EDGE_DETECT_LOCATION_BACK_LEFT,         /* 0b0100 */
    EDGE_DETECT_LOCATION_LEFT,              /* 0b0101 */
    EDGE_DETECT_LOCATION_DIAGONAL_RIGHT,    /* 0b0110 */
    EDGE_DETECT_LOCATION_NONE,              /* 0b0111 */
    EDGE_DETECT_LOCATION_BACK_RIGHT,        /* 0b1000 */
    EDGE_DETECT_LOCATION_DIAGONAL_LEFT,     /* 0b1001 */
    EDGE_DETECT_LOCATION_RIGHT,             /* 0b1010 */
    EDGE_DETECT_LOCATION_NONE,              /* 0b1011 */
    EDGE_DETECT_LOCATION_BACK,              /* 0b1100 */
    EDGE_DETECT_LOCATION_NONE,              /* 0b1101 */
    EDGE_DETECT_LOCATION_NONE,              /* 0b1110 */
    EDGE_DETECT_LOCATION_NONE,              /* 0b1111 */
};

void edge_detect_initialize(void)
{
    ASSERT(!initialized);
    qre1113_initialize();
    initialized = true;
}

/* Initially the logic was done by using a giant nested if else statement, but to clean up the code and to
 * make the code more readable, the use of a bitmask and lookup table gives a more elegant solution. */
// cppcheck-suppress unusedFunction
edge_detect_location_e edge_detect_lookup(void)
{
    /* Initialize a struct that will extract the ADC voltages and store those values. */
    qre1113_voltages_t voltages;
    qre1113_get_voltages(&voltages);

    /* Bitmask that represents each sensor as a single binary bit. Useful technique because the sensors is used
     * to only detect white or black surfaces. Used in conjunction with the lookup table to avoid giant nested if else. */
    const uint8_t edge_detected_mask = 
        ((voltages.front_left < EDGE_DETECTED_THRESHOLD) << 0) |
        ((voltages.front_right < EDGE_DETECTED_THRESHOLD) << 1) |
        ((voltages.back_left < EDGE_DETECTED_THRESHOLD) << 2) |
        ((voltages.back_right < EDGE_DETECTED_THRESHOLD) << 3);

    return edge_locations_lookup[edge_detected_mask];
}

/* Testing function to print to terminal if the sensors are detecting the correct locations.
 *
 * Call this function like so:
 * edge_detect_location_string(edge_detect_lookup());
 *
 * It will use the return output from the edge_detect_lookup() function as its input
 * and convert the returned enum to a string representation then print it out. */ 
// cppcheck-suppress unusedFunction
void edge_detect_location_print(edge_detect_location_e edge)
{
    /* Assign the appropriate string literal at run time */
    const char* edge_string;

    switch (edge) {
        case EDGE_DETECT_LOCATION_NONE:
            // cppcheck-suppress unreadVariable
            edge_string = "SAFE";
            break;
        case EDGE_DETECT_LOCATION_FRONT_LEFT:
            // cppcheck-suppress unreadVariable
            edge_string = "FRONT LEFT";
            break;
        case EDGE_DETECT_LOCATION_FRONT_RIGHT:
            // cppcheck-suppress unreadVariable
            edge_string = "FRONT RIGHT";
            break;
        case EDGE_DETECT_LOCATION_BACK_LEFT:
            // cppcheck-suppress unreadVariable
            edge_string = "BACK LEFT";
            break;
        case EDGE_DETECT_LOCATION_BACK_RIGHT:
            // cppcheck-suppress unreadVariable
            edge_string = "BACK RIGHT";
            break;
        case EDGE_DETECT_LOCATION_FRONT:
            // cppcheck-suppress unreadVariable
            edge_string = "FRONT";
            break;
        case EDGE_DETECT_LOCATION_BACK:
            // cppcheck-suppress unreadVariable
            edge_string = "BACK";
            break;
        case EDGE_DETECT_LOCATION_LEFT:
            // cppcheck-suppress unreadVariable
            edge_string = "LEFT";
            break;
        case EDGE_DETECT_LOCATION_RIGHT:
            // cppcheck-suppress unreadVariable
            edge_string = "RIGHT";
            break;
        case EDGE_DETECT_LOCATION_DIAGONAL_LEFT:
            // cppcheck-suppress unreadVariable
            edge_string = "DIAGONAL LEFT";
            break;
        case EDGE_DETECT_LOCATION_DIAGONAL_RIGHT:
            // cppcheck-suppress unreadVariable
            edge_string = "DIAGONAL RIGHT";
            break;
        default:
            // cppcheck-suppress unreadVariable
            edge_string = "SAFE";
            break;
    }
    TRACE("Edge Detect: %s\n", edge_string);
}
