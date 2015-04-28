#pragma once

// Helping functions for calculating Dubin's paths

/*
 * A 2D vector
 * Note: float should always provide enough percision for latitude and longitude points: a 32 bit
 *       float provides about 7 decimal places, this translates to about 11 mm of percision
 */
typedef struct {
    float x;
    float y;
} Vector;

// The vector definition of a line
typedef struct {
    Vector initial;
    Vector direction;
} Line;

// A circle
typedef struct {
    Vector center;
    float radius;
} Circle;

typedef enum {
    DUBINS_PATH_C1,
    DUBINS_PATH_S,
    DUBINS_PATH_C2,
} DubinsPath;

/*
 * Whether or not a point belongs to a half plane
 * @param line line defining half plane
 * @param point point to check
 * @return 1 if point belongs to half plane, 0 otherwise
 */
char belongs_to_half_plane(Line *line, Vector *point);

/*
 * Get the unit vector from current to target point
 * @param current current point
 * @param target target point
 * @param direction vector to fill
 */
void get_direction(Vector *current, Vector *target, Vector *direction);

/*
 * Get the magnitude of a vector
 * @param vector pointer to vector
 * @return magnitude of vector
 */
float get_magnitude(Vector *vector);

/*
 * Compute internal tangent lines between two circles
 * @param current pointer to current circle
 * @param target pointer to target circle
 * @param tangents pointer to array (size >= 2) to store tangent lines
 */
void get_tangents(Circle *current, Circle *target, Line *tangents);
