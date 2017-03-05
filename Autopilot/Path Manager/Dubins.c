#include <math.h>
#include "Dubins.h"
#include <stdio.h>

// makes code more readable
float sq(float v) {
    return v*v;
}

// Line that defines half plane... NOT normal
char belongs_to_half_plane(Line *line, Vector *point) {
    return (-line->direction.y*(point->x - line->initial.x)
            + line->direction.x*(point->y - line->initial.y)) > 0;
}

void get_direction(Vector *current, Vector *target, Vector *direction) {
    float m = sqrt(sq(target->x - current->x) + sq(target->y - current->y));
    direction->x = (target->x - current->x)/m;
    direction->y = (target->y - current->y)/m;
}

float get_magnitude(Vector *vector) {
    return sqrt(sq(vector->x) + sq(vector->y));
}

void get_tangents(Circle *current, Circle *target, Line *tangents) {
    // first circle, centered between current and target with average radius
    Circle right = (Circle) {
        .center = (Vector) {
            .x = (target->center.x + current->center.x)/2,
            .y = (target->center.y + current->center.y)/2,
        },
        .radius = sqrt(sq(target->center.x - current->center.x)
                + sq(target->center.y - current->center.y))/2,
    };

    // second circle, same center as current, but with radius equal to sum of radius
    // of current and target circles
    float left_radius = current->radius + target->radius;

    // parameters of line of intersection of right and left circles
    // http://math.stackexchange.com/questions/256100/how-can-i-find-the-points-at-which-two-circles-intersect
    if (current->center.y - right.center.y == 0) {
        float x = (sq(current->center.x) - sq(right.center.x) - sq(left_radius)
                + sq(right.radius))/(2*current->center.x - 2*right.center.x);
        float y = sqrt(sq(left_radius) + 2*current->center.x*x - sq(current->center.x) - sq(x)) + right.center.y;

        tangents[0].initial = (Vector) {
            .x = current->center.x + current->radius*(x - current->center.x)/(left_radius),
            .y = current->center.y + current->radius*(y - current->center.y)/(left_radius),
        };
        tangents[0].direction = (Vector) {
            .x = target->center.x - x,
            .y = target->center.y - y,
        };

        y = -y;
        tangents[1].initial = (Vector) {
            .x = current->center.x + current->radius*(x - current->center.x)/(left_radius),
            .y = current->center.y + current->radius*(y - current->center.y)/(left_radius),
        };
        tangents[1].direction = (Vector) {
            .x = target->center.x - x,
            .y = target->center.y - y,
        };
    } else {
        float m = -(current->center.x - right.center.x)/(current->center.y - right.center.y);
        float n = (sq(current->center.x) - sq(right.center.x) + sq(current->center.y)
                - sq(right.center.y) - sq(left_radius)
                + sq(right.radius))/(2*current->center.y - 2*right.center.y);

        // quadratic to solve for points of intersection of line and left circle
        // http://math.stackexchange.com/questions/228841/how-do-i-calculate-the-intersections-of-a-straight-line-and-a-circle
        float a = m*m + 1;
        float b = 2*(m*n - m*current->center.y - current->center.x);
        float c = sq(current->center.x) + sq(current->center.y) - sq(left_radius)
                - 2*n*current->center.y + n*n;

        // find first tangent line
        float x = (-b + sqrt(b*b - 4*a*c))/(2*a);
        float y = m*x + n;
        tangents[0].initial = (Vector) {
            .x = current->center.x + current->radius*(x - current->center.x)/(left_radius),
            .y = current->center.y + current->radius*(y - current->center.y)/(left_radius),
        };
        tangents[0].direction = (Vector) {
            .x = target->center.x - x,
            .y = target->center.y - y,
        };

        x = (-b - sqrt(b*b - 4*a*c))/(2*a);
        y = m*x + n;
        tangents[1].initial = (Vector) {
            .x = current->center.x + current->radius*(x - current->center.x)/(left_radius),
            .y = current->center.y + current->radius*(y - current->center.y)/(left_radius),
        };
        tangents[1].direction = (Vector) {
            .x = target->center.x - x,
            .y = target->center.y - y,
        };
    }
}
