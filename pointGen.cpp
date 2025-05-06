#include <iostream>
#include <cmath>
#include <cstdio>

#define M_PI 3.14159265358979323846

struct Leg {
    int ch[3];
    bool state;
    float offset_x;
    float offset_y;
    float offset_z;
    float rot_z;
    float P[4][3];
    void* pwm;
};

void rotateBezierAndPrint(Leg &leg, float angle_deg) {
    float pivot_x = leg.P[0][0];
    float pivot_y = (leg.P[0][1] + leg.P[3][1]) / 2.0;
    float rad = angle_deg * M_PI / 180.0;
    float cos_t = cos(rad);
    float sin_t = sin(rad);   

    float newP[4][3];

    for (int i = 0; i < 4; i++) {
        float x = leg.P[i][0];
        float y = leg.P[i][1];
        float z = leg.P[i][2];

        float x_trans = x - pivot_x;
        float y_trans = y - pivot_y;

        float x_rot = x_trans * cos_t - y_trans * sin_t;
        float y_rot = x_trans * sin_t + y_trans * cos_t;

        newP[i][0] = x_rot + pivot_x;
        newP[i][1] = y_rot + pivot_y;
        newP[i][2] = z;
    }

    // Cetak dalam format deklarasi langsung
    printf("Leg L1 = {{%d, %d, %d}, %s, %.1f, %.1f, %.1f, 45.0, {\n",
        leg.ch[0], leg.ch[1], leg.ch[2],
        leg.state ? "true" : "false",
        leg.offset_x, leg.offset_y, leg.offset_z,
        angle_deg
    );

    for (int i = 0; i < 4; i++) {
        printf("    {%.3f, %.3f, %.3f}%s\n", newP[i][0], newP[i][1], newP[i][2], i < 3 ? "," : "");
    }

    printf("}, &pwm_ll};\n");
}

int main() {
    Leg L1 = {
        {0, 1, 2},
        true,
        -63.0, 83.5, -20.0,
        0.0,
        {
            {-160, 110, -70},
            {-160, 130, -55},
            {-160, 190, -55},
            {-160, 210, -70}
        },
        nullptr
    };

    float angle = 180; 
    rotateBezierAndPrint(L1, angle);

    return 0;
}
