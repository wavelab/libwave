#include "wave/utils/config.hpp"

int main() {
    wave::ConfigParser parser;

    double kp, ki, kd;
    double roll_limit[2];

    // tell the parser what to parse
    parser.addParam("controller.k_p", &kp);
    parser.addParam("controller.k_i", &ki);
    parser.addParam("controller.k_d", &kd);
    parser.addParam("controller.min", &roll_limit[0]);
    parser.addParam("controller.max", &roll_limit[1]);

    // parser.addParam("controller.max", &roll_limit[1]);
    //                        ^                ^
    //                        |                |
    // yaml key --------------+                |
    // variable to store config value  --------+
    //
    // Note: We support the dot notation for yaml keys

    // load the config file and assign values to variables
    if (parser.load("example_params.yaml") != 0) {
        return -1;
    }
}