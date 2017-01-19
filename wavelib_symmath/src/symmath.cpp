#include "slam/symmath/symmath.hpp"


namespace slam {

void output_jacobian(
    std::string file_path,
    std::vector<GiNaC::ex> model,
    std::vector<GiNaC::symbol> states
)
{
    std::ofstream output_file;

    // setup
    output_file.open(file_path);
    output_file << GiNaC::csrc;  // make GiNaC output C for expressions
    output_file << "MatX G";
    output_file << "(" << states.size();
    output_file << ", " << states.size();
    output_file << ");" << std::endl;
    output_file << std::endl;

    // output jacobian
    for (int i = 0; i < (int) model.size(); i++) {
        output_file << "// row " << i + 1 << std::endl;
        for (int j = 0; j < (int) states.size(); j++) {
            output_file << "G(" << i << ", " << j << ") = ";
            output_file << model[i].diff(states[j]);
            output_file << ";";
            output_file << std::endl;
        }
        output_file << std::endl;
    }

    std::cout << GiNaC::dflt;  // revert GiNaC output to default
    output_file.close();
}

} // end of slam namespace
