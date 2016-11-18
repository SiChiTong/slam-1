#include "slam/symmath/symmath.hpp"


namespace slam {

void SymMath::outputMotionModelJacobian(
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
    for (int i = 0; i < (int) states.size(); i++) {
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

// void QuadrotorModel::generateMotionModelJacobian(void)
// {
//     GiNaC::symbol ph("ph"), th("th"), ps("ps"), p("p"), q("q"),
//                   r("r"), x("x"), y("y"), z("z"), vx("vx"),
//                   vy("vy"), vz("vz");
//     GiNaC::symbol Ix("Ix"), Iy("Iy"), Iz("Iz");
//     GiNaC::symbol ktau("ktau"), kt("kt");
//     GiNaC::symbol tauf("tauf"), taup("taup"), tauq("tauq"), taur("taur");
//     GiNaC::symbol m("m"), g("g"), dt("dt");
//     GiNaC::ex g1, g2, g3, g4, g5, g6, g7, g8, g9, g10, g11, g12;
//
//     // quadrotor motion model
//     g1 = ph + (p + q * sin(ph) * tan(th) + r * cos(ph) * tan(th)) * dt;
//     g2 = th + (q + cos(ph) - r * sin(ph)) * dt;
//     g3 = ps + ((1 / cos(th)) * (q * sin(ph) + r * cos(ph))) * dt;
//     g4 = p + (-((Iz - Iy) / Ix) * q * r - (ktau * p / Ix) + (1 / Ix) * taup) * dt;
//     g5 = q + (-((Ix - Iz) / Iy) * p * r - (ktau * q / Iy) + (1 / Iy) * tauq) * dt;
//     g6 = r + (-((Iy - Ix) / Iz) * p * q - (ktau * r / Iz) + (1 / Iz) * taur) * dt;
//     g7 = x + vx * dt;
//     g8 = y + vy * dt;
//     g9 = z + vz * dt;
//     g10 = vx + ((-kt * vx / m) + (1 / m) * (cos(ph) * sin(th) * cos(ps) + sin(ph) * sin(ps)) * tauf) * dt;
//     g11 = vx + ((-kt * vy / m) + (1 / m) * (cos(ph) * sin(th) * sin(ps) - sin(ph) * cos(ps)) * tauf) * dt;
//     g12 = vx + (-(kt * vz / m) + (1 / m) * (cos(ph) * cos(th)) * tauf - g) * dt;
//
//     std::vector<GiNaC::ex> model = {
//         g1, g2, g3, g4, g5,
//         g6, g7, g8, g9, g10,
//         g11, g12
//     };
//     std::vector<GiNaC::symbol> states = {
//         ph, th, ps,
//         p, q, r,
//         x, y, z,
//         vx, vy, vz
//     };
//     SymMath::outputMotionModelJacobian("/tmp/test.dat", model, states);
// }


} // end of slam namespace
