#include "slam/symmath/models.hpp"


namespace slam {

void quadrotor_jacobian(
    std::vector<GiNaC::ex> &model,
    std::vector<GiNaC::symbol> &states
)
{
    GiNaC::symbol ph("ph"), th("th"), ps("ps"),
                  p("p"), q("q"), r("r"),
                  x("x"), y("y"), z("z"),
                  vx("vx"), vy("vy"), vz("vz"),
                  Ix("Ix"), Iy("Iy"), Iz("Iz"),
                  ktau("ktau"), kt("kt"),
                  tauf("tauf"), taup("taup"), tauq("tauq"), taur("taur"),
                  m("m"), g("g"), dt("dt");
    GiNaC::ex g1, g2, g3, g4, g5, g6, g7, g8, g9, g10, g11, g12;

    // quadrotor motion model
    g1 = ph + (p + q * sin(ph) * tan(th) + r * cos(ph) * tan(th)) * dt;
    g2 = th + (q + cos(ph) - r * sin(ph)) * dt;
    g3 = ps + ((1 / cos(th)) * (q * sin(ph) + r * cos(ph))) * dt;
    g4 = p + (-((Iz - Iy) / Ix) * q * r - (ktau * p / Ix) + (1 / Ix) * taup) * dt;
    g5 = q + (-((Ix - Iz) / Iy) * p * r - (ktau * q / Iy) + (1 / Iy) * tauq) * dt;
    g6 = r + (-((Iy - Ix) / Iz) * p * q - (ktau * r / Iz) + (1 / Iz) * taur) * dt;
    g7 = x + vx * dt;
    g8 = y + vy * dt;
    g9 = z + vz * dt;
    g10 = vx + ((-kt * vx / m) + (1 / m) * (cos(ph) * sin(th) * cos(ps) + sin(ph) * sin(ps)) * tauf) * dt;
    g11 = vx + ((-kt * vy / m) + (1 / m) * (cos(ph) * sin(th) * sin(ps) - sin(ph) * cos(ps)) * tauf) * dt;
    g12 = vx + (-(kt * vz / m) + (1 / m) * (cos(ph) * cos(th)) * tauf - g) * dt;

    model = {
        g1, g2, g3, g4, g5,
        g6, g7, g8, g9, g10,
        g11, g12
    };
    states = {
        ph, th, ps,
        p, q, r,
        x, y, z,
        vx, vy, vz
    };
}

void bundle_adjustment_jacobian(std::string file_path)
{
    std::ofstream out;
    GiNaC::symbol fx("fx"), fy("fy"),
                  px("px"), py("py"),
                  r11("r11"), r12("r12"), r13("r13"),
                  r21("r21"), r22("r22"), r23("r23"),
                  r31("r31"), r32("r32"), r33("r33"),
                  x1("x1"), x2("x2"), x3("x3"),
                  c1("c1"), c2("c2"), c3("c3");
    GiNaC::ex u, v, w;

    // setup
    out.open(file_path);
    out << GiNaC::csrc;  // make GiNaC output C for expressions

    // u, v, w
    u = ((fx * r11 + px * r31) * (x1 - c1))
        + ((fx * r12 + px * r32) * (x2 - c2))
        + ((fx * r13 + px * r33) * (x3 - c3));

    v = ((fy * r21 + px * r31) * (x1 - c1))
        + ((fy * r22 + px * r32) * (x2 - c2))
        + ((fy * r23 + px * r33) * (x3 - c3));

    w = (r31 * (x1 - c1)) + (r32 * (x2 - c2)) + (r33 * (x3 - c3));

    // output symbols
    out << "// symbols" << std::endl;
    out << "double fx, fy;" << std::endl;
    out << "double px, py;" << std::endl;
    out << "double r11, r12, r13;" << std::endl;
    out << "double r21, r22, r23;" << std::endl;
    out << "double r31, r32, r33;" << std::endl;
    out << "double x1, x2, x3;" << std::endl;
    out << "double c1, c2, c3;" << std::endl;
    out << std::endl;

    // \partial{u} w.r.t \partial{R}
    out << "VecX du_dR(9);" << std::endl;
    out << "du_dr(0) = " << u.diff(r11) << ";" << std::endl;
    out << "du_dr(1) = " << u.diff(r12) << ";" << std::endl;
    out << "du_dr(2) = " << u.diff(r13) << ";" << std::endl;
    out << "du_dr(3) = " << u.diff(r21) << ";" << std::endl;
    out << "du_dr(4) = " << u.diff(r22) << ";" << std::endl;
    out << "du_dr(5) = " << u.diff(r23) << ";" << std::endl;
    out << "du_dr(6) = " << u.diff(r31) << ";" << std::endl;
    out << "du_dr(7) = " << u.diff(r32) << ";" << std::endl;
    out << "du_dr(8) = " << u.diff(r33) << ";" << std::endl;
    out << std::endl;

    // \partial{v} w.r.t \partial{R}
    out << "VecX dv_dR(9);" << std::endl;
    out << "dv_dr(0) = " << v.diff(r11) << ";" << std::endl;
    out << "dv_dr(1) = " << v.diff(r12) << ";" << std::endl;
    out << "dv_dr(2) = " << v.diff(r13) << ";" << std::endl;
    out << "dv_dr(3) = " << v.diff(r21) << ";" << std::endl;
    out << "dv_dr(4) = " << v.diff(r22) << ";" << std::endl;
    out << "dv_dr(5) = " << v.diff(r23) << ";" << std::endl;
    out << "dv_dr(6) = " << v.diff(r31) << ";" << std::endl;
    out << "dv_dr(7) = " << v.diff(r32) << ";" << std::endl;
    out << "dv_dr(8) = " << v.diff(r33) << ";" << std::endl;
    out << std::endl;

    // \partial{w} w.r.t \partial{R}
    out << "VecX dw_dR(9);" << std::endl;
    out << "dw_dr(0) = " << w.diff(r11) << ";" << std::endl;
    out << "dw_dr(1) = " << w.diff(r12) << ";" << std::endl;
    out << "dw_dr(2) = " << w.diff(r13) << ";" << std::endl;
    out << "dw_dr(3) = " << w.diff(r21) << ";" << std::endl;
    out << "dw_dr(4) = " << w.diff(r22) << ";" << std::endl;
    out << "dw_dr(5) = " << w.diff(r23) << ";" << std::endl;
    out << "dw_dr(6) = " << w.diff(r31) << ";" << std::endl;
    out << "dw_dr(7) = " << w.diff(r32) << ";" << std::endl;
    out << "dw_dr(8) = " << w.diff(r33) << ";" << std::endl;
    out << std::endl;

    // \partial{u} w.r.t \partial{C}
    out << "VecX du_dC(3);" << std::endl;
    out << "du_dC(0) = " << u.diff(c1) << ";" << std::endl;
    out << "du_dC(1) = " << u.diff(c2) << ";" << std::endl;
    out << "du_dC(2) = " << u.diff(c3) << ";" << std::endl;
    out << std::endl;

    // \partial{v} w.r.t \partial{C}
    out << "VecX dv_dC(3);" << std::endl;
    out << "dv_dC(0) = " << v.diff(c1) << ";" << std::endl;
    out << "dv_dC(1) = " << v.diff(c2) << ";" << std::endl;
    out << "dv_dC(2) = " << v.diff(c3) << ";" << std::endl;
    out << std::endl;

    // \partial{w} w.r.t \partial{C}
    out << "VecX dw_dC(3);" << std::endl;
    out << "dw_dC(0) = " << w.diff(c1) << ";" << std::endl;
    out << "dw_dC(1) = " << w.diff(c2) << ";" << std::endl;
    out << "dw_dC(2) = " << w.diff(c3) << ";" << std::endl;
    out << std::endl;

    // \partial{u} w.r.t \partial{C}
    out << "VecX du_dX(3);" << std::endl;
    out << "du_dX(0) = " << u.diff(x1) << ";" << std::endl;
    out << "du_dX(1) = " << u.diff(x2) << ";" << std::endl;
    out << "du_dX(2) = " << u.diff(x3) << ";" << std::endl;
    out << std::endl;

    // \partial{v} w.r.t \partial{C}
    out << "VecX dv_dX(3);" << std::endl;
    out << "dv_dX(0) = " << v.diff(x1) << ";" << std::endl;
    out << "dv_dX(1) = " << v.diff(x2) << ";" << std::endl;
    out << "dv_dX(2) = " << v.diff(x3) << ";" << std::endl;
    out << std::endl;

    // \partial{w} w.r.t \partial{C}
    out << "VecX dw_dX(3);" << std::endl;
    out << "dw_dX(0) = " << w.diff(x1) << ";" << std::endl;
    out << "dw_dX(1) = " << w.diff(x2) << ";" << std::endl;
    out << "dw_dX(2) = " << w.diff(x3) << ";" << std::endl;
    out << std::endl;
}


} // end of slam namespace
