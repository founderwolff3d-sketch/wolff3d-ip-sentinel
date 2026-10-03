cat << 'EOF' > clevis_parametric_generator.cpp
/**
 * ============================================================================
 * DEFENSIVE PUBLICATION & PRIOR ART DISCLOSURE
 * ============================================================================
 * TITLE: Parametric Aerospace Clevis Lug with Conformal Diamond-D TPMS Infill
 * 
 * LEGAL STATUS:
 * This software and its documented geometric specifications are placed in the
 * public domain / published defensively to establish verifiable Prior Art under
 * 35 U.S.C. § 102(a)(1), EPC Article 54(2), and equivalent worldwide patent laws.
 * 
 * CORE TECHNICAL NOVELTY DISCLOSED HEREIN:
 * 1. Monocoque Clevis bracket with 38.0 x 38.0 mm base and 25.0 x 25.0 mm M5 pattern.
 * 2. Schoen's Diamond-D (TPMS) infill with stress-tensor adaptive porosity gradation.
 * 3. Dual neutral-axis depowdering ports (Dia 4.0 mm) with Dia 6.5 mm plug counterbores.
 * 4. Multi-body zero-gap CAD embedment architecture for 2D slicer-level metallurgical fusion.
 * 5. Structural stress-relief fillets (R3.0 mm) and pin bore lead-in chamfers (0.8 mm x 45 deg).
 *
 * PROPRIETARY PROCESS EXCLUSION:
 * This disclosure provides purely mathematical and geometric algorithms. All proprietary
 * additive manufacturing execution parameters (laser toolpaths, scan strategies, machine
 * control firmware, and post-machining feeds) are strictly excluded and reserved.
 *
 * LICENSE: Apache License, Version 2.0.
 * ============================================================================
 */

#include <iostream>
#include <vector>
#include <cmath>
#include <chrono>
#include <algorithm>
#include <omp.h>

#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <Geom_Curve.hxx>
#include <BRep_Tool.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRep_Builder.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Solid.hxx>
#include <TopoDS_Compound.hxx>
#include <GProp_GProps.hxx>
#include <BRepGProp.hxx>
#include <STEPControl_Writer.hxx>
#include <Interface_Static.hxx>

struct ClevisDesignParameters {
    // External Envelope
    double base_x = 38.0;          // Base flange width (mm)
    double base_y = 38.0;          // Base flange length (mm)
    double base_z = 8.0;           // Base plate thickness (mm)
    double boss_zone_z = 16.0;     // Flange top transition height (mm)
    double waist_top_z = 38.0;     // Tapered waist top height (mm)
    double ear_width_x = 24.0;     // Lug ear total width (mm)
    double ear_center_x = 19.0;    // Symmetry center X (mm)
    double ear_center_y = 19.0;    // Symmetry center Y (mm)
    double pin_center_z = 48.0;    // Pin bore axis elevation (mm)
    double ear_radius = 12.0;      // Lug ear cylindrical arch radius (mm)
    double pin_radius = 5.0;       // Pin bore radius (Dia 10.0 mm, H7 fit)
    double hinge_gap_y0 = 11.0;    // Clevis slot start Y (mm)
    double hinge_gap_y1 = 27.0;    // Clevis slot end Y (16.0 mm clear opening)
    double skin_thickness = 1.8;   // Outer structural skin thickness (mm)

    // Standardized Mechanical Interfaces
    double tap_drill_radius = 2.10;// M5-6H pre-drilled tap radius (Dia 4.20 mm)
    double flange_fillet_r = 3.0;  // Stress-relief transition fillet radius (mm)
    double pocket_fillet_r = 3.0;  // Bottom pocket corner fillet radius (mm)
    double depowder_z = 22.0;      // Neutral axis depowdering elevation (mm)
    double depowder_dia = 4.0;     // Through depowdering port diameter (mm)
    double plug_csk_dia = 6.5;     // Port seal counterbore diameter (mm)
    double plug_csk_depth = 1.0;   // Port seal counterbore depth (mm)
};

const ClevisDesignParameters P;

// Analytical principal stress field mapping
inline double EvaluatePrincipalStressTensor(double x, double y, double z) {
    double dx = x - P.ear_center_x;
    double dy = y - P.ear_center_y;
    double dz = z - P.pin_center_z;
    double dist_pin = std::sqrt(dx * dx + dy * dy + dz * dz);

    double norm_z = std::clamp((z - P.base_z) / (P.waist_top_z - P.base_z), 0.0, 1.0);
    double sigma_tensile = 60.0 * std::exp(-dist_pin / 15.0) + 35.0 * (1.0 - norm_z);
    double tau_xz = 30.0 * std::sin(norm_z * 3.14159265);
    double dist_flange = std::max(0.0, z - P.base_z);
    double sigma_flange = 65.0 * std::exp(-dist_flange / 5.0) * (std::abs(x - P.ear_center_x) / 19.0);

    return std::sqrt(sigma_tensile * sigma_tensile + sigma_flange * sigma_flange + 3.0 * tau_xz * tau_xz);
}

// Continuous implicit field for Schoen's Diamond-D TPMS
inline double EvaluateImplicitInfillField(double x, double y, double z) {
    // Depowdering port air corridor clearance
    double dx_dp = x - P.ear_center_x;
    double dz_dp = z - P.depowder_z;
    if (std::sqrt(dx_dp * dx_dp + dz_dp * dz_dp) <= 2.6) return -2.0;

    // Hinge opening slot exclusion
    if (z >= 37.0 && y > (P.hinge_gap_y0 - 0.5) && y < (P.hinge_gap_y1 + 0.5)) return -2.0;

    // Solid structural sleeve protection for 4x M5 bolts (25x25 mm matrix)
    static const double bolts[4][2] = { {6.5, 6.5}, {31.5, 6.5}, {6.5, 31.5}, {31.5, 31.5} };
    if (z <= 18.0) {
        for (int i = 0; i < 4; ++i) {
            double bdx = x - bolts[i][0];
            double bdy = y - bolts[i][1];
            if (std::sqrt(bdx * bdx + bdy * bdy) <= 5.2) return -2.0;
        }
    }

    const double pi = 3.14159265358979323846;
    const double scale = 0.12;

    double sx = x * scale * 2.0 * pi;
    double sy = y * scale * 2.0 * pi;
    double sz = z * scale * 2.0 * pi;

    double d_val = std::sin(sx) * std::sin(sy) * std::sin(sz) +
                   std::sin(sx) * std::cos(sy) * std::cos(sz) +
                   std::cos(sx) * std::sin(sy) * std::cos(sz) +
                   std::cos(sx) * std::cos(sy) * std::sin(sz);

    double vm = EvaluatePrincipalStressTensor(x, y, z);
    double s_ratio = std::clamp(vm / 80.0, 0.0, 1.0);
    double s_hermite = 3.0 * s_ratio * s_ratio - 2.0 * s_ratio * s_ratio * s_ratio;

    const double t_light = 0.52;
    const double t_dense = 0.08;
    double t_adaptive = t_light - (t_light - t_dense) * s_hermite;

    return d_val - t_adaptive;
}

inline gp_Pnt InterpolateEdge(const gp_Pnt& p1, const gp_Pnt& p2, double v1, double v2) {
    double t = (std::abs(v2 - v1) > 1e-9) ? (-v1 / (v2 - v1)) : 0.5;
    t = std::clamp(t, 0.0, 1.0);
    return gp_Pnt(p1.X() + t * (p2.X() - p1.X()),
                  p1.Y() + t * (p2.Y() - p1.Y()),
                  p1.Z() + t * (p2.Z() - p1.Z()));
}

void PolygonizeTetrahedron(const gp_Pnt p[4], const double v[4], std::vector<TopoDS_Face>& faces) {
    int mask = 0;
    for (int i = 0; i < 4; ++i) if (v[i] > 0.0) mask |= (1 << i);
    if (mask == 0 || mask == 15) return;

    static const int edges[6][2] = { {0, 1}, {1, 2}, {2, 0}, {0, 3}, {1, 3}, {2, 3} };
    static const int tet_tri_table[16][7] = {
        {-1},
        {0, 2, 3, -1}, {0, 1, 4, -1}, {1, 4, 3,  1, 3, 2, -1},
        {1, 2, 5, -1}, {0, 1, 5,  0, 5, 3, -1}, {0, 4, 5,  0, 5, 2, -1},
        {3, 5, 4, -1}, {3, 4, 5, -1}, {0, 2, 5,  0, 5, 4, -1},
        {0, 3, 5,  0, 5, 1, -1}, {1, 5, 2, -1}, {2, 3, 4,  2, 4, 1, -1},
        {0, 4, 1, -1}, {0, 3, 2, -1}, {-1}
    };

    const int* tri = tet_tri_table[mask];
    for (int i = 0; tri[i] != -1; i += 3) {
        gp_Pnt pA = InterpolateEdge(p[edges[tri[i]][0]],   p[edges[tri[i]][1]],   v[edges[tri[i]][0]],   v[edges[tri[i]][1]]);
        gp_Pnt pB = InterpolateEdge(p[edges[tri[i+1]][0]], p[edges[tri[i+1]][1]], v[edges[tri[i+1]][0]], v[edges[tri[i+1]][1]]);
        gp_Pnt pC = InterpolateEdge(p[edges[tri[i+2]][0]], p[edges[tri[i+2]][1]], v[edges[tri[i+2]][0]], v[edges[tri[i+2]][1]]);

        BRepBuilderAPI_MakePolygon poly(pA, pB, pC, Standard_True);
        if (poly.IsDone()) {
            BRepBuilderAPI_MakeFace mf(poly.Wire());
            if (mf.IsDone()) faces.push_back(mf.Face());
        }
    }
}

TopoDS_Shape GenerateLatticeSolid(double x0, double y0, double z0,
                                  double lx, double ly, double lz,
                                  int res_x, int res_y, int res_z) {
    const double hx = lx / res_x;
    const double hy = ly / res_y;
    const double hz = lz / res_z;

    std::vector<double> field((res_x + 1) * (res_y + 1) * (res_z + 1), 0.0);

    #pragma omp parallel for collapse(2) schedule(static)
    for (int i = 0; i <= res_x; ++i) {
        for (int j = 0; j <= res_y; ++j) {
            for (int k = 0; k <= res_z; ++k) {
                if (i == 0 || i == res_x || j == 0 || j == res_y || k == 0 || k == res_z) {
                    field[(i * (res_y + 1) + j) * (res_z + 1) + k] = -2.0;
                } else {
                    double gx = x0 + i * hx;
                    double gy = y0 + j * hy;
                    double gz = z0 + k * hz;
                    field[(i * (res_y + 1) + j) * (res_z + 1) + k] = EvaluateImplicitInfillField(gx, gy, gz);
                }
            }
        }
    }

    int max_threads = omp_get_max_threads();
    std::vector<std::vector<TopoDS_Face>> thread_faces(max_threads);

    #pragma omp parallel
    {
        int tid = omp_get_thread_num();
        #pragma omp for collapse(2) schedule(dynamic)
        for (int i = 0; i < res_x; ++i) {
            for (int j = 0; j < res_y; ++j) {
                for (int k = 0; k < res_z; ++k) {
                    gp_Pnt p[8] = {
                        gp_Pnt(x0 + i*hx,     y0 + j*hy,     z0 + k*hz),
                        gp_Pnt(x0 + (i+1)*hx, y0 + j*hy,     z0 + k*hz),
                        gp_Pnt(x0 + (i+1)*hx, y0 + (j+1)*hy, z0 + k*hz),
                        gp_Pnt(x0 + i*hx,     y0 + (j+1)*hy, z0 + k*hz),
                        gp_Pnt(x0 + i*hx,     y0 + j*hy,     z0 + (k+1)*hz),
                        gp_Pnt(x0 + (i+1)*hx, y0 + j*hy,     z0 + (k+1)*hz),
                        gp_Pnt(x0 + (i+1)*hx, y0 + (j+1)*hy, z0 + (k+1)*hz),
                        gp_Pnt(x0 + i*hx,     y0 + (j+1)*hy, z0 + (k+1)*hz)
                    };

                    auto get_v = [&](int ci, int cj, int ck) {
                        return field[(ci * (res_y + 1) + cj) * (res_z + 1) + ck];
                    };

                    double v[8] = {
                        get_v(i, j, k),     get_v(i+1, j, k),     get_v(i+1, j+1, k),     get_v(i, j+1, k),
                        get_v(i, j, k+1),   get_v(i+1, j, k+1),   get_v(i+1, j+1, k+1),   get_v(i, j+1, k+1)
                    };

                    int tet_indices[6][4] = {
                        {0, 1, 2, 6}, {0, 2, 3, 6}, {0, 3, 7, 6},
                        {0, 7, 4, 6}, {0, 4, 5, 6}, {0, 5, 1, 6}
                    };

                    for (int t = 0; t < 6; ++t) {
                        gp_Pnt tp[4] = { p[tet_indices[t][0]], p[tet_indices[t][1]], p[tet_indices[t][2]], p[tet_indices[t][3]] };
                        double tv[4] = { v[tet_indices[t][0]], v[tet_indices[t][1]], v[tet_indices[t][2]], v[tet_indices[t][3]] };
                        PolygonizeTetrahedron(tp, tv, thread_faces[tid]);
                    }
                }
            }
        }
    }

    std::vector<TopoDS_Face> raw_faces;
    for (int t = 0; t < max_threads; ++t) {
        raw_faces.insert(raw_faces.end(), thread_faces[t].begin(), thread_faces[t].end());
    }

    BRepBuilderAPI_Sewing sewer(0.04);
    for (const auto& face : raw_faces) sewer.Add(face);
    sewer.Perform();

    TopoDS_Shape sewed = sewer.SewedShape();
    TopoDS_Solid retained_solid;

    for (TopExp_Explorer exp(sewed, TopAbs_SHELL); exp.More(); exp.Next()) {
        TopoDS_Shell shell = TopoDS::Shell(exp.Current());
        int facet_count = 0;
        for (TopExp_Explorer fexp(shell, TopAbs_FACE); fexp.More(); fexp.Next()) facet_count++;

        if (facet_count > 500) {
            BRepBuilderAPI_MakeSolid ms(shell);
            if (ms.IsDone()) {
                retained_solid = ms.Solid();
                break;
            }
        }
    }
    return retained_solid;
}

TopoDS_Shape BuildHousingSolid() {
    // Base zone: 38.0 x 38.0 x 16.0 mm
    TopoDS_Shape base = BRepPrimAPI_MakeBox(gp_Pnt(0.0, 0.0, 0.0), P.base_x, P.base_y, P.boss_zone_z).Shape();

    // Tapered waist section (Z: 16.0 to 38.0 mm)
    TopoDS_Shape waist = BRepPrimAPI_MakeBox(gp_Pnt(0.0, 0.0, P.boss_zone_z),
                                             P.base_x, P.base_y,
                                             P.waist_top_z - P.boss_zone_z).Shape();

    gp_Pnt pL1(0.0, -2.0, P.boss_zone_z), pL2(7.0, -2.0, P.waist_top_z), pL3(0.0, -2.0, P.waist_top_z);
    BRepBuilderAPI_MakePolygon polyL(pL1, pL2, pL3, Standard_True);
    TopoDS_Shape cutL = BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(polyL.Wire()).Face(), gp_Vec(0.0, P.base_y + 4.0, 0.0)).Shape();
    waist = BRepAlgoAPI_Cut(waist, cutL).Shape();

    gp_Pnt pR1(38.0, -2.0, P.boss_zone_z), pR2(31.0, -2.0, P.waist_top_z), pR3(38.0, -2.0, P.waist_top_z);
    BRepBuilderAPI_MakePolygon polyR(pR1, pR2, pR3, Standard_True);
    TopoDS_Shape cutR = BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(polyR.Wire()).Face(), gp_Vec(0.0, P.base_y + 4.0, 0.0)).Shape();
    waist = BRepAlgoAPI_Cut(waist, cutR).Shape();

    // Dual cylindrical-arch ears
    TopoDS_Shape earL_box = BRepPrimAPI_MakeBox(gp_Pnt(7.0, 0.0, P.waist_top_z), P.ear_width_x, P.hinge_gap_y0, P.pin_center_z - P.waist_top_z).Shape();
    gp_Ax2 earL_cyl_ax(gp_Pnt(P.ear_center_x, 0.0, P.pin_center_z), gp_Dir(0, 1, 0));
    TopoDS_Shape earL = BRepAlgoAPI_Fuse(earL_box, BRepPrimAPI_MakeCylinder(earL_cyl_ax, P.ear_radius, P.hinge_gap_y0).Shape()).Shape();

    TopoDS_Shape earR_box = BRepPrimAPI_MakeBox(gp_Pnt(7.0, P.hinge_gap_y1, P.waist_top_z), P.ear_width_x, P.base_y - P.hinge_gap_y1, P.pin_center_z - P.waist_top_z).Shape();
    gp_Ax2 earR_cyl_ax(gp_Pnt(P.ear_center_x, P.hinge_gap_y1, P.pin_center_z), gp_Dir(0, 1, 0));
    TopoDS_Shape earR = BRepAlgoAPI_Fuse(earR_box, BRepPrimAPI_MakeCylinder(earR_cyl_ax, P.ear_radius, P.base_y - P.hinge_gap_y1).Shape()).Shape();

    TopoDS_Shape housing = BRepAlgoAPI_Fuse(base, waist).Shape();
    housing = BRepAlgoAPI_Fuse(housing, earL).Shape();
    housing = BRepAlgoAPI_Fuse(housing, earR).Shape();

    // R3.0 mm transition fillet at flange-waist junction
    BRepFilletAPI_MakeFillet fillet(housing);
    for (TopExp_Explorer exp(housing, TopAbs_EDGE); exp.More(); exp.Next()) {
        TopoDS_Edge edge = TopoDS::Edge(exp.Current());
        GProp_GProps props;
        BRepGProp::LinearProperties(edge, props);
        gp_Pnt c = props.CentreOfMass();
        if (std::abs(c.Z() - P.boss_zone_z) < 1.0 && (c.X() < 8.0 || c.X() > 30.0)) {
            fillet.Add(P.flange_fillet_r, edge);
        }
    }
    fillet.Build();
    if (fillet.IsDone()) housing = fillet.Shape();

    // Pin bore (Dia 10.00 mm H7) + 0.8 mm x 45 deg lead-in chamfers
    gp_Ax2 pin_ax(gp_Pnt(P.ear_center_x, -2.0, P.pin_center_z), gp_Dir(0, 1, 0));
    housing = BRepAlgoAPI_Cut(housing, BRepPrimAPI_MakeCylinder(pin_ax, P.pin_radius, P.base_y + 4.0).Shape()).Shape();

    gp_Ax2 ch_ax1(gp_Pnt(P.ear_center_x, -0.1, P.pin_center_z), gp_Dir(0, 1, 0));
    housing = BRepAlgoAPI_Cut(housing, BRepPrimAPI_MakeCone(ch_ax1, 5.8, 5.0, 0.8).Shape()).Shape();
    gp_Ax2 ch_ax2(gp_Pnt(P.ear_center_x, 38.1, P.pin_center_z), gp_Dir(0, -1, 0));
    housing = BRepAlgoAPI_Cut(housing, BRepPrimAPI_MakeCone(ch_ax2, 5.8, 5.0, 0.8).Shape()).Shape();

    // Internal cavity (1.8 mm skin thickness)
    TopoDS_Shape cavity = BRepPrimAPI_MakeBox(gp_Pnt(7.0 + P.skin_thickness, P.skin_thickness, P.base_z),
                                              P.ear_width_x - 2.0 * P.skin_thickness,
                                              P.base_y - 2.0 * P.skin_thickness,
                                              P.waist_top_z - P.skin_thickness - P.base_z).Shape();
    housing = BRepAlgoAPI_Cut(housing, cavity).Shape();

    // Neutral-axis depowder ports (Dia 4.0 mm) + seal counterbores (Dia 6.5 mm x 1.0 mm)
    gp_Ax2 depowder_ax(gp_Pnt(P.ear_center_x, -2.0, P.depowder_z), gp_Dir(0, 1, 0));
    housing = BRepAlgoAPI_Cut(housing, BRepPrimAPI_MakeCylinder(depowder_ax, P.depowder_dia / 2.0, P.base_y + 4.0).Shape()).Shape();

    gp_Ax2 plug_ax1(gp_Pnt(P.ear_center_x, -0.1, P.depowder_z), gp_Dir(0, 1, 0));
    housing = BRepAlgoAPI_Cut(housing, BRepPrimAPI_MakeCylinder(plug_ax1, P.plug_csk_dia / 2.0, P.plug_csk_depth + 0.1).Shape()).Shape();
    gp_Ax2 plug_ax2(gp_Pnt(P.ear_center_x, P.base_y + 0.1, P.depowder_z), gp_Dir(0, -1, 0));
    housing = BRepAlgoAPI_Cut(housing, BRepPrimAPI_MakeCylinder(plug_ax2, P.plug_csk_dia / 2.0, P.plug_csk_depth + 0.1).Shape()).Shape();

    // 4x M5 pre-drilled holes (Dia 4.20 mm, 25.0 x 25.0 mm matrix)
    double bolt_coords[4][2] = { {6.5, 6.5}, {31.5, 6.5}, {6.5, 31.5}, {31.5, 31.5} };
    for (int i = 0; i < 4; ++i) {
        gp_Ax2 bolt_ax(gp_Pnt(bolt_coords[i][0], bolt_coords[i][1], -1.0), gp_Dir(0, 0, 1));
        housing = BRepAlgoAPI_Cut(housing, BRepPrimAPI_MakeCylinder(bolt_ax, P.tap_drill_radius, 20.0).Shape()).Shape();
    }

    // Base center pocket (16.0 x 16.0 x 3.0 mm) with 4x R3.0 mm vertical corner fillets
    TopoDS_Shape pocket = BRepPrimAPI_MakeBox(gp_Pnt(11.0, 11.0, -1.0), 16.0, 16.0, 4.0).Shape();
    BRepFilletAPI_MakeFillet pocket_fillet(pocket);
    for (TopExp_Explorer exp(pocket, TopAbs_EDGE); exp.More(); exp.Next()) {
        TopoDS_Edge e = TopoDS::Edge(exp.Current());
        Standard_Real f, l;
        Handle(Geom_Curve) c = BRep_Tool::Curve(e, f, l);
        if (!c.IsNull()) {
            gp_Pnt p1 = c->Value(f);
            gp_Pnt p2 = c->Value(l);
            if (std::abs(p1.X() - p2.X()) < 1e-3 && std::abs(p1.Y() - p2.Y()) < 1e-3 && std::abs(p1.Z() - p2.Z()) > 1.0) {
                pocket_fillet.Add(P.pocket_fillet_r, e);
            }
        }
    }
    pocket_fillet.Build();
    if (pocket_fillet.IsDone()) pocket = pocket_fillet.Shape();
    housing = BRepAlgoAPI_Cut(housing, pocket).Shape();

    return housing;
}

int main() {
    std::cout << "[INFO] Compiling Defensive Publication Reference: Aerospace Clevis Lug..." << std::endl;
    auto t0 = std::chrono::high_resolution_clock::now();

    // 1. Analytical B-Rep Housing
    TopoDS_Shape skin = BuildHousingSolid();

    // 2. Conformal Diamond-D TPMS Infill (Zero-gap embedment)
    TopoDS_Shape lattice = GenerateLatticeSolid(8.0, 1.0, 5.6, 22.0, 36.0, 32.4, 16, 22, 20);

    // 3. Multi-body compound assembly (ISO 10303 compliant)
    TopoDS_Compound part;
    BRep_Builder builder;
    builder.MakeCompound(part);
    builder.Add(part, skin);
    builder.Add(part, lattice);

    // 4. Neutral AP214 STEP Export
    STEPControl_Writer writer;
    Interface_Static::SetIVal("write.step.assembly", 0);
    Interface_Static::SetCVal("write.step.schema", "AP214");
    Interface_Static::SetCVal("write.step.product.name", "AEROSPACE_CLEVIS_STANDARD_25X25");
    writer.Transfer(part, STEPControl_AsIs);

    const std::string outfile = "clevis_bracket_standard_25x25.step";
    if (writer.Write(outfile.c_str()) == IFSelect_RetDone) {
        auto t1 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> dt = t1 - t0;
        std::cout << "[SUCCESS] Generated: " << outfile << " in " << dt.count() << " seconds." << std::endl;
        return 0;
    } else {
        std::cerr << "[ERROR] STEP export failed." << std::endl;
        return 1;
    }
}
EOF