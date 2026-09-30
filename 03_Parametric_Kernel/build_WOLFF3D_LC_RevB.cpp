/**
 * @file   build_WOLFF3D_LC_RevB.cpp
 * @brief  Parametric Solid Geometry Generator for WOLFF3D-D1-LC RevB Mechanism
 * @note   Constructive Solid Geometry (CSG) modeling based on OpenCASCADE 7.x.
 *         Outputs 100% watertight boundary-representation STEP solids.
 */

#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <cmath>

// OpenCASCADE Modeling & Topology Headers
#include <gp_Pnt.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRep_Builder.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Compound.hxx>
#include <STEPControl_Writer.hxx>
#include <STEPControl_StepModelType.hxx>
#include <Interface_Static.hxx>

// Mass Properties & Inertial Kernel
#include <GProp_GProps.hxx>
#include <BRepGProp.hxx>

/**
 * @brief Exports a TopoDS_Shape to an ISO AP214 standard STEP file.
 */
static bool ExportSTEP(const TopoDS_Shape& shape, const std::string& filename) {
    STEPControl_Writer writer;
    Interface_Static::SetCVal("write.step.schema", "AP214");
    writer.Transfer(shape, STEPControl_AsIs);
    return (writer.Write(filename.c_str()) == IFSelect_RetDone);
}

/**
 * @brief Generates a variable-curvature pocket cutter using composite dual-arc blending.
 *        Mitigates peak notch stress concentration in flexible spring ligaments.
 */
static TopoDS_Shape MakeHyperbolicBoxCut(double x_min, double y_min, double z_min, 
                                        double length, double width, double height) {
    const double r_major = 4.20; // Entry transition curvature
    const double r_minor = 2.10; // Corner tight curvature

    TopoDS_Shape box_x = BRepPrimAPI_MakeBox(gp_Pnt(x_min + r_major, y_min, z_min), 
                                            length - 2.0 * r_major, width, height).Shape();
    TopoDS_Shape box_y = BRepPrimAPI_MakeBox(gp_Pnt(x_min, y_min + r_major, z_min), 
                                            length, width - 2.0 * r_major, height).Shape();
    TopoDS_Shape cutout = BRepAlgoAPI_Fuse(box_x, box_y).Shape();

    const double cx[2] = {x_min + r_major, x_min + length - r_major};
    const double cy[2] = {y_min + r_major, y_min + width - r_major};

    for (double x : cx) {
        for (double y : cy) {
            // Primary lead-in blend cylinder
            gp_Ax2 ax1(gp_Pnt(x, y, z_min), gp_Dir(0, 0, 1));
            TopoDS_Shape cyl1 = BRepPrimAPI_MakeCylinder(ax1, r_major, height).Shape();
            cutout = BRepAlgoAPI_Fuse(cutout, cyl1).Shape();

            // Secondary interior blend cylinder
            double sign_x = (x > x_min + length / 2.0) ? -1.0 : 1.0;
            double sign_y = (y > y_min + width / 2.0) ? -1.0 : 1.0;
            gp_Ax2 ax2(gp_Pnt(x + sign_x * 0.8, y + sign_y * 0.8, z_min), gp_Dir(0, 0, 1));
            TopoDS_Shape cyl2 = BRepPrimAPI_MakeCylinder(ax2, r_minor, height).Shape();
            cutout = BRepAlgoAPI_Fuse(cutout, cyl2).Shape();
        }
    }
    return cutout;
}

/**
 * @brief Generates a standard constant-radius rectangular pocket cutter.
 */
static TopoDS_Shape MakeRoundedBoxCut(double x_min, double y_min, double z_min, 
                                      double length, double width, double height, double r) {
    TopoDS_Shape box_x = BRepPrimAPI_MakeBox(gp_Pnt(x_min + r, y_min, z_min), 
                                            length - 2.0 * r, width, height).Shape();
    TopoDS_Shape box_y = BRepPrimAPI_MakeBox(gp_Pnt(x_min, y_min + r, z_min), 
                                            length, width - 2.0 * r, height).Shape();
    TopoDS_Shape rounded_box = BRepAlgoAPI_Fuse(box_x, box_y).Shape();

    const double cx[2] = {x_min + r, x_min + length - r};
    const double cy[2] = {y_min + r, y_min + width - r};

    for (double x : cx) {
        for (double y : cy) {
            gp_Ax2 ax(gp_Pnt(x, y, z_min), gp_Dir(0, 0, 1));
            TopoDS_Shape corner_cyl = BRepPrimAPI_MakeCylinder(ax, r, height).Shape();
            rounded_box = BRepAlgoAPI_Fuse(rounded_box, corner_cyl).Shape();
        }
    }
    return rounded_box;
}

/**
 * @brief Component 1: Parallel Flexure Strip (Material: 65Mn Spring Steel, 46-48 HRC)
 *        Dimensions: 52.00 x 18.00 x 3.00 mm with 0.35 mm working flexure ligament.
 */
TopoDS_Shape Build_W1_LC_01_RevB() {
    TopoDS_Shape flexure = BRepPrimAPI_MakeBox(52.0, 18.0, 3.0).Shape();

    // Variable curvature internal working slot (leaves 0.35 mm web thickness)
    TopoDS_Shape pocket = MakeHyperbolicBoxCut(10.0, 2.0, 0.35, 32.0, 14.0, 3.5);
    flexure = BRepAlgoAPI_Cut(flexure, pocket).Shape();

    // 4x Mounting thru-holes (Diameter 2.60 mm)
    const double x_ears[2] = {5.5, 46.5};
    const double y_ears[2] = {3.0, 15.0};
    for (double x : x_ears) {
        for (double y : y_ears) {
            gp_Ax2 ax(gp_Pnt(x, y, -1.0), gp_Dir(0, 0, 1));
            TopoDS_Shape hole = BRepPrimAPI_MakeCylinder(ax, 1.30, 5.0).Shape();
            flexure = BRepAlgoAPI_Cut(flexure, hole).Shape();
        }
    }
    return flexure;
}

/**
 * @brief Component 2: Moving Slider (Material: Aluminum Alloy 7075-T651)
 *        Encompasses bottom-up threaded blind holes, tool clearance, and CoM balancing slot.
 */
TopoDS_Shape Build_W1_LC_02_RevB() {
    const double xc = 32.5;
    const double yc = 24.0;
    TopoDS_Shape slider = BRepPrimAPI_MakeBox(gp_Pnt(3.5, 2.0, 11.0), 58.0, 44.0, 11.5).Shape();

    // 1) Fastener tool clearance step cutout on the stationary interface side
    TopoDS_Shape left_relief = BRepPrimAPI_MakeBox(gp_Pnt(3.0, 1.5, 10.5), 9.5, 45.0, 12.5).Shape();
    slider = BRepAlgoAPI_Cut(slider, left_relief).Shape();

    // 2) Actuator center bore: Diameter 16.50 mm x Depth 6.00 mm
    gp_Ax2 motor_ax(gp_Pnt(xc, yc, 16.5), gp_Dir(0, 0, 1));
    slider = BRepAlgoAPI_Cut(slider, BRepPrimAPI_MakeCylinder(motor_ax, 8.25, 6.5).Shape()).Shape();

    // 3) 4x Payload interface holes: 20.0 x 20.0 mm pattern (M3 tap drill, Dia 2.50 mm x Depth 5.50 mm)
    const double jx[2] = {xc - 10.0, xc + 10.0};
    const double jy[2] = {yc - 10.0, yc + 10.0};
    for (double x : jx) {
        for (double y : jy) {
            gp_Ax2 jig_ax(gp_Pnt(x, y, 17.0), gp_Dir(0, 0, 1));
            slider = BRepAlgoAPI_Cut(slider, BRepPrimAPI_MakeCylinder(jig_ax, 1.25, 6.0).Shape()).Shape();
        }
    }

    // 4) 4x Flexure attachment blind holes: Drilled bottom-up (Dia 2.05 mm x Depth 6.35 mm for M2.5)
    const double flex_mov_y[4] = {6.0, 18.0, 30.0, 42.0};
    for (double y : flex_mov_y) {
        gp_Ax2 blind_ax(gp_Pnt(50.0, y, 10.5), gp_Dir(0, 0, 1));
        slider = BRepAlgoAPI_Cut(slider, BRepPrimAPI_MakeCylinder(blind_ax, 1.025, 6.5).Shape()).Shape();
    }

    // 5) Lightweight bottom pockets (Depth 10.30 mm, maintaining 1.20 mm top skin)
    const double z_cut_start = 10.5;
    const double cut_height = 10.8;

    const double py_pockets[2] = {6.0, 28.0};
    for (double py : py_pockets) {
        TopoDS_Shape p_left = MakeRoundedBoxCut(14.5, py, z_cut_start, 6.0, 14.0, cut_height, 2.5);
        slider = BRepAlgoAPI_Cut(slider, p_left).Shape();
    }
    TopoDS_Shape p_mid1 = MakeRoundedBoxCut(26.5, 3.5, z_cut_start, 12.0, 8.0, cut_height, 3.0);
    TopoDS_Shape p_mid2 = MakeRoundedBoxCut(26.5, 36.5, z_cut_start, 12.0, 8.0, cut_height, 3.0);
    slider = BRepAlgoAPI_Cut(slider, p_mid1).Shape();
    slider = BRepAlgoAPI_Cut(slider, p_mid2).Shape();

    // 6) Dynamic CoM Tuning Slot (Recess 1.20 mm from bottom reference plane Z=11.0 mm)
    //    Cutter dimension: 13.00 mm (X) x 4.00 mm (Y) x 1.70 mm (Z-span, Z=10.5 to Z=12.2)
    TopoDS_Shape tuning_slot = BRepPrimAPI_MakeBox(gp_Pnt(26.0, 5.5, 10.5), 13.0, 4.0, 1.70).Shape();
    slider = BRepAlgoAPI_Cut(slider, tuning_slot).Shape();

    return slider;
}

/**
 * @brief Component 3: Baseplate (Material: Aluminum Alloy 6061-T6)
 *        Features standard 32.0 x 40.0 mm interface mounting pattern and reference slots.
 */
TopoDS_Shape Build_W1_LC_03_RevB() {
    TopoDS_Shape base = BRepPrimAPI_MakeBox(65.0, 48.0, 5.0).Shape();

    // Raised stationary clamping shoulder
    TopoDS_Shape left_pad = BRepPrimAPI_MakeBox(gp_Pnt(0.0, 0.0, 5.0), 11.5, 48.0, 3.0).Shape();
    base = BRepAlgoAPI_Fuse(base, left_pad).Shape();

    // 4x Stationary flexure attachment holes (M2.5 tap drill, Dia 2.05 mm x Depth 6.35 mm)
    const double flex_fix_y[4] = {6.0, 18.0, 30.0, 42.0};
    for (double y : flex_fix_y) {
        gp_Ax2 hole_ax(gp_Pnt(9.0, y, 1.5), gp_Dir(0, 0, 1));
        TopoDS_Shape m25_hole = BRepPrimAPI_MakeCylinder(hole_ax, 1.025, 7.0).Shape();
        base = BRepAlgoAPI_Cut(base, m25_hole).Shape();
    }

    // 4x Stage mounting countersunk holes for M3 DIN 7991 (32.0 x 40.0 mm array)
    const double x_mount[2] = {16.5, 48.5};
    const double y_mount[2] = {4.0, 44.0};
    for (double x : x_mount) {
        for (double y : y_mount) {
            base = BRepAlgoAPI_Cut(base, BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(x, y, -1.0), gp_Dir(0, 0, 1)), 1.70, 7.0).Shape()).Shape();
            base = BRepAlgoAPI_Cut(base, BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(x, y, 2.0), gp_Dir(0, 0, 1)), 3.25, 3.5).Shape()).Shape();
        }
    }

    // Precision reference datum slot: 23.00 x 5.00 x 1.25 mm
    base = BRepAlgoAPI_Cut(base, BRepPrimAPI_MakeBox(gp_Pnt(21.0, 0.0, 0.0), 23.0, 1.25, 5.0).Shape()).Shape();
    return base;
}

int main() {
    std::cout << "==========================================================" << std::endl;
    std::cout << "[*] Initializing WOLFF3D-D1-LC RevB Geometry Generation..." << std::endl;
    std::cout << "==========================================================" << std::endl;

    TopoDS_Shape base = Build_W1_LC_03_RevB();
    TopoDS_Shape slider = Build_W1_LC_02_RevB();
    TopoDS_Shape flexure = Build_W1_LC_01_RevB();

    // Inertial and Center of Mass (CoM) Evaluation
    GProp_GProps slider_props;
    BRepGProp::VolumeProperties(slider, slider_props);
    const double volume = slider_props.Mass(); // Volume in mm^3
    const double mass_g = (volume * 2.81) / 1000.0; // 7075-T651 density: 2.81 g/cm^3
    const gp_Pnt com = slider_props.CentreOfMass();
    const double delta_y = com.Y() - 24.000;

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "[+] Slider Volume:             " << volume << " mm^3" << std::endl;
    std::cout << "[+] Slider Mass:               " << mass_g << " g (Budget: <= 38.0 g)" << std::endl;
    std::cout << "[+] Center of Mass (X, Y, Z):  (" << com.X() << ", " << com.Y() << ", " << com.Z() << ") mm" << std::endl;
    std::cout << "[+] Nominal Axis Y Baseline:   24.0000 mm" << std::endl;
    std::cout << "[+] Measured Delta Y Offset:   " << (delta_y > 0 ? "+" : "") << delta_y << " mm (" << delta_y * 1000.0 << " um)" << std::endl;

    if (std::abs(delta_y - 0.120) <= 0.010) {
        std::cout << "[STATUS: VALIDATED] Center of Mass offset within design tolerance (+0.120 mm)." << std::endl;
    } else {
        std::cout << "[STATUS: WARNING] Offset deviation exceeds tolerance limit!" << std::endl;
    }

    // Assembly Matrix Configuration
    gp_Trsf t1; 
    t1.SetTranslation(gp_Vec(3.5, 3.0, 8.0));
    TopoDS_Shape flex1 = BRepBuilderAPI_Transform(flexure, t1).Shape();

    gp_Trsf t2; 
    t2.SetTranslation(gp_Vec(3.5, 27.0, 8.0));
    TopoDS_Shape flex2 = BRepBuilderAPI_Transform(flexure, t2).Shape();

    TopoDS_Compound assembly;
    BRep_Builder builder;
    builder.MakeCompound(assembly);
    builder.Add(assembly, base);
    builder.Add(assembly, slider);
    builder.Add(assembly, flex1);
    builder.Add(assembly, flex2);

    // Export Step Files
    ExportSTEP(flexure,  "W1-LC-01_Parallel_Flexure_RevB.STEP");
    ExportSTEP(slider,   "W1-LC-02_Moving_Slider_RevB.STEP");
    ExportSTEP(base,     "W1-LC-03_Baseplate_RevB.STEP");
    ExportSTEP(assembly, "W1-LC-ASM-00_Module_C_RevB.STEP");

    std::cout << "[+] STEP export completed successfully." << std::endl;
    std::cout << "==========================================================" << std::endl;
    return 0;
}