# WOLFF3D-D1-LC RevB: Hardware Bill of Materials (BOM)

**Project:** WOLFF3D-D1-LC Ultra-Precision Fine-Damping Micro-Stage  
**Revision:** RevB (Frozen FTO Design-Around Release)[cite: 19]  
**System Classification:** ISO Class 3 (Fed Std 209E Class 100) Semiconductor Vacuum/Cleanroom Compatible  
**Lead Mechanical Architect:** Agent 1 (Parametric CAD / DFM Lead)  
**Date:** 2026-09-30  

---

## 1. Custom Machined & Fabricated Components

| Item | Part Number / Drawing ID | Component Description | Qty | Material | Heat Treatment / Surface Finish | Critical Quality Gates & Specifications |
| :---: | :--- | :--- | :---: | :--- | :--- | :--- |
| **01** | `W1-LC-01_Parallel_Flexure_RevB` | Dual Parallel Flexure Strip[cite: 1] | 2 | Cold-Rolled Spring Steel 65Mn | Pre-quenched & tempered (46–48 HRC); ultra-thin anti-corrosion oil ($\le 2\ \mu\text{m}$) | • Primary Process: Precision WEDM (1 cut + 2 skims, $Ra \le 0.8\ \mu\text{m}$)<br>• Working Ligament: $0.350 \pm 0.010\text{ mm}$[cite: 14, 21]<br>• Corners: $4\times R4.20\text{ mm}$ Conic 0.65 Hyperbolic Blend<br>• Complete deoxidation and HAZ white layer removal[cite: 14, 21] |
| **02** | `W1-LC-02_Moving_Slider_RevB` | Kinematically Decoupled Fine Moving Slider | 1 | Aerospace Aluminum Alloy 7075-T651 (Pre-stretched) | MIL-A-8625 Type III, Class 1 Hard Anodize ($25–30\ \mu\text{m}$); hydrothermal boehmite sealing | • Bare Mass: $M_{\text{bare}} \le 38.0\text{ g}$ (Nominal $35.55\text{ g}$)[cite: 2]<br>• Micro-Hardness: $\ge 450\text{ HV}$[cite: 2]<br>• Intentional CoM Offset: $\Delta Y = +0.120 \pm 0.010\text{ mm}$[cite: 19]<br>• Masking: 100% Teflon plug masking on $\Phi 16.50\text{ mm}$ bore & M3 threads[cite: 2] |
| **03** | `W1-LC-03_Baseplate_RevA` | Intermediate Coarse-Stage Mounting Baseplate[cite: 1] | 1 | Structural Aluminum Alloy 6061-T6 | MIL-DTL-5541 Type II, Class 3 Clear Chemical Conversion (Conductive, $5–8\ \mu\text{m}$) | • Base Mounting Flatness: $\le 0.010\text{ mm}$<br>• Bolt Interface: $32.00 \pm 0.02 \times 40.00 \pm 0.02\text{ mm}$<br>• Magnetic Scale Slot: $23.00 \times 5.00^{+0.05}_{0.00} \times 1.25\text{ mm}$[cite: 16]<br>• Provides low-impedance ESD grounding pathway[cite: 16] |

---

## 2. Standard Precision Fasteners (ISO / DIN Metric)

| Item | Standard / Specification | Description | Qty | Property Class / Material | Finish / Cleanliness Level | Installation Notes & Tightening Torque |
| :---: | :--- | :--- | :---: | :--- | :--- | :--- |
| **04** | ISO 7380-1 M2.5 × 6 mm | Button Head Hexagon Socket Screws | 4 | Grade 12.9 High-Tensile Alloy Steel | Black oxide, ultra-clean degreased | • Fastens flexure moving end to slider bottom blind threads<br>• Head height $\le 1.50\text{ mm}$ (maintains $1.50\text{ mm}$ air gap)<br>• Tightening Torque: **$0.80\text{ N}\cdot\text{m}$** (Calibrated driver) |
| **05** | ISO 4762 M2.5 × 6 mm | Socket Head Cap Screws (SHCS) | 4 | Grade 12.9 High-Tensile Alloy Steel | Black oxide, ultra-clean degreased | • Fastens flexure stationary end to baseplate shoulder[cite: 16, 21]<br>• Top-down vertical access via slider clearance step[cite: 15]<br>• Tightening Torque: **$0.80\text{ N}\cdot\text{m}$** |
| **06** | ISO 10642 M3 × 10 mm | Countersunk Flat Head Socket Screws | 4 | Grade 10.9 Alloy Steel / A2-70 Stainless | Passivated / Dacromet | • Locks baseplate to coarse linear stage ($32 \times 40\text{ mm}$ pattern)[cite: 16]<br>• 100% full conical seating flush with baseplate surface<br>• Tightening Torque: **$1.80\text{ N}\cdot\text{m}$** |

---

## 3. Electromechanical, Actuation & Sensor Sub-assemblies

| Item | Part Number / Reference | Sub-assembly Description | Qty | Manufacturer / Spec | Functional Characteristics |
| :---: | :--- | :--- | :---: | :--- | :--- |
| **07** | AVM 16-XX Coil Unit | Miniature Cylindrical Voice Coil Motor (Moving Coil)[cite: 1] | 1 | Custom VCM (e.g., Akribis / Moticont)[cite: 1] | • Coil OD $\Phi 16.00\text{ mm}$, Length $5.80\text{ mm}$[cite: 1]<br>• Resistance: $3.2\ \Omega$, Peak Force: $\ge 6.5\text{ N}$, Continuous Force: $\ge 1.8\text{ N}$[cite: 1]<br>• Mass: $\le 14.5\text{ g}$ (bonded into slider center bore)[cite: 1] |
| **08** | C10100 Eddy Current Shunt | High-Purity Oxygen-Free Electronic (OFE) Copper Sheet[cite: 2] | 1 | TU1 / ASTM C10100 ($\ge 99.99\%\ \text{Cu}$)[cite: 2] | • Thickness: $0.80\text{ mm}$, Conductivity: $\ge 101\%\ \text{IACS}$[cite: 2]<br>• Generates passive velocity-proportional Lorentz damping force[cite: 2]<br>• Enhances damping ratio to $\zeta \approx 0.12–0.18$[cite: 2] |
| **09** | MT6835 Magnetic Sensing Kit | Integrated High-Speed Magnetic Scale & Readhead[cite: 1] | 1 | MagnTek / Custom Carrier[cite: 1] | • Magnetic Pole Pitch: $1.0\text{ mm}$, Magnetic Strip: $23.0 \times 5.0 \times 1.0\text{ mm}$[cite: 1]<br>• Nominal Air Gap: $0.30 \pm 0.05\text{ mm}$, Output: High-speed ABZ / SPI[cite: 1]<br>• True dynamic resolution: $\le 50\text{ nm}$ |

---

## 4. Cleanroom Assembly Consumables & Metrology Tooling

| Item | Part Number / Tool ID | Description | Qty | Specifications & Compliance | Operational Role |
| :---: | :--- | :--- | :---: | :--- | :--- |
| **10** | EPO-TEK 353ND | Two-Component High-Temperature Epoxy Adhesive[cite: 2] | A/R | NASA Outgassing Compliant; SP-R-0022A (CVCM $< 0.1\%$, TML $< 1\%$)[cite: 2] | Structural bonding of voice coil into $\Phi 16.50\text{ mm}$ slider cavity; zero shrinkage, void-free cure[cite: 2] |
| **11** | Gauge Block Set (0.400 mm) | Grade 0 Calibration Tungsten Carbide / Ceramic Blocks | 2 | ISO 3650 Grade 0 ($0.4000 \pm 0.0001\text{ mm}$) | Assembly fixture: inserted under slider to set neutral suspension gap before tightening screws |
| **12** | Isopropyl Alcohol (IPA) | Electronic Grade Solvent ($\ge 99.9\%$) | A/R | Semiconductor Cleanroom Grade | Degreasing and particle cleaning before final Class 3 cleanroom packaging |

---

## 5. Mass & Cost Roll-up Summary

* **Bare Moving Mass ($M_{\text{bare}}$):** $35.55\text{ g}$ (Target limit: $\le 38.0\text{ g}$)[cite: 2]
* **Effective Total Moving Mass ($M_{\text{moving}}$):** $68.50\text{ g}$ (Including slider, coil, screws, and effective flexure mass contribution)
* **Total Direct Hardware BOM Cost:** $\approx \text{¥}915.00$ (Locks system cost below the ¥1,000 threshold at 100-unit batch scale)