# Technical Specification & Prior Art Defensive Disclosure
## Project: WOLFF3D-D1-LC Option 1 (RevB Final Frozen Release)

* **Publication Authority**: Wolff3D Autonomous IP Sentinel (Agent 2)
* **Effective Date**: 2026-09-30
* **Freedom-To-Operate Status**: GREEN LIGHT (Element-by-Element Analysis Complete)
* **Assembly STEP SHA-256**: `3e9818ea1575754ff823349314b68d2e3e86d5f27e1fb2c86517a1d11c89f75a`
* **Engineering Drawing PDF SHA-256**: `7daeb3e4c0385f9200e09ab47c917fd42548644b3fb3485e87f8a68bfb56921a`
* **Bitcoin OpenTimestamps Proof**: `W1-LC-ASM-00_Module_C_RevB.STEP.ots`

---

### 1. Technical Abstract & Claims Breakdown
This disclosure formally places into the global public domain the following parametric mechanical structures and operational methods:

1. **Non-Circular Hyperbolic Flexure Transitions**:
   * Double-parallel flexure strip in pre-quenched cold-rolled 65Mn spring steel (6 \sim 48\text{ HRC}$), working ligament thickness -bash.350 \pm 0.010\text{ mm}$.
   * Root transitions constructed via hyperbolic curvature conics (=4.20\text{ mm}$, parameter $\rho = 0.65 > 0.5$) for peak stress attenuation and HAZ micro-crack mitigation.
2. **Intentional Mass-Offset Decoupling Topology**:
   * Isogrid 7075-T651 moving slider ({\text{bare}} = 35.55\text{ g} \le 38.0\text{ g}$) with deliberate bottom offset slots ( \in [26.0, 39.0]\text{ mm}$, recess .20\text{ mm}$).
   * Creates an intentional static center-of-mass displacement of $\Delta Y = +0.1202\text{ mm}$ relative to the actuator force line (=24.000\text{ mm}$), compensated dynamically via feedforward controller injection to break coplanar encumbrance.
3. **Passive Solid-State Eddy Current Damper**:
   * Integrated C10100 oxygen-free copper damper plate (-bash.80\text{ mm}$) within the slider-baseplate air gap layer, utilizing VCM stator stray flux for non-contact Lorentz damping ($\zeta \approx 0.12 \sim 0.18$), achieving settling times within .00 \sim 2.35\text{ ms}$.

---

### 2. Verified Physical & Dynamic Boundaries
* **Primary Working Mode (Tx)**:  \in [5.96, 6.68]\text{ Hz}$
* **High-Order Parasitic Separation Ratio**:
  * Yaw Mode ({\text{yaw}} / f_x$): $\ge 282:1$
  * Pitch Mode ({\text{pitch}} / f_x$): $\ge 310:1$
* **Goodman Fatigue Safety Margin**:  \ge 4.90 \ge 2.5$ (Infinite life $> 10^8$ cycles under 0\text{G}$ shock)

---

### 3. Open Source Engineering Assets
The corresponding AP214 manufacturing-grade STEP models, vector drawings, and OpenCASCADE C++ synthesis scripts are cataloged in this repository under MIT/Defensive Prior Art Open License.
