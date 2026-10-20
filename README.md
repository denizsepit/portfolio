# M. Deniz Sepit — Electrical & Electronics Engineering Portfolio

Professional engineering portfolio showcasing analog circuit and 2-layer PCB hardware design (PSpice & KiCad), closed-loop embedded control (ArduKit), computer vision hardware architecture, and numerical modeling in MATLAB.

- **Author:** M. Deniz Sepit
- **Institution:** Izmir University of Economics (B.Sc. in Electrical and Electronics Engineering)
- **Contact:** [deniz.sepit@std.ieu.tr](mailto:deniz.sepit@std.ieu.tr)

## Repository Structure

```text
.
├── index.html                                           # Main portfolio & interactive simulation workbench
├── .nojekyll                                            # Bypasses Jekyll processing on GitHub Pages
├── assets/
│   ├── kicad_schematic.png                              # Multi-stage audio amplifier schematic (KiCad)
│   ├── kicad_pcb_routing.jpg                            # 2-layer PCB copper routing & layout
│   └── kicad_3d_assembly.jpg                            # 3D CAD board assembly render
├── matlab/
│   ├── electromechanical_drive_simulation.m             # PMSM / traction drive RK4 ODE & S-curve simulation
│   ├── smart_irrigation_telemetry_analysis.m            # ArduKit closed-loop soil moisture telemetry analysis
│   └── structure_from_motion_3d_interpolation.m         # 3D Structure from Motion & Bicubic/Lanczos kernels
└── README.md
```

## Deploying to GitHub Pages

1. Create a new repository on GitHub (e.g., `deniz-sepit-portfolio` or `<your-username>.github.io`).
2. Extract this zip archive and push all files to the `main` branch:
   ```bash
   git init
   git add .
   git commit -m "Initial commit: Electrical & Electronics Engineering portfolio"
   git branch -M main
   git remote add origin https://github.com/<your-username>/<your-repo>.git
   git push -u origin main
   ```
3. In your GitHub repository, navigate to **Settings → Pages**.
4. Under **Build and deployment**, set **Source** to **Deploy from a branch**, select the `main` branch and `/ (root)` folder, and click **Save**.
