# Simulatrix v3 Pydroid Edition - Ultimate Single-File 2D Simulation Engine (GJK + EPA Collisions) 🚀

**Author:** Dhanwanth.V
**Version:** 3.0 (Pydroid Edition)
**License:** MIT

---

## Overview 🌍

Simulatrix v3 Pydroid Edition is a high-performance, single-file 2D physics simulation engine for Android using Pydroid & Pygame. It features **GJK + EPA collision detection** for circles and convex polygons, with **rotational physics, object pooling**, and a **mobile-friendly GUI** for interactive simulation control.

---

## Features 🚀

* **Collision Detection:**

  * Circle-circle, circle-polygon, polygon-polygon collisions using GJK + EPA 💥
  * Realistic rotational collision response with torque and angular impulse 🤠
  * Support for convex polygons with arbitrary vertices 🔷

* **Physics Integration:**

  * Semi-implicit Euler integration 🌋
  * Velocity Verlet integration (toggleable at runtime) 🔄

* **Broadphase Optimization:**

  * Spatial hash grid for efficient collision pair pruning 🔧
  * Naive broadphase fallback 🔄

* **Memory Management:**

  * Object pooling for short-lived particles 🤝

* **Rendering & Debugging:**

  * Pygame-based rendering of circles & polygons 🛠️
  * Debug HUD with performance metrics 📊
  * Grid overlay toggle 🟩
  * Frame recording to PNG sequence 📷

* **Mobile GUI Controls:**

  * Touch-friendly buttons for spawning entities, toggling grid/HUD, pausing simulation, switching integrators, and clearing scene 👇

* **Example Constraints:**

  * Spring constraints between entities 🔄

---

## Installation & Requirements 📚

* Pydroid 3 app
* Python 3.7+ in Pydroid
* Pygame (tested with pygame-ce 2.5.3)

Install Pygame via Pydroid's pip:

```bash
pip install pygame-ce
```

Run the engine with:

```bash
python simulatrix_v3_pydroid.py
```

---

## Mobile GUI Controls 💃

* Touch buttons for:

  * Toggle grid overlay 🟩
  * Toggle HUD 📊
  * Toggle integrator (Euler / Verlet) 🔄
  * Pause/unpause simulation ⏸
  * Start/stop frame recording 📷
  * Spawn/remove entities 🔄
  * Clear and respawn entities 🔒

---

## Comparison with Previous Versions 📊

| Feature / Aspect        | Simulatrix v1                                   | Simulatrix v2                                       | Simulatrix v3 (Pydroid Edition)                                          |
| ----------------------- | ----------------------------------------------- | --------------------------------------------------- | ------------------------------------------------------------------------ |
| Collision Detection     | Basic circle-circle & AABB collisions using SAT | Polygon collisions with SAT, improved broadphase    | GJK + EPA for robust polygon & circle collisions 💥                      |
| Rotational Physics      | Limited / no angular velocity & torque          | Angular velocity support, simple collision response | Realistic rotational collision response with torque & angular impulse 🤠 |
| Broadphase Optimization | Naive broadphase                                | Spatial hash broadphase                             | Optimized spatial hash with neighbor caching 🔧                          |
| Integration Methods     | Semi-implicit Euler                             | Euler & Verlet (toggleable)                         | Same as v2, improved stability 🔄                                        |
| Memory Management       | No object pooling                               | Object pooling for particles                        | Enhanced object pooling with reset logic 🤝                              |
| Rendering               | Basic circle rendering                          | Polygon rendering added                             | Improved polygon rendering with rotation 🛠️                             |
| Debugging & HUD         | Minimal debug info                              | Debug HUD & grid overlay                            | Enhanced HUD with profiling & GUI buttons 📊                             |
| Constraints             | None                                            | Basic spring constraints                            | Same, improved integration 🔄                                            |
| Code Structure          | Multiple files                                  | Modularized                                         | Single-file, optimized for distribution 💾                               |
| Performance             | Moderate                                        | Improved with spatial hash                          | Further optimized with minimal temporaries & caching 🔥                  |

---

## Code Highlights 💡

* **GJK + EPA Collision Detection:** Accurate collisions & penetration depth calculation.
* **Rotational Collision Response:** Impulse updates linear & angular velocities, simulating torque.
* **Spatial Hash Grid:** Efficient broadphase, hundreds of entities in real-time.
* **Object Pooling:** Reduces garbage collection for particle-heavy simulations.
* **Mobile GUI:** Touch-friendly interactive buttons for full control.

---

## Extending Simulatrix v3 Pydroid Edition 📚

* Add concave polygon support via decomposition.
* Implement friction & rolling resistance.
* Add more constraints (rods, joints).
* Integrate with game frameworks or scripting for scenes.

---

## Acknowledgments 💜

* Thanks to Pygame & Pydroid communities for support & tools.

---

## License 📖

MIT
---
