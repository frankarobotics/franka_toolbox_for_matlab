# Changelog:

## 4.0.0 (16-10-2025)

   - New auto-build system for franka.mtlbx under github release page. 
   - Default build --> libfranka 0.18.2.
   - FrankaRobot API: Added `FrankaRobotSettings` class for centralized robot configuration.
   - FrankaRobot API: New methods for impedance control (`setJointImpedance`, `setCartesianImpedance`), guiding mode (`setGuidingMode`), frame transformations (`setEE`, `setK`), and motion control (`stop`).
   - FrankaRobot API: Improved server lifecycle handling.
   - FrankaRobot API: Support for multiple `FrankaRobot` instances.

## 3.1.0 (20-10-2025)

   - Franka Toolbox for MATLAB open source release.