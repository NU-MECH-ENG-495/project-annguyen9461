# Camera-Assisted 2D Shape Cutting (Claybot)

## Project Overview  
This project automates the process of cutting a **2D circle** into a predefined goal shape using **computer vision** and a **fixed knife attached to a linear actuator**.  

### Goal Shapes  
- **Triangle**  
- **Hexagon**  
- **Square**  

### Approach  
1. **Shape Detection**:  
   - A **camera** captures the starting **circle**.  
   - The system **overlays the goal shape** on the detected circle.  
2. **Cutting Mechanism**:  
   - A **fixed knife** is attached to a **linear actuator** that moves **up and down**.  
   - The workpiece is rotated to align with the cutting path.  
3. **Progress Tracking**:  
   - The camera continuously **analyzes the current shape**.  
   - The system updates the overlay to guide further cutting.  
   - Cutting continues **until the shape match is high enough** or all **four quadrants** are processed.  
4. **Rotational Cutting**:  
   - The **table rotates 90 degrees** at intervals to assist with even shaping.  

## Stretch Goals  
- **Teleoperation of the Allegro Hand** for cutting:  
  - **Thumbs up → Move up**  
  - **Thumbs down → Move down**  
  - **Open hand → Open tool**  
  - **Close hand → Pick up tool**  
- **Autonomous Cutting** with the Allegro Hand.  
- **Voice control** to choose a target shape.  

## Image Processing  
- **Input image** to identify the **starting circle**.  
- **Overlay goal image** to guide cutting.  
- **Continue cutting** until:  
  - **Shape match percentage** is high enough.  
  - **All four quadrants are processed** (360-degree completion).  

## Next Steps  
- Implement **image processing** for **shape tracking**.  
- Develop **cutting path generation** based on detected contours.  
- Integrate **linear actuator control** for the fixed knife.  
- Test **table rotation and quadrant-based cutting**.  
