The system aims to overcome the challenges of accurate ground segmentation, including sparsity of data, lack of spatial structure, and tight performance constraints.

The results show the high difficulty of the hill scenario, especially GndNet is not able to provide an accurate terrain estimation.

The GroundGrid system relies on 2D elevation maps to solve the terrain estimation and point cloud ground segmentation problems. The system also uses Welford’s online algorithm to calculate the variance of the point cloud data.

The key steps are:
1. Outlier Filtering: First, the method identifies and removes outlier points that are significantly below the ground level, likely due to reflections from the vehicle's body.
   
2. Point Cloud Rasterization: The remaining point cloud is rasterized into a 2D grid map, creating layers that store information like point height variance, minimum/maximum/average height, and point count for each grid cell.
   
3. Ground Cell Classification: The method uses the point height variance information to identify grid cells that contain only ground points. Cells with low variance are classified as ground, considering the cell's neighborhood to accommodate sparse point cloud data.

4. Ground Elevation Calculation: For the ground cells, the method calculates an estimated ground elevation and a confidence value based on the weighted minimum height and point count in each cell. This information is integrated with previous estimates to get a more robust ground elevation.

5. Terrain Interpolation: The ground elevation is then interpolated for cells where no ground points were detected, using a confidence-weighted average of the cell's own estimated elevation and the elevation of its neighboring cells.

6. Point Cloud Segmentation: Finally, the point cloud is segmented into ground and non-ground points based on the estimated ground elevation map.

The key idea is to leverage the 2D grid representation to efficiently perform ground detection and terrain estimation, while retaining information from previous observations to improve robustness.