# Lunar Landing Site Evalution and Path Planning

Introduction.
This project was done in collaboration with Planerty Labs at Carnegie Mellon University (CMU). The project was a part of a larger mission to explore the surface of the Moon, particularly in low-light zones like the pits, which cannot be observed by cameras on satellites orbiting the Moon. Sending a rover to explore these areas is not an option as the rover needs to maintain a line of sight with the lander at all times to transmit data. The project aimed to develop an algorithm to find the best landing site for the rover and an optimal path for it to travel.
<p align="center">
  <img width="497" alt="Lunar_project_pit_3d" src="https://user-images.githubusercontent.com/47504920/230153815-d9191c1c-8864-497d-b1b1-dcb13a27f32e.png">
</p>


Why the project?
The project aimed to use a 3D render of the surface of the Moon to generate a depth map of the surface. To achieve this, the project utilized ray tracing, a technique used in computer graphics to simulate the path of light rays as they interact with objects in a scene. By using ray tracing to render the surface of the moon, the project was able to calculate the distance between the camera (or the observer) and each point on the surface, which allowed for the generation of a depth map. This depth map then enabled the project to generate a slope map and a Line of Sight (LOS) map. Ray tracing was used in this project to render a 3D model of the lunar surface and obtain depth information that was essential in generating the slope and LOS maps, as well as identifying optimal landing sites and paths for the rover.The depth map was then used to generate a slope map, where darker shades represented higher slopes or inaccessibility. The rover had physical constraints such as maintaining a line of sight with the lander, slope constraints, and distance constraints. The landing site needed to be evaluated based on these parameters. The algorithm was developed to identify the best landing site and the most optimal path for the rover to explore.

How we did it?
Using C++, we generated a slope map from the 3D ply fly of the lunar surface. The depth map data was used to generate a Line of Sight Map (LOS Map) that represented all the points on the map where the lander would be able to communicate with the rover. The LOS Map had categories, and we generated a LOS Aggregate Map for the landing site to ensure that the entire pit was covered by the line of sight. We employed the A-Star algorithm to generate the best path for the rover to follow. We generated 36 sites around the pit space at 10-degree intervals and found the optimal sites that could be covered no matter where the lander landed in its landing circle. We used a complex G-score function that took into account different parameters and assigned penalties to generate a ranking of landing sites for the rover.

<p align="center">
  <img alt="Lunar_project_Site_Evaluator_Working" src="https://user-images.githubusercontent.com/47504920/230153940-57681acb-95b6-4d7c-981e-102dd3859e09.gif">
</p>


Results:
The algorithm was able to find the best landing site and generate an optimal path for the rover to travel. An overlay extension was created to overlay the generated path with the actual image from the Moon, making it easier to visualize.

![Lunar_project_Site_Evaluator_Working](https://user-images.githubusercontent.com/47504920/230153940-57681acb-95b6-4d7c-981e-102dd3859e09.gif)

<p align="center">
  <img width="358" alt="Lunar_project_overlay" src="https://user-images.githubusercontent.com/47504920/230153960-840e40ef-730e-4e69-bcf5-8887ed5f6883.png">
</p>
