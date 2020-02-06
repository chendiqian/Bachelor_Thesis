# Bachelor_Thesis
Multi-robot collision-free path planning
This is my bachelor project implemented with MATLAB.  

The goal is to plan the collision-free paths for multiple robots, in an environment of warehouse. The map is grid, both time and map are discrete.  

The basic implementation is multiple integer programming(MIP) with 3rd party solver namely Gurobi. So before running the program, you'll have to download Gurobi for MATLAB version. Then simply run main.m.  

With the optimal solutions, I also implemented visualization and GUI interaction, so that the users can clearly observe how the robots move on the grid map with time lapse.  

I implemented the project with sub-mapping methods, in order to speed up the optimization. One method is to split the map into sub-maps, it is optimal, but computationally expensive. Another way is to split the map with one-direction canes, its computation is much faster, but robots travels a little more distance.  

Since I have implemented GUI and recorded some videos, you can visit my demo videos through these Youtube websites  
Single robot and single task:  
https://youtu.be/XJKYyZ-L12Q  
https://youtu.be/PWc9KHyhNXA  

Multi-robot and multi-task:  
https://youtu.be/uW3nt1RSeJc  
https://youtu.be/bli8woVZxSQ  

Task generation:  
https://youtu.be/N-WhrrJRyAA  
