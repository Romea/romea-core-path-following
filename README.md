# romea_core_path_following

This project is C++ library that provides an implementation of a trajectory-following algorithm that can control a robot's deviation from its path as well as its speed. This is achieved by using the commands and sliding algorithms defined in the **romea_core_control** library.

This algorithm enables the robot to follow complex trajectories based on the **romea_core_path** library, potentially allowing for maneuvers such as half-turns or U-turns. It takes as input the results from a path matching algorithm provided in the **romea_core_path_matching** library.

## **Usage**

1. create a ROS workspace
2. cd worskpace
3. mkdir src
4. wget https://raw.githubusercontent.com/Romea/romea-core-path-following/refs/heads/main/romea_path_following_public.repos
5. vcs import src < romea_path_following_public.repos
6. build packages
   - catkin build for ROS1
   - colcon build for ROS2
7. create your application using this library

## **Contributing**

If you'd like to contribute to this library, here are some guidelines:

1. Fork the repository.
2. Create a new branch for your changes.
3. Make your changes.
4. Write tests to cover your changes.
5. Run the tests to ensure they pass.
6. Commit your changes.
7. Push your changes to your forked repository.
8. Submit a pull request.

## **License**

This project is released under the Apache License 2.0. See the LICENSE file for details.

## **Authors**

The romea_core_path_following library, written by **Jean Laneurit** and **Cyrille Pierre**, was developed during ANR Baudet Rob 2 and ANR Tiara projects. Several individuals contributed scientifically to the development of this library:

**Jean Laneurit**  
**Roland Lenain**  
**Cyrille Pierre**  
**Vincent Rousseau**  
**Benoît Thuilot**    

## **Contact**

If you have any questions or comments about romea_core_path_following library, please contact **[Jean Laneurit](mailto:jean.laneurit@inrae.fr)** or **[Cyrille Pierre](mailto:cyrille.pierre@inrae.fr)**.