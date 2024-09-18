# Robot Fabrics

This repository contains the code for the paper submission to ICRA 2025:

- Scalable Plug-and-Play Robot Fabrics Based on Kilobot Modules

## Files

- ```deformation-correcting.c``` : Straight motion controller proposed in Pratissoli et al. 2023 ([paper](https://doi.org/10.1038/s41467-023-39660-6), [code](https://github.com/ilpincy/argos3-kilobot/blob/softrobot/src/examples/behaviors/KBSR_rebuild.c)). Each Kilobot measures the distance to its neighbors to detect the deformation in the lattice-structure and corrects its motion accordingly to maintain the shape of the structure.
- ```turning.c``` : Proposed turning controller executed on each Kilobot to make the robot fabric follow a curved trajectory.

## Supplementary videos

Experiment videos can be found [here](10.15131/shef.data.27021310).