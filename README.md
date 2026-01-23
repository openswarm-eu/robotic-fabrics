# Robotic Fabrics

Robotic Fabrics is a plug-and-play framework for producing scalable, deformable modular robot entities. The software and hardware have been designed for a robotic fabric comprising up to 81 Kilobots that are connected using deformable links to move straight or turn left or right.

## Software

The Kilobot controller can be found in ```software/```.

- ```forward_open-loop.c``` : Open-loop controller to make a single Kilobot move forward.
- ```deformation-correcting.c``` : Straight motion controller proposed in Pratissoli et al. 2023 ([paper](https://doi.org/10.1038/s41467-023-39660-6), [code](https://github.com/ilpincy/argos3-kilobot/blob/softrobot/src/examples/behaviors/KBSR_rebuild.c)). Each Kilobot measures the distance to its neighbors to detect the deformation in the lattice-structure and corrects its motion accordingly to maintain the shape of the structure.
- ```turning_open-loop.c``` : Proposed turning controller executed on each Kilobot to make the robotic fabric follow a curved trajectory.

## Supplementary videos

Experiment videos can be found [here](https://doi.org/10.15131/shef.data.27021310).

## Citing Robotic Fabrics

If you use Robotic Fabrics in your research, please use the following BibTeX entry:

```BibTex
@article{obilikpa2025scalable,
    title = {Scalable Plug-and-Play Robotic Fabrics Based on Kilobot Modules},
    author = {Obilikpa, Stanley C. and Talamali, Mohamed S. and Miyauchi, Genki and Oyekan, John and Gro{\ss}, Roderich},
    journal = {IEEE Robotics and Automation Letters (RA-L)},
    volume = {10},
    number = {7},
    pages = {6832--6839},
    year = {2025},
    publisher = {IEEE},
    doi = {https://doi.org/10.1109/LRA.2025.3568313},
}
```

# Acknowledgement

Part of the source code in this repository is developed within the frame and for the purpose of the OpenSwarm project. This project has received funding from the European Unioan's Horizon Europe Framework Programme under Grant Agreement No. 101093046.

![OpenSwarm - Funded by the European Union](logos/ack.png)
