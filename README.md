# Simulator
Simulator python scripts for the SPV.

This repo contains multiple simulation tools and scripts to assist the SPV project. These bits and pieces are "work in progess", so they might not be well documented or even working. But the plan is to have these useful pieces around to be able to implement them in the SPV PC software at some point. So far, there are
* Motion controller simulator: Calculates in a tick-exact manner how a cyclespecification will be executed on the SPV hardware.
* Coordinate transformation for the bow positioning motors: A bow position given by (r, gamma) is transformed in two machine coordinates (alpha1, alpha2) which can be set on the POSX and POSY axis.
* Inverse transformation for bow positioning motors

Planned:
* Simplified motion calculator (only speeds and distances, no stepper information)
* Minimum time for given distance calculator
