# Rote

> **rote** / roʊt / *n.*  
> a fixed, habitual, or mechanical course of procedure; routine:

A library of functions that I commonly use.   
Mostly related to how cameras are modeled, transformation matrices, etc.

*Rote* is also a homophone to *wrote*, which is fitting, since I wrote this library.

## Installation
```bash
python -m pip install git+https://github.com/SebastianGrans/Rote
```

## Development 

For development, this repo can be cloned, and subsequently installed as a pip package by running: 

```bash
pip install --editable .
```

Instead of installing the package by moving it to the `site-packages` folder, an `*.egg-link` file, a form of symlink, is created. This allows for direct modifaction of the installed package.

## TODO 
Features to add: 
* Camera calibration 
* Divide into submodules
  