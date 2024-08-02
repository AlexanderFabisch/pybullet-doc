# Sphinx Documentation for PyBullet

## Why?

Since PyBullet does not have a perfectly integrated and complete
documentation, this is an attempt to improve the situation. It uses sphinx,
which produces HTML that is faster to navigate through than the
[PyBullet Quickstart Guide](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.eqlo4t4ozi7),
and it is hosted in a git repository so that anybody can contribute with pull
requests.

## Dependencies

This repository should be cloned as a subfolder of the bullet physics
SDK. To properly set up this environment, execute the following commands:

```bash
git clone https://github.com/bulletphysics/bullet3.git
cd bullet3
git clone https://github.com/AlexanderFabisch/pybullet-doc.git
# or git clone git@github.com:YourUserName/pybullet-doc.git if you forked the repo
cd pybullet-doc
```

You can install dependencies either directly in your current Python
environment with

```bash
pip install -r requirements.txt
```

or you can create a conda environment:

```bash
conda env create -f environment.yml
conda activate pybullet_doc
```

## Build Documentation

You can build the documentation from the main folder with

```bash
make html
```

The result will be located at `build/html/index.html`.
