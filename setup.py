import setuptools

setuptools.setup(
    name="multi-dof-manipulator",
    version="1.0.0",
    description="Universal 6-DOF Robotic Arm Framework",
    packages=setuptools.find_packages(),
    install_requires=[
        "numpy>=1.20",
        "pyyaml>=6.0",
    ],
    python_requires=">=3.8",
)