from setuptools import setup, find_packages

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setup(
    name='msur_lib',
    version='0.1',
    author="Photon94",
    long_description=long_description,
    description='Tools for interactive with msur auv',
    install_requires=['pydantic', ],
    packages=find_packages(),
    python_requires=">=3.8",
)
