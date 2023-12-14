from setuptools import setup, find_packages

setup(
    name="ScreamingChannels",
    version="2.0",
    packages=find_packages(),
    python_requires=">=3.0,!=3.0.*,!=3.1.*,!=3.2.*,!=3.3.*",
    entry_points={
        "console_scripts": [
            "sc-experiment = screamingchannels.reproduce:cli",
            "sc-attack = screamingchannels.attack:cli",
            "sc-triage = screamingchannels.triage:cli",
            "sc-simple = screamingchannels.simple:cli",
            "sc-waterfall = screamingchannels.waterfall:main"
        ]
    },
    install_requires=[
        "click==8.1.3",
        "numpy==1.23.0",
        "scipy==1.9.1",
        "pyserial==3.4",
        "matplotlib==3.5.2",
        "enum34==1.1.10",
        "pmt==0.0.4",
        "pyts==0.12.0",
        "llvmlite==0.39.1",
        "numba==0.56.2",
        "statsmodels==0.13.2",
        "pandas==1.4.4",
        "scikit-learn==1.1.2",
        "future==0.18.2",
        "pycryptodome==3.15.0",
        "pyzmq==24.0.0",
        "peakutils==1.3.4",
        "tabulate==0.8.10",
        "kiwisolver==1.4.4",
        "pyparsing==3.0.9"


# to use system packages
#        ln -s /usr/lib/python2.7/site-packages/gnuradio ../../../../screaming-channel/nRF52832/experiments/VENV_sc/lib/python2.7/site-packages
#        "gnuradio",
#        "osmosdr",
    ],

    author="S3@EURECOM",
    author_email="camurati@eurecom.fr, poeplau@eurecom.fr, muench@eurecom.fr",
    description="Code for our screaming channel attacks",
    license="GNU General Public License v3.0"
    # TODO URLs
)
