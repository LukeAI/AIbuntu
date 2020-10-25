# Update
I'm not maintaining this script anymore. I've been told it's useful so I'll leave it here for reference, but my advice is to look through and use it to help you do what you want to do, rather than just running it blindly. You might not need Anaconda, Autoware etc. so if you don't, don't install them.


# AIbuntu

Run with no options for more info. This script takes a *really* long time to run.
 
This script:
- uninstalls some bloat, enables extra repos for apt, installs lots of useful little things that make an Ubuntu installation complete - build essentials, codecs, spell-check dictionaries, vlc, advanced power management, common dependencies, Teamviewer.
- It downloads, compiles, installs and configures ROS and Autoware including all dependencies and a hack for dealing with environment conflicts. Note: This takes a *long* time.
- It installs Eclipse and clang-format for c/c++
- Downloads and installs the Anaconda scientific computing platform and creates sample enviornment for Tensorflow (GPU-enabled) with Spyder and another for Pytorch/Spyder (GPU enabled). These can be entered with "source activate tf" and "source activate pytorch" or through the Anaconda navigator. Note: these are included for pedagogical value and for quick testing - it is recommended to create new enviornments per application for actual development.
If unfamiliar with Anaconda, it is heavily recommended to read the docs before using: https://conda.io/docs/user-guide/tasks/manage-environments.html
Note: The Nvidia GPU driver must be enabled for these to work. This is best done through the Ubuntu Additional Drivers menu.

It is recommended to read through this script before running it so that you understand what it is doing, run without arguments as a normal unpriviliged user for further instruction.
