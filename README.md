# LEACH-DYNAMIC-NS3
This code runs on all ns3 versions upto 3.40.

Change the values of nRemoteNodes and nRelayNodes to change the number of nodes in the simulation.

Number of nodes = nRemoteNodes * nRelayNodes

To run the simulation:

> cp proj/main.cc scratch/main.cc && NS_LOG="LteCustom=level_info" 
> ./ns3 run scratch/main.cc 

To generate PacketTrace files, add the following line to the end of the previous command:

> --cwd="/<required_folder>/output_customCS"

DIRECT CHANGE TO DEV
