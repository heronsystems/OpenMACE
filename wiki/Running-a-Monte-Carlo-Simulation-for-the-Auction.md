## <a name="quick-start"> Quick Start

To run a series of Monte-Carlo simulations over varying agents, tasks, bundle sizes, and assignment limits, the following command is recommended and can be used from `$MACE_ROOT/MaceSetup_Configs/AuctionMonteCarlo/python`:

**`RunMultipleLocalSims.py -r 10 --tasksList 1 5 --agentsList 1 5 --bundleSizesList 1 5 --assignmentLimitsList 1 5 --runTimeout 300 --overallSimTimeout 7200`**

The `-r` parameter specifies the number of runs. The various lists specify which configurations are of interest when looping over simulations. The `--runTimeout` parameter specifies how long a single run is permitted to run before it is timed out, excluding any required startup time. If a run times out, the event is logged, and it will be retried up to 3 times. The `--overallSimTimeout` parameter indicates how long simulations over all agents and runs can run for over a specific tasks, bundle size, and assignment limit configuration before timing out. If this timeout occurs, that configuration is skipped and the event logged.

## Running with Docker
The Monte-Carlo simulations can be run with a Docker container located on [Dropbox](https://www.dropbox.com/home/ODROID%20Disk%20Image/DockerContainerBackups). To install Docker on your platform, use one of the following:
* [Windows](https://docs.docker.com/docker-for-windows/install/)
* [Linux](https://docs.docker.com/install/linux/docker-ce/ubuntu/)
* [Mac](https://docs.docker.com/docker-for-mac/install/)
  - Note that the Mac Docker setup has not been verified for the MACE project

Once Docker is installed on your machine, download the `mace_monte_carlo.tar` file from Dropbox. Open a command prompt or terminal and navigate to the directory where the file was downloaded. To import the Docker container, run the following command (NOTE: For Linux, you will have to use `sudo` when running docker commands):

```
docker import mace_monte_carlo.tar ubuntu1604:mace_monte_carlo
```

Note that this process does not print anything to the screen and may take a few minutes, but once the container is imported, you can run the container by executing the following command:

```
docker run -i -t ubuntu1604:mace_monte_carlo /bin/bash
```

This will start the container. Once the container is started, change into the `OpenMACE` directory and make sure to pull the latest from the `Auction_Monte_Carlo_Sim` branch:

```
cd $MACE_ROOT
git pull origin Auction_Monte_Carlo_Sim
```

You will have to provide your GitHub username and password. To be sure you are running the latest code, make sure to build the project:

```
cd $MACE_ROOT/build
make && make install && ldconfig
```
Once the latest is pulled in and built, you can start the Monte-Carlo runs (see [Quick Start](#quick-start) for instructions).

### Exiting and Re-starting Containers
When you want to exit a container, simply stop any running processes and press `Ctrl+d` in the container. This will exit the container, but will not delete it. To re-start a container, run:

```
docker ps -a
```

and take note of the `CONTAINER_ID` field of the Docker container you wish to restart. Then run:

```
docker start -ai CONTAINER_ID
```

Where `CONTAINER_ID` is the field previously noted.

### Copying log files off Docker to Host
Once a Monte-Carlo simulation has been run, we want to copy the resulting log files from the Docker container to the host machine and upload them to a central repository (in this case, Dropbox). To copy the log files from a container, open a separate command prompt/terminal and run the following command:

```
docker ps -a
```

Take note of the `CONTAINER ID` field, as you will need it later. Navigate to a directory where you wish to copy the log files to on your host machine and run the following command:

```
sudo docker cp CONTAINER_ID:/OpenMACE/logs/MonteCarlo/results/ .
```

Once you are done copying over any log files, you can remove the log files from your container. Navigate to the `/OpenMACE/MaceSetup_Configs/AuctionMonteCarlo/python/config/` directory in the container and run `rm -rf *` to remove all log files. You should also clear the results directory: `rm -rf /OpenMACE/logs/MonteCarlo/results/` in preparation for the next set of runs.

## Description

A Monte-Carlo simulation can be run for the auction, available on the [Auction_Monte_Carlo_Sim](https://github.com/heronsystems/OpenMACE/tree/Auction_Monte_Carlo_Sim) branch. The simulation can be configured to test the auction under a variety of parameters. The recommended script to use is `RunMultipleLocalSims.py`, which runs simulations over a range of bundle sizes, assignment limits, number of tasks, and number of agents.

The scripts to run Monte-Carlo simulations are located in:

**`$MACE_ROOT/MaceSetup_Configs/AuctionMonteCarlo/python`**

The output of a simulation can be found in

 **`$MACE_ROOT/logs/MonteCarlo/Simulation_N`**

 for the `MonteCarloSim.py` and `RunLocalSimVaryingAgents.py` sripts, where N is the N-th Monte-Carlo simulation run. There will be several files located in these directories. An error log (**`errors.txt`**) indicates whether any MACE instance crashed, or otherwise timed out, during a run. The config file used to setup the simulation is copied into this directory, along with a copy of all auction logs output by each MACE instance per run. There is also a file (**`results.csv`**) which provides stats about the auction averaged over all runs.

When using the `RunMultipleLocalSims.py` script (recommended), the output is moved to

**`$MACE_ROOT/logs/MonteCarlo/{T}{TASK}_B{B}_A{A}`**

where `{T}` is the number of tasks, `{TASK}` is `TPA` or `TT` depending on if the `--tasksPerAgent` flag is passed, `{B}` the size of a bundle, and `{A}` the assignment limit for a given configuration. Within each of these folders, there are sub-directories for each number of agents that contains the results for the given setup with that number of agents.

## Configuration

The simulation can be configured using an XML config file. The structure of the config file, with explanations of each value, is below:

<details>
<summary>Configuration File Explanation</summary>

```xml
<MonteCarloConfig>
        <!-- Simulation Parameters -->
	<MaxRuntime>600</MaxRuntime> <!-- Maximum runtime for an auction per run, in seconds (REQUIRED) -->

        <Worker> <!-- Sets up a worker to monitor a MACE instance. One worker is required per agent. (REQUIRED 1+) -->
		<!-- Worker Script Parameters -->
                <ScriptPort>15000</ScriptPort> <!-- Port the worker script listens on (REQUIRED) -->
		<Address>127.0.0.1</Address> <!-- Address the worker script listens on (REQUIRED) -->

                <!-- MACE Parameters -->
		<Instance>1</Instance> <!-- MACE instance ID (also used as vehicle ID) (REQUIRED) -->

		<!-- External Link Parameters. One of ethernet or Digimesh is required. -->
		<EthernetPort>16000</EthernetPort> <!-- Ethernet port (ETHERNET) -->
		<!-- <DigimeshPort>COM5</DigimeshPort> --> <!-- Digimesh Port (DIGIMESH) -->
		<!-- <BaudRate>115200</BaudRate> --> <!-- Baud Rate (DIGIMESH) -->

		<!-- Agent Parameters (OPTIONAL) -->
                <!-- Auction parameters (OPTIONAL) -->
                <!-- See Defaults section below. Any default value can be overridden for each individual worker. -->
	</Worker>

        <!-- Default values for various parameters. If not set here or per worker, a hard-coded default value is used. -->
	<Defaults>
		<!-- Agent Parameters -->
		<loiter>true</loiter> <!-- Whether a loiter task can be bid on (OPTIONAL) -->
		<survey>true</survey> <!-- Whether a survey task can be bid on (OPTIONAL) -->
		<nominalVelocity>4.5</nominalVelocity> <!-- (OPTIONAL) -->
		<nominalAcceleration>1.0</nominalAcceleration> <!-- (OPTIONAL) -->
		<nominalClimbRate>2.5</nominalClimbRate> <!-- (OPTIONAL) -->
		<fuelCutoff>-200</fuelCutoff> <!-- (OPTIONAL) -->
		<maxFuelVolume>10000.0</maxFuelVolume> <!-- (OPTIONAL) -->
		<idleFuelRate>0.05</idleFuelRate> <!-- (OPTIONAL) -->
		<translationalFuelRate>0.2</translationalFuelRate> <!-- (OPTIONAL) -->
		<climbFuelRate>0.1</climbFuelRate> <!-- (OPTIONAL) -->
		<agentType>UAV_Rotary</agentType> <!-- (OPTIONAL) -->

		<!-- Auction parameters -->
		<timeBetweenBundles>12000</timeBetweenBundles> <!-- Milliseconds an agent must wait before they can build their next bundle. (OPTIONAL) -->
		<timeBetweenDuplicateTaskRequests>10000</timeBetweenDuplicateTaskRequests>  <!-- Milliseconds an agent must wait before they can issue another request for a descriptor they requested but did not receive. (OPTIONAL) -->
		<surveyTasksToGenerate>1</surveyTasksToGenerate> <!-- Number of survey tasks the agent will generate. (OPTIONAL) -->
		<loiterTasksToGenerate>1</loiterTasksToGenerate> <!-- Number of loiter tasks the agent will generate. (OPTIONAL) -->
		<timeBetweenGeneratedTasks>100</timeBetweenGeneratedTasks> <!-- Milliseconds before the next task is generated. (OPTIONAL) -->
		<timeToAssumeConsensus>30000</timeToAssumeConsensus> <!-- Milliseconds after receiving or sending an auction message that consensus is assumed. (OPTIONAL) -->
		<startDelay>10000</startDelay> <!-- Milliseconds before the auction is started. This provides time for all instances to have launched. (OPTIONAL) -->
		<assignmentLimit>100</assignmentLimit> <!-- Max assignments an agent may have. (OPTIONAL) -->
		<bundleSize>3</bundleSize> <!-- Max size of a bid bundle. (OPTIONAL) -->
	</Defaults>
</MonteCarloConfig>
```
</details>


An example configuration file is included which can be used to run a simulation with two agents. The `GenerateLocalSimConfig.py` script can be used to generate a configuration suitable for running a simulation with all agents running on a single machine. Note that when using the `RunMultipleLocalSims.py` script, the configuration files will be automatically generated prior to the run.

## Running a Simulation with a variable number of agents (`RunMultipleLocalSims.py`, recommended)

This script allows multiple simulations to be run for a range of agents, tasks, bundle sizes, and assignment limits. Configuration files are generated automatically. The `-h` flag can be used to get a description on all available options. For instance, to run simulations for 2-10 agents, 2-10 tasks per agent, 1-10 bundle size, 1-10 assignment limit, and default values for the timeouts and number of runs, the command is

**`RunMultipleLocalSims.py --minAgents 2 --maxAgents 10 --minTasks 2 --maxTasks 10 --tasksPerAgent --minBundleSize 1 --maxBundleSize 10 --minAssignmentLimit 1 --maxAssignmentLimit 10`**

An alternative is to pass a list of agents/tasks/bundle sizes/ and assignment limits to run the simulation. For instance, to run all combinations of configurations for a list of: 1, 3 agents; 2, 5 tasks; 1, 3 bundle sizes; and 1, 3 assignment limits, for 10 runs and default timeouts, the command is

**`RunMultipleLocalSims.py -r 10 --tasksList 2 5 --agentsList 1 3 --bundeSizesList 1 3 --assignmentLimitsList 1 3`**

Note that some parameters will be adjusted within the loop.The maximum bundle size is set to not exceed the current assignment limit in the loop.

## Running a Simulation with a variable number of agents (`RunLocalSimVaryingAgents.py`)

This script runs simulations for a varying number of agents.

To generate the configuration files, use

**`GenerateLocalSimConfig.py`**

with the desired options. The `-h` flag will give a list of all available options with a description. For example, to generate configuration files for up to 10 agents, with 5 tasks per agent, and running on Ethernet, and everything else using the default values, the command would be

**`GenerateLocalSimConfig.py -a 10 -t 5 --scaled`**

After the configuration files are created, the script can be invoked as

**`RunLocalSimVaryingAgents.py --minAgents 1 --maxAgents 10 -d config -r 5`**

for example, where `--minAgents` and `--maxAgents` controls the range of agents to consider, `-d` sets the config directory, `-r` sets the number of runs, and the `-f` flag can be passed to set a base to the config filename (ie. `BASE1.xml`).

## Running a Simulation (`MonteCarloSim.py`)

This is the low level script to run a simulation.

One worker must be run per agent in the simulation. A worker can be launched by running

**`MonteCarloSim.py -w -p <Port Number>`**

which will launch and kill MACE instances as needed during the simulation.

After all worker scripts are launched, the master must be run to start the simulation, and can be launched by running

**`python MonteCarloSim.py -m -c <Config filename> -r <Number of runs>`**

Note that the **`ScriptPort`** parameter in the config file for each worker should match the corresponding port number passed to the worker.

## Details

A worker waits for a command from the master process which instructs it on how to generate the appropriate MACE config file, and then launches MACE. The MACE instance in the [Auction_Monte_Carlo_Sim](https://github.com/heronsystems/OpenMACE/tree/Auction_Monte_Carlo_Sim) branch has been modified such that it will exit upon determining that consensus was reached, which is based on whether a sufficient amount of time has passed since the last auction message was sent or received. The workers monitor their corresponding MACE instance, and report back to the master process when the instance quits, or otherwise crashes or times out. The worker also sends the auction log output to the master.

For each run, the master instructs the workers to launch a MACE instance with the specified configuration. It waits for all workers to complete the run before starting the next run. Upon completing all runs, the master extracts the auction running time from the log file output and averages it over all runs. This is done for various other statistics logged by each MACE instance as well. The logs are also saved, in case additional analysis is desirable.