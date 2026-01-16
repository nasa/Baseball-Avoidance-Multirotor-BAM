# Batch Processing

This README provides a step-by-step guide for using the `example_batch_runme.m`
script to run a batch processing workflow. This is a simple example which
generates parameter files containing random reference trajectories, uses a shell
script to run compiled simulations in parallel, then reads and plots the 
results.

## Overview

The script `example_batch_runme.m` demonstrates programmatically creating a 
series of parameter sets to evaluate with a compiled executable. It carries out
the following steps:

1. Write parameter files
2. Run the simulation using an external executable
3. Read the output files
4. Plot the results

This process is designed to be run in parallel, utilizing multiple CPU cores.

## Expected Output

The expected output for approximately 100 runs, with a simulation duration of 40
seconds each.

MATLAB:
```
Writing pFiles...done   (Execution time: 6.665s)
Running sim...done  (Execution time: 11.780s)
Loading results...done  (Execution time: 10.703s)
Plotting results...done (Execution time: 0.071s)
```

SHELL:
```
** starting the model **
'stdin' successfully changed to binary mode
Num Read = 1, size of param = 2536
** created bam_out001.mat **
...
all done
```

## Workflow

### Step 1: Compile Simulation

Run `buildExe.m` to create the `BAM_app` executable (`BAM_app.exe` on Windows).
See [AutoCodeModels](../../README.md) for more information.

### Step 2: Initialize Data Directory

Create a `_data` directory under the top level folder of the repo. If `_data/`
already exists and contains `.bin` or `.mat` files, these may interfere with
future runs.

### Step 3: Copy Executable and Example Scripts

Copy `BAM_app<.exe>` and `aRunParallel.sh` to `_data/`, and copy
`example_batch_runme.m` to the top level folder.

For example, in Windows, run the following terminal commands from the root
directory:
```
cp AutoCodeModels/examples/BatchProcessing/aRunParallel.sh _data/
cp AutoCode/BAM_app/BAM_app.exe _data/
cp AutoCodeModels/examples/BatchProcessing/example_batch_runme.m .
```
In Unix:
```
cp AutoCodeModels/examples/BatchProcessing/aRunParallel.sh _data/
cp AutoCode/BAM_app/BAM_app _data/
cp AutoCodeModels/examples/BatchProcessing/example_batch_runme.m .
```

### Step 4: Verify System Commands

In the copy of `example_batch_runme.m`, verify that the `shell_cmd` and
`app_name` variables are set appropriately for your system. Note that
`example_batch_runme.m` will create and run a `system` command, using the
supplied `shell_cmd` to run `aRunParallel.sh` (a shell script) from the `_data/`
directory. Therefore, these variables must be set correctly to elicit the
desired  behavior. The command string passed to `system` is printed as a warning for
debugging purposes.

### Step 5: Run Example Script

Navigate to the top level folder containing `setup.m`, `_data/`, and the copy of
`example_batch_runme.m`. Run the copy of `example_batch_runme.m` from there.

## Explanation of the Example Script

`example_batch_runme.m` first tries to check for the presence of required files
and directories, alerting you to any missing components or path errors. However,
it cannot debug the system command, so this may need to be modified for your
system. If any errors are raised, return to steps 2-4 in the previous section.

Next, the script checks for the existence of a `workspace.mat` workspace file.
If this does not exist, it executes the default `setup` script to initialize
data structures needed for creating parameter files without repeatedly running
`setup`. Note that `setup` does *not* need to be run to run the executable; this
is only needed for creating parameter files and processing data.

Next, `example_batch_runme.m` generates a set of unique, randomized Simulink
parameter objects. These are written to binary files named `pFile<XXX>.bin`. The
script sets turbulence intensity, reference trajectory points, and other
simulation variables before writing each file.

Once the parameter files have been created, `example_batch_runme.m` creates a
`system` command (printed as a warning for debugging purposes) which it uses to
run `aRunParallel.sh`. This is a shell script which runs the `BAM_app`
executable for each save parameter file. Simulation outputs are saved to
`bam_out<XXX>.bin`.

Finally, `example_batch_runme.m` loads the saved simulation output files into
a MATLAB workspace struct, and plots the trajectories using the 
`plot_trajectories` function.
