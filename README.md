[![Build and Release](https://github.com/MADS-NET/fmu_agent/actions/workflows/release.yml/badge.svg)](https://github.com/MADS-NET/fmu_agent/actions/workflows/release.yml)


# FMU Agent for MADS

This repository Produces a monolithic [MADS](https://github.com/pbosetti/mads) agent that can run a simulation model (a `.fmu` file) in [FMI3](https://fmi-standard.org/docs/3.0/) format. 

Functional Mockup Units (`.fmu` files) can be exported from a large number of simulation softwares like MATLAB Simulink.

> This agent has been updated for MADS v2.4.3
>
> **NOTE:** Only the last format of FMI format, i.e. FMI 3.0, is supported.
> 
> **ABI compatibility**: Different MADS versions may have different [ABI](https://en.wikipedia.org/wiki/Application_binary_interface), so whenever you upgrade MADS, you should also recompile (and re-install) the Python agent plugin.

## Build

As usual for MADS plugins and agents:

```sh
cmake -Bbuild -DMADS_INSTALL_AGENT=ON -DMADS_BUILD_FMU=ON
cmake --build build -j6
sudo cmake --install build # This installs the agent
cmake --build build -t install_fmu # This installs the compiled FMUs
```

A few notes:

- The default install prefix is the MADS folder (as for `mads -p`)
- The option `MADS_INSTALL_AGENT` (default: off) enables installation in the prefix directory of the `mads-fmu` agent, so that you can call it as `mads fmu`
- The option `MADS_BUILD_FMU` (default: off) enables compilation of the FMU units from the source files available in the `models` directory and adds the `install_fmu` target; those are intended for testing purposes


## FMUs

Both **model exchange** and **co-simulation** FMUs are supported. The FMU type is detected automatically and printed at startup.

> **NOTE:** co-simulation FMUs must be exported with a **fixed-step solver**, and the agent period must be a multiple of the solver step size. Variable-step co-simulation FMUs need the MATLAB Runtime installed on the machine, so they won't run standalone.

### Example FMUs

FMUs are actuallt zipped folders that contain some XML file providing model description and a **compiled dynamic library**, which is supposed to be run-time loaded by the software using it.

FMU files are typically exported from simulation software in the **compiled** format, which is only compatible with the working platform. In other words, a FMU file generatd on Intel Windows won't run on a Silicon macOS (and *vice-versa*).

The project `model` directory contains example FMUs in **source format**, so that they can be compiled on the working machine producing a usable `.fmu` file. When you enable the CMake switch `MADS_BUILD_FMU`, the FMUs are compiled; running `cmake --build build --target install_fmu` creates the zipped units and saves them in the `models/fmu` directory.

The provided example FMUs are:

- `DoubleMassSpringDamper`: name says it all. No inputs.
- `linear_axis`: a second-order dynamics linear actuator with a PID controller on position. Inputs: setpoint; outputs: axis position and speed.
- `machine_tool`: a cartesian manipulator made by a combination of three linear axes.


## Execution

Once the agent is compiled and you have a valid FMI3.0 compatible `my_model.fmu` file, you shall chek it for the actual naming of internal variables. Assuming that the agent has been installed (with `cmake --install build`), and that the FMU file is in your current working directory:

```sh
mads fmu my_model.fmu --inspect
```

This is giving you a table of model variables and a suitable stub section for the `mads.ini` file. Copy that into your settings file, update the fields as needed and launch the `mads broker` command, if it is not running already.

> **Note:** the inspect command reports the file name and the **model name**. The latter can be different from the file name, and it is set by the software that generated the FMU. On the MADS network, the agent running the FMU will have the name `fmu_<model name>`, also reported by the inspect command. Inless overridden by the `--name` switch, this is the expected section name in the INI settings file `mads.ini`.

Now you can run the agent:

```sh
mads fmu my_model.fmu
```

Note that by default the model runs at 100 ms period. You can change that either via the `period` value in the INI section, or override that via the `-p|--period` command line option.

The model is routinely evaluated and forward integrated at variable timestep, and the new status is published on the `pub_topic`. At any time, a **new input** can be gived via the MADS network, by publising to the `sub_topic` a JSON message as:

```json
{
  "fmu_input": {
    "in_var1": 123.0,
    "in_var2": 0.123
  }
}
```

To reset the model, send the following:

```json
{"fmu_reset": true}
```

It is suggested to use the [rerunner plugin](https://github.com/mads-net/rerunner_plugin) to visualize the model state in real time.

> **NOTE:** the model is suppsed to run in real time, i.e. the solution time for each timestep shall be shorter than the timestep itself. Relax the tolerances in the INI section if your model cannot keep up with the real time.

## Triggered mode

By default the agent runs in **free-running** mode: the model is integrated forward at every loop tick regardless of whether a new input message has arrived, and the updated state is published immediately after each integration step. The loop period is fixed by the `period` setting (default: 100 ms).

In **trigger mode** the agent instead blocks waiting for an incoming message on the subscribed topic. Only when a message arrives does the agent:

1. Advance the simulation clock and integrate forward by the elapsed wall-clock time since the last received message.
2. Publish the resulting state **before** applying the new inputs contained in the message (so the published state reflects the model response to the *previous* input).
3. Apply the new `fmu_input` values from the received message (ready for the next step).

This makes the simulation step rate slave to the rate of the upstream publisher, which is useful when the FMU is part of a closed-loop pipeline where each controller tick drives one simulation step. The `period` setting is ignored in trigger mode.

Note that a `fmu_reset` message in trigger mode still applies `fmu_input` from the same message (if present), whereas in free-running mode a reset skips input application for that tick.

### Enabling trigger mode

Via the INI settings file:

```toml
[fmu_<model_name>]
trigger_mode = true
```

Or via the command-line option:

```sh
mads fmu my_model.fmu --trigger
# or
mads fmu my_model.fmu -t
```

The command-line flag always takes precedence over the INI setting.

## Jacobian (Model Exchange only)

For **model exchange** FMUs the agent can publish the linearization of the model dynamics $\dot x = f(x, u)$ at the current state, e.g. to feed an Extended Kalman Filter:

```toml
[fmu_<model_name>]
jacobian = true      # or -j / --jacobian on the command line
jacobian_every = 1   # compute and publish every N steps (default 1)
```

The published status then contains a `jacobian` object:

```json
"jacobian": {
  "method": "central_difference",
  "states": ["x1", "x2"],
  "inputs": ["setpoint[0]", "setpoint[1]"],
  "x":    [0.1, 0.0],
  "xdot": [0.0, 0.2],
  "A": [[0.0, 1.0], [-100.0, -2.0]],
  "B": [[0.0, 0.0], [100.0, 0.0]]
}
```

- `A` = ∂f/∂x (nx × nx) and `B` = ∂f/∂u (nx × nu), row *i* being the derivative of state *i*; they are **continuous-time** matrices, evaluated at the state `x` reached at the end of the step, with the inputs applied during that step.
- `states` are the names from `modelDescription.xml` (the variables pointed by the `derivative` attribute of the `<ContinuousStateDerivative>` entries), `inputs` are the `Float64` inputs, with arrays flattened.
- If the FMU declares `providesDirectionalDerivatives="true"`, `fmi3GetDirectionalDerivative` is used (`"method": "directional_derivative"`); otherwise the Jacobian is computed with central finite differences on `fmi3GetContinuousStateDerivatives` (2·(nx + nu) evaluations). Simulink-exported FMUs do not provide directional derivatives (the function is an empty stub), so they always use finite differences.
- The FMU states and inputs are restored after the computation, so the simulation is not affected.
- For co-simulation FMUs the option is ignored with a warning: FMI 3.0 gives no access to the internal states of a co-simulation FMU.

