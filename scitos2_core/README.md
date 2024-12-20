# scitos2_core

## Overview

This package provides abstract interfaces (virtual base classes) used within the `scitos2` package to communicate with the various Scitos modules. The package contains:
* module (e.g. `battery`, `charger`, `display`, `drive`, ...)
* sink logger: a logger that reads data from MIRA logger and writes it to RCL logger.