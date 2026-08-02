# scitos2_core

## Overview

This package provides the abstract `Module` interface (a virtual base class) used within the `scitos2` package to communicate with the various Scitos modules (e.g. `battery`, `charger`, `display`, `drive`, ...). It has no MIRA dependency: modules that need to talk to MIRA use [scitos2_mira_utils](../scitos2_mira_utils), which they hold rather than inherit from.