# scitos2_mira_utils

## Overview

This package provides `scitos2_mira_utils::MiraAuthority`, a small compiled class that encapsulates a MIRA authority: the checkin/start/checkout lifecycle, channel subscriptions, and service/parameter calls against a configurable MIRA resource (default `/robot/Robot`).

It exists so that [scitos2_core](../scitos2_core) can stay a pure plugin interface with no MIRA dependency: modules *have* a `MiraAuthority` instead of *being* one via a mixin, which also makes the MIRA call helpers testable on their own instead of excluded from coverage.
