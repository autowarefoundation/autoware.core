# Autoware Node

## Abbreviations

- **ACC:** Autoware Control Center
- **AN:** Autoware Node

## Overview

AN is an `autoware.core` package designed to provide a base class for all future nodes in the
system.
It provides ability to register node to ACC.

## Usage

Check the [autoware_test_node](../../demos/autoware_test_node/README.md) package for an example of how to use `autoware::Node`.

## Design

### Registration

Relevant parameters:

| Parameter                  | Function                                                               |
| -------------------------- | ---------------------------------------------------------------------- |
| `period_timer_register_ms` | The interval at which the AN periodically attempts to register itself. |

Upon startup, AN starts a registration timer and periodically attempts to register itself to ACC via a service call.
This registration timer runs asynchronously to the rest of the node's functionality.

Upon successful registration:

- The registration timer is stopped.
- ACC will return a UUID specific to the AN instance.
