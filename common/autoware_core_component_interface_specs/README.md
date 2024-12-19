# autoware_component_interface_specs

This package defines the standardized component interface specifications for Autoware Core, ensuring consistent communication and interaction between various components in the Autoware autonomous driving stack.

## Purpose

The purpose of this package is to:

- Provide a single source of truth for component interface definitions
- Ensure consistency across different implementations
- Facilitate modular development and component interchangeability
- Document the communication protocols between Autoware Core components

## Structure

The package contains interface specifications for various components, including:

- Message definitions
- Service interfaces
- Action interfaces

## Usage
To use these interface specifications in your component:

1. Add this package as a dependency in your package.xml:
```xml
<depend>autoware_core_component_interface_specs</depend>
```

2. Use the provided interfaces in your component code.

```cpp
#include <autoware/core_component_interface_specs/localization.hpp>

// Example: Creating a publisher using the interface specs
rclcpp::Publisher<KinematicState::Message>::SharedPtr publisher_ =
create_publisher<KinematicState::Message>(
KinematicState::name,
KinematicState::get_qos());

// Example: Creating a subscription using the interface specs
auto subscriber_ = create_subscription<KinematicState::Message>(
KinematicState::name,
KinematicState::get_qos(),
std::bind(&YourClass::callback, this, std::placeholders::1));
```
