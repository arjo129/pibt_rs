# `hetpibt` - A Rust Implementation of (Heterogenous) PIBT

This crate provides a simple implementation of Priority Inhertiance and Back Tracking (PIBT), a multi-agent pathfinding algorithm. It also includes two experimental implementations of Heterogenous PIBT: one using a reasoning module with winPiBT, and another using Conflict-Based Search (CBS) as a protocol.

## Installation

To use `hetpibt` in your Rust project, add the following to your `Cargo.toml` file:

```toml
[dependencies]
hetpibt = "0.1.0" # Or the latest version
```

## Usage

The core of the library is the `pibt` function, which takes a set of agents and their goals and returns a set of paths. The heterogenous versions provide more advanced features for agents with different capabilities.

Here is a basic example of how to use the library:

```rust
use hetpibt::pibt;

// Define your agents and goals
let agents = vec![...];
let goals = vec![...];

// Run the PIBT algorithm
let paths = pibt(&agents, &goals);

// Use the resulting paths
for (agent, path) in paths {
    println!("Path for agent {:?}: {:?}", agent, path);
}
```

## Running the Examples

This crate includes several examples to visualize the different algorithms. To run the examples, you will need to have a graphical environment set up.

For example, to run the `pibt_visualization` example, use the following command:

```bash
cargo run --example pibt_visualization
```

The other examples can be run in a similar way:

```bash
cargo run --example heterogenous_visuallization
cargo run --example hierarchical_cbs_pibt_visualization
```

## Running the Tests

To run the tests for this crate, use the following command:

```bash
cargo test
```

## License

This project is licensed under the Apache License, Version 2.0. See the [LICENSE](LICENSE) file for details.
