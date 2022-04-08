# A Rust-based implementation of Edge-rs for the Raspberry Pi Pico

## Getting started

For more details see the following article on getting started for getting your environment set up
on Mac/Linux:
https://reltech.substack.com/p/getting-started-with-rust-on-a-raspberry

On a Mac, to run minicom: `minicom -D /dev/tty.usbmodem14201 -b 115200`. Note that you'll most likely
need to find the current /dev link assigned to the Pico UART for your particular machine.

## Requirements
- The standard Rust tooling (cargo, rustup) which you can install from https://rustup.rs/

- Toolchain support for the cortex-m0+ processors in the rp2040 (thumbv6m-none-eabi)

- flip-link - this allows you to detect stack-overflows on the first core, which is the only supported target for now.

## Installation of development dependencies
```
rustup target install thumbv6m-none-eabi
cargo install flip-link
cargo install cargo-edit
cargo add panic_halt
```

## Set Up Git Hooks

The ambi_mock_client repository makes use of several Git hooks to ensure that code quality standards are met and consistent. To automatically configure these hooks for your local workspace, you can run the following:
```bash
./scripts/create-git-hooks
```

This will create symlinks to the Git hooks, preserving any hooks that you may have already configured.

## Running

For a debug build
```
cargo run
```
For a release build
```
cargo run --release
```

## Debugging

To debug you can use either `cargo run` which will try and launch a gdb session connecting to a local OpenOCD instance
at localhost:3333, or you can use VSCode and the [cortex-debug extension for VSCode](https://marcelball.ca/projects/cortex-debug/) to connect to the same OpenOCD instance.

For more information on getting set up to use OpenOCD with gdb for flashing and debugging your Pico target device, [see this getting started guide](https://reltech.substack.com/p/getting-started-with-rust-on-a-raspberry).

## License

This project is licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  http://www.apache.org/licenses/LICENSE-2.0)

- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.
