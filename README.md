# v1026-demo-app

> Firmware for the V1026 LPWAN demonstrator.

This project is developed and maintained by 42 Technology (www.42technology.com)

## Features

* Is started by the Nordic 'secure boot-loader' and operates in `insecure` mode
* Connects to Cloudflare's QUIC tech demo at https://quic.tech:4433

## Dependencies

To build embedded programs using this template you'll need:

- Rust 'nightly-2019-09-28-x86_64-unknown-linux-gnu'

- `rust-std` components (pre-compiled `core` crate) for armv8m.main targets.

- GCC for bare-metal ARM (`arm-none-eabi-gcc`), with the newlib C library

- clang

- bindgen

To get these things on Ubuntu 18.04, run:

``` console
$ apt-get update && apt-get install -y curl llvm-dev libclang-dev clang git cmake pkg-config libunwind-dev golang
$ curl -Lq https://developer.arm.com/-/media/Files/downloads/gnu-rm/8-2018q4/gcc-arm-none-eabi-8-2018-q4-major-linux.tar.bz2 | tar xjf - -C ~
$ curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | bash -s - '-y'
$ source $HOME/.cargo/env
$ rustup toolchain install nightly-2019-09-28
$ rustup override set nightly-2019-09-28
$ rustup target add thumbv8m.main-none-eabi
$ export PATH=$PATH:~/gcc-arm-none-eabi-8-2018-q4-major/bin
$ export NEWLIB_PATH=~/gcc-arm-none-eabi-8-2018-q4-major/arm-none-eabi/include
$ cargo install bindgen
```

## Hardware

This application runs on the Actinius Icarus board.

## Building

To build, just run:

```console
$ ./release.sh
```

The outputs are placed in `target/bin/release`.

To flash, load up J-Link Commander, and run:

```
J-Link> usb
J-Link> connect
# select nRF9160
J-Link> h # for halt
J-Link> r # for reset
J-Link> r # for reset again (sometimes it needs two resets)
J-Link> loadfile ~/v1026-demo-app/target/bin/release/<binary name>
J-Link> r # for reset
J-Link> g # for go
```

# Upstream

This project is based on
[cortex-m-quickstart](https://github.com/rust-embedded/cortex-m-quickstart) by
the Rust Embedded team. We are grateful for their work.

# License

This crate is under these licences:

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  http://www.apache.org/licenses/LICENSE-2.0)

- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

