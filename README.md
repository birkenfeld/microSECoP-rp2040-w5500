# microSECoP demo with Raspberry Pi Pico and Wiznet W5500

Using the https://github.com/SampleEnvironment/microSECoP library.

## How to set up the hardware

* Connect a W5500 extension board to the RPi Pico. Pinout:

  SCLK - pin 10
  MOSI - pin 11
  MISO - pin 12
  CS   - pin 13
  INT  - pin 14
  RST  - pin 15

* Connect the RPi Pico to power (e.g. microUSB) and debug probe (3-pin cable)

* Connect the debug probe to the host

* Connect the W5500 ethernet port to a network that has a DHCP server running

## How to build and run

* Install Rustup: see https://rustup.rs/ (stable toolchain)

* Install the ARM toolchain and flashing tool:

```
$ rustup target add thumbv6m-none-eabi
$ apt install libudev-dev  # example for Debian based systems
$ cargo install probe-rs --features cli
```

* Build and run:

```
$ cargo run --release
```
