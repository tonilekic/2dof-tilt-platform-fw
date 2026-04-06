# Bringup sample

Interactive GPIO shell over USB CDC ACM for PCB validation.

## Build

```bash
west build -b xiao_ble/nrf52840/sense samples/bringup --pristine
west flash
```

## Usage

Connect to the board's USB serial port (115200 8N1) with any terminal
(`tio`, `minicom`, `screen`, `picocom`, PuTTY, etc.).

```
uart:~$ gpio conf gpio0 2 out    # configure P0.02 as output
uart:~$ gpio set  gpio0 2 1      # drive high
uart:~$ gpio set  gpio0 2 0      # drive low
uart:~$ gpio get  gpio0 2        # read current level
```
