# Linux NIC Driver (QEMU Simulation)

This project is a simple **Linux kernel module** that simulates a Network Interface Card (NIC) driver.  
It is built and tested inside an **Ubuntu environment running on QEMU**.

## Features
- Loads a custom NIC driver as a kernel module
- Logs messages on driver load and unload using `dmesg`
- Validated inside a QEMU-based Ubuntu VM

## Tech Stack
- C (Linux Kernel Module)
- QEMU (Virtualization)
- Ubuntu Linux

## Build & Run
1. Build the module:
   ```bash
   make
