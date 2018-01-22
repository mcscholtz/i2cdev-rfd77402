Rust i2cdev-rfd77402
===========
[![Version](https://img.shields.io/crates/v/i2cdev_rfd77402.svg)](https://crates.io/crates/i2cdev_rfd77402)
[![License](https://img.shields.io/crates/l/i2cdev.svg)](https://github.com/mcscholtz/i2cdev-rfd77402/blob/master/README.md#license)

Driver for the RFD77402 IoT 3D ToF Sensor Module. Written in Rust for Linux

Example
-----------

```rust,no_run
extern crate i2cdev_rfd77402;

use i2cdev_rfd77402::{Poll, Rfd77402, Measure};

fn main() {
    //open new i2c device
    let dev = Rfd77402::poll("/dev/i2c-1", 10).expect("Creating new device failed..");

    loop
    {
        let dist = dev.get_measurement();
        println!("dis: {0}, amp: {1}, pix: {2}, err: {3}", dist.distance, 
             dist.vector_amplitude, dist.valid_pixels, dist.error_code);
    }
}
```

Features
--------

The following features are implemented and planned for the library:

- [x] Open the device
- [x] Power sequence
- [x] Read data by polling
- [ ] Read data by interrupt

Cross Compiling
---------------

Most likely, the machine you are running on is not your development
machine (although it could be).  In those cases, you will need to
cross-compile.  See https://github.com/japaric/rust-cross for pointers.

License
-------

```
Copyright (c) 2018, Marius Scholtz <mc.scholtz@gmail.com>

Licensed under the Apache License, Version 2.0 <LICENSE-APACHE or
http://www.apache.org/license/LICENSE-2.0> or the MIT license
<LICENSE-MIT or http://opensource.org/licenses/MIT>, at your
option.  This file may not be copied, modified, or distributed
except according to those terms.
```
