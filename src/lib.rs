// Copyright 2017, Marius Scholtz <mc.scholtz@gmail.com>
//
// Licensed under the Apache License, Version 2.0 <LICENSE-APACHE or
// http://www.apache.org/license/LICENSE-2.0> or the MIT license
// <LICENSE-MIT or http://opensource.org/licenses/MIT>, at your
// option.  This file may not be copied, modified, or distributed
// except according to those terms.

#![crate_type = "lib"]
#![crate_name = "i2cdev_rfd77402"]

extern crate i2cdev;

use std::sync::atomic::AtomicBool;
use std::sync::atomic::Ordering;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use std::thread;
use std::thread::sleep;

use i2cdev::core::*;
#[allow(unused_imports)]
use i2cdev::linux::{LinuxI2CDevice, LinuxI2CError};

//Register addresses. TODO: change it to constant values instead of enum
/*
enum Register 
{
    IntrCtrlStatReg = 0x00,            //Used by I2C host to config, get status and clear interrupts
    IntrEnReg = 0x02,             //Used to enable individual source of interrupt to I2C host
    CmdReg = 0x04,           //Used to issue commands. Will generate an interrupt
    DevStatusReg = 0x06,    //General Status info
    ResultReg = 0x08,          //Report distance measured
    ResultConfReg = 0x0A,      //Measure of confidence
    CmdConfRegA = 0x0C,      //To configure internal params
    CmdConfRegB = 0x0E,      //To configure internal params
    Host2DevMailReg = 0x10,         //Mailbox. can be written to from host, read by internal mcpu. Can gen interupt based on this
    Dev2HostMailReg = 0x12,         //
    PmuConfReg = 0x14,
    I2cAddrPtrReg = 0x18,      //used in conjunction with i2c dataport
    I2cDataReg = 0x1A,   //indirect access dataport
    I2cInitConfReg = 0x1C,    //misc i2c access config
    DevPwrCtrlReg = 0x1E,    //used by mcpu to control request to pmu
    HwFwConfReg0 = 0x20,         //saturation threshold
    HwFwConfReg1 = 0x22,         //signal acquisition parameters
    HwFwConfReg2 = 0x24,         //signal acquisition parameters
    HwFwConfReg3 = 0x26,         //signal acquisition parameters
    DevChipIdReg = 0x28,     //Controller ID
    PatchMemConfReg = 0x2A,       //used to configure the 2k SRAM as Patch memory or extended memory
}
*/
//errors
#[derive(Debug)]
pub enum Rfd77402Error {
    I2CDeviceError(i2cdev::linux::LinuxI2CError),
    Timeout,
    NoNewData,
    InvalidChipID,
    UnknownState,
}

#[derive(PartialEq,Clone)]
pub enum IoPolicy {
    Poll,
    Interrupt
}

pub struct Measurement {
    pub distance: u16,
    pub error_code: u16,
    pub valid_pixels: u16,
    pub vector_amplitude: u16

}

impl std::convert::From<i2cdev::linux::LinuxI2CError> for Rfd77402Error {
    fn from(error: i2cdev::linux::LinuxI2CError) -> Self
    {
        Rfd77402Error::I2CDeviceError(error)
    }
}

pub struct Rfd77402
{
    //sender: Sender<u8>,
    new_value: Arc<AtomicBool>,
    policy: IoPolicy,
    result_reg: Arc<Mutex<u16>>,     //distance and error code and validity
    confidence_reg: Arc<Mutex<u16>>, //pixles and amplitude
}


pub trait New{
    fn new(&str, IoPolicy) -> Result<Rfd77402,Rfd77402Error>;
}

pub trait Measure {
    fn get_measurement(&self) -> Measurement;
}

impl New for Rfd77402
{
    fn new(path: &str, io: IoPolicy) -> Result<Rfd77402,Rfd77402Error>//need to add poll/interrupt option
    {
        let device = Rfd77402 
        {
            new_value: Arc::new(AtomicBool::new(false)),
            policy: io,
            result_reg: Arc::new(Mutex::new(0)),
            confidence_reg: Arc::new(Mutex::new(0))
        };

        //setup the i2c bus here and go throygh the power sequencing
        let mut i2cdev = LinuxI2CDevice::new(path, 0x4C).expect("did not get i2c device"); //speed??
        
        //initialize the device
        //println!("Initializing device...");
        loop{
            match init(&mut i2cdev, device.policy.clone()) {
                Ok(()) => break,
                _ => {
                    //There was an error resetting. Re-init the i2c bus and try again
                    i2cdev = LinuxI2CDevice::new(path, 0x4C).expect("did not get i2c device");
                    println!("Re-init i2c");
                }
            }  
        }
        //println!("Initialized device...");
        //enter a loop that updats the result_reg and confidence reg every x ms
        let (res_register, conf_register, updated_value) = (device.result_reg.clone(), device.confidence_reg.clone(), device.new_value.clone());
        //this thread need to somehow update the values of result_reg and confidence reg
        thread::spawn(move|| {
            loop {
                //TODO: Check if thread is required to exit use AtomicBool

                //set measure mode & wait for data to be ready to read
                set_measure_mode(&mut i2cdev).expect("set measure mode failed");

                //update the result & confidence register
                {   
                    //scope lock
                    let mut data = res_register.lock().expect("getting lock failed");
                    *data = read_result(&mut i2cdev).expect("read result register failed");
                
                    let mut conf = conf_register.lock().unwrap();
                    *conf = read_confidence(&mut i2cdev).expect("read confidence register failed");

                    updated_value.store(true,Ordering::SeqCst);
                }
            }
        });
        //println!("Debug: thread closed");
        return Ok(device)
    }
}

impl Measure for Rfd77402
{
    fn get_measurement(&self) -> Measurement
    {
        //If there is a new value to read....
        while !self.new_value.compare_and_swap(true,false, Ordering::SeqCst)
        {
            sleep(Duration::from_millis(1));
        }
        
        //there is a new value, return it
        {
            let res = self.result_reg.lock().unwrap();
            let dist = (*res & 0x1FFF) >> 2;
            let err = (*res & 0x6000) >> 13; //TODO: change to enum
            let conf = self.confidence_reg.lock().unwrap();
            let pix = *conf & 0x000F;
            let amp = (*conf & 0x7FF0) >> 4;

            return Measurement 
            {
                distance: dist,
                error_code: err,
                valid_pixels: pix,
                vector_amplitude: amp
            }  
        } 
    }
}

fn init(mut dev: &mut i2cdev::linux::LinuxI2CDevice, io: IoPolicy) -> Result<(),Rfd77402Error>
{
    //get initial status
    let buf = dev.smbus_read_byte_data(0x06)?;
    //println!("buf: {}", buf);
    //If device is in an unknown state. Reset it
    while buf & 0x1F != 0x00 
    {
        match reset(&mut dev) {
            Ok(()) => break,
            Err(err) => return Err(err)
        }
    }

    //println!("starting power on sequnce"); 
    
    //response ok, drive int pad high
    dev.smbus_write_byte_data(0x00,0x04)?;

    //now config interface
    dev.smbus_write_byte_data(0x1C,0x65)?;

    //turn mcpu off
    turn_mcpu_off(&mut dev)?;

    //configure interrupts if required
    if io == IoPolicy::Interrupt
    {
        //config
    }

    //read module ID, save in structure ?????
    /*
    let addr: [u8; 1] = [0xC8];
    dev.write(&addr)?;
    let buf = dev.smbus_read_block_data(0x04)?;
    print!("Module ID: ");  
    for b in buf
    {
        print!("{:X}", b);  
    }
    
    ///read Firmware ID, save somewhere?
    let addr: [u8; 1] = [0xFF];
    dev.write(&addr)?;
    let buf = dev.smbus_read_word_data(0xC0)?;
    print!("\nFirmware ID: {:X}\n", buf);  //doesnt return anything ?
    */

    //turn on
    turn_mcpu_on(&mut dev)?;

    //set some default values
    dev.smbus_write_word_data(0x0C,0xE100)?;
    dev.smbus_write_word_data(0x0E,0x10FF)?;
    dev.smbus_write_word_data(0x20,0x07D0)?;
    dev.smbus_write_word_data(0x22,0x5008)?;
    dev.smbus_write_word_data(0x24,0xA041)?;
    dev.smbus_write_word_data(0x26,0x45D4)?;

    //println!("device is now configured");
    
    set_standby_mode(&mut dev)?;

    //turn off
    turn_mcpu_off(&mut dev).expect("mcpu off falied");

    //write calibration data????

    //turn on
    turn_mcpu_on(&mut dev).expect("mcpu on falied");

    return Ok(())
}

//Done.
fn set_standby_mode(dev: &mut i2cdev::linux::LinuxI2CDevice) -> Result<(),Rfd77402Error>
{
    //println!("putting device in standby mode");
    //set standby state
    dev.smbus_write_byte_data(0x04,0x90)?;
    for _d in 0..10
    {
        let buf = dev.smbus_read_byte_data(0x06)?;
        //println!("response: {:X}", buf);  
        if buf & 0x1F == 0x00 //changed 0x0F to 0x1F
        {
            //println!("device is now in standby mode");
            return Ok(())
        }
        sleep(Duration::from_millis(10)); 
    }
    return Err(Rfd77402Error::Timeout);
}
//Done.
fn turn_mcpu_off(dev: &mut i2cdev::linux::LinuxI2CDevice) -> Result<(),Rfd77402Error>
{
    //println!("turning mcpu off");
    //set init
    dev.smbus_write_byte_data(0x15,0x05)?;

    for _d in 0..10
    {
        //turn off
        dev.smbus_write_byte_data(0x04,0x91)?;

        let buf = dev.smbus_read_byte_data(0x06)?;
        //println!("response: {:X}", buf);  
        if buf & 0x10 == 0x10
        {
            //println!("mcpu is now off");
            return Ok(())
        } 
        sleep(Duration::from_millis(10));
    }
    return Err(Rfd77402Error::Timeout);
}

fn reset(dev: &mut i2cdev::linux::LinuxI2CDevice) -> Result<(),Rfd77402Error>
{
    //println!("resetting...");
    //reset
    loop
    {
        match dev.smbus_write_byte_data(0x04,1<<6) {
            Ok(()) => break,
            Err(err) => return Err(Rfd77402Error::I2CDeviceError(err))
        }
    }
    
    //wait till it resets
    for _d in 0..10
    {
        let buf = dev.smbus_read_byte_data(0x06)?;
        //println!("response: {:X}", buf);  
        if buf & 0x1F == 0x00
        {
            //println!("device reset");
            return Ok(())
        } 
        sleep(Duration::from_millis(10));
    }
    return Err(Rfd77402Error::Timeout);
}

//Done.
fn turn_mcpu_on(dev: &mut i2cdev::linux::LinuxI2CDevice)  -> Result<(),Rfd77402Error>
{
    //println!("turning mcpu on");
    //set init
    dev.smbus_write_byte_data(0x15,0x06)?;

    for _d in 0..10
    {
        //turn on
        dev.smbus_write_byte_data(0x04,0x92)?;

        let buf = dev.smbus_read_word_data(0x06)?;
        //println!("response: {:X}", buf);  
        if buf == 0x01F8
        {
            //println!("mcpu is now online");
            return Ok(())
        } 
        sleep(Duration::from_millis(10));
    }
    return Err(Rfd77402Error::Timeout);
}

//Done. This func waits for data in the results register, then returns
fn set_measure_mode(dev: &mut i2cdev::linux::LinuxI2CDevice)  -> Result<(),Rfd77402Error>
{
    //single measure
    dev.smbus_write_byte_data(0x04,0x81)?;
    //wait till data is ready
    for _d in 0..10
    {
        let buf = dev.smbus_read_byte_data(0x00)?;
        //println!("response: {:X}", buf);  
        if buf & 0x10 == 0x10
        {
            //println!("data is ready to read");
            return Ok(())
        }
        //println!("sleeping..."); 
        sleep(Duration::from_millis(10));
    }
    return Err(Rfd77402Error::Timeout);
}

fn read_result(dev: &mut i2cdev::linux::LinuxI2CDevice) -> Result<u16,Rfd77402Error>
{
    //read result
    let buf = dev.smbus_read_word_data(0x08)?;
    Ok(buf)
}

fn read_confidence(dev: &mut i2cdev::linux::LinuxI2CDevice) -> Result<u16,Rfd77402Error>
{
    //read result
    let buf = dev.smbus_read_word_data(0x0A)?;
    Ok(buf)
}

#[allow(dead_code)]
fn get_chip_id(dev: &mut i2cdev::linux::LinuxI2CDevice) -> Result<u16,Rfd77402Error>
{
    match dev.smbus_read_word_data(0xC0) {
        Ok(id) => return Ok(id),
        Err(err) => return Err(Rfd77402Error::I2CDeviceError(err))
    }
}

#[allow(dead_code)]
fn set_vcsel_peak(dev: &mut i2cdev::linux::LinuxI2CDevice, peak: u8) -> Result<(),Rfd77402Error>
{
    let mut conf = dev.smbus_read_word_data(0xC0)?;
    conf &= 0xF000;
    let tmp = peak as u16;
    conf |= tmp << 12;
    dev.smbus_write_word_data(0xC0,conf)?;
    return Ok(())
}
