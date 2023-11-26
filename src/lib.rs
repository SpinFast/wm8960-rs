#![no_std]

pub mod common;
pub mod regmap;

#[cfg(feature = "eh02")]
use eh02::blocking::i2c;

pub const WM8960_ADDR: usize = 0x1a;
pub const WM8960_REG_MASK: u16 = 0x01FF;
pub const WM8960_ADDR_MASK: u16 = !WM8960_REG_MASK;

const WM8960_PLL_N_MIN_VALUE: u32 = 6;
const WM8960_PLL_N_MAX_VALUE: u32 = 12;

struct RegIface<I2C> {
    i2c: I2C,
    regs: [u16; 56],
}

impl<I2C, V, E> common::Interface<u8, V> for RegIface<I2C>
where
    I2C: i2c::Write<Error = E>,
    E: core::fmt::Debug,
    V: core::convert::Into<u16> + core::convert::From<u16>,
{
    type Error = E;
    fn read(&mut self, addr: u8) -> Result<V, Self::Error> {
        Ok(self.regs[addr as usize].into())
    }

    fn write(&mut self, addr: u8, value: V) -> Result<(), Self::Error> {
        let v: u16 = value.into();
        let write_buf: [u8; 2] = [addr << 1 | (v >> 8) as u8 & 0x01, (v & 0xFF) as u8];
        self.i2c.write(WM8960_ADDR as u8, &write_buf)?;
        self.regs[addr as usize] = v & WM8960_REG_MASK;
        Ok(())
    }
}

#[derive(Copy, Clone)]
pub enum Route {
    Bypass = 0,
    Playback = 1,
    PlaybackAndRecord = 2,
    Record = 3,
}

#[derive(Copy, Clone)]
pub enum Protocol {
    I2S = 2,
    LeftJustified = 1,
    RightJustified = 0,
    PCMA = 3,
    PCMB = 3 | (1 << 4),
}

#[derive(Copy, Clone)]
pub enum SysclkSource {
    Mclk = 0,
    PLL = 1,
}

#[derive(Copy, Clone)]
pub struct Sysclk {
    pub source: SysclkSource,
    pub freq: u32,
}

#[derive(Copy, Clone)]
pub enum SampleRate {
    SR48000 = 48000,
}

#[derive(Copy, Clone)]
pub enum BitWidth {
    BW16 = 16,
    BW32 = 32,
}

#[derive(Copy, Clone)]
pub struct Format {
    pub mclk_freq: u32,
    pub sample_rate: SampleRate,
    pub bit_width: BitWidth,
}

pub struct Config {
    pub master: bool,
    pub protocol: Protocol,
    pub route: Route,
    pub sysclk: Sysclk,
    pub format: Format,
    pub speaker_en: bool,
}

/// A driver for the WM8960 Codec
pub struct WM8960<I2C> {
    iface: RegIface<I2C>,
}

impl<I2C, E> WM8960<I2C>
where
    I2C: i2c::Write<Error = E>,
    E: core::fmt::Debug,
{
    /// Setup a new WM8960 and initialize it
    pub fn new(i2c: I2C, cfg: &Config) -> Result<Self, E> {
        let mut iface = RegIface {
            i2c,
            regs: regmap::RESET,
        };
        let mut regmap = regmap::Wm8960 {
            iface: &mut iface,
            addr: WM8960_ADDR,
        };
        regmap.reset().write_value(1)?;

        regmap.pwr_mgmt1().modify(|pwr1| {
            pwr1.set_vmidsel(0b01); // 50k
            pwr1.set_vref(true);
            pwr1.set_ainl(true);
            pwr1.set_ainr(true);
            pwr1.set_adcl(true);
            pwr1.set_adcr(true);
            pwr1.set_micb(true);
        })?;

        regmap.pwr_mgmt2().modify(|pwr2| {
            pwr2.set_dacr(true);
            pwr2.set_dacl(true);
            pwr2.set_lout1(true);
            pwr2.set_rout1(true);
            pwr2.set_spkr(true);
            pwr2.set_spkl(true);
        })?;

        regmap.pwr_mgmt3().modify(|pwr3| {
            pwr3.set_romix(true);
            pwr3.set_lomix(true);
            pwr3.set_lmic(true);
            pwr3.set_rmic(true);
        })?;

        //TODO adc/dac same clock, verify
        regmap.aud_intf2().write_value(regmap::AudIntf2(0x40))?;

        drop(regmap);

        let mut s = Self { iface };

        s.set_route(cfg.route)?;
        s.set_protocol(cfg.protocol)?;
        let sysclk = if let SysclkSource::PLL = cfg.sysclk.source {
            s.set_internal_pll_cfg(
                cfg.format.mclk_freq,
                cfg.sysclk.freq,
                cfg.format.sample_rate as u32,
                cfg.format.bit_width as u32,
            )?;
            cfg.sysclk.freq
        } else {
            cfg.format.mclk_freq
        };
        if cfg.master {
            s.set_master_clock(
                sysclk,
                cfg.format.sample_rate as u32,
                cfg.format.bit_width as u32,
            )?;
        }
        s.set_master(cfg.master)?;

        let mut regmap = regmap::Wm8960 {
            iface: &mut s.iface,
            addr: WM8960_ADDR,
        };
        regmap.addtl_ctl1().write_value(regmap::AddtlCtl1(0x0C0))?;
        regmap.addtl_ctl4().write_value(0x40)?;
        regmap.lbypass().write_value(regmap::InBypass(0x0))?;
        regmap.rbypass().write_value(regmap::InBypass(0x0))?;
        regmap.ladc_vol().write_value(regmap::AdacVol(0x1c3))?;
        regmap.radc_vol().write_value(regmap::AdacVol(0x1c3))?;
        regmap.ldac_vol().write_value(regmap::AdacVol(0x1e0))?;
        regmap.rdac_vol().write_value(regmap::AdacVol(0x1e0))?;
        regmap.lout1_vol().write_value(regmap::OutVol(0x16f))?;
        regmap.rout1_vol().write_value(regmap::OutVol(0x16f))?;
        regmap.lspkr_vol().write_value(regmap::OutVol(0x1ff))?;
        regmap.rspkr_vol().write_value(regmap::OutVol(0x1ff))?;
        //regmap.classd_ctl1().write_value(0x0f7)?;
        regmap.adac_ctl1().modify(|ctl| {
            ctl.set_dacmu(false);
        })?;
        //regmap.lin_vol().write_value(0x117)?;
        //regmap.rin_vol().write_value(0x117)?;

        drop(regmap);

        s.set_format(
            sysclk,
            cfg.format.sample_rate as u32,
            cfg.format.bit_width as u32,
        )?;

        Ok(s)
    }

    /// Set the audio routing option
    pub fn set_route(&mut self, route: Route) -> Result<(), E> {
        let mut regmap = regmap::Wm8960 {
            iface: &mut self.iface,
            addr: WM8960_ADDR,
        };

        match route {
            Route::Bypass => {
                // Input3 to output mixer, volume 0dB
                regmap.lout_mix().write_value(regmap::LroutMix(0x80))?;
                regmap.rout_mix().write_value(regmap::LroutMix(0x80))?;
            }
            Route::Playback => {
                // DAC to output mixer, volume 0dB
                regmap.lout_mix().write_value(regmap::LroutMix(0x100))?;
                regmap.rout_mix().write_value(regmap::LroutMix(0x100))?;
                // Power On: DAC, Output Mix, and Line Out
                regmap.pwr_mgmt2().modify(|pwr| {
                    pwr.set_dacr(true);
                    pwr.set_dacr(true);
                })?;
                regmap.pwr_mgmt3().modify(|pwr| {
                    pwr.set_romix(true);
                    pwr.set_lomix(true);
                })?;
                regmap.pwr_mgmt2().modify(|pwr| {
                    pwr.set_lout1(true);
                    pwr.set_rout1(true);
                })?;
            }
            _ => {
                unimplemented!("TODO implement recording routes");
            }
        }
        Ok(())
    }

    /// Set the audio protocol (i2s, pcm, etc) option
    pub fn set_protocol(&mut self, protocol: Protocol) -> Result<(), E> {
        let mut regmap = regmap::Wm8960 {
            iface: &mut self.iface,
            addr: WM8960_ADDR,
        };
        regmap.aud_intf1().modify(|intf| {
            intf.set_format(protocol as u8);
        })?;
        Ok(())
    }

    /// Update PLL settings given input/output/sample rate/bitwidth settings
    /// Note this was ported from the mcux wm8960 driver (like much of this)
    /// and is unverified beyond matching the C code.
    fn set_internal_pll_cfg(
        &mut self,
        mut in_mclk: u32,
        out_mclk: u32,
        _sample_rate: u32,
        _bit_width: u32,
    ) -> Result<(), E> {
        let pll_f2: u32 = out_mclk * 4;
        let mut prescale: u32 = 0;
        let mut div: u32 = 1;
        let mut frac_mode: u32 = 0;

        let mut regmap = regmap::Wm8960 {
            iface: &mut self.iface,
            addr: WM8960_ADDR,
        };
        // disable PLL power
        regmap.pwr_mgmt2().modify(|pwr2| {
            pwr2.set_pll_en(false);
        })?;
        regmap.clocking1().modify(|clk1| {
            clk1.set_clksel(false);
        })?;

        let mut pll_n = pll_f2 / in_mclk;
        if pll_n < WM8960_PLL_N_MIN_VALUE {
            in_mclk >>= 1;
            prescale = 1;
            pll_n = pll_f2 / in_mclk;
            if pll_n < WM8960_PLL_N_MIN_VALUE {
                div = 2;
                pll_n = (pll_f2 * div) / in_mclk;
            }
        }

        if (pll_n < WM8960_PLL_N_MIN_VALUE) || (pll_n > WM8960_PLL_N_MAX_VALUE) {
            panic!("pll_n out of range {}", pll_n);
        }

        let pll_r = (((pll_f2 as u64) * (div as u64) * 1000) / ((in_mclk as u64) / 1000)) as u32;
        let pll_k =
            (((1 << 24) * ((pll_r as u64) - (pll_n as u64) * 1000 * 1000)) / 1000 / 1000) as u32;
        if pll_k != 0 {
            frac_mode = 1;
        }

        regmap
            .pll_n()
            .write_value((frac_mode as u16) << 5 | (prescale as u16) << 4 | pll_n as u16 & 0xF)?;
        regmap.pll_k1().write_value(((pll_k >> 16) & 0xFF) as u16)?;
        regmap.pll_k2().write_value(((pll_k >> 8) & 0xFF) as u16)?;
        regmap.pll_k3().write_value((pll_k & 0xFF) as u16)?;
        regmap.pwr_mgmt2().modify(|pwr2| {
            pwr2.set_pll_en(true);
        })?;
        regmap.clocking1().modify(|clk1| {
            if div == 1 {
                clk1.set_sysclkdiv(0);
            } else {
                clk1.set_sysclkdiv((div << 1) as u8);
            }
            clk1.set_clksel(true);
        })?;

        Ok(())
    }

    /// Update master clock settings
    fn set_master_clock(&mut self, sysclk: u32, sample_rate: u32, bit_width: u32) -> Result<(), E> {
        let clock_divider: u32 = (sysclk * 2) / (sample_rate * bit_width * 2);
        let reg_div = match clock_divider {
            2 => 0,
            3 => 1,
            4 => 2,
            6 => 3,
            8 => 4,
            11 => 5,
            12 => 6,
            16 => 7,
            22 => 8,
            24 => 9,
            32 => 10,
            44 => 11,
            48 => 12,
            _ => panic!("Invalid bit clock divider {}", clock_divider),
        };
        let mut regmap = regmap::Wm8960 {
            iface: &mut self.iface,
            addr: WM8960_ADDR,
        };
        regmap.clocking2().modify(|clk2| {
            clk2.set_bclkdiv(reg_div);
        })?;
        Ok(())
    }

    /// Set the master/slave flag for the codec
    fn set_master(&mut self, master: bool) -> Result<(), E> {
        let mut regmap = regmap::Wm8960 {
            iface: &mut self.iface,
            addr: WM8960_ADDR,
        };
        regmap.aud_intf1().modify(|intf| {
            intf.set_ms(master);
        })?;
        Ok(())
    }

    /// Configure the audio format and dac/adc dividers
    fn set_format(&mut self, sysclk: u32, sample_rate: u32, bit_width: u32) -> Result<(), E> {
        let mut regmap = regmap::Wm8960 {
            iface: &mut self.iface,
            addr: WM8960_ADDR,
        };

        const DIVS: [u32; 7] = [256, 384, 512, 768, 1024, 1408, 1536];

        let mut div_idx: usize = usize::MAX;

        for i in 0..DIVS.len() {
            let rate = sysclk / DIVS[i];
            if rate == sample_rate {
                div_idx = i;
                break;
            }
        }
        if div_idx > DIVS.len() {
            panic!(
                "No dac/adc divider found for sysclk {}, sample_rate {}",
                sysclk, sample_rate
            );
        }

        /* Compute sample rate divider, dac and adc are the same sample rate */
        regmap.clocking1().modify(|clk1| {
            clk1.set_dacdiv(div_idx as u8);
            clk1.set_adcdiv(div_idx as u8);
        })?;

        let wl = match bit_width {
            16 => 0b00,
            20 => 0b01,
            24 => 0b10,
            32 => 0b11,
            _ => panic!("Invalid bit width {}", bit_width),
        };

        regmap.aud_intf1().modify(|intf1| {
            intf1.set_wl(wl);
        })?;

        Ok(())
    }

    /// Update the output volume for out1 (line out)
    pub fn set_out_volume(&mut self, lvol: u16, rvol: u16) -> Result<(), E> {
        let mut regmap = regmap::Wm8960 {
            iface: &mut self.iface,
            addr: WM8960_ADDR,
        };
        regmap.lout1_vol().modify(|_lvol| lvol as u32)?;
        regmap.rout1_vol().modify(|_rvol| rvol as u32)?;
        Ok(())
    }
}
