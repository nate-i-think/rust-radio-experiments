extern crate spidev;
use std::time::Duration;
use std::io;
use commands::IRQRegisters;
use spidev::{Spidev, SpidevOptions, SpidevTransfer, SpiModeFlags};

use tokio_gpiod::{Chip, Edge, Event, Input, Lines, Options, Output};

use crate::get_number_from_cmd;

pub struct E22900M30S {
    spi: Spidev,
    busy: Lines<Input>,
    reset: Lines<Output>,
    dio1: Lines<Input>,
    nss: Lines<Output>,
}

pub enum DIOResult {
    Timeout,
    AlreadyHigh,
    RisingEdge,
}

impl E22900M30S {
    /// Creates a new E22900M30S connection. Designed specfically for an RP CM4.
    /// 
    /// One must provide an existing chip, which is quite easy to
    /// instantiate: 
    /// ```
    /// use tokio_gpiod::Chip;
    /// let chip = Chip::new("gpiochip0").await?;
    /// let lora = E22900M30S::open_connection(&mut chip);
    /// ```
    /// 
    /// The function is configured already to source SPI lines
    /// from "/dev/spidev0.0".
    pub async fn open_connection(
        chip: &mut Chip,
    ) -> std::io::Result<Self> {
        // Starting at 4 MHz for testing. Chip is rated for up to
        // 16 MHz, so that can be attempted later.
        const SPI_SPEED_HZ: u32 = 4_000_000;

        // Aquire SPI line.
        let options = SpidevOptions {
            bits_per_word: Some(8),
            max_speed_hz: Some(SPI_SPEED_HZ),
            lsb_first: Some(false),
            spi_mode: Some(SpiModeFlags::SPI_MODE_0),
        };
        let mut spi = Spidev::open("/dev/spidev0.0")?;
        spi.configure(&options)?;

        // Aquire BUSY, RESET, and DIO1.
        //
        // 22: NSS/CS (out)
        // 23: DOI_1  (in)
        // 24: RESET  (out)
        // 25: BUSY   (in)
        let options = Options::output([22])
            .values([true])
            .consumer("LoRa Chip DIO1 line.");
        let nss = chip.request_lines(options).await?;        
        let options = Options::input([23])
            .edge(tokio_gpiod::EdgeDetect::Rising)
            .consumer("LoRa Chip DIO1 line.");
        let dio1 = chip.request_lines(options).await?;
        let options = Options::output([24])
            .values([true])
            .consumer("LoRa Chip RESET line.");
        let reset = chip.request_lines(options).await?;
        let options = Options::input([25])
            .edge(tokio_gpiod::EdgeDetect::Both)
            .consumer("LoRa Chip BUSY line.");
        let busy = chip.request_lines(options).await?;  

        Ok(Self {
            spi: spi,
            busy: busy,
            reset: reset,
            dio1: dio1,
            nss: nss,
        })
    }

    /// Starts a full-duplex SPI exchange with the chip.
    /// 
    /// Provide the `tx_buffer` and the mutable `rx_buffer`.
    /// 
    /// Has a timeout protection, which can be configured via
    /// `timeout`. Leaving it as `None` will remove timeout
    /// protection.
    pub async fn read_write(
        &mut self,
        tx_buffer: &[u8],
        rx_buffer: &mut [u8],
        timeout: Option<Duration>,
    ) -> std::io::Result<()> {
        if let Some(to) = timeout {
            match tokio::time::timeout(to, self.read_write_no_timer(tx_buffer, rx_buffer)).await {
                Ok(result) => result,
                Err(_)     => Err(io::Error::new(
                        io::ErrorKind::TimedOut,
                        "SPI operation timed out. It is likely your packet size was too large or the busy line is stuck high."
                    )
                ),
            }
        } else {
            self.read_write_no_timer(tx_buffer, rx_buffer).await
        }
    }

    /// Returns the value of the BUSY line.
    pub async fn get_busy(&mut self) -> std::io::Result<bool> {
        let busy = self.busy.get_values([false; 1]).await?;
        Ok(*busy.get(0).unwrap())
    }

    /// Sets the value of the RESET line.
    pub async fn set_reset(&mut self, value: bool) -> std::io::Result<()> {
        self.reset.set_values([value]).await?;
        Ok(())
    }

    pub async fn await_dio1_rising(
        &mut self,
        timeout: Option<Duration>
    ) -> std::io::Result<DIOResult> {
        // If DIO1 is high, then no need to look for a rising edge.
        if *self.dio1.get_values([false; 1]).await?.get(0).unwrap() {
            return Ok(DIOResult::AlreadyHigh);
        }

        let await_rising = if let Some(to) = timeout {
            match tokio::time::timeout(to, self.dio1.read_event()).await {
                Ok(e) => e?,
                Err(_) => return Ok(DIOResult::Timeout),
            }
        } else {
            self.dio1.read_event().await?
        };

        if await_rising.edge == Edge::Falling {
            // This *should* be unreachable. I *should* also be diagnosed with OCD.
            return Ok(DIOResult::AlreadyHigh);
        }

        Ok(DIOResult::RisingEdge)
    }

    pub async fn drain_dio1_events(
        &mut self,
    ) -> std::io::Result<()> {
        Ok(while let Ok(evt) = tokio::time::timeout(Duration::ZERO, self.dio1.read_event()).await {
            let _ = evt?;
        })
    }

    pub async fn hardware_reset(&mut self) -> std::io::Result<()> {
        self.set_reset(false).await?;
        const RESET_SIGNAL_TIME: u64 = 100;
        tokio::time::sleep(Duration::from_micros(RESET_SIGNAL_TIME)).await;
        self.set_reset(true).await?;
        //const SLEEP_TO_STBY_RC_TIME_NO_DATA_RETENTION: u64 = 3500;
        //tokio::time::sleep(Duration::from_micros(SLEEP_TO_STBY_RC_TIME_NO_DATA_RETENTION)).await;
        let mut n = 0;
        while !self.get_busy().await.unwrap() {
            n += 1;
            println!("{:?}: R_Busy LOW!", n);
        }
        let mut n = 0;
        while self.get_busy().await.unwrap() {
            n += 1;
            println!("{:?}: R_Busy HIGH!", n);
        }
        println!("Leaving hardware reset fn.");
        Ok(())
    }

    /// Internal function with no timeout watchdog.
    async fn read_write_no_timer(
        &mut self,
        tx_buffer: &[u8],
        rx_buffer: &mut [u8],
    ) -> std::io::Result<()> {
        println!("  [R/W]: FIRST BUSY check: {:?}", self.get_busy().await?);
        println!("  [R/W]: Setting NSS to LOW.\n");
        self.nss.set_values([false]).await?;
        let mut n = 0;
        while self.get_busy().await? {
            println!("  [R/W]: BUSY poll {n} was HIGH");
            tokio::time::sleep(Duration::from_micros(1)).await;
            n += 1;
        }
        println!("\n  [R/W]: Post BUSY check: BUSY: {:?}", self.get_busy().await?);
        
        println!("  [R/W]: Writing SPI data...");
        let mut transfer = SpidevTransfer::read_write(tx_buffer, rx_buffer);

        // tokio::time::sleep(Duration::from_micros(100)).await;

        //! NOTE: This is a blocking transfer obviously. For final async code please
        //!       care to wrap this in a `spawn_blcoking` so it can live happily in 
        //!       async land. :)
        self.spi.transfer(&mut transfer)?;
        self.nss.set_values([true]).await?;

        // Sleeps long enough for the BUSY line to raise,
        // which does slow down operation time but it is
        // only by 600 ns, and prevents issues such as 
        // reading the busy line right after SPI packet
        // transmission and getting delayed information.
        const T_SW: Duration = Duration::from_nanos(600);
        tokio::time::sleep(T_SW).await;

        Ok(())
    }
}

mod commands {
    use bitflags::bitflags;
    use std::time::Duration;
    use crate::lora::*;
    use crate::lora::status::*;

    pub fn parse_status_byte(status_byte: u8) -> status::Status {
        print!("Parsing status byte: {:08b}: ", status_byte);
        let chip_mode = match ((status_byte >> 4) & 0b0000_0111) {
            0x02 => ChipMode::StbyRc,
            0x03 => ChipMode::StbyXoSc,
            0x04 => ChipMode::Fs,
            0x05 => ChipMode::Rx,
            0x06 => ChipMode::Tx,
            _    => ChipMode::Invalid,
        };
        print!("Chip Mode: {:?} ", chip_mode);

        let command_status = match ((status_byte >> 1) & 0b0000_0111) {
            0x02 => CommandStatus::DataAvalibleToHost,
            0x03 => CommandStatus::CommandTimeout,
            0x04 => CommandStatus::CommandProcessingError,
            0x05 => CommandStatus::FailureToExecuteCommand,
            0x06 => CommandStatus::CommandTXDone,
            _    => CommandStatus::Invalid,
        };
        print!("Command Status: {:?}", command_status);
        println!();

        status::Status {
            chip_mode: chip_mode,
            command_status: command_status,
        }
    }

    pub enum StdbyConfig {
        StdbyRc,
        StdbyXOSC,
    }

    pub async fn set_standby(
        lora: &mut E22900M30S,
        config: StdbyConfig,
    ) -> std::io::Result<()> {
        let config_byte = match config {
            StdbyConfig::StdbyRc   => 0,
            StdbyConfig::StdbyXOSC => 1,
        };

        let tx_buffer = [0x80, config_byte];
        let mut rx_buffer = [0; 2];
        lora.read_write(&tx_buffer, &mut rx_buffer, Some(Duration::from_millis(500))).await?;
        let status = parse_status_byte(rx_buffer[1]);
        println!("Status: {:?}", status);
        Ok(())
    }

    #[repr(u8)]
    pub enum PacketType {
        GFSK = 0x00,
        LoRa = 0x01,
        FHSS = 0x03,
    }

    pub async fn set_packet_type(
        lora: &mut E22900M30S,
        packet_type: PacketType,
    ) -> std::io::Result<()> {
        let tx_buffer = [0x8A, packet_type as u8];
        let mut rx_buffer = [0; 2];
        lora.read_write(&tx_buffer, &mut rx_buffer, Some(Duration::from_millis(500))).await?;
        let status = parse_status_byte(rx_buffer[1]);
        println!("Status: {:?}", status);
        Ok(())
    }

    pub async fn set_rf_frequency(
        lora: &mut E22900M30S,
        frequency_hz: u32,
    ) -> std::io::Result<()> {
        // if !(410_000_000..=810_000_000).contains(&frequency_hz) {
        //     return Err(io::Error::new(io::ErrorKind::InvalidInput, "Frequency out of range."));
        // }

        const FREQ_STEP_MULTIPLIER: f64 = 2u64.pow(25) as f64 / 32_000_000.0;
        let rf_freq = (frequency_hz as f64 * FREQ_STEP_MULTIPLIER).round() as u32;
        let bytes = rf_freq.to_be_bytes();
        let tx_buffer = [0x86, bytes[0], bytes[1], bytes[2], bytes[3]];
        let mut rx_buffer = [0; 5];
        lora.read_write(&tx_buffer, &mut rx_buffer, Some(Duration::from_millis(500))).await?;
        Ok(())
    }

    pub enum PowerAmplifierOutput {
        // High Power:
        Dbm22,
        Dbm20,
        Dbm17,
        // Low Power:
        Dbm14,
        Dbm10,
    }

    pub async fn set_power_amplifier_config(
        lora: &mut E22900M30S, 
        output_power: PowerAmplifierOutput,
    ) -> std::io::Result<()> {
        let pa_duty_cycle = match output_power {
            PowerAmplifierOutput::Dbm22 => 0x04,
            PowerAmplifierOutput::Dbm20 => 0x03,
            PowerAmplifierOutput::Dbm17 => 0x02,
            PowerAmplifierOutput::Dbm14 => 0x04,
            PowerAmplifierOutput::Dbm10 => 0x00,
        };

        let hp_max = match output_power {
            PowerAmplifierOutput::Dbm22 => 0x07,
            PowerAmplifierOutput::Dbm20 => 0x05,
            PowerAmplifierOutput::Dbm17 => 0x03,
            PowerAmplifierOutput::Dbm14 => 0x06,
            PowerAmplifierOutput::Dbm10 => 0x03,
        };

        let tx_buffer = [0x95, pa_duty_cycle, hp_max, 0x00, 0x01];
        let mut rx_buffer = [0; 5];
        lora.read_write(&tx_buffer, &mut rx_buffer, Some(Duration::from_millis(500))).await?;
        Ok(())
    }

    #[repr(u8)]
    pub enum RampTime {
        R10u   = 0x00,
        R20u   = 0x01,
        R40u   = 0x02,
        R80u   = 0x03,
        R200u  = 0x04,
        R800u  = 0x05,
        R1700u = 0x06,
        R3400u = 0x07,
    }

    /// Configures the TX params for broadcast.
    /// 
    /// Note that for `power,` the allowed range is: 
    ///  - Low power: -17 to +14 dBm
    ///  - High power: -9 to +22 dBm
    pub async fn set_tx_params(
        lora: &mut E22900M30S,
        power: i8,
        ramp_time: RampTime,
    ) -> std::io::Result<()> {
        let tx_buffer = [0x8E, power as u8, ramp_time as u8];
        let mut rx_buffer = [0; 3];
        lora.read_write(&tx_buffer, &mut rx_buffer, Some(Duration::from_millis(500))).await?;
        Ok(())
    }

    /// Sets the base address in RAM for TX and RX buffer.
    /// 
    /// Typical usage:
    /// ```
    /// set_buffer_base_address(
    ///     &mut lora,
    ///     0x00,
    ///     0x8F, // Halfway
    /// );
    /// ```
    pub async fn set_buffer_base_address(
        lora: &mut E22900M30S,
        tx_base_address: u8,
        rx_base_address: u8,
    ) -> std::io::Result<()> {
        let tx_buffer = [0x8F, tx_base_address, rx_base_address];
        let mut rx_buffer = [0; 3];
        lora.read_write(&tx_buffer, &mut rx_buffer, Some(Duration::from_millis(500))).await?;
        Ok(())
    }

    #[repr(u8)]
    pub enum LoRaSpreadingFactor {
        SF5  = 0x5,
        SF6  = 0x6,
        SF7  = 0x7,
        SF8  = 0x8,
        SF9  = 0x9,
        SF10 = 0xA,
        SF11 = 0xB,
        SF12 = 0xC,
    }

    #[repr(u8)]
    pub enum LoRaBandwith {
        Bw7_8   = 0x00, // 7.8 kHz
        Bw10_42  = 0x08, // 10.4 kHz
        Bw15_63  = 0x01, // 15.6 kHz
        Bw20_83  = 0x09, // 20.8 kHz
        Bw31_25 = 0x02, // 31.25 kHz
        Bw41_67 = 0x0A, // 41.67 kHz
        Bw62_50  = 0x03, // 62.5 kHz
        Bw125   = 0x04, // 125 kHz
        Bw250   = 0x05, // 250 kHz
        Bw500   = 0x06, // 500 kHz
    }

    #[repr(u8)]
    pub enum LoRaCodingRate {
        CR45 = 0x01,
        CR46 = 0x02,
        CR47 = 0x03,
        CR48 = 0x04,
        CR45LI = 0x05,
        CR46LI = 0x06,
        CR48LI = 0x07,
    }

    /// Low Data Rate Optimization (LDRO)
    ///
    /// - Purpose: Improves receiver timing and sync when LoRa symbol durations are very long.
    /// - When to enable: whenever Tₛᵧₘ = 2^SF / BW ≥ 16.38 ms  
    ///   (e.g. SF11@125 kHz, SF12@125 kHz or SF12@250 kHz)
    /// - Effect when ON (`0x01`):
    ///     - Internally reduces bits-per-symbol (SF → SF–2)  
    ///     - Increases robustness to drift and burst noise  
    ///     - Slightly lowers net data throughput
    /// - Effect when OFF (`0x00`):
    ///     - Full SF bits-per-symbol  
    ///     - Maximum throughput on shorter symbols      
    #[repr(u8)]
    pub enum LowDataRateOptimize {
        Off = 0x00,
        On  = 0x01,
    }

    pub async fn set_modulation_params(
        lora: &mut E22900M30S,
        spreading_factor: LoRaSpreadingFactor,
        bandwidth: LoRaBandwith,
        coding_rate: LoRaCodingRate,
        low_data_rate_optimize: LowDataRateOptimize,
    ) -> std::io::Result<()> {
        let tx_buffer = [0x8B, spreading_factor as u8, bandwidth as u8, coding_rate as u8, low_data_rate_optimize as u8];
        let mut rx_buffer = [0; 5];
        lora.read_write(&tx_buffer, &mut rx_buffer, Some(Duration::from_millis(500))).await?;
        Ok(())
    }

    #[repr(u8)]
    pub enum HeaderType {
        ExplicitHeader = 0x00,
        ImplicitHeader = 0x01,
    }

    pub async fn set_packet_params(
        lora: &mut E22900M30S,
        preamble_length: u16,
        header_type: HeaderType,
        payload_len: u8,
        enable_crc: bool,
        invert_iq: bool,
    ) -> std::io::Result<()> {
        let preamble_bytes = preamble_length.to_be_bytes();
        let tx_buffer = [
            0x8C,
            preamble_bytes[0],
            preamble_bytes[1],
            header_type as u8,
            payload_len,
            if enable_crc { 0x01 } else { 0x00 },
            if invert_iq  { 0x01 } else { 0x00 },
        ];
        let mut rx_buffer = [0; 7];
        lora.read_write(&tx_buffer, &mut rx_buffer, Some(Duration::from_millis(500))).await?;
        Ok(())
    }

    bitflags! {
        #[derive(Debug, Clone, Copy, PartialEq, Eq)]
        pub struct IRQRegisters: u16 {
            const TxDone =           0b0000_0000_0000_0001;
            const RxDone =           0b0000_0000_0000_0010;
            const PreambleDetected = 0b0000_0000_0000_0100;
            const SyncWordValid =    0b0000_0000_0000_1000;
            const HeaderValid =      0b0000_0000_0001_0000;
            const HeaderErr =        0b0000_0000_0010_0000;
            const CrcErr =           0b0000_0000_0100_0000;
            const CadDone =          0b0000_0000_1000_0000;
            const CadDetected =      0b0000_0001_0000_0000;
            const Timeout =          0b0000_0010_0000_0000;
            const LrFhssHop =        0b0100_0000_0000_0000;
        }
    }

    pub async fn set_dio_irq_params(
        lora: &mut E22900M30S,
        irq_mask: IRQRegisters,
        dio1_mask: IRQRegisters,
        dio2_mask: IRQRegisters,
        dio3_mask: IRQRegisters,
    ) -> std::io::Result<()> {
        let irq_mask_bytes = irq_mask.bits().to_be_bytes();
        let dio1_mask_bytes = dio1_mask.bits().to_be_bytes();
        let dio2_mask_bytes = dio2_mask.bits().to_be_bytes();
        let dio3_mask_bytes = dio3_mask.bits().to_be_bytes();
        let tx_buffer = [
            0x08, 
            irq_mask_bytes[0], irq_mask_bytes[1],
            dio1_mask_bytes[0], dio1_mask_bytes[1],
            dio2_mask_bytes[0], dio2_mask_bytes[1],
            dio3_mask_bytes[0], dio3_mask_bytes[1]
        ];
        let mut rx_buffer = [0; 9];
        lora.read_write(&tx_buffer, &mut rx_buffer, Some(Duration::from_millis(500))).await?;
        Ok(())
    }

    pub async fn get_irq_status(
        lora: &mut E22900M30S,
    ) -> std::io::Result<IRQRegisters> {
        let tx_buffer = [0x12, 0x00, 0x00, 0x00];
        let mut rx_buffer = [0; 4];
        lora.read_write(&tx_buffer, &mut rx_buffer, Some(Duration::from_millis(500))).await?;
        let bits = u16::from_be_bytes([rx_buffer[2], rx_buffer[3]]);
        let irq_status = IRQRegisters::from_bits(bits);
        match irq_status {
            Some(s) => Ok(s),
            None => {
                Err(
                    std::io::Error::new(
                        io::ErrorKind::InvalidData,
                        "IRQ bitflags not recognized."
                    )
                )
            }
        }
    }

    pub async fn clear_irq_status(
        lora: &mut E22900M30S,
        bits_to_be_cleared: IRQRegisters,
    ) -> std::io::Result<()> {
        let irq_bytes = bits_to_be_cleared.bits().to_be_bytes();
        let tx_buffer = [0x02, irq_bytes[0], irq_bytes[1]];
        let mut rx_buffer = [0; 3];
        lora.read_write(&tx_buffer, &mut rx_buffer, Some(Duration::from_millis(500))).await?;
        Ok(())
    }

    /// Sets the device in transmit mode.
    /// 
    /// Note that the timeout duration is a 23-bit parameter. If left to 0x000000 the 
    /// timeout is disabled and the device stays in TX Mode until the packet is
    /// transmitted and returns to STBY_RC mode upon completion.
    /// 
    /// Timeout duration = Timeout * 15.625 microseconds.
    /// 
    /// Max timeout value is 262 seconds.
    pub async fn set_tx(
        lora: &mut E22900M30S,
        timeout: u32,
    ) -> std::io::Result<()> {
        let timeout_bytes = timeout.to_be_bytes();
        let tx_buffer = [0x83, timeout_bytes[1], timeout_bytes[2], timeout_bytes[3]];
        let mut rx_buffer = [0; 4];
        lora.read_write(&tx_buffer, &mut rx_buffer, Some(Duration::from_millis(500))).await?;
        Ok(())
    }   
}

mod status {
    use crate::lora::*;
    use std::time::Duration;

    #[derive(Debug)]
    pub struct Status {
        pub chip_mode: ChipMode,
        pub command_status: CommandStatus,
    }

    #[derive(Debug)]
    #[repr(u8)]
    pub enum ChipMode {
        Invalid,
        StbyRc = 0x02,
        StbyXoSc = 0x03,
        Fs = 0x04,
        Rx = 0x05,
        Tx = 0x06,
    }

    #[derive(Debug)]
    #[repr(u8)]
    pub enum CommandStatus {
        Invalid,
        DataAvalibleToHost = 0x02,
        CommandTimeout = 0x03,
        CommandProcessingError = 0x04,
        FailureToExecuteCommand = 0x05,
        CommandTXDone = 0x06,
    }

    pub async fn get_status(
        lora: &mut E22900M30S,
    ) -> std::io::Result<Status> {
        // Sends and recieves the GetStatus SPI transaction.
        let tx_buffer = [0xC0, 0x00];
        let mut rx_buffer = [0; 2];
        lora.read_write(&tx_buffer, &mut rx_buffer, Some(Duration::from_millis(500))).await?;
    
        let status_byte = rx_buffer[1];

        println!("[get_status()]: sb = {:08b}", status_byte);
        let chip_mode = match ((status_byte >> 4) & 0b0000_0111) {
            0x02 => ChipMode::StbyRc,
            0x03 => ChipMode::StbyXoSc,
            0x04 => ChipMode::Fs,
            0x05 => ChipMode::Rx,
            0x06 => ChipMode::Tx,
            _    => ChipMode::Invalid,
        };

        let command_status = match ((status_byte >> 1) & 0b0000_0111) {
            0x02 => CommandStatus::DataAvalibleToHost,
            0x03 => CommandStatus::CommandTimeout,
            0x04 => CommandStatus::CommandProcessingError,
            0x05 => CommandStatus::FailureToExecuteCommand,
            0x06 => CommandStatus::CommandTXDone,
            _    => CommandStatus::Invalid,
        };
        
        Ok(Status{
            chip_mode: chip_mode,
            command_status: command_status,
        })
    }
}

mod register_and_buffer {
    use crate::lora::*;
    use std::time::Duration;

    /// Writes data to the chip RAM buffer. Typically the offset value used is `0x00`.
    pub async fn write_buffer(
        lora: &mut E22900M30S,
        offset: u8,
        payload: &[u8],
    ) -> std::io::Result<()> {
        let mut tx_buffer = Vec::with_capacity(1 + 1 + payload.len());
        tx_buffer.push(0x0E);
        tx_buffer.push(offset);
        tx_buffer.extend_from_slice(payload);
        let mut rx_buffer = vec![0; tx_buffer.len()];
        lora.read_write(&tx_buffer, &mut rx_buffer, Some(Duration::from_millis(500))).await?;
        Ok(())
    }

    pub async fn write_register(
        lora: &mut E22900M30S,
        address: u16,
        value: &[u8],
    ) -> std::io::Result<()> {
        let mut tx_buffer = Vec::with_capacity(1 + 2 + value.len());
        let address_bytes = address.to_be_bytes();
        tx_buffer.push(0x0D);
        tx_buffer.push(address_bytes[0]);
        tx_buffer.push(address_bytes[1]);
        tx_buffer.extend_from_slice(value);
        let mut rx_buffer = vec![0; tx_buffer.len()];
        lora.read_write(&tx_buffer, &mut rx_buffer, Some(Duration::from_millis(500))).await?;
        Ok(())
    }
}

pub fn print_bytes(arr: &[u8]) {
    print!("[ ");
    for b in arr {
        print!("{:08b}/{:02X}, ", *b, *b);
    }
    println!("]");
}

pub async fn test() {
    println!("\n<== RPI INIT ==================>");
    println!("Creating Chip...");
    let mut chip = Chip::new("gpiochip0").await.unwrap();
    println!("Opening LoRa device...");
    let mut lora = E22900M30S::open_connection(&mut chip).await.unwrap();

    println!("\n<== CHIP RESET ================>");
    println!("Reset? = 0: ");
    if get_number_from_cmd().unwrap() == 0 {
        println!("Resetting...");
        lora.hardware_reset().await.unwrap();
        println!("Reset complete.");
    }

    println!("\n<== SetSleep() ================>");
    let tx_buffer = [0x84, 0b0000_0000];
    let mut rx_buffer = [0; 2];
    print!("TX: "); print_bytes(&tx_buffer);
    lora.read_write_no_timer(&tx_buffer, &mut rx_buffer).await.unwrap();
    print!("RX: "); print_bytes(&rx_buffer);
    tokio::time::sleep(Duration::from_micros(600));

    println!("BUSY: {:?}", lora.get_busy().await.unwrap());
    println!("Waiting...");
    tokio::time::sleep(Duration::from_millis(600));
    println!("BUSY: {:?}", lora.get_busy().await.unwrap());

    println!("\n<== GetStatus() ===============>");
    let tx_buffer = [0xC0, 0b0000_0000];
    let mut rx_buffer = [0; 2];
    print!("TX: "); print_bytes(&tx_buffer);
    lora.read_write_no_timer(&tx_buffer, &mut rx_buffer).await.unwrap();
    print!("RX: "); print_bytes(&rx_buffer);

    println!("\nDone!");
}

pub async fn test_2() {
    println!("Creating Chip...");
    let mut chip = Chip::new("gpiochip0").await.unwrap();
    println!("Opening LoRa device...");
    let mut lora = E22900M30S::open_connection(&mut chip).await.unwrap();

    println!("Reset? = 0: ");
    if get_number_from_cmd().unwrap() == 0 {
        println!("Resetting...");
        lora.hardware_reset().await.unwrap();
        println!("Reset complete.");
    }

    println!("\nStarting basic packet transmission...");

    println!("Setting device in StandByRC mode...");
    commands::set_standby(&mut lora, commands::StdbyConfig::StdbyRc).await.unwrap();
    while lora.get_busy().await.unwrap() {
        tokio::time::sleep(Duration::from_micros(100)).await;
        println!("here");
    }
    println!("Status: {:?}", status::get_status(&mut lora).await.unwrap());

    println!("Setting Packet Type to LoRa...");
    commands::set_packet_type(&mut lora, commands::PacketType::LoRa).await.unwrap();
    println!("Status: {:?}", status::get_status(&mut lora).await.unwrap());

    println!("Setting RF Frequency to 915 MHz...");
    commands::set_rf_frequency(&mut lora, 915_000_000).await.unwrap();
    println!("Status: {:?}", status::get_status(&mut lora).await.unwrap());

    println!("Setting Power Amplifer Configuration to 10 dBm...");
    commands::set_power_amplifier_config(&mut lora, commands::PowerAmplifierOutput::Dbm10).await.unwrap();
    println!("Status: {:?}", status::get_status(&mut lora).await.unwrap());

    println!("Setting TX params...");
    commands::set_tx_params(&mut lora, 10, commands::RampTime::R200u).await.unwrap();
    println!("Status: {:?}", status::get_status(&mut lora).await.unwrap());

    println!("Setting buffer base address...");
    commands::set_buffer_base_address(&mut lora, 0x00, 0x8F).await.unwrap();
    println!("Status: {:?}", status::get_status(&mut lora).await.unwrap());

    println!("Writing payload to buffer...");
    register_and_buffer::write_buffer(&mut lora, 0x00, &[0x01, 0x02, 0x03]).await.unwrap();
    println!("Status: {:?}", status::get_status(&mut lora).await.unwrap());

    println!("Setting modulation parameters...");
    commands::set_modulation_params(
        &mut lora, 
        commands::LoRaSpreadingFactor::SF7, 
        commands::LoRaBandwith::Bw125, 
        commands::LoRaCodingRate::CR45, 
        commands::LowDataRateOptimize::Off,
    ).await.unwrap();
    println!("Status: {:?}", status::get_status(&mut lora).await.unwrap());

    println!("Set packet params...");
    commands::set_packet_params(&mut lora, 8, commands::HeaderType::ExplicitHeader, 3, false, false).await.unwrap();
    println!("Status: {:?}", status::get_status(&mut lora).await.unwrap());

    println!("Configuring DIO and IRQ...");
    let irq_params = IRQRegisters::TxDone | IRQRegisters::Timeout;
    commands::set_dio_irq_params(&mut lora, 
        irq_params, 
        irq_params, 
        IRQRegisters::empty(), 
        IRQRegisters::empty(),
    ).await.unwrap();
    println!("Status: {:?}", status::get_status(&mut lora).await.unwrap());

    println!("Writing Sync Word values...");
    register_and_buffer::write_register(&mut lora, 0x0740, &[0x34]).await.unwrap();
    register_and_buffer::write_register(&mut lora, 0x0741, &[0x44]).await.unwrap();
    println!("Status: {:?}", status::get_status(&mut lora).await.unwrap());

    println!("Draining DIO1...");
    lora.drain_dio1_events().await.unwrap();
    println!("Status: {:?}", status::get_status(&mut lora).await.unwrap());

    println!("Settings successfully written.");

    println!("\nStarting transmission via SetTx()...");
    commands::set_tx(&mut lora, 60000).await.unwrap();
    println!("Transmission Send Signal Sent.");
    println!("Status: {:?}", status::get_status(&mut lora).await.unwrap());

    println!("\nWaiting for confirmation of signal transmission...");
    match lora.await_dio1_rising(Some(Duration::from_millis(1000))).await.unwrap() {
        DIOResult::Timeout => {
            println!("Timed out waiting for DIO1 to rise. Exiting...");
        },
        DIOResult::AlreadyHigh => {
            println!("DIO1 was already high. Exiting...");
        }
        DIOResult::RisingEdge => {
            println!("DIO1 Rising Edge recieved!");
        }
    }
    println!("Status: {:?}", status::get_status(&mut lora).await.unwrap());

    println!("Requesting IRQ information...");
    let status = commands::get_irq_status(&mut lora).await.unwrap();
    println!("IRQ Reason: {:?}", status);

    println!("Resetting IRQs...");
    commands::clear_irq_status(&mut lora, IRQRegisters::all()).await.unwrap();
    println!("Status: {:?}", status::get_status(&mut lora).await.unwrap());

    println!("\nAll done!");
}