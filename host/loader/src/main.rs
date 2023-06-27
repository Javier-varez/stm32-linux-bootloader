use clap::Parser;
use object::read::{File as ElfFile, Object, ObjectSymbol};
use postform_decoder::{print_log, ElfMetadata, SerialDecoder, POSTFORM_VERSION};
use postform_rtt::{
    attach_rtt, configure_rtt_mode, disable_cdebugen, download_firmware, run_core, RttError,
    RttMode,
};
use probe_rs::{DebugProbeError, DebugProbeSelector, Probe};
use probe_rs_gdb_server::GdbInstanceConfiguration;
use std::sync::atomic::{AtomicBool, Ordering};
use std::{
    fs,
    path::PathBuf,
    sync::{Arc, Mutex},
};
use thiserror::Error;

mod loader;

fn print_probes() {
    let probes = Probe::list_all();

    if !probes.is_empty() {
        println!("The following devices were found:");
        probes
            .iter()
            .enumerate()
            .for_each(|(num, link)| println!("[{}]: {:?}", num, link));
    } else {
        println!("No devices were found.");
    }
}

fn print_chips() {
    let registry = probe_rs::config::families().expect("Could not retrieve chip family registry");
    for chip_family in registry {
        println!("{}", chip_family.name);
        println!("    Variants:");
        for variant in chip_family.variants.iter() {
            println!("        {}", variant.name);
        }
    }
}

fn print_version() {
    // version from Cargo.toml e.g. "0.1.4"
    println!("{} {}", env!("CARGO_PKG_NAME"), env!("CARGO_PKG_VERSION"));
    println!("supported Postform version: {}", POSTFORM_VERSION);
}

#[derive(Error, Debug)]
enum ProbeErrors {
    #[error("There are no probes available")]
    NoProbesAvailable,
    #[error(
        "More than one probe available. Select one of them with --probe-selector or --probe-index"
    )]
    MoreThanOneProbeAvailable,
    #[error("Probe index is larger than the number of probes available")]
    IndexOutOfRange,
    #[error("Specific probe error from probe-rs")]
    OpenError(#[from] DebugProbeError),
}

/// Opens a probe with the given index or the first one if there is only one
fn open_probe(index: Option<usize>) -> Result<Probe, ProbeErrors> {
    let probes = Probe::list_all();
    if probes.is_empty() {
        return Err(ProbeErrors::NoProbesAvailable);
    }

    let index = match index {
        Some(index) if (index >= probes.len()) => {
            return Err(ProbeErrors::IndexOutOfRange);
        }
        None if (probes.len() != 1) => {
            return Err(ProbeErrors::MoreThanOneProbeAvailable);
        }
        Some(index) => index,
        None => 0,
    };

    Ok(Probe::open(probes[index].clone())?)
}

#[derive(Debug, Parser)]
struct Opts {
    /// List supported chips and exit.
    #[arg(long = "list-chips")]
    list_chips: bool,

    /// Lists all the connected probes and exit.
    #[arg(long = "list-probes")]
    list_probes: bool,

    /// The chip.
    #[arg(long, env = "POSTFORM_CHIP")]
    chip: Option<String>,

    /// The probe to open. The format is <VID>:<PID>[:<SERIAL>].
    #[arg(long, env = "POSTFORM_PROBE")]
    probe_selector: Option<DebugProbeSelector>,

    /// Index of the probe to open. Can be obtained with --list-probes.
    #[arg(long = "probe-index")]
    probe_index: Option<usize>,

    /// Path to an ELF firmware file.
    #[arg(name = "ELF")]
    elf: Option<PathBuf>,

    /// Attaches to a running target instead of downloading the firmware.
    #[arg(long, short)]
    attach: bool,

    /// Disables FW version check.
    #[arg(long, short = 'd')]
    disable_version_check: bool,

    #[arg(long, short = 'V')]
    version: bool,

    #[arg(long, short)]
    gdb_server: bool,

    #[arg(long = "load-file")]
    load_file: Option<PathBuf>,

    #[arg(long = "load-addr", value_parser=clap_num::maybe_hex::<u32>)]
    load_addr: Option<u32>,

    #[arg(long = "boot")]
    boot: bool,
}

fn run_postform(
    elf_metadata: &ElfMetadata,
    session: Arc<Mutex<probe_rs::Session>>,
    channel: probe_rs_rtt::UpChannel,
) -> color_eyre::eyre::Result<()> {
    let mut buffer = [0u8; 1024];
    let mut decoder = SerialDecoder::new(&elf_metadata);
    loop {
        let count = {
            let mut locked_session = session.lock().unwrap();
            let mut core = locked_session.core(0)?;
            channel.read(&mut core, &mut buffer[..])?
        };

        if count > 0 {
            decoder.feed_and_do(&buffer[..count], |log| {
                print_log(&log);
            });
        }

        // Close application if requested
        if !IS_APP_RUNNING.load(Ordering::Relaxed) {
            log::info!("Stopping postform thread on request");
            break;
        }

        std::thread::sleep(std::time::Duration::from_millis(100));
    }
    Ok(())
}

struct RttTransport {
    session: Arc<Mutex<probe_rs::Session>>,
    down_channel: probe_rs_rtt::DownChannel,
    up_channel: probe_rs_rtt::UpChannel,
}

impl RttTransport {
    pub fn new(
        session: Arc<Mutex<probe_rs::Session>>,
        down_channel: probe_rs_rtt::DownChannel,
        up_channel: probe_rs_rtt::UpChannel,
    ) -> Self {
        Self {
            session,
            down_channel,
            up_channel,
        }
    }
}

#[derive(Debug)]
enum RttTransportError {
    SessionError,
    ProbeRsError(probe_rs::Error),
    ProbeRsRttError(probe_rs_rtt::Error),
}

impl std::fmt::Display for RttTransportError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_fmt(format_args!("{:?}", self))?;
        Ok(())
    }
}

impl From<probe_rs::Error> for RttTransportError {
    fn from(value: probe_rs::Error) -> Self {
        Self::ProbeRsError(value)
    }
}

impl loader::Transport for RttTransport {
    type UnderlyingError = RttTransportError;

    /// Sends data to the device, returning an error if the data could not be sent
    fn send(&mut self, data: &[u8]) -> Result<(), loader::Error<Self::UnderlyingError>> {
        let Ok(mut session) = self.session.lock() else {
            return Err(loader::Error::TransportError(RttTransportError::SessionError));
        };
        let mut core = session
            .core(0)
            .map_err(RttTransportError::ProbeRsError)
            .map_err(loader::Error::TransportError)?;

        let written = self
            .down_channel
            .write(&mut core, data)
            .map_err(RttTransportError::ProbeRsRttError)
            .map_err(loader::Error::TransportError)?;
        // Messages must fit and must be sent at once. this should be guaranteed by the usage of
        // the transport (because all commands are acknowleged, so only 1 will be queued at a time.
        assert_eq!(written, data.len());
        Ok(())
    }

    /// Receives data from the device, returning an error if no data is available
    fn receive(&mut self) -> Result<Vec<u8>, loader::Error<Self::UnderlyingError>> {
        let Ok(mut session) = self.session.lock() else {
            return Err(loader::Error::TransportError(RttTransportError::SessionError));
        };
        let mut core = session
            .core(0)
            .map_err(RttTransportError::ProbeRsError)
            .map_err(loader::Error::TransportError)?;

        let mut data = vec![];
        data.resize(self.up_channel.buffer_size(), 0);
        let new_size = self
            .up_channel
            .read(&mut core, &mut data)
            .map_err(RttTransportError::ProbeRsRttError)
            .map_err(loader::Error::TransportError)?;
        if new_size == 0 {
            return Err(loader::Error::NotReady);
        }

        data.resize(new_size, 0);
        Ok(data)
    }
}

static IS_APP_RUNNING: AtomicBool = AtomicBool::new(true);

fn main() -> color_eyre::eyre::Result<()> {
    color_eyre::install()?;
    env_logger::init();

    let opts = Opts::parse();

    if opts.list_probes {
        print_probes();
        return Ok(());
    }

    if opts.list_chips {
        print_chips();
        return Ok(());
    }

    if opts.version {
        print_version();
        return Ok(());
    }

    let elf_name = opts.elf.unwrap();
    let elf_metadata = ElfMetadata::from_elf_file(&elf_name, opts.disable_version_check)?;

    let probe = if let Some(probe_selector) = opts.probe_selector {
        Probe::open(probe_selector)?
    } else {
        open_probe(opts.probe_index)?
    };

    let Some(chip) = opts.chip else {
        log::error!("No chip selected!");
        return Ok(());
    };

    let session = Arc::new(Mutex::new(
        probe.attach(chip, probe_rs::Permissions::new())?,
    ));

    let elf_contents = fs::read(elf_name.clone())?;
    let elf_file = ElfFile::parse(&elf_contents[..])?;
    let segger_rtt = elf_file
        .symbols()
        .find(|s| s.name().unwrap() == "_SEGGER_RTT")
        .ok_or(RttError::MissingSymbol("_SEGGER_RTT"))?;
    let segger_rtt_addr = segger_rtt.address();

    {
        // ctrlc::set_handler(|| {
        //     IS_APP_RUNNING.store(false, Ordering::Relaxed);
        // })?;
    }
    if !opts.attach {
        download_firmware(&session, &elf_name)?;
    }

    let rtt_channel = 0;
    configure_rtt_mode(
        session.clone(),
        segger_rtt_addr,
        rtt_channel,
        RttMode::Blocking,
    )?;
    let mut rtt = attach_rtt(session.clone(), &elf_file)?;
    if !opts.attach && !opts.gdb_server {
        run_core(session.clone())?;
    }

    if !opts.gdb_server {
        disable_cdebugen(session.clone())?;
    } else {
        let session = session.clone();
        let _ = Some(std::thread::spawn(move || {
            let gdb_connection_string = "127.0.0.1:1337";
            // This next unwrap will always resolve as the connection string is always Some(T).
            log::info!("Firing up GDB stub at {}.", gdb_connection_string);
            let config = {
                let locked_session = session.lock().unwrap();
                GdbInstanceConfiguration::from_session(&locked_session, Some(gdb_connection_string))
            };
            if let Err(e) = probe_rs_gdb_server::run(&session, config.iter()) {
                log::error!("During the execution of GDB an error was encountered:");
                log::error!("{:?}", e);
            }
        }));
    }

    let Some(log_channel) = rtt.up_channels().take(0) else {
        log::error!("Log channel is not available");
        return Ok(());
    };

    let Some(loader_up_channel) = rtt.up_channels().take(1) else {
        log::error!("Loader down channel is not available");
        return Ok(());
    };

    let Some(loader_down_channel) = rtt.down_channels().take(0) else {
        log::error!("Loader down channel is not available");
        return Ok(());
    };

    let mut ld = loader::Loader::new(RttTransport::new(
        session.clone(),
        loader_down_channel,
        loader_up_channel,
    ));

    std::thread::scope(|scope| -> color_eyre::Result<()> {
        let postform_handle =
            scope.spawn(|| run_postform(&elf_metadata, session.clone(), log_channel));

        let loader_handle = scope.spawn(move || -> Result<(), loader::Error<RttTransportError>> {
            if let Some(load_file) = opts.load_file {
                if let Some(load_addr) = opts.load_addr {
                    ld.load_file(&load_file, load_addr)?;
                }
            }

            if opts.boot {
                ld.jump_to_entrypoint()?;

                log::info!("Jump to entrypoint done. Stopping app.");
                // If we jump to entrypoint we should finish execution
                IS_APP_RUNNING.store(false, std::sync::atomic::Ordering::Relaxed);
            }
            Ok(())
        });

        loader_handle.join().unwrap()?;
        postform_handle.join().unwrap()?;
        Ok(())
    })?;

    configure_rtt_mode(session, segger_rtt_addr, rtt_channel, RttMode::NonBlocking)?;
    Ok(())
}
