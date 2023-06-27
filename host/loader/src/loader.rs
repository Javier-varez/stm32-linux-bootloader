use std::thread::sleep;

const PAGE_SIZE: usize = 256;
const SUBSECTOR_SIZE: usize = 4096;

pub trait Transport {
    type UnderlyingError;

    /// Sends data to the device, returning an error if the data could not be sent
    fn send(&mut self, data: &[u8]) -> Result<(), Error<Self::UnderlyingError>>;

    /// Receives data from the device, returning an error if no data is available
    fn receive(&mut self) -> Result<Vec<u8>, Error<Self::UnderlyingError>>;
}

#[repr(u8)]
pub enum Cmd {
    EraseSubsector = 0,
    WritePage = 1,
    JumpToEntrypoint = 3,
}

#[derive(Debug)]
pub enum Error<T> {
    TransportError(T),
    FileError(std::io::Error),
    NotReady,
    InvalidResponse,
    InvalidArgs,
    FlashError,
    UnknownCommand,
    UnalignedAddress,
}

#[repr(u8)]
#[derive(enumn::N)]
enum StatusCode {
    OutOfRange = 0,
    Busy = 1,
    InvalidArgs = 2,
    FlashError = 3,
    Handled = 0xa5,
}

impl<T> From<std::io::Error> for Error<T> {
    fn from(value: std::io::Error) -> Self {
        Self::FileError(value)
    }
}

impl<T: std::fmt::Display + std::fmt::Debug> std::fmt::Display for Error<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_fmt(format_args!("{:?}", self))?;
        Ok(())
    }
}

impl<T: std::fmt::Display + std::fmt::Debug> std::error::Error for Error<T> {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        None
    }
}

pub struct Loader<T: Transport> {
    transport: T,
}

impl<T: Transport> Loader<T> {
    pub fn new(transport: T) -> Self {
        Self { transport }
    }

    pub fn jump_to_entrypoint(&mut self) -> Result<(), Error<T::UnderlyingError>> {
        self.send_command(Cmd::JumpToEntrypoint, &[], false)?;
        Ok(())
    }

    pub fn load_file(
        &mut self,
        loadable_file: &std::path::Path,
        load_address: u32,
    ) -> Result<(), Error<T::UnderlyingError>> {
        let contents = std::fs::read(loadable_file)?;
        let file_size = contents.len();

        let num_pages = (file_size + PAGE_SIZE - 1) / PAGE_SIZE;
        let num_subsectors = (file_size + SUBSECTOR_SIZE - 1) / SUBSECTOR_SIZE;

        const SUBSECTOR_MASK: u32 = (SUBSECTOR_SIZE - 1) as u32;

        if (load_address & SUBSECTOR_MASK) != 0 {
            return Err(Error::UnalignedAddress);
        }

        // Erase subsectors
        for i in 0..num_subsectors {
            let addr = load_address + (i * SUBSECTOR_SIZE) as u32;
            self.erase_subsector(addr)?;
        }

        // Program pages
        for i in 0..num_pages {
            let addr = load_address + (i * PAGE_SIZE) as u32;
            self.program_page(addr, contents.iter().skip(i * PAGE_SIZE).take(PAGE_SIZE))?;
        }

        println!("Upload finished");

        Ok(())
    }

    pub fn erase_subsector(&mut self, address: u32) -> Result<(), Error<T::UnderlyingError>> {
        let data = u32::to_le_bytes(address);
        self.send_command(Cmd::EraseSubsector, &data, true)
    }

    pub fn program_page<'a>(
        &mut self,
        address: u32,
        contents: impl Iterator<Item = &'a u8>,
    ) -> Result<(), Error<T::UnderlyingError>> {
        let mut data = vec![];
        for byte in u32::to_le_bytes(address).iter() {
            data.push(*byte);
        }
        data.extend(contents);
        self.send_command(Cmd::WritePage, &data, true)
    }

    pub fn send_command(
        &mut self,
        cmd: Cmd,
        data: &[u8],
        wait_response: bool,
    ) -> Result<(), Error<T::UnderlyingError>> {
        let mut serialized_data = vec![cmd as u8];
        let size = 1 + 4 + data.len() + 4;
        let size_data: [u8; 4] = u32::to_le_bytes(size as u32);
        serialized_data.extend_from_slice(&size_data);
        serialized_data.extend_from_slice(data);
        let crc = crc32c::crc32c(&serialized_data);
        let crc_data: [u8; 4] = u32::to_le_bytes(crc);
        serialized_data.extend_from_slice(&crc_data);

        loop {
            match self.transport.send(&serialized_data) {
                Err(Error::NotReady) => {
                    sleep(std::time::Duration::from_millis(1));
                }
                Ok(()) => break,
                err => return err,
            };
        }

        if !wait_response {
            return Ok(());
        }

        // Now wait for response
        let response = loop {
            match self.transport.receive() {
                Ok(response) => break response,
                Err(Error::NotReady) => {
                    // Just wait
                    sleep(std::time::Duration::from_millis(1));
                }
                Err(e) => return Err(e),
            };
        };
        if response.len() == 0 {
            return Err(Error::InvalidResponse);
        }

        let code = StatusCode::n(response[0]).ok_or(Error::InvalidResponse)?;
        match code {
            StatusCode::OutOfRange => Err(Error::UnknownCommand),
            StatusCode::Busy => Err(Error::NotReady),
            StatusCode::InvalidArgs => Err(Error::InvalidArgs),
            StatusCode::FlashError => Err(Error::FlashError),
            StatusCode::Handled => Ok(()),
        }
    }
}
