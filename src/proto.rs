// -----------------------------------------------------------------------------
// Rust SECoP playground
//
// This program is free software; you can redistribute it and/or modify it under
// the terms of the GNU General Public License as published by the Free Software
// Foundation; either version 2 of the License, or (at your option) any later
// version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
// details.
//
// You should have received a copy of the GNU General Public License along with
// this program; if not, write to the Free Software Foundation, Inc.,
// 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
// Module authors:
//   Georg Brandl <g.brandl@fz-juelich.de>
//   Enrico Faulhaber <enrico.faulhaber@frm2.tum.de>
//
// -----------------------------------------------------------------------------
//
//! SECoP data type / data info definitions.

use core::marker::PhantomData;
use core::fmt::{Display, Formatter};
use core::fmt::Write as _;
use w5500_dhcp::hl::io::Write;
use embedded_hal::adc::OneShot;
use rp_pico::hal::adc::{Adc, TempSense};


pub type Result<'a, T> = core::result::Result<T, Error<'a>>;

#[derive(Debug)]
pub enum ErrorKind {
    // Internal
    Config,
    Programming,
    Parsing,
    // API defined
    Protocol,
    NoSuchModule,
    NoSuchParameter,
    NoSuchCommand,
    CommandFailed,
    CommandRunning,
    ReadOnly,
    BadValue,
    CommunicationFailed,
    Timeout,       // ATM also C.F.
    HardwareError, // ATM also C.F.
    IsBusy,
    IsError,
    Disabled,
}

#[derive(Debug)]
pub struct Error<'a> {
    kind: ErrorKind,
    message: &'static str,
    #[allow(unused)]
    original: Option<&'a str>,
}

impl<'a> Error<'a> {
    pub fn new(kind: ErrorKind, message: &'static str, original: &'a str) -> Self {
        Self { kind, message, original: Some(original) }
    }

    pub fn into_msg(self, req: &'a str, spec: &'a str) -> Msg<'a> {
        Msg::ErrMsg {
            request: req,
            specifier: spec,
            class: self.wire(),
            message: self.message,
        }
    }

    fn wire(&self) -> &'static str {
        use self::ErrorKind::*;
        match self.kind {
            Config | Programming | Parsing => "InternalError",
            Protocol => "ProtocolError",
            NoSuchModule => "NoSuchModule",
            NoSuchParameter => "NoSuchParameter",
            NoSuchCommand => "NoSuchCommand",
            CommandFailed => "CommandFailed",
            CommandRunning => "CommandRunning",
            ReadOnly => "ReadOnly",
            BadValue => "BadValue",
            CommunicationFailed => "CommunicationFailed",
            Timeout => "CommunicationFailed",
            HardwareError => "CommunicationFailed",
            IsBusy => "IsBusy",
            IsError => "IsError",
            Disabled => "Disabled",
        }
    }

    // Quick construction.

    pub fn bad_value(message: &'static str) -> Self {
        Self { kind: ErrorKind::BadValue, message, original: None }
    }

    pub fn config(message: &'static str) -> Self {
        Self { kind: ErrorKind::Config, message, original: None }
    }

    pub fn protocol(message: &'static str) -> Self {
        Self { kind: ErrorKind::Protocol, message, original: None }
    }

    pub fn no_module() -> Self {
        Self { kind: ErrorKind::NoSuchModule, message: "", original: None }
    }

    pub fn no_param() -> Self {
        Self { kind: ErrorKind::NoSuchParameter, message: "", original: None }
    }

    pub fn no_command() -> Self {
        Self { kind: ErrorKind::NoSuchCommand, message: "", original: None }
    }

    pub fn comm_failed(message: &'static str) -> Self {
        Self { kind: ErrorKind::CommunicationFailed, message, original: None }
    }
}


pub const IDENT_REPLY: &str = "ISSE&SINE2020,SECoP,V2019-09-16,v1.0\n";

/// Enum that represents any message that can be sent over the network in the
/// protocol, and some others that are only used internally.
#[derive(Debug, Clone)]
pub enum Msg<'a> {
    /// identify request
    Idn,
    /// identify reply
    IdnReply,
    /// help request
    Help,
    /// help reply
    Helping { message: &'a str },
    /// description request
    Describe,
    /// description reply
    Describing { id: &'a str, structure: &'a str },
    /// event enable request
    Activate { module: &'a str },
    /// event enable reply
    Active { module: &'a str },
    /// event disable request
    Deactivate { module: &'a str },
    /// event disable reply
    Inactive { module: &'a str },
    /// command execution request
    Do { module: &'a str, command: &'a str, arg: &'a str },
    /// command result
    Done { module: &'a str, command: &'a str, value: &'a str },
    /// change request
    Change { module: &'a str, param: &'a str, value: &'a str },
    /// change result
    Changed { module: &'a str, param: &'a str, value: &'a str },
    /// read request
    Read { module: &'a str, param: &'a str },
    /// heartbeat request
    Ping { token: &'a str },
    /// heartbeat reply
    Pong { token: &'a str, value: &'a str },
    /// error reply
    ErrMsg { request: &'a str, specifier: &'a str, class: &'a str, message: &'a str },
    /// update event
    Update { module: &'a str, param: &'a str, value: &'a str },

    // /// not a protocol message, but a collection of initial updates
    // InitUpdates { module: &'a str, updates: Vec<Msg> },
    // /// not a protocol message, indicates the connection is done
    // Quit,
}

/// An incoming message that carries around the originating line from the
/// client.  We need this line for the error message if something goes wrong.
#[derive(Clone)]
pub struct IncomingMsg<'a>(pub &'a str, pub Msg<'a>);

use self::Msg::*;

mod wire {
    pub const IDN: &str = "*IDN?";
    pub const DESCRIBE: &str = "describe";
    pub const DESCRIBING: &str = "describing";
    pub const ACTIVATE: &str = "activate";
    pub const ACTIVE: &str = "active";
    pub const DEACTIVATE: &str = "deactivate";
    pub const INACTIVE: &str = "inactive";
    pub const PING: &str = "ping";
    pub const PONG: &str = "pong";
    pub const ERROR: &str = "error";
    pub const DO: &str = "do";
    pub const DONE: &str = "done";
    pub const CHANGE: &str = "change";
    pub const CHANGED: &str = "changed";
    pub const READ: &str = "read";
    pub const UPDATE: &str = "update";
    pub const HELP: &str = "help";
    pub const HELPING: &str = "helping";
}

impl Msg<'_> {
    /// Parse a string slice containing a message.
    ///
    /// This matches a regular expression, and then creates a `Msg` if successful.
    pub fn parse(msg: &[u8]) -> core::result::Result<IncomingMsg, Msg> {
        match core::str::from_utf8(msg) {
            Err(_) => Err(Error::protocol("invalid UTF8").into_msg("", "")),
            Ok(msg) => match Self::parse_inner(&msg) {
                Ok(v) => Ok(IncomingMsg(msg, v)),
                Err(e) => Err(e.into_msg("", "")),
            }
        }
    }

    fn parse_inner(msg: &str) -> Result<Msg> {
        let mut parts = msg.trim().splitn(3, " ");

        let action = parts.next().ok_or(Error::protocol("missing action"))?;
        let specifier = parts.next().unwrap_or_default();
        let data = parts.next().unwrap_or_default();
        let mut spec_split = specifier.splitn(2, ':').map(Into::into);
        let module = spec_split.next().expect("cannot be absent");
        let mut param = || spec_split.next().ok_or(Error::protocol("missing parameter"));

        let parsed = match action {
            wire::READ =>       Read { module, param: param()? },
            wire::CHANGE =>     Change { module, param: param()?, value: data },
            wire::DO =>         Do { module, command: param()?, arg: data },
            wire::DESCRIBE =>   Describe,
            wire::ACTIVATE =>   Activate { module },
            wire::DEACTIVATE => Deactivate { module },
            wire::PING =>       Ping { token: specifier.into() },
            wire::IDN =>        Idn,
            wire::UPDATE =>     Update { module, param: param()?, value: data },
            wire::HELP =>       Help,
            _ => return Err(Error::protocol("message type not supported"))
        };

        Ok(parsed)
    }

    fn write<E, T: Write<E>>(&self, w: &mut FmtWrap<'_, E, T>) -> core::fmt::Result {
        match self {
            Update { module, param, value } =>
                write!(w, "{} {}:{} [{},{{}}]\n", wire::UPDATE, module, param, value),
            Changed { module, param, value } =>
                write!(w, "{} {}:{} [{},{{}}]\n", wire::CHANGED, module, param, value),
            Done { module, command, value } =>
                write!(w, "{} {}:{} [{},{{}}]\n", wire::DONE, module, command, value),
            Describing { id, structure } =>
                write!(w, "{} {} {}\n", wire::DESCRIBING, id, structure),
            Active { module } =>
                if module.is_empty() { write!(w, "{}\n", wire::ACTIVE) }
                else { write!(w, "{} {}\n", wire::ACTIVE, module) },
            Inactive { module } =>
                if module.is_empty() { write!(w, "{}\n", wire::INACTIVE) }
                else { write!(w, "{} {}\n", wire::INACTIVE, module) },
            Pong { token, value } =>
                write!(w, "{} {} [{},{{}}]\n", wire::PONG, token, value),
            Idn => w.write_str(wire::IDN),
            IdnReply => w.write_str(IDENT_REPLY),
            Read { module, param } =>
                write!(w, "{} {}:{}\n", wire::READ, module, param),
            Change { module, param, value } =>
                write!(w, "{} {}:{} {}\n", wire::CHANGE, module, param, value),
            Do { module, command, arg } =>
                write!(w, "{} {}:{} {}\n", wire::DO, module, command, arg),
            Describe => w.write_str(wire::DESCRIBE),
            Activate { module } =>
                if module.is_empty() { w.write_str(wire::ACTIVATE) }
                else { write!(w, "{} {}\n", wire::ACTIVATE, module) },
            Deactivate { module } =>
                if module.is_empty() { w.write_str(wire::DEACTIVATE) }
                else { write!(w, "{} {}\n", wire::DEACTIVATE, module) },
            Ping { token } =>
                if token.is_empty() { w.write_str(wire::PING) }
                else { write!(w, "{} {}\n", wire::PING, token) },
            Help => write!(w, "{}\n", wire::HELP),
            Helping { message } => write!(w, "{}  {}\n", wire::HELPING, JsonStr(message)),
            ErrMsg { request, specifier, class, message } =>
                write!(w, "{}_{} {} [{},{},{{}}]\n",
                       wire::ERROR, request, specifier, JsonStr(class), JsonStr(message)),
        }
    }
}

struct JsonStr<'a>(&'a str);

impl Display for JsonStr<'_> {
    fn fmt(&self, f: &mut Formatter) -> core::fmt::Result {
        f.write_char('"')?;
        for c in self.0.chars() {
            match c {
                '"' => f.write_str("\\\"")?,
                '\\' => f.write_str("\\\\")?,
                '\n' => f.write_str("\\n")?,
                '\r' => f.write_str("\\r")?,
                '\t' => f.write_str("\\t")?,
                c if (c as u32) < 0x20 => write!(f, "\\u{:04x}", c as u32)?,
                c => f.write_char(c)?,
            }
        }
        f.write_char('"')
    }
}

struct FmtWrap<'a, E, T: Write<E>>(&'a mut T, PhantomData<E>);

impl<'a, E, T: Write<E>> core::fmt::Write for FmtWrap<'a, E, T> {
    fn write_str(&mut self, msg: &str) -> core::result::Result<(), core::fmt::Error> {
        self.0.write(msg.as_bytes()).map_err(|_| core::fmt::Error).map(|_| ())
    }
}

impl<'a> IncomingMsg<'a> {
    pub fn bare(msg: Msg<'a>) -> Self {
        IncomingMsg("", msg)
    }
}

impl Display for IncomingMsg<'_> {
    fn fmt(&self, f: &mut Formatter) -> core::fmt::Result {
        write!(f, "{}", self.0)
    }
}

impl Display for Error<'_> {
    fn fmt(&self, f: &mut Formatter) -> core::fmt::Result {
        write!(f, "{}: {}", self.wire(), self.message)
    }
}

const DESC: &str = r##"{"modules": {"temp": {"accessibles": {"value": {"description": "current temperature", "datainfo": {"unit": "C", "type": "double"}, "readonly": true}, "status": {"description": "current status of the module", "datainfo": {"type": "tuple", "members": [{"type": "enum", "members": {"IDLE": 100, "ERROR": 400}}, {"type": "string"}]}, "readonly": true}, "pollinterval": {"description": "default poll interval", "datainfo": {"unit": "s", "min": 0.1, "max": 120.0, "type": "double"}, "readonly": false}}, "description": "temperature from on-chip sensor", "implementation": "rust", "interface_classes": ["Redable"], "features": []}}, "equipment_id": "demo.rp2040", "firmware": "rust-mini-secop", "version": "2021.02", "description": "Basic demo server for frappy", "interface": "tcp://10767"}"##;


pub struct SecNode {
    active: bool,
    pollinterval: f64,
    adc: Adc,
    sensor: TempSense,
}


impl SecNode {
    pub fn new(adc: Adc, sensor: TempSense) -> Self {
        Self {
            active: false,
            pollinterval: 5.0,
            adc,
            sensor,
        }
    }

    pub fn process<E>(&mut self, input: &[u8], writer: &mut impl Write<E>) {
        let mut writer = FmtWrap(writer, PhantomData);
        let reply = match Msg::parse(input) {
            Ok(input_msg) => match input_msg.1 {
                Msg::Idn => Msg::IdnReply,
                Msg::Ping { token } => Msg::Pong { token, value: "null" },
                Msg::Change { module, param, value } => return self.change(module, param, value, writer),
                Msg::Read { module, param } => return self.read(module, param, writer),
                Msg::Do { module, command, arg } => return self.do_(module, command, arg, writer),
                Msg::Help => Msg::Helping { message: "" },
                Msg::Activate { module } => { self.active = true; self.poll_inner(&mut writer); Msg::Active { module } },
                Msg::Deactivate { module } => { self.active = false; Msg::Inactive { module } },
                Msg::Describe => return self.describe(writer),
                _ => Error::protocol("not implemented").into_msg("", ""),
            },
            Err(e) => e,
        };
        let _ = reply.write(&mut writer);
    }

    fn get_temp(&mut self) -> f64 {
        let refv = 3.3;
        let adc_value: u16 = self.adc.read(&mut self.sensor).unwrap();
        let vbe: f64 = f64::from(adc_value) * refv / 4096.0;
        27f64 - (vbe - 0.706) / 0.001721
    }

    pub fn poll<E>(&mut self, writer: &mut impl Write<E>) {
        self.poll_inner(&mut FmtWrap(writer, PhantomData));
    }

    fn poll_inner<E, T: Write<E>>(&mut self, writer: &mut FmtWrap<'_, E, T>) {
        if self.active {
            let temp = self.get_temp();
            self.write_val("value", temp, writer);
        }
    }

    fn write_val<E, T: Write<E>>(&mut self, param: &str, val: f64, writer: &mut FmtWrap<'_, E, T>) {
        let mut buf = dtoa::Buffer::new();
        let value = buf.format(val);
        let _ = Msg::Update { module: "temp", param, value }.write(writer);
    }

    fn read<E, T: Write<E>>(&mut self, module: &str, param: &str, mut writer: FmtWrap<'_, E, T>) {
        if module == "temp" && param == "value" {
            let temp = self.get_temp();
            self.write_val("value", temp, &mut writer);
        } else if module == "temp" && param == "pollinterval" {
            self.write_val("pollinterval", self.pollinterval, &mut writer);
        } else {
            let _ = Error::no_module().into_msg(module, param).write(&mut writer);
        }
    }

    fn change<E, T: Write<E>>(&mut self, module: &str, param: &str, value: &str, mut writer: FmtWrap<'_, E, T>) {
        let _ = if module == "temp" && param == "pollinterval" {
            if let Ok(v) = value.parse() {
                self.pollinterval = v;
                Msg::Changed { module, param, value }.write(&mut writer)
            } else {
                Error::bad_value("not a number").into_msg(module, param).write(&mut writer)
            }
        } else {
            Error::no_module().into_msg(module, param).write(&mut writer)
        };
    }

    fn do_<E, T: Write<E>>(&mut self, module: &str, command: &str, _arg: &str, mut writer: FmtWrap<'_, E, T>) {
        let _ = Error::no_command().into_msg(module, command).write(&mut writer);
    }

    fn describe<E, T: Write<E>>(&mut self, mut writer: FmtWrap<'_, E, T>) {
        let _ = Msg::Describing { id: ".", structure: DESC }.write(&mut writer);
    }
}
